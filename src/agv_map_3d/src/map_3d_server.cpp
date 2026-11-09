#include "agv_map_3d/assets.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <zlib.h>
#include <archive.h>
#include <archive_entry.h>
#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cctype>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unistd.h>
#include <random>
#include <tuple>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace fs = std::filesystem;
using tcp = net::ip::tcp;
using json = nlohmann::json;
using agv_map_3d::Assets;

namespace
{
class LatestGrid
{
public:
  LatestGrid();
  void update(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  std::tuple<unsigned, std::string, std::string> get(const std::string & known_etag);

private:
  static std::string makeInstance();
  std::mutex mutex_;
  std::optional<nav_msgs::msg::OccupancyGrid> message_;
  std::string instance_;
  std::uint64_t version_ = 0;
  std::string encoded_;
};

int hex(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

constexpr std::size_t MAX_ZIP = 128 * 1024 * 1024;
constexpr std::size_t MAX_EXPANDED = 512 * 1024 * 1024;
constexpr std::size_t MAX_FILES = 1024;
const std::regex FILE_NAME("[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}");
const std::regex TILE_ID("[0-9]+");
const std::string NATIVE_ROUTE = "/native/lightning/";

using MapFiles = std::map<std::string, fs::path>;

class MapFormatError : public std::invalid_argument
{
public:
  MapFormatError(std::string code, std::string message)
  : std::invalid_argument(message), code(std::move(code)), message(std::move(message)) {}

  std::string code;
  std::string message;
};

void checkArchive(int result)
{
  if (result != ARCHIVE_OK) {
    throw MapFormatError("MAP_ARCHIVE_INVALID", "地图 ZIP 压缩包无效或已损坏");
  }
}

MapFiles readMapFiles(const fs::path & directory)
{
  MapFiles files;
  std::size_t size = 0;
  for (const auto & file : fs::directory_iterator(directory)) {
    const auto name = file.path().filename().string();
    if (name == "last_pose.yaml" || !std::regex_match(name, FILE_NAME) ||
      !fs::is_regular_file(file.symlink_status())) continue;
    const auto file_size = file.file_size();
    if (files.size() == MAX_FILES || file_size > MAX_EXPANDED - size) {
      throw MapFormatError("MAP_CONTENT_TOO_LARGE", "地图文件数量或总大小超过限制");
    }
    size += file_size;
    files.emplace(name, file.path());
  }
  for (const auto * name : {"map.yaml", "map.pgm", "global.pcd", "index.txt"}) {
    const auto file = files.find(name);
    if (file == files.end() || fs::file_size(file->second) == 0) {
      throw MapFormatError("MAP_CONTENT_INCOMPLETE", "地图文件不完整，缺少必需文件或文件为空");
    }
  }
  std::ifstream yaml(files.at("map.yaml"));
  std::string line;
  std::size_t image_fields = 0;
  while (std::getline(yaml, line)) {
    if (line.rfind("image:", 0) != 0) continue;
    std::istringstream value(line.substr(6));
    std::string image;
    value >> image;
    if (++image_fields != 1 || image != "map.pgm") {
      throw MapFormatError("MAP_CONTENT_INVALID", "地图配置 map.yaml 必须引用 map.pgm");
    }
  }
  if (image_fields != 1) {
    throw MapFormatError("MAP_CONTENT_INVALID", "地图配置 map.yaml 缺少 image 字段");
  }
  std::ifstream index(files.at("index.txt"));
  if (!std::getline(index, line)) {
    throw MapFormatError("MAP_CONTENT_INCOMPLETE", "地图索引 index.txt 为空");
  }
  while (std::getline(index, line)) {
    if (line == "# functional points" || line == "# functional points\r") break;
    std::istringstream row(line);
    std::string id;
    int x, y;
    if (row >> id && (!std::regex_match(id, TILE_ID) || !(row >> x >> y) ||
      files.count(id + ".pcd") == 0)) {
      throw MapFormatError("MAP_CONTENT_INVALID", "地图索引 index.txt 引用了无效或缺失的分块");
    }
  }
  return files;
}

la_ssize_t writeZip(struct archive *, void * client, const void * data, std::size_t size)
{
  auto & zip = *static_cast<std::string *>(client);
  if (size > MAX_ZIP - zip.size()) return -1;
  try {zip.append(static_cast<const char *>(data), size);}
  catch (const std::exception &) {return -1;}
  return static_cast<la_ssize_t>(size);
}

std::string exportMap(const fs::path & directory)
{
  const auto files = readMapFiles(directory);
  std::string zip;
  std::unique_ptr<struct archive, decltype(&archive_write_free)> writer(
    archive_write_new(), archive_write_free);
  checkArchive(archive_write_set_format_zip(writer.get()));
  checkArchive(archive_write_set_bytes_in_last_block(writer.get(), 1));
  checkArchive(archive_write_open(writer.get(), &zip, nullptr, writeZip, nullptr));
  std::array<char, 64 * 1024> buffer;
  for (const auto & [name, path] : files) {
    std::unique_ptr<struct archive_entry, decltype(&archive_entry_free)> entry(
      archive_entry_new(), archive_entry_free);
    archive_entry_set_pathname(entry.get(), name.c_str());
    archive_entry_set_filetype(entry.get(), AE_IFREG);
    archive_entry_set_perm(entry.get(), 0644);
    const auto size = fs::file_size(path);
    archive_entry_set_size(entry.get(), size);
    checkArchive(archive_write_header(writer.get(), entry.get()));
    std::ifstream input(path, std::ios::binary);
    std::uintmax_t written = 0;
    while (input.read(buffer.data(), buffer.size()) || input.gcount() != 0) {
      if (archive_write_data(writer.get(), buffer.data(), input.gcount()) != input.gcount()) {
        throw std::invalid_argument("Lightning ZIP exceeds limits");
      }
      written += input.gcount();
    }
    if (written != size || !input.eof()) throw std::runtime_error("map file changed during export");
    checkArchive(archive_write_finish_entry(writer.get()));
  }
  checkArchive(archive_write_close(writer.get()));
  return zip;
}

std::string mimeType(const fs::path & path)
{
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (extension == ".dae") return "model/vnd.collada+xml";
  if (extension == ".stl") return "model/stl";
  if (extension == ".obj") return "model/obj";
  if (extension == ".mtl") return "text/plain";
  if (extension == ".png") return "image/png";
  if (extension == ".jpg" || extension == ".jpeg") return "image/jpeg";
  if (extension == ".tga") return "image/x-tga";
  if (extension == ".bmp") return "image/bmp";
  if (extension == ".svg") return "image/svg+xml";
  if (extension == ".urdf" || extension == ".xacro" || extension == ".xml") return "application/xml";
  return "application/octet-stream";
}

class StagingDirectory
{
public:
  explicit StagingDirectory(const fs::path & root)
  {
    const auto pattern = (root / ".install-XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    if (!mkdtemp(name.data())) throw std::runtime_error("cannot create map staging directory");
    path = name.data();
  }
  ~StagingDirectory()
  {
    std::error_code error;
    fs::remove_all(path, error);
  }
  fs::path path;
};

void extractMap(const std::string & zip, const fs::path & directory)
{
  std::unique_ptr<struct archive, decltype(&archive_read_free)> reader(
    archive_read_new(), archive_read_free);
  checkArchive(archive_read_support_format_zip(reader.get()));
  checkArchive(archive_read_open_memory(reader.get(), zip.data(), zip.size()));
  struct archive_entry * entry = nullptr;
  std::array<char, 64 * 1024> buffer;
  std::size_t files = 0;
  std::size_t size = 0;
  int result;
  while ((result = archive_read_next_header(reader.get(), &entry)) != ARCHIVE_EOF) {
    checkArchive(result);
    const char * pathname = archive_entry_pathname(entry);
    if (++files > MAX_FILES) {
      throw MapFormatError("MAP_CONTENT_TOO_LARGE", "地图文件数量超过限制");
    }
    if (!pathname || !std::regex_match(pathname, FILE_NAME) ||
      std::string(pathname) == "last_pose.yaml" || archive_entry_filetype(entry) != AE_IFREG ||
      archive_entry_hardlink(entry) || archive_entry_symlink(entry) || fs::exists(directory / pathname)) {
      throw MapFormatError("MAP_ARCHIVE_INVALID", "地图 ZIP 包含非法、重复或不支持的文件路径");
    }
    std::ofstream output(directory / pathname, std::ios::binary);
    output.exceptions(std::ios::badbit | std::ios::failbit);
    la_ssize_t count;
    while ((count = archive_read_data(reader.get(), buffer.data(), buffer.size())) > 0) {
      if (static_cast<std::size_t>(count) > MAX_EXPANDED - size) {
        throw MapFormatError("MAP_CONTENT_TOO_LARGE", "地图解压后总大小超过限制");
      }
      size += count;
      output.write(buffer.data(), count);
    }
    if (count < 0) {
      throw MapFormatError("MAP_ARCHIVE_INVALID", "地图 ZIP 压缩包数据损坏");
    }
    output.close();
  }
}

bool sameFiles(const MapFiles & left, const MapFiles & right)
{
  if (left.size() != right.size()) return false;
  for (const auto & [name, path] : left) {
    const auto other = right.find(name);
    if (other == right.end() || fs::file_size(path) != fs::file_size(other->second)) return false;
    std::ifstream a(path, std::ios::binary), b(other->second, std::ios::binary);
    if (!a || !b || !std::equal(std::istreambuf_iterator<char>(a), {},
      std::istreambuf_iterator<char>(b), {})) return false;
  }
  return true;
}

http::status installMap(const fs::path & target, const std::string & zip,
  std::string & error_code, std::string & error_message)
{
  StagingDirectory staging(target.parent_path());
  const auto directory = staging.path / "map";
  fs::create_directory(directory);
  extractMap(zip, directory);
  const auto files = readMapFiles(directory);
  if (fs::is_symlink(target)) {
    error_code = "MAP_PATH_CONFLICT";
    error_message = "目标地图路径不安全，无法覆盖: " + target.filename().string();
    return http::status::conflict;
  }
  if (fs::exists(target)) {
    try {
      if (fs::is_directory(target) && sameFiles(files, readMapFiles(target))) {
        return http::status::no_content;
      }
    } catch (const std::invalid_argument &) {
      // An incomplete map at this name is preserved; callers can choose another name.
    }
    error_code = "MAP_ALREADY_EXISTS";
    error_message = "地图已存在且内容不同: " + target.filename().string() + "，请更换地图名";
    return http::status::conflict;
  }
  fs::rename(directory, target);
  return http::status::no_content;
}

bool authorized(const std::string & expected, beast::string_view actual)
{
  if (expected.empty() || actual.size() != expected.size()) return false;
  unsigned char difference = 0;
  for (std::size_t i = 0; i < expected.size(); ++i) difference |= expected[i] ^ actual[i];
  return difference == 0;
}

void setNativeError(http::response<http::string_body> & response, http::status status,
  const std::string & code, const std::string & message, const std::string & map_name)
{
  response.result(status);
  response.set(http::field::content_type, "application/json; charset=utf-8");
  response.set(http::field::cache_control, "no-store");
  if (status == http::status::method_not_allowed) {
    response.set(http::field::allow, "GET, PUT");
  }
  response.body() = json{
    {"status", static_cast<unsigned int>(status)}, {"code", code},
    {"message", message}, {"error", message}, {"map_name", map_name}
  }.dump(-1, ' ', false, json::error_handler_t::replace);
}

template<typename StringLike>
bool hasPrefix(const StringLike & value, const std::string & prefix)
{
  return value.size() >= prefix.size() &&
         std::equal(prefix.begin(), prefix.end(), value.begin());
}

std::optional<fs::path> resourcePath(beast::string_view target)
{
  target = target.substr(0, target.find('?'));
  if (target.empty() || target.front() != '/') return std::nullopt;
  std::string decoded;
  for (std::size_t i = 0; i < target.size(); ++i) {
    unsigned char value = target[i];
    if (value == '%') {
      if (i + 2 >= target.size() || hex(target[i + 1]) < 0 || hex(target[i + 2]) < 0) {
        return std::nullopt;
      }
      value = static_cast<unsigned char>(hex(target[i + 1]) * 16 + hex(target[i + 2]));
      i += 2;
    }
    if (value < 32 || value == 127 || value == '\\') return std::nullopt;
    decoded.push_back(static_cast<char>(value));
  }
  std::vector<std::string> parts;
  for (std::size_t start = 1; start < decoded.size();) {
    const auto end = decoded.find('/', start);
    const auto part = decoded.substr(start, end - start);
    if (part == "." || part == "..") return std::nullopt;
    if (!part.empty()) parts.push_back(part);
    if (end == std::string::npos) break;
    start = end + 1;
  }
  if (parts.size() < 2 || !std::all_of(parts[0].begin(), parts[0].end(), [](unsigned char c) {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '_';
    })) return std::nullopt;
  try {
    fs::path result = ament_index_cpp::get_package_share_directory(parts[0]);
    for (std::size_t i = 1; i < parts.size(); ++i) result /= parts[i];
    std::error_code error;
    if (!fs::is_regular_file(result, error) || error) return std::nullopt;
    return result;
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return std::nullopt;
  }
}

std::vector<std::string> route(beast::string_view target)
{
  target = target.substr(0, target.find('?'));
  std::string decoded;
  for (std::size_t i = 0; i < target.size(); ++i) {
    unsigned char c = target[i];
    if (c == '%') {
      if (i + 2 >= target.size() || hex(target[i + 1]) < 0 || hex(target[i + 2]) < 0) {
        throw agv_map_3d::InvalidPath("Invalid URL encoding");
      }
      c = static_cast<unsigned char>(hex(target[i + 1]) * 16 + hex(target[i + 2]));
      i += 2;
    }
    if (c < 32 || c == 127 || c == '\\') throw agv_map_3d::InvalidPath("Invalid URL character");
    decoded.push_back(static_cast<char>(c));
  }
  if (decoded.empty() || decoded[0] != '/') return {};
  std::vector<std::string> parts;
  for (std::size_t start = 1; start <= decoded.size();) {
    const auto end = decoded.find('/', start);
    parts.push_back(decoded.substr(start, end - start));
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return parts;
}

class Session : public std::enable_shared_from_this<Session>
{
public:
  Session(tcp::socket socket, Assets * assets, const std::string & token,
    const fs::path & root, LatestGrid & grid, bool mesh_enabled, bool live_enabled,
    bool native_enabled)
  : stream_(std::move(socket)), assets_(assets), token_(token), root_(root), grid_(grid),
    mesh_enabled_(mesh_enabled), live_enabled_(live_enabled), native_enabled_(native_enabled)
  {
    parser_.header_limit(8192);
    parser_.body_limit(MAX_ZIP);
  }
  void run()
  {
    stream_.expires_after(std::chrono::seconds(30));
    http::async_read(stream_, buffer_, parser_,
      [self = shared_from_this()](beast::error_code error, std::size_t) {
        if (!error) self->respond();
        else self->handleReadError(error);
      });
  }

private:
  template<class Body>
  void send(http::response<Body> response)
  {
    response.set(http::field::server, "robot_http_server");
    response.set(http::field::access_control_allow_origin, "*");
    response.keep_alive(false);
    auto message = std::make_shared<http::response<Body>>(std::move(response));
    stream_.expires_after(std::chrono::seconds(300));
    http::async_write(stream_, *message,
      [self = shared_from_this(), message](beast::error_code, std::size_t) {
        beast::error_code ignored;
        self->stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
      });
  }
  void sendJson(http::status code, const json & value)
  {
    const auto body = value.dump(-1, ' ', false, json::error_handler_t::replace);
    if (parser_.get().method() == http::verb::head) {
      http::response<http::empty_body> response{code, parser_.get().version()};
      response.set(http::field::content_type, "application/json; charset=utf-8");
      response.set(http::field::cache_control, "no-store");
      response.content_length(body.size());
      return send(std::move(response));
    }
    http::response<http::string_body> response{code, parser_.get().version()};
    response.set(http::field::content_type, "application/json; charset=utf-8");
    response.set(http::field::cache_control, "no-store");
    if (code == http::status::method_not_allowed) response.set(http::field::allow, "GET, HEAD, OPTIONS");
    response.body() = body;
    response.prepare_payload();
    send(std::move(response));
  }
  void handleReadError(const beast::error_code & error)
  {
    if (error == http::error::body_limit) {
      const auto & request = parser_.get();
      const auto target = request.target().to_string();
      const bool native_map_request = hasPrefix(target, NATIVE_ROUTE);
      const auto name = native_map_request ? target.substr(NATIVE_ROUTE.size()) : "";
      http::response<http::string_body> response{
        http::status::payload_too_large, request.version() == 0 ? 11 : request.version()};
      response.keep_alive(false);
      setNativeError(response, http::status::payload_too_large,
        native_map_request ? "MAP_UPLOAD_TOO_LARGE" : "REQUEST_BODY_TOO_LARGE",
        native_map_request ? "地图 ZIP 超过最大上传限制（128 MiB）" : "请求内容超过服务器限制",
        name);
      response.prepare_payload();
      return send(std::move(response));
    }
    beast::error_code ignored;
    stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
  }
  void respond()
  {
    try {
      const auto & request = parser_.get();
      if (hasPrefix(request.target(), NATIVE_ROUTE)) {
        if (!native_enabled_) {
          http::response<http::string_body> response{http::status::not_found, request.version()};
          response.keep_alive(false);
          setNativeError(response, http::status::not_found, "MAP_TRANSFER_DISABLED",
            "机器人未启用 Lightning 原生地图传输接口", "");
          response.prepare_payload();
          return send(std::move(response));
        }
        return respondNative();
      }
      if (request.method() == http::verb::put) {
        return sendJson(http::status::method_not_allowed,
          {{"error", "Only GET, HEAD and OPTIONS are supported"}});
      }
      if (request.method() == http::verb::options) {
        http::response<http::empty_body> response{http::status::no_content, request.version()};
        const auto target_parts = route(request.target());
        response.set(http::field::access_control_allow_methods, "GET, HEAD, OPTIONS");
        response.set(http::field::access_control_allow_headers, "Content-Type");
        response.set(http::field::access_control_allow_origin, "*");
        return send(std::move(response));
      }
      if (request.method() != http::verb::get && request.method() != http::verb::head) {
        return sendJson(http::status::method_not_allowed, {{"error", "Only GET, HEAD and OPTIONS are supported"}});
      }
      const auto parts = route(request.target());
      if (parts.size() == 2 && parts[0] == "maps" && parts[1] == "live") {
        if (!live_enabled_) return sendJson(http::status::not_found, {{"error", "Unknown route"}});
        return sendLiveMap();
      }
      if (parts.size() == 3 && parts[0] == "maps" &&
        (parts[2] == "status" || parts[2] == "cloud.ply")) {
        if (!assets_) return sendJson(http::status::not_found, {{"error", "Unknown 3D asset route"}});
        return respond3d(parts);
      }
      if (parts.size() >= 2) {
        if (!mesh_enabled_) return sendJson(http::status::not_found, {{"error", "Unknown route"}});
        const auto resource = resourcePath(request.target());
        if (!resource) return sendMeshText(http::status::not_found, "mesh not found\n");
        http::file_body::value_type resource_body;
        beast::error_code resource_error;
        resource_body.open(resource->c_str(), beast::file_mode::scan, resource_error);
        if (resource_error) return sendMeshText(http::status::internal_server_error, "read error\n");
        const auto size = resource_body.size();
        if (request.method() == http::verb::head) {
          http::response<http::empty_body> response{http::status::ok, request.version()};
          response.set(http::field::content_type, mimeType(*resource));
          response.content_length(size);
          return send(std::move(response));
        }
        http::response<http::file_body> response{http::status::ok, request.version()};
        response.set(http::field::content_type, mimeType(*resource));
        response.content_length(size);
        response.body() = std::move(resource_body);
        return send(std::move(response));
      }
      return sendJson(http::status::not_found, {{"error", "Unknown 3D asset route"}});
    } catch (const agv_map_3d::Changed & error) {
      sendJson(http::status::conflict, {{"error", error.what()}});
    } catch (const agv_map_3d::NotFound & error) {
      sendJson(http::status::not_found, {{"error", error.what()}});
    } catch (const agv_map_3d::InvalidPath & error) {
      sendJson(http::status::bad_request, {{"error", error.what()}});
    } catch (const std::exception & error) {
      std::cerr << "HTTP request: " << error.what() << '\n';
      sendJson(http::status::internal_server_error, {{"error", "HTTP request failed"}});
    }
  }
  void respond3d(const std::vector<std::string> & parts)
  {
    const auto & request = parser_.get();
    const auto state = assets_->status(parts[1]);
    if (parts[2] == "status") return sendJson(http::status::ok, state);
    if (state.value("status", "") != "ready") {
      return sendJson(http::status::conflict, {{"error", "3D map is not ready"}});
    }
    http::file_body::value_type body;
    beast::error_code error;
    body.open(assets_->cloud(parts[1], state.at("version").get<std::string>()).c_str(), beast::file_mode::scan, error);
    if (error) return sendJson(http::status::conflict, {{"error", "3D asset changed; request status again"}});
    const auto size = body.size();
    if (size != state.at("size_bytes").get<uint64_t>()) {
      return sendJson(http::status::conflict, {{"error", "3D asset changed; request status again"}});
    }
    if (request.method() == http::verb::head) {
      http::response<http::empty_body> response{http::status::ok, request.version()};
      response.set(http::field::content_type, "application/octet-stream");
      response.set(http::field::cache_control, "no-cache");
      response.content_length(size);
      return send(std::move(response));
    }
    http::response<http::file_body> response{http::status::ok, request.version()};
    response.set(http::field::content_type, "application/octet-stream");
    response.set(http::field::cache_control, "no-cache");
    response.content_length(size);
    response.body() = std::move(body);
    send(std::move(response));
  }
  void sendMeshText(http::status code, const std::string & value)
  {
    if (parser_.get().method() == http::verb::head) {
      http::response<http::empty_body> response{code, parser_.get().version()};
      response.set(http::field::content_type, "text/plain; charset=utf-8");
      response.set(http::field::access_control_allow_origin, "*");
      response.content_length(value.size());
      return send(std::move(response));
    }
    http::response<http::string_body> response{code, parser_.get().version()};
    response.set(http::field::content_type, "text/plain; charset=utf-8");
    response.set(http::field::access_control_allow_origin, "*");
    if (code == http::status::method_not_allowed) response.set(http::field::allow, "GET, HEAD, OPTIONS");
    response.body() = value;
    response.prepare_payload();
    send(std::move(response));
  }
  void sendLiveMap()
  {
    const auto [status, etag, body] = grid_.get(parser_.get()[http::field::if_none_match].to_string());
    http::response<http::string_body> response{
      static_cast<http::status>(status), parser_.get().version()};
    response.set(http::field::cache_control, "no-cache");
    response.set(http::field::content_type, "application/json");
    response.content_length(body.size());
    if (!etag.empty()) response.set(http::field::etag, etag);
    if (status == 200) response.set(http::field::content_encoding, "gzip");
    if (status != 304) response.body() = body;
    if (status == 304) response.content_length(0);
    else response.prepare_payload();
    send(std::move(response));
  }
  void respondNative()
  {
    const auto & request = parser_.get();
    const std::string request_name =
      std::string(request.target()).substr(NATIVE_ROUTE.size());
    if (request.method() == http::verb::put) {
      if (!authorized(token_, request[http::field::authorization])) {
        http::response<http::string_body> response{http::status::unauthorized, request.version()};
        response.keep_alive(false);
        setNativeError(response, http::status::unauthorized, "MAP_AUTH_FAILED",
          "地图传输认证失败，请检查机器人端 Token 配置", request_name);
        response.prepare_payload();
        return send(std::move(response));
      }
      if (!parser_.content_length() || *parser_.content_length() == 0) {
        http::response<http::string_body> response{http::status::payload_too_large, request.version()};
        response.keep_alive(false);
        setNativeError(response, http::status::payload_too_large, "MAP_UPLOAD_EMPTY",
          "上传内容为空，请检查地图 ZIP 文件", request_name);
        response.prepare_payload();
        return send(std::move(response));
      }
      stream_.expires_after(std::chrono::seconds(30));
      http::async_read(stream_, buffer_, parser_,
        [self = shared_from_this()](beast::error_code error, std::size_t) {
          if (!error) self->finishNativeUpload();
          else self->handleReadError(error);
        });
      return;
    }
    if (request.method() != http::verb::get) {
      http::response<http::string_body> response{http::status::method_not_allowed, request.version()};
      response.keep_alive(false);
      setNativeError(response, http::status::method_not_allowed, "METHOD_NOT_ALLOWED",
        "该地图接口仅支持 GET 下载和 PUT 上传", "");
      response.prepare_payload();
      return send(std::move(response));
    }
    http::response<http::string_body> response{http::status::bad_request, request.version()};
    response.keep_alive(false);
    if (!authorized(token_, request[http::field::authorization])) {
      setNativeError(response, http::status::unauthorized, "MAP_AUTH_FAILED",
        "地图传输认证失败，请检查机器人端 Token 配置", request_name);
    } else {
      const auto & name = request_name;
      if (std::regex_match(name, FILE_NAME)) {
        const auto target = root_ / name;
        try {
          if (request.method() == http::verb::get) {
            if (fs::is_symlink(target) || !fs::is_directory(target)) {
              setNativeError(response, http::status::not_found, "MAP_NOT_FOUND",
                "机器人上不存在地图: " + name, name);
            } else {
              response.body() = exportMap(target);
              response.result(http::status::ok);
              response.set(http::field::content_type, "application/zip");
            }
          } else {
            setNativeError(response, http::status::method_not_allowed, "METHOD_NOT_ALLOWED",
              "地图下载仅支持 GET 请求", name);
          }
        } catch (const MapFormatError & error) {
          setNativeError(response, http::status::unprocessable_entity,
            error.code, error.message, name);
        } catch (const std::invalid_argument &) {
          setNativeError(response, http::status::unprocessable_entity, "MAP_CONTENT_INVALID",
            "地图文件格式无效或内容不完整", name);
        } catch (const std::exception & error) {
          std::cerr << "Lightning map transfer failed: " << error.what() << '\n';
          setNativeError(response, http::status::internal_server_error, "MAP_EXPORT_FAILED",
            "机器人导出地图失败，请检查地图目录和磁盘状态", name);
        }
      } else {
        setNativeError(response, http::status::bad_request, "MAP_NAME_INVALID",
          "地图名称格式无效", name);
      }
    }
    if (response.result() != http::status::no_content) response.prepare_payload();
    send(std::move(response));
  }
  void finishNativeUpload()
  {
    const auto & request = parser_.get();
    http::response<http::string_body> response{http::status::bad_request, request.version()};
    response.keep_alive(false);
    const std::string name = std::string(request.target()).substr(NATIVE_ROUTE.size());
    if (!authorized(token_, request[http::field::authorization])) {
      setNativeError(response, http::status::unauthorized, "MAP_AUTH_FAILED",
        "地图传输认证失败，请检查机器人端 Token 配置", name);
    } else if (!parser_.content_length() || *parser_.content_length() == 0) {
      setNativeError(response, http::status::payload_too_large, "MAP_UPLOAD_EMPTY",
        "上传内容为空，请检查地图 ZIP 文件", name);
    } else if (!std::regex_match(name, FILE_NAME)) {
      setNativeError(response, http::status::bad_request, "MAP_NAME_INVALID",
        "地图名称格式无效", name);
    } else {
      try {
        std::string error_code;
        std::string error_message;
        response.result(installMap(root_ / name, request.body(), error_code, error_message));
        if (!error_message.empty()) {
          setNativeError(response, http::status::conflict, error_code, error_message, name);
        }
      }
      catch (const MapFormatError & error) {
        setNativeError(response, http::status::unprocessable_entity,
          error.code, error.message, name);
      }
      catch (const std::invalid_argument &) {
        setNativeError(response, http::status::unprocessable_entity, "MAP_ARCHIVE_INVALID",
          "地图 ZIP 压缩包无效、已损坏或地图文件不完整", name);
      }
      catch (const std::exception & error) {
        std::cerr << "Lightning map transfer failed: " << error.what() << '\n';
        setNativeError(response, http::status::internal_server_error, "MAP_IMPORT_FAILED",
          "机器人保存地图失败，请检查磁盘空间和地图目录权限", name);
      }
    }
    response.prepare_payload();
    send(std::move(response));
  }
  beast::tcp_stream stream_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
  Assets * assets_;
  std::string token_;
  fs::path root_;
  LatestGrid & grid_;
  bool mesh_enabled_;
  bool live_enabled_;
  bool native_enabled_;
};

class Server
{
public:
  Server(net::io_context & context, const tcp::endpoint & endpoint, Assets * assets,
    const std::string & token, const fs::path & root, LatestGrid & grid, bool mesh_enabled,
    bool live_enabled, bool native_enabled)
  : acceptor_(context), assets_(assets), token_(token), root_(root), grid_(grid),
    mesh_enabled_(mesh_enabled), live_enabled_(live_enabled), native_enabled_(native_enabled)
  {
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(net::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
    accept();
  }
  unsigned short port() const {return acceptor_.local_endpoint().port();}
private:
  void accept()
  {
    acceptor_.async_accept([this](beast::error_code error, tcp::socket socket) {
      if (!error) std::make_shared<Session>(std::move(socket), assets_, token_, root_, grid_,
        mesh_enabled_, live_enabled_, native_enabled_)->run();
      else if (error != net::error::operation_aborted) std::cerr << "3D accept: " << error.message() << '\n';
      if (acceptor_.is_open()) accept();
    });
  }
  tcp::acceptor acceptor_;
  Assets * assets_;
  std::string token_;
  fs::path root_;
  LatestGrid & grid_;
  bool mesh_enabled_;
  bool live_enabled_;
  bool native_enabled_;
};

LatestGrid::LatestGrid() : instance_(makeInstance()) {}

void LatestGrid::update(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  if (!message || message->data.empty() ||
    message->data.size() != message->info.width * message->info.height) return;
  std::lock_guard<std::mutex> lock(mutex_);
  message_ = *message;
  ++version_;
  encoded_.clear();
}

std::tuple<unsigned, std::string, std::string> LatestGrid::get(const std::string & known_etag)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message_) return {503, "", "{\"error\":\"map not available\"}"};
  const auto etag = "\"" + instance_ + "-" + std::to_string(version_) + "\"";
  if (known_etag == etag) return {304, etag, ""};
  if (encoded_.empty()) {
    const auto & msg = *message_;
    const auto & info = msg.info;
    const auto & pos = info.origin.position;
    const auto & rot = info.origin.orientation;
    json payload = {
      {"header", {{"frame_id", msg.header.frame_id}}},
      {"info", {{"width", info.width}, {"height", info.height}, {"resolution", info.resolution},
        {"origin", {{"position", {{"x", pos.x}, {"y", pos.y}, {"z", pos.z}}},
          {"orientation", {{"x", rot.x}, {"y", rot.y}, {"z", rot.z}, {"w", rot.w}}}}}}},
      {"data", msg.data}};
    const auto text = payload.dump(-1, ' ', false, json::error_handler_t::replace);
    z_stream stream{};
    if (deflateInit2(&stream, 1, Z_DEFLATED, 15 + 16, 8, Z_DEFAULT_STRATEGY) != Z_OK) {
      throw std::runtime_error("gzip initialization failed");
    }
    stream.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(text.data()));
    stream.avail_in = static_cast<uInt>(text.size());
    std::array<unsigned char, 65536> buffer{};
    int result;
    do {
      stream.next_out = buffer.data();
      stream.avail_out = buffer.size();
      result = deflate(&stream, Z_FINISH);
      if (result != Z_OK && result != Z_STREAM_END && result != Z_BUF_ERROR) {
        deflateEnd(&stream);
        throw std::runtime_error("gzip encoding failed");
      }
      encoded_.append(reinterpret_cast<const char *>(buffer.data()), buffer.size() - stream.avail_out);
    } while (result != Z_STREAM_END);
    deflateEnd(&stream);
  }
  return {200, etag, encoded_};
}

std::string LatestGrid::makeInstance()
{
  std::random_device random;
  std::ostringstream value;
  value << std::hex << random() << random();
  return value.str();
}

double number(const std::string & text)
{
  std::size_t consumed = 0;
  const auto value = std::stod(text, &consumed);
  if (!std::isfinite(value) || consumed != text.size()) throw std::runtime_error("Invalid numeric option: " + text);
  return value;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    LatestGrid grid;
    rclcpp::Node::SharedPtr ros_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread ros_thread;
    if (!rclcpp::ok()) rclcpp::init(argc, argv);
    ros_node = std::make_shared<rclcpp::Node>("robot_http_server");
    auto subscription = ros_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      [&grid](nav_msgs::msg::OccupancyGrid::SharedPtr message) {grid.update(message);});
    (void)subscription;
    agv_map_3d::Options options;
    const auto directory = fs::read_symlink("/proc/self/exe").parent_path();
    options.pbstream_exporter = directory / "pbstream_to_ply";
    options.pcd_exporter = directory / "pcd_to_ply";
    std::string address = "0.0.0.0";
    unsigned int port = 8080;
    bool mesh_enabled = true;
    bool live_enabled = true;
    bool maps_3d_enabled = true;
    const char * raw_token = std::getenv("NATIVE_MAP_TOKEN");
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help" || key == "-h") {
        std::cout << "Usage: robot_http_server --maps-dir DIR [--address IP] [--port 8080] [--voxel 0.1]\n"
          << "  [--exporter FILE] [--exporter-pcd FILE] [--source-format auto|pcd|pbstream] [--settle-seconds 2]\n"
          << "  [--scan-seconds 10] [--export-timeout 300] [--retry-seconds 30]\n";
        return 0;
      }
      if (++i >= argc) throw std::runtime_error("Missing option value: " + key);
      const std::string value = argv[i];
      if (key == "--maps-dir") options.root = value;
      else if (key == "--address") address = value;
      else if (key == "--exporter") options.pbstream_exporter = fs::absolute(value);
      else if (key == "--exporter-pcd") options.pcd_exporter = fs::absolute(value);
      else if (key == "--source-format") options.source_format = value;
      else if (key == "--enable-mesh") mesh_enabled = value == "true" || value == "1";
      else if (key == "--enable-live-map") live_enabled = value == "true" || value == "1";
      else if (key == "--enable-map-3d") maps_3d_enabled = value == "true" || value == "1";
      else if (key == "--voxel") options.voxel = number(value);
      else if (key == "--settle-seconds") options.settle_seconds = number(value);
      else if (key == "--scan-seconds") options.scan_seconds = number(value);
      else if (key == "--export-timeout") options.timeout_seconds = number(value);
      else if (key == "--retry-seconds") options.retry_seconds = number(value);
      else if (key == "--port") {
        const auto result = std::from_chars(value.data(), value.data() + value.size(), port);
        if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || port > 65535) {
          throw std::runtime_error("Port must be between 0 and 65535");
        }
      } else throw std::runtime_error("Unknown option: " + key);
    }
    const bool native_enabled = raw_token && *raw_token && options.source_format == "pcd";
    if (options.root.empty()) throw std::runtime_error("--maps-dir is required");
    if (options.source_format != "auto" && options.source_format != "pcd" && options.source_format != "pbstream")
      throw std::runtime_error("--source-format must be auto, pcd or pbstream");
    if (options.voxel < 0.001 || options.voxel >= 10 || options.settle_seconds < 0 ||
      options.scan_seconds <= 0 || options.timeout_seconds <= 0 || options.retry_seconds < 0)
    {
      throw std::runtime_error("Invalid voxel/timing options (voxel must be >=0.001 and <10 meters)");
    }
    const std::string token = native_enabled ? "Bearer " + std::string(raw_token) : "";
    fs::create_directories(options.root);
    const fs::path root = options.root;
    std::unique_ptr<Assets> assets;
    if (maps_3d_enabled) assets = std::make_unique<Assets>(options);
    net::io_context context{1};
    Server server(context, {net::ip::make_address(address), static_cast<unsigned short>(port)},
      assets.get(), token, root, grid, mesh_enabled, live_enabled, native_enabled);
    if (assets) assets->start();
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(ros_node);
    ros_thread = std::thread([&executor]() {executor->spin();});
    net::signal_set signals(context, SIGINT, SIGTERM);
    signals.async_wait([&](beast::error_code, int) {
      if (assets) assets->stop();
      executor->cancel();
      context.stop();
    });
    std::cout << "[robot_http_server] listening on " << address << ':' << server.port()
              << "; maps=" << fs::absolute(options.root) << std::endl;
    context.run();
    if (assets) assets->stop();
    executor->cancel();
    if (ros_thread.joinable()) ros_thread.join();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    if (rclcpp::ok()) rclcpp::shutdown();
    std::cerr << "[robot_http_server] " << error.what() << '\n';
    return 1;
  }
}
