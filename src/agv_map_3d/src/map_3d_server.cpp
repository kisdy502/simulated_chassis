#include "agv_map_3d/assets.hpp"
#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <charconv>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace fs = std::filesystem;
using tcp = net::ip::tcp;
using json = nlohmann::json;
using agv_map_3d::Assets;

namespace
{
int hex(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
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
  Session(tcp::socket socket, Assets & assets) : stream_(std::move(socket)), assets_(assets)
  {
    parser_.header_limit(8192);
    parser_.body_limit(1024);
  }
  void run()
  {
    stream_.expires_after(std::chrono::seconds(30));
    http::async_read(stream_, buffer_, parser_,
      [self = shared_from_this()](beast::error_code error, std::size_t) {if (!error) self->respond();});
  }

private:
  template<class Body>
  void send(http::response<Body> response)
  {
    response.set(http::field::server, "map_3d_server");
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
  void respond()
  {
    try {
      const auto & request = parser_.get();
      if (request.method() == http::verb::options) {
        http::response<http::empty_body> response{http::status::no_content, request.version()};
        response.set(http::field::access_control_allow_methods, "GET, HEAD, OPTIONS");
        response.set(http::field::access_control_allow_headers, "Content-Type");
        return send(std::move(response));
      }
      if (request.method() != http::verb::get && request.method() != http::verb::head) {
        return sendJson(http::status::method_not_allowed, {{"error", "Only GET, HEAD and OPTIONS are supported"}});
      }
      const auto parts = route(request.target());
      if (parts.size() != 3 || parts[0] != "maps" || (parts[2] != "status" && parts[2] != "cloud.ply")) {
        return sendJson(http::status::not_found, {{"error", "Unknown 3D asset route"}});
      }
      const auto state = assets_.status(parts[1]);
      if (parts[2] == "status") return sendJson(http::status::ok, state);
      if (state.value("status", "") != "ready") {
        return sendJson(http::status::conflict, {{"error", "3D map is not ready"}});
      }
      http::file_body::value_type body;
      beast::error_code error;
      body.open(assets_.cloud(parts[1], state.at("version").get<std::string>()).c_str(), beast::file_mode::scan, error);
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
    } catch (const agv_map_3d::Changed & error) {
      sendJson(http::status::conflict, {{"error", error.what()}});
    } catch (const agv_map_3d::NotFound & error) {
      sendJson(http::status::not_found, {{"error", error.what()}});
    } catch (const agv_map_3d::InvalidPath & error) {
      sendJson(http::status::bad_request, {{"error", error.what()}});
    } catch (const std::exception & error) {
      std::cerr << "3D request: " << error.what() << '\n';
      sendJson(http::status::internal_server_error, {{"error", "3D asset request failed"}});
    }
  }
  beast::tcp_stream stream_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
  Assets & assets_;
};

class Server
{
public:
  Server(net::io_context & context, const tcp::endpoint & endpoint, Assets & assets)
  : acceptor_(context), assets_(assets)
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
      if (!error) std::make_shared<Session>(std::move(socket), assets_)->run();
      else if (error != net::error::operation_aborted) std::cerr << "3D accept: " << error.message() << '\n';
      if (acceptor_.is_open()) accept();
    });
  }
  tcp::acceptor acceptor_;
  Assets & assets_;
};

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
    agv_map_3d::Options options;
    const auto directory = fs::read_symlink("/proc/self/exe").parent_path();
    options.pbstream_exporter = directory / "pbstream_to_ply";
    options.pcd_exporter = directory / "pcd_to_ply";
    std::string address = "0.0.0.0";
    unsigned int port = 8089;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help" || key == "-h") {
        std::cout << "Usage: map_3d_server --maps-dir DIR [--address IP] [--port 8089] [--voxel 0.1]\n"
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
    if (options.root.empty()) throw std::runtime_error("--maps-dir is required");
    if (options.source_format != "auto" && options.source_format != "pcd" && options.source_format != "pbstream")
      throw std::runtime_error("--source-format must be auto, pcd or pbstream");
    if (options.voxel < 0.001 || options.voxel >= 10 || options.settle_seconds < 0 ||
      options.scan_seconds <= 0 || options.timeout_seconds <= 0 || options.retry_seconds < 0)
    {
      throw std::runtime_error("Invalid voxel/timing options (voxel must be >=0.001 and <10 meters)");
    }
    Assets assets(options);
    net::io_context context{1};
    Server server(context, {net::ip::make_address(address), static_cast<unsigned short>(port)}, assets);
    net::signal_set signals(context, SIGINT, SIGTERM);
    signals.async_wait([&](beast::error_code, int) {assets.stop(); context.stop();});
    assets.start();
    std::cout << "[map_3d_server] listening on " << address << ':' << server.port()
              << "; maps=" << fs::absolute(options.root) << std::endl;
    context.run();
    assets.stop();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "[map_3d_server] " << error.what() << '\n';
    return 1;
  }
}
