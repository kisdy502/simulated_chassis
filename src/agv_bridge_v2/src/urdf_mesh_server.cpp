// Static package:// resources for the browser. HTTP parsing and file transfer
// are handled by Boost.Beast; package roots come from the ament index.
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cctype>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace fs = std::filesystem;
using tcp = net::ip::tcp;

namespace
{
int hexDigit(char value)
{
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  return -1;
}

std::optional<fs::path> resourcePath(beast::string_view target)
{
  // Decode once before checking path components, including encoded slashes/dots.
  const auto query = target.find('?');
  target = target.substr(0, query);
  if (target.empty() || target.front() != '/') return std::nullopt;
  std::string decoded;
  for (std::size_t i = 0; i < target.size(); ++i) {
    unsigned char value = target[i];
    if (value == '%') {
      if (i + 2 >= target.size()) return std::nullopt;
      const int high = hexDigit(target[i + 1]);
      const int low = hexDigit(target[i + 2]);
      if (high < 0 || low < 0) return std::nullopt;
      value = static_cast<unsigned char>((high << 4) | low);
      i += 2;
    }
    if (value < 32 || value == 127 || value == '\\') return std::nullopt;
    decoded.push_back(static_cast<char>(value));
  }
  std::vector<std::string> parts;
  for (std::size_t start = 1; start < decoded.size();) {
    const auto end = decoded.find('/', start);
    const auto part = decoded.substr(start, end - start);
    if (part == ".." || part == ".") return std::nullopt;
    if (!part.empty()) parts.push_back(part);
    if (end == std::string::npos) break;
    start = end + 1;
  }
  if (parts.size() < 2) return std::nullopt;
  if (!std::all_of(parts[0].begin(), parts[0].end(), [](unsigned char c) {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
             (c >= '0' && c <= '9') || c == '_';
    })) return std::nullopt;
  try {
    fs::path result = ament_index_cpp::get_package_share_directory(parts[0]);
    for (std::size_t i = 1; i < parts.size(); ++i) result /= parts[i];
    // Do not canonicalize: symlink-install intentionally points into source.
    // Installed package contents, including their symlinks, are trusted assets.
    std::error_code error;
    if (!fs::is_regular_file(result, error) || error) return std::nullopt;
    return result;
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return std::nullopt;
  }
}

std::string mimeType(const fs::path & path)
{
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
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

class Session : public std::enable_shared_from_this<Session>
{
public:
  explicit Session(tcp::socket socket) : stream_(std::move(socket))
  {
    parser_.header_limit(8192);
    parser_.body_limit(1024);
  }

  void run()
  {
    stream_.expires_after(std::chrono::seconds(30));
    http::async_read(stream_, buffer_, parser_,
      [self = shared_from_this()](beast::error_code error, std::size_t) {
        if (!error) self->respond();
      });
  }

private:
  template<class Body>
  void send(http::response<Body> response)
  {
    response.set(http::field::server, "urdf_mesh_server");
    response.set(http::field::access_control_allow_origin, "*");
    response.keep_alive(false);
    auto message = std::make_shared<http::response<Body>>(std::move(response));
    stream_.expires_after(std::chrono::seconds(30));
    http::async_write(stream_, *message,
      [self = shared_from_this(), message](beast::error_code, std::size_t) {
        beast::error_code ignored;
        self->stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
      });
  }

  void error(http::status status, const std::string & text)
  {
    if (parser_.get().method() == http::verb::head) {
      http::response<http::empty_body> response{status, parser_.get().version()};
      response.set(http::field::content_type, "text/plain; charset=utf-8");
      response.content_length(text.size());
      return send(std::move(response));
    }
    http::response<http::string_body> response{status, parser_.get().version()};
    response.set(http::field::content_type, "text/plain; charset=utf-8");
    if (status == http::status::method_not_allowed) {
      response.set(http::field::allow, "GET, HEAD, OPTIONS");
    }
    response.body() = text;
    response.prepare_payload();
    send(std::move(response));
  }

  void respond()
  {
    const auto & request = parser_.get();
    if (request.method() == http::verb::options) {
      http::response<http::empty_body> response{http::status::no_content, request.version()};
      response.set(http::field::access_control_allow_methods, "GET, HEAD, OPTIONS");
      response.set(http::field::access_control_allow_headers, "Content-Type");
      return send(std::move(response));
    }
    if (request.method() != http::verb::get && request.method() != http::verb::head) {
      return error(http::status::method_not_allowed, "Only GET, HEAD and OPTIONS are supported\n");
    }
    const auto path = resourcePath(request.target());
    if (!path) return error(http::status::not_found, "mesh not found\n");
    beast::error_code ec;
    http::file_body::value_type body;
    body.open(path->c_str(), beast::file_mode::scan, ec);
    if (ec) return error(http::status::internal_server_error, "read error\n");
    const auto size = body.size();
    if (request.method() == http::verb::head) {
      http::response<http::empty_body> response{http::status::ok, request.version()};
      response.set(http::field::content_type, mimeType(*path));
      response.content_length(size);
      return send(std::move(response));
    }
    http::response<http::file_body> response{http::status::ok, request.version()};
    response.set(http::field::content_type, mimeType(*path));
    response.content_length(size);
    response.body() = std::move(body);
    send(std::move(response));
  }

  beast::tcp_stream stream_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
};

class Server
{
public:
  Server(net::io_context & context, const tcp::endpoint & endpoint) : acceptor_(context)
  {
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(net::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
    accept();
  }

private:
  void accept()
  {
    acceptor_.async_accept([this](beast::error_code error, tcp::socket socket) {
      if (!error) std::make_shared<Session>(std::move(socket))->run();
      else if (error != net::error::operation_aborted) {
        std::cerr << "[urdf_mesh_server] accept: " << error.message() << '\n';
      }
      if (acceptor_.is_open()) accept();
    });
  }

  tcp::acceptor acceptor_;
};
}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::string host = "0.0.0.0";
    unsigned int port = 8090;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--help" || argument == "-h") {
        std::cout << "Usage: urdf_mesh_server [--host IP_ADDRESS] [--port 8090]\n";
        return 0;
      }
      if ((argument != "--host" && argument != "--port") || i + 1 == argc) {
        throw std::runtime_error("Expected --host IP_ADDRESS or --port PORT");
      }
      const std::string value = argv[++i];
      if (argument == "--host") host = value;
      else {
        const auto parsed = std::from_chars(value.data(), value.data() + value.size(), port);
        if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
            port == 0 || port > 65535) throw std::runtime_error("Port must be between 1 and 65535");
      }
    }
    net::io_context context{1};
    Server server(context, {net::ip::make_address(host), static_cast<unsigned short>(port)});
    net::signal_set signals(context, SIGINT, SIGTERM);
    signals.async_wait([&context](beast::error_code, int) { context.stop(); });
    std::cout << "[urdf_mesh_server] listening on " << host << ':' << port
              << " (package://<pkg>/<rel> -> share dir)" << std::endl;
    context.run();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "[urdf_mesh_server] " << error.what() << '\n';
    return 1;
  }
}
