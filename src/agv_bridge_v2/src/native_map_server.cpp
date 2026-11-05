#include <archive.h>
#include <archive_entry.h>
#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    namespace fs = std::filesystem;
    namespace asio = boost::asio;
    namespace http = boost::beast::http;
    using asio::ip::tcp;

    constexpr std::size_t MAX_ZIP = 128 * 1024 * 1024;
    constexpr std::size_t MAX_EXPANDED = 512 * 1024 * 1024;
    constexpr std::size_t MAX_FILES = 1024;
    const std::regex FILE_NAME("[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}");
    const std::regex TILE_ID("[0-9]+");
    const std::string ROUTE = "/native/lightning/";
    using MapFiles = std::map<std::string, fs::path>;

    void check_archive(int result)
    {
        if (result != ARCHIVE_OK)
            throw std::invalid_argument("invalid Lightning ZIP");
    }

    MapFiles read_map_files(const fs::path &directory)
    {
        MapFiles files;
        std::size_t size = 0;
        for (const auto &file : fs::directory_iterator(directory))
        {
            const auto name = file.path().filename().string();
            if (name == "last_pose.yaml" || !std::regex_match(name, FILE_NAME) ||
                !fs::is_regular_file(file.symlink_status()))
                continue;
            const auto file_size = file.file_size();
            if (files.size() == MAX_FILES || file_size > MAX_EXPANDED - size)
                throw std::invalid_argument("Lightning map exceeds limits");
            size += file_size;
            files.emplace(name, file.path());
        }
        for (const auto *name : {"map.yaml", "map.pgm", "global.pcd", "index.txt"})
        {
            const auto file = files.find(name);
            if (file == files.end() || fs::file_size(file->second) == 0)
                throw std::invalid_argument("incomplete Lightning map");
        }

        std::ifstream yaml(files.at("map.yaml"));
        std::string line;
        std::size_t image_fields = 0;
        while (std::getline(yaml, line))
        {
            if (line.rfind("image:", 0) != 0)
                continue;
            std::istringstream value(line.substr(6));
            std::string image;
            value >> image;
            if (++image_fields != 1 || image != "map.pgm")
                throw std::invalid_argument("map.yaml must reference map.pgm");
        }
        if (image_fields != 1)
            throw std::invalid_argument("map.yaml is missing image");

        std::ifstream index(files.at("index.txt"));
        if (!std::getline(index, line))
            throw std::invalid_argument("empty Lightning index");
        while (std::getline(index, line))
        {
            if (line == "# functional points" || line == "# functional points\r")
                break;
            std::istringstream row(line);
            std::string id;
            int x, y;
            if (row >> id)
            {
                if (!std::regex_match(id, TILE_ID) || !(row >> x >> y) ||
                    files.count(id + ".pcd") == 0)
                    throw std::invalid_argument("Lightning index references an invalid tile");
            }
        }
        return files;
    }

    la_ssize_t write_zip(struct archive *, void *client, const void *data, std::size_t size)
    {
        auto &zip = *static_cast<std::string *>(client);
        if (size > MAX_ZIP - zip.size())
            return -1;
        try
        {
            zip.append(static_cast<const char *>(data), size);
        }
        catch (const std::exception &)
        {
            return -1;
        }
        return static_cast<la_ssize_t>(size);
    }

    std::string export_map(const fs::path &directory)
    {
        const auto files = read_map_files(directory);
        std::string zip;
        std::unique_ptr<struct archive, decltype(&archive_write_free)> writer(
            archive_write_new(), archive_write_free);
        check_archive(archive_write_set_format_zip(writer.get()));
        check_archive(archive_write_set_bytes_in_last_block(writer.get(), 1));
        check_archive(archive_write_open(writer.get(), &zip, nullptr, write_zip, nullptr));
        std::array<char, 64 * 1024> buffer;
        for (const auto &[name, path] : files)
        {
            std::unique_ptr<struct archive_entry, decltype(&archive_entry_free)> entry(
                archive_entry_new(), archive_entry_free);
            archive_entry_set_pathname(entry.get(), name.c_str());
            archive_entry_set_filetype(entry.get(), AE_IFREG);
            archive_entry_set_perm(entry.get(), 0644);
            const auto size = fs::file_size(path);
            archive_entry_set_size(entry.get(), size);
            check_archive(archive_write_header(writer.get(), entry.get()));
            std::ifstream input(path, std::ios::binary);
            std::uintmax_t written = 0;
            while (input.read(buffer.data(), buffer.size()) || input.gcount() != 0)
            {
                if (archive_write_data(writer.get(), buffer.data(), input.gcount()) != input.gcount())
                    throw std::invalid_argument("Lightning ZIP exceeds limits");
                written += input.gcount();
            }
            if (written != size || !input.eof())
                throw std::runtime_error("map file changed during export");
            check_archive(archive_write_finish_entry(writer.get()));
        }
        check_archive(archive_write_close(writer.get()));
        return zip;
    }

    class StagingDirectory
    {
    public:
        explicit StagingDirectory(const fs::path &root)
        {
            const auto pattern = (root / ".install-XXXXXX").string();
            std::vector<char> name(pattern.begin(), pattern.end());
            name.push_back('\0');
            if (!mkdtemp(name.data()))
                throw std::runtime_error("cannot create map staging directory");
            path = name.data();
        }

        ~StagingDirectory()
        {
            std::error_code error;
            fs::remove_all(path, error);
        }

        fs::path path;
    };

    void extract_map(const std::string &zip, const fs::path &directory)
    {
        std::unique_ptr<struct archive, decltype(&archive_read_free)> reader(
            archive_read_new(), archive_read_free);
        check_archive(archive_read_support_format_zip(reader.get()));
        check_archive(archive_read_open_memory(reader.get(), zip.data(), zip.size()));
        struct archive_entry *entry = nullptr;
        std::array<char, 64 * 1024> buffer;
        std::size_t files = 0;
        std::size_t size = 0;
        int result;
        while ((result = archive_read_next_header(reader.get(), &entry)) != ARCHIVE_EOF)
        {
            check_archive(result);
            const char *pathname = archive_entry_pathname(entry);
            if (++files > MAX_FILES || !pathname || !std::regex_match(pathname, FILE_NAME) ||
                std::string(pathname) == "last_pose.yaml" || archive_entry_filetype(entry) != AE_IFREG ||
                archive_entry_hardlink(entry) || archive_entry_symlink(entry) ||
                fs::exists(directory / pathname))
                throw std::invalid_argument("invalid native map entry");

            std::ofstream output(directory / pathname, std::ios::binary);
            output.exceptions(std::ios::badbit | std::ios::failbit);
            la_ssize_t count;
            while ((count = archive_read_data(reader.get(), buffer.data(), buffer.size())) > 0)
            {
                if (static_cast<std::size_t>(count) > MAX_EXPANDED - size)
                    throw std::invalid_argument("Lightning map exceeds expanded size limit");
                size += count;
                output.write(buffer.data(), count);
            }
            if (count < 0)
                throw std::invalid_argument("invalid Lightning ZIP data");
            output.close();
        }
    }

    bool same_files(const MapFiles &left, const MapFiles &right)
    {
        if (left.size() != right.size())
            return false;
        for (const auto &[name, path] : left)
        {
            const auto other = right.find(name);
            if (other == right.end() || fs::file_size(path) != fs::file_size(other->second))
                return false;
            std::ifstream a(path, std::ios::binary), b(other->second, std::ios::binary);
            if (!a || !b || !std::equal(std::istreambuf_iterator<char>(a), {},
                                       std::istreambuf_iterator<char>(b), {}))
                return false;
        }
        return true;
    }

    http::status install_map(const fs::path &target, const std::string &zip)
    {
        StagingDirectory staging(target.parent_path());
        const auto directory = staging.path / "map";
        fs::create_directory(directory);
        extract_map(zip, directory);
        const auto files = read_map_files(directory);
        if (fs::is_symlink(target))
            return http::status::conflict;
        if (fs::exists(target))
        {
            try
            {
                if (fs::is_directory(target) && same_files(files, read_map_files(target)))
                    return http::status::no_content;
            }
            catch (const std::invalid_argument &)
            {
                // 已有同名但不完整的地图也保留，由调用方选择其他名称。
            }
            return http::status::conflict;
        }
        fs::rename(directory, target);
        return http::status::no_content;
    }

    bool authorized(const std::string &expected, boost::beast::string_view actual)
    {
        if (actual.size() != expected.size())
            return false;
        unsigned char difference = 0;
        for (std::size_t i = 0; i < expected.size(); ++i)
            difference |= expected[i] ^ actual[i];
        return difference == 0;
    }

    void serve(boost::beast::tcp_stream &stream, asio::io_context &context,
               const fs::path &root, const std::string &authorization)
    {
        boost::beast::flat_buffer buffer;
        http::request_parser<http::string_body> parser;
        parser.body_limit(MAX_ZIP);
        boost::system::error_code error;
        const auto complete = [&](boost::system::error_code result, std::size_t)
        {
            error = result;
        };
        const auto run = [&]()
        {
            context.restart();
            context.run();
        };
        stream.expires_after(std::chrono::seconds(30));
        http::async_read_header(stream, buffer, parser, complete);
        run();
        const auto &request = parser.get();
        http::response<http::string_body> response{http::status::bad_request, request.version()};
        response.keep_alive(false);
        if (error == http::error::body_limit)
            response.result(http::status::payload_too_large);
        else if (error)
            return;
        else if (!authorized(authorization, request[http::field::authorization]))
            response.result(http::status::unauthorized);
        else
        {
            const std::string path(request.target());
            const auto name = path.compare(0, ROUTE.size(), ROUTE) == 0 ? path.substr(ROUTE.size()) : "";
            if (std::regex_match(name, FILE_NAME))
            {
                const auto target = root / name;
                try
                {
                    if (request.method() == http::verb::get)
                    {
                        if (fs::is_symlink(target) || !fs::is_directory(target))
                            response.result(http::status::not_found);
                        else
                        {
                            response.body() = export_map(target);
                            response.result(http::status::ok);
                            response.set(http::field::content_type, "application/zip");
                        }
                    }
                    else if (request.method() == http::verb::put)
                    {
                        if (!parser.content_length() || *parser.content_length() == 0)
                            response.result(http::status::payload_too_large);
                        else
                        {
                            stream.expires_after(std::chrono::seconds(30));
                            http::async_read(stream, buffer, parser, complete);
                            run();
                            if (!error)
                                response.result(install_map(target, request.body()));
                        }
                    }
                    else
                        response.result(http::status::method_not_allowed);
                }
                catch (const std::invalid_argument &)
                {
                    response.result(http::status::unprocessable_entity);
                }
                catch (const std::exception &exception)
                {
                    std::cerr << "Lightning map transfer failed: " << exception.what() << '\n';
                    response.result(http::status::internal_server_error);
                }
            }
        }
        if (response.result() != http::status::no_content)
            response.prepare_payload();
        stream.expires_after(std::chrono::seconds(30));
        http::async_write(stream, response, complete);
        run();
    }
}

int main(int argc, char **argv)
{
    const char *token = std::getenv("NATIVE_MAP_TOKEN");
    if (argc != 3 || std::string(argv[1]) != "--maps-dir" || !token || !*token)
    {
        std::cerr << "Usage: native_map_server --maps-dir PATH (set NATIVE_MAP_TOKEN)\n";
        return 2;
    }

    try
    {
        const fs::path root = argv[2];
        fs::create_directories(root);
        const std::string authorization = std::string("Bearer ") + token;
        asio::io_context context;
        tcp::acceptor acceptor(context, {tcp::v4(), 8091});
        for (;;)
        {
            boost::beast::tcp_stream stream(context);
            acceptor.accept(stream.socket());
            serve(stream, context, root, authorization);
        }
    }
    catch (const std::exception &exception)
    {
        std::cerr << "Native map server failed: " << exception.what() << '\n';
        return 1;
    }
}
