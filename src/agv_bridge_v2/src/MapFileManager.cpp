#include "agv_bridge_v2/MapFileManager.hpp"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace agv_bridge
{

    namespace
    {
        std::string trim(const std::string &s)
        {
            const char *ws = " \t\r\n";
            const auto begin = s.find_first_not_of(ws);
            if (begin == std::string::npos)
            {
                return "";
            }
            const auto end = s.find_last_not_of(ws);
            return s.substr(begin, end - begin + 1);
        }

        /// 去掉两侧引号（yaml 值可能写作 image: "map0921.pgm"）
        std::string stripQuotes(const std::string &s)
        {
            if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                                  (s.front() == '\'' && s.back() == '\'')))
            {
                return s.substr(1, s.size() - 2);
            }
            return s;
        }

        bool fileExists(const std::string &path)
        {
            struct stat st;
            return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
        }

        bool directoryExists(const std::string &path)
        {
            struct stat st;
            return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
        }

        std::string joinPath(const std::string &dir, const std::string &name)
        {
            if (dir.empty())
            {
                return name;
            }
            return dir.back() == '/' ? dir + name : dir + "/" + name;
        }

        /// "[-21.45, -18.15, 0.0]" -> [-21.45, -18.15, 0.0]
        bool parseOrigin(const std::string &value, double &x, double &y, double &yaw)
        {
            std::string body = stripQuotes(trim(value));
            if (!body.empty() && body.front() == '[')
            {
                body = body.substr(1);
            }
            if (!body.empty() && body.back() == ']')
            {
                body.pop_back();
            }
            std::istringstream iss(body);
            std::string token;
            std::vector<double> parts;
            while (std::getline(iss, token, ','))
            {
                try
                {
                    parts.push_back(std::stod(trim(token)));
                }
                catch (const std::exception &)
                {
                    return false;
                }
            }
            if (parts.size() < 2)
            {
                return false;
            }
            x = parts[0];
            y = parts[1];
            yaw = parts.size() >= 3 ? parts[2] : 0.0;
            return true;
        }
    } // namespace

    MapFileManager::MapFileManager(std::string maps_dir)
        : maps_dir_(std::move(maps_dir))
    {
        // 相对路径按启动 cwd 转绝对，保证日志与脚本参数里始终可见真实目录
        if (!maps_dir_.empty() && maps_dir_[0] != '/')
        {
            char *cwd = ::getcwd(nullptr, 0);
            if (cwd != nullptr)
            {
                maps_dir_ = joinPath(cwd, maps_dir_);
                ::free(cwd);
            }
        }
        // 去掉尾部多余的 '/'
        while (maps_dir_.size() > 1 && maps_dir_.back() == '/')
        {
            maps_dir_.pop_back();
        }
    }

    std::vector<MapFileManager::MapEntry> MapFileManager::listMaps(std::string *error) const
    {
        std::set<std::string> yaml_names;
        std::set<std::string> pgm_names;
        std::set<std::string> pbstream_names;

        DIR *dir = ::opendir(maps_dir_.c_str());
        if (dir == nullptr)
        {
            if (error != nullptr)
            {
                *error = "无法打开地图目录: " + maps_dir_;
            }
            return {};
        }
        while (const dirent *entry = ::readdir(dir))
        {
            const std::string filename = entry->d_name;
            if (filename == "." || filename == "..")
            {
                continue;
            }

            // 新结构：maps/<name>/<name>.{yaml,pgm,pbstream}
            const std::string child_path = joinPath(maps_dir_, filename);
            if (isValidMapName(filename) && directoryExists(child_path))
            {
                if (fileExists(joinPath(child_path, filename + ".yaml")))
                    yaml_names.insert(filename);
                if (fileExists(joinPath(child_path, filename + ".pgm")))
                    pgm_names.insert(filename);
                if (fileExists(joinPath(child_path, filename + ".pbstream")))
                    pbstream_names.insert(filename);
                continue;
            }

            // 旧结构：maps/<name>.{yaml,pgm,pbstream}
            const auto dot = filename.rfind('.');
            if (dot == std::string::npos || dot == 0)
            {
                continue;
            }
            const std::string stem = filename.substr(0, dot);
            const std::string ext = filename.substr(dot + 1);
            if (ext == "yaml")
            {
                yaml_names.insert(stem);
            }
            else if (ext == "pgm")
            {
                pgm_names.insert(stem);
            }
            else if (ext == "pbstream")
            {
                pbstream_names.insert(stem);
            }
        }
        ::closedir(dir);

        std::vector<MapEntry> result;
        // 以 pgm+yaml 的交集为准（可导入），pbstream 存在与否仅作记录
        for (const auto &name : pgm_names)
        {
            if (yaml_names.count(name) == 0)
            {
                continue;
            }
            MapEntry item;
            item.name = name;
            item.has_yaml = true;
            item.has_pgm = true;
            item.has_pbstream = pbstream_names.count(name) > 0;
            result.push_back(item);
        }
        std::sort(result.begin(), result.end(),
                  [](const MapEntry &a, const MapEntry &b) { return a.name < b.name; });
        return result;
    }

    std::string MapFileManager::pbstreamPath(const std::string &map_name) const
    {
        const std::string nested = mapStem(map_name) + ".pbstream";
        if (fileExists(nested))
        {
            return nested;
        }
        return joinPath(maps_dir_, map_name + ".pbstream");
    }

    std::string MapFileManager::mapDirectory(const std::string &map_name) const
    {
        return joinPath(maps_dir_, map_name);
    }

    std::string MapFileManager::mapStem(const std::string &map_name) const
    {
        return joinPath(mapDirectory(map_name), map_name);
    }

    bool MapFileManager::ensureMapDirectory(
        const std::string &map_name, std::string &error) const
    {
        if (!isValidMapName(map_name))
        {
            error = "地图名非法: " + map_name;
            return false;
        }

        const std::string directory = mapDirectory(map_name);
        if (directoryExists(directory))
        {
            return true;
        }
        if (::mkdir(directory.c_str(), 0755) == 0)
        {
            return true;
        }
        error = "无法创建地图目录 " + directory + ": " + std::strerror(errno);
        return false;
    }

    bool MapFileManager::hasPbstream(const std::string &map_name) const
    {
        return fileExists(pbstreamPath(map_name));
    }

    bool MapFileManager::isValidMapName(const std::string &map_name)
    {
        if (map_name.empty() || map_name.size() > 128)
        {
            return false;
        }
        if (map_name.find("..") != std::string::npos ||
            map_name.find('/') != std::string::npos ||
            map_name.find('\\') != std::string::npos)
        {
            return false;
        }
        return true;
    }

    bool MapFileManager::parseYaml(const std::string &yaml_path, YamlMeta &meta,
                                   std::string &error) const
    {
        std::ifstream in(yaml_path);
        if (!in.is_open())
        {
            error = "无法打开 yaml: " + yaml_path;
            return false;
        }
        bool has_resolution = false;
        bool has_origin = false;
        std::string line;
        while (std::getline(in, line))
        {
            const std::string text = trim(line);
            if (text.empty() || text[0] == '#')
            {
                continue;
            }
            const auto colon = text.find(':');
            if (colon == std::string::npos)
            {
                continue;
            }
            const std::string key = trim(text.substr(0, colon));
            const std::string value = trim(text.substr(colon + 1));
            try
            {
                if (key == "image")
                {
                    meta.image = stripQuotes(value);
                }
                else if (key == "resolution")
                {
                    meta.resolution = std::stod(value);
                    has_resolution = true;
                }
                else if (key == "origin")
                {
                    if (!parseOrigin(value, meta.origin_x, meta.origin_y, meta.origin_yaw))
                    {
                        error = "origin 字段格式非法: " + value;
                        return false;
                    }
                    has_origin = true;
                }
                else if (key == "negate")
                {
                    meta.negate = std::stoi(value);
                }
                else if (key == "occupied_thresh")
                {
                    meta.occupied_thresh = std::stod(value);
                }
                else if (key == "free_thresh")
                {
                    meta.free_thresh = std::stod(value);
                }
            }
            catch (const std::exception &e)
            {
                error = "yaml 字段解析失败 (" + key + "): " + e.what();
                return false;
            }
        }
        if (meta.image.empty())
        {
            error = "yaml 缺少 image 字段: " + yaml_path;
            return false;
        }
        if (!has_resolution || meta.resolution <= 0.0)
        {
            error = "yaml 缺少合法的 resolution 字段: " + yaml_path;
            return false;
        }
        if (!has_origin)
        {
            error = "yaml 缺少 origin 字段: " + yaml_path;
            return false;
        }
        return true;
    }

    bool MapFileManager::parsePgm(const std::string &pgm_path, int &width, int &height,
                                  std::vector<uint8_t> &pixels, std::string &error) const
    {
        std::ifstream in(pgm_path, std::ios::binary);
        if (!in.is_open())
        {
            error = "无法打开 pgm: " + pgm_path;
            return false;
        }

        // 头部 4 个 token：P5 / width / height / maxval（'#' 开头为行注释）
        std::string magic;
        long w = 0, h = 0, maxval = 0;
        int fields = 0;
        while (fields < 4 && in >> magic)
        {
            if (!magic.empty() && magic[0] == '#')
            {
                // 注释：丢弃到行尾
                std::getline(in, magic);
                continue;
            }
            std::istringstream iss(magic);
            switch (fields)
            {
            case 0:
                if (magic != "P5")
                {
                    error = "pgm 不是二进制 P5 格式: " + pgm_path;
                    return false;
                }
                fields = 1;
                break;
            case 1:
                w = std::stol(magic);
                fields = 2;
                break;
            case 2:
                h = std::stol(magic);
                fields = 3;
                break;
            case 3:
                maxval = std::stol(magic);
                fields = 4;
                break;
            default:
                break;
            }
        }
        if (fields != 4 || w <= 0 || h <= 0 || w * h > 100000000L)
        {
            error = "pgm 头部非法（尺寸异常）: " + pgm_path;
            return false;
        }
        if (maxval <= 0 || maxval >= 256)
        {
            error = "pgm maxval 必须在 1~255（map_server 约定 255）: " + pgm_path;
            return false;
        }

        // maxval 后只有一个空白符，其后是二进制像素
        in.get();
        pixels.assign(static_cast<size_t>(w) * static_cast<size_t>(h), 0);
        in.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
        if (static_cast<size_t>(in.gcount()) != pixels.size())
        {
            error = "pgm 像素数据不完整（期望 " + std::to_string(pixels.size()) +
                    " 字节，实际 " + std::to_string(in.gcount()) + "）: " + pgm_path;
            return false;
        }
        width = static_cast<int>(w);
        height = static_cast<int>(h);
        return true;
    }

    bool MapFileManager::loadGrid(const std::string &map_name,
                                  nav_msgs::msg::OccupancyGrid &grid,
                                  std::string &error) const
    {
        if (!isValidMapName(map_name))
        {
            error = "地图名非法（非空且不含路径分隔符）: " + map_name;
            return false;
        }
        const std::string nested_yaml = mapStem(map_name) + ".yaml";
        const std::string yaml_path = fileExists(nested_yaml)
                                          ? nested_yaml
                                          : joinPath(maps_dir_, map_name + ".yaml");
        YamlMeta meta;
        if (!parseYaml(yaml_path, meta, error))
        {
            return false;
        }
        // image 相对路径相对 yaml 所在目录解析
        std::string pgm_path = meta.image;
        if (!pgm_path.empty() && pgm_path[0] != '/')
        {
            const auto slash = yaml_path.rfind('/');
            pgm_path = slash == std::string::npos
                           ? pgm_path
                           : yaml_path.substr(0, slash + 1) + pgm_path;
        }
        int width = 0;
        int height = 0;
        std::vector<uint8_t> pixels;
        if (!parsePgm(pgm_path, width, height, pixels, error))
        {
            return false;
        }

        grid = nav_msgs::msg::OccupancyGrid();
        grid.header.frame_id = "map";
        grid.header.stamp = rclcpp::Clock().now();
        grid.info.resolution = meta.resolution;
        grid.info.width = static_cast<uint32_t>(width);
        grid.info.height = static_cast<uint32_t>(height);
        grid.info.origin.position.x = meta.origin_x;
        grid.info.origin.position.y = meta.origin_y;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = std::sin(meta.origin_yaw / 2.0);
        grid.info.origin.orientation.w = std::cos(meta.origin_yaw / 2.0);

        grid.data.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
        for (int row = 0; row < height; ++row)
        {
            // pgm 第 0 行是世界 y 最大的一侧，OccupancyGrid 第 0 行是 y 最小的一侧
            const int out_row = height - 1 - row;
            const uint8_t *src = &pixels[static_cast<size_t>(row) * width];
            int8_t *dst = &grid.data[static_cast<size_t>(out_row) * width];
            for (int col = 0; col < width; ++col)
            {
                const double p = meta.negate != 0
                                     ? static_cast<double>(src[col]) / 255.0
                                     : (255.0 - static_cast<double>(src[col])) / 255.0;
                if (p > meta.occupied_thresh)
                {
                    dst[col] = 100;
                }
                else if (p < meta.free_thresh)
                {
                    dst[col] = 0;
                }
                else
                {
                    dst[col] = -1;
                }
            }
        }
        return true;
    }

} // namespace agv_bridge
