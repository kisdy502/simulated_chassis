#ifndef AGV_MAP_FILE_MANAGER_HPP
#define AGV_MAP_FILE_MANAGER_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace agv_bridge
{

    /**
     * @brief 地图文件管理：扫描地图目录、解析 map_server 三件套（pgm + yaml，pbstream 可选）。
     *
     * 新目录约定：<maps_dir>/<map_name>/<map_name>.yaml|pgm|pbstream。
     * 为兼容旧数据，读取时仍支持 <maps_dir>/<map_name>.* 平铺结构。
     * yaml 为 map_server 标准格式：image / resolution / origin / negate /
     * occupied_thresh / free_thresh，image 相对路径相对 yaml 所在目录解析。
     *
     * 栅格转换遵循 map_server 语义：
     *   p = (255 - pixel) / 255.0（negate=0），p > occupied_thresh → 100（占据），
     *   p < free_thresh → 0（空闲），否则 -1（未知）。
     * pgm 第 0 行是世界坐标 y 最大的一侧，OccupancyGrid 第 0 行是 y 最小的一侧，
     * 写入 data 时按行上下翻转。
     */
    class MapFileManager
    {
    public:
        struct MapEntry
        {
            std::string name;
            bool has_yaml = false;
            bool has_pgm = false;
            bool has_pbstream = false;
        };

        struct YamlMeta
        {
            std::string image;          // yaml 里 image 字段原值（可能带引号/相对路径）
            double resolution = 0.05;
            double origin_x = 0.0;
            double origin_y = 0.0;
            double origin_yaw = 0.0;
            int negate = 0;
            double occupied_thresh = 0.65;
            double free_thresh = 0.196;
        };

        explicit MapFileManager(std::string maps_dir);

        /// @brief 解析后的地图目录（相对路径会按启动 cwd 转绝对路径）
        const std::string &mapsDir() const { return maps_dir_; }

        /// @brief 扫描目录中 pgm+yaml 齐全的地图，按名称排序
        std::vector<MapEntry> listMaps(std::string *error = nullptr) const;

        /// @brief pbstream 文件是否存在（load_map 的前置条件）
        bool hasPbstream(const std::string &map_name) const;

        /// @brief pbstream 绝对路径（优先子目录，兼容旧平铺文件）
        std::string pbstreamPath(const std::string &map_name) const;

        /// @brief 新结构中的地图目录与无扩展名文件前缀。
        std::string mapDirectory(const std::string &map_name) const;
        std::string mapStem(const std::string &map_name) const;

        /// @brief 为新地图创建独立子目录。
        bool ensureMapDirectory(const std::string &map_name, std::string &error) const;

        /// @brief 读取 <map_name>.pgm/.yaml 并转换成 OccupancyGrid
        bool loadGrid(const std::string &map_name,
                      nav_msgs::msg::OccupancyGrid &grid,
                      std::string &error) const;

        /// @brief 校验地图名：非空、不含路径分隔符与 ".."，防止目录穿越
        static bool isValidMapName(const std::string &map_name);

    private:
        bool parseYaml(const std::string &yaml_path, YamlMeta &meta, std::string &error) const;
        bool parsePgm(const std::string &pgm_path, int &width, int &height,
                      std::vector<uint8_t> &pixels, std::string &error) const;

        std::string maps_dir_;
    };

} // namespace agv_bridge

#endif // AGV_MAP_FILE_MANAGER_HPP
