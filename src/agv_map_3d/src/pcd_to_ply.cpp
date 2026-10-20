// Lightning-LM 点云地图导出器：global.pcd → 二进制 PLY（XYZ, frame map）。
// 输出格式与 pbstream_to_ply 完全一致，上位机三维地图视图无差别消费。
//
// 与 Cartographer 链路的差异：lightning 的建图产物本来就是 3D 点云
// （global.pcd，含回环优化后的关键帧拼接），无需从占据体素恢复高度，
// 这里只做体素降采样控制体积。
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

constexpr size_t kMaxPoints = 5000000;  // 与 pbstream_to_ply 相同的上限保护

void write_ply(const std::string& path, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    const std::string temporary = path + ".tmp";
    std::ofstream out(temporary, std::ios::binary);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << "ply\nformat binary_little_endian 1.0\ncomment frame_id map\nelement vertex " << cloud.size()
        << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    const uint16_t endian = 1;
    if (*reinterpret_cast<const uint8_t*>(&endian) != 1) throw std::runtime_error("Little endian host required");
    for (const auto& p : cloud.points) {
        out.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
        out.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
        out.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
    }
    out.close();
    std::filesystem::rename(temporary, path);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 3 || argc > 4) {
            std::cerr << "Usage: pcd_to_ply input.pcd output.ply [voxel=0.1]\n";
            return 2;
        }
        const std::string option = argc > 3 ? argv[3] : "0.1";
        std::size_t consumed = 0;
        const double voxel = std::stod(option, &consumed);
        if (consumed != option.size() || !std::isfinite(voxel) || voxel < 0.001 || voxel >= 10) throw std::runtime_error("Invalid voxel size");

        // lightning global.pcd 为 binary compressed 的 XYZI；PCL 按字段名加载，
        // 缺 intensity 字段的点云同样可读。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDReader reader;
        if (reader.read(argv[1], *cloud) < 0 || cloud->empty())
            throw std::runtime_error("No points in PCD (empty or unreadable map)");
        // PCDs can incorrectly advertise is_dense despite NaNs. Filter explicitly,
        // in place, without allocating both XYZI and XYZ copies of the whole map.
        auto end = std::remove_if(cloud->points.begin(), cloud->points.end(), [](const auto& point) {
            return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
        });
        cloud->points.erase(end, cloud->points.end());
        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;
        if (cloud->empty()) throw std::runtime_error("All points are non-finite");

        // 统一走体素质心降采样，控制传输体积（voxel 已验证 > 0）
        pcl::PointXYZ minimum, maximum;
        pcl::getMinMax3D(*cloud, minimum, maximum);
        double cells = 1;
        for (int axis = 0; axis < 3; ++axis) {
            const double low = std::floor(minimum.data[axis] / voxel);
            const double high = std::floor(maximum.data[axis] / voxel);
            cells *= high - low + 1;
            if (low < std::numeric_limits<int32_t>::min() || high > std::numeric_limits<int32_t>::max() ||
                cells > std::numeric_limits<int32_t>::max())
                throw std::runtime_error("PCL voxel index range exceeded; increase voxel size");
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize(static_cast<float>(voxel), static_cast<float>(voxel), static_cast<float>(voxel));
        grid.setInputCloud(cloud);
        grid.filter(*filtered);
        if (filtered->empty()) throw std::runtime_error("Voxel filter removed every point; decrease voxel size");
        if (filtered->size() > kMaxPoints)
            throw std::runtime_error("Map exceeds five million points after filtering; increase voxel size");

        write_ply(argv[2], *filtered);
        std::cout << "Exported " << filtered->size() << " points (raw " << cloud->size() << ") in frame map\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
