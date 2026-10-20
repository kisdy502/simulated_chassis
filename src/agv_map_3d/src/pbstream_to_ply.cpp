#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include <cartographer/mapping/3d/hybrid_grid.h>
#include <cartographer/transform/transform.h>
#include <cmath>
#include <array>
#include <limits>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <tuple>

// Hybrid-grid cells are in submap coordinates. Apply the optimized global
// submap pose directly; applying inverse(local_pose) here would move them twice.
int main(int argc, char** argv) {
  try {
    if (argc < 3 || argc > 5) {
      std::cerr << "Usage: pbstream_to_ply input.pbstream output.ply [voxel=0.1] [probability=0.55]\n";
      return 2;
    }
    const std::string voxel_option = argc > 3 ? argv[3] : "0.1";
    const std::string threshold_option = argc > 4 ? argv[4] : "0.55";
    std::size_t voxel_used = 0, threshold_used = 0;
    const double voxel = std::stod(voxel_option, &voxel_used);
    const double threshold = std::stod(threshold_option, &threshold_used);
    if (voxel_used != voxel_option.size() || threshold_used != threshold_option.size() ||
        !std::isfinite(voxel) || voxel < 0.001 || voxel >= 10 || !std::isfinite(threshold) || threshold <= 0.5 || threshold >= 1)
      throw std::runtime_error("Invalid voxel size or occupied probability");
    cartographer::io::ProtoStreamReader reader(argv[1]);
    cartographer::io::ProtoStreamDeserializer deserializer(&reader);
    using Id = std::pair<int, int>;
    std::map<Id, cartographer::transform::Rigid3d> poses;
    for (const auto& trajectory : deserializer.pose_graph().trajectory())
      for (const auto& submap : trajectory.submap())
        poses.emplace(Id{trajectory.trajectory_id(), submap.submap_index()},
                      cartographer::transform::ToRigid3(submap.pose()));
    std::map<std::tuple<int64_t, int64_t, int64_t>, Eigen::Vector3f> points;
    cartographer::mapping::proto::SerializedData data;
    int submaps = 0;
    while (deserializer.ReadNextSerializedData(&data)) {
      if (!data.has_submap() || !data.submap().has_submap_3d()) continue;
      const auto& submap = data.submap();
      const auto pose = poses.find({submap.submap_id().trajectory_id(), submap.submap_id().submap_index()});
      if (pose == poses.end()) throw std::runtime_error("Missing optimized submap pose");
      const auto& proto = submap.submap_3d();
      const auto global_from_local = pose->second.cast<float>();
      if (!proto.has_high_resolution_hybrid_grid()) throw std::runtime_error("3D submap has no occupancy grid");
      const auto& grid = proto.high_resolution_hybrid_grid();
      if (!std::isfinite(grid.resolution()) || grid.resolution() <= 0 || grid.values_size() != grid.x_indices_size() ||
          grid.values_size() != grid.y_indices_size() || grid.values_size() != grid.z_indices_size())
        throw std::runtime_error("Invalid 3D grid");
      ++submaps;
      // Iterate serialized sparse cells directly; rebuilding HybridGrid would
      // allocate large blocks and perform unnecessary probability conversions.
      for (int i = 0; i < grid.values_size(); ++i) {
        if (cartographer::mapping::ValueToProbability(grid.values(i)) < threshold) continue;
        const Eigen::Vector3f center = Eigen::Vector3f(grid.x_indices(i), grid.y_indices(i), grid.z_indices(i)) * grid.resolution();
        const Eigen::Vector3f p = global_from_local * center;
        if (!p.allFinite()) continue;
        std::array<int64_t, 3> cell;
        for (int axis = 0; axis < 3; ++axis) {
          const double coordinate = std::floor(static_cast<double>(p[axis]) / voxel);
          if (!std::isfinite(coordinate) || coordinate < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
              coordinate >= static_cast<double>(std::numeric_limits<int64_t>::max()))
            throw std::runtime_error("Voxel coordinate outside supported range");
          cell[axis] = static_cast<int64_t>(coordinate);
        }
        points.emplace(std::make_tuple(cell[0], cell[1], cell[2]), p);
        if (points.size() > 5000000) throw std::runtime_error("Map exceeds five million points; increase voxel size");
      }
    }
    if (!reader.eof()) throw std::runtime_error("Truncated PBStream");
    if (!submaps || points.empty()) throw std::runtime_error("No occupied 3D submaps (2D maps cannot supply heights)");
    const std::string temporary = std::string(argv[2]) + ".tmp";
    std::ofstream out(temporary, std::ios::binary);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << "ply\nformat binary_little_endian 1.0\ncomment frame_id map\nelement vertex " << points.size()
        << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    const uint16_t endian = 1;
    if (*reinterpret_cast<const uint8_t*>(&endian) != 1) throw std::runtime_error("Little endian host required");
    for (const auto& entry : points)
      for (int axis = 0; axis < 3; ++axis) {
        const float value = entry.second[axis];
        out.write(reinterpret_cast<const char*>(&value), sizeof(value));
      }
    out.close();
    std::filesystem::rename(temporary, argv[2]);
    std::cout << "Exported " << points.size() << " points from " << submaps << " submaps in frame map\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
