#include <cartographer/io/proto_stream.h>
#include <cartographer/mapping/3d/hybrid_grid.h>
#include <cartographer/mapping/proto/serialization.pb.h>
#include <cartographer/transform/transform.h>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  cartographer::io::ProtoStreamWriter writer(argv[1]);
  cartographer::mapping::proto::SerializationHeader header;
  header.set_format_version(2);
  writer.WriteProto(header);
  cartographer::mapping::proto::SerializedData graph;
  auto* trajectory = graph.mutable_pose_graph()->add_trajectory();
  trajectory->set_trajectory_id(0);
  for (int i = 0; i < 2; ++i) {
    auto* submap = trajectory->add_submap();
    submap->set_submap_index(i);
    *submap->mutable_pose() = cartographer::transform::ToProto(
        cartographer::transform::Rigid3d(Eigen::Vector3d(10, 20, 3),
           Eigen::Quaterniond(Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ()))));
  }
  writer.WriteProto(graph);
  cartographer::mapping::proto::SerializedData options;
  auto* trajectory_options = options.mutable_all_trajectory_builder_options()->add_options_with_sensor_ids();
  trajectory_options->mutable_trajectory_builder_options()->mutable_trajectory_builder_3d_options();
  writer.WriteProto(options);
  for (int i = 0; i < 2; ++i) {
    cartographer::mapping::proto::SerializedData data;
    auto* submap = data.mutable_submap();
    submap->mutable_submap_id()->set_trajectory_id(0);
    submap->mutable_submap_id()->set_submap_index(i);
    auto* three = submap->mutable_submap_3d();
    *three->mutable_local_pose() = cartographer::transform::ToProto(
        cartographer::transform::Rigid3d::Translation(Eigen::Vector3d(100, 200, 0)));
    cartographer::mapping::HybridGrid grid(1.0);
    grid.SetProbability(Eigen::Array3i(1, 0, 2), 0.8);
    grid.SetProbability(Eigen::Array3i(2, 0, 2), 0.2);
    *three->mutable_high_resolution_hybrid_grid() = grid.ToProto();
    writer.WriteProto(data);
  }
  return writer.Close() ? 0 : 1;
}
