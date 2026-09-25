#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * 双 3D 雷达地面滤除节点
 *
 * 问题背景：雷达安装高度仅 0.25m（距底盘顶面 1cm），垂直视场下沿 -10°，
 * 前后倾 5° 后约 1~3m 内即可打到地面，地面回波会：
 *   1. 被 cartographer_occupancy_grid_node 投影成 2D 地图上散落的灰色点（假障碍）；
 *   2. 以近处高密度、带垂直波束量化误差的形式进入子图，干扰 3D 扫描匹配的 z 方向。
 *
 * 处理方式：订阅 /points2_1 /points2_2，把点变换到 target_frame（base_footprint，
 * z=0 即地面）做高度裁剪，保留 [min_z, max_z] 区间内的点，点本身仍留在传感器
 * 坐标系发布到 *_filtered 话题（header 不变），Cartographer 侧仅需重映射话题。
 *
 * 雷达与底盘全部是固定关节，TF 查一次按 frame 缓存即可。
 */
class GroundFilterNode : public rclcpp::Node
{
public:
  GroundFilterNode()
  : Node("pointcloud_ground_filter")
  {
    declare_parameter<double>("min_z", 0.10);   // 地面上方 10cm 以下的点全部丢弃
    declare_parameter<double>("max_z", 3.00);   // 天花板以上不参与建图
    declare_parameter<std::string>("target_frame", "base_footprint");
    min_z_ = get_parameter("min_z").as_double();
    max_z_ = get_parameter("max_z").as_double();
    target_frame_ = get_parameter("target_frame").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    pub_front_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points2_1_filtered", 10);
    pub_rear_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points2_2_filtered", 10);
    sub_front_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points2_1", 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {filter(msg, pub_front_);});
    sub_rear_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points2_2", 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {filter(msg, pub_rear_);});

    RCLCPP_INFO(
      get_logger(),
      "地面滤除已启动: 保留 %s 系 z ∈ [%.2f, %.2f] m",
      target_frame_.c_str(), min_z_, max_z_);
  }

private:
  void filter(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
  {
    // 固定关节 → 静态 TF，按 frame_id 缓存变换矩阵
    Eigen::Isometry3d sensor_to_target;
    auto it = tf_cache_.find(msg->header.frame_id);
    if (it != tf_cache_.end()) {
      sensor_to_target = it->second;
    } else {
      if (!tf_buffer_->canTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "等待 TF %s -> %s，期间点云丢弃",
          msg->header.frame_id.c_str(), target_frame_.c_str());
        return;
      }
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
      const auto & t = tf.transform;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translate(Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z));
      pose.rotate(Eigen::Quaterniond(
          t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).normalized());
      sensor_to_target = pose;
      tf_cache_[msg->header.frame_id] = pose;
    }

    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    if (n == 0) {
      return;
    }

    // 先标记保留点，再按 point_step 整行拷贝，保住 intensity 等附加字段
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    std::vector<bool> keep(n, false);
    size_t kept = 0;
    for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
      const Eigen::Vector3d p(*iter_x, *iter_y, *iter_z);
      if (!p.allFinite()) {
        continue;
      }
      const double z_base = (sensor_to_target * p).z();
      if (z_base < min_z_ || z_base > max_z_) {
        continue;
      }
      keep[i] = true;
      ++kept;
    }
    if (kept == 0) {
      return;
    }

    auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
    out->header = msg->header;
    out->fields = msg->fields;
    out->point_step = msg->point_step;
    out->height = 1;
    out->width = static_cast<uint32_t>(kept);
    out->is_bigendian = msg->is_bigendian;
    out->is_dense = true;
    out->data.resize(kept * msg->point_step);
    const uint8_t * src = msg->data.data();
    uint8_t * dst = out->data.data();
    for (size_t i = 0; i < n; ++i, src += msg->point_step) {
      if (!keep[i]) {
        continue;
      }
      std::memcpy(dst, src, msg->point_step);
      dst += msg->point_step;
    }
    pub->publish(std::move(*out));
  }

  double min_z_;
  double max_z_;
  std::string target_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string, Eigen::Isometry3d> tf_cache_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_front_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_rear_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rear_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilterNode>());
  rclcpp::shutdown();
  return 0;
}
