/**
 * dual_lidar_merge_node：双 3D 雷达点云合流节点（lightning-lm 接入专用）
 *
 * 背景：lightning-lm 只支持单一雷达话题，而本机为前后双 Mid-360（仿真）。
 * 本节点把 /points2_1（front_lidar_link）与 /points2_2（rear_lidar_link）
 * 按近似时间同步配对，经 TF 变换到 target_frame（base_link）后拼接输出，
 * 并为每个点补写 Velodyne 格式所需的 time（ms）/ ring 字段：
 *
 *   - lightning-lm 走 lidar_type=2（Velodyne）分支，要求点云字段
 *     x,y,z,intensity,time,ring；当最后一帧点 time>0 时直接采用消息内时间
 *     （given_offset_time_=true），否则按扫描角速度自推算（仿真雷达角速
 *     与真机 Velodyne 不一致，推算值有偏差）。
 *   - time_mode=zero：所有点 time=0，仅最后一个点写极小正数（1e-4ms），
 *     触发 given_offset_time_=true 且几乎无畸变修正 —— 适用于快照式仿真
 *     点云（同一仿真时刻采集，帧内无运动畸变，默认）。
 *   - time_mode=linear：按点序线性铺满 scan_period_ms —— 适用于真实旋转
 *     式雷达或需要模拟帧内扫描行为的场合。
 *
 * 输出话题 /points2_merged（参数可改），frame_id = target_frame。
 * QoS：订阅端 SensorDataQoS（best_effort，对可靠/尽力发布端均兼容）；
 *       发布端 RELIABLE（匹配 lightning-lm 的默认订阅）。
 */

#include <geometry_msgs/msg/transform_stamped.hpp>
// ROS1 风格头文件名在 Humble 不存在，改用 ROS2 实际路径
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace simulated_chassis {

class DualLidarMergeNode : public rclcpp::Node {
public:
  explicit DualLidarMergeNode(const rclcpp::NodeOptions &options)
      : Node("dual_lidar_merge_node", options) {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    time_mode_ = declare_parameter<std::string>("time_mode", "zero");
    scan_period_ms_ = declare_parameter<double>("scan_period_ms", 66.7);

    const std::string topic_front = declare_parameter<std::string>("cloud_front_topic", "/points2_1");
    const std::string topic_rear = declare_parameter<std::string>("cloud_rear_topic", "/points2_2");
    const std::string topic_out = declare_parameter<std::string>("cloud_merged_topic", "/points2_merged");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_out, 10);

    // Humble 的 message_filters::subscribe 需要 rmw qos + options 四参形式
    sub_front_.subscribe(this, topic_front, rmw_qos_profile_sensor_data,
                         rclcpp::SubscriptionOptions());
    sub_rear_.subscribe(this, topic_rear, rmw_qos_profile_sensor_data,
                        rclcpp::SubscriptionOptions());
    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_front_, sub_rear_);
    sync_->registerCallback(
        std::bind(&DualLidarMergeNode::onCloudPair, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
                "dual lidar merge: %s + %s -> %s (frame=%s, time_mode=%s, period=%.1fms)", topic_front.c_str(),
                topic_rear.c_str(), topic_out.c_str(), target_frame_.c_str(), time_mode_.c_str(), scan_period_ms_);
  }

private:
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void onCloudPair(const CloudMsg::ConstSharedPtr &front, const CloudMsg::ConstSharedPtr &rear) {
    CloudMsg out;
    out.header.stamp = front->header.stamp;  // 以前雷达时间戳为准
    out.header.frame_id = target_frame_;
    out.height = 1;
    out.is_dense = false;

    const bool ok_front = appendCloud(*front, out);
    const bool ok_rear = appendCloud(*rear, out);
    if (!ok_front && !ok_rear) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "both lidar transforms failed, frame dropped");
      return;
    }
    if (out.width == 0) {
      return;
    }

    // 保证最后一个点 time>0，触发 lightning-lm 的 given_offset_time_=true
    sensor_msgs::PointCloud2Iterator<float> out_time(out, "time");
    out_time += static_cast<uint32_t>(out.width) - 1;
    if (*out_time <= 0.0f) {
      *out_time = 1e-4f;
    }

    pub_->publish(out);
  }

  /// 将单帧点云变换到 target_frame 后追加到 out；返回是否成功
  bool appendCloud(const CloudMsg &cloud, CloudMsg &out) {
    const uint32_t n_in = cloud.width * cloud.height;
    if (n_in == 0) {
      return false;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      // 雷达外参为静态 TF（robot_state_publisher 发布），取最新值即可
      tf_msg = tf_buffer_->lookupTransform(target_frame_, cloud.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "lookupTransform(%s<-%s) failed: %s",
                           target_frame_.c_str(), cloud.header.frame_id.c_str(), e.what());
      return false;
    }
    const tf2::Transform tf(
        tf2::Quaternion(tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z,
                        tf_msg.transform.rotation.w),
        tf2::Vector3(tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z));

    if (out.width == 0) {
      initOutFields(out);
    }
    const uint32_t base = out.width;
    out.width = base + n_in;
    out.row_step = out.width * out.point_step;
    // resize（而非 assign）：保留已追加的前一帧数据，新增部分零初始化
    out.data.resize(static_cast<size_t>(out.width) * out.point_step, 0);

    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> ii;
    try {
      ii = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(cloud, "intensity");
    } catch (const std::exception &) {
      ii = nullptr;  // 无 intensity 字段时填 0
    }

    const bool linear = (time_mode_ == "linear");

    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    sensor_msgs::PointCloud2Iterator<float> oi(out, "intensity");
    sensor_msgs::PointCloud2Iterator<float> ot(out, "time");
    sensor_msgs::PointCloud2Iterator<uint16_t> oring(out, "ring");
    ox += base;
    oy += base;
    oz += base;
    oi += base;
    ot += base;
    oring += base;

    const uint32_t cols = cloud.width;
    const uint32_t rows = cloud.height;
    for (uint32_t i = 0; i < n_in; ++i, ++ix, ++iy, ++iz) {
      const tf2::Vector3 p = tf * tf2::Vector3(*ix, *iy, *iz);
      *ox = static_cast<float>(p.x());
      ++ox;
      *oy = static_cast<float>(p.y());
      ++oy;
      *oz = static_cast<float>(p.z());
      ++oz;

      if (ii) {
        *oi = **ii;
        ++(*ii);
      } else {
        *oi = 0.0f;
      }
      ++oi;

      *ot = linear ? static_cast<float>(static_cast<double>(i) / static_cast<double>(n_in) * scan_period_ms_) : 0.0f;
      ++ot;

      *oring = rows > 1 ? static_cast<uint16_t>(i / cols) : 0u;  // 有组织点云：行号即 ring
      ++oring;
    }

    return true;
  }

  /// 初始化输出字段：x,y,z,intensity,time(float) + ring(uint16)
  static void initOutFields(CloudMsg &cloud) {
    cloud.fields.clear();
    cloud.point_step = 0;
    const auto add = [&cloud](const char *name, uint8_t datatype, uint32_t size) {
      sensor_msgs::msg::PointField f;
      f.name = name;
      f.offset = cloud.point_step;
      f.datatype = datatype;
      f.count = 1;
      cloud.fields.push_back(f);
      cloud.point_step += size;
    };
    add("x", sensor_msgs::msg::PointField::FLOAT32, 4);
    add("y", sensor_msgs::msg::PointField::FLOAT32, 4);
    add("z", sensor_msgs::msg::PointField::FLOAT32, 4);
    add("intensity", sensor_msgs::msg::PointField::FLOAT32, 4);
    add("time", sensor_msgs::msg::PointField::FLOAT32, 4);
    add("ring", sensor_msgs::msg::PointField::UINT16, 2);
    cloud.point_step = 4 * ((cloud.point_step + 3) / 4);  // 4 字节对齐（22 -> 24）
    cloud.width = 0;
    cloud.height = 1;
  }

  std::string target_frame_;
  std::string time_mode_;
  double scan_period_ms_{66.7};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<CloudMsg>::SharedPtr pub_;
  message_filters::Subscriber<CloudMsg> sub_front_;
  message_filters::Subscriber<CloudMsg> sub_rear_;
  std::shared_ptr<Sync> sync_;
};

}  // namespace simulated_chassis

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simulated_chassis::DualLidarMergeNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
