/**
 * PointCloud2 本体过滤节点
 * 把每个点变换到 base_link，落在机器人本体盒子内的点置为 NaN（丢弃），
 * 避免 360° 雷达把机器人本体（含雷达自身基座、轮子等）当成障碍物。
 *
 * 参数:
 *   input_topic   输入 PointCloud2 (默认 /points2_1_raw)
 *   output_topic  输出 PointCloud2 (默认 /points2_1)
 *   base_frame    判断本体盒子的参考系 (默认 base_link)
 *   box_half_x    本体半长 (默认 0.30)
 *   box_half_y    本体半宽 (默认 0.20)
 *   box_min_z     本体底面 z (默认 -0.06)
 *   box_max_z     本体顶面 z (默认 0.06)
 *   margin        外扩余量，防止抖动/噪声 (默认 0.03)
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PointcloudFootprintFilter : public rclcpp::Node {
public:
  PointcloudFootprintFilter() : Node("pointcloud_footprint_filter") {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/points2_1_raw");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/points2_1");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    half_x_ = this->declare_parameter<double>("box_half_x", 0.30);
    half_y_ = this->declare_parameter<double>("box_half_y", 0.20);
    min_z_ = this->declare_parameter<double>("box_min_z", -0.06);
    max_z_ = this->declare_parameter<double>("box_max_z", 0.06);
    margin_ = this->declare_parameter<double>("margin", 0.03);

    eff_x_ = half_x_ + margin_;
    eff_y_ = half_y_ + margin_;
    eff_min_z_ = min_z_ - margin_;
    eff_max_z_ = max_z_ + margin_;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::SensorDataQoS qos;
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        std::bind(&PointcloudFootprintFilter::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "PointCloud2 footprint filter: %s -> %s (base=%s, box=%.3f x %.3f, z[%.3f,%.3f], margin=%.3f)",
                input_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str(),
                eff_x_, eff_y_, eff_min_z_, eff_max_z_, margin_);
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 点云坐标系 -> base_frame 的变换（静态，取最新，规避时间戳问题）
    geometry_msgs::msg::TransformStamped tfm;
    try {
      tfm = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id,
                                        tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "等待 TF %s -> %s: %s", msg->header.frame_id.c_str(),
                           base_frame_.c_str(), e.what());
      return;
    }

    // 定位 x/y/z 字段在点步内的字节偏移
    int xo = -1, yo = -1, zo = -1;
    for (const auto &f : msg->fields) {
      if (f.name == "x") xo = static_cast<int>(f.offset);
      else if (f.name == "y") yo = static_cast<int>(f.offset);
      else if (f.name == "z") zo = static_cast<int>(f.offset);
    }
    if (xo < 0 || yo < 0 || zo < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "点云缺少 x/y/z 字段，跳过过滤");
      return;
    }

    // base <- cloud 的旋转矩阵 + 平移
    const double tx = tfm.transform.translation.x;
    const double ty = tfm.transform.translation.y;
    const double tz = tfm.transform.translation.z;
    tf2::Quaternion q(tfm.transform.rotation.x, tfm.transform.rotation.y,
                      tfm.transform.rotation.z, tfm.transform.rotation.w);
    tf2::Matrix3x3 R(q);
    const double r00 = R[0][0], r01 = R[0][1], r02 = R[0][2];
    const double r10 = R[1][0], r11 = R[1][1], r12 = R[1][2];
    const double r20 = R[2][0], r21 = R[2][1], r22 = R[2][2];

    auto out = *msg;  // 深拷贝（含 data 缓冲区），就地改写
    const uint32_t step = msg->point_step;
    const size_t npts = static_cast<size_t>(msg->width) * msg->height;
    const float fnan = std::numeric_limits<float>::quiet_NaN();
    unsigned long removed = 0;

    for (size_t i = 0; i < npts; ++i) {
      uint8_t *p = out.data.data() + i * step;
      float x, y, z;
      std::memcpy(&x, p + xo, sizeof(float));
      std::memcpy(&y, p + yo, sizeof(float));
      std::memcpy(&z, p + zo, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;  // 本就是无效点
      }

      // 变换到 base_frame
      const double bx = r00 * x + r01 * y + r02 * z + tx;
      const double by = r10 * x + r11 * y + r12 * z + ty;
      const double bz = r20 * x + r21 * y + r22 * z + tz;

      // 落在本体盒子内 -> 置 NaN
      if (std::fabs(bx) <= eff_x_ && std::fabs(by) <= eff_y_ &&
          bz >= eff_min_z_ && bz <= eff_max_z_) {
        std::memcpy(p + xo, &fnan, sizeof(float));
        std::memcpy(p + yo, &fnan, sizeof(float));
        std::memcpy(p + zo, &fnan, sizeof(float));
        ++removed;
      }
    }

    pub_->publish(out);
    (void)removed;
  }

  std::string input_topic_, output_topic_, base_frame_;
  double half_x_, half_y_, min_z_, max_z_, margin_;
  double eff_x_, eff_y_, eff_min_z_, eff_max_z_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudFootprintFilter>());
  rclcpp::shutdown();
  return 0;
}
