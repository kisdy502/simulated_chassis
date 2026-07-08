#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>   // 添加这一行
#include <string>
#include <deque>
#include <vector>

namespace agv_bridge {

class ImuManager {
public:
    ImuManager(rclcpp::Node* node);
    ~ImuManager();

    bool initialize(const std::string& port, int baud_rate,
                    int window_size = 5, double gyro_limit = 300.0, double acc_limit = 4.0);
    void shutdown();

private:
    bool syncFrame();  // 新的帧同步方法
    // 串口操作
    bool openSerial(const std::string& port, int baud);
    void readAndProcess();
    void publishImu(double roll_rad, double pitch_rad, double yaw_rad,
                    double gyro_x_radps, double gyro_y_radps, double gyro_z_radps,
                    double acc_x_ms2, double acc_y_ms2, double acc_z_ms2);
    bool parseFrame9Axis(const uint8_t* frame, 
                         double& roll, double& pitch, double& yaw,
                         double& acc_x, double& acc_y, double& acc_z,
                         double& gyro_x, double& gyro_y, double& gyro_z);
    double decodeBcdAcc(const uint8_t* data);
    double decodeBcdAngle(const uint8_t* data);
    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int serial_fd_ = -1;
    std::vector<uint8_t> rx_buffer_;
    std::deque<double> gyro_buffer_;   // 仅用于 Z 轴角速率平滑（可选）
    std::deque<double> acc_buffer_;    // 仅用于 Y 轴加速度平滑（可选）
    int window_size_;
    double gyro_limit_;
    double acc_limit_;

    
};

} // namespace agv_bridge

#endif