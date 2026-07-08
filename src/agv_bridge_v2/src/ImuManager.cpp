#include "agv_bridge_v2/ImuManager.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace agv_bridge {

ImuManager::ImuManager(rclcpp::Node* node) 
    : node_(node), 
      logger_(node->get_logger().get_child("ImuManager")) 
{}

ImuManager::~ImuManager() {
    shutdown();
}

bool ImuManager::initialize(const std::string& port, int baud_rate,
                            int window_size, double gyro_limit, double acc_limit) {
    window_size_ = window_size;
    gyro_limit_ = gyro_limit;
    acc_limit_ = acc_limit;

    if (!openSerial(port, baud_rate)) {
        RCLCPP_ERROR(logger_, "无法打开串口 %s", port.c_str());
        return false;
    }

    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    // 50Hz 读取定时器（可根据需要调整）
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&ImuManager::readAndProcess, this));

    RCLCPP_INFO(logger_, "已初始化，串口: %s, 波特率: %d (32字节帧格式)", port.c_str(), baud_rate);
    return true;
}

void ImuManager::shutdown() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    if (timer_) {
        timer_->cancel();
    }
}

bool ImuManager::openSerial(const std::string& port, int baud) {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) return false;

    struct termios options;
    tcgetattr(serial_fd_, &options);

    speed_t baud_rate;
    switch (baud) {
        case 9600:   baud_rate = B9600; break;
        case 19200:  baud_rate = B19200; break;
        case 38400:  baud_rate = B38400; break;
        case 115200: baud_rate = B115200; break;
        case 230400: baud_rate = B230400; break;
        default:     baud_rate = B115200;
    }
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;  // 无校验
    options.c_cflag &= ~CSTOPB;  // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8位数据位
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;    // 1秒超时

    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIOFLUSH);
    return true;
}

// 替换原有的 readAndProcess 函数
void ImuManager::readAndProcess() {
    if (serial_fd_ < 0) {
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000, "串口未打开");
        return;
    }

    uint8_t buffer[256];
    int n = read(serial_fd_, buffer, sizeof(buffer));
    if (n <= 0) return;

    // 调试：打印原始数据
    std::string hex_str;
    for (int i = 0; i < std::min(n, 64); ++i) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", buffer[i]);
        hex_str += buf;
    }
    RCLCPP_INFO(logger_, "RAW[%d]: %s", n, hex_str.c_str());

    rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + n);

    // 循环处理完整帧
    while (true) {
        // 查找帧头 0x68
        auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), 0x68);
        if (it == rx_buffer_.end()) {
            rx_buffer_.clear();
            break;
        }
        // 移除帧头前的无效数据
        if (it != rx_buffer_.begin()) {
            rx_buffer_.erase(rx_buffer_.begin(), it);
        }
        // 至少需要 2 字节才能读取长度字段
        if (rx_buffer_.size() < 2) break;
        
        uint8_t len = rx_buffer_[1];  // 长度字段（不含帧头，含校验和）
        uint16_t frame_len = 1 + len; // 总长度 = 帧头1字节 + len
        if (rx_buffer_.size() < frame_len) break; // 数据不足，等待下一轮
        
        // 提取完整帧
        uint8_t frame[256];
        std::copy(rx_buffer_.begin(), rx_buffer_.begin() + frame_len, frame);
        
        // 计算校验和（从长度字段到数据域末尾，不包括帧头）
        uint8_t calc_checksum = 0;
        for (int i = 1; i < frame_len - 1; ++i) { // 从索引1到倒数第2字节
            calc_checksum += frame[i];
        }
        uint8_t recv_checksum = frame[frame_len - 1];
        if (calc_checksum != recv_checksum) {
            RCLCPP_WARN(logger_, "校验和错误: 计算=0x%02X, 接收=0x%02X, 丢弃帧头", calc_checksum, recv_checksum);
            rx_buffer_.erase(rx_buffer_.begin()); // 跳过当前0x68，继续搜索
            continue;
        }
        
        // 解析帧
        uint8_t cmd = frame[3]; // 命令字
        if (cmd == 0x84) {
            // 根据实际输出格式选择解析函数
            // 假设为9轴+时间戳格式（长度 len=31 对应帧长32）
            double roll, pitch, yaw;
            double acc_x, acc_y, acc_z;
            double gyro_x, gyro_y, gyro_z;
            uint32_t timestamp;
            if (parseFrame9Axis(frame, roll, pitch, yaw,
                        acc_x, acc_y, acc_z,
                        gyro_x, gyro_y, gyro_z)) {
                // 单位转换：手册中角速度单位为 °/s，加速度单位为 g
                double gyro_x_radps = gyro_x * M_PI / 180.0;
                double gyro_y_radps = gyro_y * M_PI / 180.0;
                double gyro_z_radps = gyro_z * M_PI / 180.0;
                double acc_x_ms2 = acc_x * 9.81;
                double acc_y_ms2 = acc_y * 9.81;
                double acc_z_ms2 = acc_z * 9.81;
                double roll_rad = roll * M_PI / 180.0;
                double pitch_rad = pitch * M_PI / 180.0;
                double yaw_rad = yaw * M_PI / 180.0;
                
                RCLCPP_INFO(logger_,
                    "IMU9: R=%.2f P=%.2f Y=%.2f | Gx=%.2f Gy=%.2f Gz=%.2f | Ax=%.3f Ay=%.3f Az=%.3f ",
                    roll, pitch, yaw, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
                
                publishImu(roll_rad, pitch_rad, yaw_rad,
                           gyro_x_radps, gyro_y_radps, gyro_z_radps,
                           acc_x_ms2, acc_y_ms2, acc_z_ms2);
            } else {
                RCLCPP_WARN(logger_, "9轴数据解析失败");
            }
        } else {
            RCLCPP_WARN(logger_, "未知命令字: 0x%02X", cmd);
        }
        
        // 移除已处理帧
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
    }
}

// 解析9轴数据（28字节数据域，但只有前27字节有效，最后一字节忽略）
bool ImuManager::parseFrame9Axis(const uint8_t* frame,
                                 double& roll, double& pitch, double& yaw,
                                 double& acc_x, double& acc_y, double& acc_z,
                                 double& gyro_x, double& gyro_y, double& gyro_z) {
    // frame 结构：68 len addr 84 [27字节数据] 1字节校验
    // 数据域共28字节，但有效数据为前27字节，最后一字节（索引31）为校验和
    const uint8_t* data = frame + 4;   // 跳过 68 len addr cmd
    
    // 顺序：横滚、俯仰、航向、X加速度、Y加速度、Z加速度、X角速度、Y角速度、Z角速度
    roll   = decodeBcdAngle(data + 0);
    pitch  = decodeBcdAngle(data + 3);
    yaw    = decodeBcdAngle(data + 6);
    acc_x  = decodeBcdAcc(data + 9);
    acc_y  = decodeBcdAcc(data + 12);
    acc_z  = decodeBcdAcc(data + 15);
    gyro_x = decodeBcdAngle(data + 18);
    gyro_y = decodeBcdAngle(data + 21);
    gyro_z = decodeBcdAngle(data + 24);
    // 注意：第28字节（data+27）被忽略，符合旧代码行为
    
    return true;
}

// 角度/角速度解码（2位小数）
double ImuManager::decodeBcdAngle(const uint8_t* data) {
    bool negative = ((data[0] >> 4) & 0x0F) == 1;  // 高4位为1表示负
    uint8_t hundreds = data[0] & 0x0F;            // 百位（0-9）
    uint8_t tens = (data[1] >> 4) & 0x0F;         // 十位
    uint8_t ones = data[1] & 0x0F;                // 个位
    uint8_t tenth = (data[2] >> 4) & 0x0F;        // 十分位
    uint8_t hundredth = data[2] & 0x0F;           // 百分位
    double value = (hundreds * 100.0 + tens * 10.0 + ones) +
                   (tenth * 0.1 + hundredth * 0.01);
    return negative ? -value : value;
}

// 加速度解码（3位小数）
double ImuManager::decodeBcdAcc(const uint8_t* data) {
    bool negative = ((data[0] >> 4) & 0x0F) == 1;  // 高4位为1表示负
    uint8_t tens = data[0] & 0x0F;                // 整数十位
    uint8_t ones = (data[1] >> 4) & 0x0F;         // 整数个位
    uint8_t tenth = data[1] & 0x0F;               // 十分位
    uint8_t hundredth = (data[2] >> 4) & 0x0F;    // 百分位
    uint8_t thousandth = data[2] & 0x0F;          // 千分位
    double value = (tens * 10.0 + ones) +
                   (tenth * 0.1 + hundredth * 0.01 + thousandth * 0.001);
    return negative ? -value : value;
}

void ImuManager::publishImu(double roll_rad, double pitch_rad, double yaw_rad,
                            double gyro_x_radps, double gyro_y_radps, double gyro_z_radps,
                            double acc_x_ms2, double acc_y_ms2, double acc_z_ms2) {
    RCLCPP_INFO(logger_, "publishImu 被调用");  // 添加调试日志
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = node_->now();
    RCLCPP_INFO(logger_, "stamp: %u.%u", 
                msg.header.stamp.sec, msg.header.stamp.nanosec);  // 添加调试日志
    msg.header.frame_id = "imu_link";

    // 将欧拉角转换为四元数（使用 tf2）
    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_rad);
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    // 姿态协方差（可根据实际调整）
    msg.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

    // 角速度
    msg.angular_velocity.x = gyro_x_radps;
    msg.angular_velocity.y = gyro_y_radps;
    msg.angular_velocity.z = gyro_z_radps;
    msg.angular_velocity_covariance = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};

    // 线加速度
    msg.linear_acceleration.x = acc_x_ms2;
    msg.linear_acceleration.y = acc_y_ms2;
    msg.linear_acceleration.z = acc_z_ms2;
    msg.linear_acceleration_covariance = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};

    RCLCPP_INFO(logger_, "即将调用 publish");
    imu_pub_->publish(msg);
    RCLCPP_INFO(logger_, "publish 调用完成");
}

} // namespace agv_bridge