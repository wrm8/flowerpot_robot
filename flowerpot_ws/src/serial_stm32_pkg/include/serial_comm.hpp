#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>  // 改为 libserial
#include <common_msgs_pkg/msg/serial_data.hpp>
#include <common_msgs_pkg/msg/encoder.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <memory>
#include "std_msgs/msg/bool.hpp"
#include <thread>  // 必须加这个！创建子线程

#define FRAME_SIZE 5  
#define HEADER_BYTE 0x55

#define ONLY_ENCODER 1

#define RECORD_GPS 0x04   // STM32发送的标志位，表示是否记录GPS信息
#define IF_GRASPED 0x02   // STM32发送的标志位，表示完成抓取
#define IF_RELEASED 0x01  // STM32发送的标志位，表示完成释放

using namespace std::chrono_literals;

class SerialCommNode : public rclcpp::Node {
public:
    SerialCommNode();
    ~SerialCommNode();
    
    // 接收stm32端传来的编码器数据
    void ReceiveData();

    // 更新里程计信息
    void updateOdometry();

private:
    void serialDataCallback(const common_msgs_pkg::msg::SerialData::SharedPtr msg);
    bool setupSerialPort();
    // 累加和校验
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    
    rclcpp::Subscription<common_msgs_pkg::msg::SerialData>::SharedPtr serial_data_sub_;   // 订阅serial_data话题
    // 2. 三个标志位发布器（成员变量）
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gps_flag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_flag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr release_flag_pub_;
    #ifdef ONLY_ENCODER//必须要变量声明
        rclcpp::Publisher<common_msgs_pkg::msg::Encoder>::SharedPtr encoder_pub_; // 发布编码器数据
        common_msgs_pkg::msg::Encoder encoder_msg_; // 编码器数据
    #else
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;    // 发布标准里程计
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;     // ROS2 TF广播器
        nav_msgs::msg::Odometry odom_;
    #endif

    LibSerial::SerialPort serial_; // 改为 LibSerial::SerialPort
    std::string port_;
    int baud_rate_; 
    double left_speed_ = 0.0;      // 左轮线速度(m/s)
    double right_speed_ = 0.0;     // 右轮线速度(m/s)
    double x_ = 0.0, y_ = 0.0;     // 位置(m)
    double theta_ = 0.0;           // 方向(rad)
    double wheel_separation_ = 0.355;      // 轮间距(m)
    double wheel_radius_ = 0.09;          // 轮半径(m)
    rclcpp::Time last_time_ = rclcpp::Time(0);

    bool _clear_encoder = false; // 是否清除编码器数据

    static int8_t is_recorded_; // 是否完成一次记录
};

#endif // SERIAL_COMM_HPP