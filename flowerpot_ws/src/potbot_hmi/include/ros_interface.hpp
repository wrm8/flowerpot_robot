#ifndef ROS_INTERFACE_HPP
#define ROS_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <decision_making_pkg/srv/start_transport.hpp>

class Ros2Interface : public rclcpp::Node
{
public:
    Ros2Interface();
    ~Ros2Interface();

    double latitude;
    double longitude;
    float yaw;
    bool system_ready_;
    int transported_pot_count; // 已搬运花盆数量
    
public:
    // 发送运输任务配置（摆放方式、摆放间距、开始标志）
    bool sendTransportTask(int placement_type, float placement_spacing, bool start);
    // 检查Service是否可用
    bool checkService();
    
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;   // 所有节点是否准备就绪订阅者
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr transported_pot_count_sub;
    
    // Service Client
    rclcpp::Client<decision_making_pkg::srv::StartTransport>::SharedPtr client_;
    
    bool service_ready_;     // Service是否可用
    
private:
    void gpsCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    void yawCallback(const std_msgs::msg::Float32::ConstSharedPtr msg);
    void transportedPotCountCallback(const std_msgs::msg::Int8::ConstSharedPtr msg);
    void readyCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
};

#endif