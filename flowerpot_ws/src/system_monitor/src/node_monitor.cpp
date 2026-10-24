#include "node_monitor.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <vector>

// 需要用 ros2 的命令行工具检查节点
#include <cstdio>
#include <memory>
#include <string>
#include <array>

NodeMonitor::NodeMonitor() : Node("node_monitor"), all_ready_(false)
{
    // 需要监控的节点列表（ROS2 中的节点名）
    required_nodes_.push_back("camera_node");           // 发布相机话题
    required_nodes_.push_back("imu_node");              // 发布IMU数据（包括偏航角）
    required_nodes_.push_back("radar_node");           // 发布雷达数据
    required_nodes_.push_back("lslidar_driver_node"); 
    required_nodes_.push_back("yolo11_node");             // 发布YOLOv11检测结果（花盆坐标）
    required_nodes_.push_back("gps_node");              // 发布GPS数据（经纬度）
    required_nodes_.push_back("serial_stm32_node");     // 发布与STM32通信的信息
    required_nodes_.push_back("decision_making_node");  // 发布决策结果
    required_nodes_.push_back("potbot_hmi_node");       // 发布HMI界面
    required_nodes_.push_back("radaryolo11_meld_node");      // 发布多传感器处理结果

    // 创建发布者
    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/system_ready", 10);
    
    // 创建定时器，每2秒检查一次
    check_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&NodeMonitor::checkNodes, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Node monitor started, monitoring %zu nodes", 
                required_nodes_.size());
}

NodeMonitor::~NodeMonitor()
{
}

bool NodeMonitor::isNodeRunning(const std::string& node_name)
{
    // 使用 ros2 node list 命令获取节点列表
    std::string cmd = "ros2 node list | grep -E \"(/" + node_name + "$|/" + node_name + " )\"";
    
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    
    if (!pipe) {
        return false;
    }
    
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    
    // 如果找到节点名，返回 true
    return !result.empty();
}

void NodeMonitor::checkNodes()
{
    bool all_ready = true;
    std::vector<std::string> missing_nodes;
    
    // 检查每个节点是否运行
    for (const auto& node : required_nodes_) {
        if (!isNodeRunning(node)) {
            all_ready = false;
            missing_nodes.push_back(node);
        }
    }
    
    // 状态变化时打印信息
    if (all_ready && !all_ready_) {
        RCLCPP_INFO(this->get_logger(), "All nodes are ready!");
    } else if (!all_ready && all_ready_) {
        RCLCPP_WARN(this->get_logger(), "System disconnected, some nodes are offline");
    } else if (!all_ready) {
        // 每10秒打印一次等待信息
        static int count = 0;
        if (++count >= 5) {
            count = 0;
            RCLCPP_INFO(this->get_logger(), "Waiting for nodes: %d/%zu", 
                        (int)(required_nodes_.size() - missing_nodes.size()),
                        required_nodes_.size());
            for (const auto& node : missing_nodes) {
                RCLCPP_ERROR(this->get_logger(), "missing_node: %s", node.c_str());
            }
        }
    }
    
    all_ready_ = all_ready;
    
    // 发布状态
    auto msg = std_msgs::msg::Bool();
    msg.data = all_ready_;
    ready_pub_->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}