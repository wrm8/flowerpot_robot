#ifndef NODE_MONITOR_H
#define NODE_MONITOR_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

class NodeMonitor : public rclcpp::Node {
public:
    NodeMonitor();
    ~NodeMonitor();

private:
    void checkNodes();
    bool isNodeRunning(const std::string& node_name);
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    
    std::vector<std::string> required_nodes_;
    bool all_ready_;
};

#endif