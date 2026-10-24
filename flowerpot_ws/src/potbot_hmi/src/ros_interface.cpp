#include "ros_interface.hpp"

Ros2Interface::Ros2Interface() : Node("ros2_interface"), service_ready_(false)
{
    latitude = 0;
    longitude = 0;
    yaw = 0;
    transported_pot_count = 0;
    system_ready_ = false;

    // // 订阅偏航角（从IMU获取）
    // yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    //     "/yaw_angle",  10,
    //     std::bind(&DecisionMaking::yawCallback, this, std::placeholders::_1));
    
    // 订阅GPS和Yaw数据
    gps_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "gps_data", 10, std::bind(&Ros2Interface::gpsCallback, this, std::placeholders::_1));
    yaw_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/yaw_angle", 10, std::bind(&Ros2Interface::yawCallback, this, std::placeholders::_1));
    transported_pot_count_sub = this->create_subscription<std_msgs::msg::Int8>(
        "/transported_pot_count", 10, std::bind(&Ros2Interface::transportedPotCountCallback, this, std::placeholders::_1));
    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/system_ready", 10, std::bind(&Ros2Interface::readyCallback, this, std::placeholders::_1));
    
    // 创建Service客户端
    client_ = this->create_client<decision_making_pkg::srv::StartTransport>("/start_transport");
    
    // 等待服务可用
    if(!client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service /start_transport 不可用，请确保决策节点已启动");
        service_ready_ = false;
    }
    else
    {
        // 检查Service是否可用
        checkService();
    }
}

Ros2Interface::~Ros2Interface()
{
    RCLCPP_INFO(this->get_logger(), "Ros2Interface destroyed");
}

bool Ros2Interface::checkService() {
    // 检查服务是否可用
    if (!client_->service_is_ready()) {
        service_ready_ = false;
        RCLCPP_WARN(this->get_logger(), "Service /start_transport 不可用，请确保决策节点已启动");
        return false;
    }
    
    service_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Service /start_transport 已连接");
    return service_ready_;
}

bool Ros2Interface::sendTransportTask(int placement_type, float placement_spacing, bool start) {
    // 如果Service不可用，重新检查一次
    if (!service_ready_) {
        RCLCPP_WARN(this->get_logger(), "Service未就绪，尝试重新连接...");
        checkService();
        if (!service_ready_) {
            RCLCPP_ERROR(this->get_logger(), "Service不可用，无法发送指令");
            return false;
        }
    }
    
    // 创建请求
    auto request = std::make_shared<decision_making_pkg::srv::StartTransport::Request>();
    request->placement_type = placement_type;
    request->placement_spacing = placement_spacing;
    request->start = start;
    
    // 打印发送的信息
    RCLCPP_INFO(this->get_logger(), "发送运输任务: placement_type=%d, placement_spacing=%.2f, start=%s", 
             placement_type, placement_spacing, start ? "true" : "false");
    
    // 发送异步请求
    auto result = client_->async_send_request(request);
    
    // 等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "指令发送成功: %s", response->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "指令发送失败: %s", response->message.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service调用失败，请检查决策节点是否运行");
        service_ready_ = false;  // 标记为不可用，下次重新检查
        return false;
    }
}

void Ros2Interface::gpsCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    latitude = msg->data[0];
    longitude = msg->data[1];
}

void Ros2Interface::yawCallback(const std_msgs::msg::Float32::ConstSharedPtr msg)
{
    yaw = msg->data;
}

void Ros2Interface::transportedPotCountCallback(const std_msgs::msg::Int8::ConstSharedPtr msg)
{
    transported_pot_count = msg->data;
    RCLCPP_INFO(this->get_logger(), "已搬运花盆数量更新: %d", transported_pot_count);
}

void Ros2Interface::readyCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
    system_ready_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "系统准备状态更新: %s", system_ready_ ? "已就绪" : "未就绪");
}