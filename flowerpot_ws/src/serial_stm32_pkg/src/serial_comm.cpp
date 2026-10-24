#include "serial_comm.hpp"
#include <iostream>
#include <iomanip>
#include <memory>

int8_t SerialCommNode::is_recorded_ = 0x00; // 初始化为0x00，表示默认未完成一次记录

// ros2 run serial_stm32_pkg serial_stm32_node --ros-args -p port:=/dev/ttyTHS0 -p baud_rate:=115200
SerialCommNode::SerialCommNode()
: rclcpp::Node("serial_stm32_node")  // ROS2 节点初始化
{
    // 从参数服务器获取串口配置参数
    this->declare_parameter<std::string>("port", "/dev/ttyTHS1");
    this->declare_parameter<int>("baud_rate", 9600);
    
    this->get_parameter("port", port_);
    this->get_parameter("baud_rate", baud_rate_);
    
    // 订阅serial_data话题，用于接收要发送给STM32的数据
    serial_data_sub_ = this->create_subscription<common_msgs_pkg::msg::SerialData>(
        "serial_data", 10, std::bind(&SerialCommNode::serialDataCallback, this, std::placeholders::_1));
    
    // 三个标志位发布器
    gps_flag_pub_     = this->create_publisher<std_msgs::msg::Bool>("/robot/gps_flag", 10);
    grasp_flag_pub_   = this->create_publisher<std_msgs::msg::Bool>("/robot/grasp_flag", 10);
    release_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/release_flag", 10);
    
    // 发布标准里程计
    #ifdef ONLY_ENCODER
        encoder_pub_ = this->create_publisher<common_msgs_pkg::msg::Encoder>("encoder_msg", 50);
    #else
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 50);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);  // ROS2 TF广播器
    #endif
    
    // 初始化串口
    if (!setupSerialPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port!");
        rclcpp::shutdown();
    }
    
    last_time_ = this->now();  // ROS2: 获取当前时间
}

SerialCommNode::~SerialCommNode() {
    if (serial_.IsOpen()) {  // 改为 IsOpen()
        serial_.Close();      // 改为 Close()
    }
}

bool SerialCommNode::setupSerialPort() {
    try {
        // 设置波特率
        LibSerial::BaudRate baud_rate_enum;
        switch(baud_rate_) {
            case 9600: baud_rate_enum = LibSerial::BaudRate::BAUD_9600; break;
            case 115200: baud_rate_enum = LibSerial::BaudRate::BAUD_115200; break;
            default: baud_rate_enum = LibSerial::BaudRate::BAUD_115200; break;
        }
        
        serial_.Open(port_);                    // 打开串口
        serial_.SetBaudRate(baud_rate_enum);    // 设置波特率
        serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        if (!serial_.IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port %s could not be opened.", port_.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully at %d baud.", 
                port_.c_str(), baud_rate_);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial port exception: %s", e.what());
        return false;
    }
}

// 累加和校验
uint8_t SerialCommNode::calculateChecksum(const std::vector<uint8_t>& data) {
    uint8_t sum = 0;
    for (size_t i = 0; i < data.size() - 1; ++i) { // 最后一个字节是校验位，不参与计算
        sum += data[i];
    }
    return sum;
}

void SerialCommNode::serialDataCallback(const common_msgs_pkg::msg::SerialData::SharedPtr msg) {
    if (!serial_.IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial port is not open. Cannot send data.");
        return;
    }
    

    try {

        RCLCPP_INFO(this->get_logger(), "【原始PWM】left = %d, right = %d", 
                msg->pwm_left, msg->pwm_right);

        // 发送数据到STM32
        std::vector<uint8_t> buffer(5);//之前是uint8_t导致一直出错，没有负数，跳变255，改成除以8的int16_t就好了
        // 显式转一下，消除编译器警告，逻辑不变
        buffer[0] = HEADER_BYTE;// 帧头
        buffer[1] = static_cast<uint8_t>(msg->pwm_left);
        buffer[2] = static_cast<uint8_t>(msg->pwm_right);
        buffer[3] = static_cast<uint8_t>(msg->command);
        buffer[4] = calculateChecksum(buffer);// 填充最后一个字节，确保数据长度正确

        serial_.Write(buffer);  // 发送
        // for(size_t i = 0; i < buffer.size(); ++i){
        //     RCLCPP_INFO(this->get_logger(), "buffer:%zu,%d", i, buffer[i]);
        // }
        // RCLCPP_INFO(this->get_logger(), "Sent %zu bytes to STM32", buffer.size());
    } catch (const std::exception& e) {
        RCLCPP_INFO(this->get_logger(), "Error writing to serial port: %s", e.what());
    }
}

// 更新里程计信息,编码器发布的应该是速度信息
void SerialCommNode::updateOdometry() {
    #ifdef ONLY_ENCODER
        encoder_msg_.right_speed = right_speed_;
        encoder_msg_.left_speed = left_speed_;
        encoder_pub_->publish(encoder_msg_);//括号里发的是内容，发的话题已经在上边绑定了
    #else
        // 计算时间间隔
        rclcpp::Time current_time = this->now();  // ROS2: 获取当前时间
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 计算线速度和角速度（差分驱动模型）
        double linear_vel = (right_speed_ + left_speed_) / 2.0;
        double angular_vel = (right_speed_ - left_speed_) / wheel_separation_;

        // 更新位置和方向（航迹推算）
        theta_ += angular_vel * dt;
        x_ += linear_vel * cos(theta_) * dt;
        y_ += linear_vel * sin(theta_) * dt;

        // 填充Odometry消息
        odom_.header.stamp = current_time;  // ROS2: 直接使用 rclcpp::Time
        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";

        // 位置
        odom_.pose.pose.position.x = x_;
        odom_.pose.pose.position.y = y_;
        odom_.pose.pose.position.z = 0.0;
        
        // 使用tf2设置四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_.pose.pose.orientation.x = q.x();
        odom_.pose.pose.orientation.y = q.y();
        odom_.pose.pose.orientation.z = q.z();
        odom_.pose.pose.orientation.w = q.w();

        // 速度
        odom_.twist.twist.linear.x = linear_vel;
        odom_.twist.twist.angular.z = angular_vel;

        // 发布里程计
        odom_pub_->publish(odom_);
        
        // ROS2 TF发布
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf_msg);
    #endif
}

// //试验串口信息代码
// void SerialCommNode::ReceiveData() {
//     rclcpp::Rate rate(100); // 100Hz

//     while (rclcpp::ok()) {
//         try {
//             // 只读 1 个字节，直接打印！
//             uint8_t byte;
//             serial_.ReadByte(byte);

//             // 【纯裸打印】不解析、不校验、不判断
//             RCLCPP_INFO(this->get_logger(), "RAW BYTE: 0x%02X (%d)", byte, byte);

//         } catch (const std::exception& e) {
//             rate.sleep();
//         }
//         rate.sleep();
//     }
// }

// // 试验帧同步和校验的代码
// void SerialCommNode::ReceiveData() {
//     rclcpp::Rate rate(100); // 100Hz
//     std::vector<uint8_t> buffer;

//     while (rclcpp::ok()) {
//         try {
//             uint8_t byte;
//             serial_.ReadByte(byte);  // 读1字节
//             buffer.push_back(byte);

//             // ==============================================
//             // 【正确的帧同步】滑动窗口找帧头
//             // ==============================================
//             while (buffer.size() > 0 && buffer[0] != HEADER_BYTE) {
//                 buffer.erase(buffer.begin()); // 删掉不是帧头的字节
//             }

//             // 不够一帧，继续读
//             if (buffer.size() < FRAME_SIZE) {
//                 rate.sleep();
//                 continue;
//             }

//             // ==============================================
//             // 够一帧了 → 校验
//             // ==============================================
//             uint8_t checksum = 0;
//             for (int i = 0; i < FRAME_SIZE - 1; i++) {
//                 checksum += buffer[i];
//             }

//             if (checksum == buffer[FRAME_SIZE - 1]) {
//                 // 解析正确数据
//                 left_speed_ = static_cast<int8_t>(buffer[1]);
//                 right_speed_ = static_cast<int8_t>(buffer[2]);

//                 RCLCPP_INFO(this->get_logger(), "编码器数据: 左=%d, 右=%d", left_speed_, right_speed_);
//                 updateOdometry();

//                 if(buffer[3] & RECORD_GPS) {
//                     RCLCPP_INFO(this->get_logger(), "Get gps_flag!");
//                     this->set_parameter(rclcpp::Parameter("/robot/gps_flag", true));
//                 }
//                 if(buffer[3] & IF_GRASPED) {
//                     RCLCPP_INFO(this->get_logger(), "Get grasped_status!");
//                     this->set_parameter(rclcpp::Parameter("/robot/grasped", true));
//                 }
//                 if(buffer[3] & IF_RELEASED) {
//                     RCLCPP_INFO(this->get_logger(), "Get released_status!");
//                     this->set_parameter(rclcpp::Parameter("/robot/released", true));
//                 }

//                 // 正确解析 → 清空整个帧
//                 buffer.clear();
//             } else {
//                 // 校验错 → 丢掉帧头，重新同步
//                 RCLCPP_WARN(this->get_logger(), "Checksum error");
//                 buffer.erase(buffer.begin());
//             }

//         } catch (const std::exception& e) {
//             // RCLCPP_ERROR_STREAM(this->get_logger(), "Serial read error: " << e.what());
//             buffer.clear();
//             rate.sleep();
//         }

//         rate.sleep();
//     }
// }

//原本代码
void SerialCommNode::ReceiveData() {
    rclcpp::Rate rate(100); // 100Hz
    std::vector<uint8_t> buffer;
    buffer.reserve(FRAME_SIZE);
    
    while (rclcpp::ok()) {
        try {
            // 逐字节读取，直到收到完整一帧
            while (buffer.size() < FRAME_SIZE && rclcpp::ok()) {
                uint8_t byte;
                serial_.ReadByte(byte);  // 逐字节读取
                buffer.push_back(byte);
                
                // 如果还没收到帧头，清空缓冲区
                if (buffer.size() == 1 && buffer[0] != HEADER_BYTE) {
                    buffer.clear();
                }
            }
            
            // 收到完整一帧
            if (buffer.size() == FRAME_SIZE) {
                // 验证校验和
                uint8_t checksum = 0;
                for (int i = 0; i < FRAME_SIZE - 1; i++) {
                    checksum += buffer[i];
                }
                
                if (checksum == buffer[FRAME_SIZE - 1]) {
                    // 解析数据
                    // ✅ 解析回来，除以 100 恢复 m/s
                    left_speed_ = (int8_t)buffer[1] / 100.0f;
                    right_speed_ = (int8_t)buffer[2] / 100.0f;
                    //RCLCPP_INFO(this->get_logger(), "速度：%.2f m/s, %.2f m/s", left_speed_, right_speed_);
                    // left_speed_ = buffer[1];   // 单位m/s
                    // right_speed_ = buffer[2];
                    // RCLCPP_INFO(this->get_logger(), "编码器数据: 左=%d, 右=%d", left_speed_, right_speed_);
                    updateOdometry();
                    
                    // GPS记录标志位
                    if(buffer[3] & RECORD_GPS) {
                        RCLCPP_INFO(this->get_logger(), "Get gps_flag!");
                        std_msgs::msg::Bool msg;
                        msg.data = true;
                        gps_flag_pub_->publish(msg);
                    }

                    // 抓取完成标志位
                    if(buffer[3] & IF_GRASPED) {
                        RCLCPP_INFO(this->get_logger(), "Get grasped_status!");
                        std_msgs::msg::Bool msg;
                        msg.data = true;
                        grasp_flag_pub_->publish(msg);
                    }

                    // 释放完成标志位
                    if(buffer[3] & IF_RELEASED) {
                        RCLCPP_INFO(this->get_logger(), "Get released_status!");
                        std_msgs::msg::Bool msg;
                        msg.data = true;
                        release_flag_pub_->publish(msg);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Checksum error");
                }
                buffer.clear();
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Serial read error: " << e.what());
            buffer.clear();
            rate.sleep();
        }
        
        //rclcpp::spin_some(this->shared_from_this());
        rate.sleep();

        // rclcpp::spin(node);
        // rclcpp::shutdown();
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommNode>();
    //node->ReceiveData();//死循环，不断接收数据并处理，会卡死其他回调，ros2默认单线程
    // 线程1：读串口
    std::thread receive_thread(&SerialCommNode::ReceiveData, node);
    // 线程2：spin 处理所有回调（发送数据）
    rclcpp::spin(node);

    rclcpp::shutdown();
    receive_thread.join();
    return 0;
}