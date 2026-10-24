// gps_simple_ros2.cpp
// 功能：单线程读取20Hz GPS数据，卡尔曼滤波，通过ROS话题发布经纬度

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

// ==================== 卡尔曼滤波器 ====================
class KalmanFilter {
private:
    double Q;  // 过程噪声
    double R;  // 测量噪声
    double P;  // 估计误差协方差
    double K;  // 卡尔曼增益
    double X;  // 状态估计值
    bool initialized;
    
public:
    KalmanFilter() : Q(0.01), R(0.1), P(1.0), K(0), X(0), initialized(false) {}
    
    double update(double measurement) {
        if (!initialized) {
            X = measurement;
            initialized = true;
            return X;
        }
        
        // 预测
        double X_pred = X;
        double P_pred = P + Q;
        
        // 更新
        K = P_pred / (P_pred + R);
        X = X_pred + K * (measurement - X_pred);
        P = (1 - K) * P_pred;
        
        return X;
    }
    
    void reset() {
        initialized = false;
        P = 1.0;
    }
};

// ==================== GPS解析类 ====================
class GPSParser {
private:
    KalmanFilter latFilter;
    KalmanFilter lonFilter;
    KalmanFilter altFilter;
    
    double convertLatitude(const std::string& lat, const std::string& dir) {
        if (lat.length() < 4) return 0.0;
        double degrees = std::stod(lat.substr(0, 2));
        double minutes = std::stod(lat.substr(2));
        double decimal = degrees + minutes / 60.0;
        return (dir == "S") ? -decimal : decimal;
    }
    
    double convertLongitude(const std::string& lon, const std::string& dir) {
        if (lon.length() < 5) return 0.0;
        double degrees = std::stod(lon.substr(0, 3));
        double minutes = std::stod(lon.substr(3));
        double decimal = degrees + minutes / 60.0;
        return (dir == "W") ? -decimal : decimal;
    }
    
public:
    struct GPSData {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        int satellites = 0;
        int quality = 0;
        double hdop = 0.0;
        bool isValid = false;
    };
    
    GPSData parseGGA(const std::string& sentence) {
        GPSData data;
        if (sentence.find("$GNGGA") != 0 && sentence.find("$GPGGA") != 0) {
            return data;
        }
        
        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string field;
        
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }
        
        if (fields.size() < 15) return data;
        
        try {
            if (!fields[2].empty() && !fields[3].empty()) {
                data.latitude = convertLatitude(fields[2], fields[3]);
                data.latitude = latFilter.update(data.latitude);
            }
            
            if (!fields[4].empty() && !fields[5].empty()) {
                data.longitude = convertLongitude(fields[4], fields[5]);
                data.longitude = lonFilter.update(data.longitude);
            }
            
            if (!fields[6].empty()) data.quality = std::stoi(fields[6]);
            if (!fields[7].empty()) data.satellites = std::stoi(fields[7]);
            if (!fields[8].empty()) data.hdop = std::stod(fields[8]);
            if (!fields[9].empty()) data.altitude = std::stod(fields[9]);
            data.altitude = altFilter.update(data.altitude);
            
            data.isValid = (data.latitude != 0.0 && data.longitude != 0.0);
            
        } catch (...) {}
        
        return data;
    }
};

// ==================== GPS节点类 ====================
class GPSNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gps_pub_;
    std::string port_name_;
    int baud_rate_;
    std::unique_ptr<boost::asio::io_service> io_;
    std::unique_ptr<boost::asio::serial_port> port_;
    GPSParser parser_;
    
public:
    GPSNode() : Node("gps_node") {
        // 声明参数
        this->declare_parameter<std::string>("port", "/dev/ttyACM3");
        this->declare_parameter<int>("baud_rate", 115200);
        
        // 获取参数
        this->get_parameter("port", port_name_);
        this->get_parameter("baud_rate", baud_rate_);
        
        // 创建发布者
        gps_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gps_data", 10);
        
        // RCLCPP_INFO_STREAM(this->get_logger(), "使用串口: " << port_name_);
        // RCLCPP_INFO_STREAM(this->get_logger(), "波特率: " << baud_rate_);
        // RCLCPP_INFO_STREAM(this->get_logger(), "启动GPS读取...");
        
        // 初始化串口并启动读取线程
        initSerial();
    }
    
    void initSerial() {
        try {
            io_ = std::make_unique<boost::asio::io_service>();
            port_ = std::make_unique<boost::asio::serial_port>(*io_, port_name_);
            
            port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
            port_->set_option(boost::asio::serial_port_base::character_size(8));
            port_->set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            port_->set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            
            // 设置 20Hz 频率
            std::string freq_cmd = "GNGGA 0.05\r\n";
            // RCLCPP_INFO_STREAM(this->get_logger(), "发送频率设置命令: " << freq_cmd);
            boost::asio::write(*port_, boost::asio::buffer(freq_cmd));
            
            // 等待响应
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            boost::asio::streambuf response;
            boost::system::error_code ec;
            boost::asio::read_until(*port_, response, "\r\n", ec);
            if (!ec) {
                std::istream is(&response);
                std::string resp_line;
                std::getline(is, resp_line);
                // RCLCPP_INFO_STREAM(this->get_logger(), "响应: " << resp_line);
            }
            
            // RCLCPP_INFO(this->get_logger(), "✅ GPS频率已设置为 20Hz");
            // RCLCPP_INFO(this->get_logger(), "=== 发布GPS数据 (话题: /gps_data, 格式: [纬度,经度,质量,卫星数]) ===");
            
            // 启动读取循环
            readLoop();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "错误: " << e.what());
            throw;
        }
    }
    
    void readLoop() {
        boost::asio::streambuf buffer;
        std::string line;
        
        rclcpp::Rate loop_rate(20);  // 20Hz发布频率
        
        while (rclcpp::ok()) {
            boost::asio::read_until(*port_, buffer, "\r\n");
            std::istream is(&buffer);
            std::getline(is, line);
            
            if (!line.empty() && line[0] == '$') {
                auto data = parser_.parseGGA(line);
                if (data.isValid) {
                    // 创建消息
                    auto msg = std_msgs::msg::Float64MultiArray();
                    msg.data.resize(4);
                    msg.data[0] = data.latitude;   // 纬度
                    msg.data[1] = data.longitude;  // 经度
                    msg.data[2] = data.quality;    // 质量
                    msg.data[3] = data.satellites; // 卫星数
                    
                    // 发布消息
                    gps_pub_->publish(msg);
                    
                    //打印到终端
                    std::cout << std::fixed << std::setprecision(8);
                    std::cout << data.latitude << ","
                              << data.longitude << ","
                              << data.quality << ","
                              << data.satellites << std::endl;
                }
            }
            
            //rclcpp::spin_some(this->shared_from_this());
            loop_rate.sleep();
        }
        
        port_->close();
    }
};

// ==================== 主函数 ====================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}