#ifndef RADARYOLO11_MELD_NODE_HPP_
#define RADARYOLO11_MELD_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "radar_msgs/msg/radar_target.hpp"
#include "radar_msgs/msg/radar_target_array.hpp"
#include "yolo11_msgs/msg/coordinate.hpp"
#include "yolo11_msgs/msg/coordinate_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "hungarian_algorithm.hpp"
#include <vector>
#include <memory>
#include <string>

namespace radar_yolo11_fusion {

// 相机目标结构（匹配yolo11消息格式）
struct CameraTarget {
    float x;              // X坐标（米）
    float y;              // Y坐标（米）
    float distance;       // 距离（米）
    float angle;          // 角度（弧度）
    float angle_deg;      // 角度（度）
    
    CameraTarget() : x(0), y(0), distance(0), angle(0), angle_deg(0) {}
    CameraTarget(float _x, float _y) : x(_x), y(_y) {
        updatePolar();
    }
    
    void updatePolar() {
        distance = std::hypot(x, y);
        angle = std::atan2(y, x);
        angle_deg = angle * 180.0 / M_PI;
    }
};

// 雷达目标结构（匹配radar_msgs消息格式）
struct RadarTarget {
    uint32_t id;          // ID
    float r;              // 极坐标距离（米）
    float phi;            // 极坐标角度（弧度）
    float x;              // 直角坐标X（米）
    float y;              // 直角坐标Y（米）
    float radius;         // 圆半径（米）
    float distance;       // 距离（米）
    float angle;          // 角度（弧度）
    float angle_deg;      // 角度（度）
    
    RadarTarget() : id(0), r(0), phi(0), x(0), y(0), radius(0), 
                    distance(0), angle(0), angle_deg(0) {}
    
    void updatePolar() {
        distance = r;
        angle = phi;
        angle_deg = phi * 180.0 / M_PI;
    }
};

// 数据来源
enum class DataSource {
    CAMERA_ONLY = 0,
    RADAR_ONLY = 1,
    FUSED = 2
};

// 融合后的目标
struct FusedTarget {
    hungarian::Point position;
    double distance;
    double angle;
    double angle_deg;
    DataSource source;
    double confidence;
    uint32_t id;
};

// 目标关联
struct TargetAssociation {
    int camera_idx;
    int radar_idx;
    double distance;
    double confidence;
    hungarian::Point camera_point;
    hungarian::Point radar_point;
};

class RadarYolo11MeldNode : public rclcpp::Node {
public:
    explicit RadarYolo11MeldNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());//explicit，禁止隐式类型转换，只能显式调用构造函数，不能让编译器偷偷自动转换
    ~RadarYolo11MeldNode();

private:
    // 回调函数
    void radarCallback(const radar_msgs::msg::RadarTargetArray::SharedPtr msg);
    void yolo11Callback(const yolo11_msgs::msg::CoordinateArray::SharedPtr msg);
    
    // 数据预处理
    void preprocessCameraData();
    void preprocessRadarData();
    
    // 核心算法
    std::vector<TargetAssociation> associateTargetsWithHungarian();
    std::vector<RadarTarget> validateRadarPointsWithCamera();
    std::vector<FusedTarget> fuseTargetData();
    
    // 工具函数
    double calculateDistance(const hungarian::Point& p1, const hungarian::Point& p2);
    void publishFusedTargets(const std::vector<FusedTarget>& targets);
    hungarian::Point selectClosestTarget(const std::vector<FusedTarget>& targets);
    
    // 参数配置
    void declareParameters();
    void updateParameters();
    
    // ROS2通信
    rclcpp::Subscription<radar_msgs::msg::RadarTargetArray>::SharedPtr radar_sub_;
    rclcpp::Subscription<yolo11_msgs::msg::CoordinateArray>::SharedPtr yolo11_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fused_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 数据存储
    std::vector<CameraTarget> camera_targets_;
    std::vector<RadarTarget> radar_targets_;
    std::vector<RadarTarget> validated_radar_targets_;
    
    // 参数
    double fusion_distance_threshold_;
    double association_threshold_;
    double radar_min_distance_;
    double radar_max_distance_;
    bool enable_debug_;
    
    // 匈牙利算法求解器
    hungarian::HungarianSolver hungarian_solver_;
    
    // 常量
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;
};

} // namespace radar_yolo11_fusion

#endif