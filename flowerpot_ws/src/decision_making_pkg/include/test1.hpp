// decision_making.h
#ifndef DECISION_MAKING_H
#define DECISION_MAKING_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "common_msgs_pkg/msg/serial_data.hpp"
#include "decision_making_pkg/srv/start_transport.hpp"
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include "pid_controller.h"

#define M_PI 3.14159265358979323846
#define GPS_TO_POTPOINT_DISTANCE 0.527 // 机器人GPS定位与花盆实际坐标距离(米)
#define Lidar_TO_POTPOINT_DISTANCE 0.2   // 激光雷达与花盆实际坐标距离(米)

// 消息类型别名
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using Bool = std_msgs::msg::Bool;
using Float32 = std_msgs::msg::Float32;
using Int8 = std_msgs::msg::Int8;
using SerialData = common_msgs_pkg::msg::SerialData;
using StartTransport = decision_making_pkg::srv::StartTransport;

// 静态变量声明（在cpp文件中定义）
extern bool ref_initialized;
extern double ref_lat_;
extern double ref_lon_;

struct RectangleArea {
    // 四个点，顺序：左上、右上、右下、左下
    // |（第一个点）-----------（第二个点）
    // |  o o o                      |
    // |                             |
    // |                             |
    // |                             |
    // |                             |
    // |                             |
    // |                             |
    // |                             |
    // |                             |
    // |                             |
    // |                             |
    // |（第四个点）-----------（第三个点）
    std::vector<Point2D> points;  
    bool is_complete;
    
    RectangleArea() : is_complete(false) {}
    
    // 检查矩形是否有效（四个点都不为空）
    bool isValid() const {
        return points.size() == 4;
    }
    
    // 清空矩形
    void clear() {
        points.clear();
        is_complete = false;
    }
    
    // 打印矩形信息
    void print(rclcpp::Logger logger) const {
        if (points.size() != 4) {
            RCLCPP_WARN(logger, "矩形未完成，当前点数: %zu", points.size());
            return;
        }
        
        RCLCPP_INFO(logger, "========== 目标区域矩形 ==========");
        RCLCPP_INFO(logger, "点1 (左下): %.3f, %.3f", points[0].x, points[0].y);
        RCLCPP_INFO(logger, "点2 (右下): %.3f, %.3f", points[1].x, points[1].y);
        RCLCPP_INFO(logger, "点3 (右上): %.3f, %.3f", points[2].x, points[2].y);
        RCLCPP_INFO(logger, "点4 (左上): %.3f, %.3f", points[3].x, points[3].y);
        RCLCPP_INFO(logger, "=================================");
    }
};

class DecisionMaking : public rclcpp::Node {
private:
    // 订阅者
    rclcpp::Subscription<Float64MultiArray>::SharedPtr gps_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr pot_coords_sub_; // 订阅花盆坐标的订阅者
    rclcpp::Subscription<Float32>::SharedPtr yaw_sub_;       // 订阅偏航角（从IMU获取）
    // 发布者
    rclcpp::Publisher<SerialData>::SharedPtr serial_data_pub_;
    rclcpp::Publisher<Int8>::SharedPtr transported_pot_count_pub_;  // 发布已搬运花盆数量
    // 服务
    rclcpp::Service<StartTransport>::SharedPtr config_start_service_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

private:
    // 参数
    int required_points_;                  // 需要的点数（默认4）
    double record_interval_;               // 记录间隔时间（秒），防止重复记录
    double pot_placement_spacing;          // 花盆摆放间距（米）

    enum class PlacementType {
        GRID_PLACEMENT = 0,    // 网格摆放
        TRIANGULAR_PLACEMENT = 1,  // 三角错位摆放
    };

    PlacementType pot_placement_type_ = PlacementType::GRID_PLACEMENT; // 默认摆放方式

private:
    // 当前状态
    PosePoint current_PosePoint_;              // 当前位姿
    // 目标点
    Point2D target_point_;               // 目标位置

    // 待摆放花盆坐标矩阵，用于记录每个花盆的坐标；
    std::vector<std::vector<Point2D>> target_pots_matrix_; 
    int total_targets;                   // 总的可摆放目标点数量
    static Point target_area_top_left;     // 目标区域左上角坐标（相对于参考点的平面坐标）
    static Point target_area_top_right; // 目标区域右上角坐标（相对于参考点的平面坐标）
    double target_area_bearing;          // 两点连线的方向角（弧度）

    // 已记录的GPS点
    std::vector<Point2D> recorded_points_={
        {0.5, -0.26},
        {-0.05, 0.86},
        {3, -3.4},
        {-3.1, -2}
    }; 
    rclcpp::Time last_record_time_;           // 上次记录时间
    bool gps_flag;                        // GPS标志位（从参数服务器读取）

    Point2D current_pot_coordinate; // 记录当前要抓取的花盆坐标

    Point2D Start_Point_; // 记录花盆摆放区坐标，会更新（相对于参考点的平面坐标）

    bool grasped_status = false; // 抓取完成状态
    bool released_status = false; // 释放完成状态

    // 搬运状态
    enum class TaskState {
        WAITING_START,      // 等待开始信号
        MOVING_TO_PICKUP,   // 移动到取花点
        PICKING_UP,         // 取花中
        MOVING_TO_PLACE,    // 移动到摆放点
        PLACING,            // 摆放花盆
        TASK_COMPLETE       // 任务完成
    };
    TaskState current_state;

    // 当前搬运的目标点索引,同时也表示已搬运花盆数量
    int current_target_index = 0;
    Int8 count_msg;

private:
    // 发布执行指令
    SerialData serial_msg;    //通过串口发送给stm32的数据
    int command;  //用于记录机械臂的指令

    // 抓取命令
    const int COMMAND_GRASP = 0x03;  // 抓取指令
    const int COMMAND_RELEASE = 0x04; // 释放指令
    const int COMMAND_COMMON = 0x05;  // 平常指令

private:
    // 回调函数
    void gpsCallback(const Float64MultiArray::SharedPtr msg);
    void potCoordsCallback(const Float64MultiArray::SharedPtr msg);
    void yawCallback(const Float32::SharedPtr msg);

    // 划定目标区域记录点
    void recordCurrentPosition(const Point2D& point);

    // 计算两点之间的距离
    double distance(Point2D point1, Point2D point2);
    // 获取当前要摆放的目标点
    Point2D getCurrentTargetPoint();

    bool configstartCallback(const std::shared_ptr<StartTransport::Request> req,
                             std::shared_ptr<StartTransport::Response> res);
    
    // 计算每个花盆的摆放坐标
    void calculatePotsMatrix(
        const std::vector<Point2D>& recorded_points, 
        double pot_placement_spacing,
        PlacementType placement_type = PlacementType::GRID_PLACEMENT);
    
    // 根据两点的经纬度，计算垂直于这两点连线的方向角（用于确定摆放方向）
    double calculatePerpendicularBearing(const Point& p1, const Point& p2);
    
    // 等待开始信号
    void WAITING_START_State();
    // 移动到取花点
    void MOVING_TO_PICKUP_State();
    // 取花中
    void PICKING_UP_State();
    // 移动到摆放点
    void MOVING_TO_PLACE_State();
    // 摆放花盆
    void PLACING_State();
    // 任务完成
    void TASK_COMPLETE_State();
    
    // 定时器回调（替代原来的run循环）
    void timerCallback();

public:
    DecisionMaking();
    ~DecisionMaking();
    
    // 主循环（改为非阻塞方式）
    void run();
};

#endif // DECISION_MAKING_H