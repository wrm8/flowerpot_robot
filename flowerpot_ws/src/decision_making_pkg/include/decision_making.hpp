#ifndef DECISION_MAKING_HPP_
#define DECISION_MAKING_HPP_

#include "rclcpp/rclcpp.hpp"
#include "common_msgs_pkg/msg/serial_data.hpp"  // 使用已有的消息
#include "common_msgs_pkg/msg/encoder.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pid_controller.hpp"
#include "point.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float64.hpp"   // 加这行！解决Float64找不到的错误
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"     // 加这行！解决Int32找不到的错误
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/string.hpp>
#include "decision_making_pkg/srv/start_transport.hpp"  // 添加服务头文件

class DecisionMaking : public rclcpp::Node {
public:
    DecisionMaking();
    ~DecisionMaking();

    enum class PlacementType {
        GRID_PLACEMENT = 0,    // 网格摆放
        TRIANGULAR_PLACEMENT = 1,  // 三角错位摆放
    };

    void run();//在main中，类外，要public的成员函数，不能放在private里

private:
    // 回调函数
    void fusedTargetsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void encoder_callback(const common_msgs_pkg::msg::Encoder::SharedPtr msg);
    void gpsCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    void yawCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void voiceCmdCallback(const std_msgs::msg::String::SharedPtr msg);
    // 服务回调函数
    void startTransportCallback(
        const std::shared_ptr<decision_making_pkg::srv::StartTransport::Request> request,
        std::shared_ptr<decision_making_pkg::srv::StartTransport::Response> response);
    
    // 控制函数
    void pidTimeUpdate();
    void computeControl();
    void publishControl(int16_t pwm_left, int16_t pwm_right, int8_t command);
    double computeSpeedProfile(); 
    // 根据两点的直角坐标，计算垂直于这两点连线的方向角（用于确定摆放方向）
    double calculatePerpendicularBearing(const Point2D& p1, const Point2D& p2, const Point2D& p3, const Point2D& p4);
    //
    void recordCurrentPosition(const Point2D& point);
    //计算区域内应该摆放的花盆坐标矩阵
    void calculatePotsMatrix(const std::vector<Point2D>& recorded_points, //四个点的坐标
                                                  double pot_placement_spacing,//花盆摆放间距
                                                  PlacementType placement_type);//摆放方式（网格或三角错位）
    //计算两点之间的距离
    double distance(Point2D point1, Point2D point2); 

    std::vector<Point2D> getParkingPointsFromPots(const std::vector<Point2D>& pot_points, 
                                            double target_bearing,    // 你的摆放角度 target_area_bearing
                                            double park_distance );     // 停靠距离：1.0 米
    Point2D getCurrentTargetPoint(double park_distance); // 获取当前目标点（自动返回平移1米后的停车点）

    ////////////////// 状态函数
    void WAITING_START_State();
    void MOVING_TO_PICKUP_State();
    void PICKING_UP_State();
    void MOVING_TO_PLACE_State();
    void PLACING_State();
    void BACK_TO_ORIGIN_State();
    void TASK_COMPLETE_State();
    
    

    
    // 工具函数
    //double normalizeAngle(double angle);
    
    // 订阅者和发布者
       // 三个状态标志位订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr release_flag_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fused_sub_;
    rclcpp::Subscription<common_msgs_pkg::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_sub_;  // 添加这一行
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_cmd_sub_;
    
    rclcpp::Publisher<common_msgs_pkg::msg::SerialData>::SharedPtr serial_pub_;
    // 服务端
    rclcpp::Service<decision_making_pkg::srv::StartTransport>::SharedPtr start_service_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr pid_timer_;
    
    // PID控制器
    control::PIDController angle_pid_;
    control::PIDController speed_pid_;   // 速度PID（距离控制）
    control::PIDController angle_pid_rotate_; // 旋转+直行专用PID

    int target_lost_count_ ;  // 丢目标计数器

    // 2. 三个状态标志位变量
    bool gps_flag;
    bool grasp_flag;
    bool release_flag;
    
    // 控制参数
    double target_distance_;      // 目标距离（米）
    double stop_distance_;        // 停止距离（米）
    double max_speed_;            // 最大速度
    double min_speed_;            // 最小速度

    // 梯形速度曲线参数
    double accel_distance_;       // 加速距离（米）
    double decel_distance_;       // 减速距离（米）
    // 实时速度（从编码器来）
    double current_linear_speed_;
    double total_distance_;    // 本次启动后跑了多远
    bool was_stopped_;        // 上一时刻是不是停止状态
    bool is_moving_;         // 现在是不是运动状态
    // 编码器实时速度
    int32_t encoder_left_speed_;
    int32_t encoder_right_speed_;
    
    
    // 当前目标信息
    bool has_target_;
    double current_distance_;
    double current_angle_;
    double current_speed_;   // 当前计算出的速度

private:
    //GPS获取花田四个点，并形成区域相关参数
    int required_points_;                  // 需要的点数（默认4）
    int record_interval_;               // 记录间隔时间（秒），防止重复记录
    Point2D target_area_top_left;     // 目标区域左上角坐标（相对于参考点的平面坐标）
    Point2D target_area_top_right; // 目标区域右上角坐标（相对于参考点的平面坐标）
    Point2D target_area_bottom_left;  // 目标区域左下角坐标（相对于参考点的平面坐标）
    Point2D target_area_bottom_right; // 目标区域右下角坐标（相对于参考点的平面坐标）
    double target_area_bearing;     // 目标区域方向角（度数）

    std::vector<Point2D> recorded_points_;  // 存储已记录的平面坐标点

    double pot_placement_spacing; // 花盆摆放间距（米）


    PlacementType pot_placement_type_ = PlacementType::GRID_PLACEMENT; // 默认摆放方式

    rclcpp::Time last_record_time_;  // ROS2时间类型

private:
    // 当前状态
    PosePoint current_PosePoint_;              // 当前位姿
    // 目标点
    Point2D target_point_;               // 目标位置

    // 待摆放花盆坐标矩阵，用于记录每个花盆的坐标；
    std::vector<std::vector<Point2D>> target_pots_matrix_; 
    int total_targets;                   // 总的可摆放目标点数量

    // // 测试用的GPS点，单位米
    // std::vector<Point2D> recorded_points_ = {
    // Point2D(1.93,2.02 ),
    // Point2D(3.68, 0.66),
    // Point2D(2.11, -2.05),
    // Point2D(-0.02, -0.10)
    // };


    Point2D current_pot_coordinate; // 记录当前要抓取的花盆坐标

    Point2D Start_Point_; // 记录花盆摆放区坐标，会更新（相对于参考点的平面坐标）
private:
    // ========================= 状态枚举 =========================
    enum class TaskState
    {
        WAITING_START,        // 等待开始
        MOVING_TO_PICKUP,     // 去取花点
        PICKING_UP,           // 抓取中
        MOVING_TO_PLACE,      // 去1米停车点（三合一：导航+调角+前进25cm）
        PLACING,              // 摆放
        //BACK_TO_ORIGIN,       // 回原点（三合一：后退+转向+返航）
        TASK_COMPLETE         // 完成
    };

    // ========================= 任务变量 =========================
    TaskState current_state = TaskState::WAITING_START;

    int current_target_index = 0;   // 当前花盆序号
    bool grasped_status = false;    // 抓取完成标志
    bool released_status = false;   // 释放完成标志

    Point2D back_pos_;      // 每次返回的取花点

    // 当前搬运的目标点索引,同时也表示已搬运花盆数量
    std_msgs::msg::Int32 count_msg;


private:
    
    // 抓取命令
    const int COMMAND_GRASP = 0x03;  // 抓取指令
    const int COMMAND_RELEASE = 0x04; // 释放指令
    const int COMMAND_COMMON = 0x05;  // 平常指令
    // 上次控制时间,pid定时器专用
    rclcpp::Time last_control_time_;
    double Run_Hz ; // 主循环频率（Hz）
    double Pid_Hz ; // PID控制频率（Hz）
    
    // 常量
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;
};

#endif // DECISION_MAKING_HPP_