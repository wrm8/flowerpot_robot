// decision_making.cpp
#include "decision_making.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

bool Point2D::ref_initialized = false;
double Point2D::ref_lat = 0.0;
double Point2D::ref_lon = 0.0;

Point DecisionMaking::target_area_top_left(0.0, 0.0);
Point DecisionMaking::target_area_top_right(0.0, 0.0);

DecisionMaking::DecisionMaking() : Node("decision_making_node"),
    required_points_(4),
    record_interval_(1.0),
    gps_flag(false),
    pot_placement_spacing(0.45),
    current_state(TaskState::MOVING_TO_PLACE),
    total_targets(0),
    target_area_bearing(0.0),
    command(0),
    grasped_status(false),
    released_status(false),
    current_target_index(0)
{
    // 订阅GPS数据
    gps_sub_ = this->create_subscription<Float64MultiArray>(
        "/gps_data", 10, std::bind(&DecisionMaking::gpsCallback, this, std::placeholders::_1));
    
    // 订阅花盆坐标参数
    pot_coords_sub_ = this->create_subscription<Float64MultiArray>(
        "/pot_coords", 10, std::bind(&DecisionMaking::potCoordsCallback, this, std::placeholders::_1));
    
    // 订阅偏航角（从IMU获取）
    yaw_sub_ = this->create_subscription<Float32>(
        "/yaw_angle", 10, std::bind(&DecisionMaking::yawCallback, this, std::placeholders::_1));
    
    // 创建Service服务器
    config_start_service_ = this->create_service<StartTransport>(
        "/start_transport", 
        std::bind(&DecisionMaking::configstartCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // 初始化发布者
    serial_data_pub_ = this->create_publisher<SerialData>("serial_data", 1000);
    transported_pot_count_pub_ = this->create_publisher<Int8>("transported_pot_count", 10);
    
    // 声明参数
    this->declare_parameter("robot.gps_flag", false);
    this->declare_parameter("robot.grasped", false);
    this->declare_parameter("robot.released", false);
    
    // 初始化消息
    count_msg.data = 0;
    serial_msg.pwm_left = 0;
    serial_msg.pwm_right = 0;
    serial_msg.command = COMMAND_COMMON;
    
    // 初始化记录的点（示例数据）
    recorded_points_ = {
        Point2D(0.5, -0.26),
        Point2D(-0.05, 0.86),
        Point2D(3, -3.4),
        Point2D(-3.1, -2)
    };
    
    // 创建定时器（10Hz）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DecisionMaking::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "DecisionMaking node initialized");
}

DecisionMaking::~DecisionMaking() {}

// 定时器回调
void DecisionMaking::timerCallback() {
    run();
}

// 处理接收到的GPS数据
void DecisionMaking::gpsCallback(const Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < required_points_) {
        RCLCPP_WARN(this->get_logger(), "GPS data error!");
        return;
    }
    
    double lat = msg->data[0];
    double lon = msg->data[1];
    int quality = (int)msg->data[2];

    if (quality > 3) {
        if (!Point2D::ref_initialized) {   // 只接收GPS数据中的第一个有效点作为参考点
            Point2D::ref_lat = lat;
            Point2D::ref_lon = lon;
            Point2D::ref_initialized = true;
            RCLCPP_INFO(this->get_logger(), "The reference point has been set: (%.8f, %.8f)", 
                       Point2D::ref_lat, Point2D::ref_lon);
        } else if (recorded_points_.size() < 4) { 
            this->get_parameter("robot.gps_flag", gps_flag);
            if(gps_flag == true) { 
                double time_since_last = (this->now() - last_record_time_).seconds();
                if(time_since_last < record_interval_) {
                    RCLCPP_WARN(this->get_logger(), "If the recording interval has not arrived, ignore the current point");
                    return;
                }
                // 计算目标区域的方向角(以左上角和右上角计算)
                Point temp_top_left(lat, lon);
                Point temp_top_right(lat, lon);
                
                if (recorded_points_.size() == 0) {
                    target_area_top_left = Point(lat, lon);
                    target_area_top_right = target_area_top_left;
                } else if (recorded_points_.size() == 1) {
                    target_area_top_right = Point(lat, lon);
                }
                
                target_area_bearing = calculatePerpendicularBearing(target_area_top_left, target_area_top_right); 

                Point2D current_point = Point2D::latlon_To_xy(lat, lon, Point2D::ref_lat, Point2D::ref_lon);
                recordCurrentPosition(current_point);
                RCLCPP_INFO(this->get_logger(), "Have Recorded %d point", (int)recorded_points_.size());
                this->set_parameter(rclcpp::Parameter("robot.gps_flag", false)); // 记录完当前点后将GPS标志位重置为false，等待下一次触发
            }
        } else {
            current_PosePoint_.point = Point2D::latlon_To_xy(lat, lon, Point2D::ref_lat, Point2D::ref_lon);
        }
    } 
}

bool DecisionMaking::configstartCallback(const std::shared_ptr<StartTransport::Request> req,
                                         std::shared_ptr<StartTransport::Response> res) {
    // 验证参数
    if (req->placement_spacing < 0.2 || req->placement_spacing > 2.0) {
        res->success = false;
        res->message = "The placement spacing must be between 0.2 and 2.0 meters";
        return true;
    }
    
    // 设置参数
    pot_placement_type_ = (req->placement_type == 0) ? 
                         PlacementType::GRID_PLACEMENT : 
                         PlacementType::TRIANGULAR_PLACEMENT;
    pot_placement_spacing = req->placement_spacing;
    
    RCLCPP_INFO(this->get_logger(), "Received configuration: Placement method=%s, Spacing=%.2fm", 
             req->placement_type == 0 ? "Grid" : "Triangle",
             pot_placement_spacing);
    
    // 如果需要立即开始
    if (req->start) {
        RCLCPP_INFO(this->get_logger(), "Start the moving task");
        current_state = TaskState::PICKING_UP;
    } else {
        current_state = TaskState::WAITING_START;
        RCLCPP_INFO(this->get_logger(), "The configuration has been set. Wait for the start signal");
    }
    
    res->success = true;
    res->message = "Configuration successful";
    return true;
}

// 处理待抓取花盆坐标参数更新
void DecisionMaking::potCoordsCallback(const Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() % 2 != 0) {
        RCLCPP_WARN(this->get_logger(), "The format of the flowerpot coordinate data is incorrect");
        return;
    }
    current_pot_coordinate = Point2D(msg->data[0]/100.0, msg->data[1]/100.0);  // 将坐标从厘米转换为米
    RCLCPP_DEBUG(this->get_logger(), "Pot coordinate updated: (%.2f, %.2f)", 
                 current_pot_coordinate.x, current_pot_coordinate.y);
}

// 处理接收到的yaw数据
void DecisionMaking::yawCallback(const Float32::SharedPtr yaw_msg) {
    current_PosePoint_.yaw = yaw_msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Heading Angle update: %.2f°", current_PosePoint_.yaw);
}

// 划定目标区域
void DecisionMaking::recordCurrentPosition(const Point2D& point) {
    // 检查是否已经记录了足够的点
    if (recorded_points_.size() > required_points_) {
        RCLCPP_WARN(this->get_logger(), "have been recorded %d points,Ignore the new points", (int)recorded_points_.size());
        return;
    } else if(recorded_points_.size() == required_points_) {    
        // 根据四个点的坐标计算每个花盆的摆放坐标
        for(const auto& point_: recorded_points_) {
            RCLCPP_INFO(this->get_logger(), "Point.x=%.2f, Point.y=%.2f", point_.x, point_.y);
        }
        calculatePotsMatrix(recorded_points_, pot_placement_spacing, pot_placement_type_);
        return;
    } else {
        // 记录当前点
        recorded_points_.push_back(point);
        last_record_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Record point %d/%d: (%.8f, %.8f)",
                (int)recorded_points_.size(), required_points_,
                point.x, point.y);
    }
}

// 计算两点之间的距离
double DecisionMaking::distance(Point2D point1, Point2D point2) {
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

// 根据记录的四个点计算每个花盆的摆放坐标
void DecisionMaking::calculatePotsMatrix(const std::vector<Point2D>& recorded_points, 
                                                  double pot_placement_spacing,
                                                  PlacementType placement_type) {
    if (recorded_points.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Four points are needed to define a rectangle at present: %zu", recorded_points.size());
        return;
    }
    
    Point2D top_left = recorded_points[0];
    Point2D top_right = recorded_points[1];
    Point2D bottom_right = recorded_points[2];
    Point2D bottom_left = recorded_points[3];
    
    // 计算矩形宽度和高度
    double width = (distance(bottom_left, bottom_right) 
                    + distance(top_left, top_right)) / 2.0; // 取上下边平均宽度
    double height = (distance(bottom_left, top_left) 
                    + distance(bottom_right, top_right)) / 2.0; // 取左右边平均高度
    
    RCLCPP_INFO(this->get_logger(), "Rectangle: wide=%.2fm, high=%.2fm, spacing=%.2fm", width, height, pot_placement_spacing);
    
    // 计算方向向量
    Point2D x_dir((bottom_right.x - bottom_left.x) / width,
                  (bottom_right.y - bottom_left.y) / width);
    Point2D y_dir((top_left.x - bottom_left.x) / height,
                  (top_left.y - bottom_left.y) / height);
    
    target_pots_matrix_.clear();
    total_targets = 0;
    
    if (placement_type == PlacementType::GRID_PLACEMENT) {
        // 网格摆放
        int cols = (int)(width / pot_placement_spacing) + 1;
        int rows = (int)(height / pot_placement_spacing) + 1;
        
        // 居中偏移
        double x_offset = (width - (cols - 1) * pot_placement_spacing) / 2.0;
        double y_offset = (height - (rows - 1) * pot_placement_spacing) / 2.0;
        
        for (int row = 0; row < rows; row++) {
            std::vector<Point2D> row_pots;
            for (int col = 0; col < cols; col++) {
                double x_pos = x_offset + col * pot_placement_spacing;
                double y_pos = y_offset + row * pot_placement_spacing;
                
                Point2D pot;
                pot.x = bottom_left.x + x_pos * x_dir.x + y_pos * y_dir.x;
                pot.y = bottom_left.y + x_pos * x_dir.y + y_pos * y_dir.y;
                row_pots.push_back(pot);
            }
            target_pots_matrix_.push_back(row_pots);
        }
        RCLCPP_INFO(this->get_logger(), "Grid placement: %drows x %dcols", rows, cols);
        
    } else if (placement_type == PlacementType::TRIANGULAR_PLACEMENT) {
        // 三角错位摆放
        double row_spacing = pot_placement_spacing * 0.866;  // sqrt(3)/2
        int rows = (int)(height / row_spacing) + 1;
        int cols = (int)(width / pot_placement_spacing) + 1;
        
        double x_offset = (width - (cols - 1) * pot_placement_spacing) / 2.0;
        double y_offset = (height - (rows - 1) * row_spacing) / 2.0;
        
        for (int row = 0; row < rows; row++) {
            std::vector<Point2D> row_pots;
            // 奇数行偏移半个间距
            double row_x_offset = x_offset + ((row % 2 == 1) ? pot_placement_spacing / 2.0 : 0);
            int cols_in_row = cols - ((row % 2 == 1) ? 1 : 0);
            
            for (int col = 0; col < cols_in_row; col++) {
                double x_pos = row_x_offset + col * pot_placement_spacing;
                double y_pos = y_offset + row * row_spacing;
                
                Point2D pot;
                pot.x = bottom_left.x + x_pos * x_dir.x + y_pos * y_dir.x;
                pot.y = bottom_left.y + x_pos * x_dir.y + y_pos * y_dir.y;
                row_pots.push_back(pot);
            }
            if (!row_pots.empty()) {
                target_pots_matrix_.push_back(row_pots);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Triangular placement: %drows", rows);
    }
    
    for (const auto& row : target_pots_matrix_) {
        total_targets += row.size();
    }
    RCLCPP_INFO(this->get_logger(), "total generate %d flowerpots", total_targets);
}

// 获取当前要摆放的目标点
Point2D DecisionMaking::getCurrentTargetPoint() {
    if (current_target_index >= total_targets) {
        // 所有花盆已搬运完成
        current_state = TaskState::TASK_COMPLETE;
        RCLCPP_INFO(this->get_logger(), "All the flowerpots have been moved, total %d pots", total_targets);
        return Point2D(0, 0);  // 返回原点或无效点
    }
    
    int row = 0;
    int col = 0;
    int cumulative_count = 0;
    
    // 根据摆放方式计算当前索引对应的行列
    if (pot_placement_type_ == PlacementType::GRID_PLACEMENT) {
        // 网格摆放：每行列数相同
        int cols_per_row = target_pots_matrix_.empty() ? 0 : target_pots_matrix_[0].size();
        if (cols_per_row > 0) {
            row = current_target_index / cols_per_row;
            col = current_target_index % cols_per_row;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Grid placement: index=%d, row=%d, col=%d", current_target_index, row, col);
        
    } else if (pot_placement_type_ == PlacementType::TRIANGULAR_PLACEMENT) {
        // 三角摆放：每行列数可能不同，需要遍历查找
        for (size_t r = 0; r < target_pots_matrix_.size(); r++) {
            size_t cols_in_row = target_pots_matrix_[r].size();
            
            if (current_target_index < cumulative_count + cols_in_row) {
                // 在当前行中
                row = r;
                col = current_target_index - cumulative_count;
                break;
            }
            
            cumulative_count += cols_in_row;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Triangular placement: index=%d, row=%d, col=%d", current_target_index, row, col);
    }
    
    // 边界检查
    if (row >= (int)target_pots_matrix_.size() || 
        col >= (int)target_pots_matrix_[row].size()) {
        RCLCPP_ERROR(this->get_logger(), "Index out-of-bounds: index=%d, row=%d, col=%d, Total number of rows=%zu, Current number of rows and columns=%zu",
                  current_target_index, row, col, 
                  target_pots_matrix_.size(), 
                  (row < (int)target_pots_matrix_.size() ? target_pots_matrix_[row].size() : 0));
        return Point2D(0, 0);
    }
    
    return target_pots_matrix_[row][col];
}

// 根据两点的经纬度，计算垂直于这两点连线的方向角（用于确定摆放方向）
double DecisionMaking::calculatePerpendicularBearing(const Point& p1, const Point& p2) {
    // 计算线段方向角（弧度）
    double dx = p2.longitude - p1.longitude;
    double dy = p2.latitude - p1.latitude;
    double angle = atan2(dx, dy);
    
    // 垂直方向加90度
    double perp_angle = angle + M_PI / 2.0;
    
    // 转换为度数并归一化到0-360
    double bearing = perp_angle * 180.0 / M_PI;
    bearing = fmod(bearing, 360.0);
    if (bearing < 0) bearing += 360.0;
    
    return bearing;
}

// 等待开始信号
void DecisionMaking::WAITING_START_State() {
    RCLCPP_INFO(this->get_logger(), "Waiting...");
    serial_msg.pwm_left = 0;
    serial_msg.pwm_right = 0;
    serial_msg.command = COMMAND_COMMON;   // 发送正常状态指令
    target_point_ = current_PosePoint_.point; // 以当前位姿作为取花点 
}

// 移动到取花点
void DecisionMaking::MOVING_TO_PICKUP_State() {
    RCLCPP_INFO(this->get_logger(), "MOVING_TO_PICKUP...");
    target_point_ = Start_Point_;  
    if (distance(current_PosePoint_.point, target_point_) > 0.1) {
        PWM PWM_Motor = calculatePWM(current_PosePoint_, target_point_);
        serial_msg.pwm_left = PWM_Motor.PWM_Left;
        serial_msg.pwm_right = PWM_Motor.PWM_Right;
        serial_msg.command = COMMAND_COMMON;   // 发送正常状态指令
    } else {
        RCLCPP_INFO(this->get_logger(), "Arrived at placement point, ready to place flower pots.");
        current_state = TaskState::PICKING_UP; // 进入取花状态
    }
}

// 取花中
void DecisionMaking::PICKING_UP_State() {
    RCLCPP_INFO(this->get_logger(), "PICKING_UP...");
    if (current_pot_coordinate.y > GPS_TO_POTPOINT_DISTANCE + 0.1) {
        RCLCPP_INFO(this->get_logger(), "current_pot_coordinate.y=%.2f", current_pot_coordinate.y);
        PWM PWM_Motor = calculatePWM_linear(current_pot_coordinate, 
                            Point2D(0.0, Lidar_TO_POTPOINT_DISTANCE));  // 以取花点为坐标原点，计算当前花盆坐标的PWM值
        serial_msg.pwm_left = PWM_Motor.PWM_Left;
        serial_msg.pwm_right = PWM_Motor.PWM_Right;
    } else {  // 如果y坐标小于设定值，说明花盆已经接近机器人
        serial_msg.pwm_left = 0;
        serial_msg.pwm_right = 0;
        serial_msg.command = COMMAND_GRASP;  // 发送抓取指令
    }
    this->get_parameter("robot.grasped", grasped_status);
    if(grasped_status) {
        RCLCPP_INFO(this->get_logger(), "Grasping completed, moving to placement point.");
        current_state = TaskState::MOVING_TO_PLACE;
        this->set_parameter(rclcpp::Parameter("robot.grasped", false));
    }
}

// 移动到摆放点
void DecisionMaking::MOVING_TO_PLACE_State() {
    RCLCPP_INFO(this->get_logger(), "MOVING_TO_PLACE...");
    target_point_ = getCurrentTargetPoint();  
    if (distance(current_PosePoint_.point, target_point_) > 0.1) {
        PWM PWM_Motor = calculatePWM(current_PosePoint_, target_point_);
        serial_msg.pwm_left = PWM_Motor.PWM_Left;
        serial_msg.pwm_right = PWM_Motor.PWM_Right;
        serial_msg.command = COMMAND_COMMON;   // 发送正常状态指令
    } else {
        if(current_PosePoint_.yaw < target_area_bearing - 5 
                        || current_PosePoint_.yaw > target_area_bearing + 5) {  // 如果偏航角与目标区域方向相差较大，优先调整角度
            PWM angular_pwm = calculatePWM_angular(current_PosePoint_.yaw, target_area_bearing);
            serial_msg.pwm_left = angular_pwm.PWM_Left;
            serial_msg.pwm_right = angular_pwm.PWM_Right;
            serial_msg.command = COMMAND_COMMON;   // 发送正常状态指令
            RCLCPP_INFO(this->get_logger(), "Adjusting angle…");
        } else {
             serial_msg.pwm_left = 0;
             serial_msg.pwm_right = 0;
             serial_msg.command = COMMAND_COMMON;   // 发送正常状态指令
            RCLCPP_INFO(this->get_logger(), "Arrived at placement point, ready to place flower pots.");
            current_state = TaskState::PLACING;
        }
    }
}

// 摆放花盆
void DecisionMaking::PLACING_State() {
    RCLCPP_INFO(this->get_logger(), "Placing flower pot…");

    // 实现摆放
    serial_msg.command = COMMAND_RELEASE;  // 发送释放指令
    this->get_parameter("robot.released", released_status);
    if(released_status) {
        RCLCPP_INFO(this->get_logger(), "Placement completed.");
        current_state = TaskState::MOVING_TO_PICKUP; // 返回等待下一个取花点的状态
        this->set_parameter(rclcpp::Parameter("robot.released", false));
    }

    RCLCPP_INFO(this->get_logger(), "Flower pot No. %d placed successfully.", current_target_index + 1);

    // 移动到下一个花盆
    current_target_index++;
    current_state = TaskState::MOVING_TO_PICKUP;
}

// 任务完成
void DecisionMaking::TASK_COMPLETE_State() {
    serial_msg.pwm_left = 0;
    serial_msg.pwm_right = 0;
    serial_msg.command = COMMAND_COMMON;   // 发送正常状态指令
    RCLCPP_INFO(this->get_logger(), "All flower pots transported successfully!");
}

// 主循环
void DecisionMaking::run() {
    // 更新参数
    this->get_parameter("robot.gps_flag", gps_flag);
    this->get_parameter("robot.grasped", grasped_status);
    this->get_parameter("robot.released", released_status);
    
    // 状态机
    switch (current_state) {
        // 等待开始信号,在点击开始按钮时机器人需要放在待抓取花盆区域（即需要能看到花盆）
        case TaskState::WAITING_START:
            WAITING_START_State();
            break;

        // 移动到取花点
        case TaskState::MOVING_TO_PICKUP:
            MOVING_TO_PICKUP_State();
            break;
        
        // 取花中
        case TaskState::PICKING_UP:
            PICKING_UP_State();
            break;
        
        // 移动到摆放点
        case TaskState::MOVING_TO_PLACE:
            MOVING_TO_PLACE_State();
            break;
        
        // 摆放花盆
        case TaskState::PLACING:
            PLACING_State();
            break;
        
        // 任务完成
        case TaskState::TASK_COMPLETE:
            TASK_COMPLETE_State();
            break;
    }
    
    // 发布数据
    count_msg.data = current_target_index;
    transported_pot_count_pub_->publish(count_msg);  // 发布已搬运花盆数量
    serial_data_pub_->publish(serial_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionMaking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}