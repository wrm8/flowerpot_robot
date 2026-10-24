#include "decision_making.hpp"
#include <cmath>
#include <algorithm>

bool Point2D::ref_initialized = false;
double Point2D::ref_lat = 0.0;
double Point2D::ref_lon = 0.0;

int wrm1 = 0;
int wrm2 = 0;
int wrm3 = 0;

DecisionMaking::DecisionMaking() 
    : Node("decision_making_node"),
      angle_pid_(control::PIDGains{3.5, 0.2, 0.2}, 0.1, 1000, 100.0),
      speed_pid_(control::PIDGains{380, 20, 10}, 0.1, 1000, 50.0),    // 速度PID
      angle_pid_rotate_(control::PIDGains{8.0, 0.2, 0.2}, 0.1, 1000, 200.0), // 旋转+直行专用PID
      target_lost_count_(0),
      gps_flag(false),
      grasp_flag(false),
      release_flag(false),
      
      current_linear_speed_(0.0),
      total_distance_(0.0),//本次启动后跑了多远
      was_stopped_(true),       // 上一时刻是不是停止状态
      is_moving_(false),        // 现在是不是运动状态
      // 编码器实时速度
      encoder_left_speed_(0),
      encoder_right_speed_(0),
      has_target_(false),
      current_distance_(0.0),
      current_angle_(0.0),
      current_speed_(0.0),

      required_points_(4),
      record_interval_(1.0),
      target_area_top_left(0.0, 0.0),
      target_area_top_right(0.0, 0.0),
      target_area_bottom_left(0.0, 0.0),
      target_area_bottom_right(0.0, 0.0),
      target_area_bearing(0.0),
      total_targets(0),
      pot_placement_spacing(0.5), // 花盆摆放间距，默认0.5米
      pot_placement_type_(PlacementType::GRID_PLACEMENT),//默认矩形摆放
      Run_Hz(10.0), // 主循环频率（Hz）,100ms=10HZ
      Pid_Hz(Run_Hz) // PID控制频率（Hz）
    

{
    // 声明参数
    this->declare_parameter<double>("target_distance", 0.3);   // 目标减速开始距离
    this->declare_parameter<double>("stop_distance", 0.3);     // 作业停止距离
    this->declare_parameter<double>("max_speed", 0.8);         // 最大速度
    this->declare_parameter<double>("min_speed", 0.1);         // 最小速度
    this->declare_parameter<double>("accel_distance", 0.3);       // 加速距离2米
    //this->declare_parameter<double>("decel_distance", 1.5);       // 减速距离1.5米

    // 角度PID参数
    this->declare_parameter<double>("angle_kp", 4);
    this->declare_parameter<double>("angle_ki", 0.2);
    this->declare_parameter<double>("angle_kd", 0.2);
    
    // 速度PID参数
    this->declare_parameter<double>("speed_kp", 380.0);
    this->declare_parameter<double>("speed_ki", 20.0);
    this->declare_parameter<double>("speed_kd", 10);

    // 旋转PID参数
    this->declare_parameter<double>("angle_rotate_kp", 8.0);
    this->declare_parameter<double>("angle_rotate_ki", 0.2);
    this->declare_parameter<double>("angle_rotate_kd", 0.2);

    this->get_parameter("target_distance", target_distance_);
    this->get_parameter("stop_distance", stop_distance_);
    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("min_speed", min_speed_);
    this->get_parameter("accel_distance", accel_distance_);
    //this->get_parameter("decel_distance", decel_distance_);
    
    double kp, ki, kd;
    this->get_parameter("angle_kp", kp);
    this->get_parameter("angle_ki", ki);
    this->get_parameter("angle_kd", kd);
    angle_pid_.setGains({kp, ki, kd});
    
    this->get_parameter("speed_kp", kp);
    this->get_parameter("speed_ki", ki);
    this->get_parameter("speed_kd", kd);
    speed_pid_.setGains({kp, ki, kd});

    this->get_parameter("angle_rotate_kp", kp);
    this->get_parameter("angle_rotate_ki", ki);
    this->get_parameter("angle_rotate_kd", kd);
    angle_pid_rotate_.setGains({kp, ki, kd});


    // GPS 标志
    gps_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/gps_flag", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            this->gps_flag = msg->data;});
    // 抓取标志
    grasp_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/grasp_flag", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            this->grasp_flag = msg->data;});
    // 释放标志
    release_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/release_flag", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            this->release_flag = msg->data;});
    
    // 订阅融合数据
    fused_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "fused_targets", 10,
        std::bind(&DecisionMaking::fusedTargetsCallback, this, std::placeholders::_1));
    // 订阅编码器话题（接收你发的消息）
    encoder_sub_ = this->create_subscription<common_msgs_pkg::msg::Encoder>(
        "encoder_msg",  50,            
        std::bind(&DecisionMaking::encoder_callback, this, std::placeholders::_1));
    // 订阅偏航角（从IMU获取）
    yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/yaw_angle",  10,
        std::bind(&DecisionMaking::yawCallback, this, std::placeholders::_1));
    gps_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "gps_data", 10,
        std::bind(&DecisionMaking::gpsCallback, this, std::placeholders::_1));
    // 订阅语音指令话题 /robot_action_cmd
    voice_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_action_cmd", 10,
        std::bind(&DecisionMaking::voiceCmdCallback, this, std::placeholders::_1));

    
    // 发布串口数据
    serial_pub_ = this->create_publisher<common_msgs_pkg::msg::SerialData>("serial_data", 10);
    // 创建服务
    start_service_ = this->create_service<decision_making_pkg::srv::StartTransport>(
        "/start_transport",  // 服务名称
        std::bind(&DecisionMaking::startTransportCallback, this,
              std::placeholders::_1, std::placeholders::_2));
    
    // // 控制定时器（20Hz）
    // control_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(50),
    //     std::bind(&DecisionMaking::computeControl, this));
    // last_control_time_ = this->now();

    // 构造函数里加：专门给PID算dt的定时器（20Hz）
    pid_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / Pid_Hz)),
        std::bind(&DecisionMaking::pidTimeUpdate, this));

    last_control_time_ = this->now();
    // // ========================
    // // 在这里加一行！直接测试矩阵分点！
    // // ========================
    // calculatePotsMatrix(recorded_points_, 1, PlacementType::GRID_PLACEMENT);
    
    // RCLCPP_INFO(this->get_logger(), "Decision making node started");
    // RCLCPP_INFO(this->get_logger(), "Target: %.2f m, Stop: %.2f m, Max speed: %.2f m/s",
    //             target_distance_, stop_distance_, max_speed_);
    // RCLCPP_INFO(this->get_logger(), "Accel dist: %.2f m, Decel dist: %.2f m",
    //             accel_distance_, decel_distance_);
}

DecisionMaking::~DecisionMaking() {}

void DecisionMaking::pidTimeUpdate()
{
    // 计算时间差
    rclcpp::Time now = this->now();
    double dt = (now - last_control_time_).seconds();
    if (dt <= 0) dt = 0.1;
    last_control_time_ = now;
}

void DecisionMaking::fusedTargetsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // 消息格式: [x, y, distance, angle_rad, angle_deg, source, confidence, id]
    // 每个目标8个数据
    if (msg->data.empty()) {
        has_target_ = false;
        // 🔥 强制打印
        // RCLCPP_INFO(this->get_logger(), "🔥 目标丢失 → has_target_ = false");
        RCLCPP_DEBUG(this->get_logger(), "No target detected");
        return;
    }
    
    // 取第一个目标（最近的花盆）
    current_distance_ = msg->data[2];   // 距离
    current_angle_ = msg->data[4];      // 角度（度）
    has_target_ = true;
    
    // RCLCPP_INFO(this->get_logger(), 
    //     "Target: distance=%.2f m, angle=%.1f deg",
    //     current_distance_, current_angle_);
}

void DecisionMaking::encoder_callback(const common_msgs_pkg::msg::Encoder::SharedPtr msg)
{
    // 在这里就能收到编码器数据！
    int32_t left_speed = msg->left_speed;
    int32_t right_speed = msg->right_speed;
    encoder_left_speed_ = left_speed;
    encoder_right_speed_ = right_speed;
    // 机器人中心线速度 = 左右平均
    current_linear_speed_ = (left_speed + right_speed) / 2.0;
    // ==============================================
    // 关键：累计行驶距离（单位：米）
    // 0.1 是你的控制周期 10Hz，如果你是 50Hz 就写 0.02
    // ==============================================
    // 2. 判断是否静止
    bool current_stopped = (fabs(current_linear_speed_) < 0.02);

    // ==============================================
    // 核心逻辑：只有【停止 → 启动】瞬间清零里程
    // ==============================================
    if (was_stopped_ && !current_stopped)
    {
        // 刚刚从静止 → 启动！清零！
        total_distance_ = 0.0;
        is_moving_ = true;
    }

    // 如果已经停止
    if (current_stopped) {
        is_moving_ = false;
    }

    // 记录上一帧状态
    was_stopped_ = current_stopped;

    // 3. 累计本次启动行驶距离（10Hz控制周期）
    if (is_moving_) {
        total_distance_ += current_linear_speed_ * 0.1;
    }

    //RCLCPP_INFO(this->get_logger(), "收到编码器：左=%d, 右=%d", left_speed, right_speed);
}

double DecisionMaking::computeSpeedProfile() {

    // 最后减速 80 厘米（你自己可调）
    const double DECEL_DIST = 0.8;
    
    if (is_moving_ && total_distance_ <= accel_distance_ ) {
        // 0 ~ 0.3m 线性从 min → max
        double t = total_distance_ / accel_distance_;
        t = std::clamp(t, 0.0, 1.0);
        return min_speed_ + (max_speed_ - min_speed_) * t;
        
    } else if (current_distance_ <= DECEL_DIST) {
        double distance_error = current_distance_ - stop_distance_;
        double speed_40 = speed_pid_.calculate(distance_error);//根据距离误差计算速度，输出范围是0~1
        double speed_pid = max_speed_ * speed_40;
        return speed_pid;
        
    } else {
        // 匀速阶段：最大速度
        return max_speed_;
    }
}

void DecisionMaking::computeControl() {

    if (!has_target_) {
        target_lost_count_++;
        if(target_lost_count_ >= 4)//连续3次（0.3秒）丢目标才真正停止，避免偶尔的目标丢失导致控制命令频繁切换
        {
            // 没有目标，停止
            //RCLCPP_INFO(this->get_logger(), "🔥 目标丢失 → has_target_ = false");
            publishControl(0, 0, COMMAND_COMMON);
            target_lost_count_ = 0;
            angle_pid_.reset();
            speed_pid_.reset();
        }
        return;
    }

    // if (!has_target_) {
    //         // 没有目标，停止
    //         RCLCPP_INFO(this->get_logger(), "🔥 目标丢失 → has_target_ = false");
    //         publishControl(0, 0, COMMAND_COMMON);
    //         return;
        
    // }
    // 只要收到一次目标 → 立刻清零
    target_lost_count_ = 0;

    // 检查是否到达目标
    if (current_distance_ <= stop_distance_) {
        //RCLCPP_INFO(this->get_logger(), "Reached target! distance=%.2f m", current_distance_);
        has_target_ = false;
        angle_pid_.reset();
        speed_pid_.reset();   // <--- 加这个
        was_stopped_ = true;  // <--- 加这个（关键！下次启动重新加速）
        publishControl(0, 0, COMMAND_COMMON);
        grasp_flag = true; // 到位了，设置抓取标志，用来假设抓取了花盆，进入下一个目标
       // publishControl(0, 0, COMMAND_GRASP);  // 停止并抓取
        return;
    }

    pidTimeUpdate(); // 先更新PID的时间差

    // PID控制角度误差（期望角度为90，即正对花盆）
    double angle_error = 90-current_angle_;  // ＋号表示目标在右边要右转，－号表示目标在左边要zuo转
    double distance_error = current_distance_ - stop_distance_; // 距离误差，正数表示还远着，负数表示已经过头了
    double steering = angle_pid_.calculate(angle_error);
    double speed = speed_pid_.calculate(distance_error);//根据距离误差计算速度

    //double speed = computeSpeedProfile();
    // 速度不能为负
    //speed = std::max(0.0, speed);
    speed = std::clamp(speed, -400.0, 400.0);
    steering = std::clamp(steering, -100.0, 100.0);

    int16_t pwm_left = static_cast<int16_t>(speed + steering);
    int16_t pwm_right = static_cast<int16_t>(speed - steering);    
    // int16_t pwm_left = static_cast<int16_t>(speed * 1000 );
    // int16_t pwm_right = static_cast<int16_t>(speed * 1000 );  
      
    // 限制PWM范围（✅ 修复版）
    pwm_left = std::clamp(pwm_left, (int16_t)-1000, (int16_t)1000);
    pwm_right = std::clamp(pwm_right, (int16_t)-1000, (int16_t)1000);


    // 发布控制命令
    publishControl(pwm_left, pwm_right, COMMAND_COMMON); // 0x05 是普通控制命令，不抓取,不释放
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Control: dist=%.2f, angle=%.1f, steering=%.2f, speed=%.2f, PWM=(%d, %d)",
        current_distance_, current_angle_, steering, speed, pwm_left, pwm_right);
}

void DecisionMaking::publishControl(int16_t pwm_left, int16_t pwm_right, int8_t command) {
    auto msg = std::make_shared<common_msgs_pkg::msg::SerialData>();
    // msg->data.push_back(pwm_left);
    // msg->data.push_back(pwm_right);
    // msg->data.push_back(command);
    // 直接赋值！不是 push_back！push_back 是给数组用的！
    msg->pwm_left = pwm_left/8;
    msg->pwm_right = pwm_right/8;
    msg->command = command;
    serial_pub_->publish(*msg);
}

void DecisionMaking::yawCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    // 直接获取角度
    double yaw_angle = msg->data + 13.0; // 自己调这个补偿值，调整到小车正对坐标系y时显示为0度
    //RCLCPP_INFO(this->get_logger(), "imu角度%.2f)", yaw_angle);

    // 2. 转换成 0~360° 顺时针角度（核心代码，就这一行）
    double yaw_360 = fmod(-yaw_angle + 360.0, 360.0);
    //RCLCPP_INFO(this->get_logger(), "转换后的·imu角度%.2f)", yaw_360);

    // 赋值给你自己的变量
    current_PosePoint_.yaw = yaw_360;
    //RCLCPP_INFO(this->get_logger(), "current_PosePoint_.yaw=%.2f)", current_PosePoint_.yaw);

    RCLCPP_DEBUG(this->get_logger(), "Current yaw: %.1f", yaw_360);
}

// 处理接收到的GPS数据
void DecisionMaking::gpsCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
    if (msg->data.size() < required_points_) {
        RCLCPP_WARN(this->get_logger(), "GPS data error!");
        return;
    }
    
    double lat = msg->data[0];
    double lon = msg->data[1];
    int quality = (int)msg->data[2];
    

    if (quality > 3) {
        if (!Point2D::ref_initialized) {   // 只接收GPS数据中的第一个有效点作为参考点，存第一个 GPS 点，作为整个地图的原点（0,0）
            Point2D::ref_lat = lat;//原点纬度
            Point2D::ref_lon = lon;//原点经度
            Point2D::ref_initialized = true;
            RCLCPP_INFO(this->get_logger(), "The reference point has been set: (%.8f, %.8f)", Point2D::ref_lat, Point2D::ref_lon);
        } else if (recorded_points_.size() < 4) { 
            //RCLCPP_INFO(this->get_logger(), " %d point recorded", (int)recorded_points_.size());
            
            if(gps_flag == true) { 
                // double time_since_last = (this->now() - last_record_time_).seconds();
                // if(time_since_last < record_interval_) {
                //     RCLCPP_WARN(this->get_logger(), "If the recording interval has not arrived, ignore the current point");
                //     return;
                // }
                //实时小车坐标
                Point2D current_point = Point2D::latlon_To_xy(lat, lon, Point2D::ref_lat, Point2D::ref_lon);//静态函数，调用必须写：结构体名字 + 双冒号
                recordCurrentPosition(current_point);

                // 计算目标区域的方向角(以左上角和右上角计算,第一个点左上角，第二个点右上角，第三个点右下角，第四个点左下角)
                target_area_top_left = recorded_points_.size() == 1 ? recorded_points_[0] : target_area_top_left;
                target_area_top_right = recorded_points_.size() == 2 ? recorded_points_[1] : target_area_top_right;
                target_area_bottom_right = recorded_points_.size() == 3 ? recorded_points_[2] : target_area_bottom_right;
                target_area_bottom_left = recorded_points_.size() == 4 ? recorded_points_[3] : target_area_bottom_left;

                //小车到达放花盆的地点朝向角度
                if(recorded_points_.size() >= 2){
                    target_area_bearing = calculatePerpendicularBearing(target_area_top_left, target_area_top_right, target_area_bottom_right, target_area_bottom_left); 
                    RCLCPP_INFO(this->get_logger(), "放花朝向角度 = %.8f", target_area_bearing);
                }
                
                RCLCPP_INFO(this->get_logger(), "Have Recorded %d point", (int)recorded_points_.size());
                //重置标志位
                gps_flag = false;
            }
        } else {
            current_PosePoint_.point = Point2D::latlon_To_xy(lat, lon, Point2D::ref_lat, Point2D::ref_lon);
            if(wrm2 == 20) {//2秒打印一次当前位置
                RCLCPP_INFO(this->get_logger(), "current_PosePoint_.point = (%.2f, %.2f)", current_PosePoint_.point.x, current_PosePoint_.point.y);
                wrm2 = 0;
            }
            wrm2++;
            
        }
    } 
}

// 根据两点的直角坐标，计算垂直于这两点连线的方向角（用于确定摆放方向）
double DecisionMaking::calculatePerpendicularBearing(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3) {
    // 1. 你录的任意一条边（前两个点）
    double dx = p1.x - p0.x;
    double dy = p1.y - p0.y;

    // 2. 矩形中心（4个点平均）
    Point2D center;
    center.x = (p0.x + p1.x + p2.x + p3.x) / 4.0;
    center.y = (p0.y + p1.y + p2.y + p3.y) / 4.0;

    // 3. 边中点
    Point2D mid;
    mid.x = (p0.x + p1.x) / 2.0;
    mid.y = (p0.y + p1.y) / 2.0;

    // 4. 中点 → 中心（指向内部）
    double mx = center.x - mid.x;
    double my = center.y - mid.y;

    // 5. 叉乘判断往哪边转（自动适配所有方位）
    double cross = dx * my - dy * mx;

    double angle = atan2(dx, dy);
    double perp_angle;

    if (cross > 0) {//背离花田中心
        perp_angle = angle + M_PI/2;
    } else {
        perp_angle = angle - M_PI/2;
    }

    // 转成角度（-180° ~ +180° 范围，以Y轴为0°）
    double bearing = perp_angle * 180 / M_PI;
    RCLCPP_INFO(this->get_logger(), "转换前放花朝向角度 bearing= %.8f", bearing);

    // 2. 转换成 顺时针 0 ~ 360°
    bearing = fmod(bearing + 360.0, 360.0);

    // // 关键：把角度限制在 [-180, 180] 之间
    // bearing = fmod(bearing, 360);
    // if (bearing > 180) {
    //     bearing -= 360;
    // } else if (bearing < -180) {
    //     bearing += 360;
    // }
    return bearing;
}

// 划定目标区域
void DecisionMaking::recordCurrentPosition(const Point2D& point) {
    if (recorded_points_.size() >= required_points_) {
        RCLCPP_WARN(this->get_logger(), "已经记录 %d 个点，忽略新点", (int)recorded_points_.size());
        return;
    }
    
    // 先记录点
    recorded_points_.push_back(point);
    //last_record_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Record point %d/%d: (%.2f, %.2f)",
            (int)recorded_points_.size(), required_points_,
            point.x, point.y);

    // ✅ 录完4个点**立刻**计算矩阵
    if (recorded_points_.size() == required_points_) {
        RCLCPP_INFO(this->get_logger(), "开始生成花盆摆放矩阵...");
        calculatePotsMatrix(recorded_points_, pot_placement_spacing, pot_placement_type_);
    }
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
    Point2D y_dir((bottom_left.x - top_left.x) / height,      // 修改这里：从上到下
                  (bottom_left.y - top_left.y) / height);
    
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
                pot.x = top_left.x + x_pos * x_dir.x - y_pos * y_dir.x;  // 修改这里：从 top_left 开始
                pot.y = top_left.y + x_pos * x_dir.y - y_pos * y_dir.y;  // 修改这里：从 top_left 开始
                row_pots.push_back(pot);
                
            }
            target_pots_matrix_.push_back(row_pots);//花盆坐标矩阵，第一维是行，第二维是列
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
                pot.x = top_left.x + x_pos * x_dir.x - y_pos * y_dir.x;  // 修改这里：从 top_left 开始
                pot.y = top_left.y + x_pos * x_dir.y - y_pos * y_dir.y;  // 修改这里：从 top_left 开始
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

    // 🔥 打印所有花盆坐标 —— 直接加在函数最后！
    RCLCPP_INFO(this->get_logger(), "===== 所有花盆坐标 (x, y) 米 =====");
    int index = 0;
    for (size_t i = 0; i < target_pots_matrix_.size(); i++) {
        for (size_t j = 0; j < target_pots_matrix_[i].size(); j++) {
            double x = target_pots_matrix_[i][j].x;
            double y = target_pots_matrix_[i][j].y;
            RCLCPP_INFO(this->get_logger(), "花盆 [%d]:  x=%.2f, y=%.2f",
                      index++, x, y);
        }
    }
    RCLCPP_INFO(this->get_logger(), "======================================");
}

// 计算两点之间的距离 —— ROS2 标准版本
double DecisionMaking::distance(Point2D point1, Point2D point2) {
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

// 功能：将所有花盆点，沿着 target_area_bearing 方向【反向平移1米】，得到小车停靠点
// 输入：原始花盆点列表
// 输出：停靠点列表（小车要去的地方）
std::vector<Point2D> DecisionMaking::getParkingPointsFromPots(
    const std::vector<Point2D>& pot_points, 
    double target_bearing,    // 你的摆放角度 target_area_bearing
    double park_distance )     // 停靠距离：1.0 米
{
    std::vector<Point2D> parking_points;

    // 角度转弧度
    double rad = target_bearing * M_PI / 180.0;

    // 【核心】沿着角度反向移动 1 米（sin / cos 是正北顺时针标准公式）
    double dx = -park_distance * sin(rad);
    double dy = -park_distance * cos(rad);

    // 所有花盆点统一平移
    for (const auto& pot : pot_points) {
        Point2D park_point;
        park_point.x = pot.x + dx;
        park_point.y = pot.y + dy;
        parking_points.push_back(park_point);
    }

    return parking_points;
}

// 获取当前目标点 → 自动返回【平移1米后的停车点】
Point2D DecisionMaking::getCurrentTargetPoint(double park_distance) {
    //如果还没录点，target_pots_matrix_ 是空的，直接访问会崩溃
    if (target_pots_matrix_.empty()) {
        RCLCPP_WARN(this->get_logger(), "花盆矩阵为空！请先录点");
        return Point2D(0, 0);
    }

    if (current_target_index >= total_targets) {
        current_state = TaskState::TASK_COMPLETE;
        RCLCPP_INFO(this->get_logger(), "所有花盆已搬运完成！共 %d 盆", total_targets);
        return Point2D(0, 0);
    }
    
    int row = 0, col = 0, cumulative_count = 0;

    // 网格/三角摆放取点逻辑（完全不变）
    if (pot_placement_type_ == PlacementType::GRID_PLACEMENT) {
        int cols_per_row = target_pots_matrix_.empty() ? 0 : target_pots_matrix_[0].size();
        row = current_target_index / cols_per_row;
        col = current_target_index % cols_per_row;
    } 
    else if (pot_placement_type_ == PlacementType::TRIANGULAR_PLACEMENT) {
        for (size_t r = 0; r < target_pots_matrix_.size(); r++) {
            size_t cols_in_row = target_pots_matrix_[r].size();
            if (current_target_index < cumulative_count + cols_in_row) {
                row = r;
                col = current_target_index - cumulative_count;
                break;
            }
            cumulative_count += cols_in_row;
        }
    }

    // 越界检查
    if (row >= (int)target_pots_matrix_.size() || col >= (int)target_pots_matrix_[row].size()) {
        RCLCPP_ERROR(this->get_logger(), "索引越界！");
        return Point2D(0, 0);
    }

    // 1. 获取原始花盆点
    Point2D original_pot = target_pots_matrix_[row][col];

    // 2. ✅ 调用之前的平移函数（不白写！）
    std::vector<Point2D> park_points = getParkingPointsFromPots(
        {original_pot}, 
        target_area_bearing, 
        park_distance  // 后退1米
    );
    Point2D final_park_point = park_points[0];

    // 3. 返回已经平移好的停车点
    return final_park_point;
}

void DecisionMaking::startTransportCallback(
    const std::shared_ptr<decision_making_pkg::srv::StartTransport::Request> request,
    std::shared_ptr<decision_making_pkg::srv::StartTransport::Response> response)
{
    // 接收屏幕传来的参数
    int placement_type = request->placement_type;      // 0=网格, 1=三角
    float spacing = request->placement_spacing;        // 间距（米）
    bool start = request->start;                       // 启动/停止
    
    if (start) {
        // 保存参数
        pot_placement_type_ = (placement_type == 0) ? 
            PlacementType::GRID_PLACEMENT : PlacementType::TRIANGULAR_PLACEMENT;
        pot_placement_spacing = spacing;
        
        // 启动任务
        if (current_state == TaskState::WAITING_START) {
            current_state = TaskState::MOVING_TO_PICKUP;
            response->success = true;
            response->message = "任务已启动";
            RCLCPP_INFO(this->get_logger(), "✅ 任务启动 - 模式:%d, 间距:%.2fm", 
                        placement_type, spacing);
        } else {
            response->success = false;
            response->message = "系统已在运行中";
        }
    } else {
        // 停止任务
        current_state = TaskState::WAITING_START;
        response->success = true;
        response->message = "任务已停止";
        RCLCPP_INFO(this->get_logger(), "⏹️ 任务已停止");
    }
}

void DecisionMaking::voiceCmdCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string cmd = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到语音动作指令: %s", cmd.c_str());

    // 先清空标志
    bool voice_lift_pot_flag_ = false;
    bool voice_putdown_pot_flag_ = false;

    if (cmd == "lift_pot")
    {
        voice_lift_pot_flag_ = true;
        RCLCPP_INFO(this->get_logger(), "语音触发：抬起花盆指令");
    }
    else if (cmd == "putdown_pot")
    {
        voice_putdown_pot_flag_ = true;
        RCLCPP_INFO(this->get_logger(), "语音触发：放下花盆指令");
    }
    else if (cmd == "none")
    {
        // 无搬运指令，不处理
        return;
    }
}

// #########################################################################
// ######################## 【完整状态机代码】 ###########################
// #########################################################################

// 等待开始信号
void DecisionMaking::WAITING_START_State() {
    RCLCPP_INFO(this->get_logger(), "Waiting...");
    // 发布控制命令
    publishControl(0, 0, COMMAND_COMMON); //小车不动，机械臂等待
    //target_point_ = current_PosePoint_.point;
}

// 移动到取花点并取花
void DecisionMaking::MOVING_TO_PICKUP_State() {
    //RCLCPP_INFO(this->get_logger(), "MOVING_TO_PICKUP...");
    computeControl(); // 直接调用之前的控制函数，保持一致的控制逻辑
    // 2. ✅ 关键：抓取成功 → 去摆放点
    if (grasp_flag) {      // 下位机回来的抓取成功信号
        RCLCPP_INFO(this->get_logger(), "✅ 抓取完成，前往摆放点！");
        
        // 重置状态
        has_target_ = false;   
        grasp_flag = false; // 抓取完成后重置抓取状态，准备下一次抓取

        // 切换到去花盆摆放点
        current_state = TaskState::PICKING_UP;
    }
}

// 原地旋转，并且走到停车点
void DecisionMaking::PICKING_UP_State() {
    //RCLCPP_INFO(this->get_logger(), "PICKING_UP...");
    //小车要停下的位置
    Point2D target_point = getCurrentTargetPoint(1.0); // 后退1米的点

    // ==============================================
    // 🔥 每次都打印：当前要去的停车点（平移后的点）
    // ==============================================
    if(wrm1 == 0) {
        RCLCPP_INFO(this->get_logger(), "✅当前停车点：x=%.2f, y=%.2f",
         target_point.x, target_point.y);
         wrm1 = 1;
    }
    // // ==============================================
    // static bool printed_once = false;  // 静态变量：只初始化一次
    // if (!printed_once) {
    //     RCLCPP_INFO(this->get_logger(), "===== 所有花盆坐标 (x, y) 米 =====");
    //     int index = 0;
    //     for (size_t i = 0; i < target_point.size(); i++) {
    //         for (size_t j = 0; j < target_point[i].size(); j++) {
    //             double x = target_point[i][j].x;
    //             double y = target_point[i][j].y;
    //             RCLCPP_INFO(this->get_logger(), "花盆 [%d]:  x=%.2f, y=%.2f",
    //                       index++, x, y);
    //         }
    //     }
    //     RCLCPP_INFO(this->get_logger(), "======================================");
    //     printed_once = true;  // 只打印一次
    // }

    // 2. 计算当前位置到目标点的距离
    double dist_to_goal = distance(current_PosePoint_.point, target_point);
    back_pos_ = current_PosePoint_.point;//下一次的取花点
    // 计算小车到目标点的角度，注意 atan2 的参数顺序（dx, dy）是为了让 0° 对准正北，顺时针增加
    double dx = target_point.x - current_PosePoint_.point.x;
    double dy = target_point.y - current_PosePoint_.point.y;
    double angle_to_goal = atan2(dx, dy) * 180.0 / M_PI;

    if(wrm2 == 5)
    {
        RCLCPP_INFO(this->get_logger(), "距离停车点角度=%.2f", angle_to_goal);
        RCLCPP_INFO(this->get_logger(), "距离停车点距离=%.4f", dist_to_goal);
    }
    // 2. 转换成 顺时针 0 ~ 360°
    double angle_to_goal_360 = fmod(angle_to_goal + 360.0, 360.0);
    if(wrm2 == 5)
    {
        RCLCPP_INFO(this->get_logger(), "转换后距离停车点角度=%.2f", angle_to_goal_360);
    }
    //RCLCPP_INFO(this->get_logger(), "✅目标角度：%.2f cm", angle_to_goal_360);

    if (dist_to_goal < 0.05) {
        RCLCPP_INFO(this->get_logger(), "✅ 已到达停车点！距离：%.2f cm", dist_to_goal*100);
        publishControl(0, 0, COMMAND_COMMON); // 停车

        // 这里可以切换到摆放状态
        current_state = TaskState::MOVING_TO_PLACE;
        return;
    }

    pidTimeUpdate(); // 先更新PID的时间差
    // PID控制角度误差
        double error_yaw  = current_PosePoint_.yaw-angle_to_goal_360;  // ＋号表示逆时针转，－号表示顺时针
        // 角度归一化 -180 ~ 180
        if (error_yaw > 180)
            error_yaw -= 360;
        else if (error_yaw < -180)
            error_yaw += 360;//180度以上就减360，变成负数，表示反向转

        //RCLCPP_INFO(this->get_logger(), "✅角度误差：%.2f cm", error_yaw);

        double angle_pwm = angle_pid_rotate_.calculate(error_yaw);
        angle_pwm = std::clamp(angle_pwm, -180.0, 180.0);
        // 7. 定义 PWM 变量（必须写在 if 外面！）
        int16_t pwm_left, pwm_right;

    if(error_yaw > 10 || error_yaw < -10)// 角度误差大于5度才转，否则就不转了，直接走（不然有时候会一直抖动）
    {
        pwm_left = static_cast<int16_t>( -angle_pwm);
        pwm_right = static_cast<int16_t>( angle_pwm);  
        RCLCPP_INFO(this->get_logger(), "✅角度误差：正在旋转");
    } else {
        pwm_left = static_cast<int16_t>( 500 - angle_pwm);
        pwm_right = static_cast<int16_t>( 500 + angle_pwm);    
    }
    // 限制PWM范围（✅ 修复版）
        pwm_left = std::clamp(pwm_left, (int16_t)-1000, (int16_t)1000);
        pwm_right = std::clamp(pwm_right, (int16_t)-1000, (int16_t)1000);
    // 发布控制命令
        publishControl(pwm_left, pwm_right, COMMAND_COMMON); // 0x05 是普通控制命令，不抓取,不释放

}

//  原地旋转对准花盆，走向摆放点，放下花盆
void DecisionMaking::MOVING_TO_PLACE_State() {

    RCLCPP_INFO(this->get_logger(), "MOVING_TO_PLACE...");
    Point2D target_point = getCurrentTargetPoint(0.3); // 后退0.3米的点，作为最终目标点
    // 2. 计算当前位置到目标点的距离
    double dist_to_goal = distance(current_PosePoint_.point, target_point);

    if (dist_to_goal < 0.03) {
        RCLCPP_INFO(this->get_logger(), "✅ 已到达摆花点！距离：%.2f cm", dist_to_goal*100);
        publishControl(0, 0, COMMAND_COMMON); // 停车
        if(encoder_left_speed_ == 0 && encoder_right_speed_ == 0)
        {
            publishControl(0, 0, COMMAND_COMMON); // 停车并释放
            // 判断释放是否成功
            release_flag = true; // 这里直接假设释放成功了，实际可以等下位机发回来的信号再设置这个标志位################
            if (release_flag) {      // 下位机回来的释放成功信号
            RCLCPP_INFO(this->get_logger(), "✅ 释放完成");
        
            // 重置状态
            has_target_ = false;   
            release_flag = false; // 抓取完成后重置抓取状态，准备下一次抓取

            // 这里可以切换到摆放状态
            //current_state = TaskState::PLACING;
            }
        }
        return;
    }

    pidTimeUpdate(); // 先更新PID的时间差
    // 计算小车到目标点的角度
    double dx = target_point.x - current_PosePoint_.point.x;
    double dy = target_point.y - current_PosePoint_.point.y;
    double angle_to_goal = atan2(dx, dy) * 180.0 / M_PI;
    // 2. 转换成 顺时针 0 ~ 360°
    double angle_to_goal_360 = fmod(angle_to_goal + 360.0, 360.0);
    // PID控制角度误差
    double error_yaw  = current_PosePoint_.yaw-angle_to_goal_360;  // ＋号表示逆时针转，－号表示顺时针
    
    // 角度归一化 -180 ~ 180
    if (error_yaw > 180)
        error_yaw -= 360;
    else if (error_yaw < -180)
        error_yaw += 360;

    double angle_pwm = angle_pid_rotate_.calculate(error_yaw);
    double speed = speed_pid_.calculate(dist_to_goal);//根据距离误差计算速度

    speed = std::clamp(speed, -500.0, 500.0);
    angle_pwm = std::clamp(angle_pwm, -150.0, 150.0);
    // 7. 定义 PWM 变量（必须写在 if 外面！）
    int16_t pwm_left, pwm_right;

    if(error_yaw > 3 || error_yaw < -3)// 角度误差大于3度才转，否则就不转了，直接走（不然有时候会一直抖动）
    {
        pwm_left = static_cast<int16_t>( -angle_pwm);
        pwm_right = static_cast<int16_t>( angle_pwm);    
    } else {
        pwm_left = static_cast<int16_t>( speed - angle_pwm);
        pwm_right = static_cast<int16_t>( speed + angle_pwm);    
    }
    // 限制PWM范围（✅ 修复版）
        pwm_left = std::clamp(pwm_left, (int16_t)-1000, (int16_t)1000);
        pwm_right = std::clamp(pwm_right, (int16_t)-1000, (int16_t)1000);
    // 发布控制命令
        publishControl(pwm_left, pwm_right, COMMAND_COMMON); // 0x05 是普通控制命令，不抓取,不释放
}

// 后退至停车点
void DecisionMaking::PLACING_State() {
    //RCLCPP_INFO(this->get_logger(), "Placing flower pot…");

    Point2D target_point = getCurrentTargetPoint(1.0); // 后退1.0米的点，作为最终目标点
    // 2. 计算当前位置到目标点的距离
    double dist_to_goal = distance(current_PosePoint_.point, target_point);

    if (dist_to_goal < 0.05) {
        RCLCPP_INFO(this->get_logger(), "✅ 已到达摆花点！距离：%.2f cm", dist_to_goal*100);
        publishControl(0, 0, COMMAND_COMMON); // 停车
        if(encoder_left_speed_ == 0 && encoder_right_speed_ == 0)
        {
            // ✅ 摆完一个，索引+1，下一次摆下一盆
            current_target_index++;
            // 这里可以切换到摆放状态
            current_state = TaskState::TASK_COMPLETE;

        }
        return;
    }

    pidTimeUpdate(); // 先更新PID的时间差
    // 计算小车到目标点的角度
    double dx = target_point.x - current_PosePoint_.point.x;
    double dy = target_point.y - current_PosePoint_.point.y;
    double angle_to_goal = atan2(dx, dy) * 180.0 / M_PI;
    // 2. 转换成 顺时针 0 ~ 360°
    double angle_to_goal_360 = fmod(angle_to_goal + 360.0, 360.0);
    // PID控制角度误差
    double error_yaw  = current_PosePoint_.yaw-angle_to_goal_360;  // ＋号表示顺时针转，－号表示逆时针
    
    // 角度归一化 -180 ~ 180
    if (error_yaw > 180)
        error_yaw -= 360;
    else if (error_yaw < -180)
        error_yaw += 360;

    double angle_pwm = angle_pid_rotate_.calculate(error_yaw);
    double speed = speed_pid_.calculate(dist_to_goal);//根据距离误差计算速度

    speed = std::clamp(speed, -500.0, 500.0);
    angle_pwm = std::clamp(angle_pwm, -150.0, 150.0);
    // 7. 定义 PWM 变量（必须写在 if 外面！）
    int16_t pwm_left, pwm_right;

    if(error_yaw > 3 || error_yaw < -3)// 角度误差大于3度才转，否则就不转了，直接走（不然有时候会一直抖动）
    {
        pwm_left = static_cast<int16_t>( angle_pwm);
        pwm_right = static_cast<int16_t>( -angle_pwm);    
    } else {
        pwm_left = static_cast<int16_t>( -speed - angle_pwm);
        pwm_right = static_cast<int16_t>( -speed + angle_pwm);    
    }
    // 限制PWM范围（✅ 修复版）
        pwm_left = std::clamp(pwm_left, (int16_t)-1000, (int16_t)1000);
        pwm_right = std::clamp(pwm_right, (int16_t)-1000, (int16_t)1000);
    // 发布控制命令
        publishControl(pwm_left, pwm_right, COMMAND_COMMON); // 0x05 是普通控制命令，不抓取,不释放
}

// 原地转圈，走向返回点，继续取花盆
void DecisionMaking::TASK_COMPLETE_State() {
    
    // 2. 计算当前位置到目标点的距离
    double dist_to_goal = distance(current_PosePoint_.point, back_pos_);
    // 计算小车到目标点的角度
    double dx = back_pos_.x - current_PosePoint_.point.x;
    double dy = back_pos_.y - current_PosePoint_.point.y;
    double angle_to_goal = atan2(dx, dy) * 180.0 / M_PI;
    // 2. 转换成 顺时针 0 ~ 360°
    double angle_to_goal_360 = fmod(angle_to_goal + 360.0, 360.0);

    if (dist_to_goal < 0.05) {
        RCLCPP_INFO(this->get_logger(), "✅ 已到达取花点！距离：%.2f cm", dist_to_goal*100);
        publishControl(0, 0, COMMAND_COMMON); // 停车

        // 这里可以切换到摆放状态
        current_state = TaskState::MOVING_TO_PICKUP;
        return;
    }

    pidTimeUpdate(); // 先更新PID的时间差
    // PID控制角度误差
        double error_yaw  = current_PosePoint_.yaw-angle_to_goal_360;  // ＋号表示逆时针转，－号表示顺时针
        // 角度归一化 -180 ~ 180
        if (error_yaw > 180)
            error_yaw -= 360;
        else if (error_yaw < -180)
            error_yaw += 360;

        double angle_pwm = angle_pid_rotate_.calculate(error_yaw);
        angle_pwm = std::clamp(angle_pwm, -150.0, 150.0);
        // 7. 定义 PWM 变量（必须写在 if 外面！）
        int16_t pwm_left, pwm_right;

    if(error_yaw > 5 || error_yaw < -5)// 角度误差大于5度才转，否则就不转了，直接走（不然有时候会一直抖动）
    {
        pwm_left = static_cast<int16_t>( -angle_pwm);
        pwm_right = static_cast<int16_t>( angle_pwm);    
    } else {
        pwm_left = static_cast<int16_t>( 800 - angle_pwm);
        pwm_right = static_cast<int16_t>( 800 + angle_pwm);    
    }
    // 限制PWM范围（✅ 修复版）
        pwm_left = std::clamp(pwm_left, (int16_t)-1000, (int16_t)1000);
        pwm_right = std::clamp(pwm_right, (int16_t)-1000, (int16_t)1000);
    // 发布控制命令
        publishControl(pwm_left, pwm_right, COMMAND_COMMON); // 0x05 是普通控制命令，不抓取,不释放
}

// 主循环
void DecisionMaking::run() {
    rclcpp::Rate rate(Run_Hz);
    //测试用最开始状态位置在 MOVING_TO_PICKUP，这样一启动就去找第一个花盆，方便调试！正式运行时改回 WAITING_START 就行
    //current_state = TaskState::MOVING_TO_PICKUP;
    current_state = TaskState::WAITING_START;
     RCLCPP_INFO(this->get_logger(), "✅ 已开始循环！");

    while (rclcpp::ok()) {
        //rclcpp::spin(node);//死循环，只能在 main 里用！在类的成员函数里不能用！会卡死！所以改成 spin_some()，让它处理一次回调就继续往下走！这样就能在同一个循环里既处理回调又执行状态机逻辑了！
        rclcpp::spin_some(this->shared_from_this());

        switch (current_state) {
            case TaskState::WAITING_START:
                WAITING_START_State();
                break;
            case TaskState::MOVING_TO_PICKUP:
                MOVING_TO_PICKUP_State();
                break;
            case TaskState::PICKING_UP:
                PICKING_UP_State();
                break;
            case TaskState::MOVING_TO_PLACE:
                MOVING_TO_PLACE_State();
                break;
            case TaskState::PLACING:
                PLACING_State();
                break;
            case TaskState::TASK_COMPLETE:
                TASK_COMPLETE_State();
                break;
        }

        rate.sleep();
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionMaking>();
    node->run();//运行状态机，状态机也是死循环
    rclcpp::shutdown();
    return 0;
}