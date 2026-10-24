#include "radaryolo11_meld_node.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace radar_yolo11_fusion {

RadarYolo11MeldNode::RadarYolo11MeldNode(const rclcpp::NodeOptions& options)
    : Node("radaryolo11_meld_node", options),
      fusion_distance_threshold_(4.0),
      association_threshold_(0.6),//匹配距离阈值，单位米
      radar_min_distance_(0.3),
      radar_max_distance_(15.0),
      enable_debug_(true)
{
    // 声明参数
    declareParameters();
    
    // 初始化订阅者
    radar_sub_ = this->create_subscription<radar_msgs::msg::RadarTargetArray>(
        "radar_targets", 100,
        std::bind(&RadarYolo11MeldNode::radarCallback, this, std::placeholders::_1));
    
    yolo11_sub_ = this->create_subscription<yolo11_msgs::msg::CoordinateArray>(
        "yolo11_data", 100,
        std::bind(&RadarYolo11MeldNode::yolo11Callback, this, std::placeholders::_1));
    
    // 初始化发布者
    // 1. 主要发布者：发布融合后的目标数据
    fused_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("fused_targets", 10);
    // 2. 调试发布者：发布调试信息（话题名是 serial_data，但实际是调试数据）
    debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("debugs", 10);
    
    // 状态定时器
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        [this]() {
            RCLCPP_INFO(this->get_logger(), 
                "Status: Camera: %zu targets, Radar: %zu targets, Validated: %zu",
                camera_targets_.size(), radar_targets_.size(), validated_radar_targets_.size());
        });
    
    RCLCPP_INFO(this->get_logger(), "RadarYolo11MeldNode initialized");
}

RadarYolo11MeldNode::~RadarYolo11MeldNode() {
    RCLCPP_INFO(this->get_logger(), "RadarYolo11MeldNode shutting down");
}

void RadarYolo11MeldNode::declareParameters() {
    this->declare_parameter<double>("fusion_distance_threshold", 2.0);
    this->declare_parameter<double>("association_threshold", 0.6);
    this->declare_parameter<double>("radar_min_distance", 0.3);
    this->declare_parameter<double>("radar_max_distance", 15.0);
    this->declare_parameter<bool>("enable_debug", true);
    
    updateParameters();
}

void RadarYolo11MeldNode::updateParameters() {
    this->get_parameter("fusion_distance_threshold", fusion_distance_threshold_);
    this->get_parameter("association_threshold", association_threshold_);
    this->get_parameter("radar_min_distance", radar_min_distance_);
    this->get_parameter("radar_max_distance", radar_max_distance_);
    this->get_parameter("enable_debug", enable_debug_);
}

double RadarYolo11MeldNode::calculateDistance(const hungarian::Point& p1, const hungarian::Point& p2) {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

void RadarYolo11MeldNode::preprocessCameraData() {
    for (auto& target : camera_targets_) {
        target.updatePolar();
        
        if (enable_debug_) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Camera target: (%.2f, %.2f), dist=%.2f m, angle=%.1f deg",
                target.x, target.y, target.distance, target.angle_deg);
        }
    }
}

std::vector<TargetAssociation> RadarYolo11MeldNode::associateTargetsWithHungarian() {//返回值是容器，里面存储了关联结果，每个元素包含相机索引、雷达索引、距离、置信度等信息
    std::vector<TargetAssociation> associations;//空容器，存数据
    
    if (camera_targets_.empty() || radar_targets_.empty()) {
        return associations;//返回空容器
    }
    
    // 构建匈牙利匹配的点集
    std::vector<hungarian::Point> camera_points;
    std::vector<hungarian::Point> radar_points;
    
    for (const auto& cam : camera_targets_) {
        camera_points.emplace_back(cam.x, cam.y);
    }
    for (const auto& radar : radar_targets_) {
        radar_points.emplace_back(radar.x, radar.y);
    }
    
    // 计算距离矩阵
    auto distance_matrix = hungarian::computeDistanceMatrix(camera_points, radar_points);
    
    // 匈牙利算法求最优匹配
    std::vector<int> assignment = hungarian_solver_.solveMinimization(distance_matrix);
    
    // 构建关联结果
    for (size_t i = 0; i < assignment.size(); i++) {
        if (assignment[i] != -1) {
            double match_dist = distance_matrix[i][assignment[i]];
            // 计算置信度
            double conf = 1.0 - (match_dist / association_threshold_);
            
            if (match_dist <= association_threshold_ && conf >= 0.6) { // 只有距离足够近且置信度足够高才算有效关联
                TargetAssociation assoc;
                assoc.camera_idx = i;  // 相机第几个点
                assoc.radar_idx = assignment[i];// 雷达第几个点
                assoc.distance = match_dist;  // 它们之间的距离（米）
                assoc.confidence = 1.0 - (match_dist / association_threshold_);  // 置信度（越近越高）
                assoc.camera_point = camera_points[i];  // 相机坐标(x,y)
                assoc.radar_point = radar_points[assignment[i]];  // 雷达坐标(x,y)
                associations.push_back(assoc);
                
                if (enable_debug_) {
                    RCLCPP_DEBUG(this->get_logger(), 
                        "Hungarian match: camera[%zu] <-> radar[%d], dist=%.3f m, conf=%.2f",
                        i, assignment[i], match_dist, assoc.confidence);
                }
            }
        }
    }
    
    if (enable_debug_) {
        RCLCPP_INFO(this->get_logger(), "Hungarian: %zu valid associations", associations.size());
    }
    
    return associations;
}

std::vector<RadarTarget> RadarYolo11MeldNode::validateRadarPointsWithCamera() {
    std::vector<RadarTarget> validated;
    
    if (camera_targets_.empty()) {
        if (enable_debug_) {
            RCLCPP_WARN(this->get_logger(), "No camera data for radar validation");
        }
        return validated;
    }
    
    auto associations = associateTargetsWithHungarian();
    std::vector<bool> radar_validated(radar_targets_.size(), false);
    
    for (const auto& assoc : associations) {
        validated.push_back(radar_targets_[assoc.radar_idx]);
        radar_validated[assoc.radar_idx] = true;
        
        RCLCPP_INFO(this->get_logger(), 
            "Radar target %d validated as pot (match dist: %.3f m, conf: %.2f)",
            assoc.radar_idx, assoc.distance, assoc.confidence);
    }
    
    // 输出未验证的雷达点
    for (size_t i = 0; i < radar_targets_.size(); i++) {
        if (!radar_validated[i]) {
            RCLCPP_WARN(this->get_logger(), 
                "Radar target %zu (%.2f, %.2f) is NOT a pot (no camera match)",
                i, radar_targets_[i].x, radar_targets_[i].y);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Validated %zu/%zu radar points as pots",
               validated.size(), radar_targets_.size());
    
    return validated;
}

std::vector<FusedTarget> RadarYolo11MeldNode::fuseTargetData() {
    std::vector<FusedTarget> fused_targets;

    // 1. 近距离 ≤0.4m：全部用相机（你要的功能）
    // ===================== 【你要加的，就这一段】 =====================
    // 统一判断：所有相机目标是否 ≤0.4m
    bool all_camera_close = true;
    for (const auto& cam : camera_targets_) {
        if (cam.distance > 0.45) {
            all_camera_close = false;
            break;
        }
    }

    if (all_camera_close) {//情况1：<0.4m只用相机
        if (!camera_targets_.empty() ) {
        //RCLCPP_INFO(this->get_logger(), "Only camera data");
        for (const auto& cam : camera_targets_) {
            FusedTarget target;
            target.position = hungarian::Point(cam.x, cam.y);
            target.distance = cam.distance;
            target.angle = cam.angle;
            target.angle_deg = cam.angle_deg;
            target.source = DataSource::CAMERA_ONLY;
            target.confidence = 0.95;
            target.id = 0;
            fused_targets.push_back(target);
        }
        return fused_targets;
        }
    }else{ // 情况2：相机距离>0.4m，且两者都有
        if (!camera_targets_.empty() && !radar_targets_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Fusing %zu camera and %zu radar targets",
                    camera_targets_.size(), radar_targets_.size());
            
            // 用相机验证雷达
            validated_radar_targets_ = validateRadarPointsWithCamera();
            for (const auto& radar : validated_radar_targets_) {
            
            
            // // 对每个相机点进行融合决策
            // for (const auto& camera : camera_targets_) {
            //     double camera_dist = camera.distance;
                
                // // 寻找匹配的雷达点
                // RadarTarget matched_radar;
                // bool has_radar_match = false;
                // double min_match_dist = association_threshold_;
                // double match_confidence = 0.0;
                
                // for (const auto& radar : validated_radar_targets_) {
                //     double dist = calculateDistance(
                //         hungarian::Point(camera.x, camera.y),
                //         hungarian::Point(radar.x, radar.y));
                    
                //     if (dist < min_match_dist) {
                //         matched_radar = radar;
                //         has_radar_match = true;
                //         min_match_dist = dist;
                //         match_confidence = 1.0 - (dist / association_threshold_);
                //     }
                // }
                
                FusedTarget target;
                
                // if (camera_dist > fusion_distance_threshold_) {
                //     // 远距离：使用相机
                //     target.position = hungarian::Point(camera.x, camera.y);
                //     target.distance = camera.distance;
                //     target.angle = camera.angle;
                //     target.angle_deg = camera.angle_deg;
                //     target.source = DataSource::CAMERA_ONLY;
                //     target.confidence = 0.8;
                //     target.id = 0;
                    
                //     RCLCPP_DEBUG(this->get_logger(), 
                //         "Dist %.2f m > %.2f m: using camera", camera_dist, fusion_distance_threshold_);
                // } else if (has_radar_match) {
                //     // 近距离且有雷达验证：使用雷达
                //     target.position = hungarian::Point(matched_radar.x, matched_radar.y);
                //     target.distance = matched_radar.distance;
                //     target.angle = matched_radar.angle;
                //     target.angle_deg = matched_radar.angle_deg;
                //     target.source = DataSource::FUSED;
                //     target.confidence = match_confidence;
                //     target.id = matched_radar.id;
                    
                //     RCLCPP_INFO(this->get_logger(), 
                //         "Dist %.2f m <= %.2f m: using validated radar (match dist: %.3f m)",
                //         camera_dist, fusion_distance_threshold_, min_match_dist);
                // } 
                // else {
                //     // 近距离无雷达：使用相机（降置信度）
                //     target.position = hungarian::Point(camera.x, camera.y);
                //     target.distance = camera.distance;
                //     target.angle = camera.angle;
                //     target.angle_deg = camera.angle_deg;
                //     target.source = DataSource::CAMERA_ONLY;
                //     target.confidence = 0.5;
                //     target.id = 0;
                    
                //     RCLCPP_WARN(this->get_logger(), 
                //         "Dist %.2f m but no radar match: using camera (low confidence)", camera_dist);
                // }
                // ✅ 近距离：直接用【正确雷达点】
                if (radar.distance <= fusion_distance_threshold_) {
                    target.position = hungarian::Point(radar.x, radar.y);
                    target.distance = radar.distance;
                    target.angle = radar.angle;
                    target.angle_deg = radar.angle_deg;
                    target.source = DataSource::FUSED;
                    target.confidence = 0.95;
                    target.id = radar.id;

                    RCLCPP_INFO(this->get_logger(),
                        "✅ 使用正确花盆雷达点: (%.2f, %.2f) dist=%.2f m",
                        radar.x, radar.y, radar.distance);
                }
                // ✅ 远距离：用相机
                else {
                    // 找到这个雷达对应的相机点
                    const CameraTarget* matched_cam = nullptr;
                    double min_dist = 999;

                    for (const auto& cam : camera_targets_) {
                        double d = hypot(cam.x - radar.x, cam.y - radar.y);
                        if (d < min_dist) {
                            min_dist = d;
                            matched_cam = &cam;
                        }
                    }

                    if (matched_cam) {
                        target.position = hungarian::Point(matched_cam->x, matched_cam->y);
                        target.distance = matched_cam->distance;
                        target.angle = matched_cam->angle;
                        target.angle_deg = matched_cam->angle_deg;
                        target.source = DataSource::CAMERA_ONLY;
                        target.confidence = 0.8;
                        target.id = 0;
                    }
                }
                
                fused_targets.push_back(target);
            }
            
            // 添加额外的已验证雷达点（未被相机匹配的）
            std::vector<bool> radar_used(validated_radar_targets_.size(), false);
            for (const auto& target : fused_targets) {
                if (target.source == DataSource::FUSED) {
                    for (size_t i = 0; i < validated_radar_targets_.size(); i++) {
                        if (calculateDistance(target.position, 
                            hungarian::Point(validated_radar_targets_[i].x, validated_radar_targets_[i].y)) < 0.1) {
                            radar_used[i] = true;
                            break;
                        }
                    }
                }
            }
            
            for (size_t i = 0; i < validated_radar_targets_.size(); i++) {
                if (!radar_used[i]) {
                    FusedTarget target;
                    target.position = hungarian::Point(validated_radar_targets_[i].x, validated_radar_targets_[i].y);
                    target.distance = validated_radar_targets_[i].distance;
                    target.angle = validated_radar_targets_[i].angle;
                    target.angle_deg = validated_radar_targets_[i].angle_deg;
                    target.source = DataSource::RADAR_ONLY;
                    target.confidence = 0.6;
                    target.id = validated_radar_targets_[i].id;
                    fused_targets.push_back(target);
                    
                    RCLCPP_INFO(this->get_logger(), "Extra validated radar target added");
                }
            }
        }
    }
    
    // 按距离排序
    std::sort(fused_targets.begin(), fused_targets.end(),
              [](const FusedTarget& a, const FusedTarget& b) {
                  return a.distance < b.distance;
              });
    
    return fused_targets;
}

void RadarYolo11MeldNode::publishFusedTargets(const std::vector<FusedTarget>& targets) {

    // // 👇 不管空不空，都要构造消息并发布！！！
    // for (const auto& target : targets) {
    //     fused_msg->data.push_back(target.position.x);
    //     fused_msg->data.push_back(target.position.y);
    //     fused_msg->data.push_back(target.distance);
    //     fused_msg->data.push_back(target.angle);
    //     fused_msg->data.push_back(target.angle_deg);
    //     fused_msg->data.push_back(static_cast<int>(target.source));
    //     fused_msg->data.push_back(target.confidence);
    //     fused_msg->data.push_back(static_cast<double>(target.id));
    // }

    // // 👇 必须发布！空的也要发！！！
    // fused_target_pub_->publish(*fused_msg);
    
    auto fused_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    
    for (const auto& target : targets) {
        // 发布: x, y, distance, angle(rad), angle(deg), source, confidence, id
        fused_msg->data.push_back(target.position.x);
        fused_msg->data.push_back(target.position.y);
        fused_msg->data.push_back(target.distance);
        fused_msg->data.push_back(target.angle);
        fused_msg->data.push_back(target.angle_deg);
        fused_msg->data.push_back(static_cast<int>(target.source));
        fused_msg->data.push_back(target.confidence);
        fused_msg->data.push_back(static_cast<double>(target.id));
        
        const char* source_str = "UNKNOWN";
        if (target.source == DataSource::CAMERA_ONLY) source_str = "CAMERA";
        else if (target.source == DataSource::RADAR_ONLY) source_str = "RADAR";
        else if (target.source == DataSource::FUSED) source_str = "FUSED";
        
        RCLCPP_INFO(this->get_logger(), 
            "=== Fused: (%.2f, %.2f), dist=%.2f m, angle=%.1f deg, src=%s, conf=%.2f, id=%u ===",
            target.position.x, target.position.y, target.distance, target.angle_deg, 
            source_str, target.confidence, target.id);
    }
    
    fused_target_pub_->publish(*fused_msg);

    // // 只有日志可以判断，发布不能省！
    // if (targets.empty()) {
    //     RCLCPP_WARN(this->get_logger(), "No fused targets to publish");
    //     return;
    // }//这段代码导致如果没有目标就不发布消息了，导致决策层一直拿不到消息，一直是上一次的旧数据
    
    // 调试信息
    if (enable_debug_) {
        auto debug_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        debug_msg->data.push_back(camera_targets_.size());
        debug_msg->data.push_back(radar_targets_.size());
        debug_msg->data.push_back(validated_radar_targets_.size());
        debug_msg->data.push_back(targets.size());
        debug_msg->data.push_back(fusion_distance_threshold_);
        debug_msg->data.push_back(association_threshold_);
        debug_pub_->publish(*debug_msg);
    }
}

hungarian::Point RadarYolo11MeldNode::selectClosestTarget(const std::vector<FusedTarget>& targets) {
    if (targets.empty()) {
        return hungarian::Point(0, 0);
    }
    
    auto closest = targets[0];
    for (const auto& target : targets) {
        if (target.distance < closest.distance) {
            closest = target;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Closest target: (%.2f, %.2f), dist=%.2f m, angle=%.1f deg",
               closest.position.x, closest.position.y, closest.distance, closest.angle_deg);
    
    return closest.position;
}

void RadarYolo11MeldNode::radarCallback(const radar_msgs::msg::RadarTargetArray::SharedPtr msg) {
    radar_targets_.clear();
    
    for (const auto& radar_data : msg->array) {
        RadarTarget target;
        target.id = radar_data.n;        // n字段作为ID
        target.r = radar_data.r;         // 极径
        target.phi = radar_data.phi;     // 极角
        target.x = radar_data.x;         // X坐标
        target.y = radar_data.y;         // Y坐标
        target.radius = radar_data.radius; // 半径

        // 【关键】调用 updatePolar() 来设置 distance, angle, angle_deg
        target.updatePolar();
        
        radar_targets_.push_back(target);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Raw radar: id=%u, r=%.2f, phi=%.2f rad, (%.2f, %.2f), radius=%.2f",
            target.id, target.r, target.phi, target.x, target.y, target.radius);
    }
    
    //preprocessRadarData();
    RCLCPP_INFO(this->get_logger(), "Radar: %zu targets after preprocessing", radar_targets_.size());
    
    // 执行融合，要是有相机用相机有雷达用雷达，就把判断去掉
    // if (!camera_targets_.empty()) {
    //     auto fused = fuseTargetData();
    //     publishFusedTargets(fused);
    // }
    auto fused = fuseTargetData();
        publishFusedTargets(fused);
}

void RadarYolo11MeldNode::yolo11Callback(const yolo11_msgs::msg::CoordinateArray::SharedPtr msg) {
    camera_targets_.clear();
    
    for (const auto& coord : msg->array) {
        CameraTarget target;
        // // 关键修改：将厘米转换为米（除以100）
        // float x_in_meters = coord.x / 100.0;   // 厘米 -> 米
        // float y_in_meters = coord.y / 100.0;   // 厘米 -> 米
        target.x = coord.x;
        target.y = coord.y;
        camera_targets_.push_back(target);
        
        RCLCPP_DEBUG(this->get_logger(), "Raw camera: (%.2f, %.2f)", target.x, target.y);
    }
    
    preprocessCameraData();
    RCLCPP_INFO(this->get_logger(), "Camera: %zu targets", camera_targets_.size());
    
    // 如果有雷达数据，执行融合
    // if (!radar_targets_.empty()) {
    //     auto fused = fuseTargetData();
    //     publishFusedTargets(fused);
    // }
    auto fused = fuseTargetData();
        publishFusedTargets(fused);
}

} // namespace radar_yolo11_fusion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_yolo11_fusion::RadarYolo11MeldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 