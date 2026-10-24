#ifndef POINT_H
#define POINT_H

#include <iostream>
//#include "rclcpp/rclcpp.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Point2D {//这个类只是个点
    double x, y;      // 转换后的平面坐标

    // 静态成员声明（需要在结构体外定义）
    static bool ref_initialized;
    static double ref_lat;
    static double ref_lon;

    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}//不传入是0，传入是参数

    static Point2D latlon_To_xy(double lat, double lon, double ref_lat, double ref_lon) {
        const double R = 6371000.0; // 地球半径（米）
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        double ref_lat_rad = ref_lat * M_PI / 180.0;
        double ref_lon_rad = ref_lon * M_PI / 180.0;

        // 使用平均纬度提高精度
        double avg_lat_rad = (lat_rad + ref_lat_rad) / 2.0;
        
        double x = R * (lon_rad - ref_lon_rad) * cos(avg_lat_rad);
        double y = R * (lat_rad - ref_lat_rad);
        
        return Point2D(x, y);
    }
};

struct Point {
    double latitude;
    double longitude;
    Point(double lat = 0, double lon = 0) : latitude(lat), longitude(lon) {}
};

// 位姿点（包含位置和航向）
struct PosePoint {
    Point2D point;
    double yaw;      // 航向角
    PosePoint(double x = 0, double y = 0, double yaw_ = 0) : point(x, y), yaw(yaw_) {}
};

#endif // POINT_H