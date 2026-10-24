#include "mainwindow.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QSurfaceFormat>
#include "ros_interface.hpp"

int main(int argc, char *argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    QApplication app(argc, argv);
    
    // 设置应用程序信息
    QApplication::setApplicationName("PotBot HMI");
    QApplication::setApplicationVersion("2.1.0");
    QApplication::setOrganizationName("Peng Defeng");

    // 创建ROS2接口对象（继承自Node，需要共享指针管理）
    auto rosInterface = std::make_shared<Ros2Interface>();
    
    // 创建主窗口，传递ROS2接口指针
    MainWindow w(rosInterface.get());
    
    // 显示主窗口（会自动全屏）
    w.show();
    
    // 创建单独的线程用于处理ROS2 spinning
    // 使用QTimer方式在mainwindow中已经处理，这里只需要保持节点存活
    
    int result = app.exec();
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return result;
}