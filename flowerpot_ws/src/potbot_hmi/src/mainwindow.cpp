#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QString>
#include <QMessageBox>
#include <QButtonGroup>
#include <QScreen>
#include <QFont>

MainWindow::MainWindow(Ros2Interface* rosInterface, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      robotRunning(false),
      fullScreenMode(true),  // 初始为全屏模式
      passwordInput(""),
      correctPassword("123456"),  // 默认密码
      rosInterface(rosInterface)
{
    ui->setupUi(this);
    
    // 设置窗口标志
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);  // 无边框窗口
    
    // 初始化主界面显示
    ui->ReadyStatusValueLabel->setText("未就绪");
    ui->BatteryValueLabel->setText("92%");
    ui->YawAngleValueLabel->setText("0.0°");

    ui->LatitudeValueLabel->setText("0.000000");
    ui->LongitudeValueLabel->setText("0.000000");
    ui->PotCountValueLabel->setText("0");
    
    // 确保初始显示主页面（索引0）
    ui->stackedMainArea->setCurrentIndex(0);
    
    // 设置标签按钮组
    QButtonGroup *tabGroup = new QButtonGroup(this);
    tabGroup->addButton(ui->TabSystemInfo);
    tabGroup->addButton(ui->TabParams);
    tabGroup->addButton(ui->TabDeveloper);
    tabGroup->addButton(ui->TabLog);
    tabGroup->addButton(ui->TabFirmware);
    tabGroup->setExclusive(true);
    
    // 默认选中系统信息标签
    ui->TabSystemInfo->setChecked(true);
    
    // 初始化密码输入区域为隐藏
    ui->passwordDialogFrame->setVisible(false);
    
    // 设置版本信息
    ui->VersionValue->setText("v2.1.0 (Build 2026.04.19)");
    ui->AuthorValue->setText("Peng Defeng");
    ui->BuildValue->setText("Release (ARM64)");
    ui->PlatformValue->setText("Jetson Orin Nano 8GB");
    ui->OperatingSystemValue->setText("Linux22.04/ROS2");
    ui->SensorValue->setText("IMU/Encoder/Lidar");
    
    // 调整字体大小适应7寸屏
    adjustFontSizes();

    // ROS2刷新定时器
    ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, this, &MainWindow::updateRosData);
    ros_timer->start(100);  // 每100ms更新一次ROS2数据
}

MainWindow::~MainWindow()
{
    delete ui;
}

// ROS2数据更新
void MainWindow::updateRosData()
{
    if(!rosInterface) {
        qDebug() << "ROS2 Interface not initialized";
        return;
    }

    // 处理ROS2回调
    rclcpp::spin_some(rosInterface->get_node_base_interface());

    // 添加调试信息，查看是否收到数据
    // static int counter = 0;
    // if (counter++ % 10 == 0) {  // 每1秒打印一次（10 * 100ms）
    //     qDebug() << "Yaw:" << rosInterface->yaw 
    //              << "Lat:" << rosInterface->latitude 
    //              << "Lon:" << rosInterface->longitude;
    // }

    // 更新UI
    updateLatitude(rosInterface->latitude);
    updateLongitude(rosInterface->longitude);
    updateYawAngle(rosInterface->yaw);
    updateTransportedPotCount(rosInterface->transported_pot_count);
    updateSystemReadyStatus(rosInterface->system_ready_);
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    
    // 初始显示为全屏
    if (fullScreenMode) {
        showFullScreen();
    }
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    // 可以在窗口大小改变时调整字体
    adjustFontSizes();
}

void MainWindow::adjustFontSizes()
{
    // 7寸屏 (1024x600) 的字体调整
    QFont defaultFont = font();
    defaultFont.setFamily("Microsoft YaHei");
    defaultFont.setPointSize(10);  // 基础字体大小
    
    // 可以根据需要微调各个控件的字体
    QList<QLabel*> labels = findChildren<QLabel*>();
    for (QLabel* label : labels) {
        if (label->objectName().contains("ValueLabel")) {
            QFont font = label->font();
            font.setPointSize(12);
            font.setBold(true);
            label->setFont(font);
        }
    }
}

void MainWindow::exitFullScreen()
{
    if (fullScreenMode) {
        fullScreenMode = false;
        // 退出全屏，显示为正常窗口
        showNormal();
        
        // 设置固定大小以适应7寸屏
        resize(1024, 600);
        
        // 显示提示信息
        statusBar()->showMessage("已退出全屏模式", 3000);
        
        qDebug() << "Exited full screen mode";
    }
}

void MainWindow::showPasswordDialog()
{
    // 清空之前的输入
    passwordInput.clear();
    ui->passwordDisplay->setText("");
    
    // 显示密码对话框
    ui->passwordDialogFrame->setVisible(true);
    
    // 将密码对话框提升到最上层
    ui->passwordDialogFrame->raise();
    
    // 设置焦点到密码对话框
    ui->passwordDialogFrame->setFocus();
    
    qDebug() << "Password dialog shown";
}

void MainWindow::hidePasswordDialog()
{
    ui->passwordDialogFrame->setVisible(false);
}

// 数字键盘按钮
void MainWindow::on_num1_clicked() { passwordInput += "1"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num2_clicked() { passwordInput += "2"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num3_clicked() { passwordInput += "3"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num4_clicked() { passwordInput += "4"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num5_clicked() { passwordInput += "5"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num6_clicked() { passwordInput += "6"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num7_clicked() { passwordInput += "7"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num8_clicked() { passwordInput += "8"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num9_clicked() { passwordInput += "9"; ui->passwordDisplay->setText(passwordInput); }
void MainWindow::on_num0_clicked() { passwordInput += "0"; ui->passwordDisplay->setText(passwordInput); }

void MainWindow::on_clearBtn_clicked()
{
    passwordInput.clear();
    ui->passwordDisplay->setText("");
}

void MainWindow::on_backspaceBtn_clicked()
{
    if (!passwordInput.isEmpty()) {
        passwordInput.chop(1);
        ui->passwordDisplay->setText(passwordInput);
    }
}

void MainWindow::on_confirmPasswordBtn_clicked()
{
    if (passwordInput == correctPassword) {
        // 密码正确，退出全屏
        hidePasswordDialog();
        exitFullScreen();
        
        QMessageBox::information(this, "开发者模式", "开发者模式已激活，已退出全屏模式");
    } else {
        // 密码错误
        QMessageBox::warning(this, "密码错误", "开发者模式密码错误！");
        passwordInput.clear();
        ui->passwordDisplay->setText("");
    }
}

void MainWindow::on_cancelPasswordBtn_clicked()
{
    hidePasswordDialog();
}

// 开始运行按钮的槽函数
void MainWindow::on_StartLabel_clicked()
{
    robotRunning = !robotRunning;

    int current_placement_type; // 当前选择的摆放方式（0=网格，1=三角）
    double spacing; // 当前选择的摆放间距（厘米）

    if(robotRunning && rosInterface->system_ready_)
    {
        ui->StartLabel->setText("STOP");

        qDebug() << "Robot Started";

        // 发布ROS2启动命令
        if(rosInterface)
        {
            // 读取UI参数
            spacing = ui->PotSpacingValueLabel->value();
            QString layout = ui->PotLayoutValueLabel->currentText();
            current_placement_type = (layout == "网格摆放") ? 0 : 1;
            // 发送参数
            if (rosInterface->sendTransportTask(current_placement_type, 
                                                spacing/100.0,  // 转换为米
                                                true)) {
                RCLCPP_INFO(rosInterface->get_logger(), "任务已重新启动");
            } else {
                RCLCPP_ERROR(rosInterface->get_logger(), "任务启动失败");
            }
        }
    }
    else
    {
        ui->StartLabel->setText("START");

        qDebug() << "Robot Stopped";

        // 发布ROS2停止命令
        if(rosInterface)
            rosInterface->sendTransportTask(current_placement_type, 
                                                spacing/100.0,  // 转换为米
                                                false);
    }
}

// 系统设置按钮的槽函数 - 切换到设置页面
void MainWindow::on_SettingLabel_clicked()
{
    // 切换到设置页面（索引1）
    ui->stackedMainArea->setCurrentIndex(1);
    qDebug() << "Switched to Settings Page";
}

// 返回按钮的槽函数 - 从设置页面返回主页面
void MainWindow::on_CloseSettingsBtn_clicked()
{
    // 返回主页面（索引0）
    ui->stackedMainArea->setCurrentIndex(0);
    qDebug() << "Returned to Main Page";
}

// 开发者模式按钮的槽函数
void MainWindow::on_DevModeBtn_clicked()
{
    if (fullScreenMode) {
        // 全屏模式下，显示密码对话框
        showPasswordDialog();
    } else {
        // 非全屏模式下，直接显示开发者选项
        QMessageBox::information(
            this,
            "开发者模式",
            "开发者模式已启用\n\n"
            "调试功能：\n"
            "• 串口监视器\n"
            "• 传感器数据原始输出\n"
            "• 电机直接控制\n"
            "• 实时数据曲线\n"
            "• 系统日志查看"
        );
    }
}

// 堆叠窗口切换时的处理
void MainWindow::on_stackedMainArea_currentChanged(int index)
{
    if (index == 0) {
        qDebug() << "Main Page Active";
    } else if (index == 1) {
        qDebug() << "Settings Page Active";
    }
}

// 标签切换函数
void MainWindow::updateTabSelection(QPushButton* selectedTab)
{
    QString tabText = selectedTab->text();
    qDebug() << "Tab selected:" << tabText;
    
    // 可以根据不同标签更新右侧内容
    if (selectedTab == ui->TabSystemInfo) {
        // 显示系统信息
    } else if (selectedTab == ui->TabDeveloper) {
        // 显示开发者选项
        if (fullScreenMode) {
            // 如果在全屏模式下，提示需要先退出全屏
            QMessageBox::information(this, "提示", "开发者选项需要先退出全屏模式\n请在开发者按钮中输入密码");
        }
    }
}

void MainWindow::on_TabSystemInfo_clicked() { updateTabSelection(ui->TabSystemInfo); }
void MainWindow::on_TabParams_clicked() { updateTabSelection(ui->TabParams); }
void MainWindow::on_TabDeveloper_clicked() { updateTabSelection(ui->TabDeveloper); }
void MainWindow::on_TabLog_clicked() { updateTabSelection(ui->TabLog); }
void MainWindow::on_TabFirmware_clicked() { updateTabSelection(ui->TabFirmware); }

// 更新偏航角
void MainWindow::updateYawAngle(double angle)
{
    if (ui && ui->YawAngleValueLabel) {
        ui->YawAngleValueLabel->setText(QString::number(angle, 'f', 2) + "°");
    }
}

// 更新电池电量
void MainWindow::updateBattery(int value)
{
    if (ui && ui->BatteryValueLabel) {
        ui->BatteryValueLabel->setText(QString::number(value) + "%");
        
        // 根据电量改变颜色
        if (value < 20) {
            ui->BatteryValueLabel->setStyleSheet("color: #E74C3C; font-weight: bold;");
        } else if (value < 50) {
            ui->BatteryValueLabel->setStyleSheet("color: #F39C12; font-weight: bold;");
        } else {
            ui->BatteryValueLabel->setStyleSheet("color: #27AE60; font-weight: bold;");
        }
    }
}

// 更新纬度
void MainWindow::updateLatitude(double lat)
{
    if (ui && ui->LatitudeValueLabel) {
        ui->LatitudeValueLabel->setText(QString::number(lat, 'f', 6));
    }
}

// 更新经度
void MainWindow::updateLongitude(double lon)
{
    if (ui && ui->LongitudeValueLabel) {
        ui->LongitudeValueLabel->setText(QString::number(lon, 'f', 6));
    }
}

// 更新已搬运花盆数量
void MainWindow::updateTransportedPotCount(int count)
{
    if (ui && ui->PotCountValueLabel) {
        ui->PotCountValueLabel->setText(QString::number(count));
    }
}

// 更新系统准备状态
void MainWindow::updateSystemReadyStatus(bool ready)
{
    if (ui && ui->ReadyStatusValueLabel) {
        ui->ReadyStatusValueLabel->setText(ready ? "已就绪" : "未就绪");

        // 根据状态改变颜色
        if (ready) {
            ui->ReadyStatusValueLabel->setStyleSheet("color: #27AE60; font-weight: bold;");
        } else {
            ui->ReadyStatusValueLabel->setStyleSheet("color: #E74C3C; font-weight: bold;");
        }
    }
}

void MainWindow::setupDeveloperFeatures()
{
    // 开发者模式特有的功能
    qDebug() << "Developer features setup";
}