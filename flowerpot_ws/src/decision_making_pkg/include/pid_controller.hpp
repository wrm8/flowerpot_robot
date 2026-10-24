#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include "point.hpp"

namespace control {

// PID参数结构体
struct PIDGains {
    double kp;  // 比例增益
    double ki;  // 积分增益，静态误差（静差），比如目标速度 1m/sP 项只能让车跑到 0.9m/s 就不动了，这时候 I 项会慢慢累加，让输出越来越大直到车真的跑到 1m/s，误差 = 0，积分才停止增长
    double kd;  // 微分增益
};

// PID状态结构体
struct PIDState {
    double integral;      // 积分项
    double prev_error;    // 上次误差
    double output;        // 输出值
};

class PIDController {
public:
    /**
     * @brief 构造函数
     * @param gains PID增益参数
     * @param dt 控制周期(秒)
     * @param output_limit 输出限幅值
     * @param integral_limit 积分限幅值
     */
    PIDController(const PIDGains& gains, double dt, 
                  double output_limit = 1.0, double integral_limit = 100.0);
    
    /**
     * @brief 计算PID输出
     * @param error 当前误差
     * @return 控制量
     */
    double calculate(double error);
    
    /**
     * @brief 重置状态
     */
    void reset();
    
    /**
     * @brief 更新PID参数
     */
    void setGains(const PIDGains& gains);
    
    /**
     * @brief 获取当前状态
     */
    PIDState getState() const;
    
    /**
     * @brief 设置输出限幅
     */
    void setOutputLimit(double limit);

private:
    PIDGains gains_;           // PID参数
    PIDState state_;           // PID状态
    double dt_;                // 控制周期，dt_：时间步长（比如每 0.01 秒算一次 PID）
    double output_limit_;      // 输出限幅
    double integral_limit_;    // 积分限幅
};

} // namespace control

#endif // PID_CONTROLLER_HPP_