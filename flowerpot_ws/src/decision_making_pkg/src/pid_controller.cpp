#include "pid_controller.hpp"
#include <algorithm>

namespace control {

PIDController::PIDController(const PIDGains& gains, double dt,
                             double output_limit, double integral_limit)
    : gains_(gains)
    , dt_(dt)
    , output_limit_(output_limit)
    , integral_limit_(integral_limit)
{
    reset();
}

double PIDController::calculate(double error) {
    // 比例项
    double p_term = gains_.kp * error;
    
    // 积分项（带限幅）
    state_.integral += error * dt_;//误差对时间积分
    state_.integral = std::clamp(state_.integral, -integral_limit_, integral_limit_);//积分限幅 防止积分过大导致系统不稳定
    double i_term = gains_.ki * state_.integral;
    
    // 微分项
    double d_term = gains_.kd * (error - state_.prev_error) / dt_;//误差的微分项，计算误差变化率
    
    // 保存误差
    state_.prev_error = error;
    
    // 计算输出
    state_.output = p_term + i_term + d_term;
    
    // 输出限幅
    state_.output = std::clamp(state_.output, -output_limit_, output_limit_);
    
    return state_.output;
}

void PIDController::reset() {
    state_.integral = 0.0;
    state_.prev_error = 0.0;
    state_.output = 0.0;
}

void PIDController::setGains(const PIDGains& gains) {
    gains_ = gains;
    reset();
}

PIDState PIDController::getState() const {
    return state_;
}

void PIDController::setOutputLimit(double limit) {
    output_limit_ = std::abs(limit);
}

} // namespace control