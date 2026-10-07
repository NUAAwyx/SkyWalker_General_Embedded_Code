#ifndef PID_H
#define PID_H

#include <zephyr/device.h>

/** @brief PID 配置参数（只读，ROM） */
typedef struct {
    float Kp, Ki, Kd;   // 增益
    float max_i_out;     // I 项输出限幅
    float max_out;       // 总输出限幅
    float deadband;      // 死区
} pid_config;

/** @brief PID 运行时状态（读写，RAM） */
typedef struct {
    float last_error;           // 上一拍误差（D 项用）
    float p_out, i_out, d_out;  // 各项输出
    float output;               // 最终输出
} pid_data;

/** @brief PID 计算，返回控制量 */
float pid_update(pid_data *data,
                 const pid_config *config,
                 float setpoint, float measurement, float dt,
                 float feedforward);

/**
 * @brief 从 PID 设备取只读增益
 *
 * skywalker_pid 没有 motor_api 那样的 api 结构体 —— 它提供的是【参数】不是
 * 【服务】。所以调用方（chassis / gimbal）自己持有 pid_data（每路一份），
 * 只从这里借 config（一份就够，多路共用）。
 *
 * 拿 dev->config 这件事只在这一个地方发生，dev 的布局知识不外泄。
 * Zephyr 自己也这么做，见 zephyr/drivers/w1.h:290、regulator.h:330。
 */
static inline const pid_config *pid_get_config(const struct device *dev) {
    return (const pid_config *)dev->config;
}

#endif // PID_H
