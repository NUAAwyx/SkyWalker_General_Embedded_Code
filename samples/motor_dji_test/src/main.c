#include "drivers/motors/motor_dji/motor_dji.h"
#include "drivers/pid/pid.h"
#include <zephyr/drivers/can.h>

#include "lib/dr16/dr16.h"


const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(m3508_motor));
const struct device *pid = DEVICE_DT_GET(DT_NODELABEL(speed_pid));

static motor_setpoint desired = {
    .omega = 2.0f,
    .mode = MOTOR_MODE_SPEED,
};

void motor_control_calculate_thread_entry(void *arg1, void *arg2, void *arg3);
void motor_can_send_thread_entry(void *arg1, void *arg2, void *arg3);

void motor_control_calculate_timer_callback(struct k_timer *timer_id);
void motor_can_send_timer_callback(struct k_timer *timer_id);


K_THREAD_DEFINE(motor_control_calculate_thread, 2048, motor_control_calculate_thread_entry, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(motor_can_send_thread, 2048, motor_can_send_thread_entry, NULL, NULL, NULL, 1, 0, 0);

K_TIMER_DEFINE(motor_control_calculate_timer, motor_control_calculate_timer_callback, NULL);
K_TIMER_DEFINE(motor_can_send_timer, motor_can_send_timer_callback, NULL);

K_SEM_DEFINE(motor_control_calculate_sem, 0, 1);
K_SEM_DEFINE(motor_can_send_sem, 0, 1);

void motor_control_calculate_timer_callback(struct k_timer *timer_id){
    k_sem_give(&motor_control_calculate_sem);
}

void motor_can_send_timer_callback(struct k_timer *timer_id){
    k_sem_give(&motor_can_send_sem);
}

void motor_control_calculate_thread_entry(void *arg1, void *arg2, void *arg3){

    motor_data now_data;
    motor_setpoint target;

    pid_data *pid_state = pid->data;
    const pid_config *pid_cfg = pid->config;

    float target_value;
    float now;

    while (1)
    {
        /* 循环第一行 = 这一拍的入口：timer 每 1ms give 一次，这里 take 一次，
         * 循环体的节奏就锁在 1kHz。放到循环外就只认第一次节拍。 */
        k_sem_take(&motor_control_calculate_sem, K_FOREVER);

        switch (desired.mode)
        {
        case MOTOR_MODE_POSITION:
            target_value = desired.angle;
            break;

        case MOTOR_MODE_SPEED:
            target_value = desired.omega;
            break;

        case MOTOR_MODE_TORQUE:
            target_value = desired.torque;
            break;

        default:
            break;
        }

        /* 取反馈 -> 算 PID -> 写目标，三件事每拍都要重做一遍 */
        motor_get(motor, &now_data);
        now = now_data.omega;
        pid_update(pid_state, pid_cfg, target_value, now, 0.001f, 0.0f);

        target = (motor_setpoint){
            .torque = pid_state->output,
            .mode = MOTOR_MODE_TORQUE,
        };

        motor_set(motor, &target);
    }
}

void motor_can_send_thread_entry(void *arg1, void *arg2, void *arg3){

    const motor_config *cfg = (const motor_config *)motor->config;

    struct can_frame frame = {
        .id = cfg->tx_id,
        .dlc = 8,
    };

    while (1)
    {
        /* can_pack 打的是当下的 target.torque，必须在循环内重做，
         * 否则帧内容永远停在开机那一刻 */
        k_sem_take(&motor_can_send_sem, K_FOREVER);
        motor_can_pack(motor, frame.data);
        can_send(cfg->can_dev, &frame, K_MSEC(1), NULL, NULL);
    }
}

int main(){
    const motor_config *cfg = (const motor_config *)motor->config;

    dr16_init(DEVICE_DT_GET(DT_NODELABEL(uart5)));

    k_timer_start(&motor_control_calculate_timer, K_NO_WAIT, K_MSEC(1));
    k_timer_start(&motor_can_send_timer, K_NO_WAIT, K_MSEC(1));

    can_start(cfg->can_dev);

    return 0;
}
