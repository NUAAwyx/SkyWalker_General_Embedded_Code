#include "drivers/motors/motor_dm/motor_dm.h"

#include <zephyr/drivers/can.h>


const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(dm_motor));

/* 台架初值：力矩 0。开机就让它转起来是没有意义的——第一阶段要验的是
 * 「使能握手能不能自愈」和「反馈帧解得对不对」，不是控制性能。
 * 想让轮子动，把这个数改成几 N·m（先离地、先给小的）。 */
static motor_setpoint desired = {
    .torque = 0.0f,
    .mode = MOTOR_MODE_TORQUE,
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
    uint32_t tick = 0;

    while (1)
    {
        /* 循环第一行 = 这一拍的入口：timer 每 1ms give 一次，这里 take 一次，
         * 循环体的节奏就锁在 1kHz。放到循环外就只认第一次节拍。 */
        k_sem_take(&motor_control_calculate_sem, K_FOREVER);

        motor_get(motor, &now_data);
        motor_set(motor, &desired);

        /* 每 500 拍（0.5s）打一行。串口是唯一的仪表：
         *   enabled=0 一直不动 → 使能帧没到电机（接线 / id / tx-id）
         *   enabled=1            → 握手成功，之后 torque 才是可信的
         * 有钳形表或 CAN 分析仪时这行可以删。 */
        if (++tick % 500U == 0U) {
            printk("en=%d pos=%d mrad vel=%d mrad/s cur=%d mNm t=%d C\n",
                   (int)now_data.enabled,
                   (int)(now_data.angle * 1000.0f),
                   (int)(now_data.omega * 1000.0f),
                   (int)(now_data.torque * 1000.0f),
                   (int)now_data.temperature);
        }
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
        /* can_pack 打的是当下的目标（未使能时是使能帧），必须在循环内重做，
         * 否则帧内容永远停在开机那一刻 */
        k_sem_take(&motor_can_send_sem, K_FOREVER);
        motor_can_pack(motor, frame.data);
        can_send(cfg->can_dev, &frame, K_MSEC(1), NULL, NULL);
    }
}

int main(){
    const motor_config *cfg = (const motor_config *)motor->config;

    k_timer_start(&motor_control_calculate_timer, K_NO_WAIT, K_MSEC(1));
    k_timer_start(&motor_can_send_timer, K_NO_WAIT, K_MSEC(1));

    can_start(cfg->can_dev);

    return 0;
}
