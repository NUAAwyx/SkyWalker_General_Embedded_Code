#ifndef MOTOR_DM_H
#define MOTOR_DM_H

#include "drivers/motors/motor.h"

#include <zephyr/spinlock.h>

/* ⚠️ 顺序即契约：DT_INST_ENUM_IDX(inst, type) 返回的是 binding 里 enum 的
 *    排列下标，直接当本枚举用。改这里或改 binding 的 enum 顺序，
 *    任何一边单独动都会让电机按错的协议发帧，且编译期不报错。 */
typedef enum motor_dm_type {
    DM_TYPE_MIT = 0,
    DM_TYPE_POSITION_AND_SPEED,
    DM_TYPE_SPEED_ONLY,
} motor_dm_type;

typedef struct {
    motor_config base;   /**< 公共电机配置，必须作为第一个成员 */
    motor_dm_type type;  /**< DM 电机协议 */
} motor_dm_config;

/** @brief DM 反馈帧解码出的原始整数量。
 *
 * 三个物理量都是**无符号线性映射**：x = x_min + u/(2^N - 1) * (x_max - x_min)，
 * 刻意不在这里转成带符号数——位置 16 位、速度/扭矩 12 位，
 * 转 int16_t 会让 u >= 32768 的位置变成负数，再喂给 uint_to_float 就得到错的值。 */
typedef struct {
    uint8_t id;                /**< D[0] 低 4 位：电机 ID */
    uint8_t error_code;        /**< D[0] 高 4 位：状态码，bit0 = 使能 */
    uint16_t position;         /**< D[1..2]：16 位位置原始值 */
    uint16_t velocity;         /**< D[3] + D[4]高4位：12 位速度原始值 */
    uint16_t torque;           /**< D[4]低4位 + D[5]：12 位扭矩原始值 */
    uint8_t mos_temperature;   /**< D[6]：MOS 平均温度 (℃) */
    uint8_t rotor_temperature; /**< D[7]：线圈平均温度 (℃) */
} motor_dm_raw_data;

typedef struct {
    motor_data base_data;                  /**< 公共电机运行时数据，必须作为第一个成员 */
    motor_setpoint target;                 /**< 本机的目标设定点（上层 -> 电机） */
    motor_dm_raw_data raw_data;            /**< 本机的原始反馈数据（CAN 帧解码后） */
    struct k_spinlock lock;                /**< 保护以上字段：CAN 中断写、线程读 */
} motor_dm_data;

#endif /* MOTOR_DM_H */
