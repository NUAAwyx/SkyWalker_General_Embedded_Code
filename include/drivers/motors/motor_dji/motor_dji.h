#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include <stdint.h>

#include <zephyr/spinlock.h>

#include <drivers/motors/motor.h>

/** @brief 支持的 DJI 电机型号 */
typedef enum motor_dji_type {
	MOTOR_DJI_M2006 = 0, /**< M2006 电机 */
	MOTOR_DJI_M3508,     /**< M3508 电机 */
	MOTOR_DJI_GM6020,    /**< GM6020 电机 */
} motor_dji_type;

/** @brief DJI 电机的静态配置。首成员必须是 motor_config（基类接口依赖）。 */
typedef struct {
	motor_config base;   /**< 公共电机配置，必须作为第一个成员 */
	motor_dji_type type; /**< DJI 电机型号 */
	float gear_ratio;    /**< 减速比（转子:输出） */
	float angle_offset;  /**< 软件零点 (rad)，仅 GM6020，算完角度再减 */
} motor_dji_config;

/** @brief DJI 电机的原始反馈数据 */
typedef struct {
	uint16_t encoder;
	int16_t rpm;
	int16_t current;
	int32_t temperature;
} motor_dji_raw_data;

/** @brief DJI 电机的运行时数据 */
typedef struct {
	motor_data base_data;                  /**< 公共电机运行时数据，必须作为第一个成员 */
	motor_setpoint target;                 /**< 本机的目标设定点（上层 -> 电机） */
	motor_dji_raw_data raw_data;           /**< 本机的原始反馈数据（CAN 帧解码后） */
	uint16_t prev_encoder;                 /**< 上一帧原始编码器值，用于处理单圈回绕 */
	int32_t angle_counts;                  /**< 转子侧计数累加，仅增量型电机使用，见 process_feedback */
	struct k_spinlock lock;                /**< 保护以上字段：CAN 中断写、线程读 */
} motor_dji_data;

#endif /* MOTOR_DJI_H */
