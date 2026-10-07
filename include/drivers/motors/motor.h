#ifndef MOTOR_H
#define MOTOR_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

/* ─── 公共枚举 ─── */

/** @brief 电机控制模式：决定设定点里哪个字段生效 */
enum motor_mode {
	MOTOR_MODE_POSITION = 0, /**< 位置（角度）*/
	MOTOR_MODE_SPEED,        /**< 速度（rad/s） */
	MOTOR_MODE_TORQUE,       /**< 扭矩 (Nm)*/
};

/* ─── 公共数据结构 ─── */

/** @brief 目标设定点（上层 -> 电机） */
typedef struct {
	float angle;          /**< 目标角度 (rad) */
	float omega;          /**< 目标转速 (rad/s) */
	float torque;         /**< 目标扭矩 */
	enum motor_mode mode; /**< 哪个字段生效 */
} motor_setpoint;

/**
 * @brief 公共配置（ROM，由 DT 填充；具体电机嵌入并扩展）
 */
typedef struct {
	const struct device *can_dev;              		  /**< CAN 控制器设备（phandle） */
	uint8_t id;                                		  /**< 电机 ID */
	uint32_t tx_id;                            		  /**< CAN 发送报文 ID */
	uint32_t rx_id;                            		  /**< CAN 接收报文 ID */
} motor_config;

/**
 * @brief 公共运行时数据（RAM；具体电机嵌入并扩展）
 */
typedef struct {
	bool enabled;      /**< 是否已使能 */
	float angle;       /**< 当前角度（rad） */
	float omega;       /**< 当前角速度（rad/s，【输出轴】） */

	/** 转子角速度（rad/s）。
	 *  ★ 和 omega 是【同一个量】，只差一个 gear_ratio（3508 是 19.2 倍）。
	 *    之所以两个都要，是因为功率模型里的 ω 和 I 必须取自【同一根轴】：
	 *    P = k1·I·ω + k2·I² + k3·ω² + k4
	 *    转子电流配输出轴转速，铜损那一项会差一个 G。 */
	float omega_rotor;

	float torque;      /**< 当前扭矩（N·m，【输出轴】） */

	/** 转子电流（A，实测，取自反馈帧）。
	 *  功率模型的自变量是【它】，不是 torque：torque 是输出轴的，而且本来
	 *  就是从这同一个电流值换算出来的 —— 拿 torque 反推电流是绕一圈回到原地，
	 *  中间还多乘除了一次 gear_ratio。 */
	float current;

	float temperature; /**< 当前温度（°C） */
} motor_data;

/** @brief 电机驱动接口：具体电机实现这三个回调并填入 DEVICE_DT_DEFINE 的 api */
typedef struct {
	int (*can_pack)(const struct device *dev, uint8_t *out);
	int (*set)(const struct device *dev, const motor_setpoint *sp);
	int (*get)(const struct device *dev, motor_data *out);
} motor_api;

/* ─── 分发函数（上层统一入口） ─── */

/**
 * @brief CAN 帧打包
 * @param dev 电机设备指针
 * @param out 输出缓冲区
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int motor_can_pack(const struct device *dev, uint8_t *out){
	const motor_api *api = (const motor_api *)dev->api;

	if (api == NULL || api->can_pack == NULL) {
		return -ENOSYS;
	}
	return api->can_pack(dev, out);
}

/**
 * @brief 设置电机目标
 * @param dev 电机设备指针
 * @param sp  目标设定点
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int motor_set(const struct device *dev, const motor_setpoint *sp){
	const motor_api *api = (const motor_api *)dev->api;

	if (api == NULL || api->set == NULL) {
		return -ENOSYS;
	}
	return api->set(dev, sp);
}

/**
 * @brief 读取电机当前状态
 * @param dev 电机设备指针
 * @param out 输出：当前状态
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int motor_get(const struct device *dev, motor_data *out){
	const motor_api *api = (const motor_api *)dev->api;

	if (api == NULL || api->get == NULL) {
		return -ENOSYS;
	}
	return api->get(dev, out);
}

#define MOTOR_DT_CONFIG_INST_INIT(inst)                                           			\
	{                                                                             			\
		.can_dev 		   = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_dev)),                 \
		.id 	 	       = DT_INST_PROP(inst, id),                                        \
		.tx_id 			   = DT_INST_PROP_OR(inst, tx_id, 0),                               \
		.rx_id 			   = DT_INST_PROP_OR(inst, rx_id, 0),                               \
	}

#define MOTOR_DT_DATA_INST_INIT(inst)                  \
	{                                                  \
		.enabled = false,                              \
		.angle = 0.0f, 	 							   \
		.omega = 0.0f,                                 \
		.omega_rotor = 0.0f,                           \
		.torque = 0.0f,								   \
		.current = 0.0f,                               \
		.temperature = 0.0f,                           \
	}                                                  \

#endif /* MOTOR_H */
