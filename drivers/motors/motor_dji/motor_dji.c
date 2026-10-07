#include "drivers/motors/motor_dji/motor_dji.h"

#include <zephyr/drivers/can.h>

#include <math.h>

#define DT_DRV_COMPAT skywalker_motor_dji

#define MOTOR_DJI_ENCODER_COUNTS 8192

/* 不用 <math.h> 的 M_PI：picolibc 中受 __BSD_VISIBLE 保护，-std=c17 下不可见 */
#define MOTOR_DJI_PI 3.14159265359f

#define MOTOR_DJI_TWO_PI (2.0f * MOTOR_DJI_PI)

#define MOTOR_DJI_DEG_TO_RAD 0.017453292519943295f

/* DTS 里按度写（人可读），驱动内部用 rad。 */
#define MOTOR_DJI_DT_ANGLE_RAD(inst, prop)                       \
	((float)DT_STRING_UNQUOTED(DT_DRV_INST(inst), prop) *        \
	 MOTOR_DJI_DEG_TO_RAD)

/* 电流原始值 → 转子侧扭矩系数 (N·m/LSB)，按型号索引。
 * 系数 = 满量程电流 / 16384 × Kt；再乘 gear_ratio 得输出轴扭矩。
 * 取值参考 mambo 的 dji_ratios.h，实车验证过。 */
static const float dji_current2torque[] = {
	[MOTOR_DJI_M2006]  = 0.0000089606f,
	[MOTOR_DJI_M3508]  = 0.00002200f,
	[MOTOR_DJI_GM6020] = 0.00010986f,
};

/* 反馈帧里电流字段每 1 LSB 对应多少安培。
 * C620 量程 ±20A 对 ±16384，所以是 20/16384。
 *
 * ★ 这个数不是凭空来的 —— 上面那张表里已经藏了它：
 *     c2t = 满量程/16384 × kt
 *   M3508 反代回去：0.00002200 × 16384 / 20 = 0.01802 N·m/A，正是那个 0.018。
 *   所以 "16384 格式" 和 "安培" 之间从来就只差这一个常数，不是两套单位。
 *
 * ⚠️ 量程是【电调】的属性，不是电机型号的。M2006/C610 是 ±10A，换了要改这里。
 *    但改错也不会让功率控制失效：整体刻度偏 s 倍，等效于 k1 乘 1/s、k2 乘 1/s²，
 *    标定时会被一起吸收掉 —— 代价仅仅是网上抄来的系数不能直接用。 */
#define MOTOR_DJI_CURRENT_LSB (20.0f / 16384.0f)

static void motor_dji_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data);

// ─── DTS → 静态配置（ROM）───
// 公共部分由 MOTOR_DT_CONFIG_INST_INIT 填充，其余三项 DJI 独有。
// DTS 无浮点，string 用 DT_STRING_UNQUOTED 去引号再强转 float（同 pid.c）。
#define MOTOR_DJI_CONFIG_DEFINE(inst)                                      \
	static const motor_dji_config motor_dji_config_##inst = {              \
		.base         = MOTOR_DT_CONFIG_INST_INIT(inst),                   \
		.type         = DT_INST_ENUM_IDX(inst, type),                      \
		.gear_ratio   = (float)DT_STRING_UNQUOTED(DT_DRV_INST(inst),       \
		                                          gear_ratio),             \
		.angle_offset = MOTOR_DJI_DT_ANGLE_RAD(inst, angle_offset_deg),    \
	};

// ─── 每实例独立的运行时数据（RAM），不能共享 ───
#define MOTOR_DJI_DATA_DEFINE(inst)                           \
	static motor_dji_data motor_dji_data_##inst = {           \
		.base_data 	    = MOTOR_DT_DATA_INST_INIT(inst),      \
	};

/**
 * @brief 设备初始化：为反馈 CAN ID 注册精确匹配的标准帧 filter。
 *
 * filter 命中的回调在中断上下文执行。
 *
 * @return 0 成功，-ENODEV 表示 DTS 指向的 CAN 设备不可用
 */
static int motor_dji_init(const struct device *dev) {
	const motor_dji_config *cfg = dev->config;
	struct can_filter filter = {
		.id    = cfg->base.rx_id,     /* 仅接收本电机 DTS 配置的反馈 CAN ID */
		.mask  = CAN_STD_ID_MASK,     /* 0x7FF：比较全部 11 位，因此为精确匹配 */
		.flags = 0,                   /* 未设置 CAN_FILTER_IDE，表示标准帧而非扩展帧 */
	};
	int ret;

	if (!device_is_ready(cfg->base.can_dev)) {
		return -ENODEV;
	}
	if (cfg->base.rx_id > CAN_STD_ID_MASK || cfg->gear_ratio <= 0) {
		return -EINVAL;
	}

	ret = can_add_rx_filter(cfg->base.can_dev, motor_dji_rx_callback,
				(void *)dev, &filter);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief 解析一帧 DJI 反馈：解码整数 + 给增量型电机累加计数。
 *
 * 帧布局：encoder(16) / rpm(16) / current(16) / temperature(8)。
 * 不换算角度 —— 本函数在中断上下文，浮点留在 motor_dji_data_convert。
 * 两条角度语义（绝对 / 增量）见 motor_dji.h 对 base_data.angle 的说明。
 */
static void motor_dji_process_feedback(motor_dji_data *data, const motor_dji_config *cfg, const struct can_frame *frame){
	data->raw_data.encoder = ((uint16_t)frame->data[0] << 8) | frame->data[1];
	data->raw_data.rpm = (int16_t)(((uint16_t)frame->data[2] << 8) | frame->data[3]);
	data->raw_data.current = (int16_t)(((uint16_t)frame->data[4] << 8) | frame->data[5]);
	data->raw_data.temperature = (int32_t)frame->data[6];

	if(cfg->type != MOTOR_DJI_GM6020){
		int32_t delta = (int32_t)data->raw_data.encoder - data->prev_encoder;

		if (delta > MOTOR_DJI_ENCODER_COUNTS / 2) {
			delta -= MOTOR_DJI_ENCODER_COUNTS;
		} else if (delta < -(MOTOR_DJI_ENCODER_COUNTS / 2)) {
			delta += MOTOR_DJI_ENCODER_COUNTS;
		}

		data->angle_counts += delta;

		data->prev_encoder = data->raw_data.encoder;
	}
}

/**
 * @brief DJI电机数据转换，线程中进行，处理复杂运算
 */
static void motor_dji_data_convert(const struct device *dev, motor_data *out){
	const motor_dji_config *cfg = dev->config;
	motor_dji_data *data = dev->data;
	motor_dji_raw_data raw;
	int32_t angle_counts;
	bool enabled;
	float rad_per_count;
	float angle;
	k_spinlock_key_t key;

	if (out == NULL) {
		return;
	}

	/* ① 锁内：整份取整数快照，十几条指令即出。
	 *    必须一次取齐：ISR 会在两次读之间推进一帧，
	 *    零散读会得到「角度上一帧、转速这一帧」的错位结果。 */
	key = k_spin_lock(&data->lock);
	raw          = data->raw_data;
	angle_counts = data->angle_counts;
	enabled      = data->base_data.enabled;
	k_spin_unlock(&data->lock, key);

	/* ② 锁外：全部浮点换算，算的是上面那份快照。
	 *    输出轴每计数的弧度 = 转子 8192 计数/圈 换算到输出轴 */
	rad_per_count = MOTOR_DJI_TWO_PI / (MOTOR_DJI_ENCODER_COUNTS * cfg->gear_ratio);

	if (cfg->type == MOTOR_DJI_GM6020) {
		/* 直驱 GM6020：绝对编码器，每帧用本帧值重算、不累加，无首帧问题。
		 * 先把编码器读数换成角度，软件零点偏移留到最后一步再减；
		 * 归一到 (-π, π]，零点读 0、跳变落在 ±180°。
		 * 代价：角度只在本圈内唯一，跨圈须由上层按最小弧处理。 */
		angle = fmodf((float)raw.encoder * rad_per_count,
			      MOTOR_DJI_TWO_PI); /* [0, 2π) */

		angle -= cfg->angle_offset; /* 软件零点：非直驱电机该值为 0 */

		/* 归一到 (-π, π]。fmodf 结果 < 2π、offset 在 [0, 2π) 内，
		 * 故 angle ∈ (-2π, 2π)，一次修正即可。 */
		if (angle > MOTOR_DJI_PI) {
			angle -= MOTOR_DJI_TWO_PI;
		} else if (angle <= -MOTOR_DJI_PI) {
			angle += MOTOR_DJI_TWO_PI;
		}
	} else {
		angle = (float)angle_counts * rad_per_count;
	}

	/* ③ 填 out：写的是调用者的私有内存，不共享，也不需要锁 */
	out->enabled     = enabled;
	out->angle       = angle;
	out->omega_rotor = (float)raw.rpm * MOTOR_DJI_TWO_PI / 60.0f;
	out->omega       = (float)raw.rpm * MOTOR_DJI_TWO_PI / (60.0f * cfg->gear_ratio);
	out->current     = (float)raw.current * MOTOR_DJI_CURRENT_LSB;
	out->torque      = (float)raw.current * dji_current2torque[cfg->type] * cfg->gear_ratio;
	out->temperature = (float)raw.temperature;
}

/**
 * @brief CAN 接收回调，在中断上下文被 Zephyr 调用。
 */
static void motor_dji_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data){
	const struct device *dev = user_data;
	const motor_dji_config *cfg;
	motor_dji_data *data;

	ARG_UNUSED(can_dev);
	if (dev == NULL || frame == NULL) {
		return;
	}

	cfg = dev->config;
	data = dev->data;
	motor_dji_process_feedback(data, cfg, frame);
}

/**
 * @brief 读取电机当前状态（motor_api.get 回调）
 * @param data 输出：当前状态
 * @return 0 成功，-EINVAL 参数非法
 */
static int motor_dji_get(const struct device *dev, motor_data *data){
	if (data == NULL) {
		return -EINVAL;
	}

	/* 与 CAN 中断的互斥由 data_convert 内部负责：先取整数快照，再在锁外算 */
	motor_dji_data_convert(dev, data);

	return 0;
}

/**
 * @brief 设置目标（motor_api.set 回调）
 * @param sp 目标设定点
 * @return 0 成功，-EINVAL 参数非法
 */
static int motor_dji_set(const struct device *dev, const motor_setpoint *sp){
	motor_dji_data *dji_data = dev->data;
	k_spinlock_key_t key;

	/* 校验放在锁外：不碰共享状态，失败路径也就不必持锁返回 */
	if (sp == NULL) {
		return -EINVAL;
	}

	if (sp->mode != MOTOR_MODE_POSITION && sp->mode != MOTOR_MODE_SPEED &&
	    sp->mode != MOTOR_MODE_TORQUE) {
		return -EINVAL;
	}

	/* motor_setpoint 有 16 字节，不是原子存储：不加锁时发送线程可能读到
	 * 「新的 mode 配旧的数值」，那等于用错控制模式，比数值误差严重得多。 */
	key = k_spin_lock(&dji_data->lock);
	dji_data->target = *sp;
	k_spin_unlock(&dji_data->lock, key);

	return 0;
}

/**
 * @brief CAN 帧打包
 * @param dev 电机设备指针
 * @param out 输出缓冲区
 * @return 0 成功，-EINVAL 未知ID
 */
static int motor_dji_can_pack(const struct device *dev, uint8_t *out){
	const motor_dji_config *cfg = dev->config;
	motor_dji_data *data = dev->data;

	k_spinlock_key_t key;

	float torque;
	int16_t tmp_data;

	if (out == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	torque = data->target.torque;
	k_spin_unlock(&data->lock, key);

	tmp_data = (int16_t)(torque / dji_current2torque[cfg->type] / cfg->gear_ratio);

	switch (cfg->base.id) {
		case 1:
		case 5:
			out[0] = (uint8_t)(tmp_data >> 8);
			out[1] = (uint8_t)(tmp_data & 0xFF);
			break;

		case 2:
		case 6:
			out[2] = (uint8_t)(tmp_data >> 8);
			out[3] = (uint8_t)(tmp_data & 0xFF);
			break;

		case 3:
		case 7:
			out[4] = (uint8_t)(tmp_data >> 8);
			out[5] = (uint8_t)(tmp_data & 0xFF);
			break;

		case 4:
		case 8:
			out[6] = (uint8_t)(tmp_data >> 8);
			out[7] = (uint8_t)(tmp_data & 0xFF);
			break;

		default:
			return -EINVAL;
	}
	return 0;
}

/* motor_api 实现表：三个回调均已实现，注册时交给 DEVICE_DT_DEFINE */
static const motor_api motor_dji_api = {
	.get     = motor_dji_get,
	.set     = motor_dji_set,
	.can_pack = motor_dji_can_pack,
};

// ─── 为单个 status = "okay" 的 DJI 节点注册 Zephyr device ───
#define MOTOR_DJI_INST(inst)                                       \
	MOTOR_DJI_CONFIG_DEFINE(inst);                                 \
	MOTOR_DJI_DATA_DEFINE(inst);                                   \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst),                            \
	                 motor_dji_init,                               \
	                 NULL,                                         \
	                 &motor_dji_data_##inst,                       \
	                 &motor_dji_config_##inst,                     \
	                 POST_KERNEL,                                  \
	                 91,                                           \
	                 &motor_dji_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DJI_INST)