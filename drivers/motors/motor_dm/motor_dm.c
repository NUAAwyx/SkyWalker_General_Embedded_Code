#include "drivers/motors/motor_dm/motor_dm.h"

#include <zephyr/drivers/can.h>

/* DT_DRV_COMPAT 必须在任何 DT_* 宏被展开之前定义：
 * 它把 DT_INST_xxx / DT_DRV_INST 里的 skywalker_motor_dm 换成下面这个字符串，
 * 从而让整套宏只匹配 compatible = "skywalker,motor_dm" 的节点。 */
#define DT_DRV_COMPAT skywalker_motor_dm

/* ─── MIT 模式下三个物理量的映射量程 ───
 * 必须与电机里用调试助手设的 PMAX / VMAX / TMAX 一致：手册 4.2 末尾要求
 * 「发送控制命令时，一定要与设定值保持一致，否则控制命令会发生等比例缩放」
 * ——对不上不报错，只是整个扭矩差一个固定系数。手册的预设值是 ±12.5 / ±45 / ±18。 */
#define MOTOR_DM_P_MAX 12.5f
#define MOTOR_DM_V_MAX 45.0f
#define MOTOR_DM_T_MAX 18.0f


/* 四组固定指令帧（达妙驱动控制协议 V1.4 第 4 章 CAN 通信 4.5~4.8）。*/
static const uint8_t enable_frame[]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t disable_frame[]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static const uint8_t set_zero_frame[]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
static const uint8_t clear_error_frame[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};


#define MOTOR_DM_CONFIG_DEFINE(inst)                           \
    static const motor_dm_config motor_dm_config_##inst = {    \
        .base = MOTOR_DT_CONFIG_INST_INIT(inst),               \
        .type = DT_INST_ENUM_IDX(inst, type),                  \
    };

/* raw_data 显式初始化成"映射零点"，而不是让静态存储把它默认为全 0：
 * 全 0 在映射里是量程**下界**，不是零。一帧反馈都没收到时（电机没上电、
 * CAN 没接、id 配错），motor_get() 就会报 -12.5 rad / -45 rad/s / -18 N·m
 * 这种"看起来完全合法"的错值，上层不先判 enabled 就会被骗。
 * 中点 0x7FFF / 0x7FF 解出来才是 ~0。 */
#define MOTOR_DM_DATA_DEFINE(inst)                             \
    static motor_dm_data motor_dm_data_##inst = {              \
        .base_data = MOTOR_DT_DATA_INST_INIT(inst),            \
        .raw_data.position = 0x7FFF,                           \
        .raw_data.velocity = 0x7FF,                            \
        .raw_data.torque   = 0x7FF,                            \
    };

/**
 * @brief CAN 接收回调，在中断上下文被 Zephyr 调用。
 *
 * 反馈帧字节图（达妙驱动控制协议，三种控制模式反馈格式相同）：
 *   D[0]   = status[7:4] | motor_id[3:0]      高 4 位状态码，低 4 位电机 ID
 *   D[1:2] = pos_u[15:0]                      16 位无符号线性映射
 *   D[3]   = vel_u[11:4]                      速度 bits[11:4]
 *   D[4]   = vel_u[3:0]<<4 | torq_u[11:8]     高 4 位归速度，低 4 位归扭矩
 *   D[5]   = torq_u[7:0]                      扭矩 bits[7:0]
 *   D[6]   = T_mos                            MOS 平均温度 (℃)
 *   D[7]   = T_rotor                          线圈平均温度 (℃)
 */
static void motor_dm_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data){
    const struct device *dev = user_data;
    const motor_dm_config *cfg;
    motor_dm_data *data;
    uint8_t motor_id;

    ARG_UNUSED(can_dev);

    if (dev == NULL || frame == NULL) {
        return;
    }
    /* CAN-FD 的 DLC 可以不是 8；只有经典 8 字节帧才是这个字节图 */
    if (frame->dlc != 8U) {
        return;
    }

    cfg  = dev->config;
    data = dev->data;

    /* 多台 DM 共用一个反馈仲裁 ID（主站 ID），身份靠 D[0] 低 4 位区分。
     * 不认领的话，每台电机都会记下总线上最后一帧——可能是隔壁那台的。 */
    motor_id = frame->data[0] & 0x0F;
    if (motor_id != (cfg->base.id & 0x0F)) {
        return;
    }

    data->raw_data.id         = motor_id;
    data->raw_data.error_code = frame->data[0] >> 4;

    /* 使能与否只有电机说了算，所以 base_data.enabled 只在这里写，can_pack 只读。
     * ERR 是 4 位状态字，整值 1 才是"已使能"（手册 4.1）。不能写成 & 0x01：
     * 9(欠压) / B(MOS过温) / D(通讯丢失) / E(过载) 的 bit0 也是 1，
     * 那样报着错的电机会被当成"已使能"继续收扭矩帧。 */
    data->base_data.enabled = (data->raw_data.error_code == 1);

    data->raw_data.position   = ((uint16_t)frame->data[1] << 8) | frame->data[2];

    /* 速度 12 位跨 D[3]/D[4]：D[3] 是 bits[11:4]，D[4] 高 4 位是 bits[3:0] */
    data->raw_data.velocity   = ((uint16_t)frame->data[3] << 4) | (frame->data[4] >> 4);

    /* 扭矩 12 位跨 D[4]/D[5]：D[4] 低 4 位是 bits[11:8]，D[5] 是 bits[7:0] */
    data->raw_data.torque     = ((uint16_t)(frame->data[4] & 0x0F) << 8) | frame->data[5];

    data->raw_data.mos_temperature   = frame->data[6];
    data->raw_data.rotor_temperature = frame->data[7];
}

/* 返回 int，不是 void：DEVICE_DT_DEFINE 的 init 回调签名是
 * int (*)(const struct device *)，返回 0 表示初始化成功。 */
static int motor_dm_init(const struct device *dev){
    const motor_dm_config *cfg = dev->config;

	struct can_filter filter = {
		.id    = cfg->base.rx_id,     /* 仅接收本电机 DTS 配置的反馈 CAN ID */
		.mask  = CAN_STD_ID_MASK,     /* 0x7FF：比较全部 11 位，因此为精确匹配 */
		.flags = 0,                   /* 未设置 CAN_FILTER_IDE，表示标准帧而非扩展帧 */
	};

	int ret;

	if (!device_is_ready(cfg->base.can_dev)) {
		return -ENODEV;
	}
	if (cfg->base.rx_id > CAN_STD_ID_MASK) {
		return -EINVAL;
	}

	ret = can_add_rx_filter(cfg->base.can_dev, motor_dm_rx_callback, (void *)dev, &filter);
	
    if (ret < 0) {
		return ret;
	}

	return 0;
}

/** @brief 手册 4.2 的 float_to_uint：工程值 → N 位无符号原始值。*/
static uint16_t motor_dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits){
    float span = x_max - x_min;

    if (x < x_min) {
        x = x_min;
    } else if (x > x_max) {
        x = x_max;
    }

    return (uint16_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

/** @brief 上面那个的反变换：N 位无符号原始值 → 工程值。*/
static float motor_dm_uint_to_float(uint16_t u, float x_min, float x_max, uint8_t bits){
    return x_min + (float)u * (x_max - x_min) / (float)((1 << bits) - 1);
}

/** @brief 将电机目标设定点打包成 CAN 帧数据（只使用MIT协议的力矩模式）
 *
 * 电机未使能时只发使能帧，一直重发到电机回报 ERR == 1；已使能后每次都发控制帧。
 *
 * @param dev 电机设备
 * @param out 输出的 CAN 帧数据（调用方已填好 id 和 dlc = 8）
 * @return 0 表示成功，负值表示错误
 */
static int motor_dm_can_pack(const struct device *dev, uint8_t *out){
    motor_dm_data *data = dev->data;

    k_spinlock_key_t key;

    float torque;
    bool enabled;
    uint16_t pos, vel, t_ff;
    /* kp / kd 在纯力矩模式下就是 0：它们的量程是 [0,500] 和 [0,5]，
     * 不像位置速度那样关于零对称，所以工程值 0 映射过去还是 0，不是中点。 */
    const uint16_t kp = 0U;
    const uint16_t kd = 0U;

    if (out == NULL) {
        return -EINVAL;
    }

    /* enabled 由 rx_callback 从反馈帧写，本函数只读不写 */
    key = k_spin_lock(&data->lock);
    torque  = data->target.torque;
    enabled = data->base_data.enabled;
    k_spin_unlock(&data->lock, key);

    /* 电机还没回报"已使能"就一直重发使能帧：电机上电晚、使能帧丢在总线上，
     * 都能自己恢复。电机一旦确认，下一个周期自动切到控制帧。 */
    if(!enabled){
        memcpy(out, enable_frame, sizeof(enable_frame));
    }else{
        /* 纯力矩：手册 3.1「kp=0,kd=0，给定 t_ff 即可实现给定扭矩输出」。
         *
         * 位置和速度虽然被 kp、kd（都是 0）乘掉了，原始值也不能填 0x000：
         * 它们是线性映射的原始值，0x000 是量程下界（-12.5 rad / -45 rad/s），
         * 不是零。零在原始值上是量程中点。 */
        pos  = motor_dm_float_to_uint(0.0f,   -MOTOR_DM_P_MAX, MOTOR_DM_P_MAX, 16);
        vel  = motor_dm_float_to_uint(0.0f,   -MOTOR_DM_V_MAX, MOTOR_DM_V_MAX, 12);
        t_ff = motor_dm_float_to_uint(torque, -MOTOR_DM_T_MAX, MOTOR_DM_T_MAX, 12);

        out[0] = (uint8_t)(pos >> 8);                           /* pos[15:8]            */
        out[1] = (uint8_t)pos;                                  /* pos[7:0]             */
        out[2] = (uint8_t)(vel >> 4);                           /* vel[11:4]            */
        out[3] = (uint8_t)(((vel & 0x0FU) << 4) | (kp >> 8));   /* vel[3:0] | kp[11:8]  */
        out[4] = (uint8_t)kp;                                   /* kp[7:0]              */
        out[5] = (uint8_t)(kd >> 4);                            /* kd[11:4]             */
        out[6] = (uint8_t)(((kd & 0x0FU) << 4) | (t_ff >> 8));  /* kd[3:0]  | t_ff[11:8] */
        out[7] = (uint8_t)t_ff;                                 /* t_ff[7:0]            */
    }

    return 0;
}

static int motor_dm_get(const struct device *dev, motor_data *out){
    motor_dm_data *data = dev->data;
    motor_dm_raw_data raw;

    k_spinlock_key_t key;

    if (out == NULL) {
        return -EINVAL;
    }

    key = k_spin_lock(&data->lock);
    raw = data->raw_data;
    out->enabled = data->base_data.enabled;
    k_spin_unlock(&data->lock, key);
    
    out->angle       = motor_dm_uint_to_float(raw.position, -MOTOR_DM_P_MAX, MOTOR_DM_P_MAX, 16);
    out->omega       = motor_dm_uint_to_float(raw.velocity, -MOTOR_DM_V_MAX, MOTOR_DM_V_MAX, 12);
    out->torque      = motor_dm_uint_to_float(raw.torque,   -MOTOR_DM_T_MAX, MOTOR_DM_T_MAX, 12);
    /* 温度不走映射：反馈帧 D[6] 本身就是 ℃，不是"0~255 格" */
    out->temperature = (float)raw.mos_temperature;

    return 0;
}

static int motor_dm_set(const struct device *dev, const motor_setpoint *sp){
    motor_dm_data *data = dev->data;
    const motor_dm_config *cfg = dev->config;
    k_spinlock_key_t key;

    if (sp == NULL) {
        return -EINVAL;
    }
    /* 只实现 MIT 模式的力矩模式：PV / VO 的帧布局完全不同（手册 4.3 / 4.4）， 拿 MIT 帧去发只会让电机按错误的量纲动。 */
    if (cfg->type != DM_TYPE_MIT || sp->mode != MOTOR_MODE_TORQUE) {
        return -ENOTSUP;
    }

    /* motor_setpoint 有 16 字节，不是原子存储：不加锁时发送线程可能读到
     * 「新的 mode 配旧的数值」，那等于用错控制模式，比数值误差严重得多。 */
    key = k_spin_lock(&data->lock);
    data->target = *sp;
    k_spin_unlock(&data->lock, key);

    return 0;
}

static const motor_api motor_dm_api = {
    .can_pack = motor_dm_can_pack,
    .get      = motor_dm_get,
    .set      = motor_dm_set,
};

#define MOTOR_DM_INST(inst)                                    \
	MOTOR_DM_CONFIG_DEFINE(inst);                              \
	MOTOR_DM_DATA_DEFINE(inst);                                \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst),                        \
	                 motor_dm_init,                            \
	                 NULL,                                     \
	                 &motor_dm_data_##inst,                    \
	                 &motor_dm_config_##inst,                  \
	                 POST_KERNEL,                              \
	                 91,                                       \
	                 &motor_dm_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DM_INST)