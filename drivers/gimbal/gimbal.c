#include "drivers/gimbal/gimbal.h"

#include <math.h>

#define DT_DRV_COMPAT skywalker_gimbal

/* 不用 <math.h> 的 M_PI：picolibc 里它受 __BSD_VISIBLE 保护，-std=c17 下不可见。
 * motor_dji.c 和 chassis.c 都各自 define 过，这里第三次。 */
#define GIMBAL_PI     3.14159265359f
#define GIMBAL_TWO_PI (2.0f * GIMBAL_PI)

/* DTS 里按度写（人可读），驱动内部用 rad。 */
#define GIMBAL_DEG_TO_RAD 0.017453292519943295f

/* ─── 从 DT 填充 gimbal_config（ROM） ───
 *
 * DT 只能存 string，数值靠 DT_STRING_UNQUOTED 去引号得到数值字面量、再
 * (float) 强转成编译期常量 —— 和 pid.c / chassis.c 同一套。
 *
 * 属性全部 required: false 之后，"漏配"不再有编译期报错，下面那组
 * GIMBAL_DT_ASSERTS 是唯一拦得住的地方。
 */

/** 属性长度，缺失当 0 —— 用于属性可能不存在时安全地做长度比较 */
#define GIMBAL_PROP_LEN(inst, prop) DT_PROP_LEN_OR(DT_DRV_INST(inst), prop, 0)

/** 数组元素：第 idx 个摩擦轮的设备
 *
 * ⚠️ 末尾那个逗号【是必须的，不是手滑】。
 *    DT_FOREACH_PROP_ELEM 生成的是 fn(…,0) fn(…,1) … —— 元素之间【只有空白，
 *    没有逗号】，逗号得由逐元素宏自己吐。少了它，初始化列表成了 `expr expr`
 *    并置，gcc 报的是 "called object is not a function or function pointer"，
 *    指向的却是 DEVICE_DT_GET —— 离真正的原因十万八千里。实测踩过。 */
#define GIMBAL_DEV_BY_IDX(node_id, prop, idx) \
    DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

/** 度字符串 → 弧度。属性必须存在（由 asserts 保证） */
#define GIMBAL_RAD(inst, prop)                                  \
    ((float)DT_STRING_UNQUOTED(DT_DRV_INST(inst), prop) * GIMBAL_DEG_TO_RAD)

/** 选填标量 phandle：DT 里没写就是 NULL。Zephyr 没有 DT_INST_PHANDLE_OR */
#define GIMBAL_OPT_DEV(inst, prop)                                    \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                    \
                (DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), prop))), \
                (NULL))

/** 选填标量角度（度）→ 弧度：DT 里没写就是 0.0f */
#define GIMBAL_OPT_RAD(inst, prop)                                \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                \
                (GIMBAL_RAD(inst, prop)),                         \
                (0.0f))

/** 选填数组：DT 里没写就是全 NULL 的零数组。
 *
 * ★ 用法必须是 .field = { GIMBAL_OPT_DEVS(...) } —— 外面那层花括号【不能少】。
 *   本宏吐的是"逗号分隔的元素列表"（属性不在时吐一个 0），花括号由使用点提供。
 *
 * ⚠️ 别想着把花括号包进宏里、写成 ({ ... })：那是【语句表达式】，在函数外的
 *   静态初始化列表里是非法的（braced-group within expression allowed only
 *   inside a function）。这个坑 chassis.c 里踩过一次，已实测确认。
 *
 * ⚠️ 更要命的是这种错【在没有 DT 节点时根本不会暴露】：本宏只在下面那个
 *   CONFIG_INST_INIT 里展开，而那个宏要等 DT_INST_FOREACH_STATUS_OKAY 至少
 *   有一个节点才会展开。所以它会一路静默潜伏，直到你第一次写出
 *   skywalker,gimbal 节点的那一刻才炸。 */
#define GIMBAL_OPT_DEVS(inst, prop)                                             \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                              \
                (DT_INST_FOREACH_PROP_ELEM(inst, prop, GIMBAL_DEV_BY_IDX)),     \
                (0))

/** 编译期契约检查：在 gimbal.c 里对每个实例调一次 */
#define GIMBAL_DT_ASSERTS(inst)                                                          \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, yaw_motor),                                 \
                 "gimbal 必须有 yaw-motor");                                              \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, pitch_motor),                               \
                 "gimbal 必须有 pitch-motor");                                            \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, imu),                                       \
                 "gimbal 必须有 imu —— 两个速度环都闭合在它的角速度上，没有它环无从闭合");      \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, yaw_pid),                                   \
                 "gimbal 必须有 yaw-pid（角度外环，自瞄用）");                               \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, yaw_speed_pid),                             \
                 "gimbal 必须有 yaw-speed-pid（速度内环，两种模式都在跑）");                  \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, pitch_pid),                                 \
                 "gimbal 必须有 pitch-pid（角度外环）");                                   \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, pitch_speed_pid),                           \
                 "gimbal 必须有 pitch-speed-pid（速度内环）");                              \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, pitch_min_deg) &&                           \
                 DT_INST_NODE_HAS_PROP(inst, pitch_max_deg),                             \
                 "gimbal 必须有 pitch-min-deg 和 pitch-max-deg —— 位置环不会自己停，超程会一直顶"); \
    BUILD_ASSERT(GIMBAL_PROP_LEN(inst, friction_motors) <= GIMBAL_FRICTION_MAX,          \
                 "摩擦轮数量超过 GIMBAL_FRICTION_MAX：gimbal_data 里的积分器数组是定长的");    \
    BUILD_ASSERT(GIMBAL_PROP_LEN(inst, friction_motors) == 0 ||                          \
                 DT_INST_NODE_HAS_PROP(inst, friction_pid),                              \
                 "配了 friction-motors 就必须配 friction-pid");                           \
    BUILD_ASSERT(GIMBAL_PROP_LEN(inst, friction_motors) == 0 ||                          \
                 DT_INST_NODE_HAS_PROP(inst, friction_speed_deg_s),                      \
                 "配了 friction-motors 就必须配 friction-speed-deg-s —— 摩擦轮没有设定值可用");

/** gimbal_config 的 DT 初始化列表 */
#define GIMBAL_DT_CONFIG_INST_INIT(inst)                                                      \
    {                                                                                          \
        .yaw_motor            = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), yaw_motor)),        \
        .pitch_motor          = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), pitch_motor)),      \
        .imu                  = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), imu)),              \
        .yaw_pid              = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), yaw_pid)),          \
        .yaw_speed_pid        = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), yaw_speed_pid)),    \
        .pitch_pid            = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), pitch_pid)),        \
        .pitch_speed_pid      = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), pitch_speed_pid)),  \
        .pitch_min_rad        = GIMBAL_RAD(inst, pitch_min_deg),                                \
        .pitch_max_rad        = GIMBAL_RAD(inst, pitch_max_deg),                                \
        .friction_motors      = { GIMBAL_OPT_DEVS(inst, friction_motors) },                     \
        .friction_num         = GIMBAL_PROP_LEN(inst, friction_motors),                         \
        .friction_pid         = GIMBAL_OPT_DEV(inst, friction_pid),                             \
        .friction_speed_rad_s = GIMBAL_OPT_RAD(inst, friction_speed_deg_s),                     \
    }

/** gimbal_config 的静态实例（ROM）。
 *  GIMBAL_DT_ASSERTS 就放在这里展开 —— 它是那组契约检查【唯一的调用点】。 */
#define GIMBAL_CONFIG_DEFINE(inst)                                     \
    GIMBAL_DT_ASSERTS(inst);                                           \
    static const gimbal_config gimbal_config_##inst =                  \
        GIMBAL_DT_CONFIG_INST_INIT(inst);

/** gimbal_data 的静态实例（RAM）。
 *  全零即正确的初始态：mode = MANUAL（"不跟踪"，见 gimbal_set_mode）、
 *  enabled = false、friction_on = false、每路积分器归零、两个角度基准都未建立。
 *  静态存储期由 C 保证零初始化，所以不写初始化列表。 */
#define GIMBAL_DATA_DEFINE(inst) \
    static gimbal_data gimbal_data_##inst;

/**
 * @brief 云台初始化：确认 DT 里点名的设备全都就绪。
 *
 * 这里【只做校验】，不碰控制 —— 反馈来自 IMU（另一个驱动），PID 跑在控制线程里。
 *
 * ⚠️ pitch 行程的上下限关系在这里查、不用 BUILD_ASSERT：_Static_assert 要整数
 *   常量表达式，而这两个值是 (float) 转换来的浮点量，浮点比较不是 ICE。
 *    所以"必填"由 asserts 保证、"大小关系"由这里保证，两条腿。
 *
 * @return 0 成功；-ENODEV 某设备没就绪；-EINVAL pitch 行程上下限反了
 */
static int gimbal_init(const struct device *dev) {
    const gimbal_config *cfg = dev->config;

    if (cfg->pitch_min_rad >= cfg->pitch_max_rad) {
        return -EINVAL;
    }

    /* 必需：两轴电机、IMU、四个环的增益。上面那组 asserts 保证这些一定配了。 */
    if (!device_is_ready(cfg->yaw_motor) ||
        !device_is_ready(cfg->pitch_motor) ||
        !device_is_ready(cfg->imu) ||
        !device_is_ready(cfg->yaw_pid) ||
        !device_is_ready(cfg->yaw_speed_pid) ||
        !device_is_ready(cfg->pitch_pid) ||
        !device_is_ready(cfg->pitch_speed_pid)) {
        return -ENODEV;
    }

    /* 摩擦轮：没配是正常的（不是每辆车都有），配了却没就绪才是故障。
     * ⚠️ 必须先看 friction_num 再碰 friction_pid —— friction_pid 允许是 NULL，
     *    而 device_is_ready(NULL) 返回 false（kernel/device.c:192），
     *    直接用会把"没装"误判成"故障"。这条 chassis_init 里也写过。 */
    for (uint8_t i = 0; i < cfg->friction_num; i++) {
        if (!device_is_ready(cfg->friction_motors[i])) {
            return -ENODEV;
        }
    }
    if (cfg->friction_num > 0 && !device_is_ready(cfg->friction_pid)) {
        return -ENODEV;
    }

    return 0;
}

/* ─── 接口实现 ─── */

/** 只翻一个 bool。真正的"清积分器 + 作废基准"在 gimbal_update 里 ——
 *  那里才是 pid_data 唯一的写方，见 gimbal.h 上这段的说明。 */
static int gimbal_driver_set_enabled(const struct device *dev, bool enabled) {
    gimbal_data *data = dev->data;

    data->enabled = enabled;
    return 0;
}

/** 切模式。同样【只写模式 + 作废基准】，不碰 pid_data。
 *
 *  ★ 只在切到 AUTO 时作废 yaw 基准：MANUAL 根本不用角度基准，作废它只会
 *    平白多丢一拍跟踪。pitch 基准不受模式影响（两模式都走角度环），不动。 */
static int gimbal_driver_set_mode(const struct device *dev, gimbal_mode_t mode) {
    gimbal_data *data = dev->data;

    if (mode != GIMBAL_MODE_MANUAL && mode != GIMBAL_MODE_AUTO) {
        return -EINVAL;
    }

    data->mode = mode;

    if (mode == GIMBAL_MODE_AUTO) {
        data->yaw_angle_valid = false;
    }

    return 0;
}

/** 三个指令都只写一个 float：每次都是单条对齐 store，读方不会读到半个数。
 *  三个量之间不需要"必须同一拍"（理由见 gimbal_status），所以都不上锁。
 *
 *  ★ 都不做模式检查：MANUAL 下写 yaw_angle 只是"存着"，切到 AUTO 就有得用，
 *    切模式不必重设。代价是写错模式时【静默无效】—— 这一点在 gimbal.h 的
 *    三个函数注释里各写了一遍，因为它确实是这里唯一的坑。 */
static int gimbal_driver_set_yaw_speed(const struct device *dev, float rad_s) {
    gimbal_data *data = dev->data;

    data->yaw_speed = rad_s;
    return 0;
}

static int gimbal_driver_set_yaw_angle(const struct device *dev, float rad) {
    gimbal_data *data = dev->data;

    data->yaw_angle = rad;
    return 0;
}

static int gimbal_driver_set_pitch_angle(const struct device *dev, float rad) {
    gimbal_data *data = dev->data;

    data->pitch_angle = rad;
    return 0;
}

/** 摩擦轮开关。没装摩擦轮就直说 -ENOSYS —— 让调用点知道"本车没这功能"，
 *  而不是默默记下一个永远不生效的 bool。 */
static int gimbal_driver_set_friction(const struct device *dev, bool on) {
    const gimbal_config *cfg = dev->config;
    gimbal_data *data = dev->data;

    if (cfg->friction_num == 0) {
        return -ENOSYS;
    }

    data->friction_on = on;
    return 0;
}

/** 拷一份出去。写方只有一个（gimbal_update），读方拿到的是自己栈上的副本，
 *  所以 struct 赋值这一下不会和写方打架 —— 最坏情况是拿到上一拍的值，
 *  那正是"快照"该有的语义。 */
static int gimbal_driver_get_status(const struct device *dev, gimbal_status *out) {
    const gimbal_data *data = dev->data;

    *out = data->status;
    return 0;
}

static const gimbal_api gimbal_driver_api = {
    .set_enabled      = gimbal_driver_set_enabled,
    .set_mode         = gimbal_driver_set_mode,
    .set_yaw_speed    = gimbal_driver_set_yaw_speed,
    .set_yaw_angle    = gimbal_driver_set_yaw_angle,
    .set_pitch_angle  = gimbal_driver_set_pitch_angle,
    .set_friction     = gimbal_driver_set_friction,
    .get_status       = gimbal_driver_get_status,
};

/* ═══════════════════ 云台控制一拍 ═══════════════════ */

/**
 * @brief 云台控制一拍：读 IMU → 外环 → 内环 → 下发电机
 *
 * 详版说明见 gimbal.h 的同名声明（为什么反馈全用 IMU、为什么解耦不用写代码）。
 */
void gimbal_update(const struct device *dev, float dt) {
    const gimbal_config *cfg = dev->config;
    gimbal_data *data = dev->data;

    motor_setpoint sp = {
        .angle = 0.0f, .omega = 0.0f, .torque = 0.0f,
        .mode  = MOTOR_MODE_TORQUE,
    };

    /* ① 读反馈。全部来自 IMU —— 电机自己的编码器在控制回路里【一次都不用】。
     *
     *    imu_get_angle 是唯一能一次拿到三个角、保证同拍的接口（内部有锁）；
     *    imu_get_gyro_axis 每次只要一个 float，单次对齐访问天然原子，所以
     *    两个轴分两次读 —— 这不是漏了一次调用，见 imu.h 里那段推演。 */
    float angle[3];

    imu_get_angle(cfg->imu, angle);

    float yaw_meas   = angle[IMU_AXIS_YAW];
    float pitch_meas = angle[IMU_AXIS_PITCH];
    float yaw_rate   = imu_get_gyro_axis(cfg->imu, IMU_AXIS_YAW);
    float pitch_rate = imu_get_gyro_axis(cfg->imu, IMU_AXIS_PITCH);

    /* 状态在【这里】填，不在末尾：它是"这一拍实测到的姿态"，和后面所有分支
     * 的决策无关。放这儿，失能分支就不用再抄一遍。 */
    data->status.yaw_angle   = yaw_meas;
    data->status.pitch_angle = pitch_meas;
    data->status.yaw_rate    = yaw_rate;
    data->status.pitch_rate  = pitch_rate;

    /* ② 失能：清积分器 + 作废基准 + 零扭矩下发，然后走人。
     *
     *    清积分器的动作【故意不放在 gimbal_set_enabled 里】：那里跨线程写
     *    pid_data 就得和正在算 PID 的本函数抢锁；放在这里，pid_data 从头到尾
     *    只有一个写方，锁根本不需要存在。（和 chassis 同一条。）
     *
     *    ⚠️ 基准也一并作废。这是本驱动比 chassis 多出来的一步：chassis 的指令
     *      是速度，失能期间摇杆回中就自然归零；而云台的目标角是【绝对角】，
     *      它不会因为失能而变得"离当前位置近"。不作废的话，重新使能的第一拍
     *      会拿着一个可能很久以前设的目标直接甩过去。 */
    if (!data->enabled) {
        data->yaw_angle_pid.i_out   = 0.0f;
        data->yaw_speed_pid.i_out   = 0.0f;
        data->pitch_angle_pid.i_out = 0.0f;
        data->pitch_speed_pid.i_out = 0.0f;

        for (int i = 0; i < GIMBAL_FRICTION_MAX; i++) {
            data->friction_pid[i].i_out = 0.0f;
        }

        data->yaw_angle_valid   = false;
        data->pitch_angle_valid = false;

        motor_set(cfg->yaw_motor, &sp);
        motor_set(cfg->pitch_motor, &sp);

        for (uint8_t i = 0; i < cfg->friction_num; i++) {
            motor_set(cfg->friction_motors[i], &sp);
        }
        return;
    }

    /* ③ 建基准：目标角直接对齐到实测角，也就是"本拍误差为 0、先不动"。
     *
     *    ★ 为什么必须有这一步：目标角是绝对值，而它可能是【很久以前】设的
     *      （上一次使能期间、上一次自瞄锁定时），也可能压根没设过（零初始化 = 0）。
     *      直接拿去驱动，误差就是"当前位置到那个旧值"的距离 —— 可能是一整圈。
     *      先对齐，这个跳变就不存在。
     *
     *    ⚠️ 代价：这一拍会【盖掉上层刚写进来的指令】。对连续给目标的上层无影响
     *      （下一拍它会再写一次），对只写一次的上层就是"第一帧被吃掉"。
     *      这和拨盘那条基准是同一个取舍，见 chassis_trigger_update。
     *
     *    积分器也在这里清：刚切进 AUTO 时 yaw 角度环的 i_out 还停在上一次
     *    自瞄留下的数上，不清的话第一拍会带着旧积分起步。 */
    if (!data->yaw_angle_valid) {
        data->yaw_angle             = yaw_meas;
        data->yaw_angle_valid       = true;
        data->yaw_angle_pid.i_out   = 0.0f;
    }

    if (!data->pitch_angle_valid) {
        data->pitch_angle           = pitch_meas;
        data->pitch_angle_valid     = true;
        data->pitch_angle_pid.i_out = 0.0f;
    }

    /* ④ pitch 限幅。限在【目标】上，不在输出上 —— 见 gimbal_set_pitch_angle
     *    的说明：位置环不会自己停，输出限幅拦不住"持续顶住限位"。
     *
     *    每拍都夹、不只在 setter 里夹：setter 有三个入口（本函数之外还有
     *    set_pitch_angle 和基准对齐），夹在这里是【唯一的收口点】，
     *    将来加第四个入口也不用记得再夹一次。 */
    float pitch_target = data->pitch_angle;

    if (pitch_target < cfg->pitch_min_rad) {
        pitch_target = cfg->pitch_min_rad;
    } else if (pitch_target > cfg->pitch_max_rad) {
        pitch_target = cfg->pitch_max_rad;
    }

    /* ⑤ yaw：定【速度环的设定值】。两种模式的区别只在这一步。
     *
     *    AUTO —— 角度外环串进内环：外环的输出（角速度）就是内环的设定值。
     *    MANUAL —— 角度外环【旁路】，摇杆直接给内环设定值。
     *
     *    ★ 内环本身两种模式都在跑，所以"模式"改的只是内环的输入从哪来。
     *      这也是为什么 yaw-speed-pid 是必需项而不是"手动模式才需要"。 */
    float yaw_speed_sp;

    if (data->mode == GIMBAL_MODE_AUTO) {
        /* 最短弧：目标离实测超过半圈时按另一侧走。
         * 上位机给的可能是 -π..π 的包裹角，而 IMU 的 yaw 是连续累加的，
         * 两者直接相减会得到"绕远路"的误差。remainderf 把它折到 ±π 内。
         * 注意折的是【差值】不是目标本身 —— 目标要保持连续,
         * 否则每一拍都要重新决定往哪边转，会在半圈边界上抖。 */
        float diff       = remainderf(data->yaw_angle - yaw_meas, GIMBAL_TWO_PI);
        float yaw_target = yaw_meas + diff;

        yaw_speed_sp = pid_update(&data->yaw_angle_pid,
                                  pid_get_config(cfg->yaw_pid),
                                  yaw_target, yaw_meas, dt, 0.0f);
    } else {
        /* 不用的那个环，积分器钉在零 —— 同 chassis 拨盘的理由：
         * 这样"刚切进来"和"第一次用"没有区别，也就不需要"模式变了"这个标志。 */
        data->yaw_angle_pid.i_out = 0.0f;
        yaw_speed_sp              = data->yaw_speed;
    }

    /* ⑥ 速度内环。反馈是 IMU 的【世界系】角速度，不是电机编码器 ——
     *    底盘自转时陀螺立刻看到变化，环直接反向补回来，解耦不需要额外代码。
     *    （用编码器的话底盘转、编码器纹丝不动，环会以为"已到位"。） */
    float yaw_torque = pid_update(&data->yaw_speed_pid,
                                  pid_get_config(cfg->yaw_speed_pid),
                                  yaw_speed_sp, yaw_rate, dt, 0.0f);

    /* pitch 走同样的两级，只是外环永远在（pitch 有行程，必须有个目标角）。 */
    float pitch_speed_sp = pid_update(&data->pitch_angle_pid,
                                      pid_get_config(cfg->pitch_pid),
                                      pitch_target, pitch_meas, dt, 0.0f);

    float pitch_torque = pid_update(&data->pitch_speed_pid,
                                    pid_get_config(cfg->pitch_speed_pid),
                                    pitch_speed_sp, pitch_rate, dt, 0.0f);

    /* ⑦ 下发。只 set，不 pack 也不 send —— 打包和发送是另一条线程的事
     *    （见 motor_dji_test 里那条 1ms 定时发送线程）。
     *
     *    mode 写 TORQUE 是【声明意图】：两个环算出来的是力矩，不是"让谁自己
     *    去查表"。motor_dji_can_pack 目前只读 target.torque、压根不看 mode，
     *    写对了是为了以后换上真按 mode 分派的电机时这行还是对的。 */
    sp.torque = yaw_torque;
    motor_set(cfg->yaw_motor, &sp);

    sp.torque = pitch_torque;
    motor_set(cfg->pitch_motor, &sp);

    /* ⑧ 摩擦轮。开关和使能是【与】的关系：enabled 是总闸，friction_on 是
     *    操作手的意思。两者都为真才出力。 */
    if (cfg->friction_num > 0) {
        const pid_config *fric_pid_cfg = pid_get_config(cfg->friction_pid);

        if (!data->friction_on) {
            /* 关 = 清积分器 + 零扭矩。
             *
             * ⚠️ 清积分器【必须做】：不清的话关-开一次，速度环会带着停机期间
             *    攒下的积分起步，那一下比正常启动猛得多。停机期间误差一直是
             *    -目标转速，积分一路往负里跑。
             *    这和失能清积分器是同一条，只是"关"比"失能"更常发生。 */
            sp.torque = 0.0f;

            for (uint8_t i = 0; i < cfg->friction_num; i++) {
                data->friction_pid[i].i_out = 0.0f;
                motor_set(cfg->friction_motors[i], &sp);
            }
        } else {
            for (uint8_t i = 0; i < cfg->friction_num; i++) {
                motor_data md;
                float omega_meas = 0.0f;

                /* 读失败留 0：同其它驱动 —— 宁可这一拍当"没转"，
                 * 也别拿上一拍的残值去喂速度环。 */
                if (motor_get(cfg->friction_motors[i], &md) == 0) {
                    omega_meas = md.omega;   /* 输出轴 rad/s，和 DT 里那个目标同轴 */
                }

                sp.torque = pid_update(&data->friction_pid[i], fric_pid_cfg,
                                       cfg->friction_speed_rad_s, omega_meas, dt, 0.0f);
                motor_set(cfg->friction_motors[i], &sp);
            }
        }
    }
}

/* ─── 为单个 status = "okay" 的 gimbal 节点注册 Zephyr device ───
 *
 * 级别 92 而不是 91：init 里要 device_is_ready 电机(91)/IMU(91)/PID(50)，
 * 必须排在它们之后。同一 init level 内按优先级【升序】执行。
 * 和 chassis 同为 92 —— 两个驱动互不依赖（gimbal 不读 chassis_status），
 * 所以它们之间的顺序无所谓。
 */
#define GIMBAL_INST(inst)                         \
    GIMBAL_CONFIG_DEFINE(inst);                   \
    GIMBAL_DATA_DEFINE(inst);                     \
    DEVICE_DT_DEFINE(DT_DRV_INST(inst),           \
                     gimbal_init,                 \
                     NULL,                        \
                     &gimbal_data_##inst,         \
                     &gimbal_config_##inst,       \
                     POST_KERNEL,                 \
                     92,                          \
                     &gimbal_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GIMBAL_INST)
