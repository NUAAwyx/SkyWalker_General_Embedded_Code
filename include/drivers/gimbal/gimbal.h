#ifndef GIMBAL_H
#define GIMBAL_H

#include "drivers/motors/motor.h"
#include "drivers/imu/imu.h"
#include "drivers/pid/pid.h"

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 云台工作模式
 *
 * 【不进 DT】：比赛时由拨杆/上位机切换，是运行时行为。写进 DT 会被烧成 flash
 * 常量，拨一下拨杆不该需要重编译。
 */
typedef enum {
    GIMBAL_MODE_MANUAL = 0,  /**< 手动：yaw 角度环【旁路】，摇杆/鼠标直接给角速度 */
    GIMBAL_MODE_AUTO,        /**< 自瞄：yaw 角度环串进速度环，上位机给绝对角度 */
} gimbal_mode_t;

/**
 * @brief 摩擦轮数量上限
 *
 * 摩擦轮是变长的（1~4 个都可能），而 gimbal_data 是静态存储期、C 数组长度
 * 必须编译期确定，所以这里给一个上限 + 一个运行时计数 friction_num。
 * 超了由 GIMBAL_DT_ASSERTS 拦下，不会静默截断。
 */
#define GIMBAL_FRICTION_MAX 4

/**
 * @brief 云台配置（ROM，由 DT 填充）
 *
 * 字段与 dts/bindings/gimbal/skywalker,gimbal.yaml 逐项对应。
 * 角度类字段在【DT 宏里就换算成弧度】了，所以这里存的是 rad —— 和
 * chassis 把 wheel-x 直接存米是同一个思路：进来是什么单位，用的时候是什么单位，
 * 不在控制回路里做单位转换。
 */
typedef struct {
    const struct device *yaw_motor;             /**< yaw 轴电机 */
    const struct device *pitch_motor;           /**< pitch 轴电机 */
    const struct device *imu;                   /**< 姿态反馈设备。云台环的【唯一】反馈源 */

    const struct device *yaw_pid;               /**< yaw 角度环（自瞄用，串在速度环前） */
    const struct device *yaw_speed_pid;         /**< yaw 速度环（内环，永远在跑） */
    const struct device *pitch_pid;             /**< pitch 角度环（两模式都用） */
    const struct device *pitch_speed_pid;       /**< pitch 速度环（内环） */

    float pitch_min_rad;                        /**< pitch 机械行程下限（rad）。必填，见 binding */
    float pitch_max_rad;                        /**< pitch 机械行程上限（rad） */

    /* ── 摩擦轮 ──
     * 目标转速是【装车事实】（这组摩擦轮该转多少转），所以进 DT；
     * 开不开是【打法】（按键决定），所以走运行时接口 gimbal_set_friction。 */
    const struct device *friction_motors[GIMBAL_FRICTION_MAX]; /**< 未用到的槽位是 NULL */
    uint8_t              friction_num;                         /**< 实际装了几个 */
    const struct device *friction_pid;                         /**< 共用一组增益 */
    float                friction_speed_rad_s;                 /**< 目标角速度（输出轴 rad/s） */
} gimbal_config;

/**
 * @brief 云台实测状态【快照】
 *
 * ⚠️ 拿到的不是活数据，是拷贝 —— 之后云台怎么转都不影响你手上这份。
 *    （对比 mambo 的 get_status 直接返回内部指针：那是把驱动内部地址交出去，
 *      调用者反手就能改状态，分层就没了。这里用 out 参数堵死这条路。）
 *
 * ★ 这里【故意不加锁】，判据和 chassis_status 是同一条：
 *   加锁的判据不是"它们恰好是同一个结构体"，是"这几个值【必须一起正确】"。
 *   这四个量各自独立 —— 跨拍拼出来的仍然是"一个真实存在过的姿态"，只是误差被
 *   (角速度 × 错拍时间) 限住。一个瞬态误差下一拍自己就纠正了，和一个"不存在的
 *   状态"是两回事。而且四个 float 都是单次对齐访问，本来就读不到半个数。
 *
 * ⚠️ 什么时候该回来加锁：哪天有消费者把 pitch 和 yaw 【合起来用】
 *    （比如按两者算枪口指向矢量），那就变成"必须同一拍"，那时再加。
 */
typedef struct {
    float yaw_angle;    /**< 实测 yaw 姿态角（rad，IMU 世界系） */
    float pitch_angle;  /**< 实测 pitch 姿态角（rad，IMU 世界系） */
    float yaw_rate;     /**< 实测 yaw 角速度（rad/s，陀螺原始量） */
    float pitch_rate;   /**< 实测 pitch 角速度（rad/s，陀螺原始量） */
} gimbal_status;

/**
 * @brief 云台运行时数据（RAM）
 *
 * 判据同 chassis_data：只放"这一拍算完、下一拍还要知道"的东西。
 * 一拍之内算完就丢的量（当拍误差、PID 输出）一律用局部变量。
 */
typedef struct {
    /* ── 指令：上层写，控制循环读 ── */
    gimbal_mode_t mode;          /**< 默认 MANUAL（零初始化 = 0）——见 gimbal_set_mode */
    bool          enabled;       /**< false = 输出强制清零（急停 / 未就绪） */
    bool          friction_on;   /**< 摩擦轮开关（独立按键）。仍然受 enabled 一票否决 */

    /** 手动模式的 yaw 输入：目标【角速度】(rad/s)。摇杆量。
     *  ⚠️ 只在 MANUAL 下生效；AUTO 下这个值被忽略（角度环说话）。 */
    float         yaw_speed;

    /** 自瞄模式的 yaw 输入：目标【角度】(rad)，绝对。
     *  ⚠️ 只在 AUTO 下生效；MANUAL 下这个值被忽略（摇杆说话）。
     *
     *  两个输入分成两个字段而不是共用一个"yaw 指令"，是因为它们的【量纲不同】
     *  （rad/s vs rad）。共用一个字段的话，同一行 set_yaw(x) 在不同模式下
     *  意思完全不同 —— 那种错，编译器拦不住，看代码也看不出来。
     *  上层按自己设的模式调对应的那个函数即可（切模式不必重设：另一个值
     *  一直留着，"切过去就能用"）。 */
    float         yaw_angle;

    /** pitch 目标【角度】(rad)，绝对。两种模式共用 ——
     *  pitch 有机械行程，手动时的目标角由上层把鼠标位移积出来。 */
    float         pitch_angle;

    /** 角度基准是否已建立。见 gimbal_update 里"建基准"那一段的说明。
     *  两个轴各自一份：reset 的时机不同（yaw 还受切模式影响），
     *  共用一份会让"切模式作废 yaw 基准"顺手把 pitch 基准也废掉。 */
    bool          yaw_angle_valid;
    bool          pitch_angle_valid;

    /* ── 环 ──
     * 每路一份独立积分器，理由同 chassis：四个环的误差各走各的，
     * 共用 i_out 会让任何一路的偏差带着其它路一起动。 */
    pid_data      yaw_angle_pid;    /**< 自瞄时的角度外环 */
    pid_data      yaw_speed_pid;    /**< yaw 内环，两种模式都在跑 */
    pid_data      pitch_angle_pid;  /**< pitch 外环 */
    pid_data      pitch_speed_pid;  /**< pitch 内环 */

    /* ── 摩擦轮 ── 每个轮子一份积分器：机械上独立，转速差各修各的 */
    pid_data      friction_pid[GIMBAL_FRICTION_MAX];

    /* ── 反馈：gimbal_update 每拍回填，上层随时可读。无锁，理由见 gimbal_status ── */
    gimbal_status status;
} gimbal_data;

/* ─── 接口（上层统一入口） ───
 *
 * 谁做换系 / 谁积分鼠标？—— 【不是这里】。gimbal 收的就是弧度、弧度每秒。
 * 遥控器给出的是摇杆行程、鼠标给出的是像素位移，那些换算放在 application 层。
 * 理由和 chassis 一致：云台驱动不该知道遥控器长什么样。
 */

/** @brief 云台驱动接口 */
typedef struct {
    /** @brief 使能 / 急停。失能 = 三路输出全部清零 */
    int (*set_enabled)(const struct device *dev, bool enabled);
    /** @brief 切手动 / 自瞄 */
    int (*set_mode)(const struct device *dev, gimbal_mode_t mode);
    /** @brief 手动模式：写 yaw 目标角速度 */
    int (*set_yaw_speed)(const struct device *dev, float rad_s);
    /** @brief 自瞄模式：写 yaw 目标角度（绝对） */
    int (*set_yaw_angle)(const struct device *dev, float rad);
    /** @brief 写 pitch 目标角度（绝对）。两种模式共用 */
    int (*set_pitch_angle)(const struct device *dev, float rad);
    /** @brief 摩擦轮开 / 关（独立按键） */
    int (*set_friction)(const struct device *dev, bool on);
    /** @brief 取一份实测状态快照到调用者的 buffer */
    int (*get_status)(const struct device *dev, gimbal_status *out);
} gimbal_api;

/* ─── 分发函数（上层统一入口） ─── */

/**
 * @brief 使能 / 急停云台
 *
 * 失能时驱动会【把所有积分器一并清掉，并作废两个角度基准】。
 * 清积分器的理由和 chassis 一样（重新使能时带着旧 i_out 起步会直接冲出去）；
 * 作废基准是本驱动多出来的一条，理由见 gimbal_update 里"建基准"那段 ——
 * 简单说：目标角可能是很久以前设的，直接拿去驱动会甩。
 *
 * ★ 但清这个动作【不在这里做】。set_enabled 只翻一个 bool，清积分器和作废基准
 *   由 gimbal_update 看到 enabled 变 false 之后自己动手。
 *   理由：pid_data 是每个环 20 个 float 的结构体，跨线程清它就得和正在算 PID 的
 *   update 抢锁；而让 update 自己清，pid_data 从头到尾只有一个写方，锁根本不需要
 *   存在。多写一行 if，省掉一整把锁。（和 chassis 同一条。）
 *
 * ⚠️ 摩擦轮也受这里管：急停时它【一并停转】，即使 friction_on 还是 true。
 *   friction_on 只是"操作手允许它转"，enabled 是总闸。
 *
 * @param dev     云台设备指针
 * @param enabled false = 输出强制清零
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int gimbal_set_enabled(const struct device *dev, bool enabled) {
    const gimbal_api *api = (const gimbal_api *)dev->api;

    if (api == NULL || api->set_enabled == NULL) {
        return -ENOSYS;
    }
    return api->set_enabled(dev, enabled);
}

/**
 * @brief 切换手动 / 自瞄模式
 *
 * 切到 AUTO 会【作废 yaw 角度基准】，下一拍 gimbal_update 重建它。
 * 必须这样：yaw 的目标角是绝对量，而自瞄系统的第一帧角度可能还没锁上 ——
 * 沿用上一次自瞄留下的旧目标（或者零初始化的 0）会直接甩过去。
 * 所以切完之后【至少有一拍不跟踪】，那一拍只记基准。
 *
 * ★ MANUAL 不动作废，因为手动模式根本不用角度基准。
 *
 * ⚠️ 默认（零初始化）是 MANUAL：上电时摇杆没动、自瞄还没连上，这时最安全的
 *    行为是"不跟踪任何目标"，而不是"追一个不存在的角度"。
 *
 * @param dev  云台设备指针
 * @param mode 目标模式
 * @return 0 成功，-EINVAL 未知模式，-ENOSYS 未实现
 */
static inline int gimbal_set_mode(const struct device *dev, gimbal_mode_t mode) {
    const gimbal_api *api = (const gimbal_api *)dev->api;

    if (api == NULL || api->set_mode == NULL) {
        return -ENOSYS;
    }
    return api->set_mode(dev, mode);
}

/**
 * @brief 手动模式：写 yaw 目标角速度
 *
 * 摇杆量。正的 = 逆时针（从上看）。
 *
 * ⚠️ 只在 MANUAL 下生效。AUTO 下写它不会报错，但【不会起作用】——
 *    那时是角度环在说话。两种模式的输入量纲不同，故意分成两个函数、
 *    不用一个函数带模式判断：写错函数名是编译期就能看见的，
 *    写错隐含语义不是。
 *
 * ⚠️ 这个环【不需要有界】：摇杆回中就是 0，云台自己就停。
 *    （对比 pitch：pitch 是角度量，不回中就会一直追，所以 pitch 必须限幅。）
 *
 * @param dev   云台设备指针
 * @param rad_s 目标角速度（rad/s）
 * @return 0 成功，-ENOSYS 未实现
 */
static inline int gimbal_set_yaw_speed(const struct device *dev, float rad_s) {
    const gimbal_api *api = (const gimbal_api *)dev->api;

    if (api == NULL || api->set_yaw_speed == NULL) {
        return -ENOSYS;
    }
    return api->set_yaw_speed(dev, rad_s);
}

/**
 * @brief 自瞄模式：写 yaw 目标角度（绝对）
 *
 * ⚠️ 只在 AUTO 下生效，同 gimbal_set_yaw_speed 的说明。
 *
 * ★ 这里【不用】自己做跨圈归一。驱动内部按最短弧解算：目标离实测超过 180°
 *   时按另一侧走。所以上位机给 -π..π 的包裹角还是连续角都行 ——
 *   除非你真的想让它【转过半圈以上】，那种情况最短弧会选近的那边。
 *
 * @param dev 云台设备指针
 * @param rad 目标角度（rad，绝对）
 * @return 0 成功，-ENOSYS 未实现
 */
static inline int gimbal_set_yaw_angle(const struct device *dev, float rad) {
    const gimbal_api *api = (const gimbal_api *)dev->api;

    if (api == NULL || api->set_yaw_angle == NULL) {
        return -ENOSYS;
    }
    return api->set_yaw_angle(dev, rad);
}

/**
 * @brief 写 pitch 目标角度（绝对）。两种模式共用
 *
 * ★ 驱动会把它【限幅到 DT 的 pitch-min-deg / pitch-max-deg】，所以上层
 *   不用自己夹 —— 夹在这里是权威的，夹在上层是"每个调用点都要记得夹"。
 *
 *   限幅必须在【目标】上做，不能在输出上做：位置环不会自己停，目标一旦超程，
 *   误差就永远不消，PID 会持续输出堵转电流顶着限位 —— 那是个不会自解的正反馈，
 *   输出限幅拦不住它（顶住时输出本来就是满的，钳不钳都满）。
 *
 * ⚠️ 手动模式下，这个值是上层把鼠标位移【积】出来的绝对角，不是速度。
 *    积的时候不用自己夹限位 —— 反正这里会夹，而且夹在积分后面意味着
 *    "顶到限位之后继续往同方向推，一往回拉立刻就有响应"，手感更对。
 *
 * @param dev 云台设备指针
 * @param rad 目标角度（rad，绝对）
 * @return 0 成功，-ENOSYS 未实现
 */
static inline int gimbal_set_pitch_angle(const struct device *dev, float rad) {
    const gimbal_api *api = (const gimbal_api *)dev->api;

    if (api == NULL || api->set_pitch_angle == NULL) {
        return -ENOSYS;
    }
    return api->set_pitch_angle(dev, rad);
}

/**
 * @brief 摩擦轮开 / 关（遥控器或键盘的独立按键）
 *
 * 【和拨弹盘是两码事】：拨盘由鼠标控制（在 chassis 那边，见 chassis_trigger_set），
 * 摩擦轮只认这个开关。所以鼠标左键不会顺手把摩擦轮带起来 ——
 * 那是两条独立的命令链，各自有各自的按键。
 *
 * 转速来自 DT 的 friction-speed-deg-s，本函数只控制"转不转"。
 *
 * ⚠️ 仍然受 gimbal_set_enabled 一票否决：enabled = false 时不管这里是什么，
 *    摩擦轮都不转。急停就该是全停。[开关 ∧ 使能] 才是实际输出。
 *
 * ⚠️ 关的时候驱动会把摩擦轮的积分器清掉。不清的话，关-开一次，速度环会带着
 *    停机期间攒下的积分起步，那一下比正常启动猛得多。
 *
 * @param dev 云台设备指针
 * @param on  true = 转
 * @return 0 成功，-ENOSYS 本车没配摩擦轮
 */
static inline int gimbal_set_friction(const struct device *dev, bool on) {
    const gimbal_api *api = (const gimbal_api *)dev->api;

    if (api == NULL || api->set_friction == NULL) {
        return -ENOSYS;
    }
    return api->set_friction(dev, on);
}

/**
 * @brief 读取云台实测状态快照
 *
 * @param dev 云台设备指针
 * @param out 输出：实测状态。由调用者持有，驱动不保留指针
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int gimbal_get_status(const struct device *dev, gimbal_status *out) {
    const gimbal_api *api = (const gimbal_api *)dev->api;

    if (api == NULL || api->get_status == NULL) {
        return -ENOSYS;
    }
    return api->get_status(dev, out);
}

/**
 * @brief 云台控制一拍：读 IMU → 外环 → 内环 → 下发电机
 *
 * 由控制线程按固定周期调用（和 chassis_update 同一角色）。【不发 CAN】——
 * 打包和发送是另一条线程的事。
 *
 * ── 反馈全部来自 IMU，电机只当力矩源 ──
 *
 * 两个速度环读 imu_get_gyro_axis，两个角度环（和基准）读 imu_get_angle。
 * 电机自己的编码器在控制回路里【一次都没用】，这是有意的：
 *
 *   用编码器的话，反馈是"云台相对底盘转了多少"。底盘自转时编码器【纹丝不动】
 *   —— 云台环认为"我已经到位了"，于是什么也不做，云台就跟着底盘一起转。
 *   要修就得显式加底盘 wz 补偿。
 *
 *   用 IMU 的话，反馈是"云台相对世界转了多少"。底盘自转时 IMU 立刻看到变化，
 *   速度环直接反向补回来 —— 【解耦不需要一行代码，它是反馈量的定义带来的】。
 *
 * ★ 这条也解释了 chassis.h 里那句"云台的解耦补偿现在只用 wz"：在本驱动里
 *   【已经不成立了】，gimbal 不读 chassis_status.wz，也不持有 chassis 设备。
 *   那句话是在按编码器方案设计时写的，IMU 方案下它是多余的。
 *   （代价：AUTO 模式的 yaw 目标是陀螺积出来的绝对角，长时间会漂；
 *      MANUAL 模式用角速度，不漂。要做"绝对不漂"得引磁力计或视觉重锁。）
 *
 * ── 顺序不能换 ──
 *
 * 先读这一拍的反馈，再算目标，最后才下发。反过来就是把上一拍的目标配这一拍
 * 的反馈 —— 增量式 PID 的 D 项会直接看错，而且错得不像错。
 *
 * @param dev 云台设备指针
 * @param dt  距上次调用时间（秒），必须实测，别用标称值
 */
void gimbal_update(const struct device *dev, float dt);

#endif /* GIMBAL_H */
