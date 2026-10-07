#ifndef CHASSIS_H
#define CHASSIS_H

#include "drivers/motors/motor.h"
#include "drivers/imu/imu.h"
#include "drivers/pid/pid.h"

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdbool.h>

/**
 * 底盘构型。【顺序即契约】：枚举值必须与 skywalker,chassis.yaml 里 type 的
 * enum 顺序逐项对齐 —— chassis.c 用 DT_INST_ENUM_IDX 直接拿它当数组下标查
 * 驱动方向角，往中间插一项会让后面所有构型静默错位。
 */
typedef enum{
    omni =  0,
    mecanum,
    steer,
}chassis_type;

/**
 * @brief 底盘配置（ROM，由 DT 填充）
 *
 * 字段与 dts/bindings/chassis/skywalker,chassis.yaml 逐项对应。
 *
 * 【顺序即契约】wheels / wheel_x / wheel_y / steer_motors 按同一下标对齐，
 * 下标 0 约定为左前轮、逆时针。任一项长度对不上，CHASSIS_DT_ASSERTS 会拦下。
 *
 * 本结构体【故意不含驱动方向角 alpha】：alpha 由 type + 下标推出，表放在
 * chassis.c。而且它只对 omni / mecanum 是常量 —— 舵轮的 alpha 每周期现算，
 * 表根本不参与，所以它不算"这辆车装了什么"这类 DT 事实。
 */
typedef struct {
    chassis_type type;                      /**< 构型 */
    const struct device *wheels[4];         
    float        wheel_x[4];                /**< 车体系 x，车头为正（米） */
    float        wheel_y[4];                /**< 车体系 y，车左为正（米） */
    float        wheel_radius;              /**< 轮半径（米），轮线速度 ↔ 输出轴角速度 */
    /* ★ 功率控制要的转子电流 / 转子转速【不在这里】。
     *   那两个量电机自己的反馈帧里就有，motor_data.current / omega_rotor 已经
     *   填好了（换算用的 gear_ratio、c2t 也都是电机节点的 DT 属性）。
     *   底盘再抄一份 gear_ratio，就有两个必须手动对齐的真相来源 ——
     *   改了一个忘了另一个，编译期不报错，只在功率限幅里悄悄差 19.2 倍。
     *   要哪个量就问电机要，别让底盘替电机记它的常数。 */
    const struct device *steer_motors[4];   /**< 仅 type == steer 非零 */
    const struct device *wheel_pid;         /**< 轮速环增益（各轮共用一组） */
    const struct device *steer_pid;         /**< 舵轮转向位置环增益；不适用时为 NULL */
    /** 姿态设备；没配时为 NULL。
     *
     *  ⚠️ 目前是【死字段】：全驱动只有 chassis_init 里一次 device_is_ready 检查，
     *     控制回路一次都不读它。小陀螺只是 wz 给个非零常量（见 chassis_set_speed），
     *     底盘自己不需要姿态反馈；底盘自转由车轮编码器正解成 status.wz 就够了。
     *     云台那边也【不读它】—— 云台的解耦靠"反馈全用云台 IMU"白拿，见 gimbal.h。
     *     留着是为了 binding 和这辆车已有的 DT 节点不炸，不是因为有用途。 */
    const struct device *imu;
    /* 拨盘。物理位置在底盘上，但逻辑上属于发射 —— 见 binding 里的说明。 */
    const struct device *trigger_motor;     /**< 没配时为 NULL */
    const struct device *trigger_speed_pid; /**< 【连发】速度环增益；NULL = 不支持连发 */
    const struct device *trigger_angle_pid; /**< 【单发】位置环增益；NULL = 不支持单发 */
    float        trigger_angle_deg;         /**< 【单发】每发步进角 = 360 / 拨盘槽数 */
    float        trigger_speed_deg_s;       /**< 【连发】输出轴目标角速度（度/秒）。
                                            *   <= 0 = 不支持连发，切连发会拿到 -ENOSYS。
                                            *   理由同 binding：要的是【转速】不是"发/秒"。 */
} chassis_config;

/**
 * 拨盘发射模式。【运行时状态，不是 DT】
 *
 * 比赛时由拨杆切换，所以不能进 binding —— DT 里的值会被编译成常量烧进 flash，
 * 拨一下拨杆不该需要重编译。binding 里那两个 PID 是"两种模式各自用什么增益"，
 * 是装车事实；这里选"现在用哪个"，是运行时选择。
 */
typedef enum {
    TRIGGER_MODE_SINGLE = 0,   /**< 单发：位置环，每发步进 trigger_angle_deg */
    TRIGGER_MODE_AUTO,         /**< 连发：速度环，恒速连续拨弹 */
} trigger_mode_t;

/**
 * @brief 底盘实测状态【快照】
 *
 * ⚠️ 拿到的不是活数据，是拷贝：之后底盘怎么变都不影响你手上这份。
 *    （对比：mambo 的 get_status 直接返回内部指针，调用者拿到的是驱动内部的
 *      地址，反手就能改状态 —— 分层就没了。这里用 out 参数堵死这条路。）
 *
 * ★ 这里【故意不加锁】，理由是判据要往前挪一步（和 imu.h 讲 gyro 时同一条）：
 *
 *   加锁的判据不是"它们恰好是同一个结构体"，是"这几个值【必须一起正确】"。
 *
 *   - imu 的 angle 要锁：roll 配旧 pitch 是【一个不存在的姿态】，
 *     拿它去算旋转矩阵会得到垃圾。
 *   - 这里不要锁：三个速度各自是独立的物理量，跨拍拼出来的仍然是【一个真实
 *     存在过的速度】，只是误差被 (加速度 × 错拍时间) 限住 —— 1 kHz 下是微秒级。
 *     一个瞬态误差，下一拍自己就纠正了；和一个"不存在的状态"是两回事。
 *
 *   还有一条更硬的理由：这三个量只有 update 线程写，别人只读，
 *   每个 float 都是单次对齐访问，本来就不会读到半个数。
 *
 * ⚠️ 什么时候该回来加锁：哪天有消费者把三个量【合起来用】（比如按 vx,vy 合成
 *    车体系速度矢量、再和 wz 配对做积分），那就和 angle 一样变成"必须同一拍"，
 *    那时再加，别提前加。
 *
 * ⚠️ 加字段不要紧，这个结构体是接口的形状：加字段不动签名，调用方不用改。
 */
typedef struct {
    float vx, vy;   /**< 车体系实测平移速度（m/s） */
    /** 车体系实测自转角速度（rad/s）。
     *  ⚠️ 来源是【四轮编码器正解】，不是 IMU —— 轮子打滑时它会骗人。
     *  目前没有消费者：云台不读它（云台的解耦靠"反馈全用云台 IMU"白拿，见 gimbal.h）。 */
    float wz;
} chassis_status;

/**
 * @brief 底盘运行时数据（RAM）
 *
 * 判据：只放"这一拍算完、下一拍还要知道"的东西。一拍之内算完就丢的量
 * （轮速解算结果 v_i、当拍误差、PID 输出）一律用局部变量，不进这里。
 */
typedef struct {
    /* ── 指令：上层写，控制循环读 ── */
    float vx, vy, wz;          /**< 车体系目标速度（m/s, m/s, rad/s） */
    bool  enabled;             /**< false = 输出强制清零（急停 / 未就绪） */

    /** 本拍允许的底盘总功率上限（W）。
     *  由【读裁判系统的那一层】写，驱动不自己去拿 —— 裁判系统将来是个设备，
     *  底盘不该知道它长什么样。
     *  ⚠️ <= 0 表示【不限制】。零初始化就是 0，如果 0 解释成"限到 0 瓦"，
     *     忘了设就变成车不动 —— 那种故障最难查。宁可失败朝"能动"那边倒。 */
    float power_limit;

    /* ── 轮速环 ──
     * 增益共用一份 config，但积分器必须是 4 份：四个轮子误差各走各的，
     * 共用一个 i_out 会让任何一轮的偏差带着另外三轮一起动。 */
    pid_data wheel_pid[4];

    /** 本拍四轮输出轴力矩（N·m，按 wheels[] 的下标对齐）。
     *  chassis_update 写、【已含轮速环输出、尚未含功率限幅】；
     *  chassis_power_limit 读它、乘 k、下发电机。
     *
     *  ★ 这是 chassis_data 里【唯一】为跨函数传递而存在的字段。按上面那条
     *    "一拍之内算完就丢的量一律用局部变量"的判据，它本该是局部变量 ——
     *    但限幅被拆成独立一步之后，它必须活过函数边界。判据没变，只是
     *    这里多了一条"必须跨步"的事实压过它。
     *
     *  ⚠️ 功率限幅要的电流 / 转子转速【不往这里抄】：那两个量电机自己就存着，
     *     motor_get 一问就有。抄一份就多一个要手动同步的真相来源 ——
     *     理由同 chassis_config 里那段"功率控制要的电流转速不在这里"。
     *     限幅那一步自己现读，读失败就留 0（当"轮子没转"）。
     *
     *  ⚠️ 它是【未限幅】值，不是"要发出去的值"。谁想直接拿它去发，谁就
     *     绕过了限幅 —— 想发就调 chassis_power_limit。 */
    float wheel_torque[4];

    /* ── 舵轮 ── */
    pid_data steer_pid[4];     /**< 转向位置环的 4 份独立积分器。理由同 wheel_pid：
                                *   四个转向角目标各不相同，共用积分器会互相拖。 */

    /* ── 拨盘 ── */
    trigger_mode_t trigger_mode;

    /** 开火电平（【上层写】）。同一个字段在两种模式下语义不同：
     *    单发 —— 取【上升沿】，每次 false→true 累加一个 trigger_angle_deg
     *    连发 —— 取【电平】，true = 恒速起转，false = 停
     *  ★ 上升沿由 chassis_trigger_update 判，上层【不要】自己防连按 ——
     *    防连按要在上层记住上一次按键状态，那个状态一分散到各调用点就会
     *    各写各的；放在驱动里只有一个写方。 */
    bool           trigger_fire;

    /** 上一拍的开火电平。【只有 chassis_trigger_update 写】——
     *  和 trigger_fire 分成两个字段，是为了让"边沿判定"有且只有一个写方：
     *  上层只翻 trigger_fire，本函数读它、比自己记的上一拍、再更新自己记的。
     *  若把边沿判定塞进 chassis_set_trigger()，那里就得【读-改-写】共享状态，
     *  和控制线程抢 —— 一个 bool 换一整把锁，不值。 */
    bool           trigger_fire_last;

    float          trigger_target_deg;    /**< 单发累加目标角（【输出轴】角度，不是电机轴） */
    bool           trigger_target_valid;  /**< 基准是否已建立。
                                           *   单发的目标是"当前角 + 步进角"，所以必须知道
                                           *   当前角。切模式时若直接读编码器会读到 0 ——
                                           *   初始化跑在 POST_KERNEL，CAN 还没有任何反馈。
                                           *   故：标志为 false 时，本拍只建立基准、不设目标。 */
    pid_data       trigger_speed_pid;
    pid_data       trigger_angle_pid;

    /* ── 反馈：chassis_update 每拍回填，上层随时可读。无锁，理由见 chassis_status ── */
    chassis_status status;
} chassis_data;

/* ─── 接口（上层统一入口） ───
 *
 * 谁做换系？—— 【不是这里】。chassis_set_speed 收的就是车体系速度。
 * 遥控器给出的是机体系（以操作手为参照），那一步旋转放在 application 层。
 * 理由和 mambo 一致：底盘驱动不该知道遥控器长什么样，否则换个遥控器就要改驱动。
 */

/** @brief 底盘驱动接口 */
typedef struct {
    /** @brief 写目标速度。小陀螺就是 wz 给一个非零常量，不需要额外模式 */
    int (*set_speed)(const struct device *dev, float vx, float vy, float wz);
    /** @brief 使能/急停。失能 = 输出清零 */
    int (*set_enabled)(const struct device *dev, bool enabled);
    /** @brief 写本拍允许的底盘总功率上限（W）。<= 0 = 不限制 */
    int (*set_power_limit)(const struct device *dev, float watts);
    /** @brief 取一份实测状态快照到调用者的 buffer */
    int (*get_status)(const struct device *dev, chassis_status *out);
    /** @brief 切换发射模式（单发 / 连发）。运行时可切 */
    int (*set_trigger_mode)(const struct device *dev, trigger_mode_t mode);
    /** @brief 写开火电平（单发取上升沿 / 连发取电平） */
    int (*set_trigger)(const struct device *dev, bool fire);
} chassis_api;

/* ─── 分发函数（上层统一入口） ─── */

/**
 * @brief 设置底盘目标速度（车体系）
 *
 * ⚠️ 车体系，不是机体系。遥控器摇杆给的机体系速度要先在 application 层换系。
 *
 * @param dev 底盘设备指针
 * @param vx  车体系目标 x 速度（m/s），车头为正
 * @param vy  车体系目标 y 速度（m/s），车左为正
 * @param wz  车体系目标自转角速度（rad/s），逆时针为正。非零即小陀螺
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int chassis_set_speed(const struct device *dev, float vx, float vy, float wz) {
    const chassis_api *api = (const chassis_api *)dev->api;

    if (api == NULL || api->set_speed == NULL) {
        return -ENOSYS;
    }
    return api->set_speed(dev, vx, vy, wz);
}

/**
 * @brief 使能 / 急停底盘
 *
 * 失能时驱动必须【把积分器一并清掉】——否则重新使能的那一刻，PID 带着失能前
 * 的 i_out 起步，会直接冲出去。清在失能侧就够了：失能后积分器不再更新，
 * 停在零上，下次使能自然从零开始。
 *
 * ★ 但清这个动作【不在这里做】。set_enabled 只翻一个 bool，清积分器由
 *   chassis_update 看到 enabled 变 false 之后自己动手。
 *   理由：pid_data 是四个轮子 × 20 个 float 的结构体，跨线程清它，就得和正在
 *   算 PID 的 update 抢锁；而让 update 自己清，pid_data 从头到尾只有一个写方，
 *   锁根本不需要存在。多写一行 if，省掉一整把锁。
 *
 * @param dev     底盘设备指针
 * @param enabled false = 输出强制清零
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int chassis_set_enabled(const struct device *dev, bool enabled) {
    const chassis_api *api = (const chassis_api *)dev->api;

    if (api == NULL || api->set_enabled == NULL) {
        return -ENOSYS;
    }
    return api->set_enabled(dev, enabled);
}

/**
 * @brief 写本拍允许的底盘总功率上限（W）
 *
 * 这是【读裁判系统的那一层】唯一该碰的入口。驱动不自己去拿裁判系统读数 ——
 * 裁判系统将来是个设备（串口帧），底盘不该知道它长什么样，理由同
 * chassis_set_speed 上面那段"谁做换系不是这里"。
 *
 * ⚠️ 单位是瓦，且是【底盘总功率】：`chassis_power_scale` 会把四个轮子的功耗
 *    加在一起比它。所以传进来的应该是裁判系统给的整车底盘功率上限，
 *    不是单轮上限。
 *
 * ⚠️ `<= 0` 表示【不限制】（见 chassis_data.power_limit）。零初始化就是 0，
 *    如果 0 解释成"限到 0 瓦"，忘了设就变成车不动 —— 那种故障最难查。
 *    宁可失败朝"能动"那边倒。
 *
 * ★ 不加锁：一个 float，单次对齐 store，读方（chassis_power_limit）拿到的
 *   要么是旧值要么是新值，不存在"半个数"。裁判系统 10 Hz 更新，控制线程
 *   最坏晚 1 ms 看到新值 —— 对热和判罚都无所谓。
 *
 * @param dev   底盘设备指针
 * @param watts 允许的总功率上限（W）。<= 0 = 不限制
 * @return 0 成功，-ENOSYS 未实现
 */
static inline int chassis_set_power_limit(const struct device *dev, float watts) {
    const chassis_api *api = (const chassis_api *)dev->api;

    if (api == NULL || api->set_power_limit == NULL) {
        return -ENOSYS;
    }
    return api->set_power_limit(dev, watts);
}

/**
 * @brief 读取底盘实测状态快照
 *
 * @param dev 底盘设备指针
 * @param out 输出：实测状态。由调用者持有，驱动不保留指针
 * @return 0 成功，-ENOSYS 未实现，其他负值为驱动错误码
 */
static inline int chassis_get_status(const struct device *dev, chassis_status *out) {
    const chassis_api *api = (const chassis_api *)dev->api;

    if (api == NULL || api->get_status == NULL) {
        return -ENOSYS;
    }
    return api->get_status(dev, out);
}

/**
 * @brief 切换发射模式（单发 / 连发）
 *
 * 运行时可切，拨杆一动就调它 —— 模式不是 DT 事实，见 trigger_mode_t 的说明。
 *
 * ⚠️ 切到单发会【把角度基准作废】，下一拍 chassis_trigger_update 重建它。
 *    必须这样：单发的目标是"累加基准 + k·步进角"，而刚切过来时基准还停在
 *    上一次单发留下的位置上 —— 沿用它会一步跳出去几十圈。
 *    所以切完之后【至少有一拍不能开火】，那一拍只记基准、不出力。
 *
 * @param dev  底盘设备指针
 * @param mode 目标模式
 * @return 0 成功；-EINVAL 未知模式；-ENOSYS 本车没配拨盘，或该模式缺件
 *         （连发要 trigger-speed-pid + trigger-speed-deg-s，
 *           单发要 trigger-angle-pid + trigger-angle-deg）
 */
static inline int chassis_set_trigger_mode(const struct device *dev, trigger_mode_t mode) {
    const chassis_api *api = (const chassis_api *)dev->api;

    if (api == NULL || api->set_trigger_mode == NULL) {
        return -ENOSYS;
    }
    return api->set_trigger_mode(dev, mode);
}

/**
 * @brief 写开火电平
 *
 * 【写的是电平，不是事件】—— 照原样每拍调就行，不用自己防连按、不用自己判边沿。
 * 上层从遥控器/上板拿到的是"这个键此刻按没按下"，那个量直接传进来就是对的：
 *
 *   单发：驱动取【上升沿】。按住不放只出一发，松开再按才出第二发。
 *   连发：驱动取【电平】。按住恒速转，松开停。
 *
 * ★ 为什么电平比事件好：事件要求上层自己维护"上次按没按"，那个状态一旦在
 *   多个调用点各存一份就会不一致（A 点发了 B 点不知道）。电平是无状态的，
 *   驱动的 trigger_fire_last 是唯一一份边沿状态。
 *
 * 急停（chassis_set_enabled(false)）会一并停火，不用另外调这里。
 *
 * @param dev  底盘设备指针
 * @param fire true = 按下
 * @return 0 成功；-ENOSYS 本车没配拨盘
 */
static inline int chassis_trigger_set(const struct device *dev, bool fire) {
    const chassis_api *api = (const chassis_api *)dev->api;

    if (api == NULL || api->set_trigger == NULL) {
        return -ENOSYS;
    }
    return api->set_trigger(dev, fire);
}

/**
 * @brief 底盘控制一拍：实测 → 逆解 → 四轮速环 → 存进 wheel_torque[]
 *
 * 由控制线程按固定周期调用（和 motor_dji_test 里那个
 * motor_control_calculate_thread 同一角色）。
 *
 * 【本函数不下发驱动轮】—— 算出来的力矩存进 chassis_data.wheel_torque[]，
 * 由 chassis_power_limit 限幅之后再 motor_set。理由见那个函数。
 *
 * 顺序不能换：先读实测（这一拍的反馈），再算目标，最后才存。反过来就是把
 * 上一拍的目标配这一拍的反馈。
 *
 * ⚠️ 失能时这里会【清掉四个积分器】、把 wheel_torque[] 也清零、并直接向四个
 *    驱动轮和（舵轮构型的）四个转向电机下发零扭矩。清积分器的动作故意不放在
 *    chassis_set_enabled 里：pid_data 只有本函数一个写方，自己清什么都不用加锁。
 *
 *    ★ 失能是唯一一条不经过 chassis_power_limit 的下发路径，这是有意为之：
 *      急停不该等一个功率限幅器，而且限幅器在失能时本来就无事可做。
 *
 * @param dev 底盘设备指针
 * @param dt  距上次调用时间（秒），必须实测，别用标称值
 */
void chassis_update(const struct device *dev, float dt);

/**
 * @brief 功率限幅 + 下发驱动轮 —— 底盘这一拍的【最后一道闸】
 *
 * 由控制线程在 chassis_update() 【之后】调用：
 *
 *     chassis_update(dev, dt);        // 算  → wheel_torque[]
 *     chassis_power_limit(dev);       // 限 + 发
 *     chassis_trigger_update(dev, dt);// 拨盘（另一条独立闭环，限幅不管它）
 *
 * ── 为什么拆成两步 ──
 *
 * 原来是"算 → 限 → 发"全在 chassis_update 里，限幅夹在控制回路内部。位置上
 * 看着也在下发前，但结构上它不是【闸】，只是一个步骤：以后任何新加进来的
 * 力矩生产者（超电 boost、上层覆盖、底盘随动）只要写在限幅那几行之后，
 * 就绕过了它，而且绕过时没有任何编译期或运行期提示。
 *
 * 拆开之后限幅是驱动轮的【最后一个写方】，上游怎么变都绕不过去。这是
 * USTC RoboWalker 的 Chassis_Control → Power_Limit → CAN_cmd_chassis
 * 三段式在 skywalker 里的形状（这里的"发"是 motor_set，真正打包发送是
 * 另一条线程，见 motor_dji_test 的结构）。
 *
 * ⚠️ "最终下发前"的准确含义：本函数 motor_set 之后，到 1ms 发送线程
 *    motor_can_pack 之间还隔着最多一拍。但那 1 ms 里【没有别的写方】，
 *    所以"限幅之后的值 = 最终发出的值"成立。真要挪进发送线程，代价是通用
 *    发送线程得持有 chassis 设备指针，还得给 motor_api 加一个"读回目标力矩"
 *    的接口（现在没有：motor_data.torque 是实测值）—— 不值得。
 *
 * ── 调用契约 ──
 *
 * ⚠️ 忘了调本函数，后果是【四个驱动轮完全不下发】—— 目标停在 0，车不动。
 *    这个方向是故意选的：失败朝"不动"倒，不朝"失控"倒。车轮不动是上电就
 *    看得见的显眼故障；反过来（漏调之后轮子还带着上一拍或未限幅的力矩跑）
 *    是上电看不出来的，得等撞了才知道。
 *
 * ⚠️ 失能时本函数【直接返回】。零输出由 chassis_update 的失能分支负责，
 *    两条路径只能有一条负责"输出" —— 否则这里会拿 wheel_torque[] 的陈旧值
 *    把刚刹停的轮子重新点着。
 *
 * ── 限幅怎么算 ──
 *
 * 读四轮【上拍实测】的转子电流和转子转速，解出缩放系数 k，四轮【同乘一个 k】
 * 之后下发。同乘而不是各自削减，是为了保住合力方向 —— 各自削减会把车
 * "扭"向意料之外的方向（车会走，就是不走直线）。推导和系数见
 * chassis_power_scale。
 *
 * ★ 本函数【幂等】：同一拍调两次，第二次下发的值和第一次逐位相同。
 *   所以这里绝不能写成 `wheel_torque[i] *= k` —— 那样 k 会连乘，而且乘完
 *   不可逆，限幅器就变成了一个每次调用都衰减一截的东西。用局部值乘。
 *
 * ★ 没有 dt：本函数不做积分，算的是"这一刻"的缩放系数。功率模型要的是
 *   瞬时的 I 和 ω，不是它们的时间积分。
 *
 * @param dev 底盘设备指针
 */
void chassis_power_limit(const struct device *dev);

/**
 * @brief 拨弹盘控制一拍：单发位置环 / 连发速度环
 *
 * 【和 chassis_update 分开，不是漏了合并】：两者是两条独立的闭环，控的是两个
 * 不相干的执行器（轮子 vs 拨盘），共用一个函数只会让"底盘这一拍"变成
 * "底盘 + 发射这一拍"，以后想给拨盘换个周期就得连着底盘一起改。
 *
 * 调用方式和 chassis_update 一样，从【同一条控制线程】按同一周期调即可：
 *
 *     chassis_update(dev, dt);
 *     chassis_power_limit(dev);        // ★ 必须跟在这个位置，见它的说明
 *     chassis_trigger_update(dev, dt);
 *
 * 两条闭环各自 motor_set 各自的电机，互不干涉；拨盘和限幅谁先谁后都行。
 *
 * ⚠️ 【拨盘不进底盘的功率预算】：chassis_power_limit 只管四个驱动轮，本函数
 *    照旧自己 motor_set。裁判系统上底盘和发射是分开计功率的，而且拨盘不该
 *    因为底盘超功率就被削 —— 那会让车打得时快时慢。真要给拨盘限功率，
 *    是发射那条链自己的事，别塞进底盘限幅器。
 *
 * 【没配拨盘时整个函数是空操作】—— 上层可以无脑每拍调，不用先查有没有配。
 *
 * ⚠️ 使能（chassis_set_enabled）同时管住拨盘：失能时本函数清零扭矩，
 *    并把角度基准作废，重新使能后要重新建基准才能再开火。
 *
 * @param dev 底盘设备指针
 * @param dt  距上次调用时间（秒），必须实测，别用标称值
 */
void chassis_trigger_update(const struct device *dev, float dt);

#endif