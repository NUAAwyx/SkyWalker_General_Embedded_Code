#include "drivers/chassis/chassis.h"

#include <math.h>

#define DT_DRV_COMPAT skywalker_chassis

/* ─── 从 DT 填充 chassis_config（ROM） ───
 *
 * DT 只能存 string，数值靠 DT_STRING_UNQUOTED 去掉引号得到数值字面量、再
 * (float) 强转成编译期常量 —— 和 pid.c 同一套做法，区别只是这里落在数组元素上，
 * 要用 _BY_IDX 版本。
 *
 * 属性全部 required: false 之后，"漏配"不再有编译期报错，下面那组
 * CHASSIS_DT_ASSERTS 是唯一拦得住的地方。
 */

/** 属性长度，缺失当 0 —— 用于属性可能不存在时安全地做长度比较 */
#define CHASSIS_PROP_LEN(inst, prop) DT_PROP_LEN_OR(DT_DRV_INST(inst), prop, 0)

/** 数组元素：第 idx 个轮子 / 转向电机的设备
 *
 * ⚠️ 末尾那个逗号【是必须的，不是手滑】。
 *    DT_FOREACH_PROP_ELEM 生成的是 fn(…,0) fn(…,1) fn(…,2) fn(…,3) ——
 *    元素之间【只有空白，没有逗号】。逗号得由逐元素宏自己吐。
 *    少了它，初始化列表就成了 `expr expr expr` 并置，gcc 会把它读成
 *    "拿第一个表达式的值当函数调用"，报的却是
 *    "called object is not a function or function pointer" ——
 *    错误信息指向的是 DT_STRING_UNQUOTED_BY_IDX，离真正的原因十万八千里。
 *    这个坑实测踩过，见下面各数组用法的注释。 */
#define CHASSIS_DEV_BY_IDX(node_id, prop, idx) \
    DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

/** 数组元素：第 idx 个数值（wheel-x / wheel-y）。尾逗号理由同上 */
#define CHASSIS_FLOAT_BY_IDX(node_id, prop, idx) \
    ((float)DT_STRING_UNQUOTED_BY_IDX(node_id, prop, idx)),

/** 选填标量 phandle：DT 里没写就是 NULL。Zephyr 没有 DT_INST_PHANDLE_OR */
#define CHASSIS_OPT_DEV(inst, prop)                                   \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                    \
                (DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), prop))), \
                (NULL))

/** 选填标量数值：DT 里没写就是 0.0f */
#define CHASSIS_OPT_FLOAT(inst, prop)                                 \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                    \
                ((float)DT_STRING_UNQUOTED(DT_DRV_INST(inst), prop)), \
                (0.0f))

/** 选填数组：DT 里没写就是全 NULL 的零数组。
 *
 * ★ 用法必须是 .field = { CHASSIS_OPT_DEVS(...) } —— 外面那层花括号【不能少】。
 *   本宏吐的是"逗号分隔的元素列表"（属性不在时吐一个 0），花括号由使用点提供。
 *
 * ⚠️ 别想着把花括号包进宏里、写成 ({ ... })：那是【语句表达式】,在函数外的
 *   静态初始化列表里是非法的（braced-group within expression allowed only
 *   inside a function）。已实测确认。
 *
 * ⚠️ 更要命的是这种错【在没有 DT 节点时根本不会暴露】：本宏只在
 *   CHASSIS_DT_CONFIG_INST_INIT 里展开，而那个宏要等 DT_INST_FOREACH_STATUS_OKAY
 *   至少有一个节点才会展开。所以它会一路静默潜伏，直到你第一次写出
 *   skywalker,chassis 节点的那一刻才炸。 */
#define CHASSIS_OPT_DEVS(inst, prop)                                             \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                               \
                (DT_INST_FOREACH_PROP_ELEM(inst, prop, CHASSIS_DEV_BY_IDX)),     \
                (0))

/** 编译期契约检查：在 chassis.c 里对每个实例调一次 */
#define CHASSIS_DT_ASSERTS(inst)                                                        \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, wheels),                                   \
                 "chassis 必须有 wheels —— 没有轮子不成底盘");                              \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, type),                                     \
                 "chassis 必须有 type(omni / mecanum / steer)");                        \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, wheel_x) &&                                \
                 DT_INST_NODE_HAS_PROP(inst, wheel_y),                                  \
                 "chassis 必须有 wheel-x 和 wheel-y");                                    \
    BUILD_ASSERT(CHASSIS_PROP_LEN(inst, wheels) == CHASSIS_PROP_LEN(inst, wheel_x),     \
                 "wheels 与 wheel-x 长度不等：两者按同一下标对齐，长度必须一致");               \
    BUILD_ASSERT(CHASSIS_PROP_LEN(inst, wheels) == CHASSIS_PROP_LEN(inst, wheel_y),     \
                 "wheels 与 wheel-y 长度不等：两者按同一下标对齐，长度必须一致");               \
    BUILD_ASSERT(CHASSIS_PROP_LEN(inst, wheels) == 4,                                   \
                 "轮数必须是 4：结构体里 wheels/wheel_x/wheel_y/steer_motors 都按 4 开");    \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, wheel_radius),                             \
                 "chassis 必须有 wheel-radius");                                          \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, wheel_pid),                                \
                 "chassis 必须有 wheel-pid");                                           \
    BUILD_ASSERT((chassis_type)DT_INST_ENUM_IDX(inst, type) != steer ||                 \
                 DT_INST_NODE_HAS_PROP(inst, steer_motors),                             \
                 "type = steer 时必须给 steer-motors");                                \
    BUILD_ASSERT((chassis_type)DT_INST_ENUM_IDX(inst, type) != steer ||                 \
                 DT_INST_NODE_HAS_PROP(inst, steer_pid),                                \
                 "type = steer 时必须给 steer-pid");                                    \
    BUILD_ASSERT(CHASSIS_PROP_LEN(inst, steer_motors) == 0 ||                           \
                 CHASSIS_PROP_LEN(inst, steer_motors) == CHASSIS_PROP_LEN(inst, wheels),\
                 "steer-motors 与 wheels 长度不等")

/** chassis_config 的 DT 初始化列表 */
#define CHASSIS_DT_CONFIG_INST_INIT(inst)                                                      \
    {                                                                                          \
        .type              = (chassis_type)DT_INST_ENUM_IDX(inst, type),                       \
        .wheels            = { DT_INST_FOREACH_PROP_ELEM(inst, wheels, CHASSIS_DEV_BY_IDX) },  \
        .wheel_x           = { DT_INST_FOREACH_PROP_ELEM(inst, wheel_x, CHASSIS_FLOAT_BY_IDX) },\
        .wheel_y           = { DT_INST_FOREACH_PROP_ELEM(inst, wheel_y, CHASSIS_FLOAT_BY_IDX) },\
        .wheel_radius      = (float)DT_STRING_UNQUOTED(DT_DRV_INST(inst), wheel_radius),       \
        .steer_motors      = { CHASSIS_OPT_DEVS(inst, steer_motors) },                         \
        .wheel_pid         = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), wheel_pid)),          \
        .steer_pid         = CHASSIS_OPT_DEV(inst, steer_pid),                                 \
        .imu               = CHASSIS_OPT_DEV(inst, imu),                                       \
        .trigger_motor     = CHASSIS_OPT_DEV(inst, trigger_motor),                             \
        .trigger_speed_pid = CHASSIS_OPT_DEV(inst, trigger_speed_pid),                         \
        .trigger_angle_pid = CHASSIS_OPT_DEV(inst, trigger_angle_pid),                         \
        .trigger_angle_deg = CHASSIS_OPT_FLOAT(inst, trigger_angle_deg),                       \
        .trigger_speed_deg_s = CHASSIS_OPT_FLOAT(inst, trigger_speed_deg_s),                   \
    }

/** chassis_config 的静态实例（ROM）。
 *  CHASSIS_DT_ASSERTS 就放在这里展开 —— 它是那组契约检查【唯一的调用点】。
 *  这个宏每个实例被调用一次，检查也就每个实例跑一次。 */
#define CHASSIS_CONFIG_DEFINE(inst)                                    \
    CHASSIS_DT_ASSERTS(inst);                                          \
    static const chassis_config chassis_config_##inst =               \
        CHASSIS_DT_CONFIG_INST_INIT(inst);

/** chassis_data 的静态实例（RAM）。
 *  全零即正确的初始态：vx/vy/wz = 0、enabled = false、四个轮速环与四个转向环的
 *  积分器都归零、trigger_mode = TRIGGER_MODE_SINGLE、拨盘基准未建立。
 *  静态存储期由 C 保证零初始化，所以不写初始化列表。 */
#define CHASSIS_DATA_DEFINE(inst) \
    static chassis_data chassis_data_##inst;

/**
 * @brief 底盘初始化：确认 DT 里点名的设备全都就绪。
 *
 * 这里【只做校验】，不碰控制 —— 轮速解算和 PID 都跑在控制线程里。
 * 特别地，不在这里读编码器建立拨盘基准：init 跑在 POST_KERNEL，那时 CAN 上
 * 一个反馈帧都还没有，读到的必然是 0（理由见 chassis_data.trigger_target_valid）。
 *
 * 选填设备用 NULL 表示"没装"，不装不算错误。所以必须【先判 NULL 再判就绪】——
 * device_is_ready(NULL) 返回 false（kernel/device.c:192），直接用会把"没装"
 * 误判成"故障"。
 *
 * @return 0 成功；-ENODEV 某设备没就绪；-EINVAL wheel-radius 不是正数
 */
static int chassis_init(const struct device *dev) {
    const chassis_config *cfg = dev->config;

    if (cfg->wheel_radius <= 0.0f) {
        return -EINVAL;
    }

    /* 必需：四个轮电机 + 轮速环增益。CHASSIS_DT_ASSERTS 保证这些一定配了。 */
    for (size_t i = 0; i < ARRAY_SIZE(cfg->wheels); i++) {
        if (!device_is_ready(cfg->wheels[i])) {
            return -ENODEV;
        }
    }
    if (!device_is_ready(cfg->wheel_pid)) {
        return -ENODEV;
    }

    /* 仅舵轮构型：四个转向电机 + 转向位置环增益，同样由 asserts 保证配齐 */
    if (cfg->type == steer) {
        for (size_t i = 0; i < ARRAY_SIZE(cfg->steer_motors); i++) {
            if (!device_is_ready(cfg->steer_motors[i])) {
                return -ENODEV;
            }
        }
        if (!device_is_ready(cfg->steer_pid)) {
            return -ENODEV;
        }
    }

    /* 选填：没配是正常的，配了却没就绪才是故障 */
    if ((cfg->imu != NULL && !device_is_ready(cfg->imu)) ||
        (cfg->trigger_motor != NULL && !device_is_ready(cfg->trigger_motor)) ||
        (cfg->trigger_speed_pid != NULL && !device_is_ready(cfg->trigger_speed_pid)) ||
        (cfg->trigger_angle_pid != NULL && !device_is_ready(cfg->trigger_angle_pid))) {
        return -ENODEV;
    }

    return 0;
}

/* ─── 接口实现 ─── */

/** 只写三个 float，每次都是单条对齐 store，所以读方不会读到半个数。
 *  三个量之间不需要"同一拍"（理由见 chassis_status），所以这里不上锁。 */
static int chassis_driver_set_speed(const struct device *dev, float vx, float vy, float wz) {
    chassis_data *data = dev->data;

    data->vx = vx;
    data->vy = vy;
    data->wz = wz;
    return 0;
}

/** 只翻一个 bool。真正的"清积分器"在 chassis_update 里 —— 那里才是
 *  pid_data 唯一的写方，见 chassis.h 上这段的说明。 */
static int chassis_driver_set_enabled(const struct device *dev, bool enabled) {
    chassis_data *data = dev->data;

    data->enabled = enabled;
    return 0;
}

/** 只写一个 float，单次对齐 store，理由同 set_speed。
 *
 *  写方是【读裁判系统的那一层】（10 Hz 级别），读方是控制线程里的
 *  chassis_power_limit。不加锁：最坏晚 1 ms 看到新值，对热和判罚都无所谓。
 *
 *  ★ 这里【不校验】watts 的大小，也【不存一份自己的副本】。负数和 0 是合法
 *    输入，含义是"不限制"（见 chassis_data.power_limit）—— 限到负数瓦和
 *    限到 0 瓦都不是有意义的请求，把它们一起解释成"不限制"，比在这里
 *    悄悄改成 0 好：后者会让"裁判系统还没上线"变成"车不能动"。 */
static int chassis_driver_set_power_limit(const struct device *dev, float watts) {
    chassis_data *data = dev->data;

    data->power_limit = watts;
    return 0;
}

/** 拷一份出去。写方只有一个（chassis_update），读方拿到的是自己栈上的副本，
 *  所以 struct 赋值这一下不会和写方打架 —— 最坏情况是拿到上一拍的值，那正是
 *  "快照"该有的语义。 */
static int chassis_driver_get_status(const struct device *dev, chassis_status *out) {
    const chassis_data *data = dev->data;

    *out = data->status;
    return 0;
}

/** 切模式。这里【只写模式 + 作废基准】，不碰 pid_data —— 那一条仍然是
 *  "pid_data 只有 chassis_update / chassis_trigger_update 两个写方"的延伸：
 *  跨线程清积分器要抢锁，交给控制线程自己清就不用锁（同 set_enabled）。
 *
 *  ★ 不支持的模式【直接拒绝】而不是默默收下在控制回路里变成空转：
 *    调用点拿得到 -ENOSYS 才能知道自己点的这个功能本车没有。
 *    提前 return 也保证不会"模式改了、增益没配"这种半吊子状态。 */
static int chassis_driver_set_trigger_mode(const struct device *dev, trigger_mode_t mode) {
    const chassis_config *cfg = dev->config;
    chassis_data *data = dev->data;

    if (cfg->trigger_motor == NULL) {
        return -ENOSYS;
    }

    if (mode == TRIGGER_MODE_SINGLE) {
        if (cfg->trigger_angle_pid == NULL || cfg->trigger_angle_deg <= 0.0f) {
            return -ENOSYS;
        }
    } else if (mode == TRIGGER_MODE_AUTO) {
        if (cfg->trigger_speed_pid == NULL || cfg->trigger_speed_deg_s <= 0.0f) {
            return -ENOSYS;
        }
    } else {
        return -EINVAL;
    }

    data->trigger_mode = mode;

    /* 切到单发：基准作废，下一拍重建。理由见 chassis.h 上这段的说明 ——
     * 单发的目标是"基准 + k·步进角"，刚切过来时基准还停在上一次建的位置上。
     * 这一个 bool 允许从上层线程写：单次对齐 store，和 set_enabled 同一条。 */
    if (mode == TRIGGER_MODE_SINGLE) {
        data->trigger_target_valid = false;
    }

    return 0;
}

/** 只翻一个 bool。边沿判定在 chassis_trigger_update 里 —— 那里是
 *  trigger_fire_last 唯一的写方，见 chassis.h 上这段的说明。 */
static int chassis_driver_set_trigger(const struct device *dev, bool fire) {
    const chassis_config *cfg = dev->config;
    chassis_data *data = dev->data;

    if (cfg->trigger_motor == NULL) {
        return -ENOSYS;
    }

    data->trigger_fire = fire;
    return 0;
}

static const chassis_api chassis_driver_api = {
    .set_speed       = chassis_driver_set_speed,
    .set_enabled     = chassis_driver_set_enabled,
    .set_power_limit = chassis_driver_set_power_limit,
    .get_status      = chassis_driver_get_status,
    .set_trigger_mode = chassis_driver_set_trigger_mode,
    .set_trigger     = chassis_driver_set_trigger,
};

/* ═══════════════════ 底盘控制一拍 ═══════════════════ */

/* 轮数。CHASSIS_DT_ASSERTS 已经把 wheels 的长度钉死在 4，这里只是给
 * 栈上数组一个编译期长度 —— 两处必须一致，所以不写 ARRAY_SIZE(cfg->wheels)：
 * 那种写法进了循环会带出 int/size_t 比较警告，而且它是"运行时读到的 4"，
 * 不是结构体布局的一部分。 */
#define CHASSIS_WHEEL_NUM 4

/* 不用 <math.h> 的 M_PI：picolibc 里它受 __BSD_VISIBLE 保护，-std=c17 下不可见。
 * 同一个坑 motor_dji.c 里已经踩过一次，它自己 define 了 MOTOR_DJI_PI。 */
#define CHASSIS_DEG_TO_RAD 0.017453292519943295f
#define CHASSIS_RAD_TO_DEG (1.0f / CHASSIS_DEG_TO_RAD)
#define CHASSIS_PI         3.14159265359f
#define CHASSIS_TWO_PI     (2.0f * CHASSIS_PI)
#define CHASSIS_PI_2       (0.5f * CHASSIS_PI)

/* 轮心速度低于这个值（m/s）就认为"没在动"，此时舵轮的方向没有意义。
 * 0.01 m/s ≈ 1 cm/s，比摩擦能让车爬行的速度还小，所以它只会在真正静止时命中。 */
#define CHASSIS_STEER_EPS 0.01f

/* X 型布局第 i 个轮子的驱动方向角（度）：−45° / +45° / −45° / +45°。
 *
 * ★ 别看这组数像"随便取的 ±45"，它是【对角轮平行】这条物理事实的编码：
 *   左前(i=0) 和 右后(i=2) 都是 −45°，左后(i=1) 和 右前(i=3) 都是 +45°。
 *   从车上看，四个轮子的驱动方向线正好摆成一个 X。
 *   ✅ 这组符号已对实车确认过（对角轮确实平行），不是纸上推的 —— 所以它
 *      留在 .c 里硬编码，不进 DT。辊子手性装反时整张表要镜像成 +45/−45，
 *      那才是需要动它的时刻。
 *
 * ⚠️ 一个非常自然、但【会让底盘转不动】的写错方式：
 *        α_i = 45° + 90°·i          ← 错
 *   这组数看上去更"规整"（四个方向均匀铺开 360°），但它把每个轮子的驱动
 *   方向线都穿过了车心。此时自转系数
 *        d_i = x_i·sinα_i − y_i·cosα_i
 *   恒等于 0（和 a、b 具体取多少无关），也就是【每个轮子都只能平移、不能
 *   出力矩】—— 底盘原地转不了，而平移一切正常。
 *   症状是"车能开、就是不能转"，不看 d 那一列根本想不到是 α 给错了。
 *   下面 chassis_read_alpha 的注释里留了自检方法：算 d，四个都为 0 就是它。
 *
 * omni 和 mecanum 共用这张表 —— 理由见 chassis_read_alpha。 */
#define CHASSIS_X_ALPHA_DEG(i) (((i) % 2 == 0) ? -45.0f : 45.0f)

/* ─── 功率模型系数 ───
 * P = k1·I·ω + k2·I² + k3·ω² + k4，逐电机求和。
 * 取自 H7-Framework 发布的一组 M3508 拟合值。它的 current_convert 也是
 * 20/16384，和本驱动的 MOTOR_DJI_CURRENT_LSB 同一个量程，所以单位对得上 ——
 * 这是"能当起点"的前提，量程不一样的话抄过来直接错一个常数倍。
 *
 * ⚠️ 这是【标定起点，不是真值】。四个数里只有 k2 有直接的物理意义
 *    （铜损 = I²R，0.194 对应转子等效电阻约 0.19Ω）；k1 名义上是 kt 但拟合值
 *    比 0.018 小（把效率折进去了），k3 是铁损，k4 是静态开销。换电机、换电调、
 *    换电池电压都要重量。
 *    唯一的真值是裁判系统读数 —— 本项目【现在没有裁判系统】，所以这组数目前
 *    只保证"限得住"，不保证"限得准"。上真车前必须堵转 + 跑台架重新标。
 *
 * 量级自检（I = 10A、ω_转子 = 500 rad/s，接近满载）：
 *   k1·I·ω = 78.8 W（机械输出）   k2·I²  = 19.4 W（铜损）
 *   k3·ω²  =  4.8 W（铁损）       k4     =  1.15 W（静态）
 *   合计约 104 W —— 和 M3508 在 24V 下的输入功率同量级，对得上。 */
#define CHASSIS_PWR_K1 1.5756e-2f
#define CHASSIS_PWR_K2 1.94e-1f
#define CHASSIS_PWR_K3 1.9202e-5f
#define CHASSIS_PWR_K4 1.15f

/**
 * @brief 取四个轮子【这一拍】的驱动方向角 α（rad，车体系，逆时针为正）
 *
 * α 是"轮子能出力推往哪个方向"，不是"轮子轴指向哪" —— 逆解要把车体系速度
 * 投影到这个方向上。
 *
 * omni / mecanum：X 型布局，α 由 CHASSIS_X_ALPHA_DEG(i) 给出（±45° 交替，
 *   对角轮平行）。下标 0 约定左前、逆时针，见 chassis_config。
 *   ★ 两种构型共用同一张表：全向轮和麦轮的区别在【辊子】怎么装，不在力推往
 *     哪个方向 —— 两者在地面内都只能沿驱动方向出力，辊子的作用是让自由分量
 *     滑掉而不是改变出力方向。对运动学来说它们是同一个 X 型底盘。
 *     （真正让两者不同的是打滑和效率，那属于标定，不属于解算。）
 *   ★ 自检方法：算 d_i = x_i·sinα_i − y_i·cosα_i，四个 d 全为 0 说明 α 表把
 *     驱动方向线都摆过车心了，底盘转不动。正常应该四个同号且非零。
 *
 * steer：α 是【这一拍实测的转向角】—— 转向电机此刻指哪，轮子就只能往哪出力。
 *   所以这里必须去读设备，不能查表。
 *   ★ 注意这里读的是"现在朝哪"。舵轮还需要"该朝哪"，那个由
 *     chassis_steer_alpha() 算 —— 函数分开是有意的，因为两者性质完全不同：
 *     一个是观测量，一个是决策量，混在一个函数里会看不出区别。
 */
static void chassis_read_alpha(const chassis_config *cfg, float alpha[CHASSIS_WHEEL_NUM]) {
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        if (cfg->type == steer) {
            motor_data sm;

            alpha[i] = (motor_get(cfg->steer_motors[i], &sm) == 0) ? sm.angle : 0.0f;
        } else {
            alpha[i] = CHASSIS_X_ALPHA_DEG(i) * CHASSIS_DEG_TO_RAD;
        }
    }
}

/**
 * @brief 舵轮专有：算出四个轮子【该朝哪】—— 转向角目标
 *
 * ★ 这是 steer 和 omni/mecanum 真正的分水岭，不是"多一张 α 表"：
 *
 *   omni / mecanum：α 是【输入】。轮子怎么装的是既成事实，查表就有。
 *     所以解算的形状是"把车想要的速度【投影】到 α 上"，方程欠定要解最小二乘。
 *
 *   steer：α 是【输出】。那是这一拍要算出来、再命令下去的东西。转向轮能指向
 *     任何方向，自由度是够的，所以【不该去投影】—— 该做的是先算出轮心速度
 *     【矢量】，再让轮子指向它。角度和速度来自同一个矢量，天然自洽。
 *
 *   两者前两步一模一样（都是 平移 + ω×r），分岔只在最后一步：
 *     投影式：α 已知 → 求标量 ω_i
 *     舵轮式：先由矢量定 α → 再用同一个公式求 ω_i
 *
 *   所以本函数只吐 α，轮速仍旧交给 chassis_inverse_kinematics 算 —— 那不是
 *   偷懒，是"α 定下来之后两者就是同一个问题"这件事本身。
 *
 * ⚠️ 前提：转向电机的机械零位要和车体系 α=0 对齐（都是车头方向）。
 *    mambo 那里是 `+ 90.0f` 这个偏置在干这件事，它的零位定义不一样。
 *    如果装车时模块是转着装上去的，要么机械上对零，要么这里加偏置 ——
 *    别用软件去凑一个本该在机械上解决的问题，那会让每次换模块都要改代码。
 */
static void chassis_steer_alpha(const chassis_config *cfg,
                                float vx, float vy, float wz,
                                const float alpha_meas[CHASSIS_WHEEL_NUM],
                                float alpha_target[CHASSIS_WHEEL_NUM]) {
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        /* ① 轮心速度矢量 = 平移 + ω×r。和投影式的前两步完全一样 */
        float vx_c = vx - wz * cfg->wheel_y[i];
        float vy_c = vy + wz * cfg->wheel_x[i];

        /* 没在动时 atan2(0,0) 未定义，方向随便取 —— 取【当前角】，原地不动。
         * 换个说法：这时"指向哪"不影响任何事，那就别动，省得轮子原地抽搐。 */
        if (vx_c * vx_c + vy_c * vy_c < CHASSIS_STEER_EPS * CHASSIS_STEER_EPS) {
            alpha_target[i] = alpha_meas[i];
            continue;
        }

        /* ② 相对量而不是绝对量：目标 = 当前角 + 最短那一转。
         *    ⚠️ 为什么要绕这么一圈，而不是直接 `target = atan2(...)`：
         *       atan2 的值域是 (−π, π]，而实测角【是无界的】—— 非 GM6020 的
         *       motor_data.angle 由编码器累加得来，转几圈就是几十弧度。
         *       直接拿两者相减当 PID 误差，会出现"目标 3.5、实测 20.0、
         *       误差 −16.5"，转向电机就在那里空转十几弧度去追一个早就到了的角。
         *       写成相对量，误差天然落在 (−π/2, π/2]，上面那个坑整个不存在。 */
        float diff = remainderf(atan2f(vy_c, vx_c) - alpha_meas[i], CHASSIS_TWO_PI);

        /* ③ ±180° 的取舍：α 和 α+180 是同一根轴 —— 指向后者、轮子反着转，
         *    给出【完全一样的轮心速度】。挑离当前角近的那个，转向行程最短。
         *    这不是优化，是必须做：不挑的话目标从 +179° 变成 −179° 只差 2°，
         *    PID 却会命令它转 358°，线跟着甩，舵机自己也吃不消。
         *    （轮速的反号不用在这里处理 —— 下面用 alpha_target 再投影一次，
         *      投影到 α+180 上自然得到负的轮速。） */
        if (fabsf(diff) > CHASSIS_PI_2) {
            diff += (diff > 0.0f) ? -CHASSIS_PI : CHASSIS_PI;
        }

        alpha_target[i] = alpha_meas[i] + diff;
    }
}

/**
 * @brief 逆解：车体系目标速度 → 四轮的目标输出轴角速度
 *
 * 单个轮子的方程是"把轮心速度投影到驱动方向"：
 *
 *     轮心速度 = 平移 + ω×r
 *     ω×r：ω = (0, 0, wz)，r = (x_i, y_i) → (−wz·y_i, +wz·x_i)
 *     投影到 (cos α, sin α)：
 *         v_i = vx·cos α_i + vy·sin α_i + wz·(x_i·sin α_i − y_i·cos α_i)
 *
 * 最后一项就是自转对单个轮子的贡献，只跟轮子的【位置】有关 —— 这也是为什么
 * wheel_x / wheel_y 必须按实车量，量错一点，小陀螺就会让车走圈。
 *
 * ⚠️ 这里【不乘 gear_ratio】。ω 是输出轴角速度：轮子转多快只由轮半径决定，
 *    减速比决定的是电机转子要转多快 —— 那是电机自己的事，底盘不该管。
 */
static void chassis_inverse_kinematics(const chassis_config *cfg,
                                       float vx, float vy, float wz,
                                       const float alpha[CHASSIS_WHEEL_NUM],
                                       float omega[CHASSIS_WHEEL_NUM]) {
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        float c = cosf(alpha[i]);
        float s = sinf(alpha[i]);

        float v_wheel = vx * c + vy * s
                      + wz * (cfg->wheel_x[i] * s - cfg->wheel_y[i] * c);

        omega[i] = v_wheel / cfg->wheel_radius;
    }
}

/**
 * @brief 正解：四轮实测输出轴角速度 → 车体系速度（最小二乘）
 *
 * 逆解是 3 个已知量解 4 个未知量（一车速度定死四轮转速）；反过来是 4 个方程
 * 解 3 个未知量，【超定】。所以不是"取三个方程凑"，是解正规方程
 * A^T·A·x = A^T·b，第 i 行为 (cos α_i, sin α_i, x_i·sin α_i − y_i·cos α_i)。
 *
 * ★ 为什么费这个劲：四个轮子都参与，任何一个打滑都会被其余三个摊薄。
 *   凑三个方程等于宣布"被选中的三个轮子永远不会错"，一个轮子悬空就全错。
 *
 * ⚠️ 对方程组本身退化的情况（三个轮子悬空、轮子共线）不做保护：A^T·A 奇异，
 *    行列式趋零。这不是这里能修的 —— 得由上层拿轮速差自己判"车被抬起/在打滑"，
 *    然后 chassis_set_enabled(false)。驱动层认不出这个，它只看见四个数。
 */
static void chassis_forward_kinematics(const chassis_config *cfg,
                                       const float alpha[CHASSIS_WHEEL_NUM],
                                       const float omega[CHASSIS_WHEEL_NUM],
                                       float *vx, float *vy, float *wz) {
    float ata[3][3] = {{0.0f}};
    float atb[3]    = {0.0f};

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        float c = cosf(alpha[i]);
        float s = sinf(alpha[i]);
        float a[3] = { c, s, cfg->wheel_x[i] * s - cfg->wheel_y[i] * c };
        float b    = omega[i] * cfg->wheel_radius;

        for (int r = 0; r < 3; r++) {
            for (int k = 0; k < 3; k++) {
                ata[r][k] += a[r] * a[k];
            }
            atb[r] += a[r] * b;
        }
    }

    float det = ata[0][0] * (ata[1][1] * ata[2][2] - ata[1][2] * ata[2][1])
              - ata[0][1] * (ata[1][0] * ata[2][2] - ata[1][2] * ata[2][0])
              + ata[0][2] * (ata[1][0] * ata[2][1] - ata[1][1] * ata[2][0]);

    if (fabsf(det) < 1e-6f) {
        /* 退化：给 0，不给无穷大。上层拿到 0 只是"不知道"；拿到 inf/NaN 会污染
         * 后面每一次用到它的运算，而且不会自己好。 */
        *vx = 0.0f;
        *vy = 0.0f;
        *wz = 0.0f;
        return;
    }

    /* Cramer 法则：把 atb 逐列换进 A^T·A 再求行列式。3×3 不值得上 LU。 */
    float out[3];

    for (int j = 0; j < 3; j++) {
        float m[3][3];

        for (int r = 0; r < 3; r++) {
            for (int k = 0; k < 3; k++) {
                m[r][k] = (k == j) ? atb[r] : ata[r][k];
            }
        }
        out[j] = ( m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                 - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                 + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det;
    }

    *vx = out[0];
    *vy = out[1];
    *wz = out[2];
}

/**
 * @brief 解出四轮力矩的统一缩放系数 k ∈ [0, 1]
 *
 * ★ 关键决定：四个轮子的力矩【同乘一个 k】，不是各自削减。
 *   力矩比例不变 ⇒ 合力方向不变，车还是往操作手要的方向走，只是整体变慢。
 *   各自削减会把车"扭"向意料之外的方向 —— 这是功率控制里最容易犯的错，
 *   而且症状很迷惑（车会走，就是不走直线）。
 *
 * 推导：把目标力矩乘 k，电流也乘 k（同一个电机里 τ ∝ I），ω 不变，于是
 *   P(k) = Σ[k1·(k·I_i)·ω_i + k2·(k·I_i)² + k3·ω_i² + k4]
 *        = k·(k1·ΣI_iω_i) + k²·(k2·ΣI_i²) + (k3·Σω_i² + 4·k4)
 * 令 P(k) = limit 得一元二次方程：
 *   a = k2·ΣI_i²,  b = k1·ΣI_i·ω_i,  c = k3·Σω_i² + 4·k4 − limit
 *   k = (−b + √(b²−4ac)) / (2a)
 *
 * ⚠️ I 和 ω 用的是【上一拍的实测值】（motor_get 拿到的），不是本拍要下发的
 *    目标。所以这是"事后刹车"不是"事前预测"，最坏晚一拍（1kHz 下 1 ms）。
 *    要提前，得从目标力矩反推电流 —— 那需要 kt，而 kt 在电机的 DT 节点上，
 *    底盘不该知道（这正是 gear_ratio / kt 没进 chassis_config 的原因）。
 *    1 ms 的滞后对热无所谓，对裁判系统 10 Hz 的判罚更无所谓。
 *
 * @param current      四轮转子电流（A，上拍实测）
 * @param omega_rotor  四轮转子角速度（rad/s，上拍实测）
 * @param limit        总功率上限（W）
 * @return 缩放系数 k。1 = 不用限。
 */
static float chassis_power_scale(const float current[CHASSIS_WHEEL_NUM],
                                 const float omega_rotor[CHASSIS_WHEEL_NUM],
                                 float limit) {
    if (limit <= 0.0f) {
        /* 失败开放：见 chassis_data.power_limit —— 忘了设就"能动"，不能"不动" */
        return 1.0f;
    }

    float sum_ii = 0.0f;   /* Σ I² */
    float sum_iw = 0.0f;   /* Σ I·ω */
    float sum_ww = 0.0f;   /* Σ ω² */

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        sum_ii += current[i] * current[i];
        sum_iw += current[i] * omega_rotor[i];
        sum_ww += omega_rotor[i] * omega_rotor[i];
    }

    float a = CHASSIS_PWR_K2 * sum_ii;
    float b = CHASSIS_PWR_K1 * sum_iw;
    float c = CHASSIS_PWR_K3 * sum_ww
            + (float)CHASSIS_WHEEL_NUM * CHASSIS_PWR_K4
            - limit;

    /* ★ 判"要不要缩"只能看 k = 1（满扭矩）那一点，不能看 k = 0。
     *   下面这个式子就是 P(1) ≤ limit 展开后的样子。
     *
     *   ⚠️ 这里写错过一次，而且是那种"看着很合理"的错：当时的判据是 c ≤ 0，
     *      即"零扭矩时没超限就不缩"。可零扭矩永远不超限（P(0) = 四个 k4 而已），
     *      于是限幅【一次都没生效过】，四轮满扭矩照下。整套功率控制静默失效，
     *      不报错、不崩溃，只是车该限的地方没限。差分测试才逮出来。 */
    if (a + b + c <= 0.0f) {
        return 1.0f;
    }

    if (a <= 1e-9f) {
        /* 四个轮子电流都接近 0：铜损项没了，退化成一次方程 b·k + c = 0。
         * b ≤ 0 时 f 在 [0,1] 上递增不了（且 f(1) > 0），整段都在限上，只能全砍。*/
        if (b <= 0.0f) {
            return 0.0f;
        }
        float k = -c / b;
        return (k <= 0.0f || k > 1.0f) ? 0.0f : k;
    }

    /* a > 0：开口朝上，f(k) ≤ 0 的区间就是两根之间 [k_lo, k_hi]，
     * 所以"还能用的最大 k"就是较大的那个根 k_hi。 */
    float disc = b * b - 4.0f * a * c;

    if (disc < 0.0f) {
        /* 无实根：抛物线整个在限值之上，也就是"一动不动都已经超限"。
         * 全砍 —— 这种 limit 多半本身就配错了，但朝停住那边倒比朝失控那边倒安全。*/
        return 0.0f;
    }

    float k = (-b + sqrtf(disc)) / (2.0f * a);

    /* k > 1 只可能来自"两根都大于 1"（上面已经排除了 f(1) ≤ 0），
     * 那种情况下 f 在 [0,1] 上恒正，k = 1 并不合法 —— 所以这里是 return 0
     * 而不是 clamp 到 1。clamp 在这儿会把"该停"变成"满输出"，正好反了。 */
    return (k <= 0.0f || k > 1.0f) ? 0.0f : k;
}

/**
 * @brief 底盘控制一拍：实测 → 逆解 → 四轮速环 → 存进 wheel_torque[]
 *
 * 顺序不能换：先读这一拍的反馈，再算目标。反过来就是把上一拍的
 * 目标配这一拍的反馈 —— 增量式 PID 的 D 项会直接看错，而且错得不像错。
 *
 * 【不下发驱动轮】—— 力矩存进 data->wheel_torque[]，由 chassis_power_limit
 * 限幅之后再 motor_set。拆分的理由见那个函数。这里唯一会 motor_set 的是
 * 转向电机（舵轮构型），它不进功率预算：转向是位置环、力矩小，而且把它一起
 * 削掉会让车在超功率时转向变慢 —— 那比走慢一点危险得多。
 */
void chassis_update(const struct device *dev, float dt) {
    const chassis_config *cfg = dev->config;
    chassis_data *data = dev->data;

    const pid_config *wheel_pid_cfg = pid_get_config(cfg->wheel_pid);

    float alpha[CHASSIS_WHEEL_NUM];
    float omega_meas[CHASSIS_WHEEL_NUM] = {0.0f};

    /* ① 读实测。读失败就留 0：宁可这一拍当"轮子没转"，也别拿上一拍的残值
     *    去喂 PID —— 那会把一次通信故障伪装成一个真实的速度。
     *
     *    ⚠️ 只取轮速。转子电流 / 转子转速本来也在这次 motor_get 的返回里，
     *      但它们的唯一消费者是功率限幅，而限幅现在在另一个函数里 ——
     *      不从这里转发，让 chassis_power_limit 自己现读。多一次 motor_get
     *      几乎不要钱（它就是拷贝一个结构体，锁在最里面那层），换的是
     *      "limit 那一步自给自足、能独立看懂"。 */
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        motor_data md;

        if (motor_get(cfg->wheels[i], &md) == 0) {
            omega_meas[i] = md.omega;
        }
    }

    chassis_read_alpha(cfg, alpha);

    /* ② 失能：清积分器 + 零扭矩下发 + 状态清零，然后走人。
     *    清积分器的动作【故意不放在 chassis_set_enabled 里】—— 那里跨线程写
     *    pid_data 就得和正在算 PID 的本函数抢锁；放在这里，pid_data 从头到尾
     *    只有一个写方，锁根本不需要存在。多这几行，省掉一整把锁。
     *
     *    ⚠️ 下发的 mode 写 TORQUE 是【声明意图】，不是让谁去查表：
     *      motor_dji_can_pack 只读 target.torque，压根不看 mode。写对了是为了
     *      以后换上真按 mode 分派的电机时这行还是对的。 */
    if (!data->enabled) {
        motor_setpoint sp = {
            .angle = 0.0f, .omega = 0.0f, .torque = 0.0f,
            .mode  = MOTOR_MODE_TORQUE,
        };

        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            /* i_out 既是积分累加器又是 I 项输出，清它 = 清积分。
             *
             * ★ 为什么【只清 i_out】、不顺手把 last_error 也归零：
             *   D_out = Kd·(e_now − last_error)/dt。失能期间本函数每拍都走到这里，
             *   last_error 停在失能前的最后一个误差上，是"真实存在过的值"。
             *   把它清成 0 反而引入了【一个人为构造的误差变化量】——重新使能的
             *   第一拍就会算出 D_out ≈ Kd·e/dt（1 kHz 下被放大 1000 倍），
             *   一个纯粹的假尖峰，而且 max_out 一钳它就变成"满输出起步"。
             *   留着的 last_error 让 D 项的第一拍看到的是两个真实值之差，小得多。
             *
             *   注意这里【不额外清 p_out/d_out/output】：它们每拍都会被 pid_update
             *   重算，留着不影响任何东西。归零一个会被覆盖的字段只是噪音。 */
            data->wheel_pid[i].i_out = 0.0f;
            data->steer_pid[i].i_out = 0.0f;

            /* 顺手把"这一拍算出的力矩"也清零。chassis_power_limit 在失能时
             * 直接 return，本来读不到这里 —— 但留着上一拍的残值会让
             * "wheel_torque[] 是未限幅的当拍值"这条不变量在失能期间名存实亡，
             * 谁拿调试器看这个数组都会读到假的。一行的事，把不变量补全。 */
            data->wheel_torque[i] = 0.0f;

            motor_set(cfg->wheels[i], &sp);

            /* 舵轮的转向电机【也要归零】。漏掉这一句的后果是：急停之后驱动轮
             * 停了，转向电机还顶着失能前那一拍的力矩 —— 转向环的积分器虽然清
             * 了，但电机端存的是一份【力矩】，它不因为上游不更新就自己变 0。
             * 急停时最不该出现的就是一个谁都不再管的非零输出。 */
            if (cfg->type == steer) {
                motor_set(cfg->steer_motors[i], &sp);
            }
        }

        data->status.vx = 0.0f;
        data->status.vy = 0.0f;
        data->status.wz = 0.0f;
        return;
    }

    /* ③ 定 α —— 三种构型在这里分岔，而且【只】在这里分岔。
     *
     *    omni / mecanum：α 是常量，chassis_read_alpha 刚才已经查好表了。
     *    steer：α 要现算。先用轮心速度矢量定转向目标，再拿【目标角】去投影 ——
     *      注意是目标角不是实测角。用实测角的话，轮子还没转到位时算出来的
     *      轮速就是错的；用目标角，"轮子该转多快"和"轮子该朝哪"说的是同一件事，
     *      转向环追不追得上只影响这一拍摄入的准确度，不影响解算本身自洽。
     *
     *    ★ 所以 steer 分支里没有第二套轮速公式 —— 定完 α 之后，
     *      chassis_inverse_kinematics 对三种构型是同一个函数。这是有意的：
     *      分岔的只有"α 从哪来"，不是"怎么用 α"。 */
    float alpha_target[CHASSIS_WHEEL_NUM];
    const float *alpha_used = alpha;

    if (cfg->type == steer) {
        chassis_steer_alpha(cfg, data->vx, data->vy, data->wz, alpha, alpha_target);

        /* 转向环：位置环，把转向电机驱到 alpha_target。
         * 和下面的轮速环是【两条独立的闭环】—— 转向没到位时轮速照给。
         * 反过来的做法（等转向到位再给轮速）会让车每变一次向就顿一下，
         * 而那点顿挫操作手感上比走歪更明显。 */
        const pid_config *steer_pid_cfg = pid_get_config(cfg->steer_pid);

        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            motor_setpoint sp = {
                .angle = 0.0f, .omega = 0.0f,
                .torque = pid_update(&data->steer_pid[i], steer_pid_cfg,
                                     alpha_target[i], alpha[i], dt, 0.0f),
                .mode  = MOTOR_MODE_TORQUE,
            };

            motor_set(cfg->steer_motors[i], &sp);
        }

        alpha_used = alpha_target;
    }

    /* ④ 逆解：车体系目标速度 + α → 四轮目标输出轴角速度 */
    float omega_target[CHASSIS_WHEEL_NUM];
    chassis_inverse_kinematics(cfg, data->vx, data->vy, data->wz, alpha_used, omega_target);

    /* ⑤ 轮速环：目标角速度 → 输出轴力矩。
     *    无前馈：底盘轮速环没有可用的前馈量，给 0 就是"把这事说清楚"，
     *    而不是留个参数在那儿等人填。
     *
     *    ★ 结果写进 data->wheel_torque[]，本函数到此为止 —— 限幅和下发是
     *      chassis_power_limit 的事。这里【不】顺手 motor_set 一下"先发个
     *      未限幅的、等下再覆盖"：那会在两个 motor_set 之间留一个窗口，
     *      发送线程正好抢进去，就把未限幅的力矩发出去了。 */
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        data->wheel_torque[i] = pid_update(&data->wheel_pid[i], wheel_pid_cfg,
                                           omega_target[i], omega_meas[i], dt, 0.0f);
    }

    /* ⑥ 回填状态。走正解而不是回写指令值 —— status 是"实测"，不是"我要求了多少"。
     *    打滑时这两个数会分家，而那正是最需要知道真实值的时候。
     *    ⚠️ 目前【没有消费者】：云台不读它（云台的解耦靠"反馈全用云台 IMU"白拿，
     *       见 gimbal.h）。留着是给上层做里程计/状态显示用的。 */
    chassis_forward_kinematics(cfg, alpha, omega_meas,
                               &data->status.vx, &data->status.vy, &data->status.wz);
}

/* ═══════════════════ 底盘功率限幅 + 下发 ═══════════════════ */

/**
 * @brief 功率限幅 + 下发驱动轮 —— 底盘这一拍的最后一道闸
 *
 * 完整的理由、调用契约、以及为什么不能写成 `wheel_torque[i] *= k` 都在
 * chassis.h 上这个函数的文档里。这里只补实现侧的两条：
 *
 * ⚠️ 电流 / 转速读【失败就留 0】，写法同 chassis_update ①。但两者结果的
 *    走向【正好相反】，别照搬那边的直觉：留 0 意味着 P ≈ 四个 k4（约 4.6 W），
 *    功率模型认为"这轮没在耗电"，于是 chassis_power_scale 给出 k = 1 ——
 *    【不缩】，满扭矩照发。这是本函数唯一一处失败朝"放开"倒的地方。
 *
 *    实际影响很小：在 motor_dji 上 motor_get 只因 api 没装而失败（-ENOSYS），
 *    设备指针非空的话它基本不会失败。真正会走到的是"这轮还没收到过反馈帧"，
 *    那时 base_data 全零，和这里留 0 是同一个状态。所以这一条的实际意义是
 *    "上电初期不会因为读不到电流就乱缩"，不是"读不到就刹车"。
 *    （要真区分"没数据"和"数据是 0"，得给 motor_data 加 valid 标志 ——
 *      那是电机层的事，不该在这儿用 0 兼职表达两种含义。）
 *
 * ★ 顺序：先算 k，再统一乘、统一发。不能边算边发 —— 四个轮子拿到的是同一个
 *   k，才能保证合力方向不变，见 chassis_power_scale。
 */
void chassis_power_limit(const struct device *dev) {
    const chassis_config *cfg = dev->config;
    chassis_data *data = dev->data;

    /* 失能时【什么都不做】。零输出已经由 chassis_update 的失能分支下发过了，
     * 这边再发一次就是拿 wheel_torque[] 的陈旧值把刚刹停的轮子重新点着。
     * "谁负责输出"只能有一个答案，这里选 update。 */
    if (!data->enabled) {
        return;
    }

    float current[CHASSIS_WHEEL_NUM]   = {0.0f};
    float omega_rot[CHASSIS_WHEEL_NUM] = {0.0f};

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        motor_data md;

        if (motor_get(cfg->wheels[i], &md) == 0) {
            current[i]   = md.current;
            omega_rot[i] = md.omega_rotor;
        }
    }

    const float k = chassis_power_scale(current, omega_rot, data->power_limit);

    /* 只 set，不 pack 也不 send —— 打包和发送是另一条线程的事
     * （见 motor_dji_test 里那条 1ms 定时发送线程）。 */
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        motor_setpoint sp = {
            .angle = 0.0f, .omega = 0.0f,
            /* ⚠️ 乘出来的新值，【不写回】data->wheel_torque[i]。
             *   写回的话同一拍调两次 k 就乘两次，而且不可逆 ——
             *   限幅器会变成一个每次调用都衰减一截的东西。 */
            .torque = data->wheel_torque[i] * k,
            .mode   = MOTOR_MODE_TORQUE,
        };

        motor_set(cfg->wheels[i], &sp);
    }
}

/* ═══════════════════ 拨弹盘控制一拍 ═══════════════════ */

/**
 * @brief 拨弹盘控制一拍：单发位置环 / 连发速度环
 *
 * 和 chassis_update 分开是有意的 —— 两条闭环控的是两个不相干的执行器，
 * 合并只会让"底盘这一拍"变成"底盘 + 发射这一拍"。调用方式见 chassis.h。
 */
void chassis_trigger_update(const struct device *dev, float dt) {
    const chassis_config *cfg = dev->config;
    chassis_data *data = dev->data;

    motor_setpoint sp = {
        .angle = 0.0f, .omega = 0.0f, .torque = 0.0f,
        .mode  = MOTOR_MODE_TORQUE,
    };

    float angle_meas = 0.0f;
    float omega_meas = 0.0f;
    motor_data md;

    /* 没装拨盘：空操作。上层可以无脑每拍调，不必先查有没有配 ——
     * "这台车没这功能"不该让每个调用点都长出一个 if。 */
    if (cfg->trigger_motor == NULL) {
        return;
    }

    /* 使能是所有输出的总闸，拨盘也归它管 —— 急停时最不该出现的就是
     * "轮子停了、拨盘还在转"。
     *
     * ★ trigger_fire_last 【照常跟到当前电平】，不保持旧值：
     *   保持的话，急停期间按住开火、恢复使能的那一拍会被判成上升沿 ——
     *   车一使能就自动打一发。跟着走，恢复后必须【重新按下】才开火。
     *   这和失能清积分器是同一条原则：不要在恢复的那一刻积攒"待执行"。
     *   基准一并作废，同理由。 */
    if (!data->enabled) {
        data->trigger_fire_last    = data->trigger_fire;
        data->trigger_target_valid = false;

        motor_set(cfg->trigger_motor, &sp);
        return;
    }

    /* 边沿判定。本函数是 trigger_fire_last 【唯一】的写方，所以这对读-改-写
     * 不担心和谁抢 —— 上层只翻 trigger_fire。
     *
     * ★ 放在分支【之前】、两种模式都更新，是有意的：连发期间按着键切到单发，
     *   若 last 还停在切模式前的旧值，那一拍会凭空冒出一个上升沿、白打一发。
     *   一直跟着，切模式就永远不会伪造边沿。 */
    bool rising = data->trigger_fire && !data->trigger_fire_last;

    data->trigger_fire_last = data->trigger_fire;

    /* 读反馈。读失败留 0：同 chassis_update ① —— 宁可这一拍当"没转"，
     * 也别拿上一拍的残值去喂位置环，那会把一次通信故障伪装成一个真实角度。 */
    if (motor_get(cfg->trigger_motor, &md) == 0) {
        angle_meas = md.angle;   /* 输出轴 rad。增量式编码器，从 0 起累加、不回绕 */
        omega_meas = md.omega;   /* 输出轴 rad/s */
    }

    if (data->trigger_mode == TRIGGER_MODE_AUTO) {
        /* ★ 不用的那个环，积分器钉在零。
         *   这样"刚切进来"和"第一次用"就没有区别 —— 任何时刻接手都是干净的
         *   积分器，也就不需要"模式变了"这个标志，少一份跨状态。
         *   反过来（只在使用时清）就得让 set_trigger_mode 去写 pid_data，
         *   而那正是要跨线程抢锁的那条路。 */
        data->trigger_angle_pid.i_out = 0.0f;

        /* 停火给 0 而不是"保持上一拍"：速度环追零就是刹住。拨盘不靠惯性送弹，
         * 停就得停 —— 保住"松手不再出弹"这条手感。 */
        float omega_target = data->trigger_fire
                                 ? cfg->trigger_speed_deg_s * CHASSIS_DEG_TO_RAD
                                 : 0.0f;

        sp.torque = pid_update(&data->trigger_speed_pid,
                               pid_get_config(cfg->trigger_speed_pid),
                               omega_target, omega_meas, dt, 0.0f);
        motor_set(cfg->trigger_motor, &sp);
        return;
    }

    /* ── 单发：位置环 ── */
    data->trigger_speed_pid.i_out = 0.0f;

    /* 基准未建立：本拍【只记基准，不出力】。
     *
     * ★ 为什么必须有这一步：init 跑在 POST_KERNEL，那时 CAN 上一个反馈帧都还
     *   没有，motor_get 拿到的必然是上电初值。若直接拿它当"当前角"，第一发的
     *   目标就是 上电初值 + 步进角 —— 而拨盘实际可能停在第 500 rad，车一使能
     *   就倒着狂转回去。先记基准，这个跳变就不存在。
     *   （chassis.h 里说的"读到 0"就是这个意思：它未必正好是 0，但一定是
     *     一个没反映真实位置的值。）
     *
     *   积分器也在这一拍清：刚切进单发时，角度环的 i_out 还停在上一次单发
     *   留下的数上，不清的话第一发会带着旧积分起步。 */
    if (!data->trigger_target_valid) {
        data->trigger_target_deg      = angle_meas * CHASSIS_RAD_TO_DEG;
        data->trigger_target_valid    = true;
        data->trigger_angle_pid.i_out = 0.0f;

        motor_set(cfg->trigger_motor, &sp);
        return;
    }

    /* 上升沿 = 出一发：目标按固定步进角往前挪一格。
     * 按住不放只有一个上升沿，所以单发不会因为按键持续而变成连发。 */
    if (rising) {
        data->trigger_target_deg += cfg->trigger_angle_deg;
    }

    sp.torque = pid_update(&data->trigger_angle_pid,
                           pid_get_config(cfg->trigger_angle_pid),
                           data->trigger_target_deg * CHASSIS_DEG_TO_RAD,
                           angle_meas, dt, 0.0f);
    motor_set(cfg->trigger_motor, &sp);
}

/* ─── 为单个 status = "okay" 的 chassis 节点注册 Zephyr device ───
 *
 * 级别 92 而不是 91：init 里要 device_is_ready 电机(91)/IMU(91)/PID(50)，
 * 必须排在它们之后。同一 init level 内按优先级【升序】执行。
 */
#define CHASSIS_INST(inst)                        \
    CHASSIS_CONFIG_DEFINE(inst);                  \
    CHASSIS_DATA_DEFINE(inst);                    \
    DEVICE_DT_INST_DEFINE(inst,                   \
                          chassis_init,           \
                          NULL,                   \
                          &chassis_data_##inst,   \
                          &chassis_config_##inst, \
                          POST_KERNEL,            \
                          92,                     \
                          &chassis_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CHASSIS_INST)
