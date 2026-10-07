/*
 * skywalker 最终固件 —— 骨架
 *
 * 分工：
 *   驱动（motors / chassis / gimbal / imu / pid）负责"这一拍怎么算"。
 *   这里负责"这一拍该要什么"：读遥控器 → 状态机 → 把意图翻译成驱动认的目标量。
 *
 * ★ 本文件里【不出现任何电机型号】。M3508 / GM6020 / DM 一律不提 ——
 *   型号是设备树（boards/dm_mc02.overlay）的事。这里只认 motor_set / motor_get
 *   和 chassis / gimbal 两个设备。换电机不用改这里一个字。
 *
 * 线程结构照 samples/motor_dji_test：
 *   控制线程 —— 算，只 motor_set（写目标）
 *   发送线程 —— 打包 + can_send（真正上总线）
 * 两条线程用两个 1ms k_timer 驱动。分开的理由：can_send 是阻塞调用，
 * 超时/重传会拖时间，混在控制回路里会让 PID 的 dt 抖。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "drivers/chassis/chassis.h"
#include "drivers/gimbal/gimbal.h"
#include "drivers/imu/imu.h"
#include "drivers/motors/motor.h"
#include "lib/dr16/dr16.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ═══════════════════ 设备 ═══════════════════ */

static const struct device *const chassis_dev = DEVICE_DT_GET(DT_NODELABEL(skywalker_chassis));
static const struct device *const gimbal_dev  = DEVICE_DT_GET(DT_NODELABEL(skywalker_gimbal));
static const struct device *const dr16_uart   = DEVICE_DT_GET(DT_NODELABEL(uart5));
static const struct device *const imu_dev     = DEVICE_DT_GET(DT_NODELABEL(imu));

/* ═══════════════════ 要整定的数 ═══════════════════
 * 全部集中在这里。这些是【手感参数】，不是"装车事实" ——
 * 所以留在 C 里，不进 DT：DT 是"这辆车装了什么"，这些是"打法"。
 * ⚠️ 一个都没整定过，全是起点值。 */
#define CTRL_PERIOD_MS       1
#define APP_MAX_LINEAR       3.0f     /* 平地最大平移速度（m/s） */
#define APP_MAX_ANGULAR      6.0f     /* 小陀螺最大自转角速度（rad/s） */
#define APP_POWER_LIMIT_W    80.0f    /* 底盘功率上限（W）。见 main() 里"裁判系统"一节 */

/* ── 鼠标增益 ──
 * ⚠️ 这两个数是按"量级"推出来的，不是整定出来的，上车第一个要调的就是它们。
 *
 * 鼠标的三轴是【每帧位移】不是速度（DBUS 一帧 ~14ms）。把它当速度用就是零阶
 * 保持：这一帧的命令一直保持到下一帧 —— 而这正是速度指令该有的样子，
 * 所以 yaw 直接乘增益就行，不用除以 dt。
 *
 * yaw：快速甩一下鼠标一帧大约 50~100 格，希望给到 ~3 rad/s → 增益 ≈ 0.03
 * pitch：一帧 100 格 → 0.04 rad ≈ 2.3°，甩到底十来个帧，够跟手也不飘 */
#define APP_MOUSE_YAW_GAIN   0.03f
#define APP_MOUSE_PITCH_GAIN 0.0004f

/* ── 换系用的两个修正量 ──
 * 云台 yaw 电机报出来的角是"相对它自己的零点、按它自己绕向"的数，要变成
 * "云台相对底盘转了多少"还差两件事：绕向和零点。两个都取决于装车方式，
 * 只能在实车上量，所以放这里当待定值。
 *
 * ⚠️ 怎么量：
 *   APP_YAW_SIGN —— 把云台【从上往下看逆时针】推一小段，看 motor_get 拿到的
 *                   md.angle 是涨还是跌。涨就填 +1，跌就填 -1。
 *   APP_YAW_ZERO_RAD —— 把云台转到【正对车头】，读一下当时 md.angle 是多少，
 *                       取它的负值填这里。这样"云台朝前"就对应 0。
 *
 * ⚠️ 对【相对式】零点（见 chassis_frame_yaw 里那段）：这个零点只在把云台
 *    摆正那一刻有效，电机会记住。上电时云台停在哪，参考就是哪。
 *    如果嫌麻烦，用【绝对式】的电机就能把零点固定进 DT —— 那时这两个常数
 *    里有一个可以退休。 */
#define APP_YAW_SIGN      (+1.0f)
#define APP_YAW_ZERO_RAD  (0.0f)

/* 遥控器失效判据。DBUS 一帧 ~14ms，给 100ms 已经是"连丢七帧"。
 * 超过它就当接收机掉线，原地不动 —— 而不是保持最后一帧的摇杆量继续冲。 */
#define APP_RC_TIMEOUT_MS    100

/* ── IMU ── */
#define IMU_TARGET_TEMP      50.0f   /* 加热片目标温度（°C） */
#define IMU_HEAT_PERIOD_MS   100

/* IMU 失效判据（单位：控制拍的个数，不是毫秒）。
 * 陀螺 DRDY 1kHz、控制回路 1kHz，正常情况每拍都该见到新心跳。
 * 给 20 拍 = 20ms 的容差，超了就认为姿态解算是死的。
 *
 * ★ 这个判据存在的理由很具体：云台环的【唯一】反馈源就是 IMU。IMU 一死，
 *   实测角恒为 0，而目标角还在跟着鼠标走 —— 误差越拉越大，云台会一路顶到
 *   机械限位。所以 IMU 不新鲜时必须【禁止云台使能】，不能让它带着瞎了的
 *   反馈继续跑。底盘不受影响：chassis_config.imu 目前是个死字段，底盘不读姿态。 */
#define APP_IMU_STALE_LIMIT  20

/* ── DR16 的量纲 ──
 * 摇杆原始值是 0~2047，dr16.c 已经把中位 1024 减掉了，所以这里是
 * −1024~1023，回中就是 0。上层不用再补偏置。 */
#define DR16_CH_MAX    1024.0f
#define DR16_DEADBAND  20       /* 回中抖动。不减掉的话车会在零点附近慢慢爬 */

/* DBUS 拨杆位置。1=上 2=中 3=下，0 是"没收到有效帧"。 */
#define SW_UP    1
#define SW_MID   2
#define SW_DOWN  3

/* DBUS 按键位。顺序和 dr16.h 里 key 字段的注释一致。 */
enum {
	KEY_W = 0, KEY_S, KEY_A, KEY_D, KEY_SHIFT, KEY_CTRL, KEY_Q, KEY_E,
	KEY_R, KEY_F, KEY_G, KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B,
};

/* ═══════════════════ 线程间同步 ═══════════════════
 * 定义必须放在用到它们的线程函数【之前】：K_SEM_DEFINE 是一个变量定义，
 * C 里没有"先使用后定义"这回事 —— 线程函数体里引用 ctrl_sem 的那一刻
 * 它必须已经声明过了。 */
K_SEM_DEFINE(ctrl_sem, 0, 1);
K_SEM_DEFINE(send_sem, 0, 1);

/* ═══════════════════ 姿态解算（IMU 线程）═══════════════════
 *
 * ★ 为什么必须有这条线程：imu.h 那几个函数（fetch / predict / correct /
 *   update_angle / heat_control）【都要有人按节拍调】。驱动本身不主动跑 ——
 *   它只是把 BMI08x 的 DRDY 中断转成事件，然后等着。
 *   没人调 = imu_data.angle 永远是零初始化的 0 = 云台拿一个恒为 0 的反馈
 *   去算误差 = 编译链接全过、上电就废。这类"少调一个函数"的坑没有编译期信号，
 *   只能在写的时候就想清楚"谁负责按节拍调它"。
 *
 * ★ 为什么不塞进控制线程：DRDY 是 1kHz（陀螺）/ 800Hz（加计），和控制回路的
 *   1kHz 不同源；而且一次 correct 里有几百字节的 VLA + 三次 atan2，
 *   混进控制线程会把它的 WCET 顶上去，控制回路的 jitter 直接变成 dt 误差。
 *   分开之后控制线程只管"读一份现成的快照"，两个节拍互不干扰。
 *
 * 依赖 CONFIG_EVENTS（prj.conf 里已开）。用事件不用信号量：两个中断源各占一个
 * 位，k_sem 只有"有/没有"，不携带身份，分不清是加速度计还是陀螺仪来了。 */

#define IMU_EV_ACCEL  BIT(0)
#define IMU_EV_GYRO   BIT(1)

K_EVENT_DEFINE(imu_event);

/* dt 走 480MHz 的 64 位周期计数器（分辨率 ≈ 2ns），【不用】k_uptime_get() ——
 * 毫秒尺量 1ms 周期会得到 0，F 矩阵退化成单位阵，整轮预测被静默跳过。 */
#define CYCLE_TO_SEC  (1.0f / (float)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)

/* 先乘后除：避免 CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC 不能被 1000 整除时截断 */
#define IMU_HEAT_PERIOD_CYCLES \
	((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * IMU_HEAT_PERIOD_MS / 1000U)

/** 心跳。IMU 线程每轮 +1，控制线程查它有没有在动 —— 见 APP_IMU_STALE_LIMIT。
 *  ★ 用 32 位计数而不是"最后时刻"：ARM 上一次 int64_t 读是两条指令，
 *    会被高优先级线程撕成半新半旧，判据跟着误动作。32 位对齐访问是单条。 */
static volatile uint32_t imu_heartbeat;

/* ⚠️ 必须是 static：驱动存的是【指针】不是拷贝，局部变量一出函数就悬空，
 *    而这段代码编译不给任何警告。 */
static struct sensor_trigger accel_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

static struct sensor_trigger gyro_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_GYRO_XYZ,
};

/** DRDY 回调。跑在 BMI08x 驱动的 OWN_THREAD 线程里，【只报信不干活】——
 *  一次 correct 要几百字节栈，塞不进驱动那 1536B。
 *  ⚠️ 这里绝不能 wait：post 和 wait 必须分属两个执行流，自己等自己就是死锁。 */
static void imu_drdy_handler(const struct device *dev, const struct sensor_trigger *trig) {
	ARG_UNUSED(dev);

	if (trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		k_event_post(&imu_event, IMU_EV_ACCEL);
	} else if (trig->chan == SENSOR_CHAN_GYRO_XYZ) {
		k_event_post(&imu_event, IMU_EV_GYRO);
	}
}

static void imu_thread_entry(void *a, void *b, void *c) {
	/* 锚点必须在循环【外】初始化。放进循环里 = 每轮从此刻起算：
	 * 加热条件永不成立、dt 恒为 0。 */
	uint64_t last_est_cycle  = k_cycle_get_64();
	uint64_t last_heat_cycle = k_cycle_get_64();

	while (1) {
		/* reset=false：只清命中的位，没命中的留着，所以"还没处理完又来中断"
		 * 也不会丢事件。传 true 会先清零再判断，那个窗口里的事件就没了。 */
		uint32_t ev = k_event_wait(&imu_event, IMU_EV_ACCEL | IMU_EV_GYRO,
		                           false, K_FOREVER);

		uint64_t now_cycle = k_cycle_get_64();

		/* 陀螺 1kHz：dt 只属于 predict —— correct 的增益里没有 dt。
		 * 所以 last_est_cycle 只在这个分支推进：万一丢了一次 DRDY，
		 * 下一拍的实测 dt 会变成 2ms，F 按 2ms 积分，状态仍然是对的。
		 * （要是把 dt 写死 1ms，那一次旋转就静默丢了。） */
		if (ev & IMU_EV_GYRO) {
			imu_fetch_gyro(imu_dev);
			float dt = (float)(now_cycle - last_est_cycle) * CYCLE_TO_SEC;

			last_est_cycle = now_cycle;
			imu_predict(imu_dev, dt);
		}

		/* 加计 800Hz。两个 if 【不是】 else if：一次唤醒可能同时带两个位。 */
		if (ev & IMU_EV_ACCEL) {
			imu_fetch_accel(imu_dev);
			imu_correct(imu_dev);
		}

		/* 一轮只调一次：get_angle 是有状态的（内部维护 Yaw 跨圈计数），
		 * 一拍调两次会把跨圈计数推快一倍。 */
		imu_update_angle(imu_dev);

		/* 加热 100ms 一次，节拍独立于解算 */
		if ((now_cycle - last_heat_cycle) >= IMU_HEAT_PERIOD_CYCLES) {
			imu_heat_control(imu_dev, IMU_TARGET_TEMP,
			                 (float)(now_cycle - last_heat_cycle) * CYCLE_TO_SEC);
			last_heat_cycle = now_cycle;
		}

		/* 放在最后：说明"这一轮真的算完了"，而不是"刚醒来"。
		 * 控制线程据此判断姿态是不是新鲜的。 */
		imu_heartbeat++;
	}
}

/* ═══════════════════ 跨拍要保持的状态 ═══════════════════
 * 放在文件作用域而不是函数内 static，是因为它们要跨两个函数用：
 * 使能瞬间对齐 pitch 的那一句在 remote_to_command 的【开头】，
 * 而累加 pitch 的那一句在【结尾】—— 函数内 static 在开头那一刻还没声明。 */

/** pitch 目标绝对角（rad）。上电 0，使能瞬间对齐到实际角度。 */
static float pitch_target_rad;
/** yaw 目标绝对角 —— 只在切到 AUTO（自瞄）时用得上，现在恒 MANUAL，先留着 */
static float yaw_target_rad;
/** 上一拍【云台】的使能状态，用来抓"使能沿"。
 *  抓的是云台不是底盘：底盘和云台的使能条件从加了 IMU 判据那一刻起就不一样了
 *  （云台多一道），而对齐 pitch 目标这件事只对云台有意义。 */
static bool  was_gimbal_enabled;
/** 上一拍的拨盘 / 云台模式，用来抓"变化沿"。见 remote_to_command 里的说明。 */
static trigger_mode_t last_trigger_mode = (trigger_mode_t)-1;
static gimbal_mode_t  last_gimbal_mode  = (gimbal_mode_t)-1;

/* ═══════════════════ 电机清单（全部来自 DT）═══════════════════
 *
 * ★ 这张表是【问 chassis / gimbal 要】出来的，不是在这里列 m0..m7 ——
 *   列了就等于把型号和拓扑写死在 application 里，换个车就要改代码。
 *   驱动本来就把"这辆车装了哪些电机"记在它自己的 config 里，直接拿。
 */
#define APP_MOTOR_MAX 16

static const struct device *motor_list[APP_MOTOR_MAX];
static uint8_t motor_count;

static void motor_list_add(const struct device *dev) {
	/* NULL = 这辆车没配这一项（没拨盘 / 没摩擦轮 / 不是舵轮），跳过 */
	if (dev == NULL) {
		return;
	}
	if (motor_count >= APP_MOTOR_MAX) {
		LOG_ERR("motor_list 溢出，APP_MOTOR_MAX 要调大");
		return;
	}
	motor_list[motor_count++] = dev;
}

static void motor_list_build(void) {
	const chassis_config *cc = chassis_dev->config;
	const gimbal_config  *gc = gimbal_dev->config;

	for (int i = 0; i < 4; i++) {
		motor_list_add(cc->wheels[i]);
	}
	/* 舵轮的转向电机在 omni / mecanum 上是全 NULL，add 会自己跳过。
	 * 不按 type 判也行，但按 type 判读起来更明白"这是舵轮才有的东西"。 */
	if (cc->type == steer) {
		for (int i = 0; i < 4; i++) {
			motor_list_add(cc->steer_motors[i]);
		}
	}
	motor_list_add(cc->trigger_motor);

	/* 摩擦轮是变长的，个数在 config 里（friction_num），不是 GIMBAL_FRICTION_MAX */
	for (int i = 0; i < gc->friction_num; i++) {
		motor_list_add(gc->friction_motors[i]);
	}
	motor_list_add(gc->yaw_motor);
	motor_list_add(gc->pitch_motor);

	LOG_INF("从 DT 收集到 %u 台电机", motor_count);
}

/* ═══════════════════ 小工具 ═══════════════════ */

static float clampf(float v, float lo, float hi) {
	return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/** 摇杆值 → 归一化，顺手过死区。返回 −1.0 ~ +1.0 */
static float stick_norm(int16_t raw) {
	if (raw > -DR16_DEADBAND && raw < DR16_DEADBAND) {
		return 0.0f;
	}
	return clampf((float)raw / DR16_CH_MAX, -1.0f, 1.0f);
}

/* ═══════════════════ 换系 ═══════════════════ */

/**
 * @brief 车头相对"操作手前方"偏了多少（rad）
 *
 * 遥控器摇杆给的是【机体系】—— 以操作手为参照的前后左右。底盘要的是
 * 【车体系】—— 以车头为正。两者差一个绕 z 的旋转，就是本函数返回的角。
 *
 * 来源 = 【云台 yaw 电机的编码器】。这一条是定下来的（2026-10-07）：操作手
 * 用云台瞄准，所以"云台朝向"就是"操作手的前方"，云台相对底盘转了多少，
 * 就是两个系差了多少。
 *
 * ★★ 这个接口【对电机型号免疫】，这正是它该长成这样子的原因：
 *    只走 motor_get 拿通用的 motor_data.angle，一个字都不提型号。
 *    底下可能是 3508、GM6020、还是达妙 —— 各家的 angle 语义差得很远
 *    （光 motor_dji 内部就有两套：直驱 GM6020 是绝对编码器、卷绕到 (−π,π]；
 *      其余是增量累加、连续无界），但本函数【只用 cos/sin】，
 *    而卷绕和圈数在 cos/sin 下是等价的 —— 差 2πk 的角，余弦正弦一模一样。
 *    所以不管底下是哪一种，这里都不用改。
 *
 * ⚠️⚠️ 但有一件事【必须】在实车上定，定了才有意义：
 *      两个修正量 APP_YAW_SIGN / APP_YAW_ZERO_RAD（见上面那段的量法）。
 *      现在都是占位值，也就是说换系这一步【还没真正生效】——
 *      拿不到修正量时它的实际效果等同于恒返回 0，即"车头正对操作手"：
 *      底盘不转时能用，一推小陀螺车就往斜里走。
 *
 * ⚠️ 另一个待定的点是【零点语义】，取决于电机是绝对式还是相对式：
 *      绝对式（如直驱 GM6020）—— 每圈内唯一，零点由电机自己的
 *        angle-offset-deg 定死，上电在哪都不影响，最省事。
 *      相对式（增量累加）—— angle 从上电那一刻开始累加，也就是说
 *        【上电时云台停在哪，那一处就是零点】。这样 APP_YAW_ZERO_RAD
 *        只在"上电时云台恰好摆正"的前提下才对，换个上电姿势就偏了。
 *      这一条不改本函数，但要记在账上 —— 等电机定下来再看要不要处理
 *      （绝对式的话直接退休一个常数，相对式的话得加个"上电归零"的流程）。
 */
static float chassis_frame_yaw(void) {
	const gimbal_config *gc = gimbal_dev->config;
	motor_data md;

	/* 拿不到就当 0：退化成"车头正对操作手"，是个能开的状态，
	 * 不是"不动"也不是"乱转"。yaw_motor 为 NULL 在 DT 里就编不出来，
	 * 这里查它是为了 motor_get 之前不留空指针。 */
	if (gc->yaw_motor == NULL || motor_get(gc->yaw_motor, &md) != 0) {
		return 0.0f;
	}

	return APP_YAW_SIGN * md.angle + APP_YAW_ZERO_RAD;
}

/* ═══════════════════ 遥控器 → 目标量 ═══════════════════ */

/**
 * @brief 把遥控器状态翻译成四个驱动的目标量
 *
 * @param rc       遥控器快照
 * @param link_ok  false = 遥控器掉线，按"什么都不做"处理
 * @param fresh    true = 这一拍 rc 是【新的一帧】（rc.frame_count 变了）
 *
 * ⚠️ fresh 不是优化，是正确性：下面 pitch 那一步是【跨帧累加】的，
 *    而本函数每毫秒被调一次、DBUS 十几毫秒才来一帧 —— 不看 fresh 的话
 *    同一帧的鼠标 Δ 会被累加十几遍，云台走得比手快十几倍。
 *    其余的量都是"电平"，重复写无害，所以只有 pitch 那一步需要判 fresh。
 *
 * 本函数必须【幂等】：同样的 rc + fresh 调两次，结果一样。
 * 下面几处"只在变化时调"就是为了这个 —— 见各自的注释。
 */
static void remote_to_command(const DR16_Data *rc, bool link_ok, bool imu_ok, bool fresh) {
	/* ── ① 总闸：右拨杆（+ 两条失效保护）──
	 * 只有这一处决定 enabled。
	 *
	 * 底盘和云台【共用同一个总闸】——只停一个的话，急停之后云台还会带着枪口
	 * 转，那比两个都停危险。但云台【多一道】IMU 判据，见下面。
	 *
	 * 上电默认是失能（chassis_data / gimbal_data 都是零初始化），
	 * 也就是"不拨杆车不动"，这是故意的。 */
	const bool chassis_enable = link_ok && (rc->switch_R == SW_UP);

	/* ★ 云台多一道 IMU 新鲜度。理由：云台环的【唯一】反馈源是 IMU，
	 *   而目标是跟着鼠标走的 —— IMU 一死，实测角恒为 0、目标角还在动，
	 *   误差越拉越大，云台会一路顶到机械限位。底盘不读姿态，所以不受影响。 */
	const bool gimbal_enable = chassis_enable && imu_ok;

	if (gimbal_enable && !was_gimbal_enabled) {
		/* 使能瞬间把 pitch 目标【对齐到当前实际角度】。
		 * 不对齐的话：失能期间 pitch_target_rad 停在旧值，重新使能的那一拍
		 * 云台会从当前位置猛甩到旧目标 —— 那是一发实实在在的机械冲击。
		 * 对齐之后"从哪停的就从哪接着走"，代价只是重开后 pitch 换个参考。 */
		gimbal_status gs;

		if (gimbal_get_status(gimbal_dev, &gs) == 0) {
			pitch_target_rad = gs.pitch_angle;
		}
		yaw_target_rad = 0.0f;   /* AUTO 还没做，先归零占位 */
	}
	was_gimbal_enabled = gimbal_enable;

	chassis_set_enabled(chassis_dev, chassis_enable);
	gimbal_set_enabled(gimbal_dev, gimbal_enable);

	if (!chassis_enable) {
		/* 急停 / 掉线时后面的指令一概不写：驱动自己会把输出清零、积分器清掉。
		 * 这时候还把摇杆值写进去，只是给下次使能埋个惊喜。 */
		return;
	}

	/* IMU 挂了但遥控器正常：底盘照常能开，云台的指令【照写不误】。
	 * 不特殊处理是对的 —— 云台此刻是失能的，驱动里的失能分支会把输出清零、
	 * 并作废角度基准，写进去的摇杆值一个都落不了地。等 IMU 回来时走的是
	 * 上面那条使能沿，pitch 目标会被重新对齐到当时的实际角。 */

	/* ── ② 平移 + 自转（机体系）──
	 * ⚠️ 下面三条方向是按"标准 DBUS"写的，但 dr16.c 只对 Right_X 做了取反，
	 *    其余三轴原样透传（见那里"其余三轴的方向按上层需求再定"）。
	 *    所以【第一版上车必须低速试方向】，反了就翻这里的符号，别的都不用动。 */
	float vx = stick_norm(rc->Left_Y) * APP_MAX_LINEAR;   /* 左摇杆竖直，上推为正 → 向前 */
	float vy = -stick_norm(rc->Left_X) * APP_MAX_LINEAR;  /* 左摇杆水平，右推为正 → 车体系 y 是【车左为正】，故取负 */
	float wz = stick_norm(rc->Right_X) * APP_MAX_ANGULAR;

	/* ── ③ 机体系 → 车体系 ──
	 * 把操作手要的速度，从他的系（云台朝向）表示，换算到车体系表示。
	 *
	 * 设 x̂_o = 云台朝前、ŷ_o = 云台朝左，α 是从 x̂_c 转到 x̂_o 的角。则
	 *     x̂_o = cosα·x̂_c + sinα·ŷ_c
	 *     ŷ_o = −sinα·x̂_c + cosα·ŷ_c
	 * 操作手要的 v_o = vx·x̂_o + vy·ŷ_o 展开到车体系就是下面这两行，
	 * 也就是【把向量转 +α】—— 不是 −α，方向别搞反。
	 *
	 * 验算 α = 90°（云台指向车的左边）：操作手推前进，车该往车的左边走，
	 * 即 vx_c = 0、vy_c = 1 → 下面第一行 1·cos90 − 0 = 0 ✓
	 *                              第二行 1·sin90 + 0 = 1 ✓
	 *
	 * wz 两个系共用同一根 z 轴，原样透传，不参与旋转。
	 *
	 * ⚠️ α 的两个修正量还是占位值，见 chassis_frame_yaw。 */
	const float a = chassis_frame_yaw();
	const float ca = cosf(a), sa = sinf(a);

	chassis_set_speed(chassis_dev,
	                  vx * ca - vy * sa,
	                  vx * sa + vy * ca,
	                  wz);

	/* ── ④ 拨盘：左拨杆 ──
	 * 上 = 连发，中 = 单发，下 = 停火。
	 *
	 * ⚠️⚠️ 切模式【只在变化的那一拍调】。
	 *   chassis_driver_set_trigger_mode 里有一句 trigger_target_valid = false
	 *   —— 那是为了切到单发时把角度基准作废（不然会从上次的位置一步跳出去）。
	 *   每拍都调它，等于每拍都作废基准，单发就【永远打不出一发】：
	 *   基准每拍重建、目标每拍不设，最后表现为"按了没反应"。
	 *   这个坑编译器拦不住，症状也不像"调用太频繁"，所以在这里记一笔。 */
	trigger_mode_t want;
	bool fire = false;

	switch (rc->switch_L) {
	case SW_UP:
		want = TRIGGER_MODE_AUTO;
		fire = (rc->mouse_press_l != 0);
		break;
	case SW_MID:
		want = TRIGGER_MODE_SINGLE;
		fire = (rc->mouse_press_l != 0);
		break;
	default:
		want = TRIGGER_MODE_SINGLE;
		fire = false;
		break;
	}

	if (want != last_trigger_mode) {
		chassis_set_trigger_mode(chassis_dev, want);
		last_trigger_mode = want;
	}
	chassis_trigger_set(chassis_dev, fire);

	/* ── ⑤ 云台 ──
	 * 恒 MANUAL：自瞄要上位机把绝对角度喂进来，那条链路还没做
	 * （gimbal_set_yaw_angle 就是给它准备的）。
	 *
	 * ⚠️ 和拨盘同一个坑：gimbal_driver_set_mode 切到 AUTO 时也会作废角度基准，
	 *    所以也只在变化时调。现在恒 MANUAL 其实调一次就够，但保留
	 *    "只在变化时调"的形状，等接上位机时直接改 want 就行。 */
	if (last_gimbal_mode != GIMBAL_MODE_MANUAL) {
		gimbal_set_mode(gimbal_dev, GIMBAL_MODE_MANUAL);
		last_gimbal_mode = GIMBAL_MODE_MANUAL;
	}

	/* yaw：鼠标横向 → 角速度。mouse_x 正 = 向右，而世界系 yaw 是【逆时针为正】，
	 * 向右转 = yaw 减小，所以取负。⚠️ 同 ②，上车先试方向。 */
	gimbal_set_yaw_speed(gimbal_dev, -(float)rc->mouse_x * APP_MOUSE_YAW_GAIN);

	/* pitch：鼠标纵向 → 【绝对角】，所以这里累加。
	 * ★ 只在 fresh 的那一拍累加，理由见函数头注释。
	 * 驱动会把它夹到 DT 的 pitch 行程，所以这里放心积分不用自己夹；
	 * 而且"积在后面"手感更对：顶到限位后往回拉立刻就有响应。
	 * ⚠️ mouse_y 正 = 向下，pitch 正 = 向上，所以取负。方向同样先试再定。 */
	if (fresh) {
		pitch_target_rad -= (float)rc->mouse_y * APP_MOUSE_PITCH_GAIN;
	}
	gimbal_set_pitch_angle(gimbal_dev, pitch_target_rad);

	/* 摩擦轮：按住 R 转。
	 * ⚠️ 摩擦轮是【电平】不是边沿 —— 按住才转，松开停。这和拨盘的单发/连发
	 *    是两码事，见 gimbal.h 上那段。 */
	gimbal_set_friction(gimbal_dev, ((rc->key >> KEY_R) & 1u) != 0);
}

/* ═══════════════════ 控制线程 ═══════════════════ */

static void control_thread_entry(void *a, void *b, void *c) {
	DR16_Data rc = {0};
	uint32_t last_frame  = 0;
	uint32_t last_hb     = 0;
	uint32_t stale_ticks = APP_IMU_STALE_LIMIT;  /* 上电就当它还没活，
	                                              * 收到第一次心跳才放行 */
	int64_t last_rc_ms = k_uptime_get();
	int64_t last_loop_ms = k_uptime_get();

	while (1) {
		k_sem_take(&ctrl_sem, K_FOREVER);

		/* dt 实测，不用标称的 0.001 —— 每个驱动的 update 都拿它做积分，
		 * 标称值在调度抖动下会系统性偏，积出来的 I 项就是错的。
		 * 这也是为什么 dt 是 update 的参数而不是它自己取时间。 */
		const int64_t now_ms = k_uptime_get();
		const float dt = (float)(now_ms - last_loop_ms) / 1000.0f;

		last_loop_ms = now_ms;

		dr16_get_data(&rc);

		/* fresh：帧计数变了 = 这是新的一帧。见 remote_to_command 的头注释。 */
		const bool fresh = (rc.frame_count != last_frame);

		if (fresh) {
			last_frame = rc.frame_count;
			last_rc_ms = now_ms;
		}

		/* 失控保护。上电后、收到第一帧之前 last_rc_ms 是线程启动时刻，
		 * 所以这一条同时保证"遥控器没开 = 车不动"。 */
		const bool link_ok = (now_ms - last_rc_ms) <= APP_RC_TIMEOUT_MS;

		/* 姿态新鲜度：看心跳涨没涨。用"连续多少拍没涨"而不是比时间戳，
		 * 是为了绕开 64 位量被抢占撕裂的问题（见 imu_heartbeat 的说明）。
		 * 涨了就清零，不涨就往上计 —— 计到上限就停，不用考虑溢出。 */
		if (imu_heartbeat != last_hb) {
			last_hb     = imu_heartbeat;
			stale_ticks = 0;
		} else if (stale_ticks < APP_IMU_STALE_LIMIT) {
			stale_ticks++;
		}
		const bool imu_ok = (stale_ticks < APP_IMU_STALE_LIMIT);

		remote_to_command(&rc, link_ok, imu_ok, fresh);

		/* ★ 顺序即契约：
		 *   update 只算、把四轮力矩存进 wheel_torque[]；
		 *   power_limit 是最后一道闸，限幅 + 下发；
		 *   拨盘是另一条独立闭环，不进功率预算，谁先谁后都行。 */
		chassis_update(chassis_dev, dt);
		chassis_power_limit(chassis_dev);
		chassis_trigger_update(chassis_dev, dt);

		gimbal_update(gimbal_dev, dt);
	}
}

/* ═══════════════════ 发送线程 ═══════════════════ */

/**
 * @brief 按 (CAN 口, tx_id) 分组，每组一帧
 *
 * ★ 分组规则来自 DT 里写的事实，不是按型号硬编码的：
 *     同一个 tx_id 的电机共用一帧 —— DJI 一帧装 4 台，各写各的 2 字节；
 *     达妙是每台一个 tx_id，自然每帧一台。
 *   motor_can_pack 只写【自己那 2 个字节】，所以必须先 memset 再把同组的
 *   都 pack 进来：不清零的话别的电机的槽位是上一次发送留下的残值。
 *
 * ⚠️ 上限：一帧最多 4 台（DJI 的帧布局决定的）。同一 tx_id 上挂了 5 台的话
 *    第 5 台的 id 会越界，motor_can_pack 返回 -EINVAL。这里不额外查 ——
 *    那是 DT 配错了，应该在上电时从 log 里看出来，而不是每毫秒查一遍。
 */
static void can_bus_send(void) {
	/* 已经发过帧的 (can_dev, tx_id)，用来跳过后续同组的电机 */
	const struct device *sent_dev[APP_MOTOR_MAX];
	uint32_t sent_id[APP_MOTOR_MAX];
	int sent_n = 0;

	for (int i = 0; i < motor_count; i++) {
		const motor_config *mc = motor_list[i]->config;
		bool already = false;

		for (int d = 0; d < sent_n; d++) {
			if (sent_dev[d] == mc->can_dev && sent_id[d] == mc->tx_id) {
				already = true;
				break;
			}
		}
		if (already) {
			continue;
		}
		sent_dev[sent_n] = mc->can_dev;
		sent_id[sent_n]  = mc->tx_id;
		sent_n++;

		struct can_frame frame = {
			.id  = mc->tx_id,
			.dlc = 8,
		};

		memset(frame.data, 0, sizeof(frame.data));

		/* 把同组的都塞进来。pack 打的是【当下】的 target.torque，
		 * 所以每拍必须重做 —— 把组算好后缓存起来的话，帧内容就永远停在
		 * 开机那一刻了。 */
		for (int j = 0; j < motor_count; j++) {
			const motor_config *mj = motor_list[j]->config;

			if (mj->can_dev == mc->can_dev && mj->tx_id == mc->tx_id) {
				motor_can_pack(motor_list[j], frame.data);
			}
		}

		/* 阻塞调用，但这是在【线程】里不是 ISR 里。超时给 1ms：
		 * 发送线程这一拍最坏也就拖 1ms，下一拍 timer 已经给了信号。 */
		can_send(mc->can_dev, &frame, K_MSEC(1), NULL, NULL);
	}
}

static void send_thread_entry(void *a, void *b, void *c) {
	while (1) {
		k_sem_take(&send_sem, K_FOREVER);
		can_bus_send();
	}
}

/* ═══════════════════ 线程 / 定时器 ═══════════════════ */

/* 优先级：数字小 = 优先级高（抢占式，可抢占区间 0~14）。
 *
 *   姿态 1 > 发送 2 > 控制 3
 *
 * 姿态最高：它是【生产者】，云台环的唯一反馈源。它晚一拍，控制回路这一拍
 *   读到的就是旧角度，误差里凭空多一份延迟。而且它由 DRDY 事件驱动，
 *   平时阻塞在 k_event_wait 上，只有新数据到了才跑，占不了多少 CPU。
 * 发送次之：控制线程算完只是写个结构体（几纳秒），发送线程这一拍不发出去
 *   就得等下一毫秒，电机目标随之晚一毫秒生效。
 * 控制最低：它的 dt 是【实测】的，偶尔被上面两条挤一下会被 dt 自动补偿掉，
 *   不像前两者那样会把误差直接漏进控制律。
 *
 * ⚠️ 三条线程的数字都是【在这个应用里】排的，没有绝对正确的值，只有相对次序
 *   说得通。底下 BMI08x 驱动自带的那两条投递线程不在这条链上：它们是
 *   K_PRIO_COOP(10) = −6，【协作式】，天然高于所有可抢占线程，怎么排都动不了。
 *   Kconfig 里写的 10 只是喂给宏的【索引】，不是最终优先级。
 *
 * ⚠️ 栈 2176 而不是 2048：多出的 128 是硬件 lazy stacking 的 FP 异常帧。
 *    K_FP_REGS 是必须的 —— 漏标的话 MPU 保护区按 32 算而不是 128，
 *    溢出会越过保护区继续写而检测不到。这两个数都是 imu_test 里验证过的。 */
K_THREAD_DEFINE(imu_thread,     2176, imu_thread_entry,     NULL, NULL, NULL, 1, K_FP_REGS, 0);
K_THREAD_DEFINE(send_thread,    2048, send_thread_entry,    NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(control_thread, 2048, control_thread_entry, NULL, NULL, NULL, 3, 0, 0);

static void ctrl_timer_cb(struct k_timer *t) { k_sem_give(&ctrl_sem); }
static void send_timer_cb(struct k_timer *t) { k_sem_give(&send_sem); }

K_TIMER_DEFINE(ctrl_timer, ctrl_timer_cb, NULL);
K_TIMER_DEFINE(send_timer, send_timer_cb, NULL);

/* ═══════════════════ 启动 ═══════════════════ */

/** 把清单里出现的 CAN 口都 start 一遍。去重是因为一辆车可能只用一条总线，
 *  重复 start 是错的（第二次返回 -EALREADY，但没必要去踩）。*/
static void can_buses_start(void) {
	const struct device *started[APP_MOTOR_MAX];
	int started_n = 0;

	for (int i = 0; i < motor_count; i++) {
		const motor_config *mc = motor_list[i]->config;
		bool dup = false;

		for (int d = 0; d < started_n; d++) {
			if (started[d] == mc->can_dev) {
				dup = true;
				break;
			}
		}
		if (dup) {
			continue;
		}
		started[started_n++] = mc->can_dev;

		int ret = can_start(mc->can_dev);

		if (ret != 0 && ret != -EALREADY) {
			LOG_ERR("can_start 失败: %d", ret);
		}
	}
}

int main(void) {
	if (!device_is_ready(chassis_dev)) {
		LOG_ERR("chassis 设备没就绪");
		return -1;
	}
	if (!device_is_ready(gimbal_dev)) {
		LOG_ERR("gimbal 设备没就绪");
		return -1;
	}

	/* 电机清单必须在 start 定时器【之前】建好 —— 线程一跑起来就要用它 */
	motor_list_build();
	can_buses_start();

	/* 遥控器：这一句一返回，UART 中断随时可能来 */
	dr16_init(dr16_uart);

	/* ── 把 IMU 的 DRDY 中断挂上 ──
	 * ★ 这一步【不做的话，什么都没发生】：姿态线程会永远阻塞在 k_event_wait
	 *   上（没有中断来投事件），imu_data.angle 恒为 0，云台拿一个瞎了的反馈
	 *   去算误差。编译链接全过、上电就废，没有任何报警 —— 所以下面每个
	 *   返回值都必须查。
	 *
	 * ⚠️ 两路要走【两份独立的 Kconfig + 两个源文件】，-ENOSYS 彼此独立，
	 *    只查一路等于没查（见 INTERVIEW-QUESTIONS 4.4）。
	 *
	 * 用驱动 config 里的子设备指针，DT 路径只写一次（写在本 overlay 的
	 * imu 节点上），这里不用再抄一遍。 */
	const imu_config *imu_cfg = imu_dev->config;
	int ret;

	ret = sensor_trigger_set(imu_cfg->accel_dev, &accel_trig, imu_drdy_handler);
	if (ret < 0) {
		LOG_ERR("sensor_trigger_set(accel) 失败: %d —— 云台将没有反馈，禁止使能", ret);
	}

	ret = sensor_trigger_set(imu_cfg->gyro_dev, &gyro_trig, imu_drdy_handler);
	if (ret < 0) {
		LOG_ERR("sensor_trigger_set(gyro) 失败: %d —— 云台将没有反馈，禁止使能", ret);
	}

	/* 不在这里 return -1：底盘【不依赖姿态】（chassis_config.imu 是个死字段），
	 * IMU 挂了也还能推着走。真正的保护落在控制线程那条心跳判据上 ——
	 * 它会在姿态不新鲜时拒绝使能云台。这是 fail-closed 而不是"整个固件不启动"。 */

	/* ── 裁判系统 ──
	 * 还没有这个设备。在它到位之前功率上限写死一个常数，好让限幅逻辑
	 * 真的跑起来（设 0 的话 chassis_power_scale 会解释成"不限制"，
	 * 那样限幅这一段就永远不被执行，等于埋着一个没验证过的分支）。
	 * 接上裁判系统之后，把这一句换成 chassis_set_power_limit(chassis_dev,
	 * 裁判系统给的功率上限)，10Hz 调一次就行。 */
	chassis_set_power_limit(chassis_dev, APP_POWER_LIMIT_W);

	k_timer_start(&ctrl_timer, K_NO_WAIT, K_MSEC(CTRL_PERIOD_MS));
	k_timer_start(&send_timer, K_NO_WAIT, K_MSEC(CTRL_PERIOD_MS));

	LOG_INF("skywalker 启动完成");
	return 0;
}
