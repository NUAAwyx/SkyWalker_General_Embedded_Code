#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include "drivers/imu/imu.h"
#include "lib/vofa/vofa.h"

#define IMU_TARGET_TEMP  50.0f
#define HEAT_PERIOD_MS   100	/* 对标 mambo imu_task.c:293 的 accel_count % 80 */

#define IMU_EV_ACCEL     BIT(0)
#define IMU_EV_GYRO      BIT(1)

/* dt 走 480MHz 的 64 位周期计数器（分辨率 ≈ 2ns），不用 k_uptime_get()
 * 的 1ms 毫秒尺 —— 量 1ms 周期会得到 0，F 退化成 I，整轮预测被跳过。 */
#define CYCLE_TO_SEC  (1.0f / (float)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)

/* 先乘后除：避免 CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC 不能被 1000 整除时截断 */
#define HEAT_PERIOD_CYCLES \
	((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * HEAT_PERIOD_MS / 1000U)

/* 两个中断源各占一个位。k_sem 只有"有/没有"，不携带身份，分不清是谁来了。 */
K_EVENT_DEFINE(imu_event);

void imu_estimate_thread_entry(void *arg1, void *arg2, void *arg3);
void imu_vofa_thread_entry(void *arg1, void *arg2, void *arg3);
void imu_drdy_handler(const struct device *dev, const struct sensor_trigger *trig);

/* 优先级：估计 8 > 遥测 9（数字小 = 优先级高）。具体数字是任选的，
 *   只要保证估计的数字比遥测小、且都在可抢占区间 [0,14] 内即可。
 *   估计线程在控制回路里，它的 dt 误差会被 EKF 积分进状态再反馈回来；
 *   遥测是纯消费者，晚几毫秒没有下游。
 * ⚠️ 驱动自带的投递线程【不在这条链上】：它是 K_PRIO_COOP(10) = -6，【协作式】，
 *   天然高于所有可抢占线程（0~14），应用层怎么排都动不了它。
 *   注意 Kconfig 里的 10 只是喂给宏的【索引】，不是最终优先级。*/

/* 2176 = 2048（原可用栈）+ 128（硬件 lazy stacking 的 FP 异常帧，float.rst:111）。
 * ⚠️ 这只是【声明的可用栈】。MPU 保护区（本板 128）是额外加的，SRAM 实占 2304/线程。
 * K_FP_REGS 标记让保护区按 128 而非 32 算（thread.c:238）——
 * 该标的线程漏标，溢出会越过保护区继续写而检测不到。 */
K_THREAD_DEFINE(imu_estimate_thread, 2176, imu_estimate_thread_entry, NULL, NULL, NULL, 8, K_FP_REGS, 0);
K_THREAD_DEFINE(imu_vofa_thread, 2176, imu_vofa_thread_entry, NULL, NULL, NULL, 9, K_FP_REGS, 0);

static Vofa vofa;

/* 必须是 static：驱动存的是【指针】不是拷贝（sensor.h:884-915），
 * 局部变量一出函数就悬空 —— 而这段代码编译不给任何警告。 */
static struct sensor_trigger accel_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

static struct sensor_trigger gyro_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_GYRO_XYZ,
};

/* 跑在驱动的 OWN_THREAD 线程里，只报信不干活：correct 里 ~500B 的 VLA
 * 不能塞进驱动那 1536B 的栈。trig->chan 是"谁叫我的"的身份证。
 * ⚠️ 这里绝不能 wait —— post 和 wait 必须分属两个执行流，自己等自己 = 死锁。 */
void imu_drdy_handler(const struct device *dev, const struct sensor_trigger *trig){
	ARG_UNUSED(dev);
	if (trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		k_event_post(&imu_event, IMU_EV_ACCEL);
	} else if (trig->chan == SENSOR_CHAN_GYRO_XYZ) {
		k_event_post(&imu_event, IMU_EV_GYRO);
	}
}

void imu_estimate_thread_entry(void *arg1, void *arg2, void *arg3){
	const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(imu));

	/* 锚点必须在循环外初始化。放进循环里 = 每轮从此刻起算：
	 * 加热条件永不成立，dt 恒为 0。 */
	uint64_t last_est_cycle  = k_cycle_get_64();
	uint64_t last_heat_cycle = k_cycle_get_64();

	while (1) {
		/* reset=false：只清命中的位，没命中的留着（events.c:312），
		 * 所以线程还没处理完又来中断也不会丢。传 true 会先清零再判断。 */
		uint32_t ev = k_event_wait_safe(&imu_event,
						IMU_EV_ACCEL | IMU_EV_GYRO,
						false, K_FOREVER);

		uint64_t now_cycle = k_cycle_get_64();

		/* 1000 Hz。dt 只属于 predict —— correct 的 K 里没有 dt。
		 * 所以 last_est_cycle 只在这个分支里推进：丢采样时下次实测 dt
		 * 变成 2ms，F 按 2ms 积分，状态仍对（写死 dt=1ms 才会静默丢旋转）。 */
		if (ev & IMU_EV_GYRO) {
			imu_fetch_gyro(imu_dev);
			float dt = (float)(now_cycle - last_est_cycle) * CYCLE_TO_SEC;
			last_est_cycle = now_cycle;
			imu_predict(imu_dev, dt);
		}

		/* 800 Hz。两个 if 【不是】else if：一次唤醒可能同时带两个位。 */
		if (ev & IMU_EV_ACCEL) {
			imu_fetch_accel(imu_dev);
			imu_correct(imu_dev);
		}

		/* 【写方 · 已修】原来这里是三条裸 store 写 data->angle[0..2]，
		 * 读方（优先级 9）会在任意指令边界被本线程（8）抢进来，拼出撕裂快照。
		 * 现在 imu_update_angle 内部：【锁外】算到栈上 tmp_angle，【锁内】做三行赋值。
		 * 于是发布这件事对读方是原子的 —— 要么整套旧值，要么整套新值，没有中间态。
		 * 解算（三次 atan2）留在锁外不是偷懒：临界区宽度 = 别人最坏阻塞时间，
		 * 而本线程是 1 kHz 控制回路，它的 WCET 一个周期都赔不起。 */
		/* 一轮只调一次：get_angle 有状态（内部维护 Yaw 跨圈计数）。 */
		imu_update_angle(imu_dev);

		/* 100ms 节流，独立于解算节拍 */
		if ((now_cycle - last_heat_cycle) >= HEAT_PERIOD_CYCLES) {
			imu_heat_control(imu_dev, IMU_TARGET_TEMP,
					 (float)(now_cycle - last_heat_cycle) * CYCLE_TO_SEC);
			last_heat_cycle = now_cycle;
		}
	}
}

void imu_vofa_thread_entry(void *arg1, void *arg2, void *arg3){
	const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(imu));

	/* k_sleep 等【时间】，k_event_wait 等【事件】。这里用时间是对的：
	 * VOFA 是纯消费者，误差没有下游，不会被反馈放大。 */
	while (1) {
		/* 【读方 · 已修】原来是把 data->angle 的【地址】交给 vofa_send，
		 * 真正读的时刻落在 vofa.c 的 12 次字节拷贝循环里，被抢占的窗口
		 * 是那几十条指令 —— 而且在 float【内部】撕裂，可能拼出从未存在过的值。
		 * 现在改成：先向驱动要一份【快照】到本线程的栈上，再发自己的副本。
		 *
		 * 注意局部 data 指针已经删掉了 —— 驱动不再交出内部内存，
		 * 所以这里【不可能】绕过锁。想读角度只有 imu_get_angle 一个入口。 */
		float snap[3];
		imu_get_angle(imu_dev, snap);
		vofa_send(&vofa, snap, 3);   /* 发姿态角 roll/pitch/yaw */
		k_sleep(K_MSEC(10));
	}
}

int main(void) {
	const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
	vofa_init(&vofa, uart_dev);

	const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(imu));
	if (!device_is_ready(imu_dev)) {
		printk("imu device not ready\n");
		return -ENODEV;
	}

	/* 子设备指针直接从驱动的 config 里取 —— DT 路径只写一次
	 * （写在 imu 节点的 accel-dev = <&bmi08x_accel>），应用层不用再抄一遍。 */
	const imu_config *cfg = imu_dev->config;

	/* ⚠️ 返回值【必须】检查：配置没开时（默认就是 _TRIGGER_NONE）
	 * api->trigger_set == NULL → 返回 -ENOSYS，而编译/启动全都不报错，
	 * 中断永远不来 —— 系统静默死亡。详见 INTERVIEW-QUESTIONS 4.4 */
	int ret = sensor_trigger_set(cfg->accel_dev, &accel_trig, imu_drdy_handler);
	if (ret < 0) {
		printk("sensor_trigger_set(accel) failed: %d\n", ret);
		return ret;
	}

	/* 陀螺仪走【另一份 Kconfig + 另一个源文件】，两路的 -ENOSYS 彼此独立，
	 * 只查一路等于没查。 */
	ret = sensor_trigger_set(cfg->gyro_dev, &gyro_trig, imu_drdy_handler);
	if (ret < 0) {
		printk("sensor_trigger_set(gyro) failed: %d\n", ret);
		return ret;
	}

	return 0;
}
