#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "drivers/chassis/chassis.h"
#include "drivers/gimbal/gimbal.h"

/* 把两个 driver_api 和 config 都拉到二进制里：宏没展开成合法 C 就过不了这一关。 */
static const struct device *const c = DEVICE_DT_GET(DT_NODELABEL(skywalker_chassis));
static const struct device *const g = DEVICE_DT_GET(DT_NODELABEL(skywalker_gimbal));

int main(void) {
	const chassis_config *cc = c->config;
	const gimbal_config  *gc = g->config;

	printk("chassis: type=%d n_steer=%p trig=%p ang=%f spd=%f\n",
	       (int)cc->type, (void *)cc->steer_motors[0], (void *)cc->trigger_motor,
	       (double)cc->trigger_angle_deg, (double)cc->trigger_speed_deg_s);
	printk("gimbal: n_fric=%u fric_pid=%p fric_spd=%f pitch=[%f,%f]\n",
	       (unsigned)gc->friction_num, (void *)gc->friction_pid,
	       (double)gc->friction_speed_rad_s, (double)gc->pitch_min_rad, (double)gc->pitch_max_rad);

	/* 走一遍控制线程的真实调用顺序。chassis_set_power_limit 在这里只是为了
	 * 让新增的 api 成员和它的分发函数被真正编译到 —— 少一个成员，
	 * chassis_driver_api 的初始化器会静默留空，照样过编译。 */
	chassis_set_power_limit(c, 80.0f);

	chassis_update(c, 0.001f);
	chassis_power_limit(c);            /* ★ 必须紧跟 update，见 chassis.h */
	chassis_trigger_update(c, 0.001f);
	gimbal_update(g, 0.001f);
	return 0;
}
