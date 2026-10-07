#include "drivers/imu/imu.h"

#include <string.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include "drivers/kalman_filter/kalman_filter.h"
#include "drivers/pid/pid.h"

#define DT_DRV_COMPAT skywalker_imu

static const struct imu_filter_api *imu_get_api(const char *estimator);

// ─── 从 DTS 提取 imu_config（ROM） ───
// accel-dev / gyro-dev / heat-dev / filter-dev 为 phandle。
// estimator 为 string，通过 DT_INST_PROP 提取，用于选择解算实现。
#define IMU_CONFIG_DEFINE(inst)                                           \
    static const imu_config imu_config_##inst = {                         \
        .accel_dev  = DEVICE_DT_GET(DT_INST_PHANDLE(inst, accel_dev)),    \
        .gyro_dev   = DEVICE_DT_GET(DT_INST_PHANDLE(inst, gyro_dev)),     \
        .heat_dev   = DEVICE_DT_GET(DT_INST_PHANDLE(inst, heat_dev)),     \
        .filter_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, filter_dev)),   \
        .pid_dev    = DEVICE_DT_GET(DT_INST_PHANDLE(inst, pid_dev)),      \
        .estimator  = DT_INST_PROP(inst, estimator),                      \
    };

/**
 * @brief Zephyr 设备初始化函数
 *
 * 检查所有子设备（accel / gyro / heat / filter）是否就绪，并将 data 清零。
 *
 * @param dev Zephyr 设备指针
 * @return 0 表示成功，-ENODEV 表示子设备不可用
 */
static int skywalker_imu_init(const struct device *dev) {
    const imu_config *cfg = dev->config;

    if (!device_is_ready(cfg->accel_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->gyro_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->heat_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->filter_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->pid_dev)) {
        return -ENODEV;
    }

    // 调用滤波器自身的初始化（如 EKF 设定初始四元数）
    const struct imu_filter_api *api = imu_get_api(cfg->estimator);
    if (api != NULL) {
        api->init(cfg->filter_dev);
    }

    imu_data *data = dev->data;
    data->temp = 0.0f;
    for (int i = 0; i < 3; i++) {
        data->accel[i] = 0.0f;
        data->gyro[i]  = 0.0f;
        data->angle[i] = 0.0f;
    }

    k_mutex_init(&data->mutex);
    return 0;
}

// ─── 为单个 DT 实例注册 Zephyr device ───
// data 大小固定，直接定义；config 由 IMU_CONFIG_DEFINE 生成。
#define IMU_INST(inst)                                                     \
    IMU_CONFIG_DEFINE(inst);                                               \
    static imu_data imu_data_##inst;                                       \
    DEVICE_DT_DEFINE(DT_DRV_INST(inst),                                    \
                     skywalker_imu_init,                                   \
                     NULL,                                                 \
                     &imu_data_##inst,                                     \
                     &imu_config_##inst,                                   \
                     POST_KERNEL,                                          \
                     91,                                                   \
                     NULL);

// ─── 展开所有 status = "okay" 的 DT 实例 ───
DT_INST_FOREACH_STATUS_OKAY(IMU_INST)

// ─── 传感器数据获取 ───
/**
 * @brief 从加速度计获取原始数据（含片上温度）
 *
 * 温度跟着加速度计走，不单独开一个函数：SENSOR_CHAN_DIE_TEMP 读的是 accel
 * 芯片的片上温度，而且是【读已缓存好的寄存器、不产生 I/O】，不额外花 SPI 时间。
 *
 * @param dev IMU 设备指针
 */
void imu_fetch_accel(const struct device *dev) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;

    struct sensor_value val[3];
    struct sensor_value temp;

    sensor_sample_fetch(cfg->accel_dev);
    sensor_channel_get(cfg->accel_dev, SENSOR_CHAN_ACCEL_XYZ, val);
    data->accel[0] = sensor_value_to_float(&val[0]);
    data->accel[1] = sensor_value_to_float(&val[1]);
    data->accel[2] = sensor_value_to_float(&val[2]);

    sensor_channel_get(cfg->accel_dev, SENSOR_CHAN_DIE_TEMP, &temp);
    data->temp = sensor_value_to_float(&temp);
}

/**
 * @brief 从陀螺仪获取原始数据
 *
 * @param dev IMU 设备指针
 */
void imu_fetch_gyro(const struct device *dev) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;

    struct sensor_value val[3];

    sensor_sample_fetch(cfg->gyro_dev);
    sensor_channel_get(cfg->gyro_dev, SENSOR_CHAN_GYRO_XYZ, val);
    data->gyro[0] = sensor_value_to_float(&val[0]);
    data->gyro[1] = sensor_value_to_float(&val[1]);
    data->gyro[2] = sensor_value_to_float(&val[2]);
}

// ─── 姿态解算调度 ───
//
// 拆成两个入口，是为了让【每一步的节拍由喂它的那个传感器决定】：
//
//   imu_predict 吃陀螺仪   → 陀螺仪数据就绪时调（1000 Hz）
//   imu_correct 吃加速度计 → 加速度计数据就绪时调（800 Hz）
//
// 为什么 dt 只属于 predict：
//   predict 里 F = I + 0.5·Ω·dt 是纯积分因子，dt 偏 → 旋转尺度偏，
//   而加速度计观测不了旋转尺度（它只给重力方向），yaw 又无绝对参考，
//   误差会无界累积；
//   correct 里没有 dt —— 卡尔曼增益 K = P·Hᵀ·(H·P·Hᵀ+R)⁻¹ 是
//   "按当前不确定度把状态往观测拉一把"，跟两次观测隔了多久无关。

/**
 * @brief 预测步：调用当前滤波器的 predict
 *
 * @param dev IMU 设备指针
 * @param dt  距上次 imu_predict 的实测间隔（秒）
 */
void imu_predict(const struct device *dev, float dt) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;
    const struct imu_filter_api *api = imu_get_api(cfg->estimator);

    if (api == NULL) return;

    api->predict(cfg->filter_dev, data->gyro, dt, data->angle);
}

/**
 * @brief 校正步：调用当前滤波器的 correct
 *
 * @param dev IMU 设备指针
 */
void imu_correct(const struct device *dev) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;
    const struct imu_filter_api *api = imu_get_api(cfg->estimator);

    if (api == NULL) return;

    api->correct(cfg->filter_dev, data->accel);
}

/**
 * @brief 算出姿态角并写入 data->angle（唯一写方）
 *
 * ★ 为什么角度要单独一个入口，而不是塞进 predict / correct 的末尾：
 *
 *   imu_ekf_get_angle 是【有状态】的 —— 它内部要写 ekf.YawPrev 和
 *   ekf.YawRoundCount（见本文件末尾的 Yaw 跨圈累积）。也就是说
 *   【调用次数本身就是语义的一部分】。
 *
 *   原来 predict 和 correct 捆在一起，一轮调一次。
 *   如果拆开后让两边各在末尾调一次，就变成一轮调两次。
 *   眼下不会出错，因为 ekf.YawTotal 全项目只写不读（死代码），
 *   输出取的是原始 yaw；但只要哪天把它接上（angle[2] = ekf.YawTotal，
 *   那本来就是这段代码的本意），yaw 在 ±π 附近跳变时跨圈计数就会被多算。
 *
 *   所以干净的做法是：predict / correct 都不碰 angle，
 *   由调用方在每个周期末尾调一次本函数。
 *
 * ★ 为什么 tmp_angle 必须在【锁外】算：
 *
 *   临界区宽度 = 别人最坏的阻塞时间。本函数属于 1 kHz 的控制回路，
 *   它的 WCET 是一分钱都不能多花的。对比一下两段宽度：
 *
 *     锁外：三次 atan2 + 一次 sqrt      几百个周期
 *     锁内：三行 float 赋值            <  10 个周期
 *
 *   把解算关进锁里，等于每次发布都让等锁的人白等两个数量级，而这个等待
 *   会直接加进估计线程的 WCET。tmp_angle 是【栈上局部变量】，别的线程
 *   看不见它 —— 所以算它根本不需要保护，只有【发布】到 data->angle
 *   的那三行需要。
 *
 * @param dev IMU 设备指针
 */
void imu_update_angle(const struct device *dev) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;
    const struct imu_filter_api *api = imu_get_api(cfg->estimator);

    if (api == NULL) return;

    float tmp_angle[3];
    api->get_angle(cfg->filter_dev, tmp_angle);

    // 互斥锁保护：写方是本函数，读方是 imu_get_angle
    k_mutex_lock(&data->mutex, K_FOREVER);
    data->angle[0] = tmp_angle[0];
    data->angle[1] = tmp_angle[1];
    data->angle[2] = tmp_angle[2];
    k_mutex_unlock(&data->mutex);
}

/**
 * @brief 取一份姿态角快照到调用者的 buffer（唯一读方）
 *
 * ★ 为什么是「拷贝出去」而不是「交出指针」：
 *
 *   原来的接口是让调用者自己拿 data->angle 的地址去读，而那个地址是从
 *   imu_dev->data 掏出来的驱动【私有内存】。一旦这个指针流到应用层，
 *   系统里任何一行代码都可以绕过锁直接读写它 —— 锁就退化成心理安慰，
 *   而且编译器一声不响。
 *
 *   改成拷贝之后，调用者拿到的是自己栈上的 buffer，天然不共享，
 *   也就【不可能绕过】这把锁。想读角度只有这一个入口。
 *
 *   附带好处：拷贝发生在调用者的栈上，锁只需要盖住这里的发布/取走两下，
 *   不需要在调用者那边再开一个临界区。
 *
 * ⚠️ k_mutex_lock 传 K_FOREVER 是安全的：本函数只持锁做三次读，
 *    不会申请任何别的资源，因此不构成死锁环的一边。
 *
 * @param dev       IMU 设备指针
 * @param out_angle 输出姿态角 (rad), [roll, pitch, yaw]
 */
void imu_get_angle(const struct device *dev, float out_angle[3]) {
    imu_data *data = dev->data;

    // 唯一读方。锁保证三个 float 是同一时刻的，不会拼出新 roll 配旧 pitch
    k_mutex_lock(&data->mutex, K_FOREVER);
    out_angle[0] = data->angle[0];
    out_angle[1] = data->angle[1];
    out_angle[2] = data->angle[2];
    k_mutex_unlock(&data->mutex);
}

/**
 * @brief 读单个轴的角速度（rad/s，原始量）
 *
 * 【不加锁】——这是本函数唯一值得说的地方。
 *
 * 读方（云台 yaw / pitch 两个速度内环）每次只取一个 float。单次对齐的 32 位
 * 访问在 Cortex-M 上就是一条 load，中间插不进另一个执行流，所以它天然是原子的，
 * 不需要任何同步。
 *
 * ⚠️ 不要为了"保险"把它塞进 data->mutex：那是白送的代价。锁一进，估计线程里的
 *    imu_update_angle 就多出一个可能被阻塞的点，而这个点保护的是一份根本不会
 *    被破坏的数据。临界区宽度 = 别人最坏阻塞时间，这笔账要记。详见 imu.h。
 *
 * ⚠️ 拿到的是【当前值】不是【快照】：连读两个轴得到的是两个不同时刻的值
 *    （IMU 线程可能正好在两次调用之间跑了一轮）。
 *    对 yaw / pitch 这两个互相独立的速度环，这正是想要的——各自用各自最新鲜的。
 *    但如果哪天需要"三轴同一拍"（比如用三轴角速度积分姿态），这条接口不适用。
 *
 * @param dev  IMU 设备指针
 * @param axis 轴，用 imu_axis_t（IMU_AXIS_ROLL / PITCH / YAW）
 * @return 该轴角速度 (rad/s)；越界返回 0.0f
 */
float imu_get_gyro_axis(const struct device *dev, imu_axis_t axis) {
    const imu_data *data = dev->data;

    if (axis >= ARRAY_SIZE(data->gyro)) {
        return 0.0f;
    }
    return data->gyro[axis];
}

// ─── 温度控制 ───
/**
 * @brief PID 温度控制
 *
 * 通过 pid_dev 计算 PWM 脉宽并写入 heat_dev，对标 mambo IMU 恒温方案。
 * 建议低频调用（~10 Hz），PID 参数在 DT overlay 中配置。
 *
 * @param dev         IMU 设备指针
 * @param target_temp 目标温度 (°C)
 * @param dt          距上次调用时间（秒）
 */
// 对标 mambo：维持 50°C 的基础加热量（单位 ns），作为 PID 前馈
#define HEAT_OFFSET_NS 6750000.0f

void imu_heat_control(const struct device *dev, float target_temp, float dt) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;

    // PID 计算：setpoint=target, measurement=current，offset 作为前馈（对标 mambo）
    float output = pid_update(cfg->pid_dev->data, cfg->pid_dev->config,
                              target_temp, data->temp, dt, HEAT_OFFSET_NS);

    // 负值截断（加热器不能制冷）
    if (output < 0.0f) output = 0.0f;

    // PWM 输出：周期 20 ms（50 Hz，对标 mambo），output 单位 ns
    // 通道 4 对应 overlay 里 &timers3 的 pinctrl tim3_ch4_pb1（STM32 PWM 通道从 1 起）
    uint32_t period = PWM_MSEC(20);
    int ret = pwm_set(cfg->heat_dev, 4, period, (uint32_t)output, PWM_POLARITY_NORMAL);

    // pwm_set 的失败是【静默】的：period_cycles 越过 16 位 ARR 上限、设备没就绪、
    // 通道号非法 —— 全都只返回负值。现象是"温度上不去"，不报任何错。
    // 和 sensor_trigger_set 的 -ENOSYS 同一类坑：返回值是唯一的报警器。
    //
    // 用 static 闩锁【只在状态变化时】报一次：本函数 ~10 Hz 调用，
    // 无脑每次 printk 会以 10 行/秒 的速率刷屏，而"状态没变"信息量为零。
    // 保留恢复分支是为了能区分【瞬时抖动】和【持续性故障】。
    static bool heat_err_latched = false;

    if (ret < 0 && !heat_err_latched) {
        printk("imu_heat_control: pwm_set failed: %d\n", ret);
        heat_err_latched = true;
    } else if (ret == 0 && heat_err_latched) {
        printk("imu_heat_control: pwm_set recovered\n");
        heat_err_latched = false;
    }
}



////////////////////////////////////////////////////////////////////////////////
//  EKF 姿态解算（四元数扩展卡尔曼滤波）
//  对标 mambo IMU_QuaternionEKF，包含零偏估计、LPF、卡方检验、自适应增益、Yaw 圈数
////////////////////////////////////////////////////////////////////////////////

// EKF 持久状态
static struct {
    float GyroBias[3];         // 陀螺零偏 (rad/s)
    float AccelFiltered[3];    // 加速度低通滤波值
    float ChiSquare;           // 卡方检验值
    float AdaptiveGainScale;   // 自适应增益
    float YawTotal;            // 连续 yaw（跨圈累计）
    float YawPrev;             // 上一拍 yaw
    int16_t YawRoundCount;     // 跨圈计数
    uint64_t UpdateCount;      // 累计调用次数
    uint8_t ConvergeFlag;      // 收敛标志
    uint64_t ErrorCount;       // 连续异常计数
} ekf;

#define EKF_Q1            10.0f // 四元数过程噪声系数
#define EKF_R             1e6f // 加速度观测噪声
#define EKF_CHI_THRESHOLD 1e-8f // 卡方检验阈值

/** @brief 快速 1/sqrt(x) */
static float inv_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 初始化：X = [1,0,0,0]，清零 ekf 状态
 */
static void imu_ekf_init(const struct device *dev) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;

    kf->X.pData[0] = 1.0f;
    kf->X.pData[1] = 0.0f;
    kf->X.pData[2] = 0.0f;
    kf->X.pData[3] = 0.0f;

    memset(&ekf, 0, sizeof(ekf));
    ekf.AdaptiveGainScale = 1.0f;
}

/**
 * @brief 预测：零偏估计 + 四元数运动学 F + 协方差传播
 */
static void imu_ekf_predict(const struct device *dev, const float gyro[3], float dt, float angle[3]) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;

    // ─── 1. 陀螺零偏在线估计（静止时 LPF 累积） ───
    float gyro_norm = inv_sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    if (gyro_norm > 0.0f) gyro_norm = 1.0f / gyro_norm;

    float acc_norm_val;
    arm_sqrt_f32(ekf.AccelFiltered[0] * ekf.AccelFiltered[0] +
                 ekf.AccelFiltered[1] * ekf.AccelFiltered[1] +
                 ekf.AccelFiltered[2] * ekf.AccelFiltered[2], &acc_norm_val);
    if (gyro_norm < 0.2f && fabsf(acc_norm_val - 9.8f) < 0.35f) {
        if (ekf.UpdateCount == 0) {
            ekf.GyroBias[0] = gyro[0]; ekf.GyroBias[1] = gyro[1]; ekf.GyroBias[2] = gyro[2];
        }
        ekf.GyroBias[0] = ekf.GyroBias[0] * 0.9995f + gyro[0] * 0.0005f;
        ekf.GyroBias[1] = ekf.GyroBias[1] * 0.9995f + gyro[1] * 0.0005f;
        ekf.GyroBias[2] = ekf.GyroBias[2] * 0.9995f + gyro[2] * 0.0005f;
    }

    // ─── 2. 减去零偏 ───
    float gx = gyro[0] - ekf.GyroBias[0];
    float gy = gyro[1] - ekf.GyroBias[1];
    float gz = gyro[2] - ekf.GyroBias[2];

    // ─── 3. F = I + 0.5·Ω·dt ───
    float hwx = 0.5f * gx * dt, hwy = 0.5f * gy * dt, hwz = 0.5f * gz * dt;
    kf->F.pData[0]  = 1.0f;  kf->F.pData[1]  = -hwx; kf->F.pData[2]  = -hwy; kf->F.pData[3]  = -hwz;
    kf->F.pData[4]  = hwx;   kf->F.pData[5]  = 1.0f;  kf->F.pData[6]  = hwz;  kf->F.pData[7]  = -hwy;
    kf->F.pData[8]  = hwy;   kf->F.pData[9]  = -hwz;  kf->F.pData[10] = 1.0f;  kf->F.pData[11] = hwx;
    kf->F.pData[12] = hwz;   kf->F.pData[13] = hwy;   kf->F.pData[14] = -hwx;  kf->F.pData[15] = 1.0f;

    // ─── 4. Q = EKF_Q1·dt·I ───
    for (int i = 0; i < 16; i++) kf->Q.pData[i] = 0.0f;
    float qv = EKF_Q1 * dt;
    kf->Q.pData[0] = qv; kf->Q.pData[5] = qv; kf->Q.pData[10] = qv; kf->Q.pData[15] = qv;

    KalmanFilter_Predict(kf);

    float n = inv_sqrt(kf->X.pData[0] * kf->X.pData[0] + kf->X.pData[1] * kf->X.pData[1] +
                       kf->X.pData[2] * kf->X.pData[2] + kf->X.pData[3] * kf->X.pData[3]);
    if (n > 0.0f) for (int i = 0; i < 4; i++) kf->X.pData[i] *= n;
}

/**
 * @brief 修正：LPF + 卡方检验 + 自适应增益 + 卡尔曼更新
 */
static void imu_ekf_correct(const struct device *dev, const float accel[3]) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;

    // ─── 1. 加速度低通滤波 ───
    if (ekf.UpdateCount == 0) {
        ekf.AccelFiltered[0] = accel[0];
        ekf.AccelFiltered[1] = accel[1];
        ekf.AccelFiltered[2] = accel[2];
    }
    float lpf = 0.05f;  // 截止频率 ≈ 3 Hz
    ekf.AccelFiltered[0] = ekf.AccelFiltered[0] * (1.0f - lpf) + accel[0] * lpf;
    ekf.AccelFiltered[1] = ekf.AccelFiltered[1] * (1.0f - lpf) + accel[1] * lpf;
    ekf.AccelFiltered[2] = ekf.AccelFiltered[2] * (1.0f - lpf) + accel[2] * lpf;

    // ─── 2. z = AccelFiltered / |AccelFiltered| ───
    float acc_norm;
    arm_sqrt_f32(ekf.AccelFiltered[0] * ekf.AccelFiltered[0] +
                 ekf.AccelFiltered[1] * ekf.AccelFiltered[1] +
                 ekf.AccelFiltered[2] * ekf.AccelFiltered[2], &acc_norm);
    if (acc_norm < 0.01f) return;
    float z_buf[3] = {ekf.AccelFiltered[0] / acc_norm,
                      ekf.AccelFiltered[1] / acc_norm,
                      ekf.AccelFiltered[2] / acc_norm};

    // ─── 3. 预测重力方向 h(q) 及 新息 ───
    float q0 = kf->X.pData[0], q1 = kf->X.pData[1];
    float q2 = kf->X.pData[2], q3 = kf->X.pData[3];
    float h0 = 2.0f * (q1 * q3 - q0 * q2);
    float h1 = 2.0f * (q0 * q1 + q2 * q3);
    float h2 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float y0 = z_buf[0] - h0, y1 = z_buf[1] - h1, y2 = z_buf[2] - h2;
    ekf.ChiSquare = y0 * y0 + y1 * y1 + y2 * y2;

    // ─── 4. 卡方检验 ───
    float acc_mag;
    arm_sqrt_f32(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2], &acc_mag);
    int stable = (fabsf(acc_mag - 9.8f) < 0.25f);

    if (ekf.ChiSquare < 0.5f * EKF_CHI_THRESHOLD) ekf.ConvergeFlag = 1;

    if (ekf.ChiSquare > EKF_CHI_THRESHOLD && ekf.ConvergeFlag) {
        if (stable) ekf.ErrorCount++;
        else        ekf.ErrorCount = 0;
        if (ekf.ErrorCount > 50) {
            ekf.ConvergeFlag = 0;
        } else {
            return;  // 残差异常，跳过修正
        }
    } else {
        // ─── 5. 自适应增益 ───
        if (ekf.ChiSquare > 0.1f * EKF_CHI_THRESHOLD && ekf.ConvergeFlag)
            ekf.AdaptiveGainScale = (EKF_CHI_THRESHOLD - ekf.ChiSquare) / (0.9f * EKF_CHI_THRESHOLD);
        else
            ekf.AdaptiveGainScale = 1.0f;
        ekf.ErrorCount = 0;
    }

    // ─── 6. 自适应 R（等价于缩放 K）───
    float r_adj = EKF_R / ekf.AdaptiveGainScale;
    for (int i = 0; i < 9; i++) kf->R.pData[i] = 0.0f;
    kf->R.pData[0] = r_adj; kf->R.pData[4] = r_adj; kf->R.pData[8] = r_adj;

    // ─── 7. H = ∂h/∂q ───
    memset(kf->H.pData, 0, 12 * sizeof(float));
    kf->H.pData[0] = -2.0f * q2;  kf->H.pData[1] =  2.0f * q3;  kf->H.pData[2] = -2.0f * q0;  kf->H.pData[3] = 2.0f * q1;
    kf->H.pData[4] =  2.0f * q1;  kf->H.pData[5] =  2.0f * q0;  kf->H.pData[6] =  2.0f * q3;  kf->H.pData[7] = 2.0f * q2;
    kf->H.pData[8] =  2.0f * q0;  kf->H.pData[9] = -2.0f * q1;  kf->H.pData[10]= -2.0f * q2;  kf->H.pData[11]= 2.0f * q3;

    Matrix z;
    Matrix_Init(&z, 3, 1, z_buf);
    KalmanFilter_Correct(kf, &z);

    float n = inv_sqrt(kf->X.pData[0] * kf->X.pData[0] + kf->X.pData[1] * kf->X.pData[1] +
                       kf->X.pData[2] * kf->X.pData[2] + kf->X.pData[3] * kf->X.pData[3]);
    if (n > 0.0f) for (int i = 0; i < 4; i++) kf->X.pData[i] *= n;

    ekf.UpdateCount++;
}

/**
 * @brief 四元数 → Euler (rad) + Yaw 圈数累计
 */
static void imu_ekf_get_angle(const struct device *dev, float angle[3]) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;
    float q0 = kf->X.pData[0], q1 = kf->X.pData[1];
    float q2 = kf->X.pData[2], q3 = kf->X.pData[3];

    float roll, roll_tmp, pitch, yaw;
    float asin_arg = -2.0f * (q1 * q3 - q0 * q2);
    arm_sqrt_f32(1.0f - asin_arg * asin_arg, &roll_tmp);
    arm_atan2_f32(asin_arg, roll_tmp, &roll);
    arm_atan2_f32(2.0f * (q0 * q1 + q2 * q3), 2.0f * (q0 * q0 + q3 * q3) - 1.0f, &pitch);
    arm_atan2_f32(2.0f * (q0 * q3 + q1 * q2), 2.0f * (q0 * q0 + q1 * q1) - 1.0f, &yaw);

    // Yaw 跨圈累计
    if (yaw - ekf.YawPrev > PI)       ekf.YawRoundCount--;
    else if (yaw - ekf.YawPrev < -PI) ekf.YawRoundCount++;
    ekf.YawTotal = 2.0f * PI * ekf.YawRoundCount + yaw;
    ekf.YawPrev  = yaw;

    angle[0] = roll;
    angle[1] = pitch;
    angle[2] = yaw;
}

// ─── 解算方法注册 ───
// 每种算法声明一个 imu_estimator，命名规则：imu_estimator_<算法名>。
// 新增算法：实现 init/predict/correct/get_angle → 声明 imu_estimator_xxx → 在 imu_estimators[] 里加一行。
static const imu_estimator imu_estimator_ekf = {
    .name = "ekf",
    .api  = {
        .init      = imu_ekf_init,
        .predict   = imu_ekf_predict,
        .correct   = imu_ekf_correct,
        .get_angle = imu_ekf_get_angle,
    },
};

////////////////////////////////////////////////////////////////////////////////

// ─── 算法注册表 ───
// imu_get_api 遍历此表按 name 匹配，新增解算方法只需在此加一行。
static const imu_estimator *const imu_estimators[] = {
    &imu_estimator_ekf,
};

// ─── 解算方法查找 ───
static const struct imu_filter_api *imu_get_api(const char *estimator) {
    for (size_t i = 0; i < ARRAY_SIZE(imu_estimators); i++) {
        if (strcmp(estimator, imu_estimators[i]->name) == 0) {
            return &imu_estimators[i]->api;
        }
    }
    return NULL;
}
