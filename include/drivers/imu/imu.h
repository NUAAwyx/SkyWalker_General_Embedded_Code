#ifndef IMU_H
#define IMU_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>

/**
 * @brief IMU 解算滤波器通用 API
 *
 * 每种滤波器实现此接口后封装为 imu_estimator，
 * IMU 驱动通过 imu_get_api 按 estimator 字符串查找并调用。
 * 替换滤波器只需改 DT 中 estimator 和 filter-dev，无需改 IMU 代码。
 */
struct imu_filter_api {
    /** @brief 初始化滤波器状态 */
    void (*init)     (const struct device *dev);
    /** @brief 预测（陀螺仪积分 + 协方差传播） */
    void (*predict)  (const struct device *dev, const float gyro[3], float dt, float angle[3]);
    /** @brief 修正（加速度计重力观测更新） */
    void (*correct)  (const struct device *dev, const float accel[3]);
    /** @brief 读取姿态角 (rad), [roll, pitch, yaw] */
    void (*get_angle)(const struct device *dev, float angle[3]);
};

/** @brief 解算实现条目：名称 + API */
typedef struct {
    const char *name;
    const struct imu_filter_api api;
} imu_estimator;

/** @brief IMU 配置（ROM，由 DT 填充） */
typedef struct {
    const struct device *accel_dev;  /* 加速度计设备 */
    const struct device *gyro_dev;   /* 陀螺仪设备 */
    const struct device *heat_dev;   /* PWM 加热设备 */
    const struct device *filter_dev; /* 解算滤波器设备 */
    const struct device *pid_dev;     /* 温度 PID 设备 */
    const char *estimator;           /* 解算方法，如 "ekf" */
} imu_config;

/**
 * @brief IMU 机体轴：数组下标的名字
 *
 * imu_data 里 accel / gyro / angle 的下标都用它，不要再写裸的 0/1/2。
 *
 * 三个名字就是绕这三根轴转，和云台 yaw / pitch 速度内环的说法一致 ——
 * 所以调用处能直接读成「读 pitch 轴的角速度」，而不是「读 1 号元素」。
 */
typedef enum {
    IMU_AXIS_ROLL  = 0,   /**< x 轴：横滚 */
    IMU_AXIS_PITCH = 1,   /**< y 轴：俯仰 */
    IMU_AXIS_YAW   = 2,   /**< z 轴：偏航 */
} imu_axis_t;

/**
 * @brief IMU 运行时数据
 *
 * 锁的范围【只覆盖 angle】。
 *
 * angle —— 三个 float 必须来自【同一拍】，否则读方会拼出「新 roll 配旧 pitch」。
 *   所以有唯一的一对读写方（imu_update_angle ↔ imu_get_angle），走这把锁。
 *
 * gyro —— 【跨线程，但不走锁】。理由【不再是】"只有一个执行流"：云台控制的
 *   yaw / pitch 速度内环也会读它（imu_get_gyro_axis）。真正的理由是：
 *   【读方每次只要一个 float】—— 单次对齐的 32 位访问在 Cortex-M 上是原子的，
 *   一次 load 中间插不进另一个执行流，所以它天然不需要保护。
 *   ⚠️ 这个结论的前提是"一次只要一个轴"。哪天有人要求"x/y/z 必须是同一拍"
 *      （比如拿三轴角速度去积分姿态），那是【接口】该改，不是这把锁该加东西。
 *
 * accel / temp —— 仍然只有估计线程读写，没有第二个执行流。
 *
 * ⚠️ 别顺手把 accel / gyro 塞进这把锁 —— 那不会更安全，只会让临界区变宽，
 *    而「临界区宽度 = 别人最坏阻塞时间」，白送出去的等待是要记账的。
 */
typedef struct {
    float accel[3];         /* 加速度 (m/s²)，下标见 imu_axis_t */
    float gyro[3];          /* 角速度   (rad/s)，下标见 imu_axis_t。跨线程读见 imu_get_gyro_axis */
    float temp;             /* 温度     (°C) */
    float angle[3];         /* 姿态角   (rad)，下标见 imu_axis_t */
    struct k_mutex mutex;   /* 保护 angle：唯一写方 imu_update_angle，唯一读方 imu_get_angle */
} imu_data;

/* ─── 读取：按传感器分开 ───
 * 双速率下两条分支各读各的，不再一次读两个：
 *   gyro  分支读陀螺仪，accel 分支读加速度计。
 * 各读各的，两次 SPI 事务不会都按较高的那个频率发生。
 */

/**
 * @brief 读取加速度计（含片上温度）并填入 data
 * @param dev IMU 设备指针
 */
void imu_fetch_accel(const struct device *dev);

/**
 * @brief 读取陀螺仪并填入 data
 * @param dev IMU 设备指针
 */
void imu_fetch_gyro(const struct device *dev);

/* ─── 解算：按"谁进入方程"分开 ───
 * predict 吃陀螺仪 → 陀螺仪数据就绪时调，dt 是【两次 predict 之间】的实测间隔
 * correct 吃加速度计 → 加速度计数据就绪时调，不需要 dt
 *
 * ⚠️ 这两个都【不负责更新 data->angle】。角度由 imu_update_angle 单独给出 ——
 *    原因见 imu.c 里的说明（get_angle 有状态，调用次数是语义的一部分）。
 */

/**
 * @brief 预测步：陀螺仪积分 + 协方差传播
 * @param dev IMU 设备指针
 * @param dt  距上次调用 imu_predict 的时间（秒），必须实测
 */
void imu_predict(const struct device *dev, float dt);

/**
 * @brief 校正步：加速度计重力观测更新
 * @param dev IMU 设备指针
 */
void imu_correct(const struct device *dev);

/**
 * @brief 算出姿态角并写入 data->angle（唯一写方）
 *
 * 锁只盖住最后的三行赋值。解算本身（三次 atan2 + 一次 sqrt，几百个周期）留在【锁外】：
 * 临界区宽度就是别人最坏的阻塞时间，把它关进去等于让估计线程白等两个数量级。
 * 详见 imu.c 里「锁内三行 vs 锁外几百周期」的推演。
 *
 * ⚠️ 有状态（内部维护 Yaw 跨圈计数），一轮只能调一次。
 *
 * @param dev IMU 设备指针
 */
void imu_update_angle(const struct device *dev);

/**
 * @brief 取一份姿态角【快照】到调用者的 buffer（唯一读方）
 *
 * 驱动不交出内部指针，调用者拿到的是自己栈上的副本 —— 所以调用方既不需要
 * 知道这把锁存在，也不可能绕过它。（对比：直接读 data->angle 就是绕过。）
 *
 * ⚠️ 拿到的是【快照】不是活数据：三个 float 是同一时刻的，之后不会再变。
 *    这是相对「读三次全局变量」的全部价值所在。
 *
 * @param dev       IMU 设备指针
 * @param out_angle 输出姿态角 (rad), [roll, pitch, yaw]
 */
void imu_get_angle(const struct device *dev, float out_angle[3]);

/**
 * @brief 读【单个轴】的角速度（rad/s，原始量，未经姿态解算）
 *
 * 和云台的 yaw / pitch 速度内环配套：每个环各读各的轴，互不干涉。
 *
 * ★ 为什么是"一个轴一次调用"，而不是 imu_get_angle 那样的 [3] 数组版：
 *
 *   跨线程读 gyro[3] 是【三次】访问。在三次 load 之间 IMU 线程可以跑完一轮，
 *   读方会拿到「新 x 配旧 y」的缝合怪。
 *   而只取一个 float 是【一次】访问 —— 单次对齐的 32 位访问是原子的，
 *   两个执行流之间不需要任何同步。
 *
 *   这不是"省了一把锁"，是让这把锁【根本不必要】。判据要往前挪一步：
 *   先问「这几个量必须来自同一时刻吗」——不需要，就分别读。
 *   原子性的需求来自"这些值必须一起正确"，不是来自"它们恰好是同一个数组"。
 *
 * ⚠️ 别为了和 imu_get_angle 对称而改成数组版 —— 那正好把原子性丢掉。
 *
 * @param dev  IMU 设备指针
 * @param axis 轴，用 imu_axis_t —— 写 IMU_AXIS_YAW，别写 2
 * @return 该轴角速度 (rad/s)；axis 越界返回 0.0f
 */
float imu_get_gyro_axis(const struct device *dev, imu_axis_t axis);

/**
 * @brief PID 温度控制
 *
 * 低频调用（~10 Hz），通过 pid_dev 计算 PWM 脉宽并写入 heat_dev。
 *
 * @param dev         IMU 设备指针
 * @param target_temp 目标温度 (°C)
 * @param dt          距上次调用时间（秒）
 */
void imu_heat_control(const struct device *dev, float target_temp, float dt);

#endif // IMU_H