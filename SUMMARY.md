# skywalker 通用电控框架 · v1.0 交付总结

> 2026-10-07 · 目标板 达妙 DM-MC02（STM32H723）· Zephyr RTOS
> 本文回答三个问题：**做了什么 / 做到什么程度 / 还差什么才能上真车**。

---

## 0. 一句话现状

**架构定型、全链路编译通过、一行都没上过真车。**

下面每一处"做完了"都只表示 **代码写完 + 编译通过**，不含 **"实测有效"**。
全文只有一处功能是真在硬件上跑过的（IMU 姿态，见 §1.5），其余都是"待验证"。

程度记号：

| 记号 | 含义 |
|---|---|
| ✅ | **已在硬件上跑过**，现象确认 |
| 🟡 | **代码完成、编译通过**，从未上过硬件 |
| 🟠 | **部分完成**，有已知的缺口，见备注 |
| ⬜ | **完全没做** |

---

## 1. 做了什么

### 1.1 电机驱动层 🟠

**统一抽象** `include/drivers/motors/motor.h`：上层只认 `motor_set` / `motor_get` /
`motor_can_pack`，**不知道型号**。型号由设备树决定，换电机 = 改 overlay，不改一行 C。

- **DJI 协议** `drivers/motors/motor_dji/`（323 行）🟡
  支持 M2006 / M3508 / M6020 / GM6020。ID 1~4 走 0x200、5~8 走 0x1ff。
  **两种角度语义**：GM6020 是绝对编码器（`fmodf` 折到 (−π, π] 再减零偏）；
  其余型号是增量式（`angle_counts × rad_per_count`，连续不折回）。
  ★ 因为上层只用 `cos`/`sin`，折回与圈数都不影响结果 —— 接口对型号免疫。

- **达妙协议** `drivers/motors/motor_dm/`（272 行）🟠
  含 MIT / 位置速度 / 速度 三套帧，以及使能握手自愈。
  **缺口**：`disable_frame` / `set_zero_frame` / `clear_error_frame`
  三个命令帧**定义但从未被引用** —— 也就是说「失能」「置零」「清错误」三条
  命令目前发不出去。见 §3。

- 两套协议共用同一个 binding 基类（`can-dev` / `id` / `tx-id` / `rx-id`），
  所以换型号只改 `compatible` 和 `type` 两个属性。

### 1.2 底盘 `drivers/chassis/`（993 行 + 472 行头文件）🟡

三种构型**都已实现**：`omni=0` / `mecanum=1` / `steer=2`（枚举顺序即设备树契约）。

- 正运动学逆解 + 四轮独立速度环 PID
- **USTC 式三段拆分**：`chassis_update`（算）→ `chassis_power_limit`（限）
  → 由 1 ms 发送线程 pack + send（发）
- **限幅是最后一道闸**：`chassis_power_limit` 是驱动轮的**唯一** `motor_set`
  写方，任何以后加进来的力矩生产者都绕不过它（这一点有编译期/运行期可见性）
- **幂等**：限幅不就地改 `wheel_torque[]`，重复调用结果逐位相同
- **失效朝"不动"倒**：忘了调 `chassis_power_limit` 的后果是轮子不下发（车不动），
  不是失控 —— 而且是上电就看得见的显眼故障
- 舵轮转向电机、拨盘**不进功率预算**（各自独立闭环，维持现状）
- 功率上限入口 `chassis_set_power_limit()` 已补上（此前 `power_limit` 字段
  文档写着"由裁判系统那一层写"但**没有任何写入口**，是个现成窟窿）
- 编译期断言 `CHASSIS_DT_ASSERTS`：四个数组下标对不齐时**编译不过**

### 1.3 云台 `drivers/gimbal/`（488 行 + 374 行头文件）🟡

- 两种模式：`MANUAL`（yaw 角度环旁路，摇杆/鼠标直接给角速度）/
  `AUTO`（自瞄，角度环串进速度环，上位机给绝对角度）
- yaw 角度环 + 速度环、pitch 角度环 + 速度环，**四份独立积分器**
- **摩擦轮 0~4 个**，个数从设备树属性长度算出，不手写
- 拨盘：单发 / 连发 / 停火
- pitch 行程由设备树 `pitch-min-deg` / `pitch-max-deg` 夹住
- ★ **AUTO 模式代码在、链路没接** —— `gimbal_set_yaw_angle()` 就是给它准备的
  入口，但上位机不存在，所以 `main.c` 恒 `MANUAL`。

### 1.4 拨盘 / 发射 🟠

单发与连发共用 `chassis_trigger_update`，边沿检测由 `trigger_fire_last` 独占写入。
**缺口**：连发模式的**累加与限幅策略未定**（该不该夹、夹在哪），见 §3。

### 1.5 IMU `drivers/imu/`（580 行 + 183 行头文件）✅ / 🟠

- **滤波器可插拔**：IMU 只认 `imu_estimator` 接口，靠 DT 字符串 `estimator="ekf"`
  查找。换滤波器只改 DT，不改 IMU 代码。
- 已接 EKF（`drivers/kalman_filter/`，222 行，4 状态 / 3 观测，底层用 CMSIS-DSP）
- 加热片温控（PWM + 独立 PID，目标 50 °C）
- ✅ **唯一实测过的功能**：`samples/imu_test` 里 VOFA 的立方体可以跟随板子运动
  —— 证明**整条链路是通的**（SPI 读数 → 触发 → 解算 → 串口上报）
- 🟠 **但 EKF 的解算效果没有进一步验证**，PID 也没整定过。
  "立方体跟着动"只证明**没接反、没卡死**，不证明**角度准**。

### 1.6 库 `lib/` 🟡

| 库 | 行数 | 说明 |
|---|---|---|
| `matrix` | 257 | 矩阵运算，`CONFIG_SKYWALKER_LIB_MATRIX` 的真正作用是 select CMSIS-DSP |
| `vofa` | 159 | JustFloat 协议，上位机波形 |
| `dr16` | 135 | DBUS 遥控解析 |

### 1.7 DR16 遥控 `lib/dr16/` 🟡

- 18 字节 DBUS 帧，UART5 @ 100000 baud，DMA 收进 `__nocache` 段
  （不落 nocache 段的话 `uart_stm32_async_rx_enable()` 返回 `-EFAULT` 并且**静默到底**）
- 通道中值 1024 已在驱动里减掉，摇杆回中即 0
- `switch_L` / `switch_R` 两位（1=上 2=中 3=下），`key` 16 位各占一键
- ★ **`frame_count` 是新加的**：遥控器 ~14 ms 一帧而控制环 1 kHz，同一帧会被
  连读十几次。电平量（摇杆、拨杆、按键）重复读无害，但**鼠标 Δ 是每帧增量** ——
  `pitch += mouse_y × k` 每毫秒做一次会把同一帧的 Δ 累加 14 遍，
  云台走得比手快 14 倍。这个计数同时兼任**失控保护判据**。
  计数在**锁内**自增（锁外读改写会被碰撞的中断吞掉，计数卡死 → 误判掉线）。

### 1.8 最终固件 `application/`（772 行）🟡

**三条线程**（另有 2 条传感器驱动自建的线程，运行期共 5 条）：

| 线程 | 栈 | 优先级 | 职责 |
|---|---|---|---|
| `imu_thread` | 2176 + `K_FP_REGS` | 1 | 等 accel/gyro 触发事件 → 取数 → 预测/校正 → 温控 |
| `send_thread` | 2048 | 2 | 1 ms 定时 → 按 (CAN 口, tx_id) 分组 → pack → 一发一帧 |
| `control_thread` | 2048 | 3 | 1 ms 定时 → 遥控解析 → 机体系→车体系 → 底盘/云台 update |

★ 线程划分依据是 **「节拍源不同 **或** 阻塞行为不同」**，
不是单纯的"触发源不同" —— `send_thread` 和 `control_thread` 共用同一个 1 ms
节拍，它独立出去的理由是**阻塞行为不同**（发送会被总线拖住，不能拖住控制环）。

其它要点：

- **★ 全文不出现任何电机型号名**，型号完全由 `application/boards/dm_mc02.overlay` 决定
- 电机列表由 DT 里的 chassis/gimbal 配置**自动收集**（去重），
  CAN 分组也自动做 —— 加电机只改 overlay
- 失控保护：遥控器 100 ms 无新帧 → 失能
- IMU 失效保护：心跳计数 20 拍不动 → **只停云台，不停底盘**
  （底盘不读姿态；云台一死会一路顶到机械限位）
- 上电默认**失能**，右拨杆上推才使能（"不拨杆车不动"是故意的）
- 使能瞬间把 pitch 目标**对齐到当前实际角度**，避免从旧目标猛甩

**构建结果**（2026-10-07 全量重建，`rm -rf` 后重跑）：

```
FLASH:  101868 B / 1 MB    = 9.71%
RAM:     30688 B / 320 KB  = 9.37%
```

**告警**（全部是预期的，不是坏了）：

- 5 条 C 告警，**全部来自 `motor_dm.c`**：`motor_dm_api` / `motor_dm_init` /
  三个命令帧 `defined but not used`。原因：本 overlay 全是 `motor_dji`，
  `motor_dm` 那边 `DT_INST_FOREACH_STATUS_OKAY` 展开为空。
  ⚠️ **反过来看这就是好消息** —— 哪天这些告警消失了，说明 DT 真的展开到了。
- 2 条 CMake 告警 `No SOURCES given to Zephyr library`：
  `..__skywalker_code__lib__matrix`（`matrix.c` 只在矩阵存 flash 的功能下编译，
  本车不用；该开关的真正作用是 select CMSIS-DSP）和 `drivers__counter`
  （Zephyr 内部），**无害**。

---

## 2. 没做什么

| 项 | 状态 | 说明 |
|---|---|---|
| **双板兼容** | ⬜ | 现在只支持单板（一块板跑全车）。双板场景是一块跑底盘、一块跑云台。★ 好消息：跨板的耦合量**只有一个** —— 车体系到操作手系的夹角 α，正是 `chassis_frame_yaw()` 读的那一个数。真要拆板，加一条板间链路把这个角传过去即可，底盘和云台的代码不用改。 |
| **裁判系统** | ⬜ | 全仓库零处提及，设备还不存在。所以：缓冲能量协调做不了，功率上限现在是个**写死的常量** `APP_POWER_LIMIT_W = 80.0f`。入口 `chassis_set_power_limit()` 已经留好了。 |
| **上位机 / 自瞄** | ⬜ | 接口留了（`gimbal_set_yaw_angle()` + `GIMBAL_MODE_AUTO`），链路没接。接上之后**线程数不会变成 5 条 app 线程**——自瞄数据进来仍是"值可被覆盖"，走共享状态即可，不需要新增线程。 |
| **连发模式的累加 / 限幅策略** | ⬜ | 该不该夹、夹在哪，没定。 |
| **`samples/chassis_test` 台架** | ⬜ | 此前明确"先不急"。 |
| **实际 PID 整定** | ⬜ | overlay 里所有增益都是 `k-p=8, k-i=0, k-d=0` 这一套**编出来的数字**，一个都没整定过。 |
| **实车几何 / CAN ID** | ⬜ | overlay 里全部是占位值，见 §3。 |

---

## 3. ⚠️ 上真车前必须替换的占位值

**这是全文最需要看的一节。** §1 里标 🟡 的东西全部建立在这些数字上。

### 3.1 `application/boards/dm_mc02.overlay` —— 全是编的

| 属性 | 现状 | 要做什么 |
|---|---|---|
| 全部电机的 `id` / `tx-id` / `rx-id` | 编的 | 按实车接线改。⚠️ **DJI 的对应关系是 id 1~4 ↔ `0x200`、5~8 ↔ `0x1ff`，填错电机不转而且不报错。** |
| 全部电机的 `type` / `gear-ratio` | 全写成 M3508 / 19.2032 | 云台 yaw/pitch 真车多半是 GM6020，**记得 `id` 和 `tx-id` 一起改** |
| `wheel-x` / `wheel-y` | ±0.2（编的） | 量实车。⚠️ 这是**轮子在车体系里的位置**，不是轮距轮轴。量错一点，小陀螺就会让车**走圈而不是原地转**。下标 0=左前，逆时针。 |
| `wheel-radius` | 0.06 | 量 |
| `pitch-min-deg` / `pitch-max-deg` | −20 / 30 | 量机械行程。**必须 min < max**，否则 `gimbal_init` 返回 `-EINVAL` |
| 所有 PID 的 `k-p` / `k-i` / `k-d` / `i-max` / `out-max` | 全部编的 | 台架上一项一项调 |
| `trigger-angle-deg` / `trigger-speed-deg-s` | 45 / 360 | 按拨盘槽数改（`360 / 槽数`） |
| `friction-speed-deg-s` | 6000 | 按实车定 |
| `timers3` 的 `prescaler` | 99 | 按加热片功率需求核 |

### 3.2 `application/src/main.c` —— 顶部的可调常量

| 常量 | 现状 | 说明 |
|---|---|---|
| `APP_MAX_LINEAR` / `APP_MAX_ANGULAR` | 3.0 / 6.0 | 手感 |
| `APP_POWER_LIMIT_W` | 80.0 | 接裁判系统后应改为**每拍从裁判系统读**，而不是常量 |
| `APP_MOUSE_YAW_GAIN` / `APP_MOUSE_PITCH_GAIN` | 0.03 / 0.0004 | 手感，没调过 |
| `IMU_TARGET_TEMP` | 50.0 | 按实际需求 |
| **`APP_YAW_SIGN`** | `(+1.0f)` | ★★ **不改小陀螺一定是错的** —— 见 3.3 |
| **`APP_YAW_ZERO_RAD`** | `(0.0f)` | ★★ 同上 |

### 3.3 ⚠️ 六个方向符号：**第一版上车必须低速试**

`dr16.c` 只对 `Right_X` 做了取反，其余三轴原样透传。所以下面每一个符号都是
**按"标准 DBUS"推的、没在真车上验过**。反了就翻这里的符号，**别的地方都不用动**：

| # | 位置 | 表达式 | 备注 |
|---|---|---|---|
| 1 | `main.c` ② | `vx = stick_norm(Left_Y)` | 上推为正 → 向前 |
| 2 | `main.c` ② | `vy = -stick_norm(Left_X)` | 车体系 y 是**车左为正**，故取负 |
| 3 | `main.c` ② | `wz = stick_norm(Right_X)` | ⚠️ `Right_X` **已在 `dr16.c` 里取过反了**，这里别重复 |
| 4 | `main.c` ⑤ | `gimbal_set_yaw_speed(-mouse_x × k)` | 世界系 yaw 逆时针为正，鼠标右移 = yaw 减小 |
| 5 | `main.c` ⑤ | `pitch_target_rad -= mouse_y × k` | mouse_y 正 = 向下，pitch 正 = 向上 |
| 6 | `chassis_frame_yaw()` | `APP_YAW_SIGN` / `APP_YAW_ZERO_RAD` | ★ **车体系→操作手系**的修正量 |

**关于 #6 怎么量**：这两个数是**小陀螺能不能原地转**的关键。α 是从车体系的 x̂
转到云台（操作手）朝向 x̂ 的角。先把车放平、云台朝正前方，读一次
`motor_get(yaw_motor).angle` 记作基准 → 填进 `APP_YAW_ZERO_RAD`；
再把云台转到**车左边 90°**，看读数是涨还是落 → 据此定 `APP_YAW_SIGN`。
两个值不对，操作手推前进车会斜着走。

### 3.4 换电机型号时

只改 overlay 的 `compatible` 和 `type`。**上层一行 C 都不用动** —— 这是整套框架
最主要的设计目标，也是 §1.8 "全文不出现型号名"那条规则的意义。

---

## 4. 已知未验证 / 已知风险

按危险程度排序：

1. **🔴 一切标 🟡 的东西都没上过硬件。** 编译通过不代表能跑。这是本文最重要的
   一句话 —— 5 个 sample 里只有 `imu_test` 是真跑过的。

2. **🔴 `APP_YAW_SIGN` / `APP_YAW_ZERO_RAD` 是占位值。** 填之前，
   **小陀螺的机体系→车体系变换是不生效的**（等于 α 恒为 0，车永远按车头方向走）。
   这不会报错、不会编译失败，只会"用起来不对"。

3. **🟠 `motor_dm` 的三条命令帧发不出去**（`disable_frame` / `set_zero_frame` /
   `clear_error_frame` 无人引用）。影响：达妙电机的**失能、置零、清错误**目前
   做不到。上达妙电机前必须补上 `motor_api` 对应的入口。

4. **🟠 云台"零个摩擦轮"的分支从未被任何一次构建编译到** —— overlay 里写了 2 个，
   所以 `friction_num == 0` 那条路是死代码，没验过，可能藏语法/逻辑问题。

5. **🟠 `chassis_config.imu` 是个半死字段**：只在 `chassis_init` 里做了
   `device_is_ready` 检查，**从未被读过数据**。底盘确实不读姿态，所以无害，
   但它在 DT 里容易被误读成"底盘需要 IMU"。

6. **🟠 连发模式策略未定**（累加？限幅？），单发已考虑基准作废问题。

7. **🟡 运行期共 5 条线程，其中 2 条是 BMI08x 驱动自建的**
   （`OWN_THREAD` 模式，在 `init` 里**无条件** `k_thread_create` + `K_NO_WAIT`）。
   没调 `sensor_trigger_set` 时它们会永远阻塞在信号量上 —— 白占栈但不烧 CPU。
   本项目调了，所以是正常工作的。

8. **🟡 `can_bus_send` 这一层写死了 `struct can_frame` + `can_send`。**
   做双板时若要改成 FlexCAN/FDCAN 或加板间协议，这里需要泛化。

9. **🟡 `CONFIG_FPU_SHARING` 必须开**（已开）。不开的话中断里碰浮点
   （CAN 反馈换算）会让 FPU 寄存器不被保存/恢复，任务切换后浮点结果**随机错**。

---

## 5. 怎么构建

```bash
cd /home/wyx/RoboMaster/skywalker

# 最终固件
.venv/bin/west build -b dm_mc02 skywalker_code/application -d build/app

# 设备树自检（★ 本项目唯一能逼出 DT 宏展开的构建）
.venv/bin/west build -b dm_mc02 skywalker_code/samples/dt_smoke_test -d build/dt
```

⚠️ **验证时必须 `rm -rf` 掉旧 build 再全量重建。** 增量构建会拿旧对象骗人，
而且 `*_DT_CONFIG_INST_INIT` 这类宏**只在 `DT_INST_FOREACH_STATUS_OKAY` 匹配到
≥1 个节点时才展开** —— "编译过了 + 有 `-Wunused-*` 告警" 看着干净，
其实可能是**假信号**。看到 `motor_dm` 那 5 条告警要当**好消息**看（见 §1.8）。

烧录：

```bash
.venv/bin/west flash -d build/app
```

---

## 6. sample 一览

| sample | 用途 | 程度 |
|---|---|---|
| `hello` | 板上电冒烟 | ✅ |
| `imu_test` | IMU + EKF + VOFA | ✅ 链路通 / 🟠 精度未验 |
| `dt_smoke_test` | DT 宏展开自检 | 🟡（全量构建 0 告警） |
| `motor_dji_test` | DJI 电机台架 | 🟡 代码就绪，未上台架 |
| `motor_dm_test` | 达妙电机台架 | 🟡 代码就绪，未上台架 |
| `application` | **最终固件** | 🟡 编译通过，未上真车 |

---

## 7. 下一步（按优先级）

1. **`motor_dji_test` 上台架** —— 验反馈帧解得对不对、速度环能不能跟上。
   （按既定交付顺序：motor_dji → 台架 → motor_dm → application）
2. 量实车，替换 §3.1 全部占位值
3. 低速试 §3.3 的六个方向符号
4. 整定 PID
5. 补 `motor_dm` 的三条命令帧入口（§4 第 3 条）
6. 接裁判系统 → 把 `APP_POWER_LIMIT_W` 从常量改成每拍读取
7. 接上位机自瞄 → `GIMBAL_MODE_AUTO` + `gimbal_set_yaw_angle()`
8. 双板拆分（只有一个跨板量 α，见 §2）

---

**v1.0 的准确定位：一套架构跑通、编译干净、等待装车验证的框架。**
**它是"能编译能烧进去"的，不是"能打比赛"的。**
