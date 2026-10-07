巡天御风通用电控框架

> **v1.0 · 2026-10-07 · 目标板 达妙 DM-MC02（STM32H723）· Zephyr RTOS**

## 先读这个

📄 **[SUMMARY.md](SUMMARY.md)** —— 交付总结。做了什么、做到什么程度、
**上真车前必须替换哪些占位值**、已知风险，全在里面。

⚠️ 一句话：**架构定型、编译通过、一行都没上过真车。**
除 IMU 姿态外，所有功能都是"代码写完 + 编译通过"，不含"实测有效"。

## 快速构建

```bash
cd /home/wyx/RoboMaster/skywalker

# 最终固件
.venv/bin/west build -b dm_mc02 skywalker_code/application -d build/app

# 设备树自检（本项目唯一能逼出 DT 宏展开的构建）
.venv/bin/west build -b dm_mc02 skywalker_code/samples/dt_smoke_test -d build/dt
```

## 目录

```
application/     最终固件（三条线程：imu / send / control）
boards/          dm_mc02 板级定义
drivers/         chassis · gimbal · imu · motors(dji/dm) · pid · kalman_filter
include/         全部对外头文件
lib/             dr16 · matrix · vofa
dts/bindings/    以上各驱动的设备树 binding
samples/         分模块台架与自检
```

## 设计要点

- **上层不出现电机型号名。** 型号由设备树的 `compatible` 决定，
  换电机只改 overlay，不改一行 C。
- **底盘三段拆分**：`chassis_update`（算）→ `chassis_power_limit`（限）→
  发送线程 pack + send（发）。限幅是最后一道闸，绕不过去。
- **线程划分依据**：节拍源不同 **或** 阻塞行为不同。
