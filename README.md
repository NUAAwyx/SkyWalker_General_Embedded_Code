# 巡天·御风 — 通用电控框架 (SkyWalker General Embedded Control Framework)

一个面向嵌入式电控系统（基于 Zephyr）的通用框架和板级支持包。提供通用控制任务（IMU 驱动、卡尔曼滤波、PID 控制等）、板级 DeviceTree/board 定义和示例应用，方便在自研硬件（例如 dm_mc02）上快速构建与调试飞控/电控类固件。

主要面向：嵌入式固件工程师、飞控/电控算法工程师与板级 bring-up 工程师。

## 特性
- 基于 Zephyr 项目和 west manifest（仓库包含 `west.yml`），便于与 Zephyr 生态集成。
- 板级支持（boards/…）包含 dm_mc02 的 device tree、board.yml、defconfig 与 OpenOCD 支持脚本。
- 模块化驱动目录：IMU 驱动（drivers/imu）、卡尔曼滤波、PID 控制等（drivers/*）。
- 示例应用位于 `application/`，包含 `prj.conf` 和最小入口 `main.c`。
- 使用 CMake / Kconfig / Device Tree 作为构建与配置系统，便于扩展与配置管理。

## 目录结构（注释）

```
CMakeLists.txt            # 顶层 CMake 配置
Kconfig                   # 全局 Kconfig 入口
west.yml                  # west manifest（Zephyr 工作区）
.devicetree-code.yaml     # 设备树相关工具/配置（辅助）
README.md

application/              # 应用层示例与工程
  CMakeLists.txt
  prj.conf                # Zephyr 项目配置（Kconfig 片段）
  src/
    main.c                # 应用入口（示例）

boards/                   # 板级支持（board definitions）
  damiao/
    dm_mc02/
      dm_mc02.dts         # Device Tree 源
      dm_mc02.yaml        # 元数据
      dm_mc02_defconfig   # 默认配置
      board.yml
      board.cmake
      support/
        openocd.cfg
        openocd_stlink.cfg # OpenOCD 调试脚本

drivers/                  # 驱动与控制库
  imu/
    imu.c                 # IMU 读取与处理实现（示例/参考）
    CMakeLists.txt
  kalman_filter/          # 卡尔曼滤波实现目录
  pid/                    # PID 控制器实现目录
  Kconfig                 # 驱动相关 Kconfig 条目

dts/                      # 可能的额外设备树片段
include/                  # 公共头文件（API、数据结构等）
lib/                      # 通用库（数学、工具等）
samples/                  # 示例工程或 demo
zephyr/                   # (由 west update 拉取的 Zephyr 源/工作区)
```

如何配合：主控固件通常从 application/main.c 启动，使用 drivers/ 下的模块完成传感器读取与控制算法。boards/ 下负责把硬件资源（GPIO/SPI/I2C/UART/时钟等）映射到 Zephyr 的 device tree，使驱动能在目标板上正确初始化。

## 快速上手（最短可运行路径）
先决条件（示例）
- 安装 Zephyr SDK、west 工具、CMake、ninja 和适配的交叉编译链（如 arm-none-eabi-gcc）。
- Python 环境与 Zephyr build 所需的 Python 包（参见 Zephyr 文档）。

示例命令（在一个已安装并配置好 Zephyr 的环境中）：
```bash
# 克隆仓库
git clone https://github.com/Alexei-the-rookie/SkyWalker_General_Embedded_Code.git
cd SkyWalker_General_Embedded_Code

# 初始化并更新 west（使用仓库内的 west.yml，本地 manifest）
west init -l .
west update

# 设置 Zephyr 环境（由 west update 拉取的 zephyr）
source zephyr/zephyr-env.sh

# 构建 application（示例，替换 -b 后的板名为目标板）
west build -s application -b dm_mc02 -p auto
```

刷写 / 调试（两种常见方式）
- 使用 west 的 runner（若支持 openocd）：
```bash
west flash
```
- 使用 OpenOCD + GDB（仓库提供了 boards/damiao/dm_mc02/support/openocd.cfg）：
```bash
# 启动 OpenOCD（在另一个终端）
openocd -f boards/damiao/dm_mc02/support/openocd.cfg

# 使用 arm-none-eabi-gdb 连接并下载 elf（假设构建输出默认路径）
arm-none-eabi-gdb build/zephyr/zephyr.elf -ex "target remote :3333" \
  -ex "monitor reset halt" -ex "load" -ex "monitor reset init" -ex "continue"
```

注意：具体 build/flash 命令可能依赖本地 Zephyr 版本与 runner 配置；如果 west runner 不工作，可直接用 openocd 配合 gdb。

## 配置与自定义
- prj.conf（位于 application/）控制 Zephyr Kconfig 选项，调整构建特性与驱动启用。
- 顶层 `Kconfig` 与 `drivers/Kconfig` 定义了可配置的驱动选项，扩展模块时请在相应目录添加 Kconfig 条目并在 CMakeLists.txt 中注册源文件。
- Device Tree：boards/damiao/dm_mc02/dm_mc02.dts 为板级资源定义，新增外设或更改引脚映射时请修改或添加相应的 dts 片段。

## 已知关键文件（参考）
- application/src/main.c — 应用入口（示例）。
- application/prj.conf — 项目构建配置。
- boards/damiao/dm_mc02/dm_mc02.dts — 板级设备树。
- boards/damiao/dm_mc02/support/openocd.cfg — OpenOCD 调试脚本。
- drivers/imu/imu.c — IMU 驱动实现示例。
- drivers/Kconfig — 驱动相关配置条目。
- west.yml — west manifest（Zephyr 工作区）。

## 开发与贡献
- 代码风格：遵循 Zephyr 社区的代码组织习惯（CMake + Kconfig + DTS）；请保持模块化、可配置性与硬件无关的算法实现。
- 新增驱动：在 drivers/ 下创建子目录，提供 CMakeLists.txt、Kconfig、头文件（放入 include/ 或 drivers/<name>/include）并在顶层 drivers/CMakeLists.txt 中注册。
- 板支持：在 boards/ 添加新的 board.yml、.dts、defconfig 与支持脚本（OpenOCD），并测试构建与闪存流程。
- 提交 PR 时请包含可重复的复现步骤与最小示例（最好一个能在仿真器或已有板子上跑的 sample）。

## 测试 & 调试建议
- 在新增硬件驱动前，用现有 IMU 驱动与 sample 测试总线与中断行为。
- 使用 OpenOCD 提供的配置进行在线调试（breakpoints、寄存器查看）。
- 将算法（卡尔曼、PID）拆成独立模块，并在宿主机上做单元测试（若可能），再移植到嵌入式环境以减少调试周期。

## 示例（建议查看）
- samples/ ：查看并运行仓库内的样例（若有）。
- drivers/imu/imu.c — 学习如何读取传感器、做基础滤波与发布数据。

## 许可证
仓库中未找到 LICENSE 文件（如有，请补充或在 PR 中添加）。在使用或分发前，请与仓库维护者确认许可条款。

## 常见问题
- Q：如何添加新板支持？
  A：在 boards/ 下新增目录，添加 board.yml、board.cmake、dts、defconfig 与 support 脚本；在 west/Zephyr 环境中验证 `west build -b <your_board>` 能成功构建并能被 flash。
- Q：我能否在本仓库直接修改 Zephyr 源？
  A：仓库通过 west manifest 引用了 Zephyr；一般建议通过 overlay 或模块化方式扩展，而不是直接改动上游 Zephyr 源。

## 联系与贡献者
如需合并贡献或讨论设计，请在仓库中打开 Issue 或提交 Pull Request，或直接联系仓库所有者（GitHub 用户：Alexei-the-rookie）。

---

如果你希望我把这个 README 直接提交到仓库（创建/更新 README.md），或者把 README 翻译成英文版/添加更多板级使用说明（例如更详细的 OpenOCD 使用步骤或参考的 toolchain 版本），我可以继续帮你生成对应的 PR 内容或补充文档.
