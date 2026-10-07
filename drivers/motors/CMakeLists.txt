# SPDX-License-Identifier: Apache-2.0

# 电机基类（motor.h）是纯头文件，无独立编译单元；
# 本目录只负责扇出到具体电机子驱动（对齐 Mambo 的做法）。
add_subdirectory_ifdef(CONFIG_SKYWALKER_DRIVER_MOTOR_DJI motor_dji)
add_subdirectory_ifdef(CONFIG_SKYWALKER_DRIVER_MOTOR_DM motor_dm)
