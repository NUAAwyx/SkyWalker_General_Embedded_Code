zephyr_library()

zephyr_library_sources(motor_dji.c)

zephyr_library_compile_options(
  -O3
  -fno-strict-overflow
  -fno-common
  -ffunction-sections
  -fdata-sections
  -ffreestanding
  -fno-builtin)