zephyr_library()

file(GLOB DR16_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
zephyr_library_sources(${DR16_SOURCES})

zephyr_library_compile_options(
  -O3
  -fno-strict-overflow
  -fno-common
  -ffunction-sections
  -fdata-sections
  -ffreestanding
  -fno-builtin)