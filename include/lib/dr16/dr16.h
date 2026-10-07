#ifndef DR16_H
#define DR16_H

#include "zephyr/device.h"
#include <zephyr/drivers/uart.h>

typedef struct{
    //遥控器数据
    int16_t Right_X;
    int16_t Right_Y;
    int16_t Left_X;
    int16_t Left_Y;
    int16_t Left_Front_Wheel;

    uint8_t switch_L;
    uint8_t switch_R;

    int16_t mouse_x;//鼠标x轴，负左正右
    int16_t mouse_y;//鼠标y轴，负左正右
    int16_t mouse_z;//鼠标z轴，负左正右
    uint8_t mouse_press_l;//鼠标左键，1为按下
    uint8_t mouse_press_r;//鼠标右键，1为按下

    uint16_t key;//按键 W S A D Shift Ctrl Q E R F G Z X C V B

    /* ★ 收到过多少帧。上层【必须】用它来判断"这份数据是不是新的"。
     *
     *   遥控器 ~14ms 一帧，而控制回路跑 1kHz —— 也就是说同一帧数据会被上层
     *   连续读到十几次。对于"电平"型的量（摇杆、拨杆、按键、鼠标当速度用）
     *   重复读是无害的，写多少遍都是同一个值。
     *   但【鼠标 Δ 是每帧增量】：`pitch += mouse_y * k` 这种累加如果每毫秒
     *   做一次，同一帧的 Δ 会被累加 14 遍，云台走得比手快 14 倍。
     *   有了这个计数，上层就能只在"帧真的换了"的那一拍做累加。
     *
     *   顺带也是【失控保护】的判据：计数长时间不涨 = 遥控器关了 / 接收机掉线 /
     *   线松了。此时该原地不动，而不是保持最后一帧的摇杆量继续冲。 */
    uint32_t frame_count;
} DR16_Data;

void dr16_init(const struct device *dev);
void dr16_get_data(DR16_Data *out);

#endif /* DR16_H */