#include "lib/dr16/dr16.h"

#include <zephyr/linker/section_tags.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dr16, CONFIG_LOG_DEFAULT_LEVEL);

#define DR16_RX_TIMEOUT_US 500

/* 每一路通道是 11 bit 无符号量（0~2047），物理中位不是 0 而是 1024。
 * 减掉它，摇杆回中才是 0；不减的话回中报 1024，上层每条控制律都得自己补这一项。 */
#define DR16_CH_VALUE_OFFSET 1024

static DR16_Data dr16_data;

static struct k_spinlock dr16_lock;

/* DMA 直接写物理内存、不经过 D-cache，所以接收缓冲区必须落在 nocache 段：
 * 否则 uart_stm32_async_rx_enable() 里的 stm32_buf_in_nocache() 返回 -EFAULT，
 * 而那个失败会一路静默到底。 */
static uint8_t rx_buf[36] __nocache;

/* DBUS 帧布局（18 字节，小端，无帧头、无校验）：
 *   [0..5]   4 路 11 bit 通道 + S1/S2 各 2 bit，正好凑满 48 bit
 *   [6..7]   mouse_x        int16
 *   [8..9]   mouse_y        int16
 *   [10..11] mouse_z        int16
 *   [12]     mouse_press_l
 *   [13]     mouse_press_r
 *   [14..15] key            uint16，16 个按键各占一位
 *   [16..17] 左上拨轮        int16
 * 没有帧头，所以"这帧从哪开始"没法自校验 —— 对齐只能靠回调里的 len == 18。 */
static void dr16_data_process(const uint8_t *frame){
    k_spinlock_key_t key;
    DR16_Data tmp;

    tmp.Right_X = (frame[0] | (frame[1] << 8)) & 0x07FF;
    tmp.Right_Y = ((frame[1] >> 3) | (frame[2] << 5)) & 0x07FF;
    /* ch2 是左摇杆竖直、ch3 是左摇杆水平，别按结构体里的声明顺序想当然地对号入座 */
    tmp.Left_Y  = ((frame[2] >> 6) | (frame[3] << 2) | (frame[4] << 10)) & 0x07FF;
    tmp.Left_X  = ((frame[4] >> 1) | (frame[5] << 7)) & 0x07FF;

    /* 两个拨杆各 2 bit，一起挤在 frame[5] 的高半字节里 */
    tmp.switch_R = (frame[5] >> 4) & 0x0003;
    tmp.switch_L = ((frame[5] >> 4) & 0x000C) >> 2;

    /* 鼠标三轴是有符号量，必须显式转 int16_t：右边的表达式是 int，
     * 负值在 int 里是 0xFFFFxxxx，直接截断到 int16_t 才还原出负号。 */
    tmp.mouse_x = (int16_t)(frame[6]  | (frame[7]  << 8));
    tmp.mouse_y = (int16_t)(frame[8]  | (frame[9]  << 8));
    tmp.mouse_z = (int16_t)(frame[10] | (frame[11] << 8));

    tmp.mouse_press_l = frame[12];
    tmp.mouse_press_r = frame[13];

    tmp.key = (uint16_t)(frame[14] | (frame[15] << 8));

    tmp.Left_Front_Wheel = (int16_t)(frame[16] | (frame[17] << 8));

    /* 中值是 1024 不是 0，见 DR16_CH_VALUE_OFFSET 的注释 */
    tmp.Right_X -= DR16_CH_VALUE_OFFSET;
    tmp.Right_Y -= DR16_CH_VALUE_OFFSET;
    tmp.Left_X  -= DR16_CH_VALUE_OFFSET;
    tmp.Left_Y  -= DR16_CH_VALUE_OFFSET;
    tmp.Left_Front_Wheel -= DR16_CH_VALUE_OFFSET;

    /* 右摇杆横轴往右推时原始值递减，取反才符合"右为正"的直觉。
     * 其余三轴的方向按上层需求再定，暂不取反。 */
    tmp.Right_X *= -1;

    key = k_spin_lock(&dr16_lock);
    /* 计数在【锁内】自增：放锁外先读 dr16_data.frame_count 的话，两次中断
     * 撞在一起会读到同一个值、写回同一个值，计数就卡住不动了 ——
     * 而卡住的计数在上层看来就是"遥控器掉线"，一撞车就误触发一次失控保护。 */
    tmp.frame_count = dr16_data.frame_count + 1;
    dr16_data = tmp;
    k_spin_unlock(&dr16_lock, key);
}

static void dr16_uart_callback(const struct device *dev, struct uart_event *evt, void *user_data){
    switch(evt->type){
        case UART_RX_RDY:{
            const uint8_t *frame = evt->data.rx.buf + evt->data.rx.offset;
            if(evt->data.rx.len != 18){
                break;
            }
            dr16_data_process(frame);
            break;
        }

        case UART_RX_DISABLED:
            int ret = uart_rx_enable(dev, rx_buf, sizeof(rx_buf), DR16_RX_TIMEOUT_US);
            if(ret < 0){
                LOG_ERR("dr16 rx re-enable failed: %d", ret);
            }
            break;

        default:
            break;
    }
}

void dr16_init(const struct device *dev){
    int ret;

    if(!device_is_ready(dev)){
        LOG_ERR("uart device not ready");
        return;
    }

    ret = uart_callback_set(dev, dr16_uart_callback, NULL);
    if(ret < 0){
        LOG_ERR("uart_callback_set failed: %d", ret);
        return;
    }

    /* 必须最后一句：这个调用一返回，中断随时可能来 */
    ret = uart_rx_enable(dev, rx_buf, sizeof(rx_buf), DR16_RX_TIMEOUT_US);
    if(ret < 0){
        LOG_ERR("uart_rx_enable failed: %d", ret);
    }
}

void dr16_get_data(DR16_Data *out){
    k_spinlock_key_t key;

    if(out == NULL){
        return;
    }

    key = k_spin_lock(&dr16_lock);
    *out = dr16_data;
    k_spin_unlock(&dr16_lock, key);
}
