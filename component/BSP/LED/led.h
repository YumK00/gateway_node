#ifndef LED_H
#define LED_H
#include <stdint.h>
#include "driver/gpio.h"
/* ============================================================
 * 【板载 WS2812 彩灯】—— 单总线可寻址 RGB 灯
 *
 * 数据流：
 *   LED_SetRGB(r,g,b) → 组 24bit GRB 数据 → RMT 发波形 → 灯珠锁存变色
 *
 * 为什么用 RMT：WS2812 靠"高电平持续多久"区分 0/1 码，
 *   0 码 ≈ 高0.4us+低0.8us，1 码 ≈ 高0.8us+低0.4us，
 *   时序要求亚微秒级精度。用普通 GPIO 翻转做不到，
 *   RMT 是硬件波形发生器，能精确输出这种时序。
 * ============================================================ */

/* 板载 WS2812 彩灯接在 GPIO48 */
#define LED_GPIO_PIN GPIO_NUM_48

/* 初始化：使用前必须先调用一次 */
void LED_Init(void);

/* 设置颜色：r/g/b 取值 0~255（建议 0~50，太亮刺眼） */
void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b);

/* 熄灭 */
void LED_Off(void);

/* 启动彩灯呼吸任务：内部 xTaskCreate，调用一次即可 */
void LedTask_Start(void);

#endif /* LED_H */
