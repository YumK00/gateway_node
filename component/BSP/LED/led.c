#include "led.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
/* ============================================================
 * WS2812 时序参数
 *
 * RMT 分辨率 10MHz → 1 个 tick = 0.1us
 * 每 bit 展开成一段波形（高电平 + 低电平）：
 *   0 码：高 0.4us + 低 0.8us  → 4 + 8 个 tick
 *   1 码：高 0.8us + 低 0.4us  → 8 + 4 个 tick
 * 每颗灯 24bit(GRB)，发完要拉低 50us 以上，灯珠才会锁存新颜色
 * ============================================================ */
#define WS2812_RES_HZ   10000000

#define T0H     4        /* 0 码高电平 */
#define T0L     8        /* 0 码低电平 */
#define T1H     8        /* 1 码高电平 */
#define T1L     4        /* 1 码低电平 */
#define T_RESET 500      /* 复位低电平 50us */

/* RMT 发送通道和编码器：全局变量，LED_SetRGB 会用到 */
static rmt_channel_handle_t s_tx_chan = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;

/**
 * @brief  WS2812 彩灯初始化：建 RMT 发送通道
 * @note   调用一次即可，之后用 LED_SetRGB() 改颜色
 */
void LED_Init(void)
{
    /* ---- 建 RMT 发送通道 ---- */
    rmt_tx_channel_config_t tx_cfg = {0};
    tx_cfg.gpio_num          = LED_GPIO_PIN;
    tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz     = WS2812_RES_HZ;
    tx_cfg.mem_block_symbols = 64;
    tx_cfg.trans_queue_depth = 4;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_tx_chan));

    /* ---- 用"拷贝编码器"：波形码点我们自己算好，它原样发出去 ----
     * 这个配置结构体是空的（没有成员），所以用 {} 而不是 {0} */
    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &s_copy_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_tx_chan));

    LED_Off();   /* 上电先熄灭 */
}

/**
 * @brief  设置一次彩灯颜色
 * @param  r/g/b：0~255，建议 0~50，太亮刺眼
 * @note   WS2812 数据顺序是 GRB、高位在前；每个 bit 展开成一段 RMT 波形码点
 */
void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    /* 拼成 24bit：G 在高位、R 在中位、B 在低位 */
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;

    /* 24 个数据码点 + 1 个复位码点 */
    rmt_symbol_word_t syms[25];
    for (int i = 0; i < 24; i++)
    {
        int bit = (grb >> (23 - i)) & 1;   /* 高位在前，逐位取 */
        if (bit)
        {
            syms[i].level0 = 1; syms[i].duration0 = T1H;
            syms[i].level1 = 0; syms[i].duration1 = T1L;
        }
        else
        {
            syms[i].level0 = 1; syms[i].duration0 = T0H;
            syms[i].level1 = 0; syms[i].duration1 = T0L;
        }
    }
    syms[24].level0 = 0; syms[24].duration0 = T_RESET;
    syms[24].level1 = 0; syms[24].duration1 = 0;

    /* 发出去，等发完 */
    rmt_transmit_config_t tx_cfg = {0};
    tx_cfg.loop_count = 0;
    ESP_ERROR_CHECK(rmt_transmit(s_tx_chan, s_copy_encoder, syms, sizeof(syms), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_chan, 1000));
}

/**
 * @brief  熄灭彩灯
 */
void LED_Off(void)
{
    LED_SetRGB(0, 0, 0);
}
