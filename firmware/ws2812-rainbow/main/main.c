/*
 * ESP32-P4 + WS2812B 彩虹灯
 *
 * 硬件: GPIO52 -> 两颗 WS2812B-V6 串联 (DI -> DO -> DI)
 * 驱动: 用 RMT (TX + copy encoder) 自己产生 800kHz 单线 NRZ 时序
 *       不依赖任何联网组件, 与 ESP-IDF 版本解耦
 *
 * WS2812B 时序 @10MHz RMT 分辨率 (1 tick = 100ns)
 *   0 码: 高 300ns / 低 900ns
 *   1 码: 高 900ns / 低 300ns
 *   帧间复位: 低电平 > 280us
 * 数据顺序: G-R-B, 高位先出
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"

#define LED_GPIO           52          /* WS2812B 数据引脚 */
#define LED_COUNT          2           /* 灯珠数量 */
#define RMT_RESOLUTION_HZ  10000000    /* 10MHz -> 1 tick = 100ns */

#define TICKS_0H   3    /* 300ns 高 */
#define TICKS_0L   9    /* 900ns 低 */
#define TICKS_1H   9    /* 900ns 高 */
#define TICKS_1L   3    /* 300ns 低 */
#define TICKS_RST  3000 /* 300us 复位/锁存 */

static const char *TAG = "ws2812";

static rmt_channel_handle_t s_chan   = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static rmt_symbol_word_t   *s_symbols = NULL;
static size_t               s_symbol_cap = 0;

/* 一个 rmt_symbol_word_t 装两段电平: level0/duration0 + level1/duration1 */
static const rmt_symbol_word_t SYM_BIT_0   = {
    .level0 = 1, .duration0 = TICKS_0H,
    .level1 = 0, .duration1 = TICKS_0L,
};
static const rmt_symbol_word_t SYM_BIT_1   = {
    .level0 = 1, .duration0 = TICKS_1H,
    .level1 = 0, .duration1 = TICKS_1L,
};
static const rmt_symbol_word_t SYM_LATCH = {
    .level0 = 0, .duration0 = TICKS_RST,
    .level1 = 0, .duration1 = 0,
};

static void ws2812_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz     = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_chan));

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &s_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_chan));

    /* 每颗灯 24 bit -> 24 个 symbol, 末尾再加 1 个锁存 symbol */
    s_symbol_cap = (size_t)LED_COUNT * 24 + 1;
    s_symbols = calloc(s_symbol_cap, sizeof(rmt_symbol_word_t));
    if (s_symbols == NULL) {
        ESP_LOGE(TAG, "calloc failed");
        abort();
    }
}

/* rgb 缓冲区布局: 依次 R,G,B,R,G,B,... */
static void ws2812_flush(const uint8_t *rgb)
{
    size_t n = 0;

    for (int led = 0; led < LED_COUNT; led++) {
        const uint8_t *p = &rgb[led * 3];
        /* WS2812B 期望 G-R-B 顺序 */
        const uint8_t grb[3] = { p[1], p[0], p[2] };

        for (int c = 0; c < 3; c++) {
            for (int bit = 7; bit >= 0; bit--) {
                s_symbols[n++] = (grb[c] & (1u << bit)) ? SYM_BIT_1 : SYM_BIT_0;
            }
        }
    }
    s_symbols[n++] = SYM_LATCH;

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(s_chan, s_encoder, s_symbols,
                                 n * sizeof(rmt_symbol_word_t), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_chan, 1000));
}

/* HSV -> RGB, h:0-359 s:0-100 v:0-100 */
static void hsv2rgb(uint32_t h, uint32_t s, uint32_t v,
                    uint8_t *r, uint8_t *g, uint8_t *b)
{
    h %= 360;
    uint32_t rgb_max = (uint32_t)(v * 2.55f);
    uint32_t rgb_min = (uint32_t)(rgb_max * (100 - s) / 100.0f);
    uint32_t sector  = h / 60;
    uint32_t diff    = h % 60;
    uint32_t adj     = (rgb_max - rgb_min) * diff / 60;

    switch (sector) {
    case 0:  *r = rgb_max;        *g = rgb_min + adj;  *b = rgb_min;        break;
    case 1:  *r = rgb_max - adj;  *g = rgb_max;        *b = rgb_min;        break;
    case 2:  *r = rgb_min;        *g = rgb_max;        *b = rgb_min + adj;  break;
    case 3:  *r = rgb_min;        *g = rgb_max - adj;  *b = rgb_max;        break;
    case 4:  *r = rgb_min + adj;  *g = rgb_min;        *b = rgb_max;        break;
    default: *r = rgb_max;        *g = rgb_min;        *b = rgb_max - adj;  break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "WS2812B rainbow on GPIO%d, %d LEDs", LED_GPIO, LED_COUNT);

    ws2812_init();

    uint8_t rgb[LED_COUNT * 3];
    uint32_t hue = 0;

    while (1) {
        for (int led = 0; led < LED_COUNT; led++) {
            /* 相邻灯珠相位差 360/N，形成流动彩虹 */
            uint32_t h = (hue + (uint32_t)led * (360 / LED_COUNT)) % 360;
            hsv2rgb(h, 100, 25, &rgb[led * 3], &rgb[led * 3 + 1], &rgb[led * 3 + 2]);
        }

        ws2812_flush(rgb);

        hue = (hue + 2) % 360;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
