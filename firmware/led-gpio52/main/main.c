/*
 * GPIO52 点灯测试
 *
 * 目的: 验证 GPIO52 能否正常输出电平。
 * 不确定灯珠类型, 所以两段交替:
 *
 *   A 段: LEDC PWM 呼吸 (1kHz, 10bit, 0->100%->0)
 *         -> 普通 LED / 带限流的灯珠 会看到明暗渐变
 *
 *   B 段: RMT 发送 WS2812B 数据帧 (800kHz, GRB, 2 颗)
 *         -> 地址式灯珠 会亮固定颜色
 *
 * 哪一段有反应, 就说明灯珠是哪种, 同时证明 GPIO52 输出正常。
 */

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"

#define PIN_LED        52
#define LED_COUNT      2
#define RMT_RES_HZ     10000000     /* 10MHz -> 1 tick = 100ns */

#define PWM_FREQ_HZ    1000
#define PWM_RES_BITS   LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX   1023

static const char *TAG = "ledtest";

/* ═════════════ A 段: LEDC PWM 呼吸 ═════════════ */

static void pwm_phase(void)
{
    gpio_reset_pin(PIN_LED);

    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num   = PIN_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    ESP_LOGI(TAG, "PHASE A: LEDC PWM %dHz breathing on GPIO%d", PWM_FREQ_HZ, PIN_LED);

    for (int rep = 0; rep < 3; rep++) {
        for (int d = 0; d <= PWM_DUTY_MAX; d += 8) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, d);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(6));
        }
        for (int d = PWM_DUTY_MAX; d >= 0; d -= 8) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, d);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(6));
        }
    }

    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    gpio_reset_pin(PIN_LED);
}

/* ═════════════ B 段: RMT WS2812 ═════════════ */

#define TICKS_0H   3
#define TICKS_0L   9
#define TICKS_1H   9
#define TICKS_1L   3
#define TICKS_RST  3000

static const rmt_symbol_word_t SYM_BIT_0 = {
    .level0 = 1, .duration0 = TICKS_0H, .level1 = 0, .duration1 = TICKS_0L,
};
static const rmt_symbol_word_t SYM_BIT_1 = {
    .level0 = 1, .duration0 = TICKS_1H, .level1 = 0, .duration1 = TICKS_1L,
};
static const rmt_symbol_word_t SYM_LATCH = {
    .level0 = 0, .duration0 = TICKS_RST, .level1 = 0, .duration1 = 0,
};

static void ws2812_send(rmt_channel_handle_t chan, rmt_encoder_handle_t enc,
                        rmt_symbol_word_t *buf, const uint8_t *rgb)
{
    size_t n = 0;
    for (int led = 0; led < LED_COUNT; led++) {
        const uint8_t *p = &rgb[led * 3];
        const uint8_t grb[3] = { p[1], p[0], p[2] };   /* WS2812 期望 G-R-B */
        for (int c = 0; c < 3; c++) {
            for (int bit = 7; bit >= 0; bit--) {
                buf[n++] = (grb[c] & (1u << bit)) ? SYM_BIT_1 : SYM_BIT_0;
            }
        }
    }
    buf[n++] = SYM_LATCH;

    rmt_transmit_config_t tx = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(chan, enc, buf,
                                 n * sizeof(rmt_symbol_word_t), &tx));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(chan, 1000));
}

static void ws2812_phase(void)
{
    gpio_reset_pin(PIN_LED);

    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = PIN_LED,
        .mem_block_symbols = 64,
        .resolution_hz     = RMT_RES_HZ,
        .trans_queue_depth = 4,
    };
    rmt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &chan));

    rmt_copy_encoder_config_t enc_cfg = {};
    rmt_encoder_handle_t enc = NULL;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &enc));

    ESP_ERROR_CHECK(rmt_enable(chan));

    const size_t cap = (size_t)LED_COUNT * 24 + 1;
    rmt_symbol_word_t *buf = calloc(cap, sizeof(rmt_symbol_word_t));
    if (buf == NULL) {
        ESP_LOGE(TAG, "calloc failed");
        return;
    }

    ESP_LOGI(TAG, "PHASE B: RMT WS2812 on GPIO%d (%d leds)", PIN_LED, LED_COUNT);

    /* 依次: 红 / 绿 / 蓝 / 白, 每色 1.5s */
    static const uint8_t colors[4][3] = {
        { 255,   0,   0 },
        {   0, 255,   0 },
        {   0,   0, 255 },
        { 255, 255, 255 },
    };
    static const char *names[4] = { "RED", "GREEN", "BLUE", "WHITE" };

    for (int k = 0; k < 4; k++) {
        uint8_t rgb[LED_COUNT * 3];
        for (int i = 0; i < LED_COUNT; i++) {
            rgb[i * 3 + 0] = colors[k][0];
            rgb[i * 3 + 1] = colors[k][1];
            rgb[i * 3 + 2] = colors[k][2];
        }
        ESP_LOGI(TAG, "  WS2812 -> %s", names[k]);
        for (int t = 0; t < 30; t++) {      /* 持续重发, 保持常亮 */
            ws2812_send(chan, enc, buf, rgb);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    free(buf);
    rmt_disable(chan);
    rmt_del_channel(chan);
    rmt_del_encoder(enc);
    gpio_reset_pin(PIN_LED);
}

void app_main(void)
{
    ESP_LOGI(TAG, "GPIO%d LED test start", PIN_LED);

    for (int round = 1; ; round++) {
        ESP_LOGI(TAG, "===== round %d: PWM phase =====", round);
        pwm_phase();
        ESP_LOGI(TAG, "===== round %d: WS2812 phase =====", round);
        ws2812_phase();
        ESP_LOGI(TAG, "===== round %d done, repeat in 1s =====", round);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
