/*
 * EC11L1525G01 旋转编码器测试 + WS2812 彩虹色相跟随
 *
 * ── 接线 ────────────────────────────────────────────────────
 *   GPIO2  -> 编码器 A 相
 *   GPIO4  -> 编码器 公共端 (本程序把它驱动为输出低电平, 当 GND 用)
 *   GPIO3  -> 编码器 B 相
 *   GPIO52 -> WS2812 灯珠
 *
 * ── 功能 ────────────────────────────────────────────────────
 *   - 正交解码, 能区分正向/反向:
 *       正向扭动 -> 色相增加 (红->黄->绿->青->蓝->紫)
 *       反向扭动 -> 色相减少 (往回走)
 *   - 平滑渐变: 目标是阶梯式变化, 显示色相按一阶趋近缓慢追上,
 *     所以不会一下子跳色, 而是像呼吸一样滑过去
 *   - 不扭动时不发数据, 灯保持当前颜色
 *
 * ── 关于 EC11 的"格"(detent) ────────────────────────────────
 *   机械编码器每格通常产生 4 个正交状态跳变。
 *   这里用 QDEC_PER_DETENT = 4 把它们折算成"每格"。
 *   如果你转一格色相变化太灵敏/太迟钝, 就调 HUE_PER_DETENT。
 *
 * ── 解码为什么可靠 ──────────────────────────────────────────
 *   用相位状态查表法 (16 项表), 只接受合法的相邻状态跳变,
 *   抖动产生的不合法跳变(如 A、B 同时变)计 0, 天然滤掉毛刺。
 */

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_timer.h"
#include "esp_log.h"

/* ═════════════ 引脚 ═════════════ */
#define PIN_ENC_A   2
#define PIN_ENC_B   3
#define PIN_ENC_COM 4       /* 公共端, 代码里拉低当 GND */
#define PIN_LED     52

/* ═════════════ 参数 ═════════════ */
#define QDEC_PER_DETENT   4.0f    /* 每格的正交跳变数 */
#define HUE_PER_DETENT    12.0f   /* 每格色相变化(度), 30 格 = 一圈 */
#define HUE_EASE          0.15f   /* 色相平滑系数, 越小越"黏" */
#define LED_REFRESH_MS    10      /* 色相刷新周期 */
#define ENC_POLL_MS       1       /* 编码器采样周期 */

#define LED_COUNT         2
#define LED_RMT_RES_HZ    10000000

static const char *TAG = "enc";

/* ═════════════ WS2812 ═════════════ */
static const rmt_symbol_word_t LED_BIT_0 = {
    .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9,
};
static const rmt_symbol_word_t LED_BIT_1 = {
    .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3,
};
static const rmt_symbol_word_t LED_LATCH = {
    .level0 = 0, .duration0 = 3000, .level1 = 0, .duration1 = 0,
};

static rmt_channel_handle_t s_led_chan = NULL;
static rmt_encoder_handle_t s_led_enc  = NULL;
static rmt_symbol_word_t   *s_led_buf  = NULL;

static bool led_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = PIN_LED,
        .mem_block_symbols = 64,
        .resolution_hz     = LED_RMT_RES_HZ,
        .trans_queue_depth = 4,
    };
    if (rmt_new_tx_channel(&chan_cfg, &s_led_chan) != ESP_OK) {
        ESP_LOGE(TAG, "led rmt channel failed");
        return false;
    }
    rmt_copy_encoder_config_t enc_cfg = {};
    if (rmt_new_copy_encoder(&enc_cfg, &s_led_enc) != ESP_OK) {
        return false;
    }
    if (rmt_enable(s_led_chan) != ESP_OK) {
        return false;
    }
    s_led_buf = calloc((size_t)LED_COUNT * 24 + 1, sizeof(rmt_symbol_word_t));
    return s_led_buf != NULL;
}

static void led_show(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_buf == NULL) return;

    const uint8_t grb[3] = { g, r, b };     /* WS2812 顺序 G,R,B */
    size_t n = 0;

    for (int led = 0; led < LED_COUNT; led++) {
        for (int c = 0; c < 3; c++) {
            for (int bit = 7; bit >= 0; bit--) {
                s_led_buf[n++] = (grb[c] & (1u << bit)) ? LED_BIT_1 : LED_BIT_0;
            }
        }
    }
    s_led_buf[n++] = LED_LATCH;

    rmt_transmit_config_t tx = { .loop_count = 0 };
    if (rmt_transmit(s_led_chan, s_led_enc, s_led_buf,
                     n * sizeof(rmt_symbol_word_t), &tx) == ESP_OK) {
        rmt_tx_wait_all_done(s_led_chan, 100);
    }
}

/* HSV -> RGB, h 单位度, s/v 单位 0..1 */
static void hsv2rgb(float h, float s, float v,
                    uint8_t *r, uint8_t *g, uint8_t *b)
{
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;

    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;

    float rf = 0, gf = 0, bf = 0;
    if      (h <  60.0f) { rf = c; gf = x; bf = 0; }
    else if (h < 120.0f) { rf = x; gf = c; bf = 0; }
    else if (h < 180.0f) { rf = 0; gf = c; bf = x; }
    else if (h < 240.0f) { rf = 0; gf = x; bf = c; }
    else if (h < 300.0f) { rf = x; gf = 0; bf = c; }
    else                 { rf = c; gf = 0; bf = x; }

    *r = (uint8_t)((rf + m) * 255.0f + 0.5f);
    *g = (uint8_t)((gf + m) * 255.0f + 0.5f);
    *b = (uint8_t)((bf + m) * 255.0f + 0.5f);
}

/* ═════════════ 正交解码 ═════════════
 * 状态 = (A<<1) | B, 共 4 种合法状态。
 * 表索引 = (上一状态<<2) | 当前状态, 值为本步的位移方向。
 * 非法跳变(如两位同时变)返回 0, 自动滤掉抖动。
 */
static const int8_t k_qdec[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static int s_qdec_prev = 0;
static int32_t s_qdec_pos = 0;      /* 累加的正交跳变数, 可正可负 */

static void enc_init(void)
{
    /* 公共端: 输出低电平, 给编码器当 GND 用 */
    gpio_config_t com_cfg = {
        .pin_bit_mask = 1ULL << PIN_ENC_COM,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&com_cfg));
    gpio_set_level(PIN_ENC_COM, 0);

    /* A/B 相: 输入 + 内部上拉 */
    gpio_config_t ab_cfg = {
        .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ab_cfg));

    s_qdec_prev = (gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B);
    s_qdec_pos  = 0;

    ESP_LOGI(TAG, "encoder init: A=GPIO%d B=GPIO%d COM=GPIO%d(low) initial=%d",
             PIN_ENC_A, PIN_ENC_B, PIN_ENC_COM, s_qdec_prev);
}

/* 采样一次, 返回本次的位移 (-1 / 0 / +1) */
static int enc_poll(void)
{
    int cur = (gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B);
    if (cur == s_qdec_prev) return 0;

    int8_t d = k_qdec[(s_qdec_prev << 2) | cur];
    s_qdec_prev = cur;
    s_qdec_pos += d;
    return d;
}

/* ═════════════ 主任务 ═════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "EC11 encoder + WS2812 hue test");
    ESP_LOGI(TAG, "  A=GPIO%d  B=GPIO%d  COM=GPIO%d  LED=GPIO%d",
             PIN_ENC_A, PIN_ENC_B, PIN_ENC_COM, PIN_LED);

    if (!led_init()) {
        ESP_LOGE(TAG, "LED init failed");
        return;
    }

    enc_init();

    /* 起始色: 红 */
    float hue_target  = 0.0f;      /* 阶梯目标 (累积, 不取模) */
    float hue_current = 0.0f;      /* 平滑后的色相 */
    float hue_sent    = -999.0f;   /* 上次已下发的色相 */
    int   last_detent = 0;
    int   last_dir    = 0;

    int64_t next_led_us = esp_timer_get_time();
    int64_t next_log_us = next_led_us + 500000;

    for (;;) {
        /* ── 1kHz 轮询正交编码器 ── */
        enc_poll();

        /* 把跳变数折算成"格", 只有格数变化才更新目标色相 */
        int detent = (int)lroundf((float)s_qdec_pos / QDEC_PER_DETENT);
        if (detent != last_detent) {
            int dir = (detent > last_detent) ? +1 : -1;
            if (dir != last_dir) {
                ESP_LOGI(TAG, "direction %s", (dir > 0) ? "FORWARD" : "REVERSE");
                last_dir = dir;
            }
            hue_target = (float)detent * HUE_PER_DETENT;
            last_detent = detent;
        }

        /* ── 色相平滑趋近 (10ms 一次) ── */
        int64_t now = esp_timer_get_time();
        if (now >= next_led_us) {
            next_led_us += LED_REFRESH_MS * 1000;

            hue_current += (hue_target - hue_current) * HUE_EASE;

            if (fabsf(hue_current - hue_sent) > 0.3f) {
                uint8_t r, g, b;
                hsv2rgb(hue_current, 1.0f, 0.35f, &r, &g, &b);
                led_show(r, g, b);
                hue_sent = hue_current;
            }
        }

        /* ── 每 0.5s 打印一次状态, 方便验证解码 ── */
        if (now >= next_log_us) {
            next_log_us += 500000;
            float hue_now = fmodf(hue_current, 360.0f);
            if (hue_now < 0.0f) hue_now += 360.0f;
            ESP_LOGI(TAG, "pos=%ld detent=%d target=%6.1f deg  shown=%6.1f deg",
                     (long)s_qdec_pos, last_detent,
                     hue_target, hue_now);
        }

        vTaskDelay(pdMS_TO_TICKS(ENC_POLL_MS));
    }
}
