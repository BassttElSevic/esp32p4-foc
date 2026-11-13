/*
 * ESP32-P4  BLDC —— AS5600 磁编码器闭环换相 + 速度闭环
 *
 * ── 接口 ────────────────────────────────────────────────────
 *   GPIO31 -> M1-PWM-1 (U)   GPIO30 -> M1-PWM-2 (V)
 *   GPIO29 -> M1-PWM-3 (W)   GPIO28 -> M1-EN
 *   GPIO20 -> ISENS-1 U 相电流 (ADC1_CH4)  INA199A1, 0.5Ω, 增益50
 *   GPIO21 -> ISENS-2 V 相电流 (ADC1_CH5)
 *   GPIO8  -> AS5600 SCL
 *   GPIO9  -> AS5600 SDA
 *
 * ── 串口命令 ────────────────────────────────────────────────
 *   help        命令列表
 *   cal         重新做电角度标定 (必须转子能自由转动)
 *   run         启动
 *   stop        停机
 *   f <hz>      目标电频率 (可负=反转), 现在是速度闭环的给定值
 *   i <mA>      电流上限 (作为电压幅值的上限约束)
 *   st          打印状态 (角度/转速/电压/电流)
 *
 * ── 相比开环版本的根本变化 ──────────────────────────────────
 *   换相角度 θ : 【闭环】直接从 AS5600 实测
 *   转速       : 【闭环】PI 调节 Vq, 用编码器差分测速
 *   电压       : 速度环输出 + 电流上限约束 (电压模式, 因为电流采样单向)
 *
 *   这就是电压模式 FOC: 换相和转速都是闭环的, 只是转矩靠电压间接控制。
 *   新板子(10mΩ + REF=1.65V)到位后改成电流闭环即可获得精确转矩。
 *
 * ── 上电自动标定 ────────────────────────────────────────────
 *   给两个不同电角度(0 和 π/2)各施加一次吸引磁场, 读两次编码器:
 *     - 机械位移 Δθ_m 应该等于 (π/2)/极对数
 *     - 由此**实测出极对数**, 型号不明也能适配
 *     - 顺带定出编码器方向(dir)和电角度零点(offset)
 *
 * ── 踩过的坑 ────────────────────────────────────────────────
 *   1) 相位不能用"固定步数 × 假设每步 N ms": tick=100Hz 时
 *      pdMS_TO_TICKS(2)==0, 延时失效, 频率跑到目标 50 倍。
 *   2) 周期性信号不能用瞬时值打日志(采样易混叠), 必须用窗口均值。
 *   3) INA199A1 增益是 50 V/V, 不是 1。
 *   4) V/f 斜率/前馈不能用理论算, 用实测工作点标定。
 *   5) 开环时电流目标设太小会导致转矩不足 -> 转子原地抖动(失步)。
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#include "foc_parameters.h"

static const char *TAG = "bldc";

static const ledc_channel_t s_ch[3] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
};
static const int s_pin[3] = { PIN_PWM_U, PIN_PWM_V, PIN_PWM_W };

/* ═════════════ 串口可改的运行参数 ═════════════ */
static volatile float g_f_target = 8.0f;
static volatile float g_i_limit  = I_LIMIT_DEFAULT_MA;
static volatile int   g_run      = 0;
static volatile int   g_recal    = 0;
static volatile int   g_openloop = 0;
static volatile int   g_dump     = 0;   /* >0: 剩余采样次数 */   /* 1 = 开环 V/f 测试, 不看编码器 */

/* 运行观测值 */
static volatile float g_f_meas   = 0.0f;
static volatile float g_rpm      = 0.0f;
static volatile float g_i_amp    = 0.0f;
static volatile float g_vq_now   = 0.0f;
static volatile float g_th_e_deg = 0.0f;
static volatile float g_enc_deg  = 0.0f;
static volatile int   g_pole_pairs = MOTOR_POLE_PAIRS_INIT;

/* ═════════════ 状态 ═════════════ */
static adc_oneshot_unit_handle_t s_adc    = NULL;
static adc_cali_handle_t         s_cali_u = NULL;
static adc_cali_handle_t         s_cali_v = NULL;
static i2c_master_bus_handle_t   s_bus    = NULL;
static i2c_master_dev_handle_t   s_dev    = NULL;

static int   s_enc_addr = AS5600_ADDR;
static float s_enc_dir  = 1.0f;
static float s_enc_off  = 0.0f;
static float s_span_up[2] = { 0.0f, 0.0f };
static float s_span_dn[2] = { 0.0f, 0.0f };
static bool  s_unipolar = false;
static float s_theta_ol = 0.0f;    /* 开环模式的电角度 */

/* ═════════════ 数学助手 ═════════════ */
static float wrap_pi(float a)
{
    while (a >  3.14159265f) a -= TWO_PI_F;
    while (a < -3.14159265f) a += TWO_PI_F;
    return a;
}

static float wrap_2pi(float a)
{
    a = fmodf(a, TWO_PI_F);
    if (a < 0.0f) a += TWO_PI_F;
    return a;
}

/* ═════════════ PWM ═════════════ */

static void drive_3phase_uv(float uu, float uv, float uw)
{
    const float u[3] = { uu, uv, uw };

    for (int i = 0; i < 3; i++) {
        float v = 0.5f + u[i];
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;

        uint32_t duty = (uint32_t)(v * PWM_DUTY_MAX + 0.5f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, s_ch[i], duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, s_ch[i]);
    }
}

/* 电压模式 FOC: 在电角度 th_e 上施加 (vd, vq) */
static void foc_output(float th_e, float vd, float vq)
{
    float ct = cosf(th_e);
    float st = sinf(th_e);
    float valpha = vd * ct - vq * st;
    float vbeta  = vd * st + vq * ct;

    drive_3phase_uv(valpha,
                    -0.5f * valpha + 0.5f * SQRT3_F * vbeta,
                    -0.5f * valpha - 0.5f * SQRT3_F * vbeta);
}

/* 纯 d 轴吸引: 把转子吸到电角度 th 上 */
static void apply_field(float th, float amp)
{
    foc_output(th, amp, 0.0f);
}

/* ═════════════ WS2812 状态指示灯 (GPIO52) ═════════════ */
#define PIN_LED        52
#define LED_COUNT      2
#define LED_RMT_RES_HZ 10000000

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

static void led_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = PIN_LED,
        .mem_block_symbols = 64,
        .resolution_hz     = LED_RMT_RES_HZ,
        .trans_queue_depth = 4,
    };
    if (rmt_new_tx_channel(&chan_cfg, &s_led_chan) != ESP_OK) {
        ESP_LOGW(TAG, "led rmt channel failed");
        s_led_chan = NULL;
        return;
    }

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &s_led_enc));
    ESP_ERROR_CHECK(rmt_enable(s_led_chan));

    s_led_buf = calloc((size_t)LED_COUNT * 24 + 1, sizeof(rmt_symbol_word_t));
    if (s_led_buf == NULL) {
        s_led_chan = NULL;
    }
}

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_chan == NULL || s_led_buf == NULL) return;

    const uint8_t rgb[3] = { r, g, b };
    const uint8_t grb[3] = { rgb[1], rgb[0], rgb[2] };   /* WS2812 = G,R,B */
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

static void pwm_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = s_pin[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = s_ch[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = (uint32_t)PWM_DUTY_MID,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
}

static void en_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_EN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(PIN_EN, 0);
}

/* ═════════════ AS5600 ═════════════ */

static bool as5600_read_reg(uint8_t reg, uint8_t *buf, size_t n)
{
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, n,
                                                I2C_TIMEOUT_MS);
    return err == ESP_OK;
}

/* 带重试的探测 (照 balancecar 的 i2c_bus_probe_retry) */
static bool as5600_probe_retry(void)
{
    esp_err_t err = ESP_FAIL;
    for (int i = 1; i <= I2C_PROBE_RETRY; i++) {
        err = i2c_master_probe(s_bus, AS5600_ADDR, I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "0x%02X probe OK (%d/%d)", AS5600_ADDR, i, I2C_PROBE_RETRY);
            return true;
        }
        ESP_LOGW(TAG, "0x%02X no ack (%d/%d): %s", AS5600_ADDR, i,
                 I2C_PROBE_RETRY, esp_err_to_name(err));
        if (i < I2C_PROBE_RETRY) vTaskDelay(pdMS_TO_TICKS(I2C_PROBE_GAP_MS));
    }
    ESP_LOGE(TAG, "0x%02X probe failed: %s", AS5600_ADDR, esp_err_to_name(err));
    return false;
}

/* 读原始机械角度, 返回弧度 [0, 2π) */
static bool as5600_read_angle(float *mech_rad)
{
    uint8_t b[2];
    if (!as5600_read_reg(AS5600_REG_RAW_ANGLE, b, 2)) return false;

    uint16_t raw = (uint16_t)((b[0] << 8) | b[1]) & 0x0FFF;
    *mech_rad = (float)raw / AS5600_RES * TWO_PI_F;
    return true;
}

/* ═════════════ AS5600 初始化 ═════════════ */

static void encoder_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_I2C_SDA,
        .scl_io_num        = PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_GLITCH_CNT,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
    ESP_LOGI(TAG, "i2c bus: port=0 SDA=GPIO%d SCL=GPIO%d %dHz",
             PIN_I2C_SDA, PIN_I2C_SCL, I2C_SPEED_HZ);

    /* 照 balancecar: 总线起来后先等 500ms 让传感器上电稳定 */
    vTaskDelay(pdMS_TO_TICKS(ENCODER_SETTLE_MS));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AS5600_ADDR,
        .scl_speed_hz    = I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));
    ESP_LOGI(TAG, "device attached 0x%02X @ %dHz", AS5600_ADDR, I2C_SPEED_HZ);

    /* ── 正常读一次 ──────────────────────────────────────── */
    if (!as5600_probe_retry()) {
        led_set(64, 0, 0);   /* 红: 编码器无应答 */
        return;
    }

    /* 初始化连读两次 (照 balancecar, 第一次可能是旧值) */
    uint16_t raw0 = 0, raw1 = 0;
    float    a0 = 0.0f, a1 = 0.0f;
    uint8_t  b[2] = { 0, 0 };

    for (int round = 1; round <= 2; round++) {
        if (!as5600_read_reg(AS5600_REG_RAW_ANGLE, b, 2)) {
            ESP_LOGE(TAG, "read 0x0C failed (round %d)", round);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        uint16_t raw = (uint16_t)(((uint16_t)b[0] << 8) | b[1]) & AS5600_ANGLE_MASK;
        if (round == 1) { raw0 = raw; a0 = (float)raw / AS5600_RES * TWO_PI_F; }
        else            { raw1 = raw; a1 = (float)raw / AS5600_RES * TWO_PI_F; }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    uint8_t st = 0;
    bool st_ok = as5600_read_reg(AS5600_REG_STATUS, &st, 1);

    ESP_LOGI(TAG, "AS5600 online 0x%02X  0x0C=%u/%u  angle=%.2f/%.2f deg  STATUS=%s0x%02X",
             AS5600_ADDR, raw0, raw1, a0 * 57.2958f, a1 * 57.2958f,
             st_ok ? "" : "(read failed) ", st);
    if (st_ok) {
        led_set(0, 64, 0);   /* 绿: 读成功 */
        ESP_LOGI(TAG, "magnet=%s%s%s",
                 (st & 0x20) ? "detected" : "NOT-DETECTED",
                 (st & 0x10) ? " too-weak" : "",
                 (st & 0x08) ? " too-strong" : "");
    }
}

/* ═════════════ ADC ═════════════ */

static adc_cali_handle_t make_cali(adc_channel_t ch)
{
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ch,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_handle_t h = NULL;
    if (adc_cali_create_scheme_curve_fitting(&cfg, &h) != ESP_OK) {
        ESP_LOGW(TAG, "cali ch%d unavailable, raw fallback", (int)ch);
        return NULL;
    }
    return h;
}

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH_U, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH_V, &chan_cfg));

    s_cali_u = make_cali(ADC_CH_U);
    s_cali_v = make_cali(ADC_CH_V);
}

static float adc_read_mv(adc_channel_t ch, adc_cali_handle_t cali)
{
    int sum = 0;
    for (int i = 0; i < ADC_OVERSAMPLE; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc, ch, &raw);
        sum += raw;
    }
    int avg = sum / ADC_OVERSAMPLE;
    int mv  = 0;
    if (cali) {
        adc_cali_raw_to_voltage(cali, avg, &mv);
    } else {
        mv = avg * (int)ADC_FULL_MV / 4095;
    }
    return (float)mv;
}

/* ═════════════ 电角度标定 ═════════════ */

/* 返回 true 表示标定成功。会实测出极对数、编码器方向、电角度零点。 */
static bool calibrate_electrical_angle(void)
{
    ESP_LOGI(TAG, "calibration: pulling rotor to electrical 0 deg ...");

    apply_field(0.0f, CAL_AMP);
    vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

    float m0 = 0.0f;
    if (!as5600_read_angle(&m0)) {
        ESP_LOGE(TAG, "calibration failed: encoder read error");
        return false;
    }

    ESP_LOGI(TAG, "calibration: pulling rotor to electrical 90 deg ...");
    apply_field(TWO_PI_F / 4.0f, CAL_AMP);
    vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

    float m1 = 0.0f;
    if (!as5600_read_angle(&m1)) {
        ESP_LOGE(TAG, "calibration failed: encoder read error");
        return false;
    }

    float d = wrap_pi(m1 - m0);
    ESP_LOGI(TAG, "calibration: mech moved %.2f deg for 90 deg electrical "
                  "(ideal %.2f for pp=%d)",
             d * 57.2958f, 90.0f / (float)MOTOR_POLE_PAIRS_INIT,
             MOTOR_POLE_PAIRS_INIT);

    if (fabsf(d) < 0.02f) {
        ESP_LOGE(TAG, "calibration failed: rotor did not move "
                      "(no torque, or encoder not coupled)");
        return false;
    }

    /* 极对数按 datasheet 固定 = 7, 不再用测量值覆盖。
     * 测量值只做诊断: 若明显偏离 7, 说明标定转矩不足、转子滞后。 */
    const int pp = MOTOR_POLE_PAIRS_INIT;
    float pp_meas = (3.14159265f / 2.0f) / fabsf(d);
    if (fabsf(pp_meas - (float)pp) / (float)pp > 0.15f) {
        ESP_LOGW(TAG, "pp measured %.2f differs from %d - rotor lag, "
                      "offset may be slightly off", pp_meas, pp);
    } else {
        ESP_LOGI(TAG, "pp measured %.2f (consistent with %d)", pp_meas, pp);
    }

    g_pole_pairs = pp;

    s_enc_dir = (d > 0.0f) ? 1.0f : -1.0f;
    s_enc_off = wrap_2pi(-s_enc_dir * (float)pp * m0);

    ESP_LOGI(TAG, "calibration OK: dir=%+.0f offset=%.1f deg pole_pairs=%d",
             s_enc_dir, s_enc_off * 57.2958f, pp);
    return true;
}

/* 由机械角度算电角度 */
static float mech_to_elec(float mech_rad)
{
    return wrap_2pi(s_enc_dir * (float)g_pole_pairs * mech_rad + s_enc_off);
}

/* ── 显式关闭 GPIO7 / GPIO8 ──────────────────────────────────
 * 这两脚当前固件未使用。若排母有连锡, 保持高阻输入可避免
 * 两边同时驱动造成的冲突/发热。 */
static void disable_gpio_7_8(void)
{
    const int pins[2] = { 7, 8 };
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_INPUT);
        gpio_pullup_dis(pins[i]);
        gpio_pulldown_dis(pins[i]);
    }
    ESP_LOGI(TAG, "GPIO7/GPIO8 set to high-Z input (disabled)");
}

/* ═════════════ 电机控制任务 ═════════════ */

typedef struct { float integ; } pi_t;

/* 带抗饱和的 PI: 输出限幅 ±limit */
static float pi_step(pi_t *pi, float ref, float meas, float dt, float limit)
{
    float err = ref - meas;
    float out = pi->integ + CUR_KP_AMP_PER_MA * err;

    if (out > limit) {
        out = limit;
        if (err < 0.0f) pi->integ += CUR_KI_AMP_PER_MA_S * err * dt;
    } else if (out < -limit) {
        out = -limit;
        if (err > 0.0f) pi->integ += CUR_KI_AMP_PER_MA_S * err * dt;
    } else {
        pi->integ += CUR_KI_AMP_PER_MA_S * err * dt;
    }
    if (pi->integ >  limit) pi->integ =  limit;
    if (pi->integ < -limit) pi->integ = -limit;
    return out;
}

static void motor_task(void *arg)
{
    /* 注意: I2C 现在用的就是 GPIO7/8, 所以不能再调用 disable_gpio_7_8() */
    led_init();
    led_set(0, 0, 32);      /* 蓝: 启动中 */

    pwm_init();
    en_init();
    adc_init();
    encoder_init();

    /* ── 电流零点标定 ── */
    drive_3phase_uv(0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(100));

    float sum_u = 0.0f, sum_v = 0.0f;
    const int cal_n = 200;
    for (int i = 0; i < cal_n; i++) {
        sum_u += adc_read_mv(ADC_CH_U, s_cali_u);
        sum_v += adc_read_mv(ADC_CH_V, s_cali_v);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    const float off_u_mv = sum_u / (float)cal_n;
    const float off_v_mv = sum_v / (float)cal_n;
    s_unipolar = (off_u_mv < 300.0f && off_v_mv < 300.0f);

    const float offs[2] = { off_u_mv, off_v_mv };
    for (int k = 0; k < 2; k++) {
        float up = ADC_FULL_MV - offs[k] - RAIL_MARGIN_MV;
        float dn = s_unipolar ? 0.0f : (offs[k] - RAIL_MARGIN_MV);
        if (up < 0.0f) up = 0.0f;
        if (dn < 0.0f) dn = 0.0f;
        s_span_up[k] = up;
        s_span_dn[k] = dn;
    }

    const float sens = AMP_GAIN * R_SHUNT_OHM;
    ESP_LOGI(TAG, "current offset U=%.1fmV V=%.1fmV %s", off_u_mv, off_v_mv,
             s_unipolar ? "[UNIPOLAR ref=GND]" : "[BIPOLAR]");

    /* ── 电角度标定 (需要驱动管使能) ── */
    gpio_set_level(PIN_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!calibrate_electrical_angle()) {
        ESP_LOGE(TAG, "ELECTRICAL CALIBRATION FAILED - FOC will not work");
    }
    drive_3phase_uv(0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_EN, 0);

    ESP_LOGI(TAG, "ready. type 'help'");

    /* ── 运行状态 ── */
    int     running = 0;
    pi_t    pi_d = { .integ = 0.0f };   /* d 轴电流 PI */
    pi_t    pi_q = { .integ = 0.0f };   /* q 轴电流 PI */
    pi_t    pi_spd = { .integ = 0.0f }; /* 速度外环 PI (输出 Iq 指令) */
    float   id_f = 0.0f, iq_f = 0.0f;   /* 低通后的 dq 电流 */
    float   omega_e_f = 0.0f;     /* 实测电角速度低通 */
    float   mech_prev = 0.0f;
    float   mv_last_u = 0.0f, mv_last_v = 0.0f;
    int64_t prev_enc_us = esp_timer_get_time();
    int     enc_ok_cnt = 0;
    uint32_t enc_ok_win = 0, enc_fail_win = 0;   /* 窗口内成功/失败计数 */
    int     ocp_hits = 0;

    int64_t prev_us = esp_timer_get_time();
    int64_t next_report_us = prev_us + 1000000;
    float acc_iamp = 0.0f, acc_vq = 0.0f, acc_om = 0.0f;
    float acc_iu = 0.0f, acc_iv = 0.0f, acc_iw = 0.0f;
    float acc_id = 0.0f, acc_iq = 0.0f, iq_ref_dbg = 0.0f;
    float iu_lo = 1e9f, iu_hi = -1e9f;
    float iv_lo = 1e9f, iv_hi = -1e9f;
    float iw_lo = 1e9f, iw_hi = -1e9f;
    int   acc_n = 0;

    for (;;) {
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - prev_us) / 1e6f;
        prev_us = now;
        if (dt < 0.0f)  dt = 0.0f;
        if (dt > 0.05f) dt = 0.05f;

        /* ── 重新标定请求 ── */
        if (g_recal) {
            g_recal = 0;
            g_run   = 0;
            running = 0;
            drive_3phase_uv(0.0f, 0.0f, 0.0f);
            gpio_set_level(PIN_EN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            calibrate_electrical_angle();
            drive_3phase_uv(0.0f, 0.0f, 0.0f);
            gpio_set_level(PIN_EN, 0);
            continue;
        }

        /* ── 停机 ── */
        if (!g_run) {
            if (running) {
                drive_3phase_uv(0.0f, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(20));
                gpio_set_level(PIN_EN, 0);
                running = 0;
                g_f_meas = 0.0f;
                g_rpm    = 0.0f;
                ESP_LOGW(TAG, "STOPPED");
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            prev_us = esp_timer_get_time();
            next_report_us = prev_us + 1000000;
            continue;
        }

        /* ── 启动 ── */
        if (!running) {
            gpio_set_level(PIN_EN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            pi_d.integ = 0.0f;
            pi_q.integ = 0.0f;
            pi_spd.integ = 0.0f;
            s_theta_ol = 0.0f;
            id_f = 0.0f;
            iq_f = 0.0f;
            omega_e_f = 0.0f;
            ocp_hits = 0;
            if (as5600_read_angle(&mech_prev)) { /* 初始化上一拍角度 */ }
            running = 1;
            prev_us = esp_timer_get_time();
#if USE_CURRENT_LOOP
            ESP_LOGI(TAG, "run: speed->current cascade, target %.2f Hz, I_max %.0f mA",
                     g_f_target, g_i_limit);
#else
            ESP_LOGI(TAG, "run: VOLTAGE-mode speed loop, target %.2f Hz", g_f_target);
#endif
            continue;
        }

        /* ══════ 开环模式: 不依赖编码器/标定 ══════ */
        if (g_openloop) {
            float f_ol = g_f_target;
            if (f_ol >  40.0f) f_ol =  40.0f;
            if (f_ol < -40.0f) f_ol = -40.0f;

            s_theta_ol += TWO_PI_F * f_ol * dt;
            while (s_theta_ol >= TWO_PI_F) s_theta_ol -= TWO_PI_F;
            while (s_theta_ol <  0.0f)     s_theta_ol += TWO_PI_F;

            apply_field(s_theta_ol, OL_AMP);   /* 纯 d 轴拖场 */

            float mvu = adc_read_mv(ADC_CH_U, s_cali_u);
            float mvv = adc_read_mv(ADC_CH_V, s_cali_v);
            float iu_ol = (mvu - off_u_mv) / sens;
            float iv_ol = (mvv - off_v_mv) / sens;
            float iw_ol = -(iu_ol + iv_ol);
            g_i_amp = iu_ol;

            if (now >= next_report_us) {
                next_report_us += 500000;
                ESP_LOGI(TAG,
                         "OL f=%6.2f amp=%.3f th=%6.1f | "
                         "Iu=%8.1f Iv=%8.1f Iw=%8.1f mA | mV %6.1f %6.1f",
                         f_ol, OL_AMP, s_theta_ol * 57.2958f,
                         iu_ol, iv_ol, iw_ol, mvu, mvv);
            }
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
            continue;
        }

        /* ══════ 1. 采样两相电流 (先做慢的 ADC, 让后面的角度尽量新鲜) ══════ */
        float mv_u = adc_read_mv(ADC_CH_U, s_cali_u);
        float mv_v = adc_read_mv(ADC_CH_V, s_cali_v);
        mv_last_u = mv_u;
        mv_last_v = mv_v;
        float rel_u = mv_u - off_u_mv;
        float rel_v = mv_v - off_v_mv;

        if (s_unipolar) {
            if (rel_u < 0.0f) rel_u = 0.0f;
            if (rel_v < 0.0f) rel_v = 0.0f;
        }

        float iu = SENSE_SIGN * rel_u / sens;
        float iv = SENSE_SIGN * rel_v / sens;
        float iw = -(iu + iv);

        acc_iu += iu;
        acc_iv += iv;
        acc_iw += iw;
        if (iu < iu_lo) iu_lo = iu;
        if (iu > iu_hi) iu_hi = iu;
        if (iv < iv_lo) iv_lo = iv;
        if (iv > iv_hi) iv_hi = iv;
        if (iw < iw_lo) iw_lo = iw;
        if (iw > iw_hi) iw_hi = iw;

        /* ══════ 2. 过流保护 (仅电流环模式下有效) ══════ */
        float imax = fabsf(iu);
        if (fabsf(iv) > imax) imax = fabsf(iv);
        if (fabsf(iw) > imax) imax = fabsf(iw);

#if USE_CURRENT_LOOP
        if (imax > OCP_MA) {
            if (++ocp_hits >= OCP_COUNT) {
                drive_3phase_uv(0.0f, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(20));
                gpio_set_level(PIN_EN, 0);
                g_run = 0; running = 0; ocp_hits = 0;
                ESP_LOGE(TAG, "OVERCURRENT %.0fmA -> stopped", imax);
                continue;
            }
        } else {
            ocp_hits = 0;
        }
#else
        (void)imax; (void)ocp_hits;
#endif

        /* ══════ 3. 读编码器 (放在 ADC 之后, 角度最新) ══════ */
        int64_t t_enc_us = esp_timer_get_time();
        float mech = mech_prev;
        bool  enc_ok = as5600_read_angle(&mech);
        if (enc_ok) {
            enc_ok_cnt++;
            enc_ok_win++;
            float dmech = wrap_pi(mech - mech_prev);
            mech_prev = mech;
            g_enc_deg = mech * 57.2958f;

            int64_t dt_enc = t_enc_us - prev_enc_us;
            prev_enc_us = t_enc_us;
            if (dt_enc > 100 && dt_enc < 20000) {
                float omega_e_raw = dmech / ((float)dt_enc * 1e-6f)
                                    * (float)g_pole_pairs;
                omega_e_f += SPEED_LPF_ALPHA * (omega_e_raw - omega_e_f);
            }
        } else {
            enc_ok_cnt = 0;
            enc_fail_win++;
            omega_e_f *= 0.9f;
        }

        /* ══════ 4. 电角度 + 预测补偿 ══════
         * 角度是在本轮开头发电机才动过的, 到真正输出还有 ~1 个循环周期。
         * 用实测电角速度乘以下一周期的时间做前向补偿。 */
        float th_meas = mech_to_elec(mech);
        float th_e = wrap_2pi(th_meas + omega_e_f * dt);
        g_th_e_deg = th_e * 57.2958f;

        /* ══════ 5. Clarke: 两相 (Iu,Iv) -> αβ ══════ */
        float ialpha = iu;
        float ibeta  = (iu + 2.0f * iv) * ONE_OVER_SQRT3;

        /* ══════ 6. Park: αβ -> dq (用实测电角度) ══════ */
        float ct = cosf(th_e);
        float st = sinf(th_e);
        float id =  ialpha * ct + ibeta * st;
        float iq = -ialpha * st + ibeta * ct;

        id_f += IAMP_FILT_ALPHA * (id - id_f);
        iq_f += IAMP_FILT_ALPHA * (iq - iq_f);

        acc_id += id_f;
        acc_iq += iq_f;

        /* ══════ 7. 速度环 ══════ */
        float f_tgt = g_f_target;
        if (f_tgt >  60.0f) f_tgt =  60.0f;
        if (f_tgt < -60.0f) f_tgt = -60.0f;

        float omega_ref = TWO_PI_F * f_tgt;
        float err_spd   = omega_ref - omega_e_f;

        float vd = 0.0f;
        float vq = 0.0f;

#if USE_CURRENT_LOOP
        /* ── 级联: 速度 PI -> Iq 指令 -> dq 电流 PI ── */
        float i_max = g_i_limit;

        float iq_ref = pi_spd.integ + SPD_KP_MA * err_spd;
        if (iq_ref > i_max) {
            iq_ref = i_max;
            if (err_spd < 0.0f) pi_spd.integ += SPD_KI_MA * err_spd * dt;
        } else if (iq_ref < -i_max) {
            iq_ref = -i_max;
            if (err_spd > 0.0f) pi_spd.integ += SPD_KI_MA * err_spd * dt;
        } else {
            pi_spd.integ += SPD_KI_MA * err_spd * dt;
        }
        if (pi_spd.integ >  i_max) pi_spd.integ =  i_max;
        if (pi_spd.integ < -i_max) pi_spd.integ = -i_max;

        iq_ref *= TORQUE_SIGN * s_enc_dir;
        iq_ref_dbg = iq_ref;

        vd = pi_step(&pi_d, 0.0f,   id_f, dt, AMP_MAX);
        vq = pi_step(&pi_q, iq_ref, iq_f, dt, AMP_MAX);

        float vmag = sqrtf(vd * vd + vq * vq);
        if (vmag > AMP_MAX) {
            float sc = AMP_MAX / vmag;
            vd *= sc;
            vq *= sc;
        }
#else
        /* ── 电压模式: 速度 PI 直接给 Vq, 不用电流反馈 ── */
        {
            float kp = SPD_V_KP, ki = SPD_V_KI;
            float out = pi_spd.integ + kp * err_spd;

            if (out >  AMP_MAX) { out =  AMP_MAX;
                if (err_spd < 0.0f) pi_spd.integ += ki * err_spd * dt; }
            else if (out < -AMP_MAX) { out = -AMP_MAX;
                if (err_spd > 0.0f) pi_spd.integ += ki * err_spd * dt; }
            else { pi_spd.integ += ki * err_spd * dt; }

            if (pi_spd.integ >  AMP_MAX) pi_spd.integ =  AMP_MAX;
            if (pi_spd.integ < -AMP_MAX) pi_spd.integ = -AMP_MAX;

            vq = out * TORQUE_SIGN * s_enc_dir;
            vd = 0.0f;
        }
#endif

        /* ══════ 采样导流: 每 20ms 打印一次三相电流 ══════ */
        if (g_dump > 0) {
            static int64_t dump_next = 0;
            if (dump_next == 0) dump_next = now;
            if (now >= dump_next) {
                dump_next += 10000;
                g_dump--;
                ESP_LOGI(TAG, "SMP mech=%7.2f th=%7.2f %s U=%8.1f V=%8.1f W=%8.1f mA",
                         mech * 57.2958f, th_e * 57.2958f,
                         enc_ok ? "OK  " : "FAIL", iu, iv, iw);
                if (g_dump == 0) dump_next = 0;
            }
        }

        float iamp = fabsf(iq_f);

        /* ── 输出: 反 Park + 反 Clarke ── */
        foc_output(th_e, vd, vq);

        /* 观测 */
        acc_iamp += iamp;
        acc_vq   += vq;
        acc_om   += omega_e_f;
        acc_n++;
        g_i_amp = iamp;
        g_vq_now = vq;

        vTaskDelay(pdMS_TO_TICKS(STEP_MS));

        /* ── 每秒上报 ── */
        if (now >= next_report_us) {
            float n  = (acc_n > 0) ? (float)acc_n : 1.0f;
            float ia = acc_iamp / n;
            float ome = acc_om / n;
            float fe  = ome / TWO_PI_F;
            float rpm = ome / (float)g_pole_pairs * 60.0f / TWO_PI_F;
            float tq  = ia * MOTOR_KT_NM_PER_A;

            g_f_meas = fe;
            g_rpm    = rpm;

            uint32_t tot = enc_ok_win + enc_fail_win;
            float fail_pct = tot ? (100.0f * (float)enc_fail_win / (float)tot) : 0.0f;
            ESP_LOGI(TAG,
                     "rpm=%6.1f Vq=%+.3f enc %lu/%lu | "
                     "Id=%6.1f Iq=%6.1f(ref%5.0f) | "
                     "Iu %5.1f[%6.1f..%6.1f] Iv %5.1f[%6.1f..%6.1f] mA",
                     rpm, acc_vq / n,
                     (unsigned long)enc_ok_win, (unsigned long)enc_fail_win,
                     acc_id / n, acc_iq / n, iq_ref_dbg,
                     acc_iu / n, iu_lo, iu_hi,
                     acc_iv / n, iv_lo, iv_hi);
            enc_ok_win = enc_fail_win = 0;
            iu_lo = iv_lo = iw_lo =  1e9f;
            iu_hi = iv_hi = iw_hi = -1e9f;

            acc_iamp = acc_vq = acc_om = 0.0f;
            acc_iu = acc_iv = acc_iw = 0.0f;
            acc_id = acc_iq = 0.0f;
            acc_n = 0;
            next_report_us += 1000000;
        }
    }
}

/* ═════════════ 串口命令 ═════════════ */

static int cmd_run(int argc, char **argv)
{
    (void)argc; (void)argv;
    g_run = 1;
    printf("run\n");
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    g_run = 0;
    g_openloop = 0;
    printf("stop requested\n");
    return 0;
}

static int cmd_cal(int argc, char **argv)
{
    (void)argc; (void)argv;
    g_recal = 1;
    printf("recalibrating (rotor must be free to move)\n");
    return 0;
}

static int cmd_f(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: f <hz>  (current %.2f, range -80..80)\n", g_f_target);
        return 1;
    }
    float v = strtof(argv[1], NULL);
    if (v >  80.0f) v =  80.0f;
    if (v < -80.0f) v = -80.0f;
    g_f_target = v;
    printf("target speed = %.2f Hz electrical\n", v);
    return 0;
}

static int cmd_i(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: i <mA>  (Iq ref %.1f, range -1800..1800, negative = reverse)\n",
               g_i_limit);
        return 1;
    }
    float v = strtof(argv[1], NULL);
    if (v < -1800.0f) v = -1800.0f;
    if (v >  1800.0f) v =  1800.0f;
    g_i_limit = v;
    printf("Iq reference = %.1f mA\n", v);
    return 0;
}

static int cmd_dump(int argc, char **argv)
{
    int n = (argc > 1) ? atoi(argv[1]) : 50;
    if (n < 1)   n = 1;
    if (n > 200) n = 200;
    g_dump = n;
    printf("dump %d samples @10ms\n", n);
    return 0;
}

static int cmd_ol(int argc, char **argv)
{
    (void)argc; (void)argv;
    g_openloop = !g_openloop;
    if (g_openloop) {
        g_run = 1;
        printf("open-loop ON (f=%g Hz, amp=%.3f)\n", (double)g_f_target, OL_AMP);
    } else {
        printf("open-loop OFF\n");
    }
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("state      : %s\n", g_run ? "RUNNING" : "IDLE");
    printf("enc mech   : %.1f deg\n", g_enc_deg);
    printf("elec angle : %.1f deg\n", g_th_e_deg);
    printf("pole pairs : %d\n", g_pole_pairs);
    printf("speed      : %.2f Hz elec  (target %.2f)\n", g_f_meas, g_f_target);
    printf("             %.1f rpm\n", g_rpm);
    printf("I limit    : %.1f mA   (actual %.1f mA)\n", g_i_limit, g_i_amp);
    printf("torque est : %.2f mNm (Kv=%.0f)\n",
           g_i_amp * MOTOR_KT_NM_PER_A, MOTOR_KV_RPM_PER_V);
    printf("Vq         : %.3f\n", g_vq_now);
    return 0;
}

static void console_init(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "bldc>";
    repl_cfg.max_cmdline_length = 64;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));

    esp_console_register_help_command();

    const esp_console_cmd_t cmds[] = {
        { .command = "run",  .help = "start closed-loop speed control", .func = cmd_run    },
        { .command = "stop", .help = "stop motor",                      .func = cmd_stop   },
        { .command = "cal",  .help = "re-run electrical angle calib",   .func = cmd_cal    },
        { .command = "f",    .help = "f <hz>  target electrical freq",  .func = cmd_f      },
        { .command = "i",    .help = "i <mA>  current limit",           .func = cmd_i      },
        { .command = "ol",   .help = "toggle open-loop V/f test",        .func = cmd_ol     },
        { .command = "dump", .help = "dump <n> current samples @10ms",   .func = cmd_dump   },
        { .command = "st",   .help = "print status",                    .func = cmd_status },
    };
    for (size_t k = 0; k < sizeof(cmds) / sizeof(cmds[0]); k++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[k]));
    }

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 BLDC FOC + AS5600 encoder");
    ESP_LOGI(TAG, "PWM %d/%d/%d EN %d | I2C SCL=%d SDA=%d",
             PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_EN, PIN_I2C_SCL, PIN_I2C_SDA);

    xTaskCreate(motor_task, "motor", 4096, NULL, 15, NULL);
    console_init();
}
