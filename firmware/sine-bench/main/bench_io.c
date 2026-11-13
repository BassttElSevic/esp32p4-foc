#include "bench_io.h"

#include <math.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "foc_parameters.h"

enum { SENSOR_TIMEOUT_MS = 3, ADC_MAX_CODE = 4095 };
static const ledc_channel_t channels[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cal_u, cal_v;
static i2c_master_dev_handle_t encoder;

void bench_io_disable(void)
{
    gpio_set_level(PIN_EN, 0);
}

void bench_io_enable(void)
{
    gpio_set_level(PIN_EN, 1);
}

static void setup_adc_channel(adc_channel_t channel, adc_cali_handle_t *cal)
{
    const adc_oneshot_chan_cfg_t cfg = {
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, channel, &cfg));
    const adc_cali_curve_fitting_config_t ccfg = {
        .unit_id = ADC_UNIT_1, .chan = channel,
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&ccfg, cal);
    if (err != ESP_OK) {
        *cal = NULL; /* An uncalibrated current scale cannot enforce the bench limit. */
        ESP_LOGE("bench_io", "ADC calibration unavailable on channel %d: %s", channel,
                 esp_err_to_name(err));
    }
}

void bench_io_init(void)
{
    /* Preload the low output latch before the pad becomes an output. An external
     * pull-down on the active-high EN is needed during chip reset / flashing.
     */
    ESP_ERROR_CHECK(gpio_set_level(PIN_EN, 0));
    const gpio_config_t enable = {
        .pin_bit_mask = 1ULL << PIN_EN, .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&enable));
    bench_io_disable();

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0, .freq_hz = PWM_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const int pins[] = {PIN_PWM_U, PIN_PWM_V, PIN_PWM_W};
    for (int k = 0; k < 3; ++k) {
        const ledc_channel_config_t ch = {
            .gpio_num = pins[k], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = channels[k],
            .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
            .duty = (uint32_t)(0.5f * PWM_DUTY_MAX + 0.5f),
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }

    const adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
    setup_adc_channel(ADC_CH_U, &cal_u);
    setup_adc_channel(ADC_CH_V, &cal_v);

    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0, .sda_io_num = PIN_I2C_SDA, .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = I2C_GLITCH_CNT,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    const i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = AS5600_ADDR,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev, &encoder));
}

bool bench_io_voltage(float theta, float vd, float vq)
{
    if (!isfinite(theta) || !isfinite(vd) || !isfinite(vq)) {
        bench_io_disable();
        return false;
    }
    float c = cosf(theta), s = sinf(theta);
    float a = vd * c - vq * s;
    float b = vd * s + vq * c;
    const float phases[] = {a, -0.5f*a + 0.5f*SQRT3_F*b, -0.5f*a - 0.5f*SQRT3_F*b};
    uint32_t counts[3];
    for (int k = 0; k < 3; ++k) {
        if (!isfinite(phases[k]) || fabsf(phases[k]) > 0.5f) {
            bench_io_disable();
            return false;
        }
        counts[k] = (uint32_t)((0.5f + phases[k]) * PWM_DUTY_MAX + 0.5f);
    }
    for (int k = 0; k < 3; ++k) {
        if (ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[k], counts[k]) != ESP_OK ||
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[k]) != ESP_OK) {
            bench_io_disable();
            return false;
        }
    }
    return true;
}

static bool read_mv(adc_channel_t channel, adc_cali_handle_t cal, float *mv)
{
    if (!cal) return false;
    int sum = 0;
    for (int k = 0; k < ADC_OVERSAMPLE; ++k) {
        int code;
        if (adc_oneshot_read(adc, channel, &code) != ESP_OK || code <= 0 || code >= ADC_MAX_CODE)
            return false;
        sum += code;
    }
    int value;
    if (adc_cali_raw_to_voltage(cal, sum / ADC_OVERSAMPLE, &value) != ESP_OK ||
        value <= RAIL_MARGIN_MV || value >= ADC_FULL_MV - RAIL_MARGIN_MV) return false;
    *mv = value;
    return true;
}

bool bench_io_sample(float *u_mv, float *v_mv, int *angle_code)
{
    float u, v;
    /* Contiguous AS5600 registers: STATUS, RAW ANGLE high, RAW ANGLE low.
     * MD must be set and neither field-strength warning (ML, MH) may be set.
     */
    const uint8_t reg = AS5600_REG_STATUS;
    uint8_t data[3];
    if (!read_mv(ADC_CH_U, cal_u, &u) || !read_mv(ADC_CH_V, cal_v, &v) ||
        i2c_master_transmit_receive(encoder, &reg, 1, data, sizeof(data), SENSOR_TIMEOUT_MS) != ESP_OK ||
        (data[0] & 0x38U) != 0x20U) {
        bench_io_disable();
        return false;
    }
    *u_mv = u;
    *v_mv = v;
    *angle_code = ((int)data[1] << 8 | data[2]) & AS5600_ANGLE_MASK;
    return true;
}
