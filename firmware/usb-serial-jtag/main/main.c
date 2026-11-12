/*
 * ESP32-P4  USB Serial/JTAG 收发测试
 *
 * ── 硬件 ────────────────────────────────────────────────────
 *   GPIO24 = USB Serial/JTAG D−  (DM)
 *   GPIO25 = USB Serial/JTAG D+  (DP)   复位后 USB_PU 自动使能
 *   这两根通过 Type-A 接到电脑
 *
 * ── 本程序做什么 ────────────────────────────────────────────
 *   1. 安装 usb_serial_jtag 驱动, 独占该外设 (console 不占用)
 *   2. 每秒向 USB 端发送一行心跳, 电脑侧串口能看到
 *   3. 收到任何字节就原样回显 (echo), 并把内容打到 UART 端日志
 *   4. 每秒报告一次 usb_serial_jtag_is_connected():
 *        false -> 主机还没配置本设备 (线不通 / 没枚举)
 *        true  -> 枚举成功, 链路可用
 *
 * ── 日志怎么看 ──────────────────────────────────────────────
 *   日志走 UART0 (CH343 = /dev/ttyACM0)。
 *   如果 USB 枚举成功, 电脑上会多出一个新的串口(通常是 ttyACM1),
 *   那是 P4 的原生 USB, 不是 CH343。
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "usbtest";

#define RX_BUF      512
#define TX_BUF      512
#define HB_PERIOD_MS 1000

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 USB Serial/JTAG test (D-=GPIO24, D+=GPIO25)");

    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUF,
        .tx_buffer_size = TX_BUF,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "driver install failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "usb_serial_jtag driver installed");

    bool     last_conn = false;
    uint32_t rx_total = 0;
    uint32_t tx_hb    = 0;
    int64_t  next_hb  = esp_timer_get_time() + HB_PERIOD_MS * 1000;
    int64_t  next_rpt = esp_timer_get_time() + HB_PERIOD_MS * 1000;

    uint8_t buf[64];

    for (;;) {
        int64_t now = esp_timer_get_time();

        /* ── 收: 阻塞最多 20ms, 有数据就回显 ── */
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf),
                                          pdMS_TO_TICKS(20));
        if (n > 0) {
            rx_total += (uint32_t)n;

            /* 打印收到的内容 (可打印字符直出, 其余用 hex) */
            char pretty[128];
            int  p = 0;
            for (int i = 0; i < n && p < (int)sizeof(pretty) - 6; i++) {
                unsigned char c = buf[i];
                if (c >= 32 && c < 127) {
                    pretty[p++] = (char)c;
                } else {
                    p += snprintf(pretty + p, sizeof(pretty) - p, "\\x%02X", c);
                }
            }
            pretty[p] = '\0';
            ESP_LOGI(TAG, "RX %d bytes (total %lu): \"%s\"",
                     n, (unsigned long)rx_total, pretty);

            /* 回显 */
            int w = usb_serial_jtag_write_bytes(buf, n, pdMS_TO_TICKS(50));
            ESP_LOGI(TAG, "echo back %d/%d bytes", w, n);
        }

        /* ── 每秒发心跳 ── */
        if (now >= next_hb) {
            next_hb += HB_PERIOD_MS * 1000;
            char hb[64];
            int  len = snprintf(hb, sizeof(hb),
                                "P4-USB-HB #%lu\r\n", (unsigned long)tx_hb);
            int w = usb_serial_jtag_write_bytes(hb, (size_t)len,
                                                pdMS_TO_TICKS(50));
            if (w == len) {
                tx_hb++;
            } else {
                ESP_LOGW(TAG, "heartbeat write only %d/%d bytes", w, len);
            }
        }

        /* ── 每秒报告连接状态 ── */
        if (now >= next_rpt) {
            next_rpt += HB_PERIOD_MS * 1000;
            bool conn = usb_serial_jtag_is_connected();
            if (conn != last_conn) {
                if (conn) {
                    ESP_LOGI(TAG, "*** USB CONNECTED (host configured) ***");
                } else {
                    ESP_LOGW(TAG, "*** USB DISCONNECTED ***");
                }
                last_conn = conn;
            }
            ESP_LOGI(TAG, "connected=%d  hb_sent=%lu  rx_total=%lu",
                     (int)conn, (unsigned long)tx_hb,
                     (unsigned long)rx_total);
        }
    }
}
