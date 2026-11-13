#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

typedef int portMUX_TYPE;
typedef void *TaskHandle_t;
typedef void *esp_timer_handle_t;
typedef void esp_console_repl_t;
typedef struct { void (*callback)(void *); const char *name; } esp_timer_create_args_t;
typedef struct { const char *prompt; int max_cmdline_length; } esp_console_repl_config_t;
typedef struct { int unused; } esp_console_dev_uart_config_t;
typedef struct { const char *command, *help; int (*func)(int,char **); } esp_console_cmd_t;

#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) do { assert(*(m) == 0); *(m) = 1; } while (0)
#define portEXIT_CRITICAL(m) do { assert(*(m) == 1); *(m) = 0; } while (0)
#define portMAX_DELAY UINT32_MAX
#define pdTRUE 1
#define pdPASS 1
#define ESP_ERROR_CHECK(result) assert((result) == 0)
#define ESP_CONSOLE_REPL_CONFIG_DEFAULT() {0}
#define ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT() {0}

int64_t esp_timer_get_time(void);
int esp_timer_create(const esp_timer_create_args_t *,esp_timer_handle_t *);
int esp_timer_start_periodic(esp_timer_handle_t,int64_t);
void vTaskDelay(unsigned int);
void ulTaskNotifyTake(int,uint32_t);
void xTaskNotifyGive(TaskHandle_t);
int xTaskCreate(void (*)(void *),const char *,int,void *,int,TaskHandle_t *);
int esp_console_new_repl_uart(const esp_console_dev_uart_config_t *,
                              const esp_console_repl_config_t *,esp_console_repl_t **);
int esp_console_cmd_register(const esp_console_cmd_t *);
int esp_console_start_repl(esp_console_repl_t *);
