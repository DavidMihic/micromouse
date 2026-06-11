#include <string.h>

#include "uart_link.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define UART_PORT       UART_NUM_1
#define UART_TX_GPIO    17   // ESP TX -> robot UART_RX
#define UART_RX_GPIO    18   // ESP RX -> robot UART_TX
#define UART_BAUD       115200
#define UART_RX_BUF     1024
#define LINE_MAX        512  // longest telemetry line we accept

static const char *TAG = "uart_link";

// Shared latest telemetry line, protected by a mutex.
static char latest[LINE_MAX] = "{}";
static SemaphoreHandle_t latest_mutex;

// Store a complete line as the latest telemetry (called from the RX task).
static void commit_line(const char *line)
{
    if (xSemaphoreTake(latest_mutex, portMAX_DELAY)) {
        strncpy(latest, line, sizeof(latest) - 1);
        latest[sizeof(latest) - 1] = '\0';
        xSemaphoreGive(latest_mutex);
    }
}

// Read bytes from the UART, assemble newline-terminated lines, commit each.
static void rx_task(void *arg)
{
    uint8_t chunk[256];
    char    line[LINE_MAX];
    size_t  len = 0;

    while (1) {
        int n = uart_read_bytes(UART_PORT, chunk, sizeof(chunk), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            char c = (char)chunk[i];
            if (c == '\n' || c == '\r') {
                if (len > 0) {
                    line[len] = '\0';
                    if (line[0] == '{')   // accept only JSON telemetry, ignore debug prints
                        commit_line(line);
                    len = 0;
                }
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else {
                len = 0; // overflow: drop the malformed line
                ESP_LOGW(TAG, "Line too long, dropped");
            }
        }
    }
}

size_t uart_link_get_telemetry(char *out, size_t out_size)
{
    size_t copied = 0;
    if (xSemaphoreTake(latest_mutex, portMAX_DELAY)) {
        strncpy(out, latest, out_size - 1);
        out[out_size - 1] = '\0';
        copied = strlen(out);
        xSemaphoreGive(latest_mutex);
    }
    return copied;
}

void uart_link_send(const char *line)
{
    uart_write_bytes(UART_PORT, line, strlen(line));
    uart_write_bytes(UART_PORT, "\n", 1);
}

void uart_link_init(void)
{
    latest_mutex = xSemaphoreCreateMutex();
    assert(latest_mutex);

    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(rx_task, "uart_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART%d up @ %d baud (TX=%d, RX=%d)",
             UART_PORT, UART_BAUD, UART_TX_GPIO, UART_RX_GPIO);
}