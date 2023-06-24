#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <stdbool.h>
#include <esp_log.h>
#include <string.h>
#include "lcd_spi.h"

#define TAG "LCD_SPI"
spi_device_handle_t lcd_handle = NULL;

static void gpio_init()
{
    esp_err_t ret;
    ret = gpio_set_direction(PIN_DC, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);
    ret = gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "GPIO pins init");
}

static void spi_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 2 * LCD_DISP_H_RES * LCD_DISP_V_RES};
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH1);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "SPI bus init");
    spi_device_interface_config_t devcfg = {
        .spics_io_num = PIN_CS,
        .clock_speed_hz = SPI_MASTER_FREQ_80M,
        .address_bits = 0,
        .command_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .queue_size = 10};
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &lcd_handle);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Attached LCD to SPI bus");
}

void lcd_send_cmd(spi_device_handle_t lcd_handle, const uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.tx_data[0] = cmd;
    gpio_set_level(PIN_DC, 0);
    esp_err_t ret = spi_device_polling_transmit(lcd_handle, &t);
    ESP_ERROR_CHECK(ret);
}

void lcd_send_data(spi_device_handle_t lcd_handle, const uint8_t *data, uint16_t length)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * length;
    if (length <= 4)
    {
        t.flags = SPI_TRANS_USE_TXDATA;
        for (int i = 0; i < length; i++)
        {
            t.tx_data[i] = data[i];
        }
    }
    else
    {
        t.tx_buffer = data;
    }
    gpio_set_level(PIN_DC, 1);
    esp_err_t ret = spi_device_polling_transmit(lcd_handle, &t);
    ESP_ERROR_CHECK(ret);
}

void lcd_draw_rect(spi_device_handle_t lcd_handle, uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end, const uint16_t color)
{
    if ((x_start > x_end) || (y_start > y_end))
        return;
    const uint8_t x_buf[4] = {(x_start >> 8) & 0xFF, x_start & 0xFF, (x_end >> 8) & 0xFF, x_end & 0xFF};
    const uint8_t y_buf[4] = {(y_start >> 8) & 0xFF, y_start & 0xFF, (y_end >> 8) & 0xFF, y_end & 0xFF};
    spi_device_acquire_bus(lcd_handle, portMAX_DELAY);

    lcd_send_cmd(lcd_handle, LCD_CMD_CASET);
    lcd_send_data(lcd_handle, x_buf, 4);
    lcd_send_cmd(lcd_handle, LCD_CMD_RASET);
    lcd_send_data(lcd_handle, y_buf, 4);
    size_t color_buf_size = 2 * (x_end - x_start + 1) * (y_end - y_start + 1);
    uint8_t *color_buf = heap_caps_malloc(color_buf_size, MALLOC_CAP_DMA);
    for (int i = 0; i < color_buf_size; i += 2)
    {
        color_buf[i] = (uint8_t)((color >> 8) & 0xFF);
        color_buf[i + 1] = (uint8_t)(color & 0xFF);
    }
    lcd_send_cmd(lcd_handle, LCD_CMD_RAMWR);
    lcd_send_data(lcd_handle, color_buf, (uint16_t)color_buf_size);
    free(color_buf);
    lcd_send_cmd(lcd_handle, LCD_CMD_NOP);
    spi_device_release_bus(lcd_handle);
    vTaskDelay(1);
}

void lcd_init()
{
    gpio_init();
    spi_init();

    gpio_set_level(PIN_RST, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_RST, 1);
    ESP_LOGI(TAG, "Hardware reset done");
    spi_device_acquire_bus(lcd_handle, portMAX_DELAY);

    lcd_send_cmd(lcd_handle, LCD_CMD_SWRESET); // Software Reset
    vTaskDelay(200 / portTICK_PERIOD_MS);      // At least 120ms delay after SWRST if in SLPIN
    lcd_send_cmd(lcd_handle, LCD_CMD_SLPOUT);  // Bring LCD out of sleep mode
    vTaskDelay(20 / portTICK_PERIOD_MS);       // At least 5ms delay after SLPOUT
    lcd_send_cmd(lcd_handle, LCD_CMD_MADCTL);  // Render mode
    const uint8_t madctl_buf = 0x28;           // Define buffer to pass pointer
    lcd_send_data(lcd_handle, &madctl_buf, 1); // Top-to-bottom, left-to-right, swap xy, BGR
    lcd_send_cmd(lcd_handle, LCD_CMD_COLMOD);  // Color mode
    const uint8_t colmod_buf = 0x55;           // Define buffer to pass pointer
    lcd_send_data(lcd_handle, &colmod_buf, 1); // RGB 16 bit (565)
    lcd_send_cmd(lcd_handle, LCD_CMD_NORON);   // Normal display mode
    lcd_send_cmd(lcd_handle, LCD_CMD_DISPON);  // Turn on display

    spi_device_release_bus(lcd_handle);
    ESP_LOGI(TAG, "LCD initialized");
}