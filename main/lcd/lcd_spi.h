#ifndef MAIN_LCD_SPI_H_
#define MAIN_LCD_SPI_H_

#include <driver/gpio.h>
#include <driver/spi_master.h>

#define PIN_MOSI GPIO_NUM_23
#define PIN_MISO GPIO_NUM_19
#define PIN_CLK GPIO_NUM_18
#define PIN_CS GPIO_NUM_5
#define PIN_RST GPIO_NUM_4
#define PIN_DC GPIO_NUM_22

#define LCD_DISP_V_RES 320
#define LCD_DISP_H_RES 480
#define LCD_RENDER_GAP 80

#define LCD_CMD_NOP 0x00     // No operation
#define LCD_CMD_SWRESET 0x01 // Software Reset, must wait 5ms after reset
#define LCD_CMD_SLPOUT 0x11  // Bring LCD out of sleep
#define LCD_CMD_NORON 0x13   // Normal display mode ON
#define LCD_CMD_INVOFF 0x20  // Display Inversion Off
#define LCD_CMD_INVON 0x21   // Display Inversion On
#define LCD_CMD_DISPOFF 0x28 // Display ON
#define LCD_CMD_DISPON 0x29  // Display OFF
#define LCD_CMD_CASET 0x2A   // Column Address
#define LCD_CMD_RASET 0x2B   // Row Address
#define LCD_CMD_WRDISBV 0x51 // Write display brightness
#define LCD_CMD_RRDISBV 0x52 // Read display brightness
#define LCD_CMD_MADCTL 0x36  // Render direction
#define LCD_CMD_COLMOD 0x3A  // Interface Pixel Format - number of bits per pixel
#define LCD_CMD_RAMWR 0x2C   // Write to memory

#define LCD_COLOR_BLACK 0x0000
#define LCD_COLOR_WHITE 0xFFFF
#define LCD_COLOR_RED 0xF800
#define LCD_COLOR_GREEN 0x07E0
#define LCD_COLOR_BLUE 0x001F
#define LCD_COLOR_CYAN 0x07FF
#define LCD_COLOR_MAGENTA 0xF81F
#define LCD_COLOR_YELLOW 0xFFE0
#define LCD_COLOR_ORANGE 0xFC00

extern spi_device_handle_t lcd_handle;
void lcd_init();
void lcd_send_cmd(spi_device_handle_t lcd_handle, const uint8_t cmd);
void lcd_send_data(spi_device_handle_t lcd_handle, const uint8_t *data, uint16_t length);
void lcd_draw_rect(spi_device_handle_t lcd_handle, uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end, const uint16_t color);

#endif // MAIN_LCD_SPI_H_