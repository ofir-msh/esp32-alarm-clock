#include <esp_log.h>
// #include "wifi_sntp.h"
#include "lcd_spi.h"

void app_main(void)
{
    // wifi_sntp_start();
    lcd_init();
    const uint16_t colors[] = {
        LCD_COLOR_BLACK,
        LCD_COLOR_BLUE,
        LCD_COLOR_CYAN,
        LCD_COLOR_GREEN,
        LCD_COLOR_MAGENTA,
        LCD_COLOR_ORANGE,
        LCD_COLOR_RED,
        LCD_COLOR_WHITE,
        LCD_COLOR_YELLOW};
    uint8_t x_gap = LCD_DISP_H_RES / 3, y_gap = LCD_DISP_V_RES / 3;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            uint16_t x_start = j * x_gap;
            uint16_t x_end = x_start + x_gap - 1;
            uint16_t y_start = i * y_gap;
            uint16_t y_end = y_start + y_gap - 1;
            lcd_draw_rect(lcd_handle, x_start, x_end, y_start, y_end, colors[3 * i + j]);
        }
    }
    ESP_LOGI("LCD", "Finished render");
}
