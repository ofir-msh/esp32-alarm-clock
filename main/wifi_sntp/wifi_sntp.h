#ifndef MAIN_WIFI_SNTP_H_
#define MAIN_WIFI_SNTP_H_

#include <string.h>
#include <esp_wifi.h>

#define WIFI_STA_SSID "Mashiach"
#define WIFI_STA_PASSWORD "a1234b1234"

void wifi_init();
void sntp_init();
void wifi_sntp_start();

#endif // MAIN_WIFI_SNTP_H_