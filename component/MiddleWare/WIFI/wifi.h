#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

void wifi_sta_init(void);
void wifi_scan(void);
void wifi_connect(const char* ssid,const char* password);
void wifiTask_Start(void);

/* 查询当前是否已连上 WiFi 并拿到 IP（供 MQTT 等其它模块查询） */
bool wifi_is_connected(void);

#endif