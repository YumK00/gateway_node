#include "led.h"
#include "wifi.h"
#include "can.h"
void app_main(void)
{
    LED_Init();          /* 硬件先初始化 */
    LedTask_Start();     /* 再启动任务 */

    can_init();          /* CAN：启动收帧任务 + 周期上报任务 */

    wifi_sta_init();     /* WiFi 先初始化（内部会阻塞到连上或超时） */
    wifiTask_Start();    /* 再启动网络任务：等 WiFi → 起 MQTT → 周期上报 */
}