#include "wifi.h"
#include "mqtt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

/* ============================================================
 * 【网络任务】—— 等 WiFi → 启动 MQTT → 周期上报
 *
 * 为什么这里只有"等 WiFi + 启动一次 + 周期发布"：
 *   MQTT 客户端自带内部任务，负责 keepalive、断线重连 broker，
 *   收到消息走 mqtt_event_handler 回调，不用在这里轮询。
 *   所以本任务只需把 MQTT 拉起来，再定期发自己的数据。
 * ============================================================ */

static const char *TAG = "WIFITASK";

/* 周期上报网关自身状态，对应 CAN 的 0x103 GW_STATUS。
 * A/B 节点的遥测由 CAN 收帧那边负责发，不在这里。 */
static void publish_gw_status(void)
{
    char payload[64];
    snprintf(payload, sizeof(payload),
             "{\"wifi\":%d,\"mqtt\":%d,\"cache\":0}",
             wifi_is_connected() ? 1 : 0,
             mqtt_is_connected() ? 1 : 0);
    mqtt_publish(TOPIC_STATUS_GW, payload, MQTT_QOS_TELEMETRY);
}

/**
 * @brief  网络任务函数
 */
void wifi_task(void *arg)
{
    /* 第一步：等 WiFi 连上。没 IP 就去连 broker 只会立刻失败 */
    while (!wifi_is_connected())
    {
        ESP_LOGI(TAG, "等待 WiFi 连接...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "WiFi 已就绪，启动 MQTT");

    /* 第二步：启动 MQTT 客户端（内部任务会去连 broker 并自动维护） */
    mqtt_init();

    /* 第三步：周期上报。接收走回调，不在这里轮询 */
    for (;;)
    {
        publish_gw_status();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void wifiTask_Start(void)
{
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 5, NULL);
}
