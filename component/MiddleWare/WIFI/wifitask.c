#include "wifi.h"
#include "mqtt.h"
#include "can.h"    /* can_get_stats()：CAN 侧的运行统计在这里取 */

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

/* 报 CAN 侧四个"开机至今"的累计值，走单独一条主题。
 *
 * 【为什么不并进上面那条 gateway/status】
 *   那条的 payload 只有 64 字节。四个 uint32 最坏各 10 位数字，
 *   拼起来要 84 字节 —— snprintf 遇到装不下【不报错，直接截断】，
 *   发出去是个缺尾巴的 JSON，解析端报语法错，看着像网络问题。
 *   单开一条主题，缓冲自己定，也不用去动已经跑通的那三个字段。
 *
 * 想算丢包率：a_lost / (a_lost + a_rx)、pub_lost / pub_total。
 * 口径和坑见 can.h 的注释（尤其重复帧和 A 重启那两条）。 */
static void publish_gw_stats(void)
{
    can_stats_t st = can_get_stats();

    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"a_rx\":%u,\"a_lost\":%u,\"pub_total\":%u,\"pub_lost\":%u}",
             (unsigned)st.a_rx,   (unsigned)st.a_lost,
             (unsigned)st.pub_total, (unsigned)st.pub_lost);
    mqtt_publish(TOPIC_STATUS_STATS, payload, MQTT_QOS_TELEMETRY);
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
        publish_gw_stats();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void wifiTask_Start(void)
{
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 5, NULL);
}
