#include "mqtt.h"

#include "can.h"   /* 收到命令要转成 CAN 帧发给 A/B，故依赖 CAN 模块 */

#include <assert.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "mqtt_client.h"
#include "esp_mac.h"
/* ============================================================
 * 【MQTT 客户端封装】—— 收发消息
 *
 * esp-mqtt 的 client 内部自带一个任务，负责：
 *   keepalive 心跳、断线后自动重连 broker、把事件投进回调。
 * 所以本模块不需要自己的 while(1) 循环，只要 mqtt_init() 把
 * client 建好并 start，之后一切由内部任务照看。
 *
 * 回调(mqtt_event_handler)运行在 client 内部任务的上下文里，
 * 所以别在里面做耗时操作：收到数据要么当场处理，要么丢进队列。
 * ============================================================ */

static const char *TAG = "MQTT";

/* ---------- 连接参数 ---------- */
#define MQTT_URI        "mqtt://broker.emqx.io:1883"   /* broker 地址，IP 或域名 */
#define MQTT_USERNAME   NULL                           /* 用户名，无认证填 NULL */
#define MQTT_PASSWORD   NULL                           /* 密码，无认证填 NULL */
#define MQTT_CLIENT_ID  "mqttx_6ff276f0"                 /* 客户端 ID，同一 broker 内要唯一 */

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;

/**
 * @brief  MQTT 事件回调（运行在 client 内部任务）
 */
static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "已连接 broker");
        /* 连上后立刻订阅命令通配主题，之后对端发的消息才收得到。
         * 用 QoS 1：控制命令不能丢（该停的电机没停会出事） */
        mqtt_subscribe(TOPIC_CMD_SUB, MQTT_QOS_CMD);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        /* 不用手动重连：client 内部任务会自动重试。
         * 如果是 WiFi 断了，等 WiFi 恢复后自然连上 */
        ESP_LOGW(TAG, "与 broker 断开，等待自动重连...");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "订阅成功, msg_id=%d", e->msg_id);
        break;

    case MQTT_EVENT_DATA:
        /* 收到一条消息：e->topic 是主题，e->data 是负载。
         * %.*s 按长度打印，因为负载不一定以 '\0' 结尾 */
        ESP_LOGI(TAG, "收到 [%.*s] : %.*s",
                 e->topic_len, e->topic,
                 e->data_len, e->data);

        /* 云端下发的控制命令 → 转成 CAN 帧发给 A/B 节点。
         * e->topic/e->data 都是"指针+长度"，不保证以 '\0' 结尾，
         * 所以连长度一起交给 can_bridge_mqtt，由它自己拷进本地缓冲。
         *
         * 注意：这一句跑在 client 内部任务里，而 can_send 等待总线是带
         * 超时的（最长 50ms）。控制命令很稀疏，阻塞这点时间无所谓。
         * 如果以后命令密集到影响 keepalive，就把这里改成"塞进队列"，
         * 让 CAN 的发送任务去取 —— 回调里只做搬数据，不做等待。 */
        can_bridge_mqtt(e->topic, e->topic_len, e->data, e->data_len);
        break;

    case MQTT_EVENT_ERROR:
        /* e->error_handle->error_type 指出错误类别 */
        if (e->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGE(TAG, "TCP 传输错误（broker 不可达或 WiFi 断）");
        }
        break;

    default:
        break;
    }
}

void mqtt_init(void)
{
    static char s_client_id[32];
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_client_id, sizeof(s_client_id), "esp32s3_%02x%02x%02x", mac[3], mac[4], mac[5]);

    /* v5.x 的配置结构：broker 地址用完整 URI 最省事 */
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri                   = MQTT_URI,
        .credentials.username                 = MQTT_USERNAME,
        .credentials.authentication.password  = MQTT_PASSWORD,
        .credentials.client_id                = s_client_id,
    };

    s_client = esp_mqtt_client_init(&cfg);
    assert(s_client);

    /* 注册回调：所有事件都交给 mqtt_event_handler */
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    /* start 只启动内部任务、发起第一次连接，立即返回，不阻塞 */
    esp_mqtt_client_start(s_client);
}

void mqtt_publish(const char *topic, const char *payload, int qos)
{
    if (s_client == NULL || !s_connected)
    {
        ESP_LOGW(TAG, "未连接，丢弃发布 [%s]", topic);
        return;
    }
    /* len=0 表示按 '\0' 结尾的字符串计算长度；retain=0 不保留 */
    esp_mqtt_client_publish(s_client, topic, payload, 0, qos, 0);
}

void mqtt_subscribe(const char *topic, int qos)
{
    if (s_client == NULL)
    {
        return;
    }
    esp_mqtt_client_subscribe(s_client, topic, qos);
}

bool mqtt_is_connected(void)
{
    return s_connected;
}
