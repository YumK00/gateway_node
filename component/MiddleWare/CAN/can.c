#include "can.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"   /* 遥测发布走队列解耦，见 pub_telemetry */

#include "esp_log.h"
#include "esp_err.h"

#include "driver/twai.h"

#include "protocol.h"   /* 协议契约：ID、DLC、字节布局（和 STM32 共用同一份） */
#include "mqtt.h"       /* 收帧后要往 MQTT 上转发 */
#include "wifi.h"       /* GW_STATUS 里要带上 WiFi 状态 */

/* ============================================================
 * 【CAN 网关：一侧 CAN，一侧 MQTT】
 *
 * 本文件负责三件事，对应三个去向：
 *
 *   ① CAN 收到 → MQTT 发出     can_rx_task  收帧后按 ID 分发、打包成 JSON
 *   ② MQTT 收到 → CAN 发出     can_bridge_mqtt  由 mqtt.c 的回调调用
 *   ③ 网关自己周期上报          can_tx_task  每秒发 0x103 状态 + 0x703 心跳
 *
 * 为什么收帧用"轮询"而不是中断：
 *   TWAI 驱动内部已经把中断→队列这条链做好了，twai_receive() 就是
 *   从这个队列取帧，取不到就阻塞。所以任务里写一个 while(1) 即可，
 *   不用自己碰中断。这也符合 FreeRTOS 的常规写法：
 *   "一层中断只负责把数据丢进队列，重活留给任务"。
 *
 * 遥测怎么发出去：走【队列解耦】。收帧/发送任务只把数据拷进队列就返回
 *   （永不阻塞），真正调 mqtt_publish 的只有 can_pub_task 一个任务。
 *   原因见下面"遥测发布（队列解耦）"那一节 —— 这不是优化，是踩过坑。
 *
 * 任务优先级：can_rx  = 6（比 led/wifi 的 5 高一点，先把帧收进来），
 *             can_tx  = 4（周期上报，不急），
 *             can_pub = 3（垫底：它是唯一会阻塞在 MQTT 上的任务，
 *                         见 can_pub_task 上面那段说明）。
 * ============================================================ */

static const char *TAG = "CAN";

/* ==================== 硬件与总线参数 ==================== */

/* ★★ 引脚 ★★ S3 的 TWAI 走 GPIO 交换矩阵，任意空闲 GPIO 都能用。
 * 这两个脚和 USB-Serial-JTAG(19/20)、控制台串口(43/44)、
 * Strapping(0/3/45/46)、彩灯(48) 都不冲突。
 * GPIO4/5 同时也是 ADC1_CH3/CH4，本项目没用到 ADC。 */
#define CAN_TX_GPIO         GPIO_NUM_5  /* → 收发器 TXD */
#define CAN_RX_GPIO         GPIO_NUM_4  /* ← 收发器 RXD */

/* ★★ 工作模式 ★★
 *   TWAI_MODE_NORMAL      正常模式：能收能发，还要给别人回 ACK。【正式使用】
 *   TWAI_MODE_LISTEN_ONLY 只听：不参与总线（不发、也不回 ACK）。【单独测接收用】
 *
 * 调试时用 LISTEN_ONLY 有个大好处：它完全不影响总线，所以
 * 不管自己的 TX 那根线有没有接好，只要 RX 通路是通的就能收到数据 ——
 * 这样能把"收"和"发"分开验证。
 * 但注意它不能发，所以 can_tx_task 开着也没用（twai_transmit 会直接报不支持）。 */
#define CAN_MODE            TWAI_MODE_NORMAL

#define CAN_RX_TIMEOUT_MS   100     /* twai_receive 的等待上限；超时就顺手查一下总线状态 */
#define CAN_TX_PERIOD_MS    1000    /* 网关自身周期帧(0x103 + 0x703)的发送周期 */
#define CAN_SEND_TIMEOUT_MS 50      /* 发送时等待总线空闲的最长时间 */
#define CAN_HB_TIMEOUT_MS   3000    /* 超过这么久没收到某节点心跳 → 判定掉线 */

/* A 单次 seq 跳变 >= 这个数，就判为"疑似重启/长时间断线"，不计入丢包。
 * 依据：A 每 100ms 一帧，10 ≈ 1 秒的量 —— 正常丢帧是一次丢一两帧
 * （仲裁/误码），不会一口气跳 10 个。见 can.h 里那条口径说明。 */
#define A_LOST_RESET_MAX    10

/* ==================== 本模块的状态 ==================== */

/* 控制帧的命令序号。每下发一帧 +1，接收端(执行节点)记着上一次的值，
 * 相同就丢弃 —— 用来对付 CAN 硬件自动重发导致的重复帧，见 protocol.h。 */
static uint8_t s_cid = 0;

/* 收到的帧数。只在 can_rx_task 里 ++、can_tx_task 里读，
 * 32 位读写本身就是原子的，不需要加锁。 */
static uint32_t s_rx_count = 0;

/* A_SENSOR 的序列号，用来算丢帧（protocol.h 的 PROTO_SeqLost） */
static uint8_t s_last_seq = 0;
static bool    s_seq_valid = false;   /* 第一次收到时没有"上一帧"可比，先跳过 */

/* BUS_OFF 累计次数。只用来给日志限流：
 * 总线没接好时，"离线→恢复→重启→又离线"这个循环每秒能跑好几个来回，
 * 每次都打日志会把串口刷爆。所以只在第 1 次和之后每 10 次打一条。 */
static uint32_t s_busoff_count = 0;

/* ==================== 运行统计的四个计数器 ====================
 * 语义见 can.h（can_stats_t / can_get_stats），都是开机至今的累计值。
 *
 * 两种写者情况，所以声明方式不一样：
 *   s_a_*   只有 can_rx_task 写 → 不用 volatile（和上面的 s_rx_count 同理）
 *   s_pub_*  can_rx_task 和 can_tx_task 都会写 → volatile，
 *            和下面的 s_pub_drop 一个取舍：两条任务偶尔同时 ++ 会少算
 *            一条，但这是监控数不是账目，不值得为它上锁。 */
static uint32_t s_a_rx      = 0;   /* 收到的 A 数据帧(0x101)总数 */
static uint32_t s_a_lost    = 0;   /* A 数据帧丢失总数（seq 推算） */
static volatile uint32_t s_pub_total = 0;  /* MQTT 发布总数（进过 pub_telemetry 的） */
static volatile uint32_t s_pub_lost  = 0;  /* MQTT 丢包总数（三条丢法都算） */

/* ==================== 收帧日志的节流 ====================
 *
 * 【为什么需要节流】
 * A/B 的周期数据是 10Hz，也就是每秒 20 条。如果每帧都打一条 INFO，
 * 串口会被刷得飞快，不但看不清楚，还会把 MQTT/WiFi 的关键日志冲走。
 * 但完全不打又没法确认"链路到底通没通"。
 *
 * 【做法】按"帧类型"各自限流：同一类帧 CAN_LOG_PERIOD_MS 内只打第一条。
 * 这样每类帧最多 1 条/秒，一眼就能看出各类数据是否都在正常更新。
 *
 * 【怎么调】
 *   想逐帧看（调试初期很有用）：把 CAN_LOG_PERIOD_MS 改成 0
 *   想更安静：改大（比如 5000）
 *   彻底不要：把下面几处 ESP_LOGI 换回 ESP_LOGD
 *
 * 【为什么不用 ESP_LOGI 就完事】
 * 日志级别是"全局开关"，管不了"打多快"。这类按频率限流的需求
 * 得自己在应用层做 —— 在嵌入式里这是很常见的一招。
 */
#define CAN_LOG_PERIOD_MS   1000

/* 每个"日志槽位"对应一类帧，各自记时间、互不影响 */
typedef enum {
    LOGID_A_SENSOR = 0,   /* 0x101 A 节点传感器 */
    LOGID_B_STATUS,       /* 0x102 B 节点状态   */
    LOGID_HB_A,           /* 0x701 A 节点心跳   */
    LOGID_HB_B,           /* 0x702 B 节点心跳   */
    LOGID_OTHER,          /* 其它没专门处理的帧 */
    LOGID_COUNT
} can_logid_t;

static TickType_t s_log_last[LOGID_COUNT];
static bool       s_log_seen[LOGID_COUNT];

/* 这一类帧现在可以打日志吗？可以就顺手记下时刻。
 * seen 标志是为了让"上电后第一条"一定能打出来 ——
 * 否则 tick 还不到 CAN_LOG_PERIOD_MS 时，now - 0 太小会被误判成"刚打过"。 */
static bool can_log_allow(can_logid_t id)
{
    if (CAN_LOG_PERIOD_MS > 0) {
        TickType_t now = xTaskGetTickCount();
        if (s_log_seen[id] &&
            (now - s_log_last[id]) < pdMS_TO_TICKS(CAN_LOG_PERIOD_MS)) {
            return false;
        }
        s_log_last[id] = now;
        s_log_seen[id] = true;
    }
    return true;
}

/* ---------- 每个被监视节点的状态 ----------
 * 网关只关心 A/B 两个节点：它们有没有在发心跳。
 * 掉线了就往对应的 alert 主题报一条 ALM_COMM_LOST。
 *
 * seen  : 是否收到过有效心跳。没收到过不算掉线，否则一上电就误报。
 * online: 当前是否在线。只在状态"变化"时才上报，避免每秒刷一条。 */
typedef struct {
    const char *name;         /* "A" / "B"，只用于打日志 */
    can_logid_t log_id;       /* 心跳日志用哪个节流槽位 */
    uint32_t    hb_id;        /* 该节点的心跳帧 ID */
    uint8_t     hb_magic;     /* 该节点心跳的魔数 */
    const char *alert_topic;  /* 掉线时往哪个 MQTT 主题报警 */
    TickType_t  last_seen;    /* 最后一次收到有效心跳的时刻 */
    bool        seen;
    bool        online;
} node_watch_t;

static node_watch_t s_nodes[2] = {
    { "A", LOGID_HB_A, CAN_ID_A_HEARTBEAT, HB_MAGIC_A, TOPIC_ALERT_A, 0, false, false },
    { "B", LOGID_HB_B, CAN_ID_B_HEARTBEAT, HB_MAGIC_B, TOPIC_ALERT_B, 0, false, false },
};

/* ==================== 小工具函数 ==================== */

/* 节点状态 → 中文，只给日志用 */
static const char *node_state_str(const node_watch_t *n)
{
    if (n->online) {
        return "在线";
    }
    return n->seen ? "掉线" : "无信号";   /* 无信号 = 从头到尾没冒过泡 */
}

/* float → "扩大 100 倍的 u16"（PID 参数的线上格式，见 protocol.h）。
 * 顺手做上下限保护：负数或者超过 655.35 直接夹住，
 * 不然 float 转无符号是未定义行为，可能变成一个莫名其妙的大数。 */
static uint16_t to_u16_x100(float v)
{
    if (v < 0.0f) {
        v = 0.0f;
    }
    v = v * 100.0f + 0.5f;          /* +0.5 是四舍五入，直接截断会一直偏小 */
    if (v > 65535.0f) {
        v = 65535.0f;
    }
    return (uint16_t)v;
}

/* ==================== 遥测发布（队列解耦） ====================
 *
 * 【为什么不能就地发】
 *   esp_mqtt_client_publish() 不是"丢进队列就返回"。它在【调用者的任务】
 *   里同步写 socket，而且全程握着 mqtt 的 API 锁：
 *
 *     can_rx_task
 *       └─ pub_telemetry(本文件)
 *          └─ mqtt_publish(mqtt.c)
 *             └─ esp_mqtt_client_publish(mqtt_client.c:2065)
 *                ├─ MQTT_API_LOCK          (2078，xSemaphoreTakeRecursive)
 *                └─ esp_mqtt_write         (2134)
 *                   └─ esp_transport_write  (704)
 *                      └─ tcp_write         (transport_ssl.c:248)
 *                         └─ select(...,10 秒)   ← 卡死点
 *
 *   这一整摞都在 can_rx_task 的栈上。网络一卡(WiFi 断、broker 不回 ACK)，
 *   最长要等 network_timeout_ms 才返回，默认 10000 —— 见 mqtt_config.h
 *   的 MQTT_NETWORK_TIMEOUT_MS。
 *
 * 【就地发的后果】—— 这不是假想，是真踩过的：
 *   · can_rx_task 卡死 10 秒。它是唯一调 twai_receive() 的任务，
 *     这 10 秒里没人排空 TWAI 接收队列，总线上的帧静默丢弃。
 *   · can_tx_task 紧接着卡在同一把锁上 7 秒 → 0x703 网关心跳停发
 *     → A/B 判网关掉线，B 的电机跟着停。
 *   结果就是网络抖一下，整条 CAN 侧陪着一起装死。
 *
 * 【所以】本函数只做一次内存拷贝就返回，真正发送交给 can_pub_task。
 *   网络再卡也只卡住那一个任务，CAN 收帧/发帧/判活全部不受影响。 */

/* topic 存指针就够：调用点传进来的都是 mqtt.h 里的字面量宏，静态存储。
 * payload 必须拷：它是调用点栈上的临时缓冲，出了作用域就没了。 */
typedef struct {
    const char *topic;
    char        payload[96];   /* 和 can_dispatch() 里的 payload[96] 对齐 */
} pub_item_t;

#define PUB_QUEUE_LEN     8     /* 够缓冲约一秒的 10Hz 遥测，再多只是延迟更大 */
#define CAN_PUB_TASK_PRIO 3     /* 低于 can_tx(4)：CAN 的周期帧和判活更优先 */
#define PUB_DROP_LOG_MS   1000u /* 丢包日志最多一秒一行，别刷屏 */

static QueueHandle_t     s_pub_q    = NULL;
/* 因队列满而丢掉的条数。只是统计，两个任务同时 ++ 偶尔少算一两条没关系，
 * 不用为它上锁。 */
static volatile uint32_t s_pub_drop = 0;

/* 往 MQTT 发遥测；没连上 broker 就直接丢掉。
 * 为什么不像 mqtt_publish 那样打警告：这里是 10Hz 的周期数据，
 * WiFi 断开的十几秒里会刷几百条警告，把真正的日志冲掉。
 * 丢一帧遥测无所谓，下一帧马上就来。 */
static void pub_telemetry(const char *topic, const char *payload)
{
    /* 进这个函数就算"本该发出去"一条，pub_total 从这里起算。
     * 下面三条丢法都发生在这之后，所以 丢包率 = pub_lost / pub_total。 */
    s_pub_total++;

    if (!mqtt_is_connected()) {
        s_pub_lost++;          /* ① 没连上 broker，直接丢 */
        return;
    }

    pub_item_t item;
    item.topic = topic;
    strncpy(item.payload, payload, sizeof(item.payload) - 1);
    item.payload[sizeof(item.payload) - 1] = '\0';

    /* ★ 第三个参数 0 = 队满时【不等】★
     * 这是整个改动的关键：can_rx_task / can_tx_task 在这一行永不阻塞。
     * 队满说明 MQTT 那边正卡着，这时丢一帧遥测，远好过把 CAN 拖住。 */
    if (xQueueSend(s_pub_q, &item, 0) != pdTRUE) {
        s_pub_drop++;          /* 队满，只给下面那条日志用 */
        s_pub_lost++;          /* ② 队满丢掉 */
    }
}

/* ==================== 遥测发布任务 ====================
 * 全工程唯一会阻塞在 MQTT 上的地方就是这里。
 * 它卡住最多让遥测延迟几秒、丢几帧，碰不到 CAN 收发和 0x703 心跳。 */
static void can_pub_task(void *arg)
{
    pub_item_t item;
    uint32_t   last_drop   = 0;
    TickType_t last_report = 0;

    while (1) {
        /* 没有别的活要干，就睡到有数据为止，不空转 */
        if (xQueueReceive(s_pub_q, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* ★★ 阻塞点就在这一句 ★★
         * 最长 network_timeout_ms(默认 10 秒)才返回。
         * 但此刻被卡的只有本任务 —— 这正是整个改动的目的。 */
        if (!mqtt_publish(item.topic, item.payload, MQTT_QOS_TELEMETRY)) {
            /* ③ 出了队但没发出去：入队时还连着，这会儿断了，
             *    或者 esp-mqtt 内部失败（缓冲区写不动 / outbox 超限）。 */
            s_pub_lost++;
        }

        /* 队列满导致的丢包，攒够 1 秒报一行 */
        TickType_t now = xTaskGetTickCount();
        if (s_pub_drop != last_drop
            && (now - last_report) >= pdMS_TO_TICKS(PUB_DROP_LOG_MS)) {
            ESP_LOGW(TAG, "遥测队列满，丢弃 %u 条 (MQTT 忙或断线，CAN 不受影响)",
                     (unsigned)(s_pub_drop - last_drop));
            last_drop   = s_pub_drop;
            last_report = now;
        }
    }
}

/* ==================== 发送一帧 ==================== */

void can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    twai_message_t msg = {
        .identifier       = id,
        .data_length_code = len,
        .extd             = 0,   /* 0 = 标准帧(11 位 ID)。协议规定用标准帧 */
        .rtr              = 0,   /* 0 = 数据帧，不是"远程请求帧" */
        .ss               = 0,   /* 0 = 发送失败时由硬件自动重发（默认行为） */
        .self             = 0,   /* 0 = 不自收，正常收发 */
    };

    if (len > sizeof(msg.data)) {
        ESP_LOGE(TAG, "DLC=%u 超过 8，丢弃 (id=0x%03X)", len, (unsigned)id);
        return;
    }
    if (len > 0 && data != NULL) {
        memcpy(msg.data, data, len);
    }

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(CAN_SEND_TIMEOUT_MS));
    if (err == ESP_OK) {
        /* 用 DEBUG 级别：周期帧每秒两发，打到 INFO 会把日志刷满。
         * 想在串口看到它们，把 log level 调到 Debug 即可。 */
        ESP_LOGD(TAG, "TX 0x%03X DLC=%u", (unsigned)id, len);
    } else if (err == ESP_ERR_TIMEOUT) {
        /* 最常见的两种情况：总线没人 ACK（对端没上电/没接收发器/没接终端电阻），
         * 或者正在 BUS_OFF 恢复中。 */
        ESP_LOGW(TAG, "TX 0x%03X 超时：总线忙或无人应答(查供电/收发器/120Ω 电阻)",
                 (unsigned)id);
    } else {
        ESP_LOGE(TAG, "TX 0x%03X 失败: %s", (unsigned)id, esp_err_to_name(err));
    }
}

/* ==================== 网关自己的周期帧 ==================== */

static void can_send_periodic(void)
{
    uint8_t d[8];
    static uint8_t hb_cnt = 0;   /* 心跳计数器，每发一帧 +1，0~255 自动回绕 */

    /* 0x103 GW_STATUS：把网关的网况广播给总线上其他节点。
     * 注意这和 MQTT 上发的 gw01/gateway/status 是【同一件事的两个出口】：
     *   CAN 侧 → 给 A/B 节点看（它们要据此判断联动是否还可靠）
     *   MQTT 侧 → 给云端看（在 wifitask.c 里发） */
    PROTO_PackGwStatus(d,
                       wifi_is_connected() ? 1 : 0,
                       mqtt_is_connected() ? 1 : 0,
                       12345);   /* cache_cnt：本项目还没做断网缓存，先填12345  */
    can_send(CAN_ID_GW_STATUS, d, LEN_GW_STATUS);

    /* 0x703 GW_HEARTBEAT：让 A/B 知道"网关还活着"。
     * 魔数 0xC3 是网关专属，收端先核对魔数再认这条心跳。
     *
     * ★ 这条心跳同时兼任 B 的【电机保活】★ —— B 的电机转着的时候，
     *   3 秒收不到它就停车(见 22222/Core/Src/freertos.c 的 g_gw_hb_tick)。
     *   所以这里【不要】再另加一套"每秒重发最后一条电机命令"的机制：
     *   0x703 本来就是每秒无条件发的，重发纯属多余；而且那会逼着 cid
     *   兼职当保活 —— cid 的本职只是去重。 */
    PROTO_PackHeartbeat(d, HB_MAGIC_GW, hb_cnt++);
    can_send(CAN_ID_GW_HEARTBEAT, d, LEN_HEARTBEAT);
}

/* ==================== 心跳超时检测 ==================== */

static void can_check_nodes(void)
{
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < 2; i++) {
        node_watch_t *n = &s_nodes[i];

        /* 没收到过心跳 → 不判掉线（上电初期 A/B 可能还没起来） */
        if (!n->seen || !n->online) {
            continue;
        }

        /* 用无符号相减判断超时，这样 tick 计数回绕（约 49 天一次）也不会误判。
         * 直接写 now > last_seen + TIMEOUT 在回绕时会出错。 */
        if ((now - n->last_seen) < pdMS_TO_TICKS(CAN_HB_TIMEOUT_MS)) {
            continue;
        }

        n->online = false;
        ESP_LOGW(TAG, "节点 %s 心跳超时(%d ms 没消息)，判定掉线",
                 n->name, CAN_HB_TIMEOUT_MS);

        /* 按协议报 ALM_COMM_LOST。extra 字段这里放 0，
         * 想更有用的话可以放"当时已经多久没收到"。 */
        char payload[64];
        snprintf(payload, sizeof(payload),
                 "{\"code\":%u,\"extra\":0}", (unsigned)ALM_COMM_LOST);
        pub_telemetry(n->alert_topic, payload);
    }
}

/* ==================== 收到心跳 ==================== */

static void can_on_heartbeat(node_watch_t *n, const twai_message_t *msg)
{
    /* DLC 不对 → 不是按本协议发的心跳，直接忽略 */
    if (msg->data_length_code != LEN_HEARTBEAT) {
        return;
    }

    /* 魔数不对 → ID 撞上了但内容不是心跳（脏数据/固件版本不一致） */
    if (msg->data[0] != n->hb_magic) {
        ESP_LOGW(TAG, "节点 %s 心跳魔数不符(收到 0x%02X，应为 0x%02X)，忽略",
                 n->name, msg->data[0], n->hb_magic);
        return;
    }

    n->last_seen = xTaskGetTickCount();
    n->seen = true;

    if (!n->online) {   /* 只在"离线→在线"这一刻打印，这是状态变化，不能漏 */
        n->online = true;
        ESP_LOGI(TAG, "节点 %s 上线", n->name);
    }

    /* 心跳本身按节流打：cnt 每次 +1，能一眼看出对方还在不在稳定发。
     * 如果 cnt 长时间不动，说明对方卡住了（虽然帧还在重发）。 */
    if (can_log_allow(n->log_id)) {
        //ESP_LOGI(TAG, "RX 0x%03X 节点%s 心跳 cnt=%u",
        //         (unsigned)msg->identifier, n->name, msg->data[1]);
    }
}

/* ==================== 收帧分发 ==================== */

static void can_dispatch(const twai_message_t *msg)
{
    const uint8_t *d = msg->data;
    char payload[96];

    switch (msg->identifier) {

    /* ---------- 0x101  A 节点传感器数据（周期） ---------- */
    case CAN_ID_A_SENSOR: {
        if (msg->data_length_code != LEN_A_SENSOR) {
            /* 这类错要是持续出现（比如对方固件版本不一致），10Hz 就刷屏了，
             * 所以同样按 A_SENSOR 的槽位节流 */
            if (can_log_allow(LOGID_A_SENSOR)) {
                ESP_LOGW(TAG, "0x%03X DLC=%u 应为 %u，丢弃",
                         (unsigned)msg->identifier, msg->data_length_code, LEN_A_SENSOR);
            }
            break;
        }
        A_Sensor_t s;
        PROTO_UnpackASensor(d, &s);

        /* 用序列号算丢帧：正常情况下 seq 每帧 +1，差值 2 说明中间丢了一帧。
         *
         * ★ 必须先判 seq 变了 ★
         *   seq 和上一帧相同 = 重复帧（CAN 允许硬件重传），不是丢帧。
         *   而 PROTO_SeqLost(x, x) 算出来是 255 不是 0 —— 漏判这一下，
         *   一次重复帧就往 s_a_lost 里灌 255，而且累计值洗不掉。 */
        if (s_seq_valid && s.seq != s_last_seq) {
            uint8_t lost = PROTO_SeqLost(s.seq, s_last_seq);
            if (lost >= A_LOST_RESET_MAX) {
                /* 跳变太大 → 判为 A 重启/长时间断线，不计进丢包。
                 * seq 只有 8 位、帧里又没有 boot_id，这两件事在协议上
                 * 分不开，只能用阈值近似 —— 两个方向都会错，见 can.h。 */
                ESP_LOGW(TAG, "A seq 跳变 %u → %u (%u 帧)，判为疑似重启，不计入丢包",
                         (unsigned)s_last_seq, (unsigned)s.seq, (unsigned)lost);
            } else if (lost > 0) {
                s_a_lost += lost;
                ESP_LOGW(TAG, "A 节点数据丢帧 %u 个 (seq %u → %u)",
                         (unsigned)lost, (unsigned)s_last_seq, (unsigned)s.seq);
            }
        }
        s_last_seq = s.seq;
        s_seq_valid = true;
        s_a_rx++;

        /* 原始 ADC 值直接转成 JSON，不做物理量换算 ——
         * protocol.h 规定协议层只搬"生数据"，换算留给显示端/云端。 */
        snprintf(payload, sizeof(payload),
                 "{\"ir\":%u,\"ldr\":%u,\"seq\":%u}", s.ir, s.ldr, s.seq);
        pub_telemetry(TOPIC_TELEMETRY_A, payload);

        /* 一行把"链路通不通 + 数据长什么样"都说清楚。
         * 丢帧警告在上面是单独的（而且只在真丢帧时才响），
         * 这行是常规心跳式的确认，所以按秒节流。 */
        if (can_log_allow(LOGID_A_SENSOR)) {
            ESP_LOGI(TAG, "RX 0x101 A传感器 ir=%u ldr=%u seq=%u",
                     s.ir, s.ldr, s.seq);
        }
        break;
    }

    /* ---------- 0x102  B 节点状态（周期） ---------- */
    case CAN_ID_B_STATUS: {
        if (msg->data_length_code != LEN_B_STATUS) {
            if (can_log_allow(LOGID_B_STATUS)) {
                ESP_LOGW(TAG, "0x%03X DLC=%u 应为 %u，丢弃",
                         (unsigned)msg->identifier, msg->data_length_code, LEN_B_STATUS);
            }
            break;
        }
        B_Status_t s;
        PROTO_UnpackBStatus(d, &s);

        snprintf(payload, sizeof(payload),
                 "{\"motor\":%u,\"pwm\":%u,\"mode\":%u,\"flags\":%u}",
                 s.motor, s.pwm, s.mode, s.flags);
        pub_telemetry(TOPIC_TELEMETRY_B, payload);

        if (can_log_allow(LOGID_B_STATUS)) {
            ESP_LOGI(TAG, "RX 0x102 B状态 motor=%u pwm=%u mode=%u flags=0x%02X",
                     s.motor, s.pwm, s.mode, s.flags);
        }
        break;
    }

    /* ---------- 0x011 / 0x012  报警（事件触发，出事才发） ---------- */
    case CAN_ID_ALARM_A:
    case CAN_ID_ALARM_B: {
        if (msg->data_length_code != LEN_ALARM) {
            break;
        }
        uint8_t  code  = PROTO_AlarmCode(d);
        uint16_t extra = PROTO_GetU16(d + 1);
        bool     is_a  = (msg->identifier == CAN_ID_ALARM_A);

        snprintf(payload, sizeof(payload),
                 "{\"code\":%u,\"extra\":%u}", code, extra);
        pub_telemetry(is_a ? TOPIC_ALERT_A : TOPIC_ALERT_B, payload);

        /* 报警是稀罕事，用 WARN 级别，保证在默认日志级别下一定看得见 */
        ESP_LOGW(TAG, "★ %s 节点报警 code=%u extra=%u",
                 is_a ? "A" : "B", code, extra);
        break;
    }

    /* ---------- 0x000  广播急停 / 解除 ---------- */
    case CAN_ID_ESTOP: {
        if (msg->data_length_code != LEN_ESTOP) {
            ESP_LOGW(TAG, "0x%03X DLC=%u 应为 %u，丢弃",
                     (unsigned)msg->identifier, msg->data_length_code, LEN_ESTOP);
            break;
        }
        uint8_t action = PROTO_EstopAction(d);

        /* ★ 只认 ESTOP_ACTION_CLEAR 为解除，其余一切值(包括解析不出来的)
         *   都当成急停 ★ —— 安全帧不能有"看不懂就放行"的默认。 */
        if (action == ESTOP_ACTION_CLEAR) {
            ESP_LOGW(TAG, "★ 收到广播急停解除 ESTOP_CLEAR");
        } else {
            ESP_LOGW(TAG, "★ 收到广播急停 ESTOP(action=%u)", action);
        }

        /* A/B 收到会自己执行，网关这边不需要转发，只记一笔。
         * 以后想联动（比如同时点亮红灯、推送云端），在这里加。 */
        break;
    }

    /* ---------- 0x701 / 0x702  节点心跳 ---------- */
    case CAN_ID_A_HEARTBEAT:
        can_on_heartbeat(&s_nodes[0], msg);
        break;

    case CAN_ID_B_HEARTBEAT:
        can_on_heartbeat(&s_nodes[1], msg);
        break;

    default:
        /* 协议里没定义的 ID。调试新节点时这条特别有用 ——
         * 对方要是 ID 拼错了（比如把功能组和节点号搞反），就会掉到这里。
         * 顺便把原始字节打出来，方便手工比对协议表。 */
        if (can_log_allow(LOGID_OTHER)) {
            ESP_LOGI(TAG, "RX 0x%03X DLC=%u 数据=%02X %02X %02X %02X %02X %02X %02X %02X (未定义的 ID)",
                     (unsigned)msg->identifier, msg->data_length_code,
                     d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
        }
        break;
    }
}

/* ==================== MQTT → CAN 桥接 ==================== */

void can_bridge_mqtt(const char *topic, int topic_len,
                     const char *payload, int payload_len)
{
    char t[64];
    char pl[96];
    char node[8];
    char kind[16];
    uint8_t d[8];

    /* ---------- 1. 拷到本地缓冲并补 '\0' ----------
     * esp-mqtt 给的 topic/data 是"指针 + 长度"，不一定以 '\0' 结尾，
     * 直接 strcmp/sscanf 会读到缓冲区外面去。所以先按长度拷贝、自己封口。 */
    if (topic_len <= 0 || topic_len >= (int)sizeof(t)) {
        ESP_LOGW(TAG, "主题长度异常(%d)，忽略", topic_len);
        return;
    }
    memcpy(t, topic, topic_len);
    t[topic_len] = '\0';

    if (payload_len < 0) {
        payload_len = 0;
    }
    if (payload_len > (int)sizeof(pl) - 1) {
        payload_len = (int)sizeof(pl) - 1;   /* 命令不会这么长，超了就是发错了 */
    }
    if (payload_len > 0 && payload != NULL) {
        memcpy(pl, payload, payload_len);
    }
    pl[payload_len] = '\0';

    /* ---------- 2. 拆主题：gw01/nodeB/cmd/pid ---------- */
    size_t prefix_len = strlen(MQTT_PREFIX);
    if (strncmp(t, MQTT_PREFIX, prefix_len) != 0) {
        ESP_LOGW(TAG, "主题 %s 不属于本网关前缀 %s", t, MQTT_PREFIX);
        return;
    }
    const char *rest = t + prefix_len;   /* 剩下 "/nodeB/cmd/pid" */

    /* %[^/] = 一直读到 '/' 为止。前面那个 '/' 已经在 rest 里了，所以要写进格式串 */
    if (sscanf(rest, "/%7[^/]/cmd/%15s", node, kind) != 2) {
        ESP_LOGW(TAG, "主题 %s 格式不对，应为 " MQTT_PREFIX "/<节点>/cmd/<类型>", t);
        return;
    }

    /* ---------- 3. 目标节点 → 节点号 ---------- */
    uint32_t target;
    if (strcmp(node, "nodeA") == 0) {
        target = NODE_A;
    } else if (strcmp(node, "nodeB") == 0) {
        target = NODE_B;
    } else if (strcmp(node, "all") == 0) {
        target = NODE_BROADCAST;
    } else {
        ESP_LOGW(TAG, "未知目标节点 '%s'（可用 nodeA / nodeB / all）", node);
        return;
    }

    /* ---------- 4. 命令类型 → CAN ID ----------
     * 这一步就是"查表"：主题里的 cmd/<类型> 直接对应 protocol.h 的一个功能组。
     * 急停是广播帧（ID 低 4 位是 0），不参与"目标节点"那套。 */
    uint32_t id;
    if (strcmp(kind, "estop") == 0) {
        id = CAN_ID_ESTOP;
    } else if (strcmp(kind, "motor") == 0) {
        id = CAN_ID(GRP_CTRL_MOTOR, target);
    } else if (strcmp(kind, "target") == 0) {
        id = CAN_ID(GRP_CTRL_TARGET, target);
    } else if (strcmp(kind, "pid") == 0) {
        id = CAN_ID(GRP_CTRL_PID, target);
    } else if (strcmp(kind, "mode") == 0) {
        id = CAN_ID(GRP_CTRL_MODE, target);
    } else {
        ESP_LOGW(TAG, "未知命令类型 '%s'（可用 motor/target/pid/mode/estop）", kind);
        return;
    }

    /* ---------- 5. 载荷 → CAN 字节 ----------
     * 命令序号每帧都不同，执行端靠它去重（CAN 硬件重发时可能收到两遍同一条命令）。 */
    uint8_t cid = ++s_cid;

    switch (id) {

    case CAN_ID_ESTOP: {
        /* 负载选择动作：2 = 解除，其余一律按急停。
         * ★ 默认必须是急停 ★ 负载写空、写错、写数字以外的内容时，
         *   宁可停也不能放行 —— 这条和 protocol.h 里 ESTOP 那节
         *   "拿不准就填 STOP"是同一个原则。 */
        unsigned action = ESTOP_ACTION_STOP;
        if (payload_len > 0) {
            if (sscanf(pl, "%u", &action) != 1) {
                ESP_LOGW(TAG, "estop 负载应为 1(急停) 或 2(解除)，收到 \"%s\"，按急停处理", pl);
            }
        }
        if (action != ESTOP_ACTION_CLEAR) {
            action = ESTOP_ACTION_STOP;
        }

        PROTO_PackEstop(d, (uint8_t)action);
        can_send(id, d, LEN_ESTOP);
        ESP_LOGW(TAG, "MQTT→CAN 0x%03X %s", (unsigned)id,
                 action == ESTOP_ACTION_CLEAR ? "急停解除" : "急停");
        break;
    }

    case CAN_ID(GRP_CTRL_MOTOR, NODE_A):
    case CAN_ID(GRP_CTRL_MOTOR, NODE_B): {
        /* 负载形如 "1,50"（命令,参数）或只写 "0"（参数默认 0） */
        unsigned cmd = 0, arg = 0;
        if (sscanf(pl, "%u,%u", &cmd, &arg) < 1) {
            ESP_LOGW(TAG, "motor 负载应为 \"命令\" 或 \"命令,参数\"，收到 \"%s\"", pl);
            return;
        }
        PROTO_PackCtrlMotor(d, (uint8_t)cmd, (uint8_t)arg, cid);
        can_send(id, d, LEN_CTRL_MOTOR);
        ESP_LOGI(TAG, "MQTT→CAN 0x%03X 电机 cmd=%u arg=%u", (unsigned)id, cmd, arg);
        break;
    }

    case CAN_ID(GRP_CTRL_TARGET, NODE_A):
    case CAN_ID(GRP_CTRL_TARGET, NODE_B): {
        unsigned val = 0;
        if (sscanf(pl, "%u", &val) != 1) {
            ESP_LOGW(TAG, "target 负载应为数字，收到 \"%s\"", pl);
            return;
        }
        if (val > 65535u) {
            ESP_LOGW(TAG, "target %u 超出 u16，按 65535 处理", val);
            val = 65535u;
        }
        PROTO_PackCtrlTarget(d, (uint16_t)val, cid);
        can_send(id, d, LEN_CTRL_TARGET);
        ESP_LOGI(TAG, "MQTT→CAN 0x%03X 目标值=%u", (unsigned)id, val);
        break;
    }

    case CAN_ID(GRP_CTRL_PID, NODE_B): {
        /* 负载形如 "1.5,0.2,0"（Kp,Ki,Kd 真实值，协议里会 ×100 传） */
        float kp, ki, kd;
        if (sscanf(pl, "%f,%f,%f", &kp, &ki, &kd) != 3) {
            ESP_LOGW(TAG, "pid 负载应为 \"kp,ki,kd\"，收到 \"%s\"", pl);
            return;
        }
        PROTO_PackCtrlPid(d, to_u16_x100(kp), to_u16_x100(ki), to_u16_x100(kd), cid);
        can_send(id, d, LEN_CTRL_PID);
        ESP_LOGI(TAG, "MQTT→CAN 0x%03X PID kp=%.2f ki=%.2f kd=%.2f",
                 (unsigned)id, kp, ki, kd);
        break;
    }

    case CAN_ID(GRP_CTRL_MODE, NODE_B): {
        unsigned mode = 0;
        if (sscanf(pl, "%u", &mode) != 1) {
            ESP_LOGW(TAG, "mode 负载应为 0(手动)/1(自动)，收到 \"%s\"", pl);
            return;
        }
        PROTO_PackCtrlMode(d, (uint8_t)mode, cid);
        can_send(id, d, LEN_CTRL_MODE);
        ESP_LOGI(TAG, "MQTT→CAN 0x%03X 模式=%u", (unsigned)id, mode);
        break;
    }

    default:
        /* 理论上到不了这里：上面的 kind→id 已经过滤过一遍了。
         * 但如果以后加了新功能组却忘了在这里加 case，就会掉进来。 */
        ESP_LOGW(TAG, "暂不支持的命令 ID 0x%03X", (unsigned)id);
        break;
    }
}

/* 这次 BUS_OFF 值不值得打日志：第 1 次、之后每 10 次。
 * 注意它只读不写，"计数"由 can_service_bus_state 里的 ++ 负责。 */
static bool busoff_log_now(void)
{
    return (s_busoff_count <= 1) || (s_busoff_count % 10 == 0);
}

/* ==================== 总线状态看护 ====================
 *
 * 【为什么需要这个函数】TWAI 驱动的状态机是这样的：
 *
 *      RUNNING ──错误太多──→ BUS_OFF
 *      BUS_OFF ──twai_initiate_recovery()──→ RECOVERING
 *      RECOVERING ──总线上出现 128 次"11 个连续隐性位"──→ STOPPED
 *
 *   ★ 关键：恢复完成落到的是 STOPPED，不是 RUNNING ★
 *   而 twai_transmit / twai_receive 都要求 RUNNING 才干活，
 *   其它状态一律返回 ESP_ERR_INVALID_STATE。
 *   所以恢复完必须【再调一次 twai_start()】把它拉回 RUNNING，
 *   否则控制器就永远卡在 STOPPED，收发全废 —— 这个函数就是干这件事的。
 *
 *   在接收任务的超时分支里被调用（此时总线空闲，没有实时性要求）。
 *   正常的总线不会有状态变化，这里的 switch 每次都直接落到 default，开销可忽略。
 */
static void can_service_bus_state(void)
{
    twai_status_info_t st;

    if (twai_get_status_info(&st) != ESP_OK) {
        return;
    }

    switch (st.state) {
    case TWAI_STATE_BUS_OFF:
        /* 发送错误计数(TEC)超过 255 就会到这里。最常见的原因：
         * 发出去的帧【没有任何节点回 ACK】—— 没接收发器、收发器没供电、
         * TX/RX 接反、没共地、对端没上电、CANH/CANL 接反、终端电阻过多。
         * 这时唯一出路是主动发起恢复。 */
        s_busoff_count++;
        if (busoff_log_now()) {
            ESP_LOGE(TAG, "总线离线(BUS_OFF) 累计第 %u 次：发出去的帧没人应答",
                     (unsigned)s_busoff_count);
            /* 这几个计数是判断"哪一端坏了"的关键：
             *   bus_error_count   控制器在线上看到的位错误/格式错误次数
             *   arb_lost_count    仲裁丢失（说明看到了别的节点在发）
             *   收/发错误计数     TEC/REC，BUS_OFF 时 TEC 一定 >255
             *
             * ★ bus_error_count 一直是 0 ★ → 控制器压根没在线上看到任何电平变化，
             *   也就是 RX 引脚是"聋"的：收发器没供电 / GPIO4 没接到 RXD /
             *   没共地 / CANH,CANL 没接上。这时查 TX 也没用，先查这条收的路。 */
            ESP_LOGE(TAG, "  统计: 总线错误 %u 次, 仲裁丢失 %u 次, 收错 %u 帧, TEC=%u REC=%u",
                     (unsigned)st.bus_error_count, (unsigned)st.arb_lost_count,
                     (unsigned)st.rx_missed_count,
                     (unsigned)st.tx_error_counter, (unsigned)st.rx_error_counter);
        }
        twai_initiate_recovery();
        break;

    case TWAI_STATE_RECOVERING:
        /* 等 128 次 11 位隐性位。总线空闲时几毫秒就完成，
         * 所以往往在下一次轮询时状态已经变成 STOPPED 了，这条日志不一定看得到。 */
        break;

    case TWAI_STATE_STOPPED:
        /* 两种情况会落到这里：
         *   ① 恢复刚完成（最常见）—— 必须重新 start，否则永远起不来
         *   ② 有人手动调过 twai_stop()
         * 重 start 后如果故障依旧，会再次 BUS_OFF，形成"重试循环"。
         * 这正是我们想要的：接线一修好，下一秒就自己恢复通信，不用重启板子。 */
        if (twai_start() == ESP_OK && busoff_log_now()) {
            ESP_LOGW(TAG, "总线已自动重启（累计离线 %u 次）",
                     (unsigned)s_busoff_count);
        }
        break;

    default:
        break;   /* RUNNING：一切正常，什么都不用做 */
    }
}

/* ==================== 接收任务 ==================== */

static void can_rx_task(void *arg)
{
    twai_message_t msg;
    uint32_t   last_rx      = 0;
    TickType_t last_summary = xTaskGetTickCount();

    while (1) {
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));

        if (err == ESP_OK) {
            s_rx_count++;
            can_dispatch(&msg);
        } else if (err == ESP_ERR_TIMEOUT) {
            /* 这段时间总线上一帧都没有，顺手看护一下控制器状态。
             * 注意：BUS_OFF/RECOVERING/STOPPED 时 twai_receive 并不是返回
             * 错误，而是收不到东西照样超时 —— 所以状态检查必须放在这里。 */
            can_service_bus_state();
        } else {
            ESP_LOGE(TAG, "twai_receive 出错: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));   /* 别把 CPU 打满 */
        }

        /* ---------- 每秒一行汇总：这是判断"总线到底通不通"的锚点 ----------
         * 为什么放在接收任务而不是发送任务里：
         *   发送任务是可以关掉的（调试时、或者 LISTEN_ONLY 模式下），
         *   但"收"永远在跑。只有这行始终在打，
         *   收 0 帧的时候你才能确定"板子活着、就是没数据"，
         *   而不是在猜"是不是日志根本没在跑"。 */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_summary) >= pdMS_TO_TICKS(1000)) {
            uint32_t now_rx = s_rx_count;
            ESP_LOGI(TAG, "1s 收 %u 帧 (累计 %u) | 节点A: %s | 节点B: %s",
                     (unsigned)(now_rx - last_rx), (unsigned)now_rx,
                     node_state_str(&s_nodes[0]),
                     node_state_str(&s_nodes[1]));
            last_rx      = now_rx;
            last_summary = now;
        }
    }
}

/* ==================== 发送任务（周期上报 + 掉线检测） ====================
 *
 * 注意：汇总日志已经挪到接收任务里了，这里只负责"发"和"检测掉线"，
 * 免得两个任务各打一行、每秒出两条重复的。 */

static void can_tx_task(void *arg)
{
    while (1) {
        can_send_periodic();
        can_check_nodes();
        vTaskDelay(pdMS_TO_TICKS(CAN_TX_PERIOD_MS));
    }
}

/* ==================== 初始化 ==================== */

void can_init(void)
{
    /* ---------- 1. 控制器参数 ----------
     * ★ 宏的参数顺序是 (TX, RX, 模式)，不是 (RX, TX) ★ 写反了现象是
     *   "发得出去但一帧都收不到"（或反之），比编不过还难查。
     *
     * 模式用文件顶部的 CAN_MODE。 */
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, CAN_MODE);

    /* ---------- 2. 位时序（波特率 + 采样点） ----------
     * 250kbps，protocol.h 里定死的。
     *
     * ★ 为什么不直接用 TWAI_TIMING_CONFIG_250KBITS() ★
     *   那个宏在 S3 上是：quanta=5MHz，(1+15+4)=20 tq，采样点 16/20 = 80%。
     *   而 STM32 那边（见 11111/22222 的 Core/Src/can.c）是：
     *       APB1=36MHz, Prescaler=9 → tq=4MHz, (1+13+2)=16 tq → 250kbps
     *       采样点 (1+13)/16 = 14/16 = 87.5%
     *   两边波特率对得上，但采样点差了 7.5%。两个 STM32 配置一字不差所以互收
     *   正常，ESP32 一挂上去就通不了 —— 所以这里逐项对齐 STM32 的参数。
     *
     * 换模式见文件顶部的 CAN_MODE。 */
    twai_timing_config_t t_cfg = {
        .clk_src              = TWAI_CLK_SRC_DEFAULT,
        .quanta_resolution_hz = 4000000,   /* 非 0 时驱动自己算分频，brp 填 0 即可 */
        .brp                  = 0,
        .tseg_1               = 13,
        .tseg_2               = 2,
        .sjw                  = 2,
        .triple_sampling      = false,
    };

    /* ---------- 3. 接收过滤器：全收 ----------
     * 网关要听 A、B、广播的所有帧，所以不过滤。
     * 如果以后只想听心跳，把 acceptance_code/mask 设成只比较高 7 位
     * （GRP_HEARTBEAT）就行 —— 这正是 protocol.h 把 ID 拆成
     * "功能组+节点号"的意义：过滤规则不用改，加节点只改节点号。 */
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));

    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start 失败: %s", esp_err_to_name(err));
        return;
    }

    /* 把模式也打出来，免得烧完看着日志发懵"为什么发不出去"。
     * LISTEN_ONLY 下 twai_transmit 会直接返回 ESP_ERR_NOT_SUPPORTED。 */
    ESP_LOGI(TAG, "TWAI 已启动: TX=GPIO%d RX=GPIO%d 250kbps 采样点87.5%% 模式=%s",
             CAN_TX_GPIO, CAN_RX_GPIO,
             CAN_MODE == TWAI_MODE_NORMAL ? "NORMAL" : "LISTEN_ONLY");

    /* ---------- 4. 启动任务 ----------
     * 收帧任务优先级 6（略高于 led/wifi 的 5），先保证帧不丢；
     * 发送任务 4，周期活儿不急；
     * 发布任务 3，垫底 —— 见 can_pub_task 上面那段说明。 */

    /* 队列必须先建好，pub_telemetry() 要用它。建不出来就别启动任务了，
     * 否则收帧任务一跑就是空指针解引用。 */
    s_pub_q = xQueueCreate(PUB_QUEUE_LEN, sizeof(pub_item_t));
    if (s_pub_q == NULL) {
        ESP_LOGE(TAG, "遥测队列创建失败(内存不足)，CAN 任务未启动");
        return;
    }

    xTaskCreate(can_rx_task,  "can_rx",  4096, NULL, 6,                NULL);
    xTaskCreate(can_tx_task,  "can_tx",  4096, NULL, 4,                NULL);
    xTaskCreate(can_pub_task, "can_pub", 4096, NULL, CAN_PUB_TASK_PRIO, NULL);
}

/* ==================== 运行统计 ====================
 * 四个数的语义见 can.h。这里只是把四个 static 拷进一份结构体。
 *
 * 按值返回而不是返回指针：调用方要拿 a_rx / a_lost 相除算丢包率，
 * 给指针的话两个字段之间可能隔着一次更新，会算出瞬时错误的比例。
 * 给一份拷贝就没这个问题。40 字节不到，返回很便宜。
 *
 * 注意这不是原子快照 —— 四个字段各自是原子的，但四个值不是同一瞬间取的。
 * 对"开机至今"这种单调累加的量来说无所谓：偏差最多一帧。 */
can_stats_t can_get_stats(void)
{
    can_stats_t s;

    s.a_rx      = s_a_rx;
    s.a_lost    = s_a_lost;
    s.pub_total = s_pub_total;
    s.pub_lost  = s_pub_lost;

    return s;
}
