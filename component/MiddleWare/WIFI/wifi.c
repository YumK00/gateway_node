#include "wifi.h"

#include <string.h>
#include <assert.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi_default.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

/* ============================================================
 * 【WiFi STA】—— 扫描 / 连接
 *
 * 初始化顺序不能乱，每一步都依赖上一步：
 *   nvs_flash_init                → WiFi 驱动要把校准数据存 NVS
 *   esp_netif_init                → TCP/IP 协议栈
 *   esp_event_loop_create_default → 事件循环，WiFi 状态变化靠它分发
 *   esp_netif_create_default_wifi_sta → 建 STA 网卡（连上后 IP 挂这上面）
 *   esp_wifi_init                 → 初始化 WiFi 驱动
 *   esp_wifi_set_mode + start     → 设为 STA 模式并开机
 *
 * wifi_scan() 和 wifi_connect() 共用上面这一套，所以抽成 wifi_init_once()：
 *   这几步是全局的，只能做一次，重复调用会返回 ESP_ERR_INVALID_STATE。
 *
 * 连接是"异步发起、事件通知结果"：
 *   esp_wifi_connect() 只是把连接请求丢给驱动，马上返回；
 *   真正连上/失败是通过 WIFI_EVENT / IP_EVENT 事件回调告知的。
 *   所以 wifi_connect() 里用事件组(EventGroup)阻塞等结果，才像个同步函数。
 * ============================================================ */

static const char *TAG = "WIFI";

#define WIFI_SSID "iPhone"
#define WIFI_PASSWORD "kym123456"

#define DEFAULT_SCAN_LIST_SIZE 12

/* 一次性初始化是否已完成 */
static bool s_wifi_inited = false;

/* wifi_connect() 最多等多久拿到 IP */
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* 事件组：用来把"事件回调里发生的事情"同步给阻塞等待的 wifi_connect() */
static EventGroupHandle_t s_wifi_events = NULL;
#define WIFI_CONNECTED_BIT  BIT0   /* 连上并拿到 IP */
#define WIFI_FAIL_BIT       BIT1   /* 重试次数用尽，彻底失败 */

/* 已重连次数 */
static int s_retry_num = 0;

/* 当前是否已连上并拿到 IP（供其它任务查询，如 MQTT 判断何时能连 broker） */
static bool s_wifi_connected = false;

/**
 * @brief  把加密方式枚举转成人类可读的字符串
 * @note   枚举里 WIFI_AUTH_WPA2_ENTERPRISE 和 WIFI_AUTH_ENTERPRISE 是同一个值，
 *         switch 里只能写一个，两个都写会报 "重复的 case 值"
 */
static const char *authmode_str(wifi_auth_mode_t mode)
{
    switch (mode)
    {
    case WIFI_AUTH_OPEN:          return "OPEN";       /* 开放，无密码 */
    case WIFI_AUTH_WEP:           return "WEP";        /* 老古董，已不安全 */
    case WIFI_AUTH_WPA_PSK:       return "WPA";
    case WIFI_AUTH_WPA2_PSK:      return "WPA2";       /* 目前最常见 */
    case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA/WPA2";
    case WIFI_AUTH_ENTERPRISE:    return "ENTERPRISE"; /* 企业认证 802.1X */
    case WIFI_AUTH_WPA3_PSK:      return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    case WIFI_AUTH_WAPI_PSK:      return "WAPI";       /* 国标 */
    case WIFI_AUTH_OWE:           return "OWE";        /* 增强型开放：无密码但仍加密 */
    default:                      return "UNKNOWN";
    }
}


static const char *wifi_reason_str(uint8_t reason)
{
    switch(reason)
    {
        case 2: return "AUTH_FAILED 认证失败(大概率密码错误)";
        case 3: return "DISASSOC AP主动断开";
        case 8: return "ASSOC_TIMEOUT 关联超时，信号弱";
        case 15: return "4WAY_HANDSHAKE_TIMEOUT 四次握手超时(密码/加密不匹配)";
        case 201: return "ASSOC_LEAVE 本设备主动断开";
        case 202: return "ASSOC_NOT_FOUND 找不到该SSID热点";
        case 203: return "ASSOC_REJECT AP拒绝接入(路由MAC过滤/设备满)";
        default: return "未知断开原因";
    }
}

/**
 * @brief  WiFi 事件回调：由事件循环在系统任务里调用
 * @note   注意这里运行在事件循环任务的上下文，不要在回调里做耗时操作，
 *         否则会卡住整个事件系统。所以这里只置位，具体等待交给 wifi_connect()
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /* ---- WiFi 事件 ---- */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        /* 驱动起来了。这里故意不自动 esp_wifi_connect()：
         * 那会拿此时还没设置的(空) SSID 去连，白白失败一轮，
         * 连接统一由 wifi_connect() 显式发起 */
        ESP_LOGI(TAG, "WiFi 驱动已启动");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        /* 连接失败或中途掉线都会走到这里，reason 是断线原因码，
         * 常见：15=四次握手超时(多半密码错)  201=热点没找到 */
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "断开连接，reason=%d (%s)", disc->reason, wifi_reason_str(disc->reason));
        s_wifi_connected = false;   /* 掉线了，其它模块应暂停网络操作 */

        disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "第%d次连接失败,原因:%d (%s)", ++s_retry_num, disc->reason, wifi_reason_str(disc->reason));
        wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    }
    /* ---- IP 事件 ---- */
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        /* 连上 AP 只是第一步，还要等 DHCP 分到 IP 才算真正可用。
         * 之后才能 socket 收发数据 */
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "拿到 IP: " IPSTR ", 网关: " IPSTR,
                 IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.gw));

        s_retry_num = 0;   /* 成功一次就把重连计数清零 */
        s_wifi_connected = true;   /* 标记已连上，MQTT 等模块可以开工了 */
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief  一次性初始化：NVS 之外的整条 WiFi 链路
 * @note 
 */
static void wifi_init_once(void)
{
    if (s_wifi_inited)
    {
        return;
    }

    /* TCP/IP 协议栈，网络相关的一切都建立在它之上 */
    ESP_ERROR_CHECK(esp_netif_init());

    /* 默认事件循环。WiFi 的"扫完了/连上了/断开了/拿到IP了"
     * 都是往这个循环里投事件来通知的 */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 建一个默认 STA 网卡。连上后 DHCP 分配的 IP 就挂在它上面 */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    /* 事件组要在注册回调之前建好，回调里会用到它 */
    s_wifi_events = xEventGroupCreate();
    assert(s_wifi_events);

    /* 订阅事件：WiFi 状态变化 + IP 获取。
     * 最后一个参数 NULL 表示不关心事件投递到哪个任务 */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    /* 初始化 WiFi 驱动。WIFI_INIT_CONFIG_DEFAULT() 是官方默认参数宏，
     * 里面已经填好收发缓冲数量、任务优先级等一堆默认值 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 配置只放内存、不写 flash。
     * 因为 SSID/密码是代码里写死的，每次开机重新 set_config 就行，
     * 写 flash 反而白耗擦写寿命 */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    uint8_t mac[6];//   84:C7:BB:7C:00:30
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ESP32 WiFi STA MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    /* 设为 STA 模式并启动。
     * 少了 esp_wifi_start()，后面的 scan/connect 都会返回 ESP_ERR_WIFI_NOT_STARTED */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_inited = true;
}

/**
 * @brief  NVS + WiFi 协议栈初始化，然后连接路由器
 * @note   首次上电调用；NVS 分区满或版本不匹配时需先擦除重建
 */
void wifi_sta_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* 分区满 / 里面存的是旧版本格式：擦掉重建才能用 */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_init_once();
    //wifi_scan();
    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
}

/**
 * @brief  连接指定热点，阻塞到拿到 IP 或超时为止
 * @param  ssid     ：热点名，最长 32 字节
 * @param  password ：密码，最长 64 字节；传 NULL 或 "" 表示开放热点
 * @note   内部依赖 wifi_init_once()，所以不用先手动调 wifi_scan()
 */
void wifi_connect(const char *ssid, const char *password)
{

    if (ssid == NULL || ssid[0] == '\0')
    {
        ESP_LOGE(TAG, "SSID 为空，无法连接");
        return;
    }

    /* 先量一下长度：超了 strncpy 会静默截断*/
    if (strlen(ssid) > sizeof(((wifi_sta_config_t *)0)->ssid) - 1)
    {
        ESP_LOGE(TAG, "SSID 过长（最长 32 字节）");
        return;
    }
    if (password != NULL && strlen(password) > sizeof(((wifi_sta_config_t *)0)->password) - 1)
    {
        ESP_LOGE(TAG, "密码过长（最长 64 字节）");
        return;
    }

    /* ---- 填配置 ---- */
    wifi_config_t wifi_cfg = {0};   /* 必须清零，否则 threshold.pmf_cfg 等字段是随机值 */

    /* ssid/password 在结构体里是 uint8_t 数组，所以强转一下 */
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    if (password != NULL)
    {
        strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    }

    /* 加密门限：低于这个等级的热点直接不连。
     * 默认是 WPA2，所以连老旧的 WPA/WEP 热点会失败 */
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /* PMF(管理帧保护)：capable=我支持，required=必须对方也支持。
     * 设 capable 能在 WPA2/WPA3 混合热点上兼容得更好 */
    wifi_cfg.sta.pmf_cfg.capable  = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    if (password == NULL || password[0] == '\0')
    {
        /* 开放热点：门限必须降到 OPEN，
         * 否则上面那个 WPA2 门限会让驱动直接拒绝连接 */
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        ESP_LOGW(TAG, "未提供密码，按开放热点连接");
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    /* ---- 清状态，避免上一轮的结果影响这一轮 ---- */
    s_retry_num = 0;
    xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    ESP_LOGI(TAG, "正在连接 %s ...", ssid);
    ESP_ERROR_CHECK(esp_wifi_connect());

    /* ---- 阻塞等结果 ----
     * 等"连上"或"彻底失败"任一位置位；两个都没等到就是超时。
     * xEventGroupWaitBits 等待期间本任务被挂起，不占 CPU */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,   /* 不自动清除，留给下次判断 */
                                           pdFALSE,   /* 任一位满足即可返回 */
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "已连接到 %s", ssid);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "连接 %s 失败（密码错误或热点不在范围内）", ssid);
    }
    else
    {
        ESP_LOGE(TAG, "连接 %s 超时（%d ms）", ssid, WIFI_CONNECT_TIMEOUT_MS);
    }
}

/**
 * @brief  查询当前 WiFi 是否已连上并拿到 IP
 * @note   给 MQTT 等"必须有网才能开工"的模块用
 */
bool wifi_is_connected(void)
{
    return s_wifi_connected;
}

/**
 * @brief  初始化 WiFi（STA 模式）并扫描周围热点，结果打到日志
 * @note   初始化只做一次；重复调用等于"重新扫描一遍"
 */
void wifi_scan(void)
{
    /* ================= 扫描 ================= */
    ESP_LOGI(TAG, "开始扫描...");

    wifi_scan_config_t scan_cfg = {
        .ssid        = NULL,   /* NULL = 不按名字过滤，扫全部热点 */
        .bssid       = NULL,
        .channel     = 0,      /* 0 = 全部信道都扫 */
        .show_hidden = true,   /* 隐藏了 SSID 的热点也列出来（SSID 显示为空） */
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
        /* ACTIVE = 主动发探测帧去问，比 PASSIVE(被动等 beacon)快得多 */
        .scan_time.active.min = 120,  /* 每个信道最短停留 ms，太短容易漏热点 */
        .scan_time.active.max = 300,  /* 最长停留，超了就换下个信道 */
    };

    /* 第二个参数 blocking=true：阻塞到扫完才返回。
     * 传 false 就得自己去收 WIFI_EVENT_SCAN_DONE 事件才知道扫完了。
     * 这里用结构体而不是像原来那样传 NULL，是为了能控制扫描时长 */
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_cfg, true));

    /* ================= 取结果 ================= */
    uint16_t ap_count = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

    /* number 是"传入数组容量、返回实际条数"的双向参数。
     * 先填上数组大小，驱动最多回这么多条，不会写越界 */
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));

    /* 扫到的总数可能超过数组容量，说明一下避免误会 */
    if (ap_count > number)
    {
        ESP_LOGI(TAG, "共扫到 %u 个热点，只列出前 %u 个（数组只有 %d 格）",
                 (unsigned)ap_count, (unsigned)number, DEFAULT_SCAN_LIST_SIZE);
    }
    else
    {
        ESP_LOGI(TAG, "共扫到 %u 个热点", (unsigned)ap_count);
    }

    /* 注意循环上界用 number（实际条数），不是 DEFAULT_SCAN_LIST_SIZE。
     * 数组后半段是 memset 出来的空记录，打出来只会刷一堆空行。
     *
     * SSID 放最后打印：中文 SSID 是 3 字节/字，会把前面列宽顶歪，
     * 放末尾其余列就能对齐 */
    for (int i = 0; i < number; i++)
    {
        wifi_ap_record_t *ap = &ap_info[i];
        ESP_LOGI(TAG, "[%2d] RSSI:%4d dBm  信道:%2d  加密:%-10s  BSSID:%02X:%02X:%02X:%02X:%02X:%02X  SSID:%s",
                 i + 1,
                 ap->rssi,      /* 信号强度，越接近 0 越强：-30 很好 / -90 很差 */
                 ap->primary,   /* 主信道 */
                 authmode_str(ap->authmode),
                 /* BSSID 是 AP 的 MAC。同一个 SSID 有多个路由器时靠它区分 */
                 ap->bssid[0], ap->bssid[1], ap->bssid[2],
                 ap->bssid[3], ap->bssid[4], ap->bssid[5],
                 (const char *)ap->ssid);
    }
}
