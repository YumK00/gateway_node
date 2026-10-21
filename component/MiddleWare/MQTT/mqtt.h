#ifndef MQTT_H
#define MQTT_H

#include <stdbool.h>

/* ============================================================
 * 【MQTT 客户端封装】—— 网关(NODE_GW) 与 云端/App 之间收发
 *
 * 网关是"桥"：一侧是 CAN(A/B 节点)，一侧是 MQTT(云端)。
 * 主题设计跟着 protocol.h 的功能组一一对应，不做二次发明：
 *
 *   CAN 上行(0x1xx / 0x01x)  →  网关发布  →  gw/<节点>/telemetry、alert
 *   CAN 下行(0x3xx / 0x000)  ←  网关订阅  ←  gw/<节点>/cmd/<类型>
 *
 * 用法（必须在 WiFi 连上之后）：
 *   mqtt_init();                              // 建客户端并启动（内部自带任务）
 *   mqtt_publish(TOPIC_STATUS_GW, "{}", MQTT_QOS_TELEMETRY);
 *
 * 断线重连：client 内部任务会自动重连 broker，无需手动处理。
 * ============================================================ */

/* ---------- 主题前缀 ----------
 * 三个节点共用一棵主题树，根节点就叫 gw。
 * 想换名字只改这一处，收发两边都跟着变。 */
#define MQTT_PREFIX  "gw0805"

/* ---------- 上行：网关发布（CAN 收到 → MQTT 发出） ---------- */
#define TOPIC_TELEMETRY_A  MQTT_PREFIX "/nodeA/telemetry"   /* ← CAN 0x101 A_SENSOR  */
#define TOPIC_TELEMETRY_B  MQTT_PREFIX "/nodeB/telemetry"   /* ← CAN 0x102 B_STATUS  */
#define TOPIC_ALERT_A      MQTT_PREFIX "/nodeA/alert"       /* ← CAN 0x011 ALARM_A   */
#define TOPIC_ALERT_B      MQTT_PREFIX "/nodeB/alert"       /* ← CAN 0x012 ALARM_B   */
#define TOPIC_STATUS_GW    MQTT_PREFIX "/gateway/status"    /* ← CAN 0x103 GW_STATUS */
#define TOPIC_STATUS_STATS MQTT_PREFIX "/gateway/stats"     /* ← can_get_stats() 的四个累计值 */

/* topic 要按目标节点拼出来，用这两个格式化宏：
 *   char t[48];
 *   snprintf(t, sizeof(t), TOPIC_FMT_TELEMETRY, "A");   → "gw/nodeA/telemetry"
 * 注意 %s 只能填 "A" / "B" 这种不带斜杠的短名。 */
#define TOPIC_FMT_TELEMETRY  MQTT_PREFIX "/%s/telemetry"
#define TOPIC_FMT_ALERT      MQTT_PREFIX "/%s/alert"

/* ---------- 下行：网关订阅（MQTT 收到 → 转成 CAN 帧发出） ----------
 * 一条通配订全部：gw/<目标节点>/cmd/<命令类型>
 *   + 匹配单层        → 吃掉 nodeA / nodeB / all 那一段
 *   # 匹配剩余所有层  → cmd 后面接 motor / target / pid / mode / estop 都收得到
 *
 * 每个具体 topic 对应哪一帧 CAN，查这张表就行：
 *   gw/nodeA/cmd/motor   → 0x301 CTRL_MOTOR_A
 *   gw/nodeA/cmd/target  → 0x311 CTRL_TARGET_A
 *   gw/nodeB/cmd/motor   → 0x302 CTRL_MOTOR_B
 *   gw/nodeB/cmd/target  → 0x312 CTRL_TARGET_B
 *   gw/nodeB/cmd/pid     → 0x322 CTRL_PID_B
 *   gw/nodeB/cmd/mode    → 0x332 CTRL_MODE_B
 *   gw/all/cmd/estop     → 0x000 ESTOP（广播，负载 1=急停 2=解除）
 *
 * 为什么把命令类型放进 topic 而不是塞进 payload：
 *   CAN 侧已经按功能组拆好了(GRP_CTRL_MOTOR / GRP_CTRL_PID / ...)，
 *   这里再压成一个 cmd topic 靠 payload 里的 type 区分，等于把
 *   刚设计好的功能组重新发明一遍，还多一层解析。按 cmd/<类型> 展开，
 *   网关收到 "gw/nodeB/cmd/pid" 直接查表就知道该发 0x322。
 *
 * 注意：CAN 侧的心跳(0x7xx) 不映射到 MQTT。
 *   broker 靠 MQTT 自己的 keepalive 判断网关死没死；
 *   云端判断 A/B 掉线，看它们的 telemetry 多久没更新就够了。 */
#define TOPIC_CMD_SUB      MQTT_PREFIX "/+/cmd/#"

/* ---------- QoS ----------
 * 遥测数据丢一帧无所谓，下一帧马上就来        → QoS 0
 * 控制命令丢了就是丢了(该停的电机没停)        → QoS 1 */
#define MQTT_QOS_TELEMETRY  0
#define MQTT_QOS_CMD        1

/* 初始化并启动 MQTT 客户端（需在 WiFi 连上之后调用） */
void mqtt_init(void);

/* 发布一条消息。payload 以 '\0' 结尾；qos 取 MQTT_QOS_xxx。
 * 返回 true = 已经交给 esp-mqtt 发出去；false = 没发出去
 * （没连上 broker，或者 esp-mqtt 内部失败）。 */
bool mqtt_publish(const char *topic, const char *payload, int qos);

/* 订阅一个主题；qos 取 MQTT_QOS_xxx */
void mqtt_subscribe(const char *topic, int qos);

/* 查询是否已连上 broker */
bool mqtt_is_connected(void);

#endif /* MQTT_H */
