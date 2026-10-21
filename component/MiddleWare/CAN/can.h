#ifndef CAN_H
#define CAN_H

#include <stdint.h>

/* ============================================================
 * 【CAN 通信封装】—— 网关(NODE_GW) 与 A/B 节点之间的物理链路
 *
 * ESP32-S3 用片内的 TWAI 控制器(兼容 CAN 2.0A)接一条 CAN 总线，
 * 和 STM32 的 11111(NODE_A)、22222(NODE_B) 通信。
 *
 * ★ ESP32 只有 CAN 控制器，没有物理层 ★
 *   必须外挂一个收发器(TJA1050 / SN65HVD230 / MCP2551)：
 *     ESP  TX(GPIO5) ──→ 收发器 TXD      收发器 CANH ─┐
 *     ESP  RX(GPIO4) ←── 收发器 RXD      收发器 CANL ─┤──→ 总线
 *     ESP  GND ────────── 收发器 GND                  ┘  (必须共地)
 *   总线两端各要一个 120Ω 终端电阻。
 *
 * 接口只暴露三个：
 *   can_init()        初始化并启动收发任务（收帧 + 转发 + 周期上报）
 *   can_send()        手工发一帧（一般不用自己调，bridge 内部会调）
 *   can_bridge_mqtt() MQTT 收到命令 → 转成 CAN 帧（由 mqtt.c 的回调调用）
 * ============================================================ */

/* 初始化 TWAI：装驱动、启动、创建收/发任务。调用一次即可。 */
void can_init(void);

/* 发一帧标准帧(11 位 ID)。
 *   id  —— CAN ID，用 protocol.h 里的 CAN_ID_xxx
 *   data—— 数据字节，长度必须和 protocol.h 的 LEN_xxx 一致
 *   len —— DLC，0~8
 * 内部带超时，发不出去会打日志并返回，不会卡死。 */
void can_send(uint32_t id, const uint8_t *data, uint8_t len);



/* ==================== 运行统计 ====================
 * 就四个数，都是【开机至今】的累计值：
 *
 *   a_rx       收到的 A 数据帧(0x101)总数
 *   a_lost     A 数据帧丢失总数
 *              丢包率 = a_lost / (a_lost + a_rx)
 *   pub_total  MQTT 发布总数
 *   pub_lost   MQTT 丢包总数
 *              丢包率 = pub_lost / pub_total
 *
 * 采集在 can.c，上报到 gw0805/gateway/status（mqtt.h 的 TOPIC_STATUS_GW）。
 * 只有 A 能测丢包 —— 只有它的 0x101 带 seq，B 的 0x102 没有。
 *
 * ★ 两条必须做对的地方，做错会永久污染累计值：
 * 1. seq 相同是重复帧，不是丢帧。PROTO_SeqLost(x, x) 算出来是 255
 *    不是 0（看它的表达式 (x-x-1) & 0xFF）。CAN 允许重复帧，所以
 *    必须先判 seq != 上次的 seq，否则一次就往 a_lost 里灌 255。
 * 2. A 重启会让 seq 从 0 重来，算出一个大跳变，被当成一次巨大丢包。
 *    seq 只有 8 位、帧里又没有 boot_id，两者在协议上分不开，只能
 *    用阈值近似：lost >= 10（≈1 秒的帧量）判为疑似重启，不计入 a_lost。
 */
typedef struct {
    uint32_t a_rx;        /* 收到的 A 数据帧(0x101)总数 */
    uint32_t a_lost;      /* A 数据帧丢失总数（seq 推算） */
    uint32_t pub_total;   /* MQTT 发布总数（本该发出去的条数，含丢掉的） */
    uint32_t pub_lost;    /* MQTT 丢包总数：没连上 + 队列满 + 发失败 */
} can_stats_t;

/* 取一份快照（按值返回，调用方拿到自己的一份拷贝）。 */
can_stats_t can_get_stats(void);

/* MQTT 主题命令 → CAN 帧。
 *   topic/payload 由 esp-mqtt 回调给出，【不保证以 '\0' 结尾】，
 *   所以必须把长度一起传进来，函数内部自己拷到本地缓冲。
 *
 * 主题格式：<MQTT_PREFIX>/<目标节点>/cmd/<命令类型>
 *   gw0805/nodeA/cmd/motor   负载 "1,50"      → 0x301 CTRL_MOTOR_A
 *   gw0805/nodeA/cmd/target  负载 "2000"      → 0x311 CTRL_TARGET_A
 *   gw0805/nodeB/cmd/motor   负载 "3,80"      → 0x302 CTRL_MOTOR_B
 *   gw0805/nodeB/cmd/pid     负载 "1.5,0.2,0" → 0x322 CTRL_PID_B
 *   gw0805/nodeB/cmd/mode    负载 "1"         → 0x332 CTRL_MODE_B
 *   gw0805/all/cmd/estop     负载 "1"/"2"     → 0x000 ESTOP(广播)
 *                          负载 1=急停(默认), 2=解除；写空或写错按急停 */
void can_bridge_mqtt(const char *topic, int topic_len,
                     const char *payload, int payload_len);

#endif /* CAN_H */
