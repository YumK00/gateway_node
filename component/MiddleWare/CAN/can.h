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

/* MQTT 主题命令 → CAN 帧。
 *   topic/payload 由 esp-mqtt 回调给出，【不保证以 '\0' 结尾】，
 *   所以必须把长度一起传进来，函数内部自己拷到本地缓冲。
 *
 * 主题格式：<MQTT_PREFIX>/<目标节点>/cmd/<命令类型>
 *   gw01/nodeA/cmd/motor   负载 "1,50"      → 0x301 CTRL_MOTOR_A
 *   gw01/nodeA/cmd/target  负载 "2000"      → 0x311 CTRL_TARGET_A
 *   gw01/nodeB/cmd/motor   负载 "3,80"      → 0x302 CTRL_MOTOR_B
 *   gw01/nodeB/cmd/pid     负载 "1.5,0.2,0" → 0x322 CTRL_PID_B
 *   gw01/nodeB/cmd/mode    负载 "1"         → 0x332 CTRL_MODE_B
 *   gw01/all/cmd/estop     负载 "1"/"2"     → 0x000 ESTOP(广播)
 *                          负载 1=急停(默认), 2=解除；写空或写错按急停 */
void can_bridge_mqtt(const char *topic, int topic_len,
                     const char *payload, int payload_len);

#endif /* CAN_H */
