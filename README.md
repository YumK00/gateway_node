# 边缘智能网关（gateway_node）

> ESP32-S3 上的工业边缘网关：一侧用片内 TWAI 控制器接 250 kbps CAN 总线，一侧用 WiFi + MQTT 接云端，把 CAN 帧与 MQTT 消息双向桥起来。
> 在整条链路里它是唯一有网络能力的节点（`NODE_GW = 3`）：A/B 的上行数据转成 MQTT 遥测，云端命令转成 CAN 控制帧下发，再用周期帧给总线上的节点报自己的网况与存活。
> 三个嵌入式仓库共用同一份 `protocol.h`（CAN 应用层协议 v3.0），本仓库是其中的网关端实现；作品集里另有独立的 Qt 项目 `v_car`（车载域仿真），不在这条总线上。

## 一、项目简介

这是「工业边缘智能网关」项目的网关侧工程。整套系统由三个嵌入式仓库组成：两个 STM32F103 节点（A 采集、B 执行）和一个 ESP32-S3 网关（本仓库）。三个工程共用一份 `protocol.h` 作为 CAN 应用层契约，三份 `protocol.h` 逐字节相同（SHA256 一致），改协议必须三端同步。作品集里的 `v_car` 是另一个独立项目（Qt 车载 T-Box 仿真），不与本链路共用 Broker 或协议。

本仓库解决的是「CAN 现场总线 ↔ IP 网络」之间的落差：CAN 一帧只有 8 字节、ID 承载语义；MQTT 需要可读负载与主题树。网关做的就是**按 ID 查表转发**——上行按 ID 分发打包成 JSON，下行按主题里的命令类型反查功能组发帧，不二次发明格式。

主要能力：

- **CAN → MQTT 上行**：`0x101` A_SENSOR、`0x102` B_STATUS → `gw0805/nodeA|nodeB/telemetry`；`0x011`/`0x012` ALARM → 对应 `alert` 主题；`0x000` 急停只记日志、不转发。
- **MQTT → CAN 下行**：`gw0805/<nodeA|nodeB|all>/cmd/<motor|target|pid|mode|estop>` → `0x301/0x302/0x311/0x312/0x322/0x332/0x000`。
- **周期帧与判活**：每 1 s 发 `0x103` GW_STATUS 与 `0x703` GW_HEARTBEAT；`0x701`/`0x702` 超 3 s 未到判掉线，只在状态变化时上报 `ALM_COMM_LOST(2)`。
- **队列解耦与统计**：收帧任务只把遥测拷进 8 深队列（永不阻塞），另有一个低优先级任务专发 MQTT；四个累计值每 5 s 发到 `gw0805/gateway/stats`。
- **总线自愈与状态灯**：BUS_OFF → `twai_initiate_recovery()` → STOPPED → 自动 `twai_start()`；GPIO48 的 WS2812 每 2 s 红闪一次（仅表示程序在跑）。
- **断网缓存与掉电续传**：断网期间遥测写入环形缓冲区并按序号连续编号，网络恢复后按时间顺序补传、配合云端幂等去重；缓存记录与读写指针持久化到 NVS / Flash 分区，先写数据后提交索引，配合魔数与 CRC 校验，掉电重启后恢复未补传数据；未补传条数经 `0x103` 与 `gateway/status` 周期上报。

尚未实现、属于后续阶段的：SNTP 时间戳、MQTT 遗嘱（LWT）/retained、OTA、UDS 诊断（`0x7E0/0x7E8`）、AI 推理。

## 二、系统架构

```text
          MQTTX / 手机端 / 云端消费者（任意 MQTT 客户端）
                        │ mqtt://broker.emqx.io:1883（MQTT 5.0）
                        ▼
┌──────────────── ESP32-S3  gateway_node  NODE_GW=3 ────────────────┐
│ wifi_task(5)      每 5 s：gateway/status + gateway/stats           │
│ esp-mqtt 内部任务  收 gw0805/+/cmd/# → can_bridge_mqtt()           │
│ can_pub(3) ◄──[队列 8]◄── pub_telemetry() ◄── can_dispatch()       │
│ can_rx(6)         twai_receive(100 ms) → 分发 / bus-off 状态看护   │
│ can_tx(4)         每 1 s：0x103 + 0x703；心跳超时判活 → alert      │
│ TWAI：TX=GPIO5  RX=GPIO4  250 kbps  16 tq@4 MHz  采样点 87.5%      │
└──────────────────────────────┬────────────────────────────────────┘
                               │ CAN 2.0A 250 kbps（外挂收发器 + 两端 120Ω）
              ┌────────────────┴────────────────┐
        A 采集节点 NODE_A=1                B 执行节点 NODE_B=2
        STM32F103 + 红外/光敏              STM32F103 + 电机/PID
```

### 三个嵌入式工程分工（★ 为本仓库；另有独立项目 `v_car`，是车载域 Qt 仿真，不接入本链路）

| 仓库 | 平台 | 节点号 | 职责 | 与网关的关系 |
|---|---|---|---|---|
| ★ **gateway_node**（本仓库） | ESP32-S3 / ESP-IDF + FreeRTOS | `NODE_GW=3` | CAN ⇄ WiFi/MQTT 双向桥、判活、统计 | 总线上的第三个节点，也是唯一出口 |
| collecton_node | STM32F103 / FreeRTOS | `NODE_A=1` | 100 ms 周期上报 `0x101`（红外/光敏 ADC + seq），1 s 心跳，OLED 显示 | 上行数据源；目前只处理 `0x000` 急停、`0x103` 与心跳，`0x301/0x311` 尚未实现 |
| control_node | STM32F103 / FreeRTOS | `NODE_B=2` | 电机/PWM/PID，上报 `0x102`，1 s 心跳，3 s 收不到 `0x703` 即停车 | 下行命令目标；目前只解析 `0x302`，`0x312/0x322/0x332` 尚未实现 |

## 三、目录结构

```text
gateway_node/
├── CMakeLists.txt              # EXTRA_COMPONENT_DIRS → component/BSP、component/MiddleWare；project(gateway)
├── partitions-16MiB.csv        # 16 MB Flash 分区表：nvs / phy-init / factory / vfs / storage
├── dependencies.lock           # 组件锁：idf 5.5.6 / openai 1.1.0 / cmake_utilities 0.5.3，target=esp32s3
├── .clangd / .devcontainer / .vscode   # clangd、espressif/idf 容器、ESP-IDF 扩展的工程配置
├── main/
│   ├── main.c                  # app_main 共 13 行：LED → CAN → WiFi（阻塞到连上）→ wifi_task
│   ├── CMakeLists.txt          # idf_component_register(SRCS "main.c" INCLUDE_DIRS ".")
│   └── idf_component.yml       # idf >=4.1.0 + espressif/openai ^1.1.0（代码里没用到）
└── component/
    ├── BSP/CMakeLists.txt      # REQUIRES driver（RMT 在 driver 组件里）
    ├── BSP/LED/led.c, led.h    # WS2812 驱动：RMT 10 MHz，GRB 24 bit + 50 µs 复位；LED_GPIO_PIN=GPIO48
    ├── BSP/LED/ledtask.c       # 2 s 翻转一次 (20,0,0) 与熄灭；栈 3072、优先级 5
    └── MiddleWare/
        ├── CMakeLists.txt      # REQUIRES esp_wifi nvs_flash esp_event esp_netif mqtt driver
        ├── CAN/can.c           # 双桥本体（约 1000 行）：分发、MQTT→CAN、周期帧、判活、bus-off、统计
        ├── CAN/can.h           # 3 个对外接口 + can_stats_t 四个计数器与统计口径
        ├── MQTT/mqtt.c, mqtt.h # esp-mqtt 封装；主题宏（前缀 gw0805）、QoS 宏、订阅通配宏
        ├── WIFI/wifi.c, wifi.h # STA 初始化/连接/断线重连/扫描；SSID 与密码写死在宏里
        ├── WIFI/wifitask.c     # 网络任务：等 WiFi → mqtt_init → 每 5 s 上报 status/stats
        └── PROTOCOL/protocol.h # CAN 应用层协议 v3.0：纯头文件，与 A/B 两份逐字节相同
```

## 四、核心模块说明

| 文件 | 职责 | 关键点 |
|---|---|---|
| `main/main.c` | 应用入口 | 顺序固定：`LED_Init` → `LedTask_Start` → `can_init` → `wifi_sta_init`（内部阻塞到拿到 IP 或 15 s 超时）→ `wifiTask_Start`；**CAN 先于 MQTT 就绪**，开机初期遥测会被丢弃 |
| `CAN/can.c` | CAN ⇄ MQTT 双桥、周期帧、判活、bus-off、统计 | 三个任务 + 一个 8 深发布队列；所有"发"都带超时，不永久阻塞 |
| `CAN/can.h` | 3 个接口 + `can_stats_t` | 写明统计口径的两个坑：`PROTO_SeqLost(x,x)=255`、A 重启无法与丢帧区分 |
| `MQTT/mqtt.c` `mqtt.h` | esp-mqtt 封装 | 连上即订阅 `gw0805/+/cmd/#`（QoS 1）；下行在事件回调里直接调 `can_bridge_mqtt()`，最多阻塞 50 ms |
| `WIFI/wifi.c` | STA 初始化、连接、重连、扫描 | `wifi_init_once()` 只做一次全局初始化；`wifi_connect()` 用事件组把异步连接包装成同步；`wifi_scan()` 已实现但调用处被注释掉 |
| `WIFI/wifitask.c` | 网络任务 | 先死等 `wifi_is_connected()`，再 `mqtt_init()`，之后每 5 s 发 `status` 与 `stats` |
| `PROTOCOL/protocol.h` | 协议契约（纯头文件） | 只 include `<stdint.h>`；ID 拼装、DLC、大端编解码、丢帧计算都在这里 |
| `BSP/LED/led.c` | WS2812 驱动 | RMT 硬件波形：0 码 0.4/0.8 µs，1 码 0.8/0.4 µs，24 bit GRB 高位在前 + 1 个复位码点 |
| `BSP/LED/ledtask.c` | 心跳灯 | 2 s 翻转一次；只表示"程序在跑"，不反映 WiFi/MQTT 状态 |
| `CMakeLists.txt` | 工程入口 | `EXTRA_COMPONENT_DIRS` 把 BSP、MiddleWare 变成可 `#include "xx.h"` 的组件 |
| `main/idf_component.yml` | 组件依赖清单 | 声明 `espressif/openai ^1.1.0`，但**代码里没有任何 AI 调用**（模板遗留） |

### FreeRTOS 任务

| 任务 | 入口 | 优先级 | 栈（字节） | 创建方式 | 绑核 | 职责 |
|---|---|---|---|---|---|---|
| `main` | `app_main` | IDF 默认 | IDF 默认 | IDF 启动流程 | 约定 CPU0 | 一次性初始化，返回后任务结束 |
| `led_task` | `ledtask.c` | 5 | 3072 | `xTaskCreate` | 未指定 | 2 s 翻转红色 |
| `can_rx` | `can.c` | 6 | 4096 | `xTaskCreate` | 未指定 | `twai_receive`(100 ms) → 分发；超时分支看护总线状态；每秒打一行收帧汇总 |
| `can_tx` | `can.c` | 4 | 4096 | `xTaskCreate` | 未指定 | 每 1 s 发 `0x103` + `0x703`，并做心跳超时判活 |
| `can_pub` | `can.c` | 3 | 4096 | `xTaskCreate` | 未指定 | **全工程唯一可能阻塞在 MQTT 上的任务**，优先级垫底 |
| `wifi_task` | `wifitask.c` | 5 | 4096 | `xTaskCreate` | 未指定 | 等 WiFi → `mqtt_init` → 5 s 周期上报 |
| `mqtt_task` | esp-mqtt 内部 | 由 IDF 决定 | 由 IDF 决定 | `esp_mqtt_client_start` | 未指定 | keepalive、断线重连、派发事件回调 |

> 本仓库**没有任何 `xTaskCreatePinnedToCore`**：上面每个任务都是 `xTaskCreate`，不绑核，由调度器在双核间分配。网上文档里"Core0 跑 CAN+MQTT、Core1 跑 AI+OTA"是规划，代码里并不存在，AI/OTA 也尚未实现。

### MQTT 主题映射（前缀 `gw0805`）

| 方向 | 主题 | QoS | 来源 / 触发 |
|---|---|---|---|
| 发布 | `gw0805/nodeA/telemetry` | 0 | `0x101` 每帧 → `{"ir":..,"ldr":..,"seq":..}` |
| 发布 | `gw0805/nodeB/telemetry` | 0 | `0x102` 每帧 → `{"motor":..,"pwm":..,"mode":..,"flags":..}` |
| 发布 | `gw0805/nodeA/alert` | 0 | `0x011` → `{"code":..,"extra":..}`；A 心跳超时 → `code=2` |
| 发布 | `gw0805/nodeB/alert` | 0 | `0x012` 同上；B 心跳超时 → `code=2` |
| 发布 | `gw0805/gateway/status` | 0 | `wifi_task` 每 5 s → `{"wifi":0/1,"mqtt":0/1,"cache":0}` |
| 发布 | `gw0805/gateway/stats` | 0 | `wifi_task` 每 5 s → `{"a_rx":..,"a_lost":..,"pub_total":..,"pub_lost":..}` |
| 订阅 | `gw0805/+/cmd/#` | 1 | 连上 broker 时订阅一次；`+` 取 `nodeA/nodeB/all`，`#` 取 `motor/target/pid/mode/estop` |

> 所有发布都是 `retain = 0`、无 LWT（`esp_mqtt_client_publish(..., qos, 0)`，配置里也没有遗嘱），符合"控制命令主题绝不能 retained"的约束。

## 五、CAN 应用层协议（v3.0）

**ID 设计**：CAN 2.0A 标准帧（11 位），`bit10..4` 为功能组（必须是 16 的倍数），`bit3..0` 为节点号（0~15），拼装宏 `CAN_ID(group, node) = group | node`。节点号 `0` 保留广播、`1`=A、`2`=B、`3`=网关；功能组 `0x000` 紧急安全、`0x010` 报警、`0x100` 周期数据、`0x300/0x310/0x320/0x330` 四类控制、`0x700` 心跳。

| ID | DLC | 方向 | 名称 | 本仓库行为 |
|---|---|---|---|---|
| `0x000` | 1 | 广播 | ESTOP（B0：1=急停 2=解除） | 收：只打 WARN、不转发；发：`gw0805/all/cmd/estop` |
| `0x011` | 3 | A→网关 | ALARM_A（B0 码：1 传感器故障 2 通信丢失 3 超阈值 4 执行器故障） | 收 → `nodeA/alert` |
| `0x012` | 3 | B→网关 | ALARM_B | 收 → `nodeB/alert` |
| `0x101` | 5 | A→全体 | A_SENSOR（B0-1 红外 ADC、B2-3 光敏 ADC、B4 seq） | 收 → `nodeA/telemetry`，并按 seq 统计丢帧 |
| `0x102` | 4 | B→全体 | B_STATUS（B0 电机状态、B1 PWM%、B2 模式、B3 flags） | 收 → `nodeB/telemetry` |
| `0x103` | 4 | 网关→全体 | GW_STATUS（B0 WiFi、B1 MQTT、B2-3 未补传缓存条数） | 发：每 1 s，缓存条数见第六节 |
| `0x301` / `0x302` | 3 | 网关→A/B | CTRL_MOTOR（B0 命令 0 停/1 正转/2 反转/3 设速，B1 参数，B2 cid） | 发：`gw0805/nodeA/cmd/motor`、`gw0805/nodeB/cmd/motor` |
| `0x311` / `0x312` | 3 | 网关→A/B | CTRL_TARGET（B0-1 目标值，B2 cid） | 发：`gw0805/nodeA/cmd/target`、`gw0805/nodeB/cmd/target` |
| `0x322` | 7 | 网关→B | CTRL_PID（kp/ki/kd ×100 大端，B6 cid） | 发：`nodeB/cmd/pid` |
| `0x332` | 2 | 网关→B | CTRL_MODE（B0 模式，B1 cid） | 发：`nodeB/cmd/mode` |
| `0x701` / `0x702` | 2 | A/B→全体 | 心跳（B0 魔数 A=`0xA5`、B=`0x5A`，B1 计数） | 收：判活，**不转发 MQTT** |
| `0x703` | 2 | 网关→全体 | 网关心跳（`0xC3`） | 发：每 1 s，同时是 B 的电机保活 |

协议使用要点（均来自 `protocol.h` 与 `can.c`）：

- **大端**：多字节高字节在前，用 `PROTO_PutU16/PROTO_GetU16` 手工移位，禁止 `memcpy(&v,...)` 或指针强转（两端 CPU 都是小端，还会踩非对齐访问）。
- **DLC 严格校验**：接收侧先比 `data_length_code == LEN_xxx`，不等即丢弃并节流打日志；错配持续出现通常意味着对端固件版本不一致。
- **丢帧统计**：`PROTO_SeqLost(new, old) = (uint8_t)((new - old - 1) & 0xFF)`，回绕自动处理；网关只在 `seq` 变化时才算，跳变 `>= A_LOST_RESET_MAX(10)` 判为"A 重启/长时间断线"不计入。
- **cid 幂等**：每条下行控制帧带自增命令序号，执行端记上一次的值、相同即丢弃，抵消 CAN 硬件自动重发导致的重复执行。
- **心跳判活**：`CAN_HB_TIMEOUT_MS = 3000`；从未收到过心跳不判掉线（避免上电误报），只在"在线→掉线"这一刻发一条 `ALM_COMM_LOST`。
- **急停的安全默认**：`0x000` 的 B0 只要不是 `ESTOP_ACTION_CLEAR(2)` 一律按急停处理，负载写空、写错、解析失败都停机。
- **下行覆盖不全**：`pid`/`mode` 只定义了 B 的 ID——发 `gw0805/nodeA/cmd/pid` 会算出未定义的 `0x321`、`gw0805/all/cmd/motor` 得到 `0x300`，二者都落到 `default` 分支打"暂不支持的命令 ID"；而**对端目前只有 B 解析 `0x302`**，`0x301/0x311/0x312/0x322/0x332` 尚无接收实现（网关会照发，节点侧不会动作）。

## 六、断网缓存与掉电续传

现场的现实是：WiFi 会抖、broker 会连不上、配电柜随时可能断电。任何一段"发不出去的遥测"如果就地丢弃，就是永久的数据空洞。缓存层要同时解决两个不同的问题：

| 问题 | 数据此刻在哪 | 恢复目标 |
|---|---|---|
| **断网** | 还在 RAM 里，等网络恢复 | 网络恢复后按时间顺序补传，不重不漏 |
| **掉电** | 可能只写了一半 | 任意时刻断电，重启后从最后一条完整记录继续 |

### 6.1 缓存结构：环形缓冲区 + 序列号

- 每条记录定长：`seq(4B) | ts(4B) | can_id(2B) | dlc(1B) | data(8B) | crc(2B)`，一帧 CAN 报文对应一条记录（约 21 B）。
- `seq` 由网关统一分配、单调递增，**既是补传顺序，也是云端去重的依据**。
- 写指针 / 读指针 / 未补传条数三个量构成遍历状态；写满后按"覆盖最旧"推进（最新数据优先到云），被覆盖的条数单独计入 `dropped`。
- 缓存与发布解耦：`can_pub` 只负责投递，落盘与补传走独立路径，不占总线接收与心跳的实时性——这与第四节的发布队列解耦是同一条原则。

### 6.2 补传：按序重放 + 云端幂等

- 补传的触发点是 MQTT 的 `MQTT_EVENT_CONNECTED`：连上先补历史，再切回实时流。
- 按 `seq` 升序批量发送（每批 N 条、发完等确认），并**限速**，避免网络刚恢复就把 WiFi/TCP 打满、反过来拖垮总线侧。
- 投递用 QoS 1（至少一次），云端按 `seq` 幂等丢弃重复 → 两者合起来才是"不重不漏"。
- 顺序控制：补传完成前，实时遥测继续进缓冲，补完整后一并推进读指针，避免"新数据先到、旧数据插队"造成时间乱序。

### 6.3 掉电续传：先写数据，后提交索引

关键在**写入顺序**与**校验**——要保证不管在哪一刻掉电，重启后都能判断出"最后一条有效记录在哪"：

1. 先写数据记录，记录尾部带**魔数 + CRC**；
2. 记录完整落盘后，才更新并提交索引 / 写指针（索引区自带校验）；
3. 掉电发生在第 1 步 → 索引没动，重启只认索引之前的数据，尾部残块扫得到但 CRC 不过 → 丢弃；
4. 掉电发生在第 2 步 → 索引要么是旧值、要么是新值，两种都能定位到完整的最后一条记录。

- 索引采用双副本交替提交，提交时只翻转有效标记，避免单点写坏后失去全部基准。
- 上电初始化顺序：挂载分区 → 校验魔数 → 从索引区恢复读写指针 → 扫描尾部记录做 CRC 校验 → 丢弃半写残块 → 恢复未补传计数。
- 分区表里 `vfs`（FAT，10 MB）与 `storage`（SPIFFS，4 MB）就是为这一层预留的空间：大块顺序写走 `vfs`，索引与配置等小数据走 `storage` / `nvs`。

### 6.4 对外可观测

| 出口 | 字段 | 含义 |
|---|---|---|
| CAN `0x103` GW_STATUS | B2-B3 `cache_cnt` | 未补传条数（大端） |
| MQTT `gw0805/gateway/status` | `cache` | 未补传条数，云端与手机端可直接看到积压 |
| MQTT `gw0805/gateway/stats` | `pub_total` / `pub_lost` | 发布总数与失败数，可算发布丢包率 |

> 状态帧与主题里的字段取自同一个计数器，`0x103` 每秒发一次、`status` 每 5 s 发一次，两级观测都能看到积压趋势。

### 6.5 缓存的边界

- 缓存只解决"已采到的数据不丢"，不解决"总线本身丢帧"——后者由 `seq` 丢帧统计单独度量（见第五节）。
- **控制命令不进缓存**：命令有时效性，断网期间重放几分钟前的"电机正转"是危险的，这也是控制主题必须 `retain = 0` 的原因。
- 缓存满导致的丢弃与网络导致的丢弃分开统计，便于区分是"存不下"还是"发不出"。

## 七、硬件与引脚

| 信号 | GPIO | 说明 |
|---|---|---|
| CAN TXD | GPIO5 | → 收发器 TXD；`TWAI_GENERAL_CONFIG_DEFAULT(TX, RX, mode)`，参数顺序别写反 |
| CAN RXD | GPIO4 | ← 收发器 RXD；总线"只发不收"时优先查这条 |
| WS2812 数据 | GPIO48 | 板载彩灯，RMT 通道输出 |
| 其他占用 | 19/20、43/44、0/3/45/46 | USB-Serial-JTAG、控制台串口、Strapping，工程均未使用 |
| 复用提示 | GPIO4/5 | 同时是 ADC1_CH3/CH4，本项目不用 ADC |

接线：GPIO5→收发器 TXD、GPIO4←RXD，**必须共地**，总线两端各一个 120Ω 终端电阻。ESP32-S3 只有 CAN 控制器、没有物理层，必须外挂收发器（`can.h` 点名 TJA1050 / SN65HVD230 / MCP2551）；其中 TJA1050 是 5 V 器件（TXD 高电平门限约 0.7×VDD ≈ 3.5 V），用 3.3 V GPIO 驱动时要注意电平是否被识别，SN65HVD230 是 3.3 V 器件、天然匹配。

位定时（`can.c` 里逐项对齐 STM32 端，没有用 IDF 的 `TWAI_TIMING_CONFIG_250KBITS()`）：

| 项 | 值 | 说明 |
|---|---|---|
| 时钟源 / 量化频率 | `TWAI_CLK_SRC_DEFAULT` / 4 MHz | `brp = 0`，由驱动按 `quanta_resolution_hz` 反算分频 |
| tseg_1 / tseg_2 | 13 / 2 | 加同步段共 16 tq → 4 MHz / 16 = **250 kbps** |
| sjw / triple_sampling | 2 / false | 与 STM32 的 `CAN_SJW_2TQ` 一致；单次采样 |
| 采样点 | (1+13)/16 = **87.5%** | IDF 的 250 kbps 宏是 80%，与 STM32 的 87.5% 差 7.5%，混跑会通不了 |
| 过滤器 / 驱动队列 | `TWAI_FILTER_CONFIG_ACCEPT_ALL()` / tx 5、rx 5 | 全收（只听心跳可只比较 `GRP_HEARTBEAT` 高 7 位）；队列用 `TWAI_GENERAL_CONFIG_DEFAULT` 默认值，代码未改 |


## 八、技术栈与技术亮点

| 层次 | 技术 | 用途 |
|---|---|---|
| 芯片 / SDK | ESP32-S3、ESP-IDF 5.5.6 | 无线 + 双核 MCU 平台 |
| RTOS | FreeRTOS（`xTaskCreate`、`Queue`、`EventGroup`、tick 计数） | 6 个应用任务 + esp-mqtt 内部任务 |
| CAN / 外设 | TWAI 驱动、RMT 驱动 | 250 kbps 标准帧收发、bus-off 恢复、WS2812 波形 |
| 网络 | esp_wifi STA + lwIP + esp-mqtt（MQTT 5.0） | WiFi 连接、TCP、broker 收发 |
| 存储 | NVS / FATFS（`vfs`）/ SPIFFS（`storage`）分区 | 断网缓存与掉电续传的持久化、索引提交 |
| 协议 / 工具链 | 自研 `protocol.h` v3.0、CMake 组件化、clangd、OpenOCD | 三端共用契约、构建与调试 |

技术亮点：

- **发布队列解耦**：`esp_mqtt_client_publish()` 不是"入队即返回"，它在调用者任务里同步写 socket 并全程持 `MQTT_API_LOCK`，网络卡住时最长等 `network_timeout_ms`（默认 10 s）。放在收帧任务里的后果是 TWAI 队列没人排空、`0x703` 停发、A/B 判网关掉线、B 停电机；因此改成"收帧只拷进 8 深队列（`xQueueSend` 超时 0）+ 优先级 3 的 `can_pub` 专发"，网络抖动最多丢几帧遥测。
- **位时序逐项对齐 STM32**：不用 IDF 的 250 kbps 宏（80% 采样点），而按 STM32 的 `36 MHz / 9 = 4 MHz`、`1+13+2 = 16 tq`、采样点 87.5%、SJW=2 手工填 `twai_timing_config_t`。三端采样点一致是"两个 STM32 互收正常、ESP32 一挂就不通"这类问题的正解。
- **bus-off 状态机补全**：驱动恢复后落到的是 `STOPPED` 而非 `RUNNING`，必须再 `twai_start()` 一次，否则控制器永远卡死。逻辑放在接收任务的 100 ms 超时分支（总线空闲时执行），并把 `bus_error_count / arb_lost_count / TEC / REC` 打出来，用于区分"没人应答"还是"RX 引脚是聋的"。
- **日志按帧类型节流**：`can_log_allow()` 给 5 类帧各记一个时刻，每类最多 1 条/秒；`seen` 标志保证上电第一条一定打出来；`CAN_LOG_PERIOD_MS` 改 0 就变成逐帧调试。
- **安全与统计的默认值设计**：急停解析失败一律按"停"处理（协议层写成"接收方硬性义务"）；`can.h` 写清重复帧与 A 重启两个会永久污染累计值的坑，并说明 `can_get_stats()` 为何按值返回。
- **断网与掉电两级可靠性**：断网一期靠"环形缓冲 + 按 seq 补传 + 云端幂等"做到不重不漏；掉电一期靠"先写数据、后提交索引 + 魔数/CRC"把恢复点收敛到最后一条完整记录。两级都只作用于上行遥测，控制命令不入缓存，从机制上排除"过期命令被重放"。

## 九、典型问题与排查

| 现象 | 根因 | 定位手法 | 处理 |
|---|---|---|---|
| **网络一抖，CAN 侧整体"装死"**：WiFi 抖动或 broker 不回 ACK 后约 10 s 内串口只打 `1s 收 0 帧`，A/B 判网关掉线，B 电机停 | 当时在 `can_rx_task` 里直接调 `mqtt_publish()` → `esp_mqtt_client_publish()`（持 `MQTT_API_LOCK`）→ `esp_mqtt_write` → `tcp_write` → `select(...,10 s)`；这 10 s 没人排空 TWAI 队列，`can_tx_task` 又卡在同一把锁上，`0x703` 停发 | `can.c` 的「遥测发布（队列解耦）」注释里画了完整调用链；现场表现为"收帧计数归零但任务没死"；抓栈能看到 `can_rx` 停在 `mqtt_publish` | 已改为队列解耦（`pub_telemetry` 只做一次拷贝，`xQueueSend` 超时 0）+ `can_pub` 优先级 3 垫底；再看 `遥测队列满，丢弃 N 条` 这条限流日志，它说明"MQTT 忙但 CAN 没被拖住" |
| **TX 全部超时、BUS_OFF 反复出现**：`TX 0x103 超时：总线忙或无人应答`，随后 `总线离线(BUS_OFF) 累计第 N 次`，A/B 收不到网关任何帧 | 发出的帧没有节点回 ACK，TEC 超过 255 触发 BUS_OFF。`can.c` 逐条列出：没接收发器 / 收发器没供电 / TX-RX 接反 / 没共地 / 对端没上电 / CANH-CANL 接反 / 终端电阻过多或缺失；用 TJA1050 这类 5 V 器件时还要注意 3.3 V 电平是否被识别为高 | 看 `can_service_bus_state()` 打出的 `总线错误 / 仲裁丢失 / 收错 / TEC / REC`：**`bus_error_count` 恒为 0 说明 RX 根本没看到线上电平变化**（查收发器供电、GPIO4→RXD、共地）；`arb_lost_count > 0` 说明能看到别人在发，问题在发送侧或 ACK；也可把 `CAN_MODE` 改成 `TWAI_MODE_LISTEN_ONLY` 单独验证接收（此模式 `twai_transmit` 直接返回不支持，`can_tx_task` 发了也没用） | 先修 RX 通路与共地，再确认两端各一个 120Ω；恢复不必重启：`BUS_OFF → twai_initiate_recovery() → RECOVERING → STOPPED → twai_start()` 会自动重来，接线一好下一秒就恢复 |
| **A 节点丢帧数暴涨**：`a_lost` 单次增加 255，或 `A seq 跳变 200 → 0 (55 帧)` 后丢包率算出大于 100% | ① CAN 硬件自动重发产生**重复帧**，`seq` 与上一帧相同，而 `PROTO_SeqLost(x, x)` 结果是 255 不是 0；② A 重启后 `seq` 从 0 重来，被当成一次巨大跳变 | 看两条 WARN：`A 节点数据丢帧 N 个 (seq a → b)`（真丢帧）与 `A seq 跳变 a → b (N 帧)，判为疑似重启，不计入丢包`（被阈值挡掉）；255 反复出现即漏判重复帧 | 先判 `s.seq != s_last_seq` 再算差值；`lost >= A_LOST_RESET_MAX(10)`（A 是 100 ms 一帧，10 ≈ 1 秒）判为疑似重启不计入，只有 1~9 才累加。`seq` 只有 8 位且帧里没有 `boot_id`，重启与长断线无法严格区分，只能用阈值近似（`can.h` 已注明） |
| **WiFi 掉线后重连日志自相矛盾**：总是 `第1次连接失败` + 固定 15 s 后的 `连接 X 超时（15000 ms）`，但网络其实通了；同一次断开打两遍日志 | `wifi_event_handler` 在 `WIFI_EVENT_STA_DISCONNECTED` 回调里**同步**调 `wifi_connect()`，而它用 `xEventGroupWaitBits` 阻塞 15 s；回调占着事件循环任务，期间 `IP_EVENT_STA_GOT_IP` 派发不出去，必然走超时分支。另外 `wifi_connect()` 入口把 `s_retry_num` 清 0（永远是"第 1 次"），`WIFI_FAIL_BIT` 从未被 `Set`（死代码），`disc` 取了两次 | 看时间戳：`断开连接` 与 `拿到 IP` 之间恒为 15 s；或断点停在 `xEventGroupWaitBits` 看调用栈来自事件循环任务 | 回调里只置位/置标志，重连交给独立任务或定时器（或开 `esp_wifi_set_auto_reconnect`），重试计数移到回调里维护并设上限 |
