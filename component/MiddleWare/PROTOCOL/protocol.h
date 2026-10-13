#ifndef __PROTOCOL_H
#define __PROTOCOL_H

/* ============================================================
 * protocol.h —— CAN 通信协议  v3.0
 *
 * ★ STM32(A/B) 和 ESP32(网关) 必须用完全一样的这一份 ★
 *   改了一边忘了另一边 = 收到一堆垃圾数据
 *
 * ------------------------------------------------------------
 * 【这是一个"纯头文件库"】
 * 本文件只有宏和 static inline 函数，没有 .c，也没有全局变量。
 * 每个包含它的 .c 都会各自拿到一份函数副本(static inline)，
 * 所以不会重复定义、不会链接冲突 —— 这正是协议定义该有的形态。
 *
 * 【可移植性硬性要求】
 * 只允许 include <stdint.h>(标准 C，任何平台都有)。
 * 绝对不要在这里 include "stm32f1xx_hal.h" 之类的东西 ——
 * ESP32 那边没有 HAL，一加这个头文件就直接编不过了。
 * 同理，这里不能出现任何 HAL_xxx / 寄存器操作。
 *
 * ------------------------------------------------------------
 * 【ID 设计：为什么要拆成"功能组 + 节点号"】
 *
 *   如果每个节点的每种数据都拍一个 ID(0x101、0x102、0x103...),
 *   节点一多 ID 就会用完、而且看一个 ID 根本猜不出它是什么。
 *   所以把 11 位 ID 切成两段含义：
 *
 *     bit:  10 9 8 7 6 5 4 | 3 2 1 0
 *           └──── 功能组 ────┘└─ 节点号 ─┘
 *           高 7 位, 16 的倍数   低 4 位, 0~15
 *
 *   - 功能组 = "这是什么数据 / 干什么用"(周期数据? 电机命令? 心跳?)
 *   - 节点号 = "谁发的 / 发给谁"
 *
 *   好处：CAN 的接收过滤器是按"掩码"工作的。
 *   想只听 A 节点的所有数据 → 掩码比较低 4 位 == NODE_A 即可；
 *   想只听所有人的心跳    → 比较高 7 位 == GRP_HEARTBEAT 即可。
 *   分组规则一旦定死，加新节点只是把节点号 +1，过滤器不用重写。
 *
 *   例：0x101 = GRP_PERIODIC(0x100) | NODE_A(1)
 *              = "周期数据" + "来自 A 节点"
 *
 * ------------------------------------------------------------
 * 【线上格式约定】(跨平台的关键，改了就两端对不上)
 *
 *   1. 帧类型：CAN 2.0A 标准帧(11 位 ID)，不是扩展帧(29 位)。
 *      所以 ID 最大 0x7FF，本文件里所有 ID 都没超过。
 *   2. 波特率：250kbps —— 两端必须一致，不一致会满屏错误帧。
 *   3. 多字节【大端】：高字节在前，和 CANopen / J1939 一致。
 *      注意 STM32 和 ESP32 都是小端 CPU，所以不能直接把 uint16_t
 *      的地址强转成 uint8_t* 发出去 —— 必须走下面的
 *      PROTO_PutU16 / PROTO_GetU16 逐字节转换。
 *   4. 传感器值直接传 ADC 原始值 0~4095，不做物理量换算。
 *      换算(电压、百分比)放到显示端/网关做，协议层保持"生数据"，
 *      这样换传感器不用改协议。
 *   5. 每个帧的 DLC 必须和 LEN_xxx 完全一致。
 *      接收方通常会先检查 frame.len == LEN_xxx 再解析，
 *      长度对不上会被直接丢弃(见 freertos.c 的 canRxTask)。
 *
 * ------------------------------------------------------------
 * 【改协议时请遵守】
 *   - 只增不改：先加新的 ID/字段，等两端都升级完再停用旧的。
 *   - 改了任何帧的字节布局或 DLC，务必同步更新本文件里的
 *     "帧总表"和该帧下方的字节表格，并通知 ESP32 那边。
 * ============================================================ */

#include <stdint.h>

/* ==================== 节点号（ID 低 4 位） ====================
 * 占 4 位，所以最多 16 个节点，0~15。
 */

#define NODE_BROADCAST      0u      /* 0 号保留给广播：全体都收，谁都不该用它当自己的地址 */
#define NODE_A              1u      /* 11111 传感器节点 */
#define NODE_B              2u      /* 22222 执行节点   */
#define NODE_GW             3u      /* ESP32-S3 网关    */
/* 4~15 预留：以后加节点直接往下排，不用动分组规则 */

/* ==================== ID 拼装宏：功能组 + 节点号 ====================
 *
 * 用 | 而不是 + 把两段拼起来。因为两段在二进制上不重叠：
 *   功能组 = 16 的倍数(低 4 位恒为 0)
 *   节点号 = 0~15      (高 7 位恒为 0)
 * 所以 | 和 + 结果相同，但 | 更明确地表达"这是往空位里填位"。
 *
 * ★ 加新的 GRP_xxx 时，值必须是 16 的倍数(低 4 位为 0) ★
 *   否则会和节点号那 4 位打架。比如 GRP = 0x001 配 NODE_A：
 *   (0x001 | 0x001) 还是 0x001，两个不同节点会撞成同一个 ID。
 *
 * ★ 上界 ★ 标准帧 ID 只有 11 位，group | node 不能超过 0x7FF。
 *   本文件最大的 GRP_HEARTBEAT = 0x700，加节点号后仍是 0x70x，安全。
 *
 * 用法：CAN_ID(GRP_PERIODIC, NODE_A)  →  0x101
 */
#define CAN_ID(group, node) ((uint32_t)((group) | (node)))

/* ==================== 功能组基址（高 7 位） ====================
 *
 * 按"用途"划分，同时留出 0x10 的间隔方便以后往组内插新帧。
 * 数值本身也带一点语义上的顺序：0x0xx 是紧急/报警，
 * 0x1xx 是节点自发上报，0x3xx 是控制端下发的命令，0x7xx 是心跳。
 */
#define GRP_ESTOP           0x000u  /* 紧急安全 */
#define GRP_ALARM           0x010u  /* 事件报警 */
#define GRP_PERIODIC        0x100u  /* 周期数据 */
#define GRP_CTRL_MOTOR      0x300u  /* 电机命令 */
#define GRP_CTRL_TARGET     0x310u  /* 目标值/阈值 */
#define GRP_CTRL_PID        0x320u  /* PID 参数 */
#define GRP_CTRL_MODE       0x330u  /* 模式/配置 */
#define GRP_HEARTBEAT       0x700u  /* 心跳 */

/* ==================== 帧总表（速查） ====================
 *
 * 每行格式：ID | DLC | 发送方 → 接收方 | 名称
 *
 *   0x000 |  1 | 广播 → 全体   | ESTOP     紧急停止 / 解除 (看 B0)
 *   0x011 |  3 | A    → 网关   | ALARM_A   A 节点报警
 *   0x012 |  3 | B    → 网关   | ALARM_B   B 节点报警
 *   0x101 |  5 | A    → 全体   | A_SENSOR  A 的传感器数据(周期)
 *   0x102 |  4 | B    → 全体   | B_STATUS  B 的状态(周期)
 *   0x103 |  4 | 网关 → 全体   | GW_STATUS 网关状态(周期)
 *   0x301 |  3 | 网关 → A      | CTRL_MOTOR_A  控制 A 的电机
 *   0x302 |  3 | 网关 → B      | CTRL_MOTOR_B  控制 B 的电机
 *   0x311 |  3 | 网关 → A      | CTRL_TARGET_A 设置 A 的目标值
 *   0x312 |  3 | 网关 → B      | CTRL_TARGET_B 设置 B 的目标值
 *   0x322 |  7 | 网关 → B      | CTRL_PID_B    设置 B 的 PID
 *   0x332 |  2 | 网关 → B      | CTRL_MODE_B   切换 B 的模式
 *   0x701 |  2 | A    → 全体   | A_HEARTBEAT
 *   0x702 |  2 | B    → 全体   | B_HEARTBEAT
 *   0x703 |  2 | 网关 → 全体   | GW_HEARTBEAT
 *   0x7E0 / 0x7E8 | - | 诊断请求/响应(UDS 惯例，阶段 2/4)
 *
 * 下面每个帧的注释里都有对应的"字节表格"，两张表要对得上。
 */

/* ==================== 具体帧 ID ==================== */

/* --- 紧急安全 ---
 * 急停和解除【共用这一个广播 ID】，靠载荷 B0 区分。
 *
 * ★ 为什么不给它们各分一个 ID ★
 *   GRP_ESTOP = 0x000，而 CAN_ID() 是 group|node。
 *   想区分就得动低 4 位，可低 4 位的 1/2/3 已经是 NODE_A/B/GW 了 ——
 *   0x000|1 = 0x001 会和 A 节点撞成同一个 ID（这正是之前版本
 *   把急停和解除都写成 NODE_BROADCAST 的原因，见下面 ESTOP 那节）。
 *   所以状态只能放载荷里，ID 保持"全 0 广播"。
 *
 *   顺带一个好处：0x000 是最小的标准帧 ID，仲裁时优先级最高，
 *   总线上有别的流量时急停也能抢到总线，这本来也正是急停要的。 */
#define CAN_ID_ESTOP        (CAN_ID(GRP_ESTOP, NODE_BROADCAST))   /* 0x000 广播 */

/* --- 事件报警（阶段 2） ---
 * 报警是"事件触发"，不是周期发送：出事才发一帧，所以 ID 里区分
 * 是哪个节点报的警，网关收到后能直接看出源头。 */
#define CAN_ID_ALARM_A      (CAN_ID(GRP_ALARM, NODE_A))           /* 0x011 */
#define CAN_ID_ALARM_B      (CAN_ID(GRP_ALARM, NODE_B))           /* 0x012 */

/* --- 周期数据 ---
 * 节点定期(如 10Hz)上报自身状态，网关和对方节点都能听。 */
#define CAN_ID_A_SENSOR     (CAN_ID(GRP_PERIODIC, NODE_A))        /* 0x101 */
#define CAN_ID_B_STATUS     (CAN_ID(GRP_PERIODIC, NODE_B))        /* 0x102 */
#define CAN_ID_GW_STATUS    (CAN_ID(GRP_PERIODIC, NODE_GW))       /* 0x103 */

/* --- 控制指令（目标节点在 ID 低 4 位） ---
 * 这里节点号的含义从"谁发的"变成"发给谁"：
 * 控制帧一律由网关下发，所以低 4 位用来标识目标执行节点。 */
#define CAN_ID_CTRL_MOTOR_A (CAN_ID(GRP_CTRL_MOTOR,  NODE_A))     /* 0x301 */
#define CAN_ID_CTRL_MOTOR_B (CAN_ID(GRP_CTRL_MOTOR,  NODE_B))     /* 0x302 */
#define CAN_ID_CTRL_TARGET_A (CAN_ID(GRP_CTRL_TARGET, NODE_A))    /* 0x311 */
#define CAN_ID_CTRL_TARGET_B (CAN_ID(GRP_CTRL_TARGET, NODE_B))    /* 0x312 */
#define CAN_ID_CTRL_PID_B   (CAN_ID(GRP_CTRL_PID,    NODE_B))     /* 0x322 */
#define CAN_ID_CTRL_MODE_B  (CAN_ID(GRP_CTRL_MODE,   NODE_B))     /* 0x332 */

/* --- 心跳 ---
 * 每个节点定期发一帧，携带魔数。
 * 目的：① 让对方知道"我还活着"；② 超时收不到就判定掉线(ALM_COMM_LOST)。
 * 魔数的作用是防止把别的帧误当心跳 —— 收到 ID 对了但魔数不对，
 * 说明是脏数据或版本不匹配，直接忽略。 */
#define CAN_ID_A_HEARTBEAT  (CAN_ID(GRP_HEARTBEAT, NODE_A))       /* 0x701 */
#define CAN_ID_B_HEARTBEAT  (CAN_ID(GRP_HEARTBEAT, NODE_B))       /* 0x702 */
#define CAN_ID_GW_HEARTBEAT (CAN_ID(GRP_HEARTBEAT, NODE_GW))      /* 0x703 */

/* --- 诊断/升级（UDS 惯例，阶段 2/4） ---
 * 0x7E0/0x7E8 是汽车行业 ISO 14229(UDS) 的诊断请求/响应 ID，
 * 直接沿用惯例，方便以后接标准的诊断工具。 */
#define CAN_ID_DIAG_REQ     0x7E0u
#define CAN_ID_DIAG_RSP     0x7E8u

/* ==================== DLC（数据长度） ====================
 * 每个帧的字节数。CAN 一帧最多 8 字节，所以这里的值都在 0~8。
 * 命名规则：LEN_<帧名>，和上面的 CAN_ID_<帧名> 一一对应。
 *
 * 注意 DLC 也是协议的一部分：发送方按这个长度打包，
 * 接收方按这个长度校验，两边对不上就当没收到。
 */
#define LEN_ESTOP           1u      /* 0x000 */
#define LEN_ALARM           3u      /* 0x011 / 0x012 */
#define LEN_A_SENSOR        5u      /* 0x101 */
#define LEN_B_STATUS        4u      /* 0x102 */
#define LEN_GW_STATUS       4u      /* 0x103 */
#define LEN_CTRL_MOTOR      3u      /* 0x301 / 0x302 */
#define LEN_CTRL_TARGET     3u      /* 0x311 / 0x312 */
#define LEN_CTRL_PID        7u      /* 0x322 */
#define LEN_CTRL_MODE       2u      /* 0x332 */
#define LEN_HEARTBEAT       2u      /* 0x701 / 0x702 / 0x703 */

/* ==================== 心跳魔数 ====================
 * 每个节点一个独特的值，收心跳时先核对。
 * 只对 ID 不对魔数 → 数据不可信，丢弃。
 */
#define HB_MAGIC_A          0xA5u
#define HB_MAGIC_B          0x5Au
#define HB_MAGIC_GW         0xC3u

/* ==================== 枚举值 ====================
 * 这些是"载荷里某个字节"的取值含义。
 * 区分两类：
 *   - 单值枚举：整个字节只有一个含义，用 == 比较
 *   - 位标志(flags)：一个字节里每一位一个含义，可以叠加，
 *     必须用 & 判断、用 | 组合(见 ST_xxx)
 */

/* CTRL_MOTOR 的 B0：电机命令 */
#define MOTOR_STOP          0u      /* 停 */
#define MOTOR_FWD           1u      /* 正转 */
#define MOTOR_REV           2u      /* 反转 */
#define MOTOR_SETSPEED      3u      /* 调速，具体速度看 B1 */

/* B_STATUS 的 B0（电机实际状态，是"回读"不是"命令"） */
#define MST_IDLE            0u      /* 停着 */
#define MST_FWD             1u      /* 正在正转 */
#define MST_REV             2u      /* 正在反转 */

/* B_STATUS 的 B2（工作模式） */
#define MODE_MANUAL         0u      /* 手动：听网关的命令 */
#define MODE_AUTO           1u      /* 自动：本地按传感器自己决策 */

/* B_STATUS 的 B3（状态位，按位或）
 * 这几位是可叠加的，比如"通信正常 + 已触发阈值" = ST_COMM_OK | ST_THRESH。
 * 判断某个位：if (flags & ST_ALARM) ...
 * 千万别写成 if (flags == ST_ALARM)，那样会漏掉"还带了别的位"的情况。 */
#define ST_COMM_OK          (1u << 0)   /* bit0：和网关通信正常 */
#define ST_THRESH           (1u << 1)   /* bit1：已触发阈值 */
#define ST_ALARM            (1u << 2)   /* bit2：有报警未处理 */

/* ESTOP 的 B0（急停动作） */
#define ESTOP_ACTION_STOP   1u      /* 急停：立刻停机 */
#define ESTOP_ACTION_CLEAR  2u      /* 解除急停：恢复运行 */

/* ALARM 的 B0（报警码） */
#define ALM_SENSOR_FAULT    1u      /* 传感器坏了/读数离谱 */
#define ALM_COMM_LOST       2u      /* 心跳超时，判定掉线 */
#define ALM_THRESH_OVER     3u      /* 超过阈值 */
#define ALM_ACTUATOR_FAULT  4u      /* 执行器(电机)故障 */

/* ==================== 大端编解码 ====================
 *
 * 【为什么需要这两个函数？】
 * 协议规定线上是多字节大端，而 STM32(ARM) 和 ESP32(Xtensa/RISC-V)
 * 的 CPU 都是小端。如果偷懒这样写：
 *
 *     memcpy(d, &value, 2);          // ✗ 直接把内存搬过去
 *     *(uint16_t *)d = value;        // ✗ 强转指针
 *
 * 在 0x1234 上会发出 34 12(小端)，和协议要求的高字节在前正好相反。
 * 更糟的是 *(uint16_t *)d 这种写法还可能因为数组没按 2 字节对齐
 * 而触发"非对齐访问"硬件异常。
 *
 * 所以统一用"手动移位"的方式编解码：显式、可移植、与 CPU 字节序无关，
 * 也避开了对齐问题。收发两端都用这两个函数，就绝对不会错。
 */
static inline void PROTO_PutU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);          /* 高字节在前 */
    p[1] = (uint8_t)(v & 0xFF);
}

static inline uint16_t PROTO_GetU16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ==================== 0x101 A_SENSOR ====================
 * 方向：A → 全体(周期发送)
 * DLC ：5
 *
 * 字节布局：
 *   B0-B1  红外传感器 ADC 原始值 0~4095  (大端)
 *   B2-B3  光敏传感器 ADC 原始值 0~4095  (大端)
 *   B4     序列号 seq，0~255 循环自增
 *
 * 方向提醒：光敏 越大 = 越暗；红外 越大 = 无遮挡。
 * 用 seq 算丢帧：每发一帧 +1，接收方比对两次的差值就知道中间丢了几帧
 * (见文件末尾的 PROTO_SeqLost)。
 */
static inline void PROTO_PackASensor(uint8_t *d, uint16_t ir, uint16_t ldr, uint8_t seq)
{
    PROTO_PutU16(d + 0, ir);
    PROTO_PutU16(d + 2, ldr);
    d[4] = seq;
}

/* 解包后的存放结构。字段名和上面的参数一一对应。 */
typedef struct {
    uint16_t ir;      /* 红外 ADC 原始值 */
    uint16_t ldr;     /* 光敏 ADC 原始值 */
    uint8_t  seq;     /* 序列号 */
} A_Sensor_t;

static inline void PROTO_UnpackASensor(const uint8_t *d, A_Sensor_t *s)
{
    s->ir  = PROTO_GetU16(d + 0);
    s->ldr = PROTO_GetU16(d + 2);
    s->seq = d[4];
}

/* ==================== 0x102 B_STATUS ====================
 * 方向：B → 全体(周期发送)
 * DLC ：4
 *
 * 字节布局：
 *   B0  电机状态  取 MST_xxx
 *   B1  当前 PWM  0~100(百分比，不是寄存器值)
 *   B2  工作模式  取 MODE_xxx
 *   B3  状态位    ST_xxx 的按位或，判断时用 &
 */
static inline void PROTO_PackBStatus(uint8_t *d, uint8_t motor, uint8_t pwm,
                                     uint8_t mode, uint8_t flags)
{
    d[0] = motor;
    d[1] = pwm;
    d[2] = mode;
    d[3] = flags;
}

typedef struct {
    uint8_t motor;    /* MST_xxx */
    uint8_t pwm;      /* 0~100 */
    uint8_t mode;     /* MODE_xxx */
    uint8_t flags;    /* ST_xxx 位或 */
} B_Status_t;

static inline void PROTO_UnpackBStatus(const uint8_t *d, B_Status_t *s)
{
    s->motor = d[0];
    s->pwm   = d[1];
    s->mode  = d[2];
    s->flags = d[3];
}

/* ==================== 0x103 GW_STATUS ====================
 * 方向：网关 → 全体(周期发送)
 * DLC ：4
 *
 * 字节布局：
 *   B0     WiFi 状态(0=断开, 非0=已连)
 *   B1     MQTT 状态(0=断开, 非0=已连)
 *   B2-B3  本地缓存条数 0~65535(大端)
 *          断网时数据先存本地，这个数字告诉别的节点"我积压了多少"。
 */
static inline void PROTO_PackGwStatus(uint8_t *d, uint8_t wifi, uint8_t mqtt,
                                      uint16_t cache_cnt)
{
    d[0] = wifi;
    d[1] = mqtt;
    PROTO_PutU16(d + 2, cache_cnt);
}

/* ==================== 0x701/0x702/0x703 心跳 ====================
 * 方向：各节点 → 全体(周期发送，比数据帧频率低)
 * DLC ：2
 *
 * 字节布局：
 *   B0  魔数 取 HB_MAGIC_x(每个节点不同)
 *   B1  计数器，每发一帧 +1，0~255 循环
 *
 * 接收方判断逻辑：ID 对 + 魔数对 才算心跳有效；
 * 若超过若干周期没收到 → 判定该节点掉线，报 ALM_COMM_LOST。
 * B1 除了计数，也能用来查心跳丢了几个，算法同 PROTO_SeqLost。
 */
static inline void PROTO_PackHeartbeat(uint8_t *d, uint8_t magic, uint8_t cnt)
{
    d[0] = magic;
    d[1] = cnt;
}

/* ==================== 0x302 CTRL_MOTOR（目标已在 ID 里） ====================
 * 方向：网关 → A/B(由 ID 低 4 位决定收件人)
 * DLC ：3
 *
 * 字节布局：
 *   B0  命令      取 MOTOR_xxx
 *   B1  参数      命令为 MOTOR_SETSPEED 时表示速度 0~100，其余命令忽略此字节
 *   B2  命令序号 cid
 *
 * 【cid 是干什么的】
 * CAN 有自动重发，网络抖动时可能收到重复帧。执行端记住上一次的 cid，
 * 相同就丢弃，避免"电机反转命令被执行两次"这类问题 —— 这叫幂等。
 */
static inline void PROTO_PackCtrlMotor(uint8_t *d, uint8_t cmd, uint8_t arg, uint8_t cid)
{
    d[0] = cmd;
    d[1] = arg;
    d[2] = cid;
}

typedef struct {
    uint8_t cmd;        /* MOTOR_xxx */
    uint8_t arg;        /* SETSPEED 时为速度 0~100 */
    uint8_t cid;        /* 命令序号，用于幂等 */
} Ctrl_Motor_t;

static inline void PROTO_UnpackCtrlMotor(const uint8_t *d, Ctrl_Motor_t *c)
{
    c->cmd = d[0];
    c->arg = d[1];
    c->cid = d[2];
}

/* ==================== 0x312 CTRL_TARGET ====================
 * 方向：网关 → A/B
 * DLC ：3
 *
 * 字节布局：
 *   B0-B1  目标值 0~65535(大端)
 *   B2     命令序号 cid
 *
 * 目标值具体含义随节点而定：A 节点可能是光照阈值，
 * B 节点可能是目标转速。协议层不关心，只管搬运。
 */
static inline void PROTO_PackCtrlTarget(uint8_t *d, uint16_t target, uint8_t cid)
{
    PROTO_PutU16(d + 0, target);
    d[2] = cid;
}

/* 只取目标值，不关心中间的 cid */
static inline uint16_t PROTO_GetTarget(const uint8_t *d)
{
    return PROTO_GetU16(d + 0);
}

/* ==================== 0x322 CTRL_PID ====================
 * 方向：网关 → B
 * DLC ：7
 *
 * 字节布局：
 *   B0-B1  Kp × 100 (大端)
 *   B2-B3  Ki × 100 (大端)
 *   B4-B5  Kd × 100 (大端)
 *   B6     命令序号 cid
 *
 * 【为什么乘 100 传整数，不直接传 float？】
 *   1) 浮点在线上的字节序、格式都没有统一标准，两端容易对不上；
 *   2) 不少小单片机传 float 要走软件浮点，慢且占空间。
 *   定死"整数 = 真实值 × 100"，精度 0.01，对 PID 参数足够。
 *   注意：范围是 0~655.35，再大就溢出了。
 *   接收端除回 100.0f 得到真实值(见下面的 Unpack)。
 */
static inline void PROTO_PackCtrlPid(uint8_t *d, uint16_t kp100, uint16_t ki100,
                                     uint16_t kd100, uint8_t cid)
{
    PROTO_PutU16(d + 0, kp100);
    PROTO_PutU16(d + 2, ki100);
    PROTO_PutU16(d + 4, kd100);
    d[6] = cid;
}

/* 解包后是真实浮点值(已经除过 100)，可以直接喂给 PID 计算 */
typedef struct {
    float kp;
    float ki;
    float kd;
    uint8_t cid;
} Pid_Param_t;

static inline void PROTO_UnpackCtrlPid(const uint8_t *d, Pid_Param_t *p)
{
    p->kp  = PROTO_GetU16(d + 0) / 100.0f;
    p->ki  = PROTO_GetU16(d + 2) / 100.0f;
    p->kd  = PROTO_GetU16(d + 4) / 100.0f;
    p->cid = d[6];
}

/* ==================== 0x332 CTRL_MODE ====================
 * 方向：网关 → B
 * DLC ：2
 *
 * 字节布局：
 *   B0  模式 取 MODE_xxx
 *   B1  命令序号 cid
 */
static inline void PROTO_PackCtrlMode(uint8_t *d, uint8_t mode, uint8_t cid)
{
    d[0] = mode;
    d[1] = cid;
}

/* ==================== 0x000 ESTOP ====================
 * 方向：任意 → 广播(全体)
 * DLC ：1
 *
 * 字节布局：
 *   B0  动作 取 ESTOP_ACTION_xxx (1=急停, 2=解除)
 *
 * 一个 ID 承担两个动作，靠 B0 区分；是广播，所以载荷里不用放节点号。
 *
 * 【接收方的硬性义务 —— 安全帧和普通帧的规矩不一样】
 *   B0 读到不认识的值时，必须按"急停"执行，不能按"没收到"处理。
 *   理由：这个字节是唯一区分"停"和"放行"的地方，一旦解析失败就
 *   当成放行，等于给了一个静默失效的急停 —— 比编不过严重得多。
 *   所以下面的两个函数之外，接收方也别自己写 if (b0 == 1) else {放行}。
 *
 * 【发送方的硬性义务】
 *   同理，拿不准填什么就填 ESTOP_ACTION_STOP。停一下是安全的，
 *   误放行不是。
 */
static inline void PROTO_PackEstop(uint8_t *d, uint8_t action)
{
    d[0] = action;
}

static inline uint8_t PROTO_EstopAction(const uint8_t *d)
{
    return d[0];
}

/* ==================== 0x011/0x012 ALARM ====================
 * 方向：A / B → 网关(事件触发，出事才发)
 * DLC ：3
 *
 * 字节布局：
 *   B0     报警码 取 ALM_xxx
 *   B1-B2  附加数据(大端)，含义随报警码而定
 *          例：ALM_THRESH_OVER 时放当时超过阈值的那个读数
 *
 * 是哪个节点报的警，看 ID 低 4 位即可，载荷里不用再放节点号。
 */
static inline void PROTO_PackAlarm(uint8_t *d, uint8_t code, uint16_t extra)
{
    d[0] = code;
    PROTO_PutU16(d + 1, extra);
}

static inline uint8_t PROTO_AlarmCode(const uint8_t *d)
{
    return d[0];
}

/* ==================== 丢帧计算 ====================
 *
 * 返回"上一帧到这一帧之间丢了几帧"。
 *
 * 【原理】发送方每发一帧 seq 就 +1。正常连续收到时：
 *     新 seq = 旧 seq + 1   →  差值 1   →  丢帧数 0
 * 如果中间丢了一帧：
 *     新 seq = 旧 seq + 2   →  差值 2   →  丢帧数 1
 * 所以：丢帧数 = (新 - 旧) - 1
 *
 * 【为什么不用写 if 判断回绕】
 * seq 是 uint8_t，255 之后回绕到 0。看回绕那一步：
 *     旧 255，新 0 → (0 - 255 - 1) = -256
 *     -256 取模 256 正好是 0 → 丢帧数 0 ✓
 * 表达式实际是在 32 位里算完、最后才用 (uint8_t) 截断回 8 位的，
 * 而"截断"就是取模 256 —— 于是回绕自然被处理掉了，不用额外分支。
 * 比手写 if (new < old) 更简洁，也不会有边界漏洞。
 *
 * 【注意】单次最多只能反映 255 帧以内的丢帧；丢超过 255 帧时会
 * 得到偏小的值。周期帧这个范围内已经够用。
 */
static inline uint8_t PROTO_SeqLost(uint8_t new_seq, uint8_t old_seq)
{
    return (uint8_t)((uint8_t)(new_seq - old_seq - 1u) & 0xFFu);
}

#endif /* __PROTOCOL_H */
