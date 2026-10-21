/**
  ******************************************************************************
  * @file    dtu.c
  * @brief   DTU(DR154) 可靠上报云平台实现 —— 契约 v1.2
  *
  * 通过 USART1(RS232, 9600 8N1) 发单行JSON到DR154，DR154以MQTT透传到云端。
  * 云端落库成功后回 agri/device/{id}/ack (upload_ack)，本模块按 '{}' 对象边界解析
  * （兼容 DTU 下行把 '\n' 剥掉的情况，不再依赖尾随换行切帧），校验
  * type=upload_ack / status=stored / msg_id 全等后调用 FS_MarkConfirmed 出队。
  *
  * 上报状态机（契约 §8）：
  *   每 DTU_NEW_RECORD_MS 持久化一条新记录(flash_store)；发送槽≥DTU_SEND_SLOT_MS；
  *   优先补历史(每3条插1条最新)；未确认退避重发(上限300s)；链路疑似故障走周期探测。
  *
  * 可靠性加固（2026-10-21）：
  *   - ACK 解析不再整缓冲 strstr，改为"临界区拷出完整对象→NUL 缓冲→全等比较"；
  *   - 与 USART1 中断共享的接收缓冲在取/移数据时关中断；
  *   - 发送前检查 UART 返回值，串口未发出不算"等待云端 ACK"；
  *   - 退避重发后转 SENDING（原实现停在 BACKOFF 每 tick 重发，退避失效）；
  *   - 指数退避位移做钳位，消除 1u<<(n-1) 溢出 UB；
  *   - boot_id 落盘失败进入"身份存储故障"，禁止生成新记录，避免撞云端历史。
  ******************************************************************************
  */
#include "dtu.h"
#include "flash_store.h"
#include "modbus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

/* ====== USART1 ====== */
extern UART_HandleTypeDef huart1;

/* ====== 上行/下行帧缓冲 ====== */
#define DTU_TX_BUF_SIZE   512
#define DTU_RX_BUF_SIZE   256

static uint8_t  s_rx_byte = 0;      /* USART1 单字节中断缓冲 */
static uint8_t  s_rx_buf[DTU_RX_BUF_SIZE];
static volatile uint16_t s_rx_len = 0;      /* 已收到字节数 */

/* ====== 状态机 ====== */
static Dtu_State_t  s_state = DTU_IDLE;
static uint32_t     s_next_action_ms = 0;   /* 下一个发送/探测动作时刻 */

/* 当前正在等待ACK的 msg_id（boot_id/seq） */
static uint32_t     s_pending_boot = 0;
static uint32_t     s_pending_seq  = 0;
static uint32_t     s_retry_cnt    = 0;
static uint32_t     s_last_send_ms = 0;     /* 上次发送槽时刻 */

/* 补传计数：每3条历史插1条最新 */
static uint32_t     s_hist_sent    = 0;
static uint32_t     s_last_record_ms = 0;   /* 上次持久化新记录时刻 */
static uint32_t     s_seq          = 0;      /* 本次启动内 seq 递增（契约 §4） */

/* 设备身份 */
static uint32_t     s_boot_id = 0;
/* 身份层状态：true=本次 boot_id 已成功落盘并校验，允许生成新记录 */
static bool         s_identity_ok = false;

/* ====== 内部函数 ====== */

/* 按 valid 位图输出一个数值：有效则输出格式化数值，无效输出 null（契约 §3.3）。
   禁止把无效量输出为 0 —— 0 是合法读数（夜间光照、CO2 为 0）。 */
static int Dtu_PutValue(char *buf, size_t pos, size_t maxlen,
                        int valid, const char *fmt, double value)
{
    if (!valid) {
        const char *null_str = "null";
        size_t l = strlen(null_str);
        if (pos + l >= maxlen) return -1;
        memcpy(&buf[pos], null_str, l);
        return (int)l;
    }
    int n = snprintf(&buf[pos], maxlen - pos, fmt, value);
    return (n < 0) ? -1 : n;
}

/* 组上行遥测 JSON（契约 §3 格式）到 buf；返回长度。rec 是待发送记录 */
static uint16_t Dtu_BuildTelemetry(const FS_Record_t *rec, char *buf, uint16_t maxlen)
{
    char  dev[48];   /* "device_id/boot_id/seq"，需容纳最长数字，dev[8] 会截断 msg_id */
    snprintf(dev, sizeof(dev), "%s/%lu/%lu", DTU_DEVICE_ID,
             (unsigned long)rec->boot_id, (unsigned long)rec->seq);

    /* valid 位图 */
    int v[FS_CH_NUM];
    for (int i = 0; i < FS_CH_NUM; i++)
        v[i] = (rec->valid >> i) & 1;

    /* 定点还原（契约 §10.6）：÷倍率。
       光照按 uint32 无符号还原，不走 float 中转（避免 >2^24 丢精度/溢出）。 */
    uint32_t light_u32 = (uint32_t)rec->data[FS_CH_LIGHT];

    size_t pos = 0;
    int n = snprintf(buf, maxlen,
        "{\"type\":\"telemetry\",\"schema_version\":1,\"device_id\":\"%s\","
        "\"msg_id\":\"%s\",\"boot_id\":%lu,\"seq\":%lu,"
        "\"sampled_at\":%lu,\"time_quality\":%d,\"uptime_ms\":%llu,"
        "\"data\":{",
        DTU_DEVICE_ID, dev,
        (unsigned long)rec->boot_id, (unsigned long)rec->seq,
        (unsigned long)rec->sampled_at, (int)rec->time_quality,
        (unsigned long long)rec->uptime_ms);
    if (n < 0 || (size_t)n >= maxlen) return (uint16_t)(maxlen - 1);
    pos = (size_t)n;

    /* 每个通道：有效输出数值，无效输出 null */
    struct { const char *key; int valid; const char *fmt; double val; } fields[] = {
        { "\"temperature\":",     v[FS_CH_TEMPERATURE],    "%.1f", (double)((int32_t)rec->data[FS_CH_TEMPERATURE])    / 10.0 },
        { "\"humidity\":",        v[FS_CH_HUMIDITY],       "%.1f", (double)((int32_t)rec->data[FS_CH_HUMIDITY])       / 10.0 },
        { "\"co2\":",             v[FS_CH_CO2],            "%.0f", (double)((int32_t)rec->data[FS_CH_CO2]) },
        { "\"light\":",           v[FS_CH_LIGHT],          "%.0f", (double)light_u32 },
        { "\"pressure\":",        v[FS_CH_PRESSURE],       "%.1f", (double)((int32_t)rec->data[FS_CH_PRESSURE])       / 10.0 },
        { "\"soil_temperature\":",v[FS_CH_SOIL_TEMP],      "%.1f", (double)((int32_t)rec->data[FS_CH_SOIL_TEMP])      / 10.0 },
        { "\"soil_moisture\":",   v[FS_CH_SOIL_MOISTURE],  "%.1f", (double)((int32_t)rec->data[FS_CH_SOIL_MOISTURE])  / 10.0 },
        { "\"soil_ec\":",         v[FS_CH_SOIL_EC],        "%.0f", (double)((int32_t)rec->data[FS_CH_SOIL_EC]) },
    };

    for (size_t i = 0; i < sizeof(fields)/sizeof(fields[0]); i++) {
        size_t kl = strlen(fields[i].key);
        if (pos + kl >= maxlen) return (uint16_t)(maxlen - 1);
        memcpy(&buf[pos], fields[i].key, kl);
        pos += kl;
        int w = Dtu_PutValue(buf, pos, maxlen, fields[i].valid, fields[i].fmt, fields[i].val);
        if (w < 0) return (uint16_t)(maxlen - 1);
        pos += (size_t)w;
        if (pos + 1 >= maxlen) return (uint16_t)(maxlen - 1);
        buf[pos++] = ',';
    }

    n = snprintf(&buf[pos], maxlen - pos,
        "\"valid\":[%d,%d,%d,%d,%d,%d,%d,%d]}}\n",
        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);
    if (n < 0) return (uint16_t)(maxlen - 1);
    pos += (size_t)n;
    if (pos >= maxlen) pos = maxlen - 1;
    buf[pos] = '\0';
    return (uint16_t)pos;
}

/* 通过 USART1 发送一帧（阻塞发完）。返回 0 已发出，-1 未发出 */
static int Dtu_Send(const uint8_t *data, uint16_t len)
{
    /* 9600bps 下 512B 需 ~533ms，200ms 会中途超时截断报文；放宽到 1000ms 覆盖整帧 */
    return (HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000) == HAL_OK) ? 0 : -1;
}

/* 发送一条记录；返回 1=已从串口发出（不代表云端入库），0=串口未发出。
   tx 改为 static，避免 512B 栈占用叠加 Dtu_BuildTelemetry 的浮点/snprintf 峰值。 */
static int Dtu_SendRecord(const FS_Record_t *rec)
{
    static char s_tx_buf[DTU_TX_BUF_SIZE];
    uint16_t len = Dtu_BuildTelemetry(rec, s_tx_buf, sizeof(s_tx_buf));
    if (Dtu_Send((const uint8_t*)s_tx_buf, len) != 0) {
        return 0;   /* 串口未发出：不计为云端失败，调用方稍后重试 */
    }
    s_pending_boot = rec->boot_id;
    s_pending_seq  = rec->seq;
    s_last_send_ms = HAL_GetTick();
    return 1;
}

/* 在扁平 JSON 对象中取字符串字段值（允许 key 与 value 间有空白）。
   成功返回 true 并写入 out（NUL 结尾）。 */
static bool Dtu_GetStr(const char *obj, const char *key, char *out, size_t cap)
{
    const char *p = strstr(obj, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != ':') return false;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != '"') return false;
    p++;
    const char *e = strchr(p, '"');
    if (!e) return false;
    size_t n = (size_t)(e - p);
    if (n == 0 || n >= cap) return false;
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

/* 解析 ACK 对象：校验 type=upload_ack、status=stored、msg_id 与期望完全相等
   （msg_id 形如 "JY001/<boot>/<seq>"，天然覆盖 device_id 与编号格式）。
   obj 必须是 NUL 结尾的完整对象。匹配则确认出队。 */
static int Dtu_ParseAck(const char *obj)
{
    char type[24], status[16], msgid[64];

    if (!Dtu_GetStr(obj, "\"type\"", type, sizeof(type)) ||
        strcmp(type, "upload_ack") != 0) return 0;
    if (!Dtu_GetStr(obj, "\"status\"", status, sizeof(status)) ||
        strcmp(status, "stored") != 0) return 0;
    if (!Dtu_GetStr(obj, "\"msg_id\"", msgid, sizeof(msgid))) return 0;

    char expect[64];
    int n = snprintf(expect, sizeof(expect), "%s/%lu/%lu",
                     DTU_DEVICE_ID,
                     (unsigned long)s_pending_boot,
                     (unsigned long)s_pending_seq);
    if (n <= 0 || (size_t)n >= sizeof(expect)) return 0;

    if (strcmp(msgid, expect) != 0) return 0;

    FS_MarkConfirmed(s_pending_boot, s_pending_seq);
    s_state = DTU_IDLE;
    s_retry_cnt = 0;
    return 1;
}

/* 处理接收缓冲：按 '{}' 对象边界切帧（兼容 DTU 下行剥掉 '\n'）。
   - 与 USART1 中断里的 Dtu_OnRxByte 共享缓冲，取/移数据时关中断；
   - 只把"完整对象"拷出，在临界区外解析（避免 Flash 写入期间长时间关中断）。 */
static void Dtu_ProcessRx(void)
{
    static char s_obj[DTU_RX_BUF_SIZE + 1];

    for (;;) {
        uint16_t objlen = 0;

        /* --- 临界区：仅操作共享缓冲，不做 Flash 操作 --- */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        int startp = -1, endp = -1;
        for (uint16_t i = 0; i < s_rx_len; i++) {
            if (s_rx_buf[i] == '{') {
                startp = (int)i;
            } else if (s_rx_buf[i] == '}' && startp >= 0) {
                endp = (int)i;
                break;
            }
        }
        if (startp >= 0 && endp > startp) {
            objlen = (uint16_t)(endp - startp + 1);
            if (objlen > DTU_RX_BUF_SIZE) objlen = DTU_RX_BUF_SIZE;
            memcpy(s_obj, &s_rx_buf[startp], objlen);
            uint16_t rest = (uint16_t)(s_rx_len - (uint16_t)(endp + 1));
            if (rest) memmove(&s_rx_buf[0], &s_rx_buf[endp + 1], rest);
            s_rx_len = rest;
        } else if (startp >= 0) {
            /* 有 '{' 但未闭合：丢弃其前的垃圾，保留未完成对象 */
            if (startp > 0) {
                uint16_t rest = (uint16_t)(s_rx_len - (uint16_t)startp);
                memmove(&s_rx_buf[0], &s_rx_buf[startp], rest);
                s_rx_len = rest;
            }
        } else {
            s_rx_len = 0;   /* 无 '{'：全是垃圾，清空 */
        }

        __set_PRIMASK(primask);
        /* --- 临界区结束 --- */

        if (objlen == 0) break;
        s_obj[objlen] = '\0';
        Dtu_ParseAck(s_obj);   /* Flash 操作在此，中断已恢复 */
    }
}

/* 判断是否到时间生成一条新记录并持久化（契约 §8.1：每 DTU_NEW_RECORD_MS） */
static void Dtu_MaybeNewRecord(void)
{
    if (!s_identity_ok) return;   /* 身份未落盘校验：禁止生成新记录，避免撞云端历史 */
    uint32_t now = HAL_GetTick();
    if (now - s_last_record_ms >= DTU_NEW_RECORD_MS) {
        s_last_record_ms = now;
        FS_Record_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.boot_id   = s_boot_id;
        rec.seq       = ++s_seq;   /* 本次启动内从1递增；重发保留原编号 */
        rec.sampled_at= 0;      /* 无RTC，契约§5 填0 */
        rec.time_quality = 0;   /* 无可信时间 */
        rec.uptime_ms = now;    /* 采样时刻运行时长 */
        rec.confirmed = false;

        /* 从 g_env 填 8 通道 + valid */
        rec.data[FS_CH_TEMPERATURE]    = FS_FIXED_DIV10(g_env.air_temp);
        rec.data[FS_CH_HUMIDITY]       = FS_FIXED_DIV10(g_env.air_humi);
        rec.data[FS_CH_CO2]            = FS_FIXED_DIV1(g_env.co2);
        /* 光照：g_env.lux 已是 uint32，直接无符号落库，不走 int32/float 中转 */
        rec.data[FS_CH_LIGHT]          = FS_FIXED_UINT1(g_env.lux);
        rec.data[FS_CH_PRESSURE]       = FS_FIXED_DIV10(g_env.pressure);
        rec.data[FS_CH_SOIL_TEMP]      = FS_FIXED_DIV10(g_env.soil_temp);
        rec.data[FS_CH_SOIL_MOISTURE]  = FS_FIXED_DIV10(g_env.soil_humi);
        rec.data[FS_CH_SOIL_EC]        = FS_FIXED_DIV1(g_env.soil_ec);

        /* valid：百叶箱→temp/humi/co2/light/pressure；土壤→soil三量 */
        uint8_t valid = 0;
        if (g_env.ble_box_valid) valid |= 0x1F;   /* bit0-4: temp,humi,co2,light,pressure */
        if (g_env.soil_valid)    valid |= 0xE0;   /* bit5-7: soil_temp,soil_moist,soil_ec */
        rec.valid = valid;

        /* 写入失败（队列满/Flash IO 失败）时不递增 seq，避免编号空洞 */
        if (FS_Append(&rec) != FS_OK) {
            s_seq--;
        }
    }
}

/* 上报状态机（契约 §8.2/§8.3） */
static void Dtu_RunStateMachine(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state) {
    case DTU_IDLE:
        /* 有待发记录则开一个发送槽（读队首发送，不移除索引，等 ACK 才出队） */
        if (FS_PendingCount() > 0 && (now - s_last_send_ms >= DTU_SEND_SLOT_MS)) {
            FS_Record_t rec;
            /* 契约 §8.2：默认补历史（队首，保留原 msg_id/sampled_at）；
               每补传 DTU_HIST_PER_NEW 条历史，插发一条最新，避免实时数据被饿死。
               插队只是调度优先级，不改变采样节律（不额外生成样本）。 */
            bool send_latest = false;
            if (s_hist_sent >= DTU_HIST_PER_NEW) {
                if (FS_PeekLatest(&rec)) {
                    send_latest = true;
                    s_hist_sent = 0;
                }
            }
            if (!send_latest) {
                if (!FS_Peek(&rec, true)) break;
                s_hist_sent++;
            }
            if (Dtu_SendRecord(&rec)) {
                s_state = DTU_SENDING;
            } else {
                /* 串口未发出：不计云端失败，稍后重试 */
                s_state = DTU_BACKOFF;
                s_next_action_ms = now + DTU_SEND_SLOT_MS;
            }
        }
        break;

    case DTU_SENDING:
        /* 等待ACK：超时(如8s)进入退避 */
        if (now - s_last_send_ms >= 8000) {
            s_retry_cnt++;
            /* 指数退避：2s/4s/8s/16s/32s...；位移钳位到 6，消除溢出 UB，上限 300s */
            uint32_t shift = (s_retry_cnt > 7) ? 6 : (s_retry_cnt - 1);
            uint32_t backoff = (1u << shift) * (uint32_t)DTU_SEND_SLOT_MS;
            if (backoff > 300000u) backoff = 300000u;
            s_next_action_ms = now + backoff;
            s_state = DTU_BACKOFF;
        }
        break;

    case DTU_BACKOFF:
        if ((int32_t)(now - s_next_action_ms) >= 0) {
            if (s_retry_cnt >= 5) {
                /* 连续失败转周期探测 */
                s_state = DTU_PROBE;
                s_next_action_ms = now;   /* 立即探测一次 */
            } else {
                /* 重发当前 pending（同一 msg_id，保留编号；队首即 pending） */
                FS_Record_t rec;
                if (FS_Peek(&rec, true)) {
                    if (Dtu_SendRecord(&rec)) s_state = DTU_SENDING;
                    else s_next_action_ms = now + DTU_SEND_SLOT_MS;
                } else {
                    s_state = DTU_IDLE;
                }
            }
        }
        break;

    case DTU_PROBE:
        /* 周期探测：每30s发一条最新试探，收到ACK即恢复 */
        if ((int32_t)(now - s_next_action_ms) >= 0) {
            FS_Record_t rec;
            bool pending = FS_Peek(&rec, true);
            if (pending) {
                (void)Dtu_SendRecord(&rec);   /* 结果忽略，保持探测节奏 */
            } else {
                s_state = DTU_IDLE;           /* 无待发，恢复空闲 */
            }
            s_next_action_ms = now + 30000;
        }
        break;
    }
}

/* ====== 公开接口 ====== */

void Dtu_Init(void)
{
    FS_Init();

    /* 身份层：只有 FS_NextBootId 成功落盘并校验通过才允许生成新记录。
       失败则进入"身份存储故障"：禁止新建/上传新数据，但保留补传已持久化旧记录。 */
    uint32_t nb = FS_NextBootId();
    if (nb != 0) {
        s_boot_id = nb;
        s_identity_ok = true;
    } else {
        s_boot_id = FS_GetBootId();
        s_identity_ok = false;
    }

    s_state = DTU_IDLE;
    s_rx_len = 0;
    s_last_send_ms = 0;
    s_pending_boot = 0;
    s_pending_seq  = 0;
    /* 以当前时刻为基准，第一条记录在 DTU_NEW_RECORD_MS 后生成（契约 §8.1）。
       若置 0，HAL_GetTick() 起点接近 0 会导致上电瞬间立即写一条。 */
    s_last_record_ms = HAL_GetTick();
    s_hist_sent = 0;
    s_retry_cnt = 0;

    /* 启动 USART1 单字节中断接收（收 ACK） */
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/* 由 HAL_UART_RxCpltCallback 转发（见 app.c） */
void Dtu_OnRxByte(uint8_t byte)
{
    if (s_rx_len < DTU_RX_BUF_SIZE) {
        s_rx_buf[s_rx_len++] = byte;
    } else {
        s_rx_len = 0;   /* 超长丢弃 */
    }
}

uint8_t *Dtu_RxByteBuf(void) { return &s_rx_byte; }

void Dtu_Tick(void)
{
    Dtu_MaybeNewRecord();
    Dtu_RunStateMachine();
    Dtu_ProcessRx();
}

uint32_t Dtu_PendingCount(void) { return FS_PendingCount(); }
uint32_t Dtu_DroppedCount(void) { return FS_DroppedCount(); }
uint32_t Dtu_LastBootId(void)   { return s_boot_id; }
uint8_t  Dtu_IdentityOk(void)   { return s_identity_ok ? 1 : 0; }
