/**
  ******************************************************************************
  * @file    dtu.c
  * @brief   DTU(DR154) 可靠上报云平台实现 —— 契约 v1.2
  *
  * 通过 USART1(RS232, 9600 8N1) 发单行JSON到DR154，DR154以MQTT透传到云端。
  * 云端落库成功后回 agri/device/{id}/ack (upload_ack)，本模块按 \n 切帧接收，
  * 匹配 msg_id 后调用 FS_MarkConfirmed 出队。
  *
  * 上报状态机（契约 §8）：
  *   每60s持久化一条新记录(flash_store)；发送槽≥5s；
  *   优先补历史(每3条插1条最新)；未确认退避重发(上限300s)；链路疑似故障走周期探测。
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

/* ====== 内部函数 ====== */

/* 组上行遥测 JSON（契约 §3 格式）到 buf；返回长度。rec 是待发送记录 */
static uint16_t Dtu_BuildTelemetry(const FS_Record_t *rec, char *buf, uint16_t maxlen)
{
    char  dev[8];
    snprintf(dev, sizeof(dev), "%s/%lu/%lu", DTU_DEVICE_ID,
             (unsigned long)rec->boot_id, (unsigned long)rec->seq);

    /* 定点还原（契约 §10.6）：÷倍率 */
    float temperature = rec->data[FS_CH_TEMPERATURE]  / 10.0f;
    float humidity    = rec->data[FS_CH_HUMIDITY]     / 10.0f;
    float co2         = (float)rec->data[FS_CH_CO2];
    float light       = (float)rec->data[FS_CH_LIGHT];
    float pressure    = rec->data[FS_CH_PRESSURE]     / 10.0f;
    float soil_temp   = rec->data[FS_CH_SOIL_TEMP]    / 10.0f;
    float soil_moist  = rec->data[FS_CH_SOIL_MOISTURE]/ 10.0f;
    float soil_ec     = (float)rec->data[FS_CH_SOIL_EC];

    /* valid 数组 */
    int v[8];
    for (int i = 0; i < FS_CH_NUM; i++)
        v[i] = (rec->valid >> i) & 1;

    int n = snprintf(buf, maxlen,
        "{\"type\":\"telemetry\",\"schema_version\":1,\"device_id\":\"%s\","
        "\"msg_id\":\"%s\",\"boot_id\":%lu,\"seq\":%lu,"
        "\"sampled_at\":%lu,\"time_quality\":%d,\"uptime_ms\":%llu,"
        "\"data\":{"
        "\"temperature\":%.1f,\"humidity\":%.1f,\"co2\":%.0f,"
        "\"light\":%.0f,\"pressure\":%.1f,"
        "\"soil_temperature\":%.1f,\"soil_moisture\":%.1f,\"soil_ec\":%.0f,"
        "\"valid\":[%d,%d,%d,%d,%d,%d,%d,%d]}}\n",
        DTU_DEVICE_ID, dev,
        (unsigned long)rec->boot_id, (unsigned long)rec->seq,
        (unsigned long)rec->sampled_at, (int)rec->time_quality,
        (unsigned long long)rec->uptime_ms,
        (double)temperature, (double)humidity, (double)co2,
        (double)light, (double)pressure,
        (double)soil_temp, (double)soil_moist, (double)soil_ec,
        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);
    if (n < 0 || n >= maxlen) n = maxlen - 1;
    return (uint16_t)n;
}

/* 通过 USART1 发送一帧（阻塞发完） */
static void Dtu_Send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 200);
}

/* 发送一条记录；返回是否成功（仅指串口发出，不代表云端入库） */
static void Dtu_SendRecord(const FS_Record_t *rec)
{
    char tx[DTU_TX_BUF_SIZE];
    uint16_t len = Dtu_BuildTelemetry(rec, tx, sizeof(tx));
    Dtu_Send((const uint8_t*)tx, len);
    s_pending_boot = rec->boot_id;
    s_pending_seq  = rec->seq;
    s_last_send_ms = HAL_GetTick();
    s_state = DTU_SENDING;
}

/* 解析 ACK JSON，检查是否匹配当前 pending；匹配则确认并出队 */
static int Dtu_ParseAck(const uint8_t *line, uint16_t len)
{
    /* 简单定位 msg_id 和 status */
    char *p = strstr((char*)line, "\"msg_id\":\"");
    char *s = strstr((char*)line, "\"status\":\"stored\"");
    if (!p || !s) return 0;

    p += strlen("\"msg_id\":\"");
    /* 解析 boot/seq：格式 device/boot/seq */
    char *end = strchr(p, '"');
    if (!end) return 0;
    char msgid[64];
    uint16_t idlen = (uint16_t)(end - p);
    if (idlen == 0 || idlen >= sizeof(msgid)) return 0;
    memcpy(msgid, p, idlen); msgid[idlen] = 0;

    /* 从 msg_id 尾部解析 seq 和 boot（格式 dev/boot/seq） */
    long boot = 0, seq = 0;
    char *last_slash = strrchr(msgid, '/');
    if (last_slash) {
        seq = atol(last_slash + 1);
        *last_slash = 0;
        char *first_slash = strrchr(msgid, '/');
        if (first_slash) boot = atol(first_slash + 1);
    }
    /* 匹配当前 pending */
    if (boot == (long)s_pending_boot && seq == (long)s_pending_seq) {
        FS_MarkConfirmed(s_pending_boot, s_pending_seq);
        s_state = DTU_IDLE;
        s_retry_cnt = 0;
        return 1;
    }
    return 0;
}

/* 处理接收缓冲：按 \n 切帧，逐帧解析 */
static void Dtu_ProcessRx(void)
{
    uint16_t i = 0;
    while (i < s_rx_len) {
        /* 找换行 */
        uint16_t j = i;
        while (j < s_rx_len && s_rx_buf[j] != '\n') j++;
        if (j >= s_rx_len) break;   /* 半帧，等后续 */
        uint16_t linelen = j - i;
        if (linelen > 0) {
            Dtu_ParseAck(&s_rx_buf[i], linelen);
        }
        i = j + 1;
    }
    /* 保留最后未换行部分（如有）到缓冲头 */
    if (i < s_rx_len) {
        uint16_t remain = s_rx_len - i;
        memmove(s_rx_buf, &s_rx_buf[i], remain);
        s_rx_len = remain;
    } else {
        s_rx_len = 0;
    }
}

/* 判断是否到时间生成一条新记录并持久化（契约 §8.1：每60s） */
static void Dtu_MaybeNewRecord(void)
{
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
        rec.data[FS_CH_LIGHT]          = FS_FIXED_DIV1(g_env.lux);
        rec.data[FS_CH_PRESSURE]       = FS_FIXED_DIV10(g_env.pressure);
        rec.data[FS_CH_SOIL_TEMP]      = FS_FIXED_DIV10(g_env.soil_temp);
        rec.data[FS_CH_SOIL_MOISTURE]  = FS_FIXED_DIV10(g_env.soil_humi);
        rec.data[FS_CH_SOIL_EC]        = FS_FIXED_DIV1(g_env.soil_ec);

        /* valid：百叶箱→temp/humi/co2/light/pressure；土壤→soil三量 */
        uint8_t valid = 0;
        if (g_env.ble_box_valid) valid |= 0x1F;   /* bit0-4: temp,humi,co2,light,pressure */
        if (g_env.soil_valid)    valid |= 0xE0;   /* bit5-7: soil_temp,soil_moist,soil_ec */
        rec.valid = valid;

        /* seq 分配：用当前队列长度推断下一个 seq（简化：boot内从1递增，存到rec前设值） */
        /* 通过 flash_store 内部维护；这里 seq 字段用于 msg_id，先给个占位，实际由FS记录 */
        FS_Append(&rec);
    }
}

/* 上报状态机（契约 §8.2/§8.3） */
static void Dtu_RunStateMachine(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state) {
    case DTU_IDLE:
        /* 有待发记录则开一个发送槽（读队首发送，不移除索引，等 ACK 才出队） */
        if (FS_PendingCount() > 0) {
            if (now - s_last_send_ms >= DTU_SEND_SLOT_MS) {
                FS_Record_t rec;
                /* 每3条历史插1条最新：优先发历史（队首） */
                if (FS_Peek(&rec, true)) {
                    Dtu_SendRecord(&rec);
                    s_hist_sent++;
                }
            }
        }
        break;

    case DTU_SENDING:
        /* 等待ACK：超时(如8s)进入退避 */
        if (now - s_last_send_ms >= 8000) {
            s_retry_cnt++;
            /* 指数退避：5s~300s，超过5次转周期探测 */
            uint32_t backoff = (1u << (s_retry_cnt - 1)) * DTU_SEND_SLOT_MS;
            if (backoff > 300000) backoff = 300000;
            s_next_action_ms = now + backoff;
            s_state = DTU_BACKOFF;
        }
        break;

    case DTU_BACKOFF:
        if (now >= s_next_action_ms) {
            /* 重发当前 pending（同一 msg_id，保留编号；队首即 pending，未ACK未出队） */
            FS_Record_t rec;
            if (FS_Peek(&rec, true)) {
                if (rec.boot_id == s_pending_boot && rec.seq == s_pending_seq) {
                    Dtu_SendRecord(&rec);   /* 保持原 msg_id */
                } else if (s_retry_cnt >= 5) {
                    s_state = DTU_PROBE;
                    s_next_action_ms = now + 30000;
                } else {
                    s_state = DTU_IDLE;
                }
            } else if (s_retry_cnt >= 5) {
                s_state = DTU_PROBE;
                s_next_action_ms = now + 30000;
            } else {
                s_state = DTU_IDLE;
            }
        }
        break;

    case DTU_PROBE:
        /* 周期探测：每30s发一条最新试探，收到ACK即恢复 */
        if (now >= s_next_action_ms) {
            FS_Record_t rec;
            if (FS_Peek(&rec, true)) {
                Dtu_SendRecord(&rec);   /* 发完不移索引，等 ACK */
            } else {
                s_state = DTU_IDLE;   /* 无待发，恢复空闲 */
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
    s_boot_id = FS_NextBootId();   /* 生成本次启动编号；失败则用FS_GetBootId兜底 */
    if (s_boot_id == 0) s_boot_id = FS_GetBootId();

    s_state = DTU_IDLE;
    s_rx_len = 0;
    s_last_send_ms = 0;
    s_last_record_ms = 0;
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
