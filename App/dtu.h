/**
  ******************************************************************************
  * @file    dtu.h
  * @brief   DTU(DR154) 可靠上报云平台 —— RS232/USART1，MQTT透传
  *          契约 v1.2 §2/§3/§7/§8
  *
  *  上行:   agri/device/{device_id}/up   单行JSON + \n
  *  下行:   agri/device/{device_id}/ack  单行JSON(upload_ack)；
 *          本端按 '{}' 对象边界解析，兼容 DTU 下行剥掉 '\n' 的情况。
  *
  *  上报状态机（契约 §8.2/§8.3）：
  *    - 每60s持久化一条新记录(flash_store)
  *    - 发送槽间隔≥5s；优先补历史(每3条插1条最新)，重试占槽
  *    - 收到匹配ACK才 MarkConfirmed(出队)；失败退避重发，保留周期探测
  ******************************************************************************
  */
#ifndef __DTU_H
#define __DTU_H

#include "app_def.h"

/* 上报身份（与云端 Device.device_id 一致） */
#define DTU_DEVICE_ID     "JY001"

/* ====== 节律参数（契约 §8）—— 唯一定义处，flash_store.h 不再重复定义 ======
   默认 = 生产节律：记录 60s/条，补传发送槽 5s/条。

   与采集解耦（三层节拍，互不要求同频）：
     · 传感器采集：快档 5s（app.c COLLECT_FAST_MS）、慢档 30s（COLLECT_SLOW_MS）
     · 记录持久化：本文件 DTU_NEW_RECORD_MS = 60s（写 Flash + 唯一 msg_id）
     · 屏幕刷新：翻页 5s；采集成功后立即刷新当前页数值（app.c）

   "记录生成后立即发送"：记录周期(60s)远大于发送槽(5s)，新记录生成时发送槽
   必然已空闲，因此无需额外等待，下一次状态机调度即发出（不等到下一个 60s）。

   【主机测试】run_host_tests.js 会加 -DJY_HOST_TEST，保持 10s/2s 快速档，
   让既有断言（推进节奏、退避 2s/4s/8s…）继续成立，且单次测试更快。
   若要临时做上板快速验证，可在这里定义 DTU_FAST_TEST_RHYTHM。
   正式部署：两者都不要定义（默认走生产档）。 */
#if defined(JY_HOST_TEST) || defined(DTU_FAST_TEST_RHYTHM)
#define DTU_SEND_SLOT_MS      2000      /* 快速验证：发送槽 2s */
#define DTU_NEW_RECORD_MS     10000     /* 快速验证：每 10s 一条记录 */
#else
#define DTU_SEND_SLOT_MS      5000      /* 生产：发送槽（补传节奏）5s */
#define DTU_NEW_RECORD_MS     60000     /* 生产：每 60s 生成一条记录 */
#endif
#define DTU_HIST_PER_NEW      3         /* 每补传3条历史，插1条最新 */

/* DTU 状态机 */
typedef enum {
    DTU_IDLE = 0,        /* 无待发/待确认，链路空闲 */
    DTU_SENDING,         /* 正在等待 ACK（已发未确认） */
    DTU_BACKOFF,         /* 退避中，等待下次重试 */
    DTU_PROBE,           /* 周期探测中（链路疑似故障） */
} Dtu_State_t;

void Dtu_Init(void);              /* 初始化：boot_id、启动接收、上报身份 */

/* 主循环周期性调用（建议每1ms）：
     - 驱动上报状态机
     - 处理 ACK 帧
     - 按节律生成新记录并持久化 */
void Dtu_Tick(void);

/* UART1 收到一字节（由 HAL_UART_RxCpltCallback 转发） */
void Dtu_OnRxByte(uint8_t byte);

/* 返回 USART1 单字节接收缓冲地址（供启动 HAL_UART_Receive_IT 用） */
uint8_t *Dtu_RxByteBuf(void);

/* 诊断信息（可显示在屏幕/日志） */
uint32_t Dtu_PendingCount(void);
uint32_t Dtu_DroppedCount(void);
uint32_t Dtu_LastBootId(void);

/* 身份层状态：1=本次 boot_id 已成功落盘并校验，允许生成新记录；
   0=身份存储故障/损坏（禁止新建数据，仅补传已持久化旧记录）。 */
uint8_t Dtu_IdentityOk(void);

/* 存储层状态：1=数据区初始化成功（可追加）；0=初始化失败（禁止写入） */
uint8_t Dtu_StorageOk(void);

/* 64 位运行时长（ms），抗 HAL_GetTick 32 位回绕 */
uint64_t Dtu_UptimeMs(void);

#endif /* __DTU_H */
