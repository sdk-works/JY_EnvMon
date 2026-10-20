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

/* 节律参数（契约 §8）
   注意：以下为【测试临时值】，验证 ACK 重复问题解决后需改回：
         DTU_SEND_SLOT_MS 5000、DTU_NEW_RECORD_MS 60000。 */
#define DTU_SEND_SLOT_MS      2000      /* 测试：发送槽最小间隔 2s（正式 5000） */
#define DTU_NEW_RECORD_MS     10000     /* 测试：每10s生成一条新记录（正式 60000） */
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

#endif /* __DTU_H */
