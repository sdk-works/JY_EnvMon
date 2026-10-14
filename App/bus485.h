/**
  ******************************************************************************
  * @file    bus485.h
  * @brief   485总线收发封装：方向脚控制 + 中断接收 + 帧间隔超时
  *          板上两路485：
  *            485-1 = USART3 / T2_EN(PD3)  → 接传感器（百叶箱08 + 土壤18）
  *            485-2 = UART5  / T3_EN(PD4)  → 接LED大屏
  ******************************************************************************
  */
#ifndef __BUS485_H
#define __BUS485_H

#include "app_def.h"

/* 485端口枚举 */
typedef enum
{
    BUS_SENSOR = 0,    /* USART3, T2_EN */
    BUS_SCREEN = 1,    /* UART5,  T3_EN */
    BUS_NUM
} BusPort_t;

#define BUS_RX_BUF_SIZE   64

typedef struct
{
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;

    uint8_t  tx_buf[BUS_RX_BUF_SIZE];
    uint8_t  rx_buf[BUS_RX_BUF_SIZE];
    volatile uint16_t rx_len;        /* 已收到的字节数 */
    volatile uint8_t  rx_done;       /* 一帧接收完成标志 */
    volatile uint32_t last_byte_ms;  /* 最后一字节到达时刻 */
} Bus_t;

extern Bus_t g_bus[BUS_NUM];

/* 初始化：启动空闲中断接收 */
void Bus_Init(void);

/* 发送一帧（阻塞发完，自动切换方向） */
AppErr_t Bus_Send(BusPort_t port, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/* 查询是否收到一帧数据（非阻塞）
 * 返回值：0=未完成，>0=帧长度；*out_buf 拷贝出数据 */
uint16_t Bus_PollRx(BusPort_t port, uint8_t *out_buf, uint16_t max_len);

/* UART接收完成回调（由HAL_UART_RxCpltCallback转发调用） */
void Bus_OnRxByte(UART_HandleTypeDef *huart);

/* 在主循环里周期性调用，做帧间隔超时判断（建议每1ms调一次） */
void Bus_Tick1ms(void);

#endif /* __BUS485_H */
