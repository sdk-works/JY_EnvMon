/**
  ******************************************************************************
  * @file    bus485.c
  * @brief   485总线收发封装实现
  ******************************************************************************
  */
#include "bus485.h"
#include <string.h>

Bus_t g_bus[BUS_NUM] = {0};

/* 每路串口的中断接收临时字节 */
static uint8_t s_rx_byte[BUS_NUM];

/* 帧间隔超时：Modbus-RTU 3.5字符时间，9600bps下约4ms；取5ms留余量 */
#define BUS_FRAME_GAP_MS   5

void Bus_Init(void)
{
    /* 485-1：USART3, T2_EN=PD3 */
    g_bus[BUS_SENSOR].huart       = &huart3;
    g_bus[BUS_SENSOR].en_port     = T2_EN_GPIO_Port;
    g_bus[BUS_SENSOR].en_pin      = T2_EN_Pin;

    /* 485-2：UART5, T3_EN=PD4 */
    g_bus[BUS_SCREEN].huart       = &huart5;
    g_bus[BUS_SCREEN].en_port     = T3_EN_GPIO_Port;
    g_bus[BUS_SCREEN].en_pin      = T3_EN_Pin;

    /* 两路都默认拉回接收态 */
    HAL_GPIO_WritePin(T2_EN_GPIO_Port, T2_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(T3_EN_GPIO_Port, T3_EN_Pin, GPIO_PIN_RESET);

    /* 启动中断接收（单字节模式） */
    for (int i = 0; i < BUS_NUM; i++)
    {
        g_bus[i].rx_len  = 0;
        g_bus[i].rx_done = 0;
        HAL_UART_Receive_IT(g_bus[i].huart, &s_rx_byte[i], 1);
    }
}

AppErr_t Bus_Send(BusPort_t port, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    Bus_t *bus = &g_bus[port];

    if (len == 0 || len > BUS_RX_BUF_SIZE) return APP_ERR_FRAME;

    /* 清掉接收缓冲区残留 */
    bus->rx_len = 0;
    bus->rx_done = 0;
    __HAL_UART_FLUSH_DRREGISTER(bus->huart);
    __HAL_UART_CLEAR_OREFLAG(bus->huart);

    /* 切发送态 */
    HAL_GPIO_WritePin(bus->en_port, bus->en_pin, GPIO_PIN_SET);

    /* 阻塞发送 */
    HAL_StatusTypeDef st = HAL_UART_Transmit(bus->huart, (uint8_t *)data, len, timeout_ms);

    /* 等最后一个字节真正移位出完（TC标志），再切接收态 */
    uint32_t t0 = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - t0) > timeout_ms) break;
    }

    /* 切回接收态 */
    HAL_GPIO_WritePin(bus->en_port, bus->en_pin, GPIO_PIN_RESET);

    if (st != HAL_OK) return APP_ERR_TIMEOUT;
    return APP_OK;
}

uint16_t Bus_PollRx(BusPort_t port, uint8_t *out_buf, uint16_t max_len)
{
    Bus_t *bus = &g_bus[port];
    if (!bus->rx_done) return 0;

    uint16_t n = bus->rx_len;
    if (n > max_len) n = max_len;
    memcpy(out_buf, bus->rx_buf, n);

    bus->rx_len = 0;
    bus->rx_done = 0;
    return n;
}

/* 由HAL_UART_RxCpltCallback在中断里调用 */
void Bus_OnRxByte(UART_HandleTypeDef *huart)
{
    Bus_t *bus = NULL;
    int idx = -1;
    for (int i = 0; i < BUS_NUM; i++)
    {
        if (g_bus[i].huart == huart) { bus = &g_bus[i]; idx = i; break; }
    }
    if (bus == NULL) return;

    /* 追加到缓冲区（满了就丢弃，不溢出） */
    if (bus->rx_len < BUS_RX_BUF_SIZE)
    {
        bus->rx_buf[bus->rx_len++] = s_rx_byte[idx];
        bus->last_byte_ms = HAL_GetTick();
    }

    /* 重新挂接收中断 */
    HAL_UART_Receive_IT(huart, &s_rx_byte[idx], 1);
}

void Bus_Tick1ms(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < BUS_NUM; i++)
    {
        if (g_bus[i].rx_len > 0 && !g_bus[i].rx_done)
        {
            if ((now - g_bus[i].last_byte_ms) >= BUS_FRAME_GAP_MS)
            {
                g_bus[i].rx_done = 1;   /* 一帧收完 */
            }
        }
    }
}

/* 注意：HAL_UART_RxCpltCallback 由 app.c 统一实现（回环测试版） */
