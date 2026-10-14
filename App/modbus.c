/**
  ******************************************************************************
  * @file    modbus.c
  * @brief   Modbus-RTU 基础实现
  ******************************************************************************
  */
#include "modbus.h"
#include "bus485.h"
#include <string.h>

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;   /* 低字节先发，高字节后发 */
}

AppErr_t Modbus_ReadHolding(BusPort_t port, uint8_t slave_addr,
                            uint16_t reg_start, uint16_t reg_num,
                            uint16_t *out_data, uint16_t *out_len,
                            uint32_t timeout_ms)
{
    uint8_t  tx[16];
    uint8_t  rx[BUS_RX_BUF_SIZE];
    uint16_t crc;

    /* 组请求帧：addr 0x03 startHi startLo numHi numLo crcLo crcHi */
    tx[0] = slave_addr;
    tx[1] = 0x03;
    tx[2] = (reg_start >> 8) & 0xFF;
    tx[3] =  reg_start & 0xFF;
    tx[4] = (reg_num   >> 8) & 0xFF;
    tx[5] =  reg_num   & 0xFF;
    crc = Modbus_CRC16(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    /* 清接收缓冲，发请求 */
    AppErr_t send_err = Bus_Send(port, tx, 8, 100);
    if (send_err != APP_OK) return send_err;

    /* 等从机回复：轮询Bus_PollRx */
    uint32_t t0 = HAL_GetTick();
    uint16_t rxlen = 0;
    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        /* Main loop is blocked here, so service receive-frame completion. */
        Bus_Tick1ms();
        rxlen = Bus_PollRx(port, rx, sizeof(rx));
        if (rxlen > 0) break;
    }
    if (rxlen == 0) return APP_ERR_TIMEOUT;

    /* 校验帧格式：addr, 0x03, byte_count, data..., crcLo, crcHi
     * 期望长度 = 3 + 2*reg_num + 2 */
    uint16_t expect_len = 3 + 2 * reg_num + 2;
    if (rxlen != expect_len) return APP_ERR_FRAME;
    if (rx[0] != slave_addr)      return APP_ERR_FRAME;
    if (rx[1] != 0x03)            return APP_ERR_FRAME;
    uint8_t byte_cnt = rx[2];
    if (byte_cnt != 2 * reg_num)  return APP_ERR_FRAME;

    /* 校验CRC */
    uint16_t recv_crc  = rx[rxlen-2] | (rx[rxlen-1] << 8);
    uint16_t calc_crc  = Modbus_CRC16(rx, rxlen - 2);
    if (recv_crc != calc_crc) return APP_ERR_CRC;

    /* 解析寄存器值（大端） */
    for (uint16_t i = 0; i < reg_num; i++)
    {
        out_data[i] = (rx[3 + 2*i] << 8) | rx[4 + 2*i];
    }
    if (out_len) *out_len = reg_num;
    return APP_OK;
}
