/**
  ******************************************************************************
  * @file    modbus.h
  * @brief   Modbus-RTU 基础：CRC16 计算 + 读保持寄存器封装
  ******************************************************************************
  */
#ifndef __MODBUS_H
#define __MODBUS_H

#include "app_def.h"
#include "bus485.h"

/* 计算Modbus CRC16（Modbus多项式0xA001，低字节在前） */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);

/* 读保持寄存器（功能码0x03）
 *   port        : 在哪条485总线上发
 *   slave_addr  : 从机地址
 *   reg_start   : 起始寄存器地址
 *   reg_num     : 要读几个寄存器
 *   out_data    : 输出寄存器值（uint16数组，主机序）
 *   out_len     : 实际读到的寄存器个数（可为NULL）
 *   timeout_ms  : 等待从机回复的超时
 */
AppErr_t Modbus_ReadHolding(BusPort_t port, uint8_t slave_addr,
                            uint16_t reg_start, uint16_t reg_num,
                            uint16_t *out_data, uint16_t *out_len,
                            uint32_t timeout_ms);

#endif /* __MODBUS_H */
