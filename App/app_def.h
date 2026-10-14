/**
  ******************************************************************************
  * @file    app_def.h
  * @brief   全局公共定义：传感器数据结构、设备地址、错误码
  *          京阳温室环境监测 JY_EnvMon
  ******************************************************************************
  */
#ifndef __APP_DEF_H
#define __APP_DEF_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ====== 设备Modbus从机地址（发货单确定） ====== */
#define SLAVE_BLE_BOX     0x08    /* 百叶箱：温湿度/CO2/光照/大气压 */
#define SLAVE_SOIL        0x12    /* 土壤三合一：土壤温湿度/EC  地址18=0x12 */

/* ====== 串口实例 ====== */
extern UART_HandleTypeDef huart1;   /* RS232 → DTU        */
extern UART_HandleTypeDef huart3;   /* 485-1 → 传感器总线 */
extern UART_HandleTypeDef huart5;   /* 485-2 → LED大屏    */

/* ====== 全局传感器数据（所有模块共用） ====== */
typedef struct
{
    /* 百叶箱 */
    float    air_temp;       /* 空气温度   ℃   */
    float    air_humi;       /* 空气湿度   %RH */
    float    co2;            /* CO2浓度   ppm  */
    float    lux;            /* 光照强度   lux  */
    float    pressure;        /* 大气压强   hPa  */

    /* 土壤三合一 */
    float    soil_temp;      /* 土壤温度   ℃   */
    float    soil_humi;      /* 土壤湿度   %   */
    float    soil_ec;        /* 土壤EC     us/cm */

    /* 数据有效标志（0=无效，非0=有效） */
    uint8_t  ble_box_valid;
    uint8_t  soil_valid;

    /* 采集时间戳（ms） */
    uint32_t last_update_ms;
} EnvData_t;

extern EnvData_t g_env;

/* ====== 错误码 ====== */
typedef enum
{
    APP_OK          = 0,
    APP_ERR_TIMEOUT = 1,
    APP_ERR_CRC     = 2,
    APP_ERR_FRAME   = 3,
    APP_ERR_BUSY    = 4,
} AppErr_t;

#endif /* __APP_DEF_H */
