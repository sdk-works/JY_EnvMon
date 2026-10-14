/**
  ******************************************************************************
  * @file    sensor.c
  * @brief   百叶箱 + 土壤三合一 Modbus 采集
  *
  * 百叶箱(08): 读0x01F4起8个寄存器
  *   0x01F4=湿度/10  0x01F5=温度/10  0x01F7=CO2ppm
  *   0x01F9=气压/10  0x01FA+0x01FB=光照(32位)
  *
  * 土壤(0x12=18): 读0x0000起5个寄存器
  *   0x0000=含水率  0x0001=温度/10  0x0002=EC
  ******************************************************************************
  */
#include "sensor.h"
#include "modbus.h"
#include <string.h>

EnvData_t g_env;

/* 读百叶箱全部环境数据 */
AppErr_t BLE_Box_Read(void)
{
    uint16_t regs[8];
    uint16_t cnt = 0;
    AppErr_t err = Modbus_ReadHolding(BUS_SENSOR, SLAVE_BLE_BOX,
                                      0x01F4, 8,
                                      regs, &cnt, 300);
    if (err != APP_OK || cnt < 8) return err;

    g_env.air_humi  = regs[0] / 10.0f;
    g_env.air_temp  = regs[1] / 10.0f;
    g_env.co2       = (float)regs[3];
    g_env.pressure  = regs[5] / 10.0f;
    g_env.lux       = ((uint32_t)regs[6] << 16) | regs[7];

    g_env.ble_box_valid = 1;
    return APP_OK;
}

/* 读土壤三合一 */
AppErr_t Soil_Read(void)
{
    uint16_t regs[5];
    uint16_t cnt = 0;
    AppErr_t err = Modbus_ReadHolding(BUS_SENSOR, SLAVE_SOIL,
                                      0x0000, 5,
                                      regs, &cnt, 300);
    if (err != APP_OK || cnt < 5) return err;

    g_env.soil_humi = (float)regs[0];
    g_env.soil_temp = regs[1] / 10.0f;
    g_env.soil_ec   = (float)regs[2];

    g_env.soil_valid = 1;
    return APP_OK;
}

/* 采集全部传感器 */
void Sensor_Collect(void)
{
    if (BLE_Box_Read() != APP_OK) g_env.ble_box_valid = 0;
    if (Soil_Read() != APP_OK)    g_env.soil_valid = 0;
    g_env.last_update_ms = HAL_GetTick();
}
