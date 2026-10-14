/**
  ******************************************************************************
  * @file    dtu.h
  * @brief   DTU上传云平台（RS232 / USART1）
  *          待确认DTU型号后补：透传? MQTT? 数据格式?
  ******************************************************************************
  */
#ifndef __DTU_H
#define __DTU_H

#include "app_def.h"

void Dtu_Init(void);
void Dtu_Report(const EnvData_t *env);   /* 把最新数据上报云平台 */

#endif
