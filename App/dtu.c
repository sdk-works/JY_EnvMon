/**
  ******************************************************************************
  * @file    dtu.c
  * @brief   DTU上传实现（空骨架）
  ******************************************************************************
  */
#include "dtu.h"
#include <stdio.h>

void Dtu_Init(void)
{
    /* TODO: 若DTU有AT指令初始化（透传/服务器地址/端口），在这里发 */
}

void Dtu_Report(const EnvData_t *env)
{
    /* TODO: 组JSON或二进制帧，通过HAL_UART_Transmit(&huart1,...)发给DTU */
    (void)env;
}
