/**
  ******************************************************************************
  * @file    main.h  （主机测试桩）
  * @brief   真实 main.h 会 include stm32f4xx_hal.h 并声明 huart1/3/5。
  *          这里同样把 stub 的 HAL 头引进来，使 app_def.h 的
  *          `extern UART_HandleTypeDef huart1;` 能正常展开。
  *          编译时把 tests/host/stub 放在 -I 最前面即可覆盖真实 main.h。
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"

#endif
