/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用总调度：在main.c里调用APP_Init()和APP_Loop()
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#include "app_def.h"

/* 在所有外设MX_*_Init()之后调用一次 */
void APP_Init(void);

/* 在while(1)里反复调用 */
void APP_Loop(void);

#endif
