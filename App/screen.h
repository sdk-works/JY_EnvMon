/**
  ******************************************************************************
  * @file    screen.h
  * @brief   BX-5K1 LED 大屏驱动接口
  ******************************************************************************
  */
#ifndef __SCREEN_H
#define __SCREEN_H

#include "app_def.h"

/* 显示第 idx 项指标（0..7）：
 *   左侧中文名称(X0,Y2,24×12) + 右上数值(X26,Y0,38×8) + 右下单位(X26,Y8,38×8)
 *   由 app.c 每 8 秒切换下一项。 */
void Screen_ShowItem(uint8_t idx);

/* 通用一帧文本（诊断页用）：中文名称 ≤2 字 + 数值 ≤5 字符 + 单位 ≤5 字符。
   传入 NULL/空串则该区域留空。 */
void Screen_ShowText(const uint8_t *name, uint16_t name_len,
                     const char *val, const char *uni);

#endif
