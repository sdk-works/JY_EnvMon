/**
  ******************************************************************************
  * @file    screen.h
  * @brief   BX-5K1 LED大屏驱动接口
  ******************************************************************************
  */
#ifndef __SCREEN_H
#define __SCREEN_H

#include "app_def.h"

/* 上屏一条单行固定文本（不滚动，由 app.c 定时切换下一条） */
void Screen_ShowFixed(const uint8_t *text, uint16_t len);

/* 格式化8条轮播文本 */
void Screen_FormatTemp(uint8_t *out, uint16_t *out_len);
void Screen_FormatHumi(uint8_t *out, uint16_t *out_len);
void Screen_FormatCO2(uint8_t *out, uint16_t *out_len);
void Screen_FormatLux(uint8_t *out, uint16_t *out_len);
void Screen_FormatPressure(uint8_t *out, uint16_t *out_len);
void Screen_FormatSoilTemp(uint8_t *out, uint16_t *out_len);
void Screen_FormatSoilHumi(uint8_t *out, uint16_t *out_len);
void Screen_FormatEC(uint8_t *out, uint16_t *out_len);

#endif
