/**
  ******************************************************************************
  * @file    app.c
  * @brief   京阳温室环境监测主调度
  *
  * 周期：
  *   每3秒  采集一次传感器
  *   按文本长短决定每条停留时间，播完再切下一条
  ******************************************************************************
  */
#include "app.h"
#include "sensor.h"
#include "screen.h"
#include "bus485.h"

#define COLLECT_PERIOD_MS   3000
#define SCREEN_ITEM_NUM     8

/* 每条文本的停留时间(ms)，按像素宽度分档：
 *   最短(≤72px) = 8秒   → 土湿、EC
 *   中(80~88px) = 10秒  → 温度、湿度、CO2、光照、土温
 *   长(≥100px)  = 12秒  → 气压
 */
static const uint16_t s_dwell_ms[SCREEN_ITEM_NUM] = {
    10000,  /* 0 温度 */
    10000,  /* 1 湿度 */
    10000,  /* 2 CO2 */
    10000,  /* 3 光照 */
    12000,  /* 4 气压 */
    10000,  /* 5 土温 */
    8000,   /* 6 土湿 */
    8000,   /* 7 EC */
};

static uint32_t s_last_collect = 0;
static uint32_t s_last_screen  = 0;
static uint8_t  s_screen_idx   = 0;

void APP_Init(void)
{
    Bus_Init();
    s_last_collect = 0;
    s_last_screen  = 0;
    s_screen_idx   = 0;
}

void APP_Loop(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  text[32];
    uint16_t tlen;

    Bus_Tick1ms();

    if (now - s_last_collect >= COLLECT_PERIOD_MS) {
        s_last_collect = now;
        Sensor_Collect();
    }

    /* 当前条停留时间到了，切下一条 */
    if (now - s_last_screen >= s_dwell_ms[s_screen_idx]) {
        s_last_screen = now;

        switch (s_screen_idx) {
        case 0: Screen_FormatTemp(text, &tlen); break;
        case 1: Screen_FormatHumi(text, &tlen); break;
        case 2: Screen_FormatCO2(text, &tlen); break;
        case 3: Screen_FormatLux(text, &tlen); break;
        case 4: Screen_FormatPressure(text, &tlen); break;
        case 5: Screen_FormatSoilTemp(text, &tlen); break;
        case 6: Screen_FormatSoilHumi(text, &tlen); break;
        case 7: Screen_FormatEC(text, &tlen); break;
        default:
            s_screen_idx = 0;
            return;
        }

        Screen_ShowScroll(text, tlen);
        s_screen_idx = (s_screen_idx + 1) % SCREEN_ITEM_NUM;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Bus_OnRxByte(huart);
}
