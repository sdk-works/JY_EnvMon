/**
  ******************************************************************************
  * @file    app.c
  * @brief   京阳温室环境监测主调度
  *
 * 周期：
 *   每3秒  采集一次传感器
 *   每8秒  切换一项屏幕指标（左名称 + 右上数值 + 右下单位）
  ******************************************************************************
  */
#include "app.h"
#include "sensor.h"
#include "screen.h"
#include "bus485.h"
#include "dtu.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_iwdg.h"

#define COLLECT_PERIOD_MS   3000
#define SCREEN_ITEM_NUM     8

/* IWDG：喂狗必须在主循环确认一轮正常推进后执行（契约 §11） */
#define IWDG_TIMEOUT_MS     4000
IWDG_HandleTypeDef hiwdg;

static void App_IwdgInit(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_128;   /* LSI~32kHz/128 = 250Hz */
    hiwdg.Init.Reload    = (uint32_t)(250 * (IWDG_TIMEOUT_MS / 1000)); /* 4s = 1000 */
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        /* IWDG 使能失败不应阻塞主功能 */
    }
}

/* 每项指标静态显示停留时间：统一 8 秒（布局固定，不再按文本宽度分档） */
#define SCREEN_DWELL_MS     8000

static uint32_t s_last_collect = 0;
static uint32_t s_last_screen  = 0;
static uint8_t  s_screen_idx   = 0;

void APP_Init(void)
{
    Bus_Init();
    Dtu_Init();
    s_last_collect = 0;
    s_last_screen  = 0;
    s_screen_idx   = 0;

    /* 软件使能 IWDG（4s 超时） */
    App_IwdgInit();
}

/* 主循环末尾喂狗：一轮调度正常推进后才喂（契约 §11） */
static void App_FeedWatchdog(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

void APP_Loop(void)
{
    uint32_t now = HAL_GetTick();

    Bus_Tick1ms();
    Dtu_Tick();

    if (now - s_last_collect >= COLLECT_PERIOD_MS) {
        s_last_collect = now;
        Sensor_Collect();
    }

    /* 当前条停留时间到了，切下一条 */
    if (now - s_last_screen >= SCREEN_DWELL_MS) {
        s_last_screen = now;

        Screen_ShowItem(s_screen_idx);
        s_screen_idx = (uint8_t)((s_screen_idx + 1) % SCREEN_ITEM_NUM);
    }

    App_FeedWatchdog();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        uint8_t *buf = Dtu_RxByteBuf();
        Dtu_OnRxByte(*buf);
        HAL_UART_Receive_IT(&huart1, buf, 1);   /* 继续收 */
    } else {
        Bus_OnRxByte(huart);
    }
}
