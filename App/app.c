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

/* ====== 上板诊断 ====== */
static AppDiag_t s_diag;
static uint32_t  s_last_loop_tick = 0;

static void App_FeedWatchdog(void);   /* 前置声明：迁移进度回调要用 */

/* 读 RCC 复位标志并转成原因码，随后清标志（否则下次复位会看到累积的旧标志）。
   判定顺序：看门狗 > 软件 > 上电/欠压 > 复位脚 —— 上电时 POR/PIN 会同时置位，
   必须先判 POR，否则一次正常上电会被记成"复位脚复位"。 */
static uint8_t App_GetResetCause(void)
{
    uint8_t cause = APP_RST_UNKNOWN;

    if      (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))  cause = APP_RST_IWDG;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))  cause = APP_RST_WWDG;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))  cause = APP_RST_LOWPOWER;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))   cause = APP_RST_SOFT;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))   cause = APP_RST_POWER_ON;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))   cause = APP_RST_BOR;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))   cause = APP_RST_PIN;

    __HAL_RCC_CLEAR_RESET_FLAGS();
    return cause;
}

void APP_GetDiag(AppDiag_t *out)
{
    if (out == NULL) return;
    *out = s_diag;
    FS_GetDiag(&out->fs);      /* 迁移统计以 flash_store 为准 */
}

void APP_Init(void)
{
    /* 复位原因必须在任何清标志之前读 */
    s_diag.reset_cause  = App_GetResetCause();
    s_diag.max_loop_ms  = 0;
    s_diag.max_collect_ms = 0;
    s_diag.max_screen_ms  = 0;
    s_diag.loop_count   = 0;

    /* 【为什么必须在 Dtu_Init 之前想清楚】整扇区迁移要擦 2 个 128KB 扇区并复制
       最多 2044 条记录，耗时是"秒级"，可能超过 IWDG 的 4s 超时。此前只在主循环
       末尾喂狗，迁移期间无法喂 → 满缓存时每次迁移都可能被看门狗复位，而复位后
       队列仍满、下一次采样又触发迁移，形成"复位→迁移→复位"的循环。
       这里注册"按进展喂狗"：擦完一个扇区、每复制 32 条、写完提交标识各喂一次。
       Flash 真卡死时没有进展 → 仍然会复位，不会把真故障喂没。
       注：FS_Init 只读扇区（最多首次使用擦一次空扇区），不会触发迁移，
       所以放在 IWDG 初始化之后注册即可（App_FeedWatchdog 另有防空指针保护）。 */
    Bus_Init();
    Dtu_Init();
    s_last_collect = 0;
    s_last_screen  = 0;
    s_screen_idx   = 0;

    /* 软件使能 IWDG（4s 超时） */
    App_IwdgInit();

    /* IWDG 起来之后再挂迁移进度回调（回调里就是 HAL_IWDG_Refresh） */
    FS_SetProgressHook(App_FeedWatchdog);

    s_last_loop_tick = HAL_GetTick();
}

/* 主循环末尾喂狗：一轮调度正常推进后才喂（契约 §11）。
   迁移期间由 flash_store 按进展调用本函数，故必须容忍"IWDG 尚未初始化"。 */
static void App_FeedWatchdog(void)
{
    if (hiwdg.Instance == NULL) return;
    HAL_IWDG_Refresh(&hiwdg);
}

void APP_Loop(void)
{
    uint32_t now = HAL_GetTick();

    /* 迭代耗时：把 485 采集、屏显阻塞发送、Flash 迁移全部包含在内的"整圈时间"。
       这是判断"离看门狗复位还有多远"的唯一直接指标，比单独量某一项更有意义。 */
    {
        uint32_t dt = now - s_last_loop_tick;
        if (dt > s_diag.max_loop_ms) s_diag.max_loop_ms = dt;
        s_last_loop_tick = now;
        s_diag.loop_count++;
    }

    Bus_Tick1ms();
    Dtu_Tick();

    if (now - s_last_collect >= COLLECT_PERIOD_MS) {
        uint32_t t0 = HAL_GetTick();
        s_last_collect = now;
        Sensor_Collect();
        {
            uint32_t dt = HAL_GetTick() - t0;
            if (dt > s_diag.max_collect_ms) s_diag.max_collect_ms = dt;
        }
    }

    /* 当前条停留时间到了，切下一条 */
    if (now - s_last_screen >= SCREEN_DWELL_MS) {
        uint32_t t0 = HAL_GetTick();
        s_last_screen = now;

        Screen_ShowItem(s_screen_idx);
        s_screen_idx = (uint8_t)((s_screen_idx + 1) % SCREEN_ITEM_NUM);

        {
            uint32_t dt = HAL_GetTick() - t0;
            if (dt > s_diag.max_screen_ms) s_diag.max_screen_ms = dt;
        }
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

/* UART 错误恢复：Flash 擦写期间若 ACK 到达触发 ORE/FE/NE/PE，
   若不处理，USART1 会永久停止接收 ACK（记录将无限重发）。
   读 SR 再读 DR 清除全部错误标志，然后重新武装单字节中断接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        volatile uint32_t tmp = huart->Instance->SR;
        tmp = huart->Instance->DR;
        (void)tmp;
        uint8_t *buf = Dtu_RxByteBuf();
        HAL_UART_Receive_IT(&huart1, buf, 1);
    }
}
