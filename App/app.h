/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用总调度：在main.c里调用APP_Init()和APP_Loop()
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#include "app_def.h"
#include "flash_store.h"

/* 在所有外设MX_*_Init()之后调用一次 */
void APP_Init(void);

/* 在while(1)里反复调用 */
void APP_Loop(void);

/* ==================== 上板诊断（不影响控制流） ==================== */

/* 复位原因（上电时读 RCC->CSR 并清零；IWDG 出现即说明某处阻塞 > 看门狗超时） */
typedef enum {
    APP_RST_UNKNOWN  = 0,
    APP_RST_POWER_ON = 1,   /* 上电/掉电复位 */
    APP_RST_BOR      = 2,   /* 欠压复位 */
    APP_RST_PIN      = 3,   /* 外部复位脚（按键/编程器复位） */
    APP_RST_SOFT     = 4,   /* 软件复位 */
    APP_RST_IWDG     = 5,   /* 独立看门狗：某处阻塞超过超时，必须查 */
    APP_RST_WWDG     = 6,   /* 窗口看门狗（未启用） */
    APP_RST_LOWPOWER = 7,   /* 低功耗/待机唤醒 */
} AppRstCause_t;

typedef struct {
    uint8_t  reset_cause;     /* 本次复位原因 AppRstCause_t */
    uint8_t  _rsv[3];
    /* 主循环单次迭代最长耗时：485 采集、屏显(9600bps 阻塞发送)、Flash 迁移
       全部串在同一条路径上。注意：迁移期间看门狗是按"进展"喂的，所以这一个
       数值可以超过超时而不代表会复位 —— 真正的看门狗指标是下面的喂狗间隔。 */
    uint32_t max_loop_ms;
    /* 【看门狗安全裕度的唯一直接指标】相邻两次喂狗之间的最大间隔，单位 ms。
       计时点在 App_FeedWatchdog()：主循环末尾 + 迁移进度回调（擦完扇区/每复制
       32 条/写完提交标识）都走这里，因此覆盖了所有阻塞路径。
       判定：必须显著小于看门狗超时的下限（见 docs/BOARD_TEST_PLAN.md）。
       没有示波器也能读，屏显"喂狗"页就是这个数。 */
    uint32_t max_feed_gap_ms;
    uint32_t max_collect_ms;  /* 单次 Sensor_Collect 最长耗时 */
    uint32_t max_screen_ms;   /* 单次屏幕刷新最长耗时 */
    uint32_t loop_count;      /* 迭代计数：停住不动即可判定死循环 */
    FS_Diag_t fs;             /* 迁移次数/耗时/阶段（读取时从 flash_store 刷新） */
} AppDiag_t;

/* 读取诊断快照（调试器 live watch / 后续接调试串口都从这里取） */
void APP_GetDiag(AppDiag_t *out);

#endif
