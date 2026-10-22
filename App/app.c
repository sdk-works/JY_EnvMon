/**
  ******************************************************************************
  * @file    app.c
  * @brief   京阳温室环境监测主调度
  *
 * 三层节拍（互不要求同频）：
 *   采集  快档 5s（百叶箱温湿度/CO2/光照/气压）+ 慢档 30s（土壤温湿度/EC）
 *   记录  60s 一条持久化遥测（写 Flash + 唯一 msg_id，见 dtu.h）
 *   屏显  每页停留 5s 翻页；采集成功后立即刷新当前页数值（不等上传）
  ******************************************************************************
  */
#include "app.h"
#include "sensor.h"
#include "screen.h"
#include "bus485.h"
#include "dtu.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_iwdg.h"
#include <string.h>
#include <stdio.h>

#define COLLECT_FAST_MS     5000    /* 百叶箱：温湿度 / CO2 / 光照(+气压) */
#define COLLECT_SLOW_MS     30000   /* 土壤三合一：温湿度 / EC */
#define SCREEN_ITEM_NUM     8

/* ==================== 诊断页（上板测试用，量产值 0） ====================
 * 现场没有示波器、没有调试串口，所有观测必须落在 LED 屏上：
 *   阶段 / 待发 / 丢弃 / 迁移 / 复位 / 喂狗 / 耗时 / 代次 / 采集 / 屏显
 * 打开后轮显顺序为：诊断 10 页 → 正常 8 页，上电从"阶段"页开始（断电后第一眼就能看到
 * 上次断在迁移的哪个阶段）。量产固件把这里置 0，诊断代码和字符串一起消失。
 * 【为什么加"采集""屏显"两页】喂狗间隔 = 擦除(不可切分) + 采集 + 屏显 + 串口，
 * 只报总间隔无法说明是谁占的；拆开才能判断"是不是擦除已经接近超时"。 */
#define APP_DIAG_PAGES      1
#define SCREEN_DIAG_NUM     10
#define SCREEN_DIAG_DWELL_MS 5000

/* ==================== 定点断电辅助（仅 D 组用，默认关闭） ====================
 * 没有示波器/GPIO 打点时，"断电恰好落在迁移窗口内"靠运气：正常联网时迁移只有
 * 毫秒级，积压时也才 1~2s，而间隔是 42m30s。
 * 打开本开关后，设备每 15s 主动做一次整扇区迁移 → 迁移窗口占比可达 10%~30%，
 * 随机断电几次就能命中；命中与否不用猜：下次上电屏显"阶段"页会写明上次断在
 * 哪个阶段（M2/M3，由 Flash 现场反推，见 flash_store.h 的 FS_MigIntPhase_t）。
 * 副作用：擦写次数按 15s 一次递增，只能短时使用；测完务必改回 0 重新编译。
 * 前置：先让积压达到"有记录可复制"（B 组结束时的状态最合适）。 */
#define APP_FORCED_MIG_TEST   0
#define APP_FORCED_MIG_MS     15000

/* IWDG 超时 8s，取值依据全部来自 STM32F407 数据手册（DS8626）：
 *   - LSI 频率 17 / 32 / 47 kHz（min/typ/max），故"标称 4s"实际是 2.72s~7.53s；
 *   - 128KB 扇区擦除（PSIZE=x32）：typ 1s，max 2s。这是整条迁移路径上唯一
 *     无法被"按进展喂狗"切分的单步（HAL_FLASHEx_Erase 内部轮询到擦完才返回）；
 *   - 若仍用 4s 标称：LSI 偏快（47kHz）时只有 2.72s，对 2s 的擦除上限仅余 0.7s，
 *     批次差异 + 高温就可能擦到一半被复位；
 *   - 改 8s 标称：47kHz 时 5.45s，对 2s 擦除上限有 2.7 倍余量；其余步骤（每复制
 *     32 条喂一次，字编程 max 100µs ⇒ 单段约 50ms）远在超时之内。
 * 代价：真卡死时最坏 15s（17kHz）才复位；对采样设备可接受。
 * 若确认实测 max_feed_gap_ms 远小于 2s，可把本值改回 4000。 */
#define IWDG_TIMEOUT_MS     8000
IWDG_HandleTypeDef hiwdg;

static void App_IwdgInit(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;   /* LSI~32kHz/256 = 125Hz */
    hiwdg.Init.Reload    = (uint32_t)(125u * (IWDG_TIMEOUT_MS / 1000u)); /* 8s = 1000 */
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        /* IWDG 使能失败不应阻塞主功能 */
    }
}

/* 每项指标静态显示停留时间：统一 5 秒（名称/数值/单位始终同屏） */
#define SCREEN_DWELL_MS     5000

static uint32_t s_last_collect_fast = 0;
static uint32_t s_last_collect_slow = 0;
static uint32_t s_last_screen  = 0;
static uint8_t  s_screen_idx   = 0;   /* 下一屏要显示的页号 */
static uint8_t  s_current_idx  = 0;   /* 当前已显示的页号（用于采集后就地刷新） */
static uint8_t  s_displayed    = 0;   /* 是否已经显示过至少一屏 */

/* ====== 上板诊断 ====== */
static AppDiag_t s_diag;
static uint32_t  s_last_loop_tick = 0;
static uint32_t  s_last_feed_tick = 0;
#if APP_FORCED_MIG_TEST
static uint32_t  s_last_forced_mig = 0;
#endif

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

#if APP_DIAG_PAGES
/* ---- 诊断页：名称用 GB2312（2 个汉字 = 4 字节），数值/单位用 ASCII(≤5) ---- */
static const uint8_t DN_PHASE[] = {0xBD, 0xD7, 0xB6, 0xCE};   /* 阶段 */
static const uint8_t DN_PEND [] = {0xB4, 0xFD, 0xB7, 0xA2};   /* 待发 */
static const uint8_t DN_DROP [] = {0xB6, 0xAA, 0xC6, 0xFA};   /* 丢弃 */
static const uint8_t DN_MIG  [] = {0xC7, 0xA8, 0xD2, 0xC6};   /* 迁移 */
static const uint8_t DN_RST  [] = {0xB8, 0xB4, 0xCE, 0xBB};   /* 复位 */
static const uint8_t DN_FEED [] = {0xCE, 0xB9, 0xB9, 0xB7};   /* 喂狗 */
static const uint8_t DN_COST [] = {0xBA, 0xC4, 0xCA, 0xB1};   /* 耗时 */
static const uint8_t DN_BOOT [] = {0xB4, 0xFA, 0xB4, 0xCE};   /* 代次 */
static const uint8_t DN_COLL [] = {0xB2, 0xC9, 0xBC, 0xAF};   /* 采集 */
static const uint8_t DN_SCR  [] = {0xC6, 0xC1, 0xCF, 0xD4};   /* 屏显 */

/* 复位原因 -> 4 字符内缩写，方便屏显 */
static const char *App_RstShort(uint8_t c)
{
    switch (c) {
    case APP_RST_POWER_ON: return "POR";
    case APP_RST_BOR:      return "BOR";
    case APP_RST_PIN:      return "PIN";
    case APP_RST_SOFT:     return "SOFT";
    case APP_RST_IWDG:     return "IWDG";   /* 出现即说明有阻塞超过超时 */
    case APP_RST_WWDG:     return "WWDG";
    case APP_RST_LOWPOWER: return "LPWR";
    default:               return "RST";
    }
}

static void App_ShowDiag(uint8_t i)
{
    char val[12];
    char uni[8];
    const uint8_t *name = DN_PHASE;
    AppDiag_t d;

    APP_GetDiag(&d);
    val[0] = '\0';
    uni[0] = '\0';

    switch (i) {
    case 0:   /* 上次断电/中断时迁移所处阶段（Flash 现场反推，跨复位有效） */
        name = DN_PHASE;
        strcpy(val, (d.fs.last_int_phase == FS_MIG_INT_COPY)          ? "M2" :
                    (d.fs.last_int_phase == FS_MIG_INT_AFTER_COMMIT)  ? "M3" : "OK");
        break;
    case 1:   /* 未确认队列长度：断网积压 / 补传下降都看这个 */
        name = DN_PEND;
        (void)sprintf(val, "%lu", (unsigned long)Dtu_PendingCount());
        break;
    case 2:   /* 容量淘汰累计：正常联网时应长期为 0，满缓存后每次 +255 */
        name = DN_DROP;
        (void)sprintf(val, "%lu", (unsigned long)Dtu_DroppedCount());
        break;
    case 3:   /* 迁移成功次数 + 失败次数（F 后为失败数） */
        name = DN_MIG;
        (void)sprintf(val, "%lu", (unsigned long)d.fs.migrate_ok);
        (void)sprintf(uni, "F%lu", (unsigned long)d.fs.migrate_fail);
        break;
    case 4:   /* 本次复位原因 */
        name = DN_RST;
        strcpy(val, App_RstShort(d.reset_cause));
        break;
    case 5:   /* 相邻两次喂狗的最大间隔：看门狗安全裕度的唯一直接指标 */
        name = DN_FEED;
        (void)sprintf(val, "%lu", (unsigned long)d.max_feed_gap_ms);
        strcpy(uni, "ms");
        break;
    case 6:   /* 单次迁移最长耗时（含 2 次扇区擦除） */
        name = DN_COST;
        (void)sprintf(val, "%lu", (unsigned long)d.fs.max_ms);
        strcpy(uni, "ms");
        break;
    case 7:   /* 启动代次：每次上电 +1（掉电跳号属正常） */
        name = DN_BOOT;
        (void)sprintf(val, "%lu", (unsigned long)Dtu_LastBootId());
        break;
    case 8:   /* 单次采集最长耗时（不含迁移：迁移在 Dtu_Tick 里按进展喂狗） */
        name = DN_COLL;
        (void)sprintf(val, "%lu", (unsigned long)d.max_collect_ms);
        strcpy(uni, "ms");
        break;
    case 9:   /* 单次刷屏最长耗时（串行送屏，是喂狗间隔里除擦除外的主要组成） */
        name = DN_SCR;
        (void)sprintf(val, "%lu", (unsigned long)d.max_screen_ms);
        strcpy(uni, "ms");
        break;
    default:
        return;
    }

    Screen_ShowText(name, 4, val, uni[0] ? uni : NULL);
}
#endif /* APP_DIAG_PAGES */

void APP_Init(void)
{
    /* 复位原因必须在任何清标志之前读 */
    s_diag.reset_cause  = App_GetResetCause();
    s_diag.max_loop_ms  = 0;
    s_diag.max_feed_gap_ms = 0;
    s_diag.max_collect_ms = 0;
    s_diag.max_screen_ms  = 0;
    s_diag.loop_count   = 0;

    /* 【为什么必须有按进展喂狗】整次迁移要擦 2 个 128KB 扇区、复制最多 2044 条
       64 字节记录，总耗时是"秒级"。此前只在主循环末尾喂狗，迁移期间无法喂
       → 满缓存时每次迁移都可能被看门狗复位，而复位后队列仍满、下一次采样又
       触发迁移，形成"复位→迁移→复位"的循环。
       现在的喂狗点（fs_flash_store.c，共 4 处）：
         ① 主循环末尾 App_FeedWatchdog()        —— 进入迁移前最后一次
         ② FS_CopyPendingTo：擦完目标扇区后
         ③ FS_CopyPendingTo：每复制 32 条（尾部不足 32 条不再单独喂）
         ④ FS_CopyPendingTo：写完提交标识后   —— 进入"擦旧扇区"前最后一次
         ⑤ FS_RecycleFrom：擦完旧扇区后
       所以【两次扇区擦除之间一定隔着喂狗】，不存在 2s+2s 叠加：
       最长连续不喂狗 = 单次擦除（≤2s）+ 尾部不满 32 条的复制
                       （≤31 条 × 16 word × 100µs ≈ 50ms）≈ 2.05s。
       对照最坏超时 5.45s（LSI 47kHz / PSC 256 / Reload 1000）余量 2.65 倍。
       Flash 真卡死时没有进展 → 仍然会复位，不会把真故障喂没。
       注：FS_Init 在 Dtu_Init 内部、早于下面的 App_IwdgInit()，所以它首次使用
       时的擦除发生在 IWDG 启动之前，不受超时约束（这也是它可以不喂狗的原因；
       不是"注册在 IWDG 之后所以安全"）。App_FeedWatchdog 另有空指针保护。 */
    Bus_Init();
    Dtu_Init();
    s_last_collect_fast = 0;
    s_last_collect_slow = 0;
    s_last_screen  = 0;
    s_current_idx  = 0;
    s_displayed    = 0;
#if APP_DIAG_PAGES
    s_screen_idx   = SCREEN_ITEM_NUM;   /* 上电第一屏就是诊断页（阶段在最前） */
#else
    s_screen_idx   = 0;
#endif

    /* 软件使能 IWDG（8s 超时，见 IWDG_TIMEOUT_MS 处的取值依据） */
    App_IwdgInit();
    s_last_feed_tick = HAL_GetTick();

    /* IWDG 起来之后再挂迁移进度回调（回调里就是 HAL_IWDG_Refresh） */
    FS_SetProgressHook(App_FeedWatchdog);

    s_last_loop_tick = HAL_GetTick();
}

/* 喂狗：主循环末尾 + 迁移进度回调都走这里。
   【同时统计相邻两次喂狗的间隔】——这是"离看门狗复位还有多远"的唯一直接指标，
   没有示波器也能读（屏显"喂狗"页）。注意不能用 max_loop_ms 代替：
   迁移期间是"按进展喂狗"，整圈耗时可以远大于超时而不复位。 */
static void App_FeedWatchdog(void)
{
    if (hiwdg.Instance == NULL) return;

    {
        uint32_t now = HAL_GetTick();
        uint32_t gap = now - s_last_feed_tick;
        if (gap > s_diag.max_feed_gap_ms) s_diag.max_feed_gap_ms = gap;
        s_last_feed_tick = now;
    }
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

#if APP_FORCED_MIG_TEST
    /* D 组定点断电辅助：周期性强制迁移，把"迁移窗口"从 1% 放大到可命中的比例 */
    if (now - s_last_forced_mig >= APP_FORCED_MIG_MS) {
        s_last_forced_mig = now;
        (void)FS_Recycle();
    }
#endif

    /* ---- 采集：快档 5s（百叶箱）/ 慢档 30s（土壤），两个定时器互相独立 ---- */
    {
        bool     acq_fast = false;
        bool     acq_slow = false;
        uint32_t t0, dt;

        if (now - s_last_collect_fast >= COLLECT_FAST_MS) {
            s_last_collect_fast = now;
            t0 = HAL_GetTick();
            Sensor_CollectFast();
            dt = HAL_GetTick() - t0;
            if (dt > s_diag.max_collect_ms) s_diag.max_collect_ms = dt;
            acq_fast = true;
        }
        if (now - s_last_collect_slow >= COLLECT_SLOW_MS) {
            s_last_collect_slow = now;
            t0 = HAL_GetTick();
            Sensor_CollectSlow();
            dt = HAL_GetTick() - t0;
            if (dt > s_diag.max_collect_ms) s_diag.max_collect_ms = dt;
            acq_slow = true;
        }

        /* ---- 翻页：每页停留 5s（诊断页同为 5s） ---- */
        {
            uint32_t dwell = (s_screen_idx >= SCREEN_ITEM_NUM) ? SCREEN_DIAG_DWELL_MS
                                                               : SCREEN_DWELL_MS;
            bool     switched = false;

            if (now - s_last_screen >= dwell) {
                s_last_screen = now;
                s_current_idx = s_screen_idx;
                s_displayed   = 1;
                t0 = HAL_GetTick();

#if APP_DIAG_PAGES
                if (s_screen_idx < SCREEN_ITEM_NUM) {
                    Screen_ShowItem(s_screen_idx);
                } else {
                    App_ShowDiag((uint8_t)(s_screen_idx - SCREEN_ITEM_NUM));
                }
                s_screen_idx = (uint8_t)((s_screen_idx + 1) %
                                         (SCREEN_ITEM_NUM + SCREEN_DIAG_NUM));
#else
                Screen_ShowItem(s_screen_idx);
                s_screen_idx = (uint8_t)((s_screen_idx + 1) % SCREEN_ITEM_NUM);
#endif

                dt = HAL_GetTick() - t0;
                if (dt > s_diag.max_screen_ms) s_diag.max_screen_ms = dt;
                switched = true;
            }

            /* ---- 采集成功后就地刷新当前指标页数值（无需等翻页/上传） ----
               仅当本拍没有翻页、当前是指标页、且该页依赖的采集组确实更新过时才重发，
               避免用快档数据对土壤页做无意义重绘。 */
            if (!switched && s_displayed && s_current_idx < SCREEN_ITEM_NUM &&
                ((s_current_idx <= 4 && acq_fast) || (s_current_idx >= 5 && acq_slow))) {
                t0 = HAL_GetTick();
                Screen_ShowItem(s_current_idx);
                dt = HAL_GetTick() - t0;
                if (dt > s_diag.max_screen_ms) s_diag.max_screen_ms = dt;
            }
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
