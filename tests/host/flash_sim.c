/**
  ******************************************************************************
  * @file    flash_sim.c
  * @brief   主机测试后端：内存 Flash 模拟 + UART 记录 + 时间基准
  *
  * 【模拟严格性 —— 这是本测试的核心价值】
  *   STM32F4 手册规定同一 word 不得重复编程。宽松模拟（把新值与原值按位与后
  *   一律返回成功）会把"对同一 word 二次编程"的缺陷掩盖成"能跑"。
  *   本后端一旦发现对非擦除态 word 再次编程，立即返回 HAL_ERROR 并累计违规计数，
  *   测试用 flash_sim_violations() 断言必须为 0。
  ******************************************************************************
  */
#include "stm32f4xx_hal.h"
#include "app_def.h"
#include "bus485.h"
#include <string.h>
#include <stdio.h>
#include <windows.h>

#define FLASH_BASE_ADDR   0x08000000u
#define FLASH_SIM_SIZE    (1u * 1024u * 1024u)   /* 0x08000000 ~ 0x080FFFFF */

/* F407 扇区表：S0-S3 16KB，S4 64KB，S5-S11 128KB */
typedef struct { uint32_t addr; uint32_t size; } Sector_t;
static const Sector_t SECTORS[12] = {
    { 0x08000000u,  16u*1024u }, { 0x08004000u,  16u*1024u },
    { 0x08008000u,  16u*1024u }, { 0x0800C000u,  16u*1024u },
    { 0x08010000u,  64u*1024u }, { 0x08020000u, 128u*1024u },
    { 0x08040000u, 128u*1024u }, { 0x08060000u, 128u*1024u },
    { 0x08080000u, 128u*1024u }, { 0x080A0000u, 128u*1024u },
    { 0x080C0000u, 128u*1024u }, { 0x080E0000u, 128u*1024u },
};

/* 把 0x08000000 起的 1MB 真正映射到进程地址空间：
   flash_store.c 里的地址计算直接解引用，必须落在合法内存上。
   用普通静态数组不行 —— 它的地址与 0x0801xxxx 不重合。 */
static uint8_t *g_flash = NULL;

static int flash_map(void)
{
    if (g_flash) return 1;
    g_flash = (uint8_t *)VirtualAlloc((void *)(uintptr_t)FLASH_BASE_ADDR, FLASH_SIM_SIZE,
                                      MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    if (!g_flash) {
        fprintf(stderr, "flash_sim: VirtualAlloc(0x%08X) 失败，无法模拟 Flash\n",
                (unsigned)FLASH_BASE_ADDR);
        return 0;
    }
    memset(g_flash, 0xFF, FLASH_SIM_SIZE);
    return 1;
}

/* 违规 / 统计计数 */
static uint32_t g_reprogram_cnt = 0;   /* 同 word 二次编程次数（必须为 0） */
static uint32_t g_outrange_cnt  = 0;   /* 越界访问 */
static uint32_t g_prog_ok_cnt   = 0;
static uint32_t g_erase_cnt     = 0;

/* 故障注入 */
int g_sim_erase_fail = 0;    /* 置 1 让下一次擦除返回 HAL_ERROR */

/* UART 记录 */
char     g_sim_tx[SIM_TX_MAX];
uint16_t g_sim_tx_len   = 0;
uint32_t g_sim_tx_count = 0;
int      g_sim_uart_fail = 0;

/* 时间基准 */
uint32_t g_sim_tick = 0;

/* 被测模块引用的全局对象 */
UART_HandleTypeDef huart1;
EnvData_t g_env;

/* ===== 测试辅助 ===== */
void flash_sim_reset(void)
{
    flash_map();
    memset(g_flash, 0xFF, FLASH_SIM_SIZE);
    g_reprogram_cnt = 0;
    g_outrange_cnt  = 0;
    g_prog_ok_cnt   = 0;
    g_erase_cnt     = 0;
    g_sim_erase_fail = 0;
    g_sim_tick      = 0;
    g_sim_tx_len    = 0;
    g_sim_tx_count  = 0;
    g_sim_uart_fail = 0;
    memset(g_sim_tx, 0, sizeof(g_sim_tx));
    memset(&g_env, 0, sizeof(g_env));
    memset(&huart1, 0, sizeof(huart1));
}

uint32_t flash_sim_violations(void) { return g_reprogram_cnt + g_outrange_cnt; }
uint32_t flash_sim_reprogram(void)  { return g_reprogram_cnt; }
uint32_t flash_sim_programmed(void) { return g_prog_ok_cnt; }
uint32_t flash_sim_erases(void)     { return g_erase_cnt; }

/* 直接往 Flash 模拟区塞数据（构造"身份损坏"等场景） */
void flash_sim_poke(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t off;
    flash_map();
    if (addr < FLASH_BASE_ADDR) return;
    off = addr - FLASH_BASE_ADDR;
    if (off + len > FLASH_SIM_SIZE) return;
    memcpy(&g_flash[off], data, len);
}

uint8_t flash_sim_peek(uint32_t addr)
{
    uint32_t off;
    flash_map();
    if (addr < FLASH_BASE_ADDR) return 0xFF;
    off = addr - FLASH_BASE_ADDR;
    if (off >= FLASH_SIM_SIZE) return 0xFF;
    return g_flash[off];
}

/* 统计某扇区被擦除后又被写入的字节数（用于判断迁移是否真的发生） */
uint32_t flash_sim_nonblank(uint32_t addr, uint32_t len)
{
    uint32_t n = 0, off;
    flash_map();
    if (addr < FLASH_BASE_ADDR) return 0;
    off = addr - FLASH_BASE_ADDR;
    if (off + len > FLASH_SIM_SIZE) len = FLASH_SIM_SIZE - off;
    for (uint32_t i = 0; i < len; i++) if (g_flash[off + i] != 0xFF) n++;
    return n;
}

/* ===== HAL 实现 ===== */
uint32_t HAL_GetTick(void) { return g_sim_tick; }

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void)   { return HAL_OK; }

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)
{
    if (!pEraseInit || !SectorError) return HAL_ERROR;
    if (g_sim_erase_fail) { g_sim_erase_fail = 0; *SectorError = pEraseInit->Sector; return HAL_ERROR; }

    for (uint32_t n = 0; n < pEraseInit->NbSectors; n++) {
        uint32_t s = pEraseInit->Sector + n;
        if (s >= 12) { *SectorError = s; return HAL_ERROR; }
        uint32_t off = SECTORS[s].addr - FLASH_BASE_ADDR;
        memset(&g_flash[off], 0xFF, SECTORS[s].size);
        g_erase_cnt++;
    }
    *SectorError = 0xFFFFFFFFu;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
    uint32_t old, nv;

    if (TypeProgram != FLASH_TYPEPROGRAM_WORD) return HAL_ERROR;
    if (Address < FLASH_BASE_ADDR || (Address - FLASH_BASE_ADDR) + 4u > FLASH_SIM_SIZE) {
        g_outrange_cnt++;
        return HAL_ERROR;
    }
    uint32_t off = Address - FLASH_BASE_ADDR;
    memcpy(&old, &g_flash[off], 4);
    nv = (uint32_t)Data;

    /* 【严格】同一 word 二次编程：真实 F4 会置 PGAERR/PGPERR（有 ECC 的型号还会
       产生不可纠正错误）。这里直接拒绝并计数，避免"按位与后照样成功"掩盖缺陷。 */
    if (old != 0xFFFFFFFFu) {
        g_reprogram_cnt++;
        return HAL_ERROR;
    }
    if ((old & nv) != nv) {   /* 只能 1→0（擦除态下恒成立，保留以表达语义） */
        g_reprogram_cnt++;
        return HAL_ERROR;
    }

    memcpy(&g_flash[off], &nv, 4);
    g_prog_ok_cnt++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
    (void)huart; (void)Timeout;
    if (g_sim_uart_fail) return HAL_ERROR;

    if (Size >= SIM_TX_MAX) Size = SIM_TX_MAX - 1;
    memcpy(g_sim_tx, pData, Size);
    g_sim_tx[Size] = '\0';
    g_sim_tx_len = Size;
    g_sim_tx_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)huart; (void)pData; (void)Size;
    return HAL_OK;
}

/* ===== 485 桩（modbus.c 链接需要，测试不走这条路） ===== */
AppErr_t Bus_Send(BusPort_t port, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    (void)port; (void)data; (void)len; (void)timeout_ms;
    return APP_ERR_TIMEOUT;
}
uint16_t Bus_PollRx(BusPort_t port, uint8_t *buf, uint16_t cap)
{
    (void)port; (void)buf; (void)cap;
    return 0;
}
void Bus_Tick1ms(void) { }
void Bus_OnRxByte(UART_HandleTypeDef *huart) { (void)huart; }
