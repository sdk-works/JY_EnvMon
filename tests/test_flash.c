/**
 * flash_store host 单元测试
 *
 * 本机无 ARMCC/Keil。用 gcc 在 host 侧编译**真实未修改**的 App/flash_store.c：
 *  - 用 mmap 把模拟 Flash 固定映射到 0x08000000，使源码中的绝对地址访问原样生效；
 *  - 模拟 Flash 编程"只能 1→0、擦除回 0xFF"的物理特性；
 *  - mock HAL_FLASH_* / HAL_GetTick / Modbus_CRC16。
 *
 * 重点验证契约 §10.5：迁移必须逐条 CRC + 总数核对 + 完成标识；
 * 失败时不得擦除旧扇区（此前实现会静默丢数据）。
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/mman.h>
#ifndef MAP_ANONYMOUS
#define MAP_ANONYMOUS MAP_ANON
#endif

#include "stm32f4xx_hal.h"
#include "modbus.h"

/* ── 模拟 Flash：映射到 STM32 真实地址，512KB ─────────────────── */
#define FLASH_BASE 0x08000000UL
#define FLASH_SIZE (512 * 1024)
static uint8_t *g_flash = NULL;

static const uint32_t SECTOR_OFF[8] = {
    0x00000, 0x04000, 0x08000, 0x0C000,
    0x10000, 0x20000, 0x40000, 0x60000,
};
static const uint32_t SECTOR_LEN[8] = {
    0x4000,  0x4000,  0x4000,  0x4000,
    0x10000, 0x20000, 0x20000, 0x20000,
};

uint32_t g_inject_fail_addr = 0xFFFFFFFFUL;   /* 注入编程失败的地址 */
int      g_erase_calls = 0;

/* ── HAL mock ─────────────────────────────────────────────────── */
void HAL_FLASH_Unlock(void) {}
void HAL_FLASH_Lock(void) {}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *er, uint32_t *err)
{
    g_erase_calls++;
    if (er->Sector >= 8) return HAL_ERROR;
    memset(g_flash + SECTOR_OFF[er->Sector], 0xFF, SECTOR_LEN[er->Sector]);
    *err = 0xFFFFFFFFU;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint32_t val)
{
    uint32_t off = addr - FLASH_BASE;
    if (off + 4 > FLASH_SIZE) return HAL_ERROR;
    if (addr == g_inject_fail_addr) return HAL_ERROR;          /* 注入失败 */
    /* Flash 物理语义：只能把 1 写成 0，不能把 0 写回 1 */
    uint8_t *p = g_flash + off;
    p[0] &= (uint8_t)(val & 0xFF);
    p[1] &= (uint8_t)((val >> 8) & 0xFF);
    p[2] &= (uint8_t)((val >> 16) & 0xFF);
    p[3] &= (uint8_t)((val >> 24) & 0xFF);
    return HAL_OK;
}

uint32_t HAL_GetTick(void) { static uint32_t t = 0; return (t += 10); }

/* ── Modbus CRC16（与 App/modbus.c 实现一致） ─────────────────── */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

/* ── 被测单元：真实源码 ───────────────────────────────────────── */
#include "flash_store.h"
#include "flash_store.c"

/* ── 断言 ─────────────────────────────────────────────────────── */
static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  [FAIL] %s\n", msg); g_fail++; } \
    else         { printf("  [ok]   %s\n", msg); } \
} while (0)

static void fs_reset(void)
{
    memset(g_flash, 0xFF, FLASH_SIZE);
    g_erase_calls = 0;
    g_inject_fail_addr = 0xFFFFFFFFUL;
    FS_Init();
}

static FS_Record_t make_rec(uint32_t boot, uint32_t seq, int32_t temp)
{
    FS_Record_t r;
    memset(&r, 0, sizeof(r));
    r.boot_id = boot; r.seq = seq;
    r.sampled_at = 0; r.time_quality = 0; r.uptime_ms = 1000UL * seq;
    r.data[FS_CH_TEMPERATURE] = temp;
    r.data[FS_CH_LIGHT] = 4000000000u;      /* 接近 uint32 上限，验证无溢出 */
    r.valid = 0xFF; r.confirmed = false;
    return r;
}

int main(void)
{
    /* 把模拟 Flash 固定映射到 0x08000000 */
    void *m = mmap((void *)FLASH_BASE, FLASH_SIZE, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    if (m == MAP_FAILED) { perror("mmap"); return 2; }
    g_flash = (uint8_t *)m;

    printf("=== 固件 flash_store 单元测试（host + mmap 模拟 Flash）===\n\n");

    /* 1. 追加 + 读回，验证 uint32 光照不溢出 */
    printf("[1] 追加记录并读回\n");
    fs_reset();
    for (uint32_t i = 1; i <= 5; i++) {
        FS_Record_t r = make_rec(1, i, 200 + (int32_t)i);
        if (FS_Append(&r) != FS_OK) { printf("  append 失败\n"); g_fail++; }
    }
    CHECK(FS_PendingCount() == 5, "待发计数 = 5");
    {
        FS_Record_t r;
        bool ok = FS_Peek(&r, true);
        CHECK(ok && r.seq == 1, "队首 seq=1");
        CHECK(r.data[FS_CH_LIGHT] == 4000000000u, "光照 uint32 完整保留（未溢出成负）");
        CHECK(r.data[FS_CH_TEMPERATURE] == 201, "温度定点值正确");
    }

    /* 2. ACK 确认后出队 */
    printf("\n[2] ACK 确认后出队\n");
    FS_MarkConfirmed(1, 1);
    CHECK(FS_PendingCount() == 4, "确认一条后待发 = 4");
    {
        FS_Record_t r;
        FS_Peek(&r, true);
        CHECK(r.seq == 2, "队首前进到 seq=2");
    }

    /* 3. 迁移完整性 */
    printf("\n[3] 扇区迁移后数据完整\n");
    {
        uint32_t before = FS_PendingCount();
        CHECK(FS_Recycle() == FS_OK, "迁移返回 FS_OK");
        CHECK(FS_PendingCount() == before, "迁移后待发条数不变");
        int bad = 0;
        uint32_t n = FS_PendingCount();
        for (uint32_t i = 0; i < n; i++) {
            FS_Record_t r;
            if (!FS_Peek(&r, true)) { bad++; break; }
            if (r.data[FS_CH_LIGHT] != 4000000000u) bad++;
            FS_MarkConfirmed(r.boot_id, r.seq);   /* 推进队首 */
        }
        CHECK(bad == 0, "迁移后所有记录 CRC/内容正确（逐条校验）");
    }

    /* 4. 核心：注入写入失败 → 必须保留旧扇区（P0-1） */
    printf("\n[4] 迁移失败时保留旧扇区（契约 §10.5）\n");
    fs_reset();
    for (uint32_t i = 1; i <= 3; i++) {
        FS_Record_t r = make_rec(2, i, 300);
        FS_Append(&r);
    }
    {
        uint32_t cnt_before = FS_PendingCount();
        uint32_t target = (s_act_data_addr == FS_SECTOR5_ADDR) ? FS_SECTOR6_ADDR : FS_SECTOR5_ADDR;
        g_inject_fail_addr = target + FLASH_STORE_SECTOR_HDR + 52;  /* 第1条提交标识 */
        FS_Err_t e = FS_Recycle();
        CHECK(e != FS_OK, "写入失败 -> 迁移返回错误（不再静默成功）");
        CHECK(FS_PendingCount() == cnt_before, "失败后待发条数不变");
        FS_Record_t r;
        bool ok = FS_Peek(&r, true);
        CHECK(ok && r.seq == 1, "失败后仍能读到原始数据（旧扇区未被擦）");
    }

    /* 5. PeekLatest（契约 §8.2 插队需要） */
    printf("\n[5] PeekLatest 返回最新记录\n");
    fs_reset();
    for (uint32_t i = 1; i <= 4; i++) {
        FS_Record_t r = make_rec(3, i, 400);
        FS_Append(&r);
    }
    {
        FS_Record_t head, tail;
        FS_Peek(&head, true);
        bool ok = FS_PeekLatest(&tail);
        CHECK(ok, "PeekLatest 返回 true");
        CHECK(head.seq == 1, "队首 seq=1（最旧）");
        CHECK(tail.seq == 4, "队尾 seq=4（最新）");
        CHECK(FS_PendingCount() == 4, "PeekLatest 不出队，计数不变");
    }

    /* 6. 源记录 CRC 损坏 → 整体迁移失败，不产生残缺扇区 */
    printf("\n[6] 源记录损坏时迁移失败\n");
    fs_reset();
    for (uint32_t i = 1; i <= 3; i++) {
        FS_Record_t r = make_rec(4, i, 500);
        FS_Append(&r);
    }
    {
        uint32_t off = (s_act_data_addr + FLASH_STORE_SECTOR_HDR + FLASH_STORE_REC_SIZE) - FLASH_BASE;
        g_flash[off] = 0x00;      /* 破坏第 2 条，CRC 必然不匹配 */
        uint32_t cb = FS_PendingCount();
        CHECK(FS_Recycle() != FS_OK, "CRC 损坏 -> 迁移失败");
        CHECK(FS_PendingCount() == cb, "失败后计数不变（不丢数据）");
    }

    printf("\n=== 结果：%s ===\n", g_fail == 0 ? "全部通过" : "存在失败");
    return g_fail == 0 ? 0 : 1;
}
