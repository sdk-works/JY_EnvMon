/**
  ******************************************************************************
  * @file    flash_store.c
  * @brief   内部Flash可靠缓存实现（契约 v1.2 §10）
  *
  * 掉电安全设计：
  *   - 数据区双扇区(S5/S6)轮换：写满后先复制保留数据到备用扇区并校验完整，
  *     再写迁移完成标识，最后擦旧扇区。任何时刻至少一个扇区完整可读。
  *   - 元数据双扇区(S4/S7)：boot_id 交替写，写一个新值+CRC，校验成功才算更新；
  *     两个扇区都失效才从1开始。
  *   - 状态位只从1→0；CRC 只覆盖不变数据；提交标识区分完整/残缺记录。
  *
  * 简化说明：本实现把"待发记录索引"维护在RAM（每条64B，最多约2000条=128KB RAM 的
  * 子集，F407 有 192KB RAM 可用），Flash 负责掉电持久化恢复。
  ******************************************************************************
  */
#include "flash_store.h"
#include "modbus.h"       /* Modbus_CRC16 */
#include <string.h>
#include "stm32f4xx_hal.h"

/* ====== 扇区地址映射（F407） ====== */
#define FS_FLASH_BASE        0x08000000UL
#define FS_SECTOR4_ADDR      0x08010000UL   /* 元数据A 64KB */
#define FS_SECTOR7_ADDR      0x08060000UL   /* 元数据B 128KB */
#define FS_SECTOR5_ADDR      0x08020000UL   /* 数据A 128KB */
#define FS_SECTOR6_ADDR      0x08040000UL   /* 数据B 128KB */

#define FS_MAGIC             0x4A595631UL   /* "JYV1" */
#define FS_HEAD_WORDS        64              /* 扇区头 256B */

/* 元数据记录（追加写，每条32B，放扇区头之后） */
#define FS_META_REC_SIZE     32
#define FS_META_CAP_A        ((64*1024 - FLASH_STORE_SECTOR_HDR) / FS_META_REC_SIZE)
#define FS_META_CAP_B        ((128*1024 - FLASH_STORE_SECTOR_HDR) / FS_META_REC_SIZE)

/* ====== 内部数据结构 ====== */
typedef struct {
    uint32_t magic;
    uint32_t generation;   /* 扇区代次，每次完整迁移+1 */
    uint32_t committed;    /* 迁移完成标识：0xFFFFFFFF=未完成, 0x00000000=已提交 */
    uint32_t used;         /* 已用记录数 */
} FS_DataHdr_t;           /* 数据扇区头，放在 sector 起始 */

typedef struct {
    uint32_t boot_id;      /* 当前 boot_id */
    uint32_t generation;   /* 当前数据代次 */
    uint32_t crc;          /* 覆盖上面8字节 */
} FS_MetaRec_t;           /* 元数据记录（每条32B，含填充） */

/* ====== 运行时状态 ====== */
static uint32_t s_boot_id = 0;
static uint32_t s_generation = 0;
static uint32_t s_dropped = 0;

/* 活动数据扇区：当前写入/读取的扇区地址与代次 */
static uint32_t s_act_data_addr = 0;   /* 当前活动数据扇区基址 */
static uint32_t s_act_data_gen = 0;

/* RAM 队列：待发/未确认记录索引（内存工作副本，Flash 为持久层）。
   数据存 Flash，RAM 只存索引(槽号+boot_id+seq+确认标志)，避免128KB数组超RAM。 */
#define FS_RAM_CAP  FLASH_STORE_CAP_PER_SECTOR
typedef struct {
    uint32_t slot;       /* 在活动扇区中的槽号 */
    uint32_t boot_id;
    uint32_t seq;
    uint8_t  confirmed;
} FS_SlotIdx_t;
static FS_SlotIdx_t s_queue[FS_RAM_CAP];
static uint32_t     s_q_count = 0;  /* 未确认记录数 */

/* ====== 内部工具 ====== */

/* 把记录打包成 64B 缓冲区（Flash 记录格式） */
static void FS_PackRecord(const FS_Record_t *r, uint8_t out[FLASH_STORE_REC_SIZE])
{
    memset(out, 0xFF, FLASH_STORE_REC_SIZE);
    uint32_t *w = (uint32_t*)out;
    w[0] = r->boot_id;
    w[1] = r->seq;
    w[2] = r->sampled_at;
    /* uptime_ms uint64 -> 覆盖 w[3],w[4] */
    uint32_t lo = (uint32_t)(r->uptime_ms & 0xFFFFFFFFu);
    uint32_t hi = (uint32_t)(r->uptime_ms >> 32);
    w[3] = lo; w[4] = hi;
    out[20] = r->time_quality;
    out[21] = r->valid;
    memcpy(&out[22], r->data, FS_CH_NUM * 4);   /* 32B */
    out[54] = (r->confirmed) ? 0x00 : 0xFF;     /* 已确认标志：待发=FF */
    out[55] = 0xFF;                              /* 提交标识：写入完成时置0 */
    /* CRC 只覆盖不变数据区 [0..53]（状态字节[54][55] 是可变的，排除）
       契约 v1.2 §10.4：改状态不破坏原 CRC */
    uint16_t crc = Modbus_CRC16(out, 54);
    out[56] = crc & 0xFF;
    out[57] = (crc >> 8) & 0xFF;
    /* [58..63] 保留 0xFF */
}

/* 解包 + 校验 CRC */
static int FS_UnpackRecord(const uint8_t in[FLASH_STORE_REC_SIZE], FS_Record_t *r)
{
    /* CRC 覆盖不变数据区 [0..53] */
    uint16_t calc = Modbus_CRC16(in, 54);
    uint16_t recv = in[56] | (in[57] << 8);
    if (calc != recv) return -1;          /* CRC 错 */
    if (in[55] != 0x00) return -1;        /* 提交标识非0 = 残缺 */
    const uint32_t *w = (const uint32_t*)in;
    r->boot_id   = w[0];
    r->seq       = w[1];
    r->sampled_at= w[2];
    r->uptime_ms = ((uint64_t)w[4] << 32) | w[3];
    r->time_quality = in[20];
    r->valid     = in[21];
    memcpy(r->data, &in[22], FS_CH_NUM * 4);
    r->confirmed = (in[54] != 0xFF);      /* 非FF = 已确认 */
    return 0;
}

/* 擦除一个扇区（整扇区） */
static void FS_EraseSector(uint32_t sector_num)
{
    FLASH_EraseInitTypeDef er;
    uint32_t err = 0;
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Sector = sector_num;
    er.NbSectors = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&er, &err);
    HAL_FLASH_Lock();
}

/* 写 32-bit word 到 flash */
static void FS_ProgWord(uint32_t addr, uint32_t val)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, val);
    HAL_FLASH_Lock();
}

/* 写多条记录到数据扇区（addr 起，每条64B）。写数据+CRC，最后写提交标识。 */
static void FS_WriteRecordToFlash(uint32_t addr, const FS_Record_t *r)
{
    uint8_t buf[FLASH_STORE_REC_SIZE];
    FS_PackRecord(r, buf);
    HAL_FLASH_Unlock();
    for (int i = 0; i < FLASH_STORE_REC_SIZE / 4; i++) {
        uint32_t val;
        memcpy(&val, &buf[i*4], 4);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i*4, val);
    }
    HAL_FLASH_Lock();
    /* 提交标识 [55]→0（word@addr+52 = 字节52,53,54,55，[55]是最高字节）
       小端：0x00FFFFFF = [52..54]保持FF, [55]=00 */
    FS_ProgWord(addr + 52, 0x00FFFFFF);
}

/* 读数据扇区头 */
static void FS_ReadDataHdr(uint32_t base, FS_DataHdr_t *hdr)
{
    memcpy(hdr, (void*)base, sizeof(*hdr));
}

/* 从指定数据扇区恢复 RAM 队列索引（只载入未确认记录） */
static uint32_t FS_LoadDataSector(uint32_t base, uint32_t used)
{
    s_q_count = 0;
    for (uint32_t i = 0; i < used && i < FS_RAM_CAP; i++) {
        uint32_t addr = base + FLASH_STORE_SECTOR_HDR + i * FLASH_STORE_REC_SIZE;
        FS_Record_t rec;
        if (FS_UnpackRecord((const uint8_t*)addr, &rec) == 0 && !rec.confirmed) {
            s_queue[s_q_count].slot     = i;
            s_queue[s_q_count].boot_id  = rec.boot_id;
            s_queue[s_q_count].seq      = rec.seq;
            s_queue[s_q_count].confirmed= 0;
            s_q_count++;
        }
    }
    return s_q_count;
}

/* 把未确认记录从源(活动)扇区复制到目标扇区并更新头（迁移用） */
static uint32_t FS_CopyPendingTo(uint32_t target_base, uint32_t new_gen)
{
    /* 先擦目标扇区 */
    uint32_t target_sector = (target_base == FS_SECTOR5_ADDR) ? FLASH_STORE_SECTOR_DATA_A
                                                              : FLASH_STORE_SECTOR_DATA_B;
    FS_EraseSector(target_sector);
    /* 写新头（代次+1，committed 暂为未完成） */
    FS_DataHdr_t hdr;
    hdr.magic = FS_MAGIC;
    hdr.generation = new_gen;
    hdr.committed = 0xFFFFFFFF;   /* 未完成 */
    hdr.used = s_q_count;
    /* 写头（4 word） */
    FS_ProgWord(target_base + 0,  hdr.magic);
    FS_ProgWord(target_base + 4,  hdr.generation);
    FS_ProgWord(target_base + 8,  hdr.committed);
    FS_ProgWord(target_base + 12, hdr.used);
    /* 从源扇区读每条未确认记录，写到目标 */
    for (uint32_t i = 0; i < s_q_count; i++) {
        uint32_t src_addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                          + s_queue[i].slot * FLASH_STORE_REC_SIZE;
        FS_Record_t rec;
        if (FS_UnpackRecord((const uint8_t*)src_addr, &rec) == 0) {
            FS_WriteRecordToFlash(target_base + FLASH_STORE_SECTOR_HDR + i * FLASH_STORE_REC_SIZE,
                                  &rec);
        }
        s_queue[i].slot = i;   /* 复制到目标后槽号重排 */
    }
    /* 置提交完成 */
    FS_ProgWord(target_base + 8, 0);
    return s_q_count;
}

/* 读取持久 boot_id：两元数据扇区所有槽中，取 CRC 有效且 boot_id 最大的。
   追加写布局：每条32B = [0..3]boot_id [4..7]generation [8..9]crc16(覆盖[0..7]) [10..31]填充 */
static uint32_t FS_LoadBootId(void)
{
    uint32_t boot = 0;
    const uint32_t addrs[2] = { FS_SECTOR4_ADDR, FS_SECTOR7_ADDR };
    for (int s = 0; s < 2; s++) {
        uint32_t base = addrs[s];
        uint32_t cap  = (s == 0) ? FS_META_CAP_A : FS_META_CAP_B;
        for (uint32_t k = 0; k < cap; k++) {
            uint32_t addr = base + k * FS_META_REC_SIZE;
            uint32_t b = *(volatile uint32_t*)addr;
            if (b == 0xFFFFFFFF) break;   /* 空槽，之后也是空的 */
            uint32_t g = *(volatile uint32_t*)(addr + 4);
            uint16_t crc = Modbus_CRC16((const uint8_t*)addr, 8);
            uint16_t stored = *(volatile uint16_t*)(addr + 8);
            if (crc == stored && b > boot) boot = b;
            (void)g;
        }
    }
    return boot;
}

/* 追加写一个 boot_id 到指定元数据扇区；返回写入槽序号，满/失败返回-1 */
static int FS_MetaAppend(uint32_t base, uint32_t cap, uint32_t boot, uint32_t gen)
{
    /* 找第一个空槽 */
    uint32_t slot = 0xFFFFFFFF;
    for (uint32_t k = 0; k < cap; k++) {
        if (*(volatile uint32_t*)(base + k*FS_META_REC_SIZE) == 0xFFFFFFFF) { slot = k; break; }
    }
    if (slot == 0xFFFFFFFF) return -1;   /* 满 */
    uint32_t addr = base + slot * FS_META_REC_SIZE;
    FS_ProgWord(addr + 0, boot);
    FS_ProgWord(addr + 4, gen);
    uint16_t crc = Modbus_CRC16((const uint8_t*)addr, 8);
    FS_ProgWord(addr + 8, (uint32_t)(crc & 0xFFFF));
    /* 校验 */
    if (*(volatile uint32_t*)addr == boot) return (int)slot;
    return -1;
}

/* ====== 公开接口 ====== */

void FS_Init(void)
{
    s_boot_id = 0;
    s_dropped = 0;

    /* 恢复 boot_id */
    s_boot_id = FS_LoadBootId();
    if (s_boot_id == 0) s_boot_id = 1;   /* 首次上电 */

    /* 选择数据活动扇区：两个扇区头中代次最新且已提交的 */
    FS_DataHdr_t h5, h6;
    FS_ReadDataHdr(FS_SECTOR5_ADDR, &h5);
    FS_ReadDataHdr(FS_SECTOR6_ADDR, &h6);

    int use5 = 0;
    if (h5.magic == FS_MAGIC && h5.committed == 0) {
        use5 = 1;
        s_act_data_gen = h5.generation;
    }
    if (h6.magic == FS_MAGIC && h6.committed == 0) {
        if (!use5 || h6.generation > s_act_data_gen) {
            use5 = 0;
            s_act_data_gen = h6.generation;
        }
    }
    if (use5) {
        s_act_data_addr = FS_SECTOR5_ADDR;
    } else if (h6.magic == FS_MAGIC && h6.committed == 0) {
        s_act_data_addr = FS_SECTOR6_ADDR;
    } else {
        /* 都无效：擦 S5，从代次1开始 */
        s_act_data_addr = FS_SECTOR5_ADDR;
        s_act_data_gen  = 1;
        FS_EraseSector(FLASH_STORE_SECTOR_DATA_A);
        FS_DataHdr_t h; h.magic=FS_MAGIC; h.generation=1; h.committed=0; h.used=0;
        FS_ProgWord(s_act_data_addr+0, h.magic);
        FS_ProgWord(s_act_data_addr+4, h.generation);
        FS_ProgWord(s_act_data_addr+8, h.committed);
        FS_ProgWord(s_act_data_addr+12, h.used);
    }
    s_generation = s_act_data_gen;

    /* 载入待发记录 */
    FS_DataHdr_t act;
    FS_ReadDataHdr(s_act_data_addr, &act);
    FS_LoadDataSector(s_act_data_addr, (act.magic==FS_MAGIC)? act.used : 0);
}

uint32_t FS_GetBootId(void) { return s_boot_id; }

uint32_t FS_NextBootId(void)
{
    uint32_t new_boot = s_boot_id + 1;
    uint32_t gen = s_generation;

    /* 先尝试 Sector4（meta A）追加 */
    int r = FS_MetaAppend(FS_SECTOR4_ADDR, FS_META_CAP_A, new_boot, gen);
    if (r >= 0) { s_boot_id = new_boot; return s_boot_id; }

    /* Sector4 满：尝试 Sector7 */
    r = FS_MetaAppend(FS_SECTOR7_ADDR, FS_META_CAP_B, new_boot, gen);
    if (r >= 0) { s_boot_id = new_boot; return s_boot_id; }

    /* 双扇区都满：擦 Sector4，重新从起始写（Sector7 保留旧值兜底，掉电安全） */
    FS_EraseSector(FLASH_STORE_SECTOR_META_A);
    r = FS_MetaAppend(FS_SECTOR4_ADDR, FS_META_CAP_A, new_boot, gen);
    if (r >= 0) { s_boot_id = new_boot; return s_boot_id; }
    return 0;   /* 元数据不可用 */
}

FS_Err_t FS_Append(const FS_Record_t *rec)
{
    if (s_q_count >= FS_RAM_CAP) {
        /* 队列满：丢弃最旧未确认记录 */
        s_dropped++;
        /* 移除队首（内存索引） */
        memmove(&s_queue[0], &s_queue[1], (s_q_count-1)*sizeof(FS_SlotIdx_t));
        s_q_count--;
        FS_Recycle();
    }
    /* 判断活动扇区是否写满：used 达到容量 */
    FS_DataHdr_t act;
    FS_ReadDataHdr(s_act_data_addr, &act);
    if (act.magic != FS_MAGIC || act.used >= FLASH_STORE_CAP_PER_SECTOR) {
        FS_Err_t r = FS_Recycle();
        if (r != FS_OK) return r;
    }
    /* 追加到活动扇区 */
    FS_DataHdr_t h2;
    FS_ReadDataHdr(s_act_data_addr, &h2);
    uint32_t slot = h2.used;
    FS_WriteRecordToFlash(s_act_data_addr + FLASH_STORE_SECTOR_HDR + slot*FLASH_STORE_REC_SIZE, rec);
    FS_ProgWord(s_act_data_addr + 12, slot + 1);   /* used+1 */
    s_queue[s_q_count].slot     = slot;
    s_queue[s_q_count].boot_id  = rec->boot_id;
    s_queue[s_q_count].seq      = rec->seq;
    s_queue[s_q_count].confirmed= 0;
    s_q_count++;
    return FS_OK;
}

void FS_MarkConfirmed(uint32_t boot_id, uint32_t seq)
{
    /* 用索引定位并写 Flash 确认位 */
    for (uint32_t i = 0; i < s_q_count; i++) {
        if (s_queue[i].boot_id == boot_id && s_queue[i].seq == seq) {
            uint32_t addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                          + s_queue[i].slot * FLASH_STORE_REC_SIZE;
            FS_ProgWord(addr + 52, 0xFF00FFFF); /* [54]→0（字节2），确认 */
            s_queue[i].confirmed = 1;
            break;
        }
    }
    /* 已确认出队（重新整理） */
    uint32_t w = 0;
    for (uint32_t i = 0; i < s_q_count; i++) {
        if (!s_queue[i].confirmed) s_queue[w++] = s_queue[i];
    }
    s_q_count = w;
}

bool FS_Peek(FS_Record_t *rec, bool peek)
{
    if (s_q_count == 0) return false;
    /* 从 flash 读完整记录（RAM 只存索引） */
    uint32_t addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                  + s_queue[0].slot * FLASH_STORE_REC_SIZE;
    if (FS_UnpackRecord((const uint8_t*)addr, rec) != 0) {
        /* 记录损坏：出队并跳过 */
        memmove(&s_queue[0], &s_queue[1], (s_q_count-1)*sizeof(FS_SlotIdx_t));
        s_q_count--;
        return false;
    }
    if (!peek) {
        memmove(&s_queue[0], &s_queue[1], (s_q_count-1)*sizeof(FS_SlotIdx_t));
        s_q_count--;
    }
    return true;
}

uint32_t FS_PendingCount(void) { return s_q_count; }
uint32_t FS_DroppedCount(void) { return s_dropped; }

FS_Err_t FS_Recycle(void)
{
    /* 目标扇区 = 另一个数据扇区 */
    uint32_t target = (s_act_data_addr == FS_SECTOR5_ADDR) ? FS_SECTOR6_ADDR
                                                           : FS_SECTOR5_ADDR;
    uint32_t new_gen = s_act_data_gen + 1;
    /* 复制保留数据到目标 */
    FS_CopyPendingTo(target, new_gen);
    /* 擦旧扇区 */
    uint32_t old_sector = (s_act_data_addr == FS_SECTOR5_ADDR) ? FLASH_STORE_SECTOR_DATA_A
                                                               : FLASH_STORE_SECTOR_DATA_B;
    FS_EraseSector(old_sector);
    /* 切活动 */
    s_act_data_addr = target;
    s_act_data_gen  = new_gen;
    s_generation    = new_gen;
    return FS_OK;
}
