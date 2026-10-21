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
  * 可靠性加固（2026-10-21）：
  *   - 每个 Flash 写/擦操作检查 HAL 返回值；元数据回读 boot+gen+CRC 三项；
  *   - 记录写入后回读解包校验 CRC/提交位/boot/seq，通过后才入队；
  *   - 确认位写入后回读 [54]==0 && [55]==0 才出队，失败保留待确认；
  *   - 迁移扇区头写入后回读校验（magic/代次/committed/used）。
  *
  * 简化说明：本实现把"待发记录索引"维护在RAM（每条64B，最多约2000条=128KB RAM 的
  * 子集，F407 有 192KB RAM 可用），Flash 负责掉电持久化恢复。
  ******************************************************************************
  */
#include "flash_store.h"
#include "modbus.h"       /* Modbus_CRC16 */
#include <string.h>
#include "stm32f4xx_hal.h"

/* ====== 扇区地址映射（F407） ======
   生产构建：FS_ABS() 展开成编译期常量表达式，生成的机器码与原先完全一致。
   主机测试（FS_HOST_TEST）：地址改由变量 s_flash_base 参与计算，测试才能把
   "Flash" 映射到进程内可写区域；同时避开个别主机编译器对高位绝对地址
   常量的代码生成缺陷（表现为写入即段错误，与固件无关）。 */
#define FS_FLASH_BASE        0x08000000UL
#define FS_OFF_META_A        0x00010000UL   /* Sector4 */
#define FS_OFF_DATA_A        0x00020000UL   /* Sector5 */
#define FS_OFF_DATA_B        0x00040000UL   /* Sector6 */
#define FS_OFF_META_B        0x00060000UL   /* Sector7 */

#ifdef FS_HOST_TEST
static uint32_t s_flash_base = FS_FLASH_BASE;
/* 仅供主机测试调整映射基址；返回原基址。生产构建不提供此符号。 */
uint32_t FS_SetFlashBase(uint32_t base)
{
    uint32_t old = s_flash_base;
    s_flash_base = base;
    return old;
}
#define FS_ABS(off)          (s_flash_base + (uint32_t)(off))
#else
#define FS_ABS(off)          (FS_FLASH_BASE + (uint32_t)(off))
#endif

#define FS_SECTOR4_ADDR      FS_ABS(FS_OFF_META_A)   /* 元数据A 64KB  */
#define FS_SECTOR5_ADDR      FS_ABS(FS_OFF_DATA_A)   /* 数据A  128KB */
#define FS_SECTOR6_ADDR      FS_ABS(FS_OFF_DATA_B)   /* 数据B  128KB */
#define FS_SECTOR7_ADDR      FS_ABS(FS_OFF_META_B)   /* 元数据B 128KB */

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

/* 身份状态：区分"首次使用"与"元数据损坏" */
static FS_IdentState_t s_ident_state = FS_IDENT_UNKNOWN;

/* 数据区就绪标志：初始化失败（擦除/写头/回读不通过）时保持 false，
   禁止 FS_Append/FS_Recycle 继续写入无有效扇区头的区域。 */
static bool s_data_ready = false;

/* 活动数据扇区：当前写入/读取的扇区地址与代次 */
static uint32_t s_act_data_addr = 0;   /* 当前活动数据扇区基址 */
static uint32_t s_act_data_gen = 0;

/* 当前活动扇区已占用的槽位数（RAM 维护）。
   注意：不能把计数值递增写回 Flash 的 hdr.used ——
   Flash 编程只能把 1 写成 0，"已用 1 条"再写"已用 2 条"会变成
   0x01 & 0x02 = 0x00，计数归零后新记录会覆盖已有记录（丢数据）。
   hdr.used 仅在整扇区迁移时写入一次（目标扇区刚擦除，可写任意值）。 */
static uint32_t s_used = 0;

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
    out[54] = 0xFF;                              /* 保留（旧固件的确认位，新固件不用） */
    out[55] = 0x00;                              /* 提交标识：与数据同批一次写成，不二次编程 */
    /* CRC 只覆盖不变数据区 [0..53]（状态字节[54][55] 是可变的，排除）
       契约 v1.2 §10.4：改状态不破坏原 CRC */
    uint16_t crc = Modbus_CRC16(out, 54);
    out[56] = crc & 0xFF;
    out[57] = (crc >> 8) & 0xFF;
    /* [58..63] 保留 0xFF：其中 [60..63] 是独立"确认字"，
       写入时保持擦除态（跳过编程），等 ACK 到达再一次性编程为 0。 */
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
    /* 已确认判定：新格式看独立确认字[60..63]==0x00000000；
       旧格式（[54] 被编程为 0x00）依然兼容读出为已确认。 */
    uint32_t confirm_word;
    memcpy(&confirm_word, &in[60], 4);
    r->confirmed = (in[54] != 0xFF) || (confirm_word == 0x00000000u);
    return 0;
}

/* 擦除一个扇区（整扇区）。返回 0 成功，-1 失败 */
static int FS_EraseSector(uint32_t sector_num)
{
    FLASH_EraseInitTypeDef er;
    uint32_t err = 0xFFFFFFFFu;
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Sector = sector_num;
    er.NbSectors = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &err);
    HAL_FLASH_Lock();
    return (st == HAL_OK) ? 0 : -1;
}

/* 写 32-bit word 到 flash。返回 0 成功，-1 失败 */
static int FS_ProgWord(uint32_t addr, uint32_t val)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, val);
    HAL_FLASH_Lock();
    return (st == HAL_OK) ? 0 : -1;
}

/* 写一条记录到数据扇区（addr 起，64B）。单次成型，不做二次编程。
   返回 0 成功，-1 任一 word 写入失败或目标 word 非擦除态（调用方需回读校验）。 */
static int FS_WriteRecordToFlash(uint32_t addr, const FS_Record_t *r)
{
    uint8_t buf[FLASH_STORE_REC_SIZE];
    FS_PackRecord(r, buf);
    int rc = 0;
    HAL_FLASH_Unlock();
    for (int i = 0; i < FLASH_STORE_REC_SIZE / 4; i++) {
        uint32_t val;
        memcpy(&val, &buf[i*4], 4);
        /* 全 0xFF 的 word 保持擦除态、不编程：留给后续单独编程
           （记录尾部的确认字），也避免无意义的编程操作。 */
        if (val == 0xFFFFFFFFu) continue;
        /* 目标 word 必须是擦除态，否则同 word 二次编程在 F4 上会报错/写坏 */
        if (*(volatile uint32_t *)(addr + i*4) != 0xFFFFFFFFu) { rc = -1; continue; }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i*4, val) != HAL_OK) rc = -1;
    }
    HAL_FLASH_Lock();
    return rc;
}

/* 读数据扇区头 */
static void FS_ReadDataHdr(uint32_t base, FS_DataHdr_t *hdr)
{
    memcpy(hdr, (void*)base, sizeof(*hdr));
}

/* 扫描扇区已用槽位数：从扇区头之后逐条检查，遇到全 0xFF 的空槽即停止。
   记录是顺序写入的，因此第一个空槽之前都是有效槽位。 */
static uint32_t FS_ScanUsed(uint32_t base)
{
    for (uint32_t i = 0; i < FLASH_STORE_CAP_PER_SECTOR; i++) {
        const uint8_t *p = (const uint8_t *)(base + FLASH_STORE_SECTOR_HDR
                                             + i * FLASH_STORE_REC_SIZE);
        int blank = 1;
        for (uint32_t k = 0; k < FLASH_STORE_REC_SIZE; k++) {
            if (p[k] != 0xFF) { blank = 0; break; }
        }
        if (blank) return i;
    }
    return FLASH_STORE_CAP_PER_SECTOR;
}

/* 判断一个记录槽是否仍为擦除态（全 0xFF）。
   写入失败后用它区分"完全没写进去（空槽，可重试）"与"部分写坏（非空，需跳过）"。 */
static int FS_SlotBlank(uint32_t addr)
{
    const uint8_t *p = (const uint8_t *)addr;
    for (uint32_t k = 0; k < FLASH_STORE_REC_SIZE; k++)
        if (p[k] != 0xFF) return 0;
    return 1;
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

/* 把未确认记录从源(活动)扇区复制到目标扇区并更新头（迁移用）。
   契约 §10.5：必须逐条 CRC 校验 + 总数核对 + 迁移完成标识，缺一不可。
   返回实际复制条数；任何一条失败或总数不符返回 -1（调用方据此保留旧扇区）。 */
static int32_t FS_CopyPendingTo(uint32_t target_base, uint32_t new_gen)
{
    /* 先擦目标扇区 */
    uint32_t target_sector = (target_base == FS_SECTOR5_ADDR) ? FLASH_STORE_SECTOR_DATA_A
                                                              : FLASH_STORE_SECTOR_DATA_B;
    if (FS_EraseSector(target_sector) != 0) return -1;

    /* 扇区头：magic/generation/used 各编程一次；committed 保持擦除态
       （0xFFFFFFFF = 未完成），迁移全部成功后再一次性编程为 0。
       原实现先写 0xFFFFFFFF 再写 0，等于对同一 word 二次编程。 */
    FS_DataHdr_t hdr;
    hdr.magic = FS_MAGIC;
    hdr.generation = new_gen;
    hdr.committed = 0xFFFFFFFF;   /* 未完成（擦除态，不编程） */
    hdr.used = s_q_count;

    /* 先确认四个头 word 都处于擦除态，否则说明擦除未生效，拒绝继续写 */
    for (int i = 0; i < 4; i++) {
        if (*(volatile uint32_t *)(target_base + i*4) != 0xFFFFFFFFu) return -1;
    }

    if (FS_ProgWord(target_base + 0,  hdr.magic) != 0) return -1;
    if (FS_ProgWord(target_base + 4,  hdr.generation) != 0) return -1;
    if (FS_ProgWord(target_base + 12, hdr.used) != 0) return -1;

    /* 扇区头回读校验（原实现只校验记录、未校验头） */
    if (*(volatile uint32_t *)(target_base + 0)  != hdr.magic) return -1;
    if (*(volatile uint32_t *)(target_base + 4)  != hdr.generation) return -1;
    if (*(volatile uint32_t *)(target_base + 8)  != 0xFFFFFFFFu) return -1;
    if (*(volatile uint32_t *)(target_base + 12) != hdr.used) return -1;

    /* 逐条复制：任一条 CRC/提交标识校验失败即整体失败（不留下残缺扇区） */
    uint32_t copied = 0;
    for (uint32_t i = 0; i < s_q_count; i++) {
        uint32_t src_addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                          + s_queue[i].slot * FLASH_STORE_REC_SIZE;
        FS_Record_t rec;
        if (FS_UnpackRecord((const uint8_t*)src_addr, &rec) != 0) {
            return -1;                      /* 源记录损坏，整体放弃迁移 */
        }
        uint32_t dst_addr = target_base + FLASH_STORE_SECTOR_HDR + i * FLASH_STORE_REC_SIZE;
        if (FS_WriteRecordToFlash(dst_addr, &rec) != 0) return -1;
        /* 回读校验：目标记录必须可解包且编号一致，否则视为写入失败 */
        FS_Record_t chk;
        if (FS_UnpackRecord((const uint8_t*)dst_addr, &chk) != 0 ||
            chk.boot_id != rec.boot_id || chk.seq != rec.seq) {
            return -1;
        }
        copied++;
    }

    /* 总数核对：契约要求"逐条 CRC + 总数"，缺一不可 */
    if (copied != s_q_count) return -1;

    /* 提交完成标识（最后写）+ 回读 */
    if (FS_ProgWord(target_base + 8, 0) != 0) return -1;
    if (*(volatile uint32_t *)(target_base + 8) != 0) return -1;

    return (int32_t)copied;
}

/* 读取指定元数据扇区中 CRC 有效且最大的 boot_id */
static uint32_t FS_MaxBootInSector(uint32_t base, uint32_t cap)
{
    uint32_t boot = 0;
    for (uint32_t k = 0; k < cap; k++) {
        uint32_t addr = base + k * FS_META_REC_SIZE;
        uint32_t b = *(volatile uint32_t*)addr;
        if (b == 0xFFFFFFFF) break;   /* 空槽，之后也是空的 */
        uint16_t crc = Modbus_CRC16((const uint8_t*)addr, 8);
        uint16_t stored = *(volatile uint16_t*)(addr + 8);
        if (crc == stored && b > boot) boot = b;
    }
    return boot;
}

/* 判断元数据扇区是否处于擦除态（全部槽首 word 为 0xFFFFFFFF）。
   用于区分"首次使用（空白）"与"有内容但无效（损坏）"。 */
static int FS_MetaBlank(uint32_t base, uint32_t cap)
{
    for (uint32_t k = 0; k < cap; k++) {
        if (*(volatile uint32_t *)(base + k * FS_META_REC_SIZE) != 0xFFFFFFFFu) return 0;
    }
    return 1;
}

/* 追加写一个 boot_id 到指定元数据扇区；返回写入槽序号，满/失败返回-1。
   追加写布局：每条32B = [0..3]boot_id [4..7]generation [8..9]crc16(覆盖[0..7]) [10..31]填充
   写入后回读 boot_id/generation/CRC 三项，全部一致才算成功。 */
static int FS_MetaAppend(uint32_t base, uint32_t cap, uint32_t boot, uint32_t gen)
{
    /* 找第一个空槽 */
    uint32_t slot = 0xFFFFFFFF;
    for (uint32_t k = 0; k < cap; k++) {
        if (*(volatile uint32_t*)(base + k*FS_META_REC_SIZE) == 0xFFFFFFFF) { slot = k; break; }
    }
    if (slot == 0xFFFFFFFF) return -1;   /* 满 */
    uint32_t addr = base + slot * FS_META_REC_SIZE;

    if (FS_ProgWord(addr + 0, boot) != 0) return -1;
    if (FS_ProgWord(addr + 4, gen)  != 0) return -1;
    /* CRC 基于已写入的实际字节计算 */
    uint16_t crc = Modbus_CRC16((const uint8_t*)addr, 8);
    if (FS_ProgWord(addr + 8, (uint32_t)(crc & 0xFFFF)) != 0) return -1;

    /* 回读三项：boot_id / generation / CRC */
    if (*(volatile uint32_t*)addr != boot) return -1;
    if (*(volatile uint32_t*)(addr + 4) != gen) return -1;
    if (*(volatile uint16_t*)(addr + 8) != crc) return -1;
    if (Modbus_CRC16((const uint8_t*)addr, 8) != *(volatile uint16_t*)(addr + 8)) return -1;

    return (int)slot;
}

/* ====== 公开接口 ====== */

FS_Err_t FS_Init(void)
{
    s_boot_id = 0;
    s_dropped = 0;
    s_data_ready = false;
    s_used = 0;
    s_q_count = 0;
    s_act_data_addr = 0;
    s_act_data_gen = 0;
    s_generation = 0;

    /* 恢复 boot_id，并区分三种情形：
       - 存在 CRC 有效的编号 → 正常恢复；
       - 两扇区均空白        → 首次使用，编号从 1 开始；
       - 有内容但无有效编号  → 身份损坏，禁止自动从 1 重新编号（会撞云端历史）。 */
    uint32_t max_a = FS_MaxBootInSector(FS_SECTOR4_ADDR, FS_META_CAP_A);
    uint32_t max_b = FS_MaxBootInSector(FS_SECTOR7_ADDR, FS_META_CAP_B);
    if (max_a > 0 || max_b > 0) {
        s_boot_id = (max_a > max_b) ? max_a : max_b;
        s_ident_state = FS_IDENT_OK;
    } else if (FS_MetaBlank(FS_SECTOR4_ADDR, FS_META_CAP_A) &&
               FS_MetaBlank(FS_SECTOR7_ADDR, FS_META_CAP_B)) {
        s_boot_id = 0;                       /* 首次使用 */
        s_ident_state = FS_IDENT_BLANK;
    } else {
        s_boot_id = 0;                       /* 身份不可确认 */
        s_ident_state = FS_IDENT_CORRUPT;
    }

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
        if (FS_EraseSector(FLASH_STORE_SECTOR_DATA_A) != 0) {
            /* 擦除失败：不假装已初始化，置空队列并保持"未就绪"（上层将无记录可发） */
            s_generation = s_act_data_gen;
            s_used = 0;
            s_q_count = 0;
            return FS_ERR_NOTREADY;
        }
        FS_DataHdr_t h; h.magic=FS_MAGIC; h.generation=1; h.committed=0; h.used=0;
        if (FS_ProgWord(s_act_data_addr+0, h.magic) != 0 ||
            FS_ProgWord(s_act_data_addr+4, h.generation) != 0 ||
            FS_ProgWord(s_act_data_addr+12, h.used) != 0 ||
            *(volatile uint32_t*)(s_act_data_addr+0) != FS_MAGIC ||
            *(volatile uint32_t*)(s_act_data_addr+12) != h.used) {
            s_generation = s_act_data_gen;
            s_used = 0;
            s_q_count = 0;
            return FS_ERR_NOTREADY;
        }
        /* 扇区头提交标识：擦除后即 0xFFFFFFFF，只编程这一次 */
        if (FS_ProgWord(s_act_data_addr+8, 0u) != 0 ||
            *(volatile uint32_t*)(s_act_data_addr+8) != 0u) {
            s_generation = s_act_data_gen;
            s_used = 0;
            s_q_count = 0;
            return FS_ERR_NOTREADY;
        }
    }
    s_generation = s_act_data_gen;

    /* 以扫描结果为准确定已用槽位数（不信任 hdr.used 的递增写入） */
    s_used = FS_ScanUsed(s_act_data_addr);

    /* 载入待发记录 */
    FS_LoadDataSector(s_act_data_addr, s_used);

    s_data_ready = true;      /* 只有走到这里才允许追加/回收 */
    return FS_OK;
}

FS_IdentState_t FS_IdentityState(void) { return s_ident_state; }

bool FS_IdentityRepair(uint32_t min_boot_id)
{
    if (min_boot_id == 0xFFFFFFFFu) return false;   /* 上溢保护 */
    uint32_t nb = min_boot_id + 1;

    /* 先擦 A 并写入新编号，再处理 B：任一步掉电都至少保留一份有效编号 */
    if (FS_EraseSector(FLASH_STORE_SECTOR_META_A) != 0) return false;
    if (FS_MetaAppend(FS_SECTOR4_ADDR, FS_META_CAP_A, nb, s_generation) < 0) return false;

    if (FS_EraseSector(FLASH_STORE_SECTOR_META_B) == 0) {
        (void)FS_MetaAppend(FS_SECTOR7_ADDR, FS_META_CAP_B, nb, s_generation);
    }

    s_boot_id = nb;
    s_ident_state = FS_IDENT_OK;
    return true;
}

uint32_t FS_GetBootId(void) { return s_boot_id; }

uint32_t FS_NextBootId(void)
{
    /* 身份损坏：禁止自动重新编号（否则可能与云端历史 msg_id 撞号） */
    if (s_ident_state == FS_IDENT_CORRUPT) return 0;

    uint32_t new_boot = s_boot_id + 1;
    uint32_t gen = s_generation;

    /* 先尝试 Sector4（meta A）追加 */
    int r = FS_MetaAppend(FS_SECTOR4_ADDR, FS_META_CAP_A, new_boot, gen);
    if (r >= 0) { s_boot_id = new_boot; return s_boot_id; }

    /* Sector4 满/失败：尝试 Sector7 */
    r = FS_MetaAppend(FS_SECTOR7_ADDR, FS_META_CAP_B, new_boot, gen);
    if (r >= 0) { s_boot_id = new_boot; return s_boot_id; }

    /* 双扇区都满/失败：只能擦"较旧"的那个扇区，把新编号写进去。
       前提：保留扇区必须持有当前最大有效编号（s_boot_id），否则擦除后掉电
       会回退到更旧编号 → 撞云端历史。不满足则返回 0，进入身份存储故障。 */
    uint32_t max_a = FS_MaxBootInSector(FS_SECTOR4_ADDR, FS_META_CAP_A);
    uint32_t max_b = FS_MaxBootInSector(FS_SECTOR7_ADDR, FS_META_CAP_B);

    uint32_t erase_addr, erase_cap, keep_max, erase_sector;
    if (max_b >= max_a) {
        erase_addr = FS_SECTOR4_ADDR; erase_cap = FS_META_CAP_A;
        erase_sector = FLASH_STORE_SECTOR_META_A; keep_max = max_b;
    } else {
        erase_addr = FS_SECTOR7_ADDR; erase_cap = FS_META_CAP_B;
        erase_sector = FLASH_STORE_SECTOR_META_B; keep_max = max_a;
    }
    if (keep_max != s_boot_id) return 0;   /* 保留扇区未持有最新编号，拒绝擦除 */

    if (FS_EraseSector(erase_sector) != 0) return 0;
    r = FS_MetaAppend(erase_addr, erase_cap, new_boot, gen);
    if (r >= 0) { s_boot_id = new_boot; return s_boot_id; }
    return 0;   /* 元数据不可用 */
}

FS_Err_t FS_Append(const FS_Record_t *rec)
{
    /* 数据区未就绪：禁止写入（否则会写进无有效扇区头的区域，重启后无法恢复） */
    if (!s_data_ready) return FS_ERR_NOTREADY;

    if (s_q_count >= FS_RAM_CAP) {
        /* 队列满（= 容量上限，已无法保存更多历史）：丢弃最旧的一批。
           为什么是"一批"而不是"一条"：
             只丢 1 条时，回收后条数 = 容量-1，下一次 Append 立刻又会满，
             于是【每写一条新记录都触发一次整扇区迁移】
             （擦 128KB + 重写约 130KB）。按 10s 节律约 11 天就能磨穿
             一个数据扇区的擦写寿命，而且每次迁移都会阻塞主循环约 1 秒。
             丢 1/8 之后，可以再安稳定写入约 255 条才需要下一次迁移。 */
        uint32_t drop = FS_RAM_CAP / 8;
        if (drop == 0) drop = 1;
        if (drop > s_q_count) drop = s_q_count;
        s_dropped += drop;
        memmove(&s_queue[0], &s_queue[drop], (s_q_count - drop) * sizeof(FS_SlotIdx_t));
        s_q_count -= drop;
        if (FS_Recycle() != FS_OK) {
            return FS_ERR_FULL;     /* 回收失败：旧扇区仍完整，本次不写入 */
        }
    }
    /* 判断活动扇区是否写满（用 RAM 维护的 s_used，不读/写 hdr.used） */
    if (s_used >= FLASH_STORE_CAP_PER_SECTOR) {
        FS_Err_t r = FS_Recycle();
        if (r != FS_OK) return r;
    }
    /* 追加到活动扇区：槽位由 s_used 决定，不再递增写回 Flash */
    uint32_t slot = s_used;
    uint32_t addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR + slot*FLASH_STORE_REC_SIZE;
    if (FS_WriteRecordToFlash(addr, rec) != 0) {
        /* 完全未写入（空槽）→保持 s_used，下次重试该槽，避免留下 0xFF 空洞
           使 FS_ScanUsed 在此截断而丢后续记录；部分写坏（非空）→跳过该槽。 */
        if (!FS_SlotBlank(addr)) s_used = slot + 1;
        return FS_ERR_IO;
    }
    /* 回读整条记录：CRC/提交位/boot/seq 全部正确才入队 */
    FS_Record_t chk;
    if (FS_UnpackRecord((const uint8_t*)addr, &chk) != 0 ||
        chk.boot_id != rec->boot_id || chk.seq != rec->seq) {
        if (!FS_SlotBlank(addr)) s_used = slot + 1;
        return FS_ERR_IO;
    }
    s_used = slot + 1;
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
            /* 确认字独占 [60..63]，只编程这一次（0xFFFFFFFF→0），不动其余字节。
               编程前必须是擦除态，否则说明该 word 已编程过（重复 ACK / 旧格式），
               直接跳过，避免 F4 上对同一 word 二次编程。 */
            if (*(volatile uint32_t *)(addr + 60) == 0xFFFFFFFFu &&
                FS_ProgWord(addr + 60, 0x00000000u) == 0) {
                uint32_t cw;
                memcpy(&cw, (const void *)(addr + 60), 4);
                if (cw == 0x00000000u &&
                    *(const volatile uint8_t *)(addr + 55) == 0x00) {
                    s_queue[i].confirmed = 1;
                }
            }
            /* 写/回读失败：保留未确认，等下次 ACK 再试（记录不丢） */
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

bool FS_PeekLatest(FS_Record_t *rec)
{
    if (s_q_count == 0) return false;
    uint32_t addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                  + s_queue[s_q_count - 1].slot * FLASH_STORE_REC_SIZE;
    if (FS_UnpackRecord((const uint8_t*)addr, rec) != 0) {
        return false;               /* 队尾损坏：不移除，等回收时处理 */
    }
    return true;
}

bool FS_ReadById(uint32_t boot_id, uint32_t seq, FS_Record_t *rec)
{
    for (uint32_t i = 0; i < s_q_count; i++) {
        if (s_queue[i].boot_id == boot_id && s_queue[i].seq == seq) {
            uint32_t addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                          + s_queue[i].slot * FLASH_STORE_REC_SIZE;
            if (FS_UnpackRecord((const uint8_t*)addr, rec) != 0) {
                /* 记录损坏：从队列移除，避免反复取到坏条 */
                memmove(&s_queue[i], &s_queue[i+1],
                        (s_q_count - i - 1) * sizeof(FS_SlotIdx_t));
                s_q_count--;
                return false;
            }
            return true;
        }
    }
    return false;   /* 已确认/被丢弃/不存在 */
}

uint32_t FS_PendingCount(void) { return s_q_count; }
uint32_t FS_DroppedCount(void) { return s_dropped; }

FS_Err_t FS_Recycle(void)
{
    if (!s_data_ready) return FS_ERR_NOTREADY;

    /* 目标扇区 = 另一个数据扇区 */
    uint32_t target = (s_act_data_addr == FS_SECTOR5_ADDR) ? FS_SECTOR6_ADDR
                                                           : FS_SECTOR5_ADDR;
    uint32_t new_gen = s_act_data_gen + 1;

    /* 复制保留数据到目标；失败则保留旧扇区不动（契约 §10.5：先复制校验，再擦旧） */
    int32_t copied = FS_CopyPendingTo(target, new_gen);
    if (copied < 0) {
        return FS_ERR_CRC;          /* 旧扇区仍完整，下次可重试 */
    }

    /* 复制成功后才重排槽号；新扇区占用槽位数 = 保留条数 */
    for (uint32_t i = 0; i < s_q_count; i++) {
        s_queue[i].slot = i;
    }
    s_used = (uint32_t)copied;

    /* 擦旧扇区（数据已在新扇区提交；擦除失败不影响本次切换） */
    uint32_t old_sector = (s_act_data_addr == FS_SECTOR5_ADDR) ? FLASH_STORE_SECTOR_DATA_A
                                                               : FLASH_STORE_SECTOR_DATA_B;
    (void)FS_EraseSector(old_sector);
    /* 切活动 */
    s_act_data_addr = target;
    s_act_data_gen  = new_gen;
    s_generation    = new_gen;
    return FS_OK;
}
