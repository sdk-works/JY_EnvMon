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
 * 可靠性加固（2026-10-21 第二批，针对第三方审查 1d48590 的三项发现）：
 *   - 元数据"空白"判定改为整条记录（32B）全检，不再只看槽首 word：
 *     槽首被擦回 FF、槽尾仍有残留时，必须判为"身份损坏"而非"首次使用"；
 *   - FS_IdentityRepair 增加单调性下限：min_boot_id 低于已知最大编号一律拒绝，
 *     下限 = max(人工下限, 内存编号, 两元数据扇区有效最大, 活动数据区有效最大)，
 *     且全部校验在擦除之前完成，杜绝 3→1 这种编号倒退；
 *   - 确认字三态且"不确定 = 仍需补传"（FF=待确认 / 00=已确认 / 其它=不确定）：
 *     不确定值一律保留补传资格，绝不当成已确认 —— 云端按 msg_id 幂等去重，
 *     重复上传可处理；误判成功造成漏传则无法补救。不确定状态通过"迁移到新槽"
 *     恢复可写状态（新槽确认字为擦除态），再重发等 ACK；
 *   - 队列满时先迁移、成功后才丢掉最旧 1/8（原实现先丢后迁移，迁移失败会
 *     静默丢掉 255 条已持久化记录）。
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

/* ====== 迁移可观测性 ====== */
static FS_Diag_t s_migdiag;   /* 全零初始化：各阶段码 IDLE/NONE 都是 0 */
static void (*s_progress_hook)(void) = 0;
static void (*s_phase_hook)(uint8_t) = 0;

/* 报告"迁移有进展"，用于按进展喂看门狗 */
static void FS_Progress(void)
{
    if (s_progress_hook) s_progress_hook();
}

/* 切换阶段：记录 + 通知观察者（GPIO 打点 / 日志） */
static void FS_Phase(uint8_t ph)
{
    s_migdiag.phase = ph;
    if (s_phase_hook) s_phase_hook(ph);
}

void FS_GetDiag(FS_Diag_t *out)
{
    if (out) *out = s_migdiag;
}

void FS_SetProgressHook(void (*fn)(void))       { s_progress_hook = fn; }
void FS_SetPhaseHook(void (*fn)(uint8_t phase)) { s_phase_hook = fn; }

/* ====== 内部工具 ====== */

/* 整扇区迁移（内部）：first 之前的待发记录视为本次丢弃，不复制。 */
static FS_Err_t FS_RecycleFrom(uint32_t first);

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

/* 确认字三态。第三态（既非全 FF 也非全 0）= 状态不确定：
   可能是写确认字时掉电留下的半成品，也可能是位翻转。两种成因都无法排除，
   因此【不能】推断"ACK 已收到"。
   取舍：云端按 msg_id 幂等去重，重复上传可处理；误判成功导致漏传无法补救。
   所以不确定一律保留补传资格，靠"迁移到新槽"恢复可写确认状态后重发。 */
#define FS_CONFIRM_PRISTINE  0   /* 0xFFFFFFFF 待确认，可编程 */
#define FS_CONFIRM_DONE      1   /* 0x00000000 已确认 */
#define FS_CONFIRM_BAD       2   /* 其它值：状态不确定，仍需补传 */

static int FS_ConfirmState(uint32_t confirm_word)
{
    if (confirm_word == 0xFFFFFFFFu) return FS_CONFIRM_PRISTINE;
    if (confirm_word == 0x00000000u) return FS_CONFIRM_DONE;
    return FS_CONFIRM_BAD;
}

/* 读某个记录槽的确认字状态 */
static int FS_ConfirmStateAt(uint32_t rec_addr)
{
    uint32_t cw;
    memcpy(&cw, (const void *)(rec_addr + 60), 4);
    return FS_ConfirmState(cw);
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
    /* 已确认判定：新格式看独立确认字[60..63]；
       旧格式（[54] 被编程为 0x00）依然兼容读出为已确认。
       注意：确认字为"不确定值"时【不算已确认】—— 保守策略见 FS_ConfirmState。 */
    uint32_t confirm_word;
    memcpy(&confirm_word, &in[60], 4);
    r->confirmed = (in[54] != 0xFF) ||
                   (FS_ConfirmState(confirm_word) == FS_CONFIRM_DONE);
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

/* 在 [from..used) 槽范围内找出同一 (boot_id, seq) 的"更靠后的副本"。
   确认字不确定的记录会被搬到更靠后的新槽（见 FS_RelocateSlot），旧槽仍留在
   原地且 CRC 有效；重启扫描时若两条都载入队列，同一条数据会被当成两条补传。
   记录是顺序追加的，(boot_id, seq) 不会合法地重复出现，所以"靠后的为准"。 */
static int FS_HasLaterCopy(uint32_t base, uint32_t from, uint32_t used,
                           uint32_t boot, uint32_t seq)
{
    for (uint32_t i = from; i < used; i++) {
        uint32_t addr = base + FLASH_STORE_SECTOR_HDR + i * FLASH_STORE_REC_SIZE;
        FS_Record_t r;
        if (FS_UnpackRecord((const uint8_t*)addr, &r) == 0 &&
            r.boot_id == boot && r.seq == seq) {
            return 1;
        }
    }
    return 0;
}

/* 从指定数据扇区恢复 RAM 队列索引（只载入未确认记录） */
static uint32_t FS_LoadDataSector(uint32_t base, uint32_t used)
{
    s_q_count = 0;
    for (uint32_t i = 0; i < used && i < FS_RAM_CAP; i++) {
        uint32_t addr = base + FLASH_STORE_SECTOR_HDR + i * FLASH_STORE_REC_SIZE;
        FS_Record_t rec;
        if (FS_UnpackRecord((const uint8_t*)addr, &rec) == 0 && !rec.confirmed) {
            /* 确认字不确定的记录保留补传资格；但若它已经被搬到更靠后的槽，
               这里的旧槽就是陈旧副本，跳过（否则会重复补传同一 (boot,seq)）。 */
            if (FS_ConfirmStateAt(addr) == FS_CONFIRM_BAD &&
                FS_HasLaterCopy(base, i + 1, used, rec.boot_id, rec.seq)) {
                continue;
            }
            s_queue[s_q_count].slot     = i;
            s_queue[s_q_count].boot_id  = rec.boot_id;
            s_queue[s_q_count].seq      = rec.seq;
            s_queue[s_q_count].confirmed= 0;
            s_q_count++;
        }
    }
    return s_q_count;
}

/* 把队列第 idx 条记录复制到一个干净新槽，并把索引改指新槽。
   触发条件：原槽的确认字是"不确定值"——那个 word 既不能补写成 0、也不能改回 FF
   （F4 禁止同 word 二次编程），只能换一个槽才能重新获得可写确认状态。
   旧槽留在原地成为陈旧副本：重启扫描时由 FS_HasLaterCopy 去重，
   下一次整扇区迁移时自然消失（迁移只复制队列里的记录）。
   返回 0 成功。失败时队列不变，该记录继续保留补传资格。 */
static int FS_RelocateSlot(uint32_t idx)
{
    /* 扇区已满时不在 ACK 路径上做迁移：迁移会重排队列索引，且队列满时
       迁移后仍然无处可写。保留补传资格，等下一次采样触发正常回收
       （回收会把 pending 记录写进新扇区，确认字天然可写）后再确认。 */
    if (s_used >= FLASH_STORE_CAP_PER_SECTOR) return -1;

    FS_Record_t rec;
    uint32_t src = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                 + s_queue[idx].slot * FLASH_STORE_REC_SIZE;
    if (FS_UnpackRecord((const uint8_t*)src, &rec) != 0) return -1;

    uint32_t slot = s_used;
    uint32_t dst  = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                  + slot * FLASH_STORE_REC_SIZE;
    rec.confirmed = false;
    if (FS_WriteRecordToFlash(dst, &rec) != 0) {
        if (!FS_SlotBlank(dst)) s_used = slot + 1;   /* 半成品槽：跳过，不覆盖 */
        return -1;
    }
    FS_Record_t chk;
    if (FS_UnpackRecord((const uint8_t*)dst, &chk) != 0 ||
        chk.boot_id != rec.boot_id || chk.seq != rec.seq) {
        if (!FS_SlotBlank(dst)) s_used = slot + 1;
        return -1;
    }
    s_used = slot + 1;
    s_queue[idx].slot = slot;
    return 0;
}

/* 把未确认记录从源(活动)扇区复制到目标扇区并更新头（迁移用）。
   first 之前的记录视为本次要丢弃的最旧一批，不复制（队列满时用）。
   keep = s_q_count - first，即需要保留的条数。
   契约 §10.5：必须逐条 CRC 校验 + 总数核对 + 迁移完成标识，缺一不可。
   返回实际复制条数；任何一条失败或总数不符返回 -1（调用方据此保留旧扇区）。 */
static int32_t FS_CopyPendingTo(uint32_t target_base, uint32_t new_gen,
                               uint32_t first, uint32_t keep)
{
    /* 先擦目标扇区（128KB 扇区擦除在 F4 上按数据手册是"秒级"操作，
       是整个迁移里最长的单步，也是定点断电测试的第 1 个目标阶段） */
    uint32_t target_sector = (target_base == FS_SECTOR5_ADDR) ? FLASH_STORE_SECTOR_DATA_A
                                                              : FLASH_STORE_SECTOR_DATA_B;
    FS_Phase(FS_PHASE_ERASE_TARGET);
    /* 【擦除前先喂一次】上一处喂狗在主循环末尾，从那时到这里的路径是
       "采集 → 刷屏 → 串口 → FS_Append 前处理"，这三项耗时没有手册上界。
       不在擦除前补一次喂狗，最坏间隔就变成"采集+刷屏+擦除"，
       把不可测项混进了看门狗预算。补这一次后，最长间隔恒等于擦除本身。 */
    FS_Progress();
    if (FS_EraseSector(target_sector) != 0) return -1;
    FS_Progress();

    /* 扇区头：magic/generation/used 各编程一次；committed 保持擦除态
       （0xFFFFFFFF = 未完成），迁移全部成功后再一次性编程为 0。
       原实现先写 0xFFFFFFFF 再写 0，等于对同一 word 二次编程。 */
    FS_DataHdr_t hdr;
    hdr.magic = FS_MAGIC;
    hdr.generation = new_gen;
    hdr.committed = 0xFFFFFFFF;   /* 未完成（擦除态，不编程） */
    hdr.used = keep;

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

    /* 逐条复制：任一条 CRC/提交标识校验失败即整体失败（不留下残缺扇区）。
       每复制 FS_PROGRESS_STEP 条报告一次进展（按进展喂狗，见 FS_SetProgressHook）。 */
    FS_Phase(FS_PHASE_COPY_RECORDS);
    uint32_t copied = 0;
    for (uint32_t i = 0; i < keep; i++) {
        uint32_t src_addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                          + s_queue[first + i].slot * FLASH_STORE_REC_SIZE;
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
        if ((copied % FS_PROGRESS_STEP) == 0) FS_Progress();
    }

    /* 总数核对：契约要求"逐条 CRC + 总数"，缺一不可 */
    if (copied != keep) return -1;

    /* 提交完成标识（最后写）+ 回读 */
    FS_Phase(FS_PHASE_COMMIT);
    if (FS_ProgWord(target_base + 8, 0) != 0) return -1;
    if (*(volatile uint32_t *)(target_base + 8) != 0) return -1;
    FS_Progress();

    return (int32_t)copied;
}

/* 读取指定元数据扇区中 CRC 有效且最大的 boot_id */
static uint32_t FS_MaxBootInSector(uint32_t base, uint32_t cap)
{
    uint32_t boot = 0;
    for (uint32_t k = 0; k < cap; k++) {
        uint32_t addr = base + k * FS_META_REC_SIZE;
        uint32_t b = *(volatile uint32_t*)addr;
        if (b == 0xFFFFFFFF) {
            /* 槽首 word 是擦除态：本槽不可能是有效记录，但【不能 break】。
               "槽首为 FF、槽尾有残留"的槽真实存在（见 FS_MetaSlotBlank），
               它后面仍可能出现有效记录。 */
            continue;
        }
        uint16_t crc = Modbus_CRC16((const uint8_t*)addr, 8);
        uint16_t stored = *(volatile uint16_t*)(addr + 8);
        if (crc == stored && b > boot) boot = b;
    }
    return boot;
}

/* 一个元数据槽（32B）是否完全处于擦除态。
   必须整条检查：只查槽首 word 时，"槽首被擦回 FF、其余字节仍有残留"的槽
   会被误判为空白，于是"有内容但无效"被当成"首次使用"，程序自动从 1 重新
   编号 —— 直接与云端历史 msg_id 撞号。 */
static int FS_MetaSlotBlank(uint32_t addr)
{
    for (uint32_t off = 0; off < FS_META_REC_SIZE; off += 4) {
        if (*(volatile uint32_t *)(addr + off) != 0xFFFFFFFFu) return 0;
    }
    return 1;
}

/* 判断元数据扇区是否处于擦除态（每条记录 32B 全部为 0xFF）。
   用于区分"首次使用（空白）"与"有内容但无效（损坏）"。 */
static int FS_MetaBlank(uint32_t base, uint32_t cap)
{
    for (uint32_t k = 0; k < cap; k++) {
        if (!FS_MetaSlotBlank(base + k * FS_META_REC_SIZE)) return 0;
    }
    return 1;
}

/* 扫描指定数据扇区中 CRC 有效记录里的最大 boot_id。
   身份修复时用它兜底：元数据可能全丢，但数据区里还留着"这台机器用到过
   哪个编号"的证据，恢复下限不能低于它。 */
static uint32_t FS_MaxBootInData(uint32_t base, uint32_t used)
{
    uint32_t boot = 0;
    if (used > FLASH_STORE_CAP_PER_SECTOR) used = FLASH_STORE_CAP_PER_SECTOR;
    for (uint32_t i = 0; i < used; i++) {
        uint32_t addr = base + FLASH_STORE_SECTOR_HDR + i * FLASH_STORE_REC_SIZE;
        FS_Record_t rec;
        if (FS_UnpackRecord((const uint8_t*)addr, &rec) == 0 && rec.boot_id > boot) {
            boot = rec.boot_id;
        }
    }
    return boot;
}

/* 追加写一个 boot_id 到指定元数据扇区；返回写入槽序号，满/失败返回-1。
   追加写布局：每条32B = [0..3]boot_id [4..7]generation [8..9]crc16(覆盖[0..7]) [10..31]填充
   写入后回读 boot_id/generation/CRC 三项，全部一致才算成功。 */
static int FS_MetaAppend(uint32_t base, uint32_t cap, uint32_t boot, uint32_t gen)
{
    /* 找第一个"整条"空槽（槽首 FF 但槽尾脏的槽不可用：对它编程会失败，
       也会让 FS_ScanUsed 之类的顺序假设失效） */
    uint32_t slot = 0xFFFFFFFF;
    for (uint32_t k = 0; k < cap; k++) {
        if (FS_MetaSlotBlank(base + k * FS_META_REC_SIZE)) { slot = k; break; }
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

/* 上电反推"上次迁移中断在哪一阶段"（判据见 flash_store.h 的 FS_MigIntPhase_t）。
   纯读操作，不改任何 Flash 内容；上板断电测试全靠它，不需要示波器打点。 */
static uint8_t FS_InferInterrupted(const FS_DataHdr_t *h5, const FS_DataHdr_t *h6)
{
    int v5 = (h5->magic == FS_MAGIC);
    int v6 = (h6->magic == FS_MAGIC);
    int c5 = v5 && (h5->committed == 0u);
    int c6 = v6 && (h6->committed == 0u);

    if (c5 && c6) {
        /* 两个扇区都已提交：正常稳态下旧扇区会被擦掉，所以只剩一个有效。
           代次相邻说明新扇区已提交、旧扇区还没擦完（或擦除失败）。 */
        if ((h5->generation + 1u == h6->generation) ||
            (h6->generation + 1u == h5->generation)) {
            return FS_MIG_INT_AFTER_COMMIT;
        }
        return FS_MIG_INT_NONE;
    }
    /* 头已写、未提交：说明死在"逐条复制"里（或复制校验失败）。
       此时旧扇区仍完整，上电后应选旧扇区，目标扇区留待下次迁移重擦。 */
    if (v5 && !c5) return FS_MIG_INT_COPY;
    if (v6 && !c6) return FS_MIG_INT_COPY;

    /* 一空一提交 = 正常稳态；擦目标阶段中断也长这样，且旧扇区完好、无危害。 */
    return FS_MIG_INT_NONE;
}

FS_Err_t FS_Init(void)
{
    s_boot_id = 0;
    s_dropped = 0;
    /* 迁移统计按"本次上电"计数：复位后归零。跨复位只能靠云端 boot_id/seq
       与本地扇区代次(generation)推断，不在 Flash 里额外写计数（省擦写）。 */
    s_migdiag.migrate_ok   = 0;
    s_migdiag.migrate_fail = 0;
    s_migdiag.last_ms      = 0;
    s_migdiag.max_ms       = 0;
    s_migdiag.phase        = FS_PHASE_IDLE;
    s_migdiag.fail_phase   = FS_PHASE_IDLE;
    s_migdiag.last_int_phase = FS_MIG_INT_NONE;
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

    /* 先反推上次迁移中断阶段（要在选择活动扇区、可能擦 S5 之前做，否则痕迹被擦掉） */
    s_migdiag.last_int_phase = FS_InferInterrupted(&h5, &h6);

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

    /* 【恢复下限】恢复只能往前走，绝不能倒退。
       下限 = max(人工给定下限,
                  内存中的当前编号,
                  两个元数据扇区里 CRC 有效记录的最大编号,
                  活动数据区里 CRC 有效记录的最大编号)。
       任一项都必须在擦除之前读完、判完：先擦后校验的写法一旦判断有误，
       旧编号就已经没了，不能回滚。
       数据区那一项是关键兜底：元数据可能整片丢失，但数据区里的记录还留着
       "这台机器用到过哪个编号"的证据。 */
    uint32_t floor_boot = min_boot_id;
    if (s_boot_id > floor_boot) floor_boot = s_boot_id;

    uint32_t max_a = FS_MaxBootInSector(FS_SECTOR4_ADDR, FS_META_CAP_A);
    uint32_t max_b = FS_MaxBootInSector(FS_SECTOR7_ADDR, FS_META_CAP_B);
    if (max_a > floor_boot) floor_boot = max_a;
    if (max_b > floor_boot) floor_boot = max_b;

    if (s_data_ready) {
        uint32_t max_d = FS_MaxBootInData(s_act_data_addr, s_used);
        if (max_d > floor_boot) floor_boot = max_d;
    }

    if (min_boot_id < floor_boot) return false;     /* 拒绝编号倒退 */
    if (floor_boot == 0xFFFFFFFFu) return false;    /* 上溢保护 */

    uint32_t nb = floor_boot + 1;

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

    FS_Progress();          /* 擦除前补喂：本函数在 FS_Init 内调用，此时 IWDG 尚未启动，
                               但保留这次调用以覆盖"将来被移到运行期"的情况 */
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
             丢 1/8 之后，可以再安稳定写入约 255 条才需要下一次迁移。
           【顺序】必须"先迁移、成功后才丢"：旧实现先砍 RAM 队列再调 Recycle，
           一旦 Recycle 中途失败（擦除/编程出错），那 255 条就同时从 RAM 和
           上传队列里消失，而 Flash 里明明还在 —— 属于静默丢数据。
           现在把 first=drop 传给迁移：迁移只复制保留部分，失败时队列与
           Flash 都保持原样，一条不丢。 */
        uint32_t drop = FS_RAM_CAP / 8;
        if (drop == 0) drop = 1;
        if (drop >= s_q_count) drop = 1;
        if (FS_RecycleFrom(drop) != FS_OK) {
            return FS_ERR_FULL;     /* 回收失败：旧扇区与队列都完整，本次不写入 */
        }
        s_dropped += drop;
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
               编程前先判三态，避免对同一 word 二次编程（F4 会直接拒绝）。 */
            uint32_t cw = 0;
            memcpy(&cw, (const void *)(addr + 60), 4);
            int cs = FS_ConfirmState(cw);

            if (cs == FS_CONFIRM_DONE) {
                s_queue[i].confirmed = 1;      /* 重复 ACK：早已确认 */
            } else if (cs == FS_CONFIRM_BAD) {
                /* 状态不确定 → 保留补传资格。先搬到新槽恢复可写状态，
                   再在新槽上写确认字（这一步失败就继续待发，下次 ACK 再来）。 */
                if (FS_RelocateSlot(i) == 0) {
                    addr = s_act_data_addr + FLASH_STORE_SECTOR_HDR
                         + s_queue[i].slot * FLASH_STORE_REC_SIZE;
                    cs = FS_ConfirmStateAt(addr);
                } else {
                    cs = FS_CONFIRM_BAD;       /* 搬不动：继续待发 */
                }
            }

            if (cs == FS_CONFIRM_PRISTINE) {
                if (FS_ProgWord(addr + 60, 0x00000000u) == 0) {
                    uint32_t rw;
                    memcpy(&rw, (const void *)(addr + 60), 4);
                    if (rw == 0x00000000u &&
                        *(const volatile uint8_t *)(addr + 55) == 0x00) {
                        s_queue[i].confirmed = 1;
                    }
                }
                /* 编程失败（没写进去 / 写坏）：一律保留待确认。
                   写坏的 word 下次会走上面的迁移分支，不会死循环。 */
            }
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

static FS_Err_t FS_RecycleFrom(uint32_t first)
{
    if (!s_data_ready) return FS_ERR_NOTREADY;
    if (first > s_q_count) first = s_q_count;
    uint32_t keep = s_q_count - first;

    /* 耗时统计的起点：整段迁移（擦目标 + 复制 + 提交 + 擦旧）都在计时内，
       上板用 last_ms/max_ms 与看门狗超时对比，判断是否需要更长超时或改策略。 */
    uint32_t t0 = HAL_GetTick();

    /* 目标扇区 = 另一个数据扇区 */
    uint32_t target = (s_act_data_addr == FS_SECTOR5_ADDR) ? FS_SECTOR6_ADDR
                                                           : FS_SECTOR5_ADDR;
    uint32_t new_gen = s_act_data_gen + 1;

    /* 复制保留数据到目标；失败则保留旧扇区不动（契约 §10.5：先复制校验，再擦旧）。
       失败时 RAM 队列也一条未改 —— 调用方不会误以为自己已经丢了记录。 */
    int32_t copied = FS_CopyPendingTo(target, new_gen, first, keep);
    if (copied < 0) {
        /* 记录失败发生在哪个阶段：这是"断电/写坏"定位的关键信息 */
        s_migdiag.migrate_fail++;
        s_migdiag.fail_phase = s_migdiag.phase;
        FS_Phase(FS_PHASE_IDLE);
        return FS_ERR_CRC;          /* 旧扇区仍完整，下次可重试 */
    }

    /* 复制成功后才重排槽号；新扇区占用槽位数 = 保留条数 */
    for (uint32_t i = 0; i < keep; i++) {
        s_queue[i] = s_queue[first + i];
        s_queue[i].slot      = i;
        s_queue[i].confirmed = 0;
    }
    s_q_count = keep;
    s_used = (uint32_t)copied;

    /* 擦旧扇区（数据已在新扇区提交；擦除失败不影响本次切换） */
    uint32_t old_sector = (s_act_data_addr == FS_SECTOR5_ADDR) ? FLASH_STORE_SECTOR_DATA_A
                                                               : FLASH_STORE_SECTOR_DATA_B;
    FS_Phase(FS_PHASE_ERASE_OLD);
    FS_Progress();          /* 同上：擦除前补喂，避免把不可切分的擦除接到别处的阻塞后面 */
    (void)FS_EraseSector(old_sector);
    FS_Progress();
    /* 切活动 */
    s_act_data_addr = target;
    s_act_data_gen  = new_gen;
    s_generation    = new_gen;

    /* 统计：整个迁移的实测耗时 */
    s_migdiag.migrate_ok++;
    s_migdiag.last_ms = HAL_GetTick() - t0;
    if (s_migdiag.last_ms > s_migdiag.max_ms) s_migdiag.max_ms = s_migdiag.last_ms;
    FS_Phase(FS_PHASE_IDLE);
    return FS_OK;
}

FS_Err_t FS_Recycle(void) { return FS_RecycleFrom(0); }
