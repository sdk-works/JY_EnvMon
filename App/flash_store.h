/**
  ******************************************************************************
  * @file    flash_store.h
  * @brief   内部Flash可靠缓存：双扇区轮换 + 独立元数据扇区
  *          契约 v1.2 §10
  *
  * 数据区：Sector5(0x08020000) / Sector6(0x08040000)，各128KB，轮换写
  * 元数据：Sector4(0x08010000) / Sector7(0x08060000)，独立扇区，boot_id/代次
  *
 * 记录格式 64B（小端）：
 *   [0..3]   boot_id      uint32
 *   [4..7]   seq          uint32
 *   [8..11]  sampled_at   uint32 (秒, 无RTC=0)
 *   [12..19] uptime_ms    uint64 (采样时刻运行时长)
 *   [20]     time_quality 0/1
 *   [21]     valid 位图   8 bit (bit0=temp ... bit7=soil_ec)
 *   [22..53] data[8]      8×uint32 (光照用uint32, 其余int32定点)
 *   [54]     保留         恒 0xFF（旧固件用此位表示确认，读取时兼容）
 *   [55]     提交标识     写入时一次置 0x00（不做二次编程）
 *   [56..57] CRC16        覆盖[0..53]不变部分
 *   [58..59] 保留         0xFF
 *   [60..63] 确认字       0xFFFFFFFF=待确认, 0x00000000=已确认,
 *                         其它值=状态不确定（写确认字时掉电/位翻转）→ 仍需补传，
 *                         通过"复制到新槽"恢复可写状态后重发；绝不当成已确认
 *                         （云端按 msg_id 幂等去重，重复可处理，漏传不可补救）
 *
 * 【编程约束】STM32F4 只能把 1 写成 0，且同一 word 不做二次编程。
 *   - 记录写入为"逐 word 一次成型"：全 0xFF 的 word 直接跳过（保持擦除态），
 *     以便后续单独编程；
 *   - 确认字独占一个 word，在收到 ACK 时由 0xFFFFFFFF 编程为 0x00000000，
 *     全程只编程这一次，不触碰记录其余字节；
 *   - 扇区头的 committed 同样只编程一次（擦除后即 0xFFFFFFFF，不再预写）。
 ******************************************************************************
  */
#ifndef __FLASH_STORE_H
#define __FLASH_STORE_H

#include "app_def.h"
#include <stdint.h>
#include <stdbool.h>

/* ====== 扇区地址与容量 ====== */
/* 【代码区上限】Sector4 起始 0x08010000，因此程序镜像必须限制在 64KB 以内
   （uvprojx 中 IROM1 Size = 0x10000）。一旦代码增长越过 0x08010000，
   擦写 S4~S7 就会破坏程序本身 —— 链接阶段会直接报空间不足，属预期保护。 */
#define FLASH_STORE_SECTOR_META_A   4        /* Sector4: 元数据1 (64KB) */
#define FLASH_STORE_SECTOR_META_B   7        /* Sector7: 元数据2 (128KB) */
#define FLASH_STORE_SECTOR_DATA_A   5        /* Sector5: 数据A   (128KB) */
#define FLASH_STORE_SECTOR_DATA_B   6        /* Sector6: 数据B   (128KB) */
#define FLASH_STORE_DATA_SECTOR_SIZE   (128*1024)   /* 数据扇区字节容量 */
#define FLASH_STORE_META_SECTOR_A_SIZE (64*1024)    /* Sector4 64KB */
#define FLASH_STORE_META_SECTOR_B_SIZE (128*1024)   /* Sector7 128KB */

/* 单条记录 64B；数据扇区头预留 256B，实际可用条数 */
#define FLASH_STORE_REC_SIZE       64
#define FLASH_STORE_SECTOR_HDR     256
#define FLASH_STORE_CAP_PER_SECTOR ((FLASH_STORE_DATA_SECTOR_SIZE - FLASH_STORE_SECTOR_HDR) / FLASH_STORE_REC_SIZE)
/* (128KB-256)/64 = 2044 条/扇区 */

/* 采样节律【不在本模块定义】
   唯一控制点是 dtu.h 的 DTU_NEW_RECORD_MS（快速验证 10s / 生产 60s）。
   此处原先另有一份 FLASH_STORE_SAMPLE_MS=60000，与 dtu.h 的 10s 互相矛盾
   且从未被引用，已删除，避免"两处数值谁是准"的歧义。
   容量换算：2044 条/扇区 → 10s 约 5h40m，60s 约 34h。 */

/* 数据定点倍率（契约 §10.6）
   注意：光照量用 FS_FIXED_UINT1，保持 uint32 语义，禁止走 float/int32 中转。 */
#define FS_FIXED_DIV10(x)   ((int32_t)((x) * 10.0f))
#define FS_FIXED_DIV1(x)    ((int32_t)(x))
#define FS_FIXED_UINT1(x)   ((uint32_t)(x))

/* 8 通道索引（与 valid 位图、契约 §3.2 对应） */
enum {
    FS_CH_TEMPERATURE = 0,
    FS_CH_HUMIDITY,
    FS_CH_CO2,
    FS_CH_LIGHT,
    FS_CH_PRESSURE,
    FS_CH_SOIL_TEMP,
    FS_CH_SOIL_MOISTURE,
    FS_CH_SOIL_EC,
    FS_CH_NUM   /* = 8 */
};

/* 缓存状态 */
typedef enum {
    FS_OK         = 0,
    FS_ERR_FULL   = 1,   /* 队列满，丢弃最旧未确认记录 */
    FS_ERR_CRC    = 2,
    FS_ERR_BUSY   = 3,
    FS_ERR_EMPTY  = 4,
    FS_ERR_IO     = 5,   /* Flash 写入/回读校验失败 */
    FS_ERR_NOTREADY = 6, /* 存储未就绪（初始化失败），禁止追加/回收 */
} FS_Err_t;

/* 身份（boot_id）状态：用于区分"首次使用"与"元数据损坏" */
typedef enum {
    FS_IDENT_UNKNOWN = 0,  /* 尚未扫描 */
    FS_IDENT_BLANK   = 1,  /* 两元数据扇区均空白 = 首次使用，编号从 1 开始 */
    FS_IDENT_OK      = 2,  /* 已恢复有效编号 */
    FS_IDENT_CORRUPT = 3,  /* 有内容但无有效记录 = 身份损坏，禁止自动重新编号 */
} FS_IdentState_t;

/* 一条待上传记录（内存态，从Flash读出后使用） */
typedef struct {
    uint32_t boot_id;
    uint32_t seq;
    uint32_t sampled_at;
    uint64_t uptime_ms;
    uint8_t  time_quality;
    uint8_t  valid;
    uint32_t data[FS_CH_NUM];
    bool     confirmed;    /* 已收到ACK */
} FS_Record_t;

/* ====== 接口 ====== */

/* 初始化：扫描元数据+数据扇区，恢复 boot_id/代次/待发记录。上电必调。
   返回 FS_OK 表示数据区已就绪（可追加）；FS_ERR_NOTREADY 表示数据区
   初始化失败（擦除/写头/回读校验不通过），此时严禁追加与回收。 */
FS_Err_t FS_Init(void);

/* 身份状态（首次使用 / 正常 / 损坏） */
FS_IdentState_t FS_IdentityState(void);

/* 身份损坏时的显式恢复：由人工确认云端历史后调用，min_boot_id 为安全下限
   （新编号 = max(min_boot_id, 已知最大编号) + 1）。
   【单调性】min_boot_id 低于本机已知最大编号（内存编号 / 两元数据扇区有效最大 /
   活动数据区有效最大）时直接返回 false，且不擦除任何内容；
   因此本接口不可能把编号往小改（例如当前 3 时传入 0 不会变成 1）。
   不提供任何自动调用路径，避免静默撞号。
   返回 true 表示已重写并校验成功；调用后需重启（或重新 Dtu_Init）生效。 */
bool FS_IdentityRepair(uint32_t min_boot_id);

/* 读取当前 boot_id（跨重启持久化，单调递增）；首次上电为1。
   身份损坏（FS_IDENT_CORRUPT）时返回 0。 */
uint32_t FS_GetBootId(void);

/* 生成本次启动的启动编号：读持久值+1并安全落盘；成功后才允许发消息。
   每次上电只调用一次。返回本次 boot_id；0 表示身份不可用（损坏或写失败） */
uint32_t FS_NextBootId(void);

/* 追加一条新采样记录到缓存（按采样节律调用）。返回 FS_OK / FS_ERR_FULL */
FS_Err_t FS_Append(const FS_Record_t *rec);

/* 标记某条已确认（收到匹配ACK）。rec 按 boot_id+seq 定位。 */
void FS_MarkConfirmed(uint32_t boot_id, uint32_t seq);

/* 读出队列头部一条未确认记录（供发送）。无则返回 false。
   peek=true 不移动读指针；peek=false 出队一条 */
bool FS_Peek(FS_Record_t *rec, bool peek);

/* 读出队列尾部一条未确认记录（最新一条），供契约 §8.2 插队发送。
   不出队；无则返回 false。 */
bool FS_PeekLatest(FS_Record_t *rec);

/* 按完整编号读取某条待发记录（用于重发"当前 pending"，而不是队首）。
   记录已确认/被丢弃/损坏时返回 false。 */
bool FS_ReadById(uint32_t boot_id, uint32_t seq, FS_Record_t *rec);

/* 队列中未确认记录数 */
uint32_t FS_PendingCount(void);

/* 丢弃计数（队列满时累计，状态页可见） */
uint32_t FS_DroppedCount(void);

/* 整扇区迁移/回收（内部，掉电安全）。由 FS_Peek 出队触发或满时调用 */
FS_Err_t FS_Recycle(void);

/* ==================== 迁移可观测性（上板测试用） ====================
 * 迁移是本模块唯一长时间阻塞主循环的操作：擦 2 个扇区 + 复制最多 2044 条记录。
 * 上板必须知道"断电时正处在哪个阶段"以及"迁移到底阻塞多久"，故提供阶段标记、
 * 进度回调与统计计数。三者都不改变数据路径，只是把内部时序暴露出来。 */

/* 迁移阶段编号（阶段回调的入参也是这些值） */
typedef enum {
    FS_PHASE_IDLE         = 0,   /* 不在迁移中 */
    FS_PHASE_ERASE_TARGET = 1,   /* 擦目标扇区 */
    FS_PHASE_COPY_RECORDS = 2,   /* 逐条复制 + 回读校验 */
    FS_PHASE_COMMIT       = 3,   /* 写迁移完成标识（committed 0xFFFFFFFF→0） */
    FS_PHASE_ERASE_OLD    = 4,   /* 擦旧扇区（已在 RAM 侧切换完成） */
} FS_Phase_t;

/* 上次上电"死在迁移的哪个阶段"——由 Flash 现场反推，不需要示波器/GPIO 打点。
   原理：一次迁移在 Flash 里留下的痕迹是确定的，上电扫描两个数据扇区即可判定：
     - 目标扇区整片空白、旧扇区已提交  → 正常稳态，或死在擦目标阶段（两者不可区分）
     - 目标扇区头已写但 committed 仍为 0xFFFFFFFF → 死在复制阶段
     - 两个扇区都已提交且代次相邻     → 死在提交之后、擦旧扇区之前/之中
   "擦目标阶段"断电不留痕迹也无需区分：那时旧扇区完好，数据一条没少。
   注意：迁移"校验失败返回 -1"留下的痕迹与断电完全相同，故统一叫"中断"。 */
typedef enum {
    FS_MIG_INT_NONE         = 0,   /* 未检测到中断（或中断发生在擦目标阶段，无痕迹无危害） */
    FS_MIG_INT_COPY         = 2,   /* 复制记录阶段中断（目标扇区头已写、未提交） */
    FS_MIG_INT_AFTER_COMMIT = 3,   /* 已提交、擦旧扇区之前/之中中断 */
} FS_MigIntPhase_t;

typedef struct {
    uint32_t migrate_ok;     /* 本次上电成功迁移次数 */
    uint32_t migrate_fail;   /* 本次上电迁移失败次数（失败时旧扇区保持有效） */
    uint32_t last_ms;        /* 最近一次成功迁移耗时（ms，含 2 次擦除） */
    uint32_t max_ms;         /* 本次上电最长迁移耗时（ms） */
    uint8_t  phase;          /* 当前/最近阶段 FS_Phase_t */
    uint8_t  fail_phase;     /* 最近一次失败时所处阶段 */
    uint8_t  last_int_phase; /* 上次断电/中断所处阶段 FS_MigIntPhase_t（扫描时确定，跨复位有效） */
    uint8_t  _rsv;
} FS_Diag_t;

/* 复制一条记录的估算、擦除一次扇区的估算都不做假设：
   上板实测 last_ms/max_ms 才是唯一依据（LSI 驱动的看门狗超时也有容差）。 */
void FS_GetDiag(FS_Diag_t *out);

/* 进度回调：每擦完一个扇区、每复制 FS_PROGRESS_STEP 条记录、写完成标识后各调一次。
   用途：在长时间阻塞期间按【真实进展】喂看门狗 ——
     迁移最坏情况（擦 2×128KB + 复制 2044 条）明显可能超过 4s 的 IWDG 超时，
     若不喂狗会在迁移中途被复位；若整段无条件喂狗，则真正的 Flash 卡死
     又不会被发现。按进展喂狗两者兼顾：有进展说明工作正常，卡住则不喂。
   回调在临界路径上，必须极短（HAL_IWDG_Refresh 即可）。传 NULL 取消。 */
void FS_SetProgressHook(void (*fn)(void));

/* 阶段变化回调：进/出每个阶段各调一次（结束时 phase=FS_PHASE_IDLE）。
   用途：GPIO 打点接示波器/逻辑分析仪，做"定点断电"测试；或调试串口打日志。
   同样必须极短。传 NULL 取消。 */
void FS_SetPhaseHook(void (*fn)(uint8_t phase));

/* 进度回调的触发粒度（复制记录的条数间隔） */
#define FS_PROGRESS_STEP 32

#endif /* __FLASH_STORE_H */
