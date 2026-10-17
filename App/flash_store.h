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
  *   [54]     已确认标志   0xFF=待发, 某位写0=已确认
  *   [55]     提交标识     0xFF=未完成, 0x00=已写入完成
  *   [56..57] CRC16        覆盖[0..55]不变部分
  *   [58..63] 保留/对齐    0xFF
  ******************************************************************************
  */
#ifndef __FLASH_STORE_H
#define __FLASH_STORE_H

#include "app_def.h"
#include <stdint.h>
#include <stdbool.h>

/* ====== 扇区地址与容量 ====== */
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
/* (128KB-256)/64 ≈ 2046 条/扇区；保守对外约 2000 条 ≈ 33h */

/* 采样节律（契约 §8.1） */
#define FLASH_STORE_SAMPLE_MS      60000   /* 每60秒持久化一条新记录 */

/* 数据定点倍率（契约 §10.6） */
#define FS_FIXED_DIV10(x)   ((int32_t)((x) * 10.0f))
#define FS_FIXED_DIV1(x)    ((int32_t)(x))

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
} FS_Err_t;

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

/* 初始化：扫描元数据+数据扇区，恢复 boot_id/代次/待发记录。上电必调 */
void FS_Init(void);

/* 读取当前 boot_id（跨重启持久化，单调递增）；首次上电为1 */
uint32_t FS_GetBootId(void);

/* 生成本次启动的启动编号：读持久值+1并安全落盘；成功后才允许发消息。
   每次上电只调用一次。返回本次 boot_id；失败返回0表示元数据不可用 */
uint32_t FS_NextBootId(void);

/* 追加一条新采样记录到缓存（按采样节律调用）。返回 FS_OK / FS_ERR_FULL */
FS_Err_t FS_Append(const FS_Record_t *rec);

/* 标记某条已确认（收到匹配ACK）。rec 按 boot_id+seq 定位。 */
void FS_MarkConfirmed(uint32_t boot_id, uint32_t seq);

/* 读出队列头部一条未确认记录（供发送）。无则返回 false。
   peek=true 不移动读指针；peek=false 出队一条 */
bool FS_Peek(FS_Record_t *rec, bool peek);

/* 队列中未确认记录数 */
uint32_t FS_PendingCount(void);

/* 丢弃计数（队列满时累计，状态页可见） */
uint32_t FS_DroppedCount(void);

/* 整扇区迁移/回收（内部，掉电安全）。由 FS_Peek 出队触发或满时调用 */
FS_Err_t FS_Recycle(void);

#endif /* __FLASH_STORE_H */
