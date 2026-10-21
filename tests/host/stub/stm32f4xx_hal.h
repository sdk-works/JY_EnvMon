/**
  ******************************************************************************
  * @file    stm32f4xx_hal.h  （主机测试桩）
  * @brief   只提供被测源码实际用到的 HAL 符号，并把 Flash 操作重定向到
  *          tests/host/flash_sim.c 的内存模拟后端。
  *
  * 【关键】模拟后端按 STM32F4 手册的编程语义工作：
  *   1) 只能把 1 写成 0（擦除态 0xFF → 任何值）；
  *   2) 同一个 word 不允许二次编程 —— 一旦该 word 已被编程过（非 0xFFFFFFFF），
  *      再次编程直接返回 HAL_ERROR 并记录一次违规。
  *   这正好能抓出"对同一 word 重复编程"的代码缺陷；按位与式的宽松模拟会掩盖它。
  ******************************************************************************
  */
#ifndef __STM32F4XX_HAL_H
#define __STM32F4XX_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef struct { int _sim; } UART_HandleTypeDef;

/* 真实 bus485.h 的 Bus_t 用到 GPIO 指针；主机测试不碰引脚，给个空类型即可 */
typedef struct { int _sim; } GPIO_TypeDef;

/* 真实 main.h / app_def.h 可能引用的其它句柄类型（测试不驱动） */
typedef struct { int _sim; } TIM_HandleTypeDef;
typedef struct { int _sim; } DMA_HandleTypeDef;

/* Flash 编程/擦除常量（值取自 HAL 定义，测试只需区分即可） */
#define FLASH_TYPEPROGRAM_BYTE   0x00U
#define FLASH_TYPEPROGRAM_HALFWORD 0x01U
#define FLASH_TYPEPROGRAM_WORD   0x02U
#define FLASH_TYPEERASE_SECTORS  0x00U
#define FLASH_VOLTAGE_RANGE_3    0x02U

typedef struct {
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Sector;
    uint32_t NbSectors;
    uint32_t VoltageRange;
} FLASH_EraseInitTypeDef;

/* ===== 时间基准（测试可自由推进） ===== */
extern uint32_t g_sim_tick;
uint32_t HAL_GetTick(void);

/* ===== Flash（内存模拟） ===== */
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

/* ===== UART（记录发送内容） ===== */
#define SIM_TX_MAX 4096
extern char     g_sim_tx[SIM_TX_MAX];      /* 最近一帧报文 */
extern uint16_t g_sim_tx_len;
extern uint32_t g_sim_tx_count;            /* 累计发送帧数 */
extern int      g_sim_uart_fail;           /* 置 1 让下一次发送返回 HAL_ERROR */

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                      uint16_t Size);

/* ===== 中断屏蔽（主机为单线程，空实现） ===== */
#define __get_PRIMASK()        (0u)
#define __disable_irq()        do { } while (0)
#define __set_PRIMASK(mask)    do { (void)(mask); } while (0)

#endif /* __STM32F4XX_HAL_H */
