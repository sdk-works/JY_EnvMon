#ifndef __STM32F4XX_HAL_H_MOCK
#define __STM32F4XX_HAL_H_MOCK
/* host 测试用 HAL mock：仅提供 flash_store.c 用到的符号 */
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

#define FLASH_TYPEERASE_SECTORS 0U
#define FLASH_VOLTAGE_RANGE_3   2U
#define FLASH_TYPEPROGRAM_WORD  2U

typedef struct {
    uint32_t TypeErase;
    uint32_t Sector;
    uint32_t NbSectors;
    uint32_t VoltageRange;
} FLASH_EraseInitTypeDef;

void              HAL_FLASH_Unlock(void);
void              HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *er, uint32_t *err);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint32_t val);
uint32_t          HAL_GetTick(void);

#endif
