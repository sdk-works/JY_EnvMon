#ifndef __MAIN_H_MOCK
#define __MAIN_H_MOCK
/* host 测试用：仅提供 flash_store 依赖的类型声明 */
#include <stdint.h>
typedef struct { int dummy; } UART_HandleTypeDef;
typedef struct { int dummy; } IWDG_HandleTypeDef;
#endif
