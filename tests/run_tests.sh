#!/usr/bin/env bash
# 固件 host 单元测试（无需 Keil/ARMCC）
#
# 用 gcc 编译真实的 App/flash_store.c，以 mmap 模拟 STM32 内部 Flash
# （固定映射到 0x08000000，物理语义"只能 1→0、擦除回 0xFF"），
# mock HAL_FLASH_* / HAL_GetTick / Modbus_CRC16。
#
# 用法：bash tests/run_tests.sh
set -euo pipefail

cd "$(dirname "$0")"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

cp ../App/flash_store.c ../App/flash_store.h ../App/app_def.h "$WORK/"
cp ./*.c ./*.h "$WORK/"

echo "=== flash_store 单元测试 ==="
gcc -std=gnu11 -Wall -Wno-int-to-pointer-cast \
    -o "$WORK/test_flash" "$WORK/test_flash.c"
"$WORK/test_flash"

echo
echo "=== JSON 生成单元测试 ==="
gcc -std=gnu11 -Wall -o "$WORK/test_json" "$WORK/test_json.c"
"$WORK/test_json"
