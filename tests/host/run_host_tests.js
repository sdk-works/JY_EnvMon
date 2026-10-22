/**
 * tests/host/run_host_tests.js
 *
 * 用 TCC（node-tinycc 内嵌的 libtcc.dll）在主机上编译并运行主机测试。
 * 被测对象是【真实固件源码】：App/dtu.c、App/flash_store.c、App/modbus.c，
 * 通过 tests/host/stub 提供 HAL 替身，不做任何逻辑复制。
 *
 * 用法：
 *   NODE_PATH=<含 node-tinycc 的 node_modules> node tests/host/run_host_tests.js
 * 或设置 TCC_MODULE_DIR 指向 node-tinycc 所在目录。
 *
 * 若本机有 gcc/clang，也可直接：
 *   gcc -I tests/host/stub -I App -o test.exe App/dtu.c App/flash_store.c \
 *       App/modbus.c tests/host/flash_sim.c tests/host/test_dtu.c
 */
'use strict';

const path = require('path');
const fs = require('fs');

const ROOT = path.resolve(__dirname, '..', '..');
const CANDIDATES = [
  process.env.TCC_MODULE_DIR,
  path.join(__dirname, 'node_modules'),
  'C:/Users/salut/WorkBuddy/2026-10-17-11-45-13/.workbuddy/tmp/hostc/node_modules',
].filter(Boolean);

let tcc = null, dll = null;
for (const dir of CANDIDATES) {
  const modPath = path.join(dir, 'node-tinycc');
  const dllPath = path.join(modPath, 'win64', 'libtcc.dll');
  if (fs.existsSync(modPath) && fs.existsSync(dllPath)) {
    tcc = require(modPath);
    dll = dllPath;
    break;
  }
}
if (!tcc) {
  console.error('找不到 node-tinycc（或 win64/libtcc.dll）。');
  console.error('请设置 TCC_MODULE_DIR，或改用 gcc/clang 直接编译（见文件头注释）。');
  process.exit(2);
}

const STUB = path.join(ROOT, 'tests', 'host', 'stub');
const APP  = path.join(ROOT, 'App');
const HOST = path.join(ROOT, 'tests', 'host');

const SOURCES = [
  path.join(APP,  'dtu.c'),
  path.join(APP,  'flash_store.c'),
  path.join(APP,  'modbus.c'),
  path.join(HOST, 'flash_sim.c'),
  path.join(HOST, 'test_dtu.c'),
];

for (const f of SOURCES) {
  if (!fs.existsSync(f)) { console.error('缺少源文件: ' + f); process.exit(2); }
}

const t = new tcc.Tcc(dll);
// stub 必须排在 App 前面，才能覆盖 main.h / stm32f4xx_hal.h
// FS_HOST_TEST 让 flash_store.c 的扇区地址改由变量计算（测试需把 Flash 映射进进程）
// JY_HOST_TEST 让 dtu.h 保持 10s/2s 快速节拍（固件默认已是 60s/5s 生产节拍）
t.setOptions(`-DFS_HOST_TEST -DJY_HOST_TEST -I"${STUB}" -I"${APP}" -I"${HOST}"`);

for (const f of SOURCES) {
  try {
    t.addFile(f);
  } catch (e) {
    console.error('编译失败: ' + f + '\n' + e.message);
    process.exit(2);
  }
}

process.chdir(ROOT);          // 测试内部用相对路径写 last_result.txt
const rc = t.run(0, null);

const outFile = path.join(HOST, 'last_result.txt');
if (fs.existsSync(outFile)) {
  process.stdout.write(fs.readFileSync(outFile, 'utf8'));
}

console.log(rc === 0 ? '\n>>> HOST TESTS PASSED <<<' : '\n>>> HOST TESTS FAILED <<<');
process.exit(rc === 0 ? 0 : 1);
