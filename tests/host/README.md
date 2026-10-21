# 主机测试（Host Tests）

在 PC 上直接编译并运行**真实固件源码**的测试，用于在烧录前验证上报状态机与 Flash 缓存逻辑。

## 与旧测试的区别

| | `tests/test_json.c`（旧） | `tests/host/`（本目录） |
|---|---|---|
| 被测对象 | 把 `dtu.c` 的组包逻辑**抄一遍** | 直接编译 `App/dtu.c`、`App/flash_store.c`、`App/modbus.c` |
| 能发现的问题 | 抄本与固件不一致时**发现不了** | ACK 解析、退避、插队、pending、身份三态、存储门控都被真实覆盖 |
| Flash 行为 | 按位与后一律成功（会掩盖缺陷） | 严格模拟 F4：同 word 二次编程**直接失败并计数** |

## 怎么跑

### 方式一：TCC（无需装 gcc，推荐）

```bash
# 一次性准备（装到任意目录）
npm install node-tinycc

# 运行
cd JY_EnvMon
NODE_PATH=<node-tinycc 所在 node_modules> node tests/host/run_host_tests.js
```

脚本会自动找到 `win64/libtcc.dll`，编译并执行，最后打印
`===== RESULT: N passed, M failed =====`，退出码 0 表示全通过。

### 方式二：gcc / clang

```bash
cd JY_EnvMon
gcc -DFS_HOST_TEST \
    -I tests/host/stub -I App -I tests/host \
    -o test.exe \
    App/dtu.c App/flash_store.c App/modbus.c \
    tests/host/flash_sim.c tests/host/test_dtu.c
./test.exe
```

> `-DFS_HOST_TEST` 必须加：它让 `flash_store.c` 的扇区地址改为由变量计算，
> 测试才能把 0x08000000 起 1MB 映射到进程内存。不加则生产行为不变（编译期常量）。

## 覆盖的用例（18 组 / 158 条断言）

| 用例 | 验证点 |
|---|---|
| `first_boot` | 空白 Flash → 身份 BLANK、`boot_id=1`、存储就绪 |
| `build_and_send` | 报文全字段（type/schema/device_id/msg_id/boot_id/seq/sampled_at/8 通道/valid/`\n` 结尾） |
| `null_channels` | 无效通道输出 `null`（0 是合法读数，不能当无效） |
| `ack_ok` | 正确 ACK 出队；确认位写入无二次编程 |
| `ack_negative` | 他机 ACK / `status!=stored` / `type!=upload_ack` / 编号错 / 缺字段 → 全部不确认 |
| `ack_framing` | 字段间带空格、拆两半跨 Tick、两个 ACK 粘包 |
| `backoff_same_msgid` | 超时 8s，退避 2/4/8/16/32s，5 次后转 30s 探测；全程同一 `msg_id` |
| `interleave` | 每 3 条历史插 1 条最新（实测序列 1,2,3,5,4） |
| `late_ack_after_jump` | 插队后 ACK 迟到仍能匹配（旧实现会换成队首记录而失败） |
| `power_cycle` | 重启后 `boot_id` 递增、未确认记录从 Flash 恢复 |
| `identity_corrupt` | 元数据有垃圾 → 拒绝自动重新编号、拒绝生成新记录 |
| `storage_not_ready` | 擦除失败 → `FS_Append`/`FS_Recycle` 拒绝、不生成记录 |
| `many_records` | 连续 10 条全流程无二次编程 |
| `review_meta_nonblank_tail` | 审查 P1-1：槽首 word 为 FF 但槽尾有残留 → 必须判 CORRUPT（槽 0 与第 100 槽各一组）；全空白仍走首次使用；坏槽应被跳过、不卡死追加 |
| `review_repair_monotonic` | 审查 P1-2：`FS_IdentityRepair(0)` / `(2)` 在当前编号 3 时必须拒绝且不改编号；`(9)` 成功得 10 并持久化 |
| `confirm_word_corrupt` | 审查 P2：确认字被写坏（0x0000FF00）仍能出队、重启后不再排队；写确认字途中掉电（半成品）也能出队；完全没写进去则保留待确认、重试后出队 |
| `full_cache_migration` | **2044 条满缓存**：填满不触发迁移；第 2045 条 ⇒ 丢 1/8(255) + 擦 2 个扇区；丢的是队首连续 255 条且保留记录内容完好；迁移后再写 254 条不再迁移；重启后 2044 条完整恢复 |
| `power_loss_in_migration` | **迁移途中掉电**（第 600 个 word 失败）：返回 `FS_ERR_FULL`、RAM 队列一条不少、丢弃计数不增加；重启回到旧扇区、2044 条完整、无二次编程；之后仍能正常迁移 |

### 故障注入能力（`flash_sim.c`）

| 接口 | 作用 |
|---|---|
| `g_sim_erase_fail` | 下一次擦除返回 `HAL_ERROR` |
| `flash_sim_fail_program_after(n)` | 再成功 n 次编程后，让下一次编程失败（**不写入**） |
| `flash_sim_partial_program_once()` | 下一次编程"写一半再失败"：把 word 写成既非全 FF 也非全 0（模拟掉电瞬间的半成品），用于复现确认字被写坏 |
| `flash_sim_poke(addr, buf, len)` | 直接改写模拟 Flash，用于构造元数据损坏/确认字损坏 |

## 判别力验证（变异测试）

- 去掉 `Dtu_ParseAck` 里的 `status=="stored"` 校验：立即报 4 条失败。
- 把 `App/flash_store.c` 换回 `1d48590` 版本（保留本目录全部新测试）：
  **17 条失败** —— P1-1 的 7 条、P1-2 的 3 条、P2 的 4 条，以及
  "迁移失败时不得丢队列/不得计丢弃/旧扇区仍可读" 3 条（旧实现先丢 1/8 再迁移）。
  换回新实现后 158/158 全绿。说明这批断言确实能区分新旧实现，不是"全绿摆设"。

## 未覆盖（不要据此认为已验证）

- **真实芯片的 Flash 编程行为**：本测试只能证明"代码不再对同一 word 二次编程"，
  不能证明具体批次芯片的编程时序/电压表现 —— 需实机验证。
- **UART 物理层**：`HAL_UART_Transmit` 被替身替换，真实 9600bps 时序未验证。
- **49.7 天 `HAL_GetTick` 回绕**：`Dtu_Tick` 用 `(int32_t)(now - last)` 比较，
  理论上可跨回绕，但没有跑过 49.7 天的等价时间压缩测试。
- **长时间擦写寿命**：满缓存路径的擦除次数已统计（每次丢 1/8 后约 255 条一迁），
  但真实擦写寿命曲线仍需实机长测。
- **看门狗、中断优先级、低功耗**：均未涉及。
