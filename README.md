# JY_EnvMon 京阳温室环境监测系统

基于 STM32F407VET6 的温室环境监测终端，采集百叶箱与土壤传感器数据，通过 LED 大屏轮播显示，并预留 DTU 上传云平台接口。

## 硬件

| 项目 | 型号/参数 |
|---|---|
| 主控 | STM32F407VET6 @ 168MHz（嘉立创 LK1C3R PLC 板） |
| 晶振 | 8MHz HSE |
| 百叶箱 | RS-BYH-CO2-M，Modbus 地址 08，9600bps |
| 土壤三合一 | RS-ECTH-N01，Modbus 地址 0x12(18)，9600bps |
| LED 屏 | 仰邦 BX-5K1，64×16 单色，屏号 001，9600bps |

## 串口分配

| 串口 | 方向脚 | 连接设备 |
|---|---|---|
| USART1 (PA9/PA10) | — | RS232 → DTU（待对接云平台） |
| USART3 (PC10/PC11) | PD3 = T2_EN | 485-1 → 百叶箱 + 土壤传感器（共线） |
| UART5 (PC12/PD2) | PD4 = T3_EN | 485-2 → LED 大屏 |
| SWD | PA13/PA14 | 烧录调试 |

## 采集数据

| 寄存器 | 含义 | 倍率 |
|---|---|---|
| 百叶箱 0x01F4 | 空气湿度 | ÷10 (%RH) |
| 百叶箱 0x01F5 | 空气温度 | ÷10 (℃) |
| 百叶箱 0x01F7 | CO₂ 浓度 | ppm |
| 百叶箱 0x01F9 | 大气压 | ÷10 (kPa) |
| 百叶箱 0x01FA~FB | 光照强度 | Lux（32 位） |
| 土壤 0x0000 | 土壤含水率 | % |
| 土壤 0x0001 | 土壤温度 | ÷10 (℃) |
| 土壤 0x0002 | 电导率 | us/cm |

## LED 轮播

8 条循环滚动，速度 09（左移），停留时间按文本长短：

- 温度:xx.x℃ → 湿度:xx.x% → CO2:xxxppm → 光照:xxxxLux
- → 气压:xxx.xkPa → 土温:xx.x℃ → 土湿:xx% → EC:xxxxus/cm

## 工程结构

```
App/
  app.c        主调度（采集周期 3s，屏切换按各条独立计时）
  sensor.c     Modbus 采集百叶箱 + 土壤
  screen.c     BX-5K1 字库卡协议组包发送
  modbus.c     Modbus RTU CRC16 + 读保持寄存器
  bus485.c     485 方向脚控制 + 中断接收 + 帧间隔判断
  dtu.c        DTU/云平台上传（已实现，遵循 COMMS_CONTRACT.md v1.2）
Core/          CubeMX 生成
Drivers/       HAL + CMSIS
MDK-ARM/       Keil 工程
```

## 编译

Keil MDK-ARM，工具链 ARMCC V5.06，目标芯片 STM32F407VET6。

## 版本与修复

### v1.1.0-fix-p0（提交 16e7c07）

修复评估发现的三个 P0 问题（详见 `docs/COMMS_CONTRACT.md` 与 `tests/`）：

1. **Flash 数据覆盖（最严重）**：原实现每次追加都把「已用条数」递增写回 Flash 的 `hdr.used`。
   Flash 只能 1→0，`0x01 & 0x02 = 0x00` 导致计数归零，后续记录反复覆盖 slot0（实测 5 条只剩 1 条）。
   修复：RAM 维护 `s_used`，上电 `FS_ScanUsed()` 扫描空槽；`hdr.used` 仅迁移时写一次。
2. **光照溢出**：`g_env.lux` 由 `float` 改为 `uint32_t`，新增 `FS_FIXED_UINT1`；dtu/screen 按无符号处理。
3. **无效值误报**：`Dtu_BuildTelemetry` 按 valid 位图逐字段输出，无效通道输出 `null`（合法 0 仍输出 0）。

附带：迁移逐条回读校验 + 总数核对 + 失败不擦旧扇区；新增 `FS_PeekLatest` 实现 §8.2 插队；
`Dtu_Init` 改用 `HAL_GetTick()` 作基准（避免上电立即写一条）。

### 单元测试

`tests/run_tests.sh` 用 gcc + mmap 把模拟 Flash 固定映射到 `0x08000000`，编译**真实** `App/flash_store.c`，
共 21 项断言（含修复前后对照）。本机无 Keil/gcc 时可在服务器上运行。
