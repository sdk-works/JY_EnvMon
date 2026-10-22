/**
  ******************************************************************************
  * @file    test_dtu.c
  * @brief   主机测试：直接编译真实的 App/dtu.c + App/flash_store.c + App/modbus.c
  *
  * 【与旧测试的区别】
  *   tests/test_json.c 是"把 dtu.c 的组包逻辑抄一遍"，测的是抄本，不是固件。
  *   本测试通过 tests/host/stub 提供 HAL 替身，直接编译真实源文件，因此
  *   ACK 解析、退避、插队、pending 精确重发、身份三态、数据区门控都被真实覆盖。
  *
  * 【Flash 模拟严格性】见 tests/host/flash_sim.c：同 word 二次编程直接失败并计数。
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"     /* stub：Flash/UART/Tick 替身 */
#include "dtu.h"
#include "flash_store.h"

/* ===== 模拟后端接口 ===== */
extern void     flash_sim_reset(void);
extern uint32_t flash_sim_violations(void);
extern uint32_t flash_sim_reprogram(void);
extern uint32_t flash_sim_programmed(void);
extern uint32_t flash_sim_erases(void);
extern void     flash_sim_poke(uint32_t addr, const uint8_t *data, uint32_t len);
extern uint32_t flash_sim_nonblank(uint32_t addr, uint32_t len);
extern int      g_sim_erase_fail;
extern void     flash_sim_fail_program_after(uint32_t n);
extern void     flash_sim_partial_program_once(void);

#define SECTOR4_ADDR  0x08010000u
#define SECTOR5_ADDR  0x08020000u
#define SECTOR6_ADDR  0x08040000u
#define SECTOR7_ADDR  0x08060000u

/* ===== 断言框架 ===== */
static int  g_pass = 0, g_fail = 0;
static FILE *g_out = NULL;
static const char *g_case = "";

static void ck(int cond, const char *msg)
{
    if (cond) {
        g_pass++;
    } else {
        g_fail++;
        fprintf(g_out, "  FAIL [%s] %s  (line %d)\n", g_case, msg, __LINE__);
    }
}

static void begin(const char *name)
{
    g_case = name;
    fprintf(g_out, "[CASE] %s\n", name);
}

/* ===== 时间/输入辅助 ===== */
static void adv(uint32_t ms) { g_sim_tick += ms; }

static void feed(const char *s)
{
    while (*s) Dtu_OnRxByte((uint8_t)(*s++));
}

static void feed_ack(const char *msg_id, const char *status)
{
    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"upload_ack\",\"msg_id\":\"%s\",\"status\":\"%s\"}",
             msg_id, status);
    feed(buf);
}

static void extract_msgid(const char *json, char *out, size_t cap)
{
    const char *p = strstr(json, "\"msg_id\":\"");
    size_t n;
    out[0] = '\0';
    if (!p) return;
    p += 10;
    {
        const char *e = strchr(p, '"');
        n = e ? (size_t)(e - p) : 0;
    }
    if (n >= cap) n = cap - 1;
    memcpy(out, p, n);
    out[n] = '\0';
}

/* 生成一条记录并等到它被发出（返回 1 表示确实发出了新帧） */
static int gen_and_send(void)
{
    uint32_t before = g_sim_tx_count;
    adv(DTU_NEW_RECORD_MS);
    Dtu_Tick();
    if (g_sim_tx_count == before) {      /* 发送槽未到，再等一个槽 */
        adv(DTU_SEND_SLOT_MS);
        Dtu_Tick();
    }
    return (g_sim_tx_count > before);
}

/* 推进一个发送槽，返回是否发出了新帧 */
static int send_slot(void)
{
    uint32_t before = g_sim_tx_count;
    adv(DTU_SEND_SLOT_MS);
    Dtu_Tick();
    return (g_sim_tx_count > before);
}

/* ============================================================
 * 用例
 * ============================================================ */

/* 1. 首次上电：身份 BLANK、boot_id=1、存储就绪 */
static void case_first_boot(void)
{
    begin("first_boot");
    flash_sim_reset();
    Dtu_Init();

    ck(FS_IdentityState() == FS_IDENT_BLANK, "首次上电应为 BLANK");
    ck(Dtu_LastBootId() == 1, "首次 boot_id 应为 1（不是注释里矛盾的 2）");
    ck(Dtu_IdentityOk() == 1, "身份应就绪");
    ck(Dtu_StorageOk() == 1, "数据区应就绪");
    ck(Dtu_PendingCount() == 0, "初始待发应为 0");
    ck(flash_sim_violations() == 0, "初始化不应有 Flash 编程违规");
}

/* 2. 生成并发送一条：报文关键字段正确 */
static void case_build_and_send(void)
{
    begin("build_and_send");
    flash_sim_reset();
    Dtu_Init();
    g_env.ble_box_valid = 1; g_env.soil_valid = 1;
    g_env.air_temp = 25.3f; g_env.air_humi = 61.2f; g_env.co2 = 830.0f;
    g_env.lux = 12345u; g_env.pressure = 1013.2f;
    g_env.soil_temp = 18.7f; g_env.soil_humi = 42.5f; g_env.soil_ec = 512.0f;

    adv(DTU_NEW_RECORD_MS);
    Dtu_Tick();

    ck(Dtu_PendingCount() == 1, "10s 后应生成 1 条记录");
    ck(g_sim_tx_count == 1, "应已发出 1 帧");
    ck(strstr(g_sim_tx, "\"type\":\"telemetry\"") != NULL, "type=telemetry");
    ck(strstr(g_sim_tx, "\"schema_version\":1") != NULL, "schema_version=1");
    ck(strstr(g_sim_tx, "\"device_id\":\"JY001\"") != NULL, "device_id=JY001");
    ck(strstr(g_sim_tx, "\"msg_id\":\"JY001/1/1\"") != NULL, "msg_id=JY001/1/1");
    ck(strstr(g_sim_tx, "\"boot_id\":1") != NULL, "boot_id=1");
    ck(strstr(g_sim_tx, "\"seq\":1") != NULL, "seq=1");
    ck(strstr(g_sim_tx, "\"sampled_at\":0") != NULL, "sampled_at=0");
    ck(strstr(g_sim_tx, "\"temperature\":25.3") != NULL, "温度定点还原");
    ck(strstr(g_sim_tx, "\"light\":12345") != NULL, "光照 uint32 直通");
    ck(strstr(g_sim_tx, "\"valid\":[1,1,1,1,1,1,1,1]") != NULL, "valid 位图");
    ck(g_sim_tx[g_sim_tx_len - 1] == '\n', "报文以 \\n 结尾");
}

/* 3. 无效通道输出 null（0 是合法读数，不能当无效） */
static void case_null_channels(void)
{
    begin("null_channels");
    flash_sim_reset();
    Dtu_Init();
    g_env.ble_box_valid = 0; g_env.soil_valid = 0;
    memset(&g_env.air_temp, 0, sizeof(g_env.air_temp));

    adv(DTU_NEW_RECORD_MS);
    Dtu_Tick();

    {
        const char *p = g_sim_tx;
        int n = 0;
        while ((p = strstr(p, "null")) != NULL) { n++; p += 4; }
        ck(n == 8, "8 个无效通道都应输出 null");
    }
    ck(strstr(g_sim_tx, "\"valid\":[0,0,0,0,0,0,0,0]") != NULL, "valid 全 0");
}

/* 4. ACK 正例：正确 ACK 出队，且无二次编程 */
static void case_ack_ok(void)
{
    begin("ack_ok");
    flash_sim_reset();
    Dtu_Init();
    ck(gen_and_send(), "应发出 1 帧");

    feed_ack("JY001/1/1", "stored");
    Dtu_Tick();

    ck(Dtu_PendingCount() == 0, "正确 ACK 后应出队");
    ck(flash_sim_reprogram() == 0, "确认位写入不应出现同 word 二次编程");
}

/* 5. ACK 负例矩阵：他机/状态错/类型错/编号错，都不能确认 */
static void case_ack_negative(void)
{
    begin("ack_negative");
    flash_sim_reset();
    Dtu_Init();
    ck(gen_and_send(), "应发出 1 帧");

    feed_ack("JY002/1/1", "stored");        /* 别的设备 */
    Dtu_Tick();
    ck(Dtu_PendingCount() == 1, "他机 ACK 不应确认");

    feed_ack("JY001/1/1", "duplicate");     /* 状态非 stored */
    Dtu_Tick();
    ck(Dtu_PendingCount() == 1, "status!=stored 不应确认");

    feed("{\"type\":\"telemetry\",\"msg_id\":\"JY001/1/1\",\"status\":\"stored\"}");
    Dtu_Tick();
    ck(Dtu_PendingCount() == 1, "type!=upload_ack 不应确认");

    feed_ack("JY001/1/9", "stored");        /* 编号不对 */
    Dtu_Tick();
    ck(Dtu_PendingCount() == 1, "msg_id 不匹配不应确认");

    feed("{\"type\":\"upload_ack\",\"msg_id\":\"JY001/1/1\"}");   /* 缺 status */
    Dtu_Tick();
    ck(Dtu_PendingCount() == 1, "缺 status 字段不应确认");

    feed_ack("JY001/1/1", "stored");
    Dtu_Tick();
    ck(Dtu_PendingCount() == 0, "最终正确 ACK 应确认");
}

/* 6. ACK 格式兼容：字段间空白、跨两次 Tick、两个对象粘在一起 */
static void case_ack_framing(void)
{
    begin("ack_framing");
    flash_sim_reset();
    Dtu_Init();
    ck(gen_and_send(), "应发出 1 帧");

    /* 6a 字段间带空格 */
    feed("{ \"type\" : \"upload_ack\" , \"msg_id\" : \"JY001/1/1\" , \"status\" : \"stored\" }");
    Dtu_Tick();
    ck(Dtu_PendingCount() == 0, "带空格的 ACK 应能解析");

    /* 6b 拆成两半，中间跨一次 Tick（模拟 DTU 分片到达） */
    ck(gen_and_send(), "应发出第 2 帧");
    feed("{\"type\":\"upload_ack\",\"msg_id\":\"JY001/1/2\"");
    Dtu_Tick();                                  /* 未闭合，不应确认 */
    ck(Dtu_PendingCount() == 1, "半包不应被当作完整对象");
    feed(",\"status\":\"stored\"}");
    Dtu_Tick();
    ck(Dtu_PendingCount() == 0, "补齐后应能确认");

    /* 6c 两个 ACK 粘在一起（DTU 下行无换行） */
    ck(gen_and_send(), "应发出第 3 帧");
    feed("{\"type\":\"upload_ack\",\"msg_id\":\"JY001/1/3\",\"status\":\"stored\"}"
         "{\"type\":\"upload_ack\",\"msg_id\":\"JY001/1/3\",\"status\":\"stored\"}");
    Dtu_Tick();
    ck(Dtu_PendingCount() == 0, "粘包 ACK 应能确认");
    ck(flash_sim_reprogram() == 0, "重复 ACK 不应触发二次编程");
}

/* 7. 断网退避：2s→4s→8s→16s→32s，每次重发都是同一条，5 次后转 30s 探测 */
static void case_backoff_same_msgid(void)
{
    char first[48], again[48];
    uint32_t c0, c1;
    int i;

    begin("backoff_same_msgid");
    flash_sim_reset();
    Dtu_Init();
    ck(gen_and_send(), "应发出 1 帧");
    extract_msgid(g_sim_tx, first, sizeof(first));
    c0 = g_sim_tx_count;

    /* 差 1ms 未到 8s 超时 */
    adv(8000 - 1); Dtu_Tick();
    ck(g_sim_tx_count == c0, "未到 8s 不应有动作");

    /* 到 8s：进入退避，第 1 次退避 2s */
    adv(1); Dtu_Tick();
    adv(1999); Dtu_Tick();
    ck(g_sim_tx_count == c0, "退避 2s 内不应重发");
    adv(1); Dtu_Tick();
    ck(g_sim_tx_count == c0 + 1, "退避 2s 到点应重发");
    extract_msgid(g_sim_tx, again, sizeof(again));
    ck(strcmp(first, again) == 0, "第 1 次重发必须是同一 msg_id");

    /* 再超时：第 2 次退避 4s */
    adv(8000); Dtu_Tick();
    adv(3999); Dtu_Tick();
    ck(g_sim_tx_count == c0 + 1, "第二次退避为 4s，3.999s 不应重发");
    adv(1); Dtu_Tick();
    ck(g_sim_tx_count == c0 + 2, "退避 4s 到点应重发");
    extract_msgid(g_sim_tx, again, sizeof(again));
    ck(strcmp(first, again) == 0, "第 2 次重发仍是同一 msg_id");

    /* 第 3/4/5 次超时：退避 8s / 16s / 32s */
    for (i = 0; i < 3; i++) {
        adv(8000); Dtu_Tick();
        adv(2000u << (i + 2)); Dtu_Tick();
    }

    /* retry>=5 后转周期探测：先立即探测一次 */
    c1 = g_sim_tx_count;
    Dtu_Tick();
    ck(g_sim_tx_count == c1 + 1, "转 PROBE 后应探测一次");
    extract_msgid(g_sim_tx, again, sizeof(again));
    ck(strcmp(first, again) == 0, "探测仍是同一 msg_id（未换记录）");

    /* 探测节奏应为 30s，而不是每个发送槽都发 */
    adv(5000); Dtu_Tick();
    ck(g_sim_tx_count == c1 + 1, "PROBE 期间不应 5s 就发");
    adv(25000); Dtu_Tick();
    ck(g_sim_tx_count == c1 + 2, "PROBE 应约每 30s 探测一次");
}

/* 8. 插队：每 3 条历史插 1 条最新（真实 dtu.c 的调度逻辑） */
static void case_interleave(void)
{
    char seqs[6][40];
    int  ns = 0, i;

    begin("interleave");
    flash_sim_reset();
    Dtu_Init();

    /* 先让发送全部失败，把 5 条记录堆在队列里 */
    g_sim_uart_fail = 1;
    for (i = 0; i < 5; i++) { adv(DTU_NEW_RECORD_MS); Dtu_Tick(); }
    ck(Dtu_PendingCount() == 5, "应积压 5 条");
    g_sim_uart_fail = 0;

    /* 恢复链路：重发队首（seq1）并确认 */
    adv(DTU_SEND_SLOT_MS);
    Dtu_Tick();
    extract_msgid(g_sim_tx, seqs[ns], sizeof(seqs[0]));
    ck(strcmp(seqs[ns], "JY001/1/1") == 0, "恢复后应先重发 seq1");
    feed_ack(seqs[ns], "stored");
    Dtu_Tick();
    ns++;
    ck(Dtu_PendingCount() == 4, "seq1 确认后剩 4 条");

    /* 之后每次发送都立即确认，观察调度顺序 */
    while (ns < 5 && Dtu_PendingCount() > 0) {
        adv(DTU_SEND_SLOT_MS);
        Dtu_Tick();
        extract_msgid(g_sim_tx, seqs[ns], sizeof(seqs[0]));
        feed_ack(seqs[ns], "stored");
        Dtu_Tick();
        ns++;
    }

    /* 期望顺序：1,2,3,5,4（第 4 次插队发最新的 seq5，再回到 seq4） */
    ck(ns >= 5, "应至少发送 5 次");
    if (ns >= 5) {
        ck(strcmp(seqs[1], "JY001/1/2") == 0, "第 2 帧应为 seq2");
        ck(strcmp(seqs[2], "JY001/1/3") == 0, "第 3 帧应为 seq3");
        ck(strcmp(seqs[3], "JY001/1/5") == 0, "第 4 帧应插队发最新 seq5");
        ck(strcmp(seqs[4], "JY001/1/4") == 0, "第 5 帧回到 seq4");
    }
}

/* 9. 插队后 ACK 迟到：pending 必须是发出过的那条（P1-3 的核心回归）
   旧实现：退避重发读队首 → 把 pending 换成 seq4 → 迟到的 seq5 ACK 永远匹配不上。 */
static void case_late_ack_after_jump(void)
{
    char id[48], re[48];
    FS_Record_t rr;
    uint32_t before;
    int  i;

    begin("late_ack_after_jump");
    flash_sim_reset();
    Dtu_Init();

    g_sim_uart_fail = 1;
    for (i = 0; i < 5; i++) { adv(DTU_NEW_RECORD_MS); Dtu_Tick(); }
    g_sim_uart_fail = 0;
    ck(Dtu_PendingCount() == 5, "应积压 5 条");

    /* 恢复链路：重发 pending（seq1）并确认 */
    adv(DTU_SEND_SLOT_MS); Dtu_Tick();
    extract_msgid(g_sim_tx, id, sizeof(id));
    ck(strcmp(id, "JY001/1/1") == 0, "恢复后应先重发 seq1");
    feed_ack("JY001/1/1", "stored"); Dtu_Tick();
    ck(FS_ReadById(1, 1, &rr) == false, "seq1 应已确认出队");

    /* 再确认 seq2、seq3（此后 s_hist_sent 达到阈值，下一帧应插队） */
    for (i = 0; i < 2; i++) {
        adv(DTU_SEND_SLOT_MS); Dtu_Tick();
        extract_msgid(g_sim_tx, id, sizeof(id));
        feed_ack(id, "stored"); Dtu_Tick();
    }

    /* 插队那条：应为 seq5，故意不回 ACK */
    adv(DTU_SEND_SLOT_MS); Dtu_Tick();
    extract_msgid(g_sim_tx, id, sizeof(id));
    ck(strcmp(id, "JY001/1/5") == 0, "应插队发送最新的 seq5");

    /* 超时 → 退避 → 重发必须仍是 seq5 */
    adv(8000); Dtu_Tick();
    adv(2000); Dtu_Tick();
    extract_msgid(g_sim_tx, re, sizeof(re));
    ck(strcmp(re, "JY001/1/5") == 0, "退避重发必须仍是 seq5（不是队首 seq4）");

    /* 迟到的 ACK 必须能对上 */
    before = Dtu_PendingCount();
    feed_ack("JY001/1/5", "stored"); Dtu_Tick();
    ck(FS_ReadById(1, 5, &rr) == false, "迟到 ACK 应把 seq5 确认出队");
    ck(Dtu_PendingCount() <= before, "确认后待发数不应增加");
}

/* 10. 断电重启：boot_id 递增、未确认记录恢复、seq 从 1 重新开始 */
static void case_power_cycle(void)
{
    int i;
    begin("power_cycle");
    flash_sim_reset();
    Dtu_Init();

    g_sim_uart_fail = 1;             /* 让记录留在待发状态 */
    for (i = 0; i < 3; i++) { adv(DTU_NEW_RECORD_MS); Dtu_Tick(); }
    g_sim_uart_fail = 0;
    ck(Dtu_PendingCount() == 3, "重启前应有 3 条未确认");

    /* 模拟断电重启：保留 Flash 内容，重新初始化全部 RAM 状态 */
    Dtu_Init();

    ck(Dtu_LastBootId() == 2, "重启后 boot_id 应递增到 2");
    ck(FS_IdentityState() == FS_IDENT_OK, "重启后身份应有效");
    ck(Dtu_StorageOk() == 1, "重启后数据区应就绪");
    ck(Dtu_PendingCount() == 3, "未确认记录应从 Flash 恢复");
    ck(flash_sim_reprogram() == 0, "重启过程不应二次编程");
}

/* 11. 身份损坏：禁止自动重新编号、禁止新建记录 */
static void case_identity_corrupt(void)
{
    uint8_t junk[32];

    begin("identity_corrupt");
    flash_sim_reset();
    memset(junk, 0xAA, sizeof(junk));           /* 非 FF 且 CRC 无效 */
    flash_sim_poke(SECTOR4_ADDR, junk, sizeof(junk));

    Dtu_Init();

    ck(FS_IdentityState() == FS_IDENT_CORRUPT, "有垃圾无有效记录应为 CORRUPT");
    ck(FS_NextBootId() == 0, "损坏时禁止自动编号");
    ck(Dtu_LastBootId() == 0, "损坏时 boot_id 应为 0");
    ck(Dtu_IdentityOk() == 0, "损坏时身份不就绪");

    adv(3 * DTU_NEW_RECORD_MS);
    Dtu_Tick();
    ck(Dtu_PendingCount() == 0, "身份损坏时不得生成新记录");
    ck(g_sim_tx_count == 0, "身份损坏时不得上传新数据");
}

/* 12. 数据区初始化失败：禁止追加 */
static void case_storage_not_ready(void)
{
    FS_Record_t rec;

    begin("storage_not_ready");
    flash_sim_reset();
    g_sim_erase_fail = 1;               /* 让首次擦除失败 */
    Dtu_Init();

    ck(Dtu_StorageOk() == 0, "擦除失败应标记未就绪");

    memset(&rec, 0, sizeof(rec));
    rec.boot_id = 9; rec.seq = 1;
    ck(FS_Append(&rec) == FS_ERR_NOTREADY, "未就绪时 FS_Append 必须拒绝");
    ck(FS_Recycle() == FS_ERR_NOTREADY, "未就绪时 FS_Recycle 必须拒绝");

    adv(3 * DTU_NEW_RECORD_MS);
    Dtu_Tick();
    ck(Dtu_PendingCount() == 0, "未就绪时不得生成新记录");
}

/* 13. 连续 10 条：全流程无 Flash 编程违规（覆盖重复写入/确认） */
static void case_many_records(void)
{
    char id[48];
    int  i;

    begin("many_records");
    flash_sim_reset();
    Dtu_Init();

    for (i = 0; i < 10; i++) {
        adv(DTU_NEW_RECORD_MS);
        Dtu_Tick();
        if (g_sim_tx_count > 0) {
            extract_msgid(g_sim_tx, id, sizeof(id));
            feed_ack(id, "stored");
            Dtu_Tick();
        }
    }
    ck(flash_sim_reprogram() == 0, "10 条全流程无同 word 二次编程");
    ck(Dtu_PendingCount() == 0, "全部应确认出队");
    ck(Dtu_LastBootId() == 1, "同一启动内 boot_id 不变");
}

/* ============================================================
 * 第三方审查 1d48590 三项发现 + 满缓存/掉电回归
 * ============================================================ */

/* 直接向缓存追加一条记录（绕过 Dtu 采样节律，用于存储层压测） */
static int append_seq(uint32_t boot, uint32_t seq, uint32_t temp)
{
    FS_Record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.boot_id     = boot;
    rec.seq         = seq;
    rec.sampled_at  = 1700000000u + seq;
    rec.uptime_ms   = (uint64_t)seq * 10000u;
    rec.time_quality= 1;
    rec.valid       = 0x01;
    rec.data[FS_CH_TEMPERATURE] = temp;
    return (FS_Append(&rec) == FS_OK);
}

/* 14. 元数据"空白"必须整条判定（审查 P1-1）
   构造"槽首 word 为 FF、槽尾有残留"的元数据：旧实现只查首 word，会把它
   当成首次使用并从 1 重新编号 → 与云端历史 msg_id 撞号。 */
static void case_review_meta_nonblank_tail(void)
{
    uint8_t dirty[4] = {0, 0, 0, 0};

    begin("review_meta_nonblank_tail");
    flash_sim_reset();
    flash_sim_poke(SECTOR4_ADDR + 4, dirty, 4);      /* 槽 0 首 word 仍全 FF */
    Dtu_Init();

    ck(FS_IdentityState() == FS_IDENT_CORRUPT, "槽首 FF、槽尾有残留不得判为空白");
    ck(FS_NextBootId() == 0, "身份损坏时禁止自动编号");
    ck(Dtu_LastBootId() == 0, "身份损坏时 boot_id 应为 0");
    ck(Dtu_IdentityOk() == 0, "身份损坏时身份不就绪");
    adv(3 * DTU_NEW_RECORD_MS);
    Dtu_Tick();
    ck(Dtu_PendingCount() == 0, "身份损坏时不得生成新记录");

    /* 远端槽（第 100 条记录位置）残留，同样必须判为损坏 */
    flash_sim_reset();
    flash_sim_poke(SECTOR7_ADDR + 100 * 32 + 12, dirty, 4);
    Dtu_Init();
    ck(FS_IdentityState() == FS_IDENT_CORRUPT, "远端槽残留同样必须判为损坏");
    ck(Dtu_LastBootId() == 0, "远端槽残留时同样禁止自动编号");

    /* 反向：真正全空白的扇区仍应走首次使用（状态 BLANK，编号从 1 开始） */
    flash_sim_reset();
    Dtu_Init();
    ck(FS_IdentityState() == FS_IDENT_BLANK, "全空白应判为首次使用（BLANK）");
    ck(Dtu_LastBootId() == 1, "首次使用编号为 1");

    /* 已有有效编号 + 一个"槽尾脏"的坏槽：必须跳过坏槽继续追加，不能卡死 */
    flash_sim_poke(SECTOR4_ADDR + 32 + 8, dirty, 4);   /* 槽 1 的 CRC 位置被写脏 */
    ck(FS_NextBootId() == 2, "遇到槽尾脏的坏槽应跳过并继续追加编号");
    ck(FS_IdentityState() != FS_IDENT_CORRUPT, "跳槽追加后身份不得为损坏态");
    FS_Init();
    ck(FS_GetBootId() == 2, "跳槽写入的编号应可持久恢复");
    ck(flash_sim_reprogram() == 0, "身份判定过程不得二次编程");
}

/* 15. 身份修复不允许编号倒退（审查 P1-2） */
static void case_review_repair_monotonic(void)
{
    begin("review_repair_monotonic");
    flash_sim_reset();
    Dtu_Init();                       /* boot 1 */
    ck(FS_NextBootId() == 2, "setup: boot 2");
    ck(FS_NextBootId() == 3, "setup: boot 3");
    ck(FS_GetBootId() == 3, "setup: 当前编号 3");

    ck(!FS_IdentityRepair(0), "必须拒绝低于已知编号的恢复下限");
    ck(!FS_IdentityRepair(2), "必须拒绝任何倒退请求");
    ck(FS_GetBootId() == 3, "拒绝后编号必须保持 3");
    ck(FS_IdentityState() == FS_IDENT_BLANK, "拒绝后身份状态不变（仍为首次使用态）");

    ck(FS_IdentityRepair(9), "高于已知编号的恢复应成功");
    ck(FS_GetBootId() == 10, "新编号 = 下限 + 1");
    ck(FS_IdentityState() == FS_IDENT_OK, "修复后身份有效");

    FS_Init();
    ck(FS_GetBootId() == 10, "恢复结果必须持久化");
    Dtu_Init();
    ck(Dtu_LastBootId() == 11, "重启后应在此基础上递增");
    ck(flash_sim_reprogram() == 0, "修复过程不得二次编程");
}

/* 16. 确认字状态不确定时：保留补传资格 + 换槽恢复可写（审查 P2 修正版）
   规则：FF=待确认、00=已确认、其它=不确定（仍需补传）。
   不确定值绝不能当成"已确认"—— 云端按 msg_id 幂等去重，重复可处理；漏传不可补救。
   恢复路径：把记录复制到新槽（确认字为擦除态），再写确认字、出队。 */
static void case_confirm_word_corrupt(void)
{
    FS_Record_t rr;
    uint8_t bad[4] = {0x00, 0xFF, 0x00, 0x00};        /* 0x0000FF00：半成品 */
    uint32_t addr  = SECTOR5_ADDR + 256;              /* 首条记录 */

    begin("confirm_word_corrupt");
    flash_sim_reset();
    Dtu_Init();
    ck(append_seq(1, 1, 123), "前置：写入 1 条待发记录");

    /* A：确认字是不确定值 → 必须仍算待发（不得当成已确认丢掉） */
    flash_sim_poke(addr + 60, bad, 4);
    FS_Init();
    ck(FS_PendingCount() == 1, "确认字不确定必须仍保留补传资格");

    /* A2：ACK 到达 → 换槽恢复可写并确认出队 */
    FS_MarkConfirmed(1, 1);
    ck(FS_PendingCount() == 0, "换槽后写确认字应能出队");
    ck(flash_sim_reprogram() == 0, "不得对不确定的确认字二次编程");

    /* A3：重启后既不能复活，也不能重复补传 */
    FS_Init();
    ck(FS_PendingCount() == 0, "重启后不得复活已确认记录");
    ck(!FS_Peek(&rr, true), "队列应为空");

    /* A4：换槽确实多占了一个槽（迁移/去重逻辑的前提） */
    ck(flash_sim_nonblank(SECTOR5_ADDR + 256 + 64, 8) > 0, "新槽应已写入副本");

    /* B：写确认字途中掉电（HAL 只写了一半）→ 不确定 → 仍待发 */
    flash_sim_reset();
    Dtu_Init();
    ck(append_seq(1, 1, 55), "前置：再写 1 条");
    flash_sim_partial_program_once();
    FS_MarkConfirmed(1, 1);
    ck(FS_PendingCount() == 1, "写确认字途中掉电应保留补传资格");
    FS_MarkConfirmed(1, 1);                            /* ACK 重发后再来一次 */
    ck(FS_PendingCount() == 0, "再收到 ACK 后应换槽确认成功");
    ck(flash_sim_reprogram() == 0, "全程不得二次编程");

    /* C：完全没写进去（确认字仍是 FF）→ 待确认，重试即可 */
    flash_sim_reset();
    Dtu_Init();
    ck(append_seq(1, 1, 66), "前置：再写 1 条");
    flash_sim_fail_program_after(0);
    FS_MarkConfirmed(1, 1);
    ck(FS_PendingCount() == 1, "没写进去时应保留待确认");
    FS_MarkConfirmed(1, 1);
    ck(FS_PendingCount() == 0, "重试 ACK 应能正常确认出队");

    /* D：不确定状态也能靠整扇区迁移修好（迁移写出的是干净确认字） */
    flash_sim_reset();
    Dtu_Init();
    ck(append_seq(1, 1, 77), "前置：再写 1 条");
    flash_sim_poke(SECTOR5_ADDR + 256 + 60, bad, 4);
    ck(FS_Recycle() == FS_OK, "迁移应成功");
    FS_MarkConfirmed(1, 1);
    ck(FS_PendingCount() == 0, "迁移到新扇区后确认字可写，ACK 应能出队");
    ck(flash_sim_reprogram() == 0, "迁移路径不得二次编程");
}

/* 17. 满缓存 2044 条 + 批量丢弃 + 整扇区迁移 + 重启恢复 */
static void case_full_cache_migration(void)
{
    FS_Record_t rr;
    uint32_t before;
    int i;
    const int CAP = (int)FLASH_STORE_CAP_PER_SECTOR;   /* 2044 */

    begin("full_cache_migration");
    flash_sim_reset();
    Dtu_Init();
    ck(FS_PendingCount() == 0, "初始队列为空");

    for (i = 0; i < CAP; i++) {
        if (!append_seq(1, (uint32_t)i + 1, 200u + (uint32_t)i)) break;
    }
    ck(i == CAP, "应能连续写入 2044 条");
    ck(FS_PendingCount() == (uint32_t)CAP, "满缓存应为 2044 条待发");
    ck(flash_sim_erases() == 1, "填满过程只应有首次建扇区的 1 次擦除");

    /* 第 2045 条：触发"丢最旧 1/8 + 整扇区迁移" */
    before = flash_sim_erases();
    ck(append_seq(1, (uint32_t)CAP + 1, 900), "满队列仍应接受新记录");
    ck(flash_sim_erases() == before + 2, "迁移 = 擦目标扇区 + 擦旧扇区");
    ck(FS_DroppedCount() == (uint32_t)(CAP / 8), "应丢弃最旧 1/8 = 255 条");
    ck(FS_PendingCount() == (uint32_t)(CAP - CAP / 8 + 1), "保留条数 = 2044-255+1");

    /* 丢的是最旧的一批，保留记录内容完好 */
    ck(!FS_ReadById(1, 1, &rr), "seq1 应已丢弃");
    ck(!FS_ReadById(1, 255, &rr), "seq255 应已丢弃");
    ck(FS_ReadById(1, 256, &rr), "seq256 应保留");
    ck(rr.seq == 256 && rr.data[FS_CH_TEMPERATURE] == 455u, "保留记录内容必须完好");
    ck(FS_ReadById(1, (uint32_t)CAP + 1, &rr), "最新记录应保留");
    ck(flash_sim_reprogram() == 0, "迁移过程不得对同一 word 二次编程");

    /* 迁移后还能稳定写 254 条而不触发新迁移（否则又是每条一迁移 → 磨穿扇区） */
    before = flash_sim_erases();
    for (i = 0; i < 254; i++) {
        if (!append_seq(1, (uint32_t)CAP + 2 + (uint32_t)i, 1000u + (uint32_t)i)) break;
    }
    ck(i == 254, "迁移后应还能写 254 条");
    ck(flash_sim_erases() == before, "这 254 条不应触发迁移");
    ck(FS_PendingCount() == (uint32_t)CAP, "此时又回到满缓存");

    /* 断电重启：新扇区被选中，记录数与内容完整恢复 */
    FS_Init();
    ck(FS_PendingCount() == (uint32_t)CAP, "重启后记录数应完整恢复");
    ck(FS_ReadById(1, 256, &rr), "重启后最旧保留记录仍在");
    ck(FS_ReadById(1, (uint32_t)CAP + 255, &rr), "重启后最新记录仍在");
    ck(FS_GetBootId() == 1, "单纯 FS_Init 不应改变编号");
    ck(flash_sim_reprogram() == 0, "重启扫描不得产生编程");
}

/* 18. 迁移过程中掉电：旧扇区必须完整、一条不丢 */
static void case_power_loss_in_migration(void)
{
    FS_Record_t rec, rr;
    int i;
    const int CAP = (int)FLASH_STORE_CAP_PER_SECTOR;

    begin("power_loss_in_migration");
    flash_sim_reset();
    Dtu_Init();
    for (i = 0; i < CAP; i++) {
        if (!append_seq(1, (uint32_t)i + 1, 10u)) break;
    }
    ck(i == CAP, "前置：填满 2044 条");

    memset(&rec, 0, sizeof(rec));
    rec.boot_id = 1; rec.seq = (uint32_t)CAP + 1; rec.valid = 0x01;

    /* 迁移动到第 600 个 word 时掉电（目标扇区已擦、头已写、记录只复制了一部分） */
    flash_sim_fail_program_after(600);
    ck(FS_Append(&rec) == FS_ERR_FULL, "迁移中途掉电应返回 FS_ERR_FULL");
    ck(FS_PendingCount() == (uint32_t)CAP, "失败时 RAM 队列一条都不能少");
    ck(FS_DroppedCount() == 0, "失败时不得计丢弃");
    ck(FS_ReadById(1, 1, &rr), "失败后旧扇区仍可读");

    /* 断电重启：必须回到旧扇区，记录完整 */
    FS_Init();
    {
        /* 同时验证"断电阶段自证"：复制阶段中断 → 屏显 M2（D 组唯一观测手段） */
        FS_Diag_t d;
        FS_GetDiag(&d);
        ck(d.last_int_phase == FS_MIG_INT_COPY, "上电应反推出上次断在复制阶段（M2）");
    }
    ck(FS_PendingCount() == (uint32_t)CAP, "重启后记录数不变");
    ck(FS_ReadById(1, 1, &rr) && rr.seq == 1, "重启后最旧记录仍在");
    ck(FS_ReadById(1, (uint32_t)CAP, &rr) && rr.seq == (uint32_t)CAP, "重启后最新记录仍在");
    ck(flash_sim_reprogram() == 0, "掉电重来不得触发二次编程");

    /* 恢复后还能正常迁移 */
    ck(append_seq(1, (uint32_t)CAP + 1, 777u), "恢复后应能继续写入");
    ck(FS_DroppedCount() == (uint32_t)(CAP / 8), "恢复后丢弃计数应为 255");
    ck(FS_ReadById(1, (uint32_t)CAP + 1, &rr), "恢复后新记录可读");
    ck(FS_PendingCount() == (uint32_t)(CAP - CAP / 8 + 1), "恢复后条数正确");
    ck(flash_sim_reprogram() == 0, "恢复迁移不得二次编程");
}

/* 19. 迁移可观测性：阶段序列 / 按进展喂狗 / 统计计数
   上板"定点断电"测试依赖阶段标记，满缓存迁移不会饿死看门狗依赖进度回调。
   这里把这两件事在主机上钉住：阶段顺序错了，断电点就定位错了。 */
static uint32_t s_phases[16];
static uint32_t s_phase_n;
static uint32_t s_progress_n;

static void on_phase(uint8_t ph) { if (s_phase_n < 16) s_phases[s_phase_n++] = ph; }
static void on_progress(void)    { s_progress_n++; }

static void case_migration_diag(void)
{
    FS_Diag_t d;
    uint32_t i;
    const uint32_t CAP  = (uint32_t)FLASH_STORE_CAP_PER_SECTOR;
    const uint32_t DROP = CAP / 8;          /* 255 */

    begin("migration_diag");
    flash_sim_reset();
    Dtu_Init();
    for (i = 1; i <= 5; i++) ck(append_seq(1, i, i), "前置：写 5 条");

    s_phase_n = 0; s_progress_n = 0;
    FS_SetPhaseHook(on_phase);
    FS_SetProgressHook(on_progress);

    ck(FS_Recycle() == FS_OK, "手动迁移应成功");
    /* 阶段顺序 = 定点断电的坐标轴，必须严格按 擦目标→复制→提交→擦旧→空闲 */
    ck(s_phase_n == 5, "阶段回调应为 5 次（含收尾 IDLE）");
    ck(s_phases[0] == FS_PHASE_ERASE_TARGET, "阶段1 = 擦目标扇区");
    ck(s_phases[1] == FS_PHASE_COPY_RECORDS, "阶段2 = 复制记录");
    ck(s_phases[2] == FS_PHASE_COMMIT,       "阶段3 = 写完成标识");
    ck(s_phases[3] == FS_PHASE_ERASE_OLD,    "阶段4 = 擦旧扇区");
    ck(s_phases[4] == FS_PHASE_IDLE,         "收尾回到 IDLE");

    FS_GetDiag(&d);
    ck(d.migrate_ok == 1 && d.migrate_fail == 0, "迁移计数应 +1、失败 0");
    ck(d.phase == FS_PHASE_IDLE, "结束后阶段应为 IDLE");
    ck(d.max_ms == d.last_ms, "max_ms 不应小于 last_ms");
    /* 5 次：擦目标前补喂 1 + 擦目标后 1 + 擦旧前补喂 1 + 擦旧后 1 + 提交 1
       （复制不足 32 条不计）。擦除前补喂是本轮新增的：让"采集/刷屏/串口"
       这些无手册上界的耗时不再叠加到擦除上，见 BOARD_TEST_PLAN §4.2。 */
    ck(s_progress_n == 5, "短迁移的进度回调应为 5 次（两擦各前后各一次 + 提交）");

    /* 满缓存迁移：复制 1789 条，必须按进展多次喂狗，否则 4s 看门狗会中途复位 */
    flash_sim_reset();
    Dtu_Init();
    for (i = 1; i <= CAP; i++) { if (!append_seq(1, i, i)) break; }
    ck(i == CAP + 1, "前置：填满 2044 条");
    s_progress_n = 0;
    ck(append_seq(1, CAP + 1, 999u), "第 2045 条应写入成功（触发迁移）");
    ck(s_progress_n >= (CAP - DROP) / FS_PROGRESS_STEP,
       "满缓存迁移必须按进展多次喂狗（否则被 IWDG 复位）");
    FS_GetDiag(&d);
    ck(d.migrate_ok == 1 && d.phase == FS_PHASE_IDLE, "满缓存迁移应计 1 次且已收尾");

    /* 失败路径：必须记录失败阶段，且阶段回到 IDLE（不能停留在"正在迁移"） */
    flash_sim_reset();
    Dtu_Init();
    for (i = 1; i <= CAP; i++) { if (!append_seq(1, i, i)) break; }
    s_phases[0] = 0xFF;
    s_phase_n = 0;
    flash_sim_fail_program_after(600);              /* 复制途中写失败 */
    ck(!append_seq(1, CAP + 1, 7u), "迁移失败时不得写入新记录");
    FS_GetDiag(&d);
    ck(d.migrate_fail == 1 && d.migrate_ok == 0, "失败计数应为 1");
    ck(d.fail_phase == FS_PHASE_COPY_RECORDS, "失败阶段应记录为复制记录");
    ck(d.phase == FS_PHASE_IDLE, "失败后阶段必须回到 IDLE（可按此判故障）");
    ck(FS_PendingCount() == CAP, "失败时队列一条都不能少");

    FS_SetPhaseHook(0);
    FS_SetProgressHook(0);
}

/* 19. 断电阶段自证：不接示波器/不接 GPIO，靠上电扫描 Flash 现场反推"上次断在哪个
   阶段"。这是 D 组上板断电测试唯一的观测手段，判据必须钉死。 */
static void case_interrupt_phase_infer(void)
{
    FS_Diag_t d;
    FS_Record_t rr;
    uint8_t hdr[16];
    uint32_t magic = 0x4A595631u;          /* "JYV1" */
    uint32_t committed = 0u;
    uint32_t not_committed = 0xFFFFFFFFu;

    begin("interrupt_phase_infer");

    /* A：正常稳态 → NONE。擦目标阶段断电也长这样（旧扇区完好），无需区分。 */
    flash_sim_reset();
    Dtu_Init();
    ck(append_seq(1, 1, 31), "前置：写 1 条");
    FS_GetDiag(&d);
    ck(d.last_int_phase == FS_MIG_INT_NONE, "正常稳态应报 NONE");

    /* B：先成功迁移一次（活动扇区切到 S6），再人工造出
          "S5 已提交(gen1)、S6 已提交(gen2)" 的现场 = 提交后没来得及擦旧扇区。 */
    ck(append_seq(1, 2, 32), "前置：再写 1 条");
    ck(FS_Recycle() == FS_OK, "前置：迁移成功");
    {
        uint32_t gen1 = 1u, used = 2u;
        memcpy(hdr + 0, &magic, 4);
        memcpy(hdr + 4, &gen1, 4);
        memcpy(hdr + 8, &committed, 4);
        memcpy(hdr + 12, &used, 4);
        flash_sim_poke(SECTOR5_ADDR, hdr, 16);
    }
    FS_Init();
    FS_GetDiag(&d);
    ck(d.last_int_phase == FS_MIG_INT_AFTER_COMMIT, "两扇区都已提交且代次相邻 → M3");
    ck(FS_PendingCount() == 2, "必须选中代次较新的扇区，记录一条不少");
    ck(FS_ReadById(1, 2, &rr) && rr.seq == 2, "最新记录仍可读");

    /* C：目标扇区头已写、committed 仍是 0xFFFFFFFF = 死在复制阶段 → M2。
          上电必须拒绝这个未提交扇区，回到旧扇区。 */
    {
        uint32_t gen3 = 3u, used = 2u;
        memcpy(hdr + 0, &magic, 4);
        memcpy(hdr + 4, &gen3, 4);
        memcpy(hdr + 8, &not_committed, 4);
        memcpy(hdr + 12, &used, 4);
        flash_sim_poke(SECTOR5_ADDR, hdr, 16);
    }
    FS_Init();
    FS_GetDiag(&d);
    ck(d.last_int_phase == FS_MIG_INT_COPY, "头已写未提交 → M2");
    ck(FS_PendingCount() == 2, "未提交扇区不得被选中，记录不丢");
    ck(FS_ReadById(1, 1, &rr), "最旧记录仍可读");

    /* D：判定只读不改：反推过程本身不得写/擦 Flash，且随后仍能正常迁移覆盖现场 */
    ck(flash_sim_reprogram() == 0, "反推过程不得触发二次编程");
    ck(append_seq(1, 3, 33), "反推后应能继续写入");
    ck(FS_Recycle() == FS_OK, "反推现场必须能被下一次迁移正常覆盖");
    FS_Init();
    FS_GetDiag(&d);
    ck(d.last_int_phase == FS_MIG_INT_NONE, "干净迁移完成后应回到 NONE");
    ck(FS_PendingCount() == 3, "三轮之后记录仍完整");
}

/* ============================================================ */
int main(void)
{
    g_out = fopen("tests/host/last_result.txt", "w");
    if (!g_out) g_out = stdout;

    fprintf(g_out, "=== JY_EnvMon host tests (real dtu.c / flash_store.c) ===\n");

    case_first_boot();
    case_build_and_send();
    case_null_channels();
    case_ack_ok();
    case_ack_negative();
    case_ack_framing();
    case_backoff_same_msgid();
    case_interleave();
    case_late_ack_after_jump();
    case_power_cycle();
    case_identity_corrupt();
    case_storage_not_ready();
    case_many_records();
    case_review_meta_nonblank_tail();
    case_review_repair_monotonic();
    case_confirm_word_corrupt();
    case_full_cache_migration();
    case_power_loss_in_migration();
    case_migration_diag();
    case_interrupt_phase_infer();

    fprintf(g_out, "\n===== RESULT: %d passed, %d failed =====\n", g_pass, g_fail);
    fprintf(g_out, "flash program ops = %u, erase ops = %u, reprogram violations = %u\n",
            flash_sim_programmed(), flash_sim_erases(), flash_sim_reprogram());

    if (g_out != stdout) fclose(g_out);
    return (g_fail == 0) ? 0 : 1;
}
