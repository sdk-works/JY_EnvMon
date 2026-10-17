/* 验证 P0-2/P0-3：光照 uint32 不溢出 + 无效通道输出 null */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* 复刻 dtu.c 的 Dtu_PutValue / 字段构造逻辑做等价验证 */
#define FS_CH_NUM 8
enum { FS_CH_TEMPERATURE=0, FS_CH_HUMIDITY, FS_CH_CO2, FS_CH_LIGHT,
       FS_CH_PRESSURE, FS_CH_SOIL_TEMP, FS_CH_SOIL_MOISTURE, FS_CH_SOIL_EC };

typedef struct {
    uint32_t boot_id, seq, sampled_at;
    uint64_t uptime_ms;
    uint8_t  time_quality, valid;
    uint32_t data[FS_CH_NUM];
} FS_Record_t;

static int Dtu_PutValue(char *buf, size_t pos, size_t maxlen,
                        int valid, const char *fmt, double value)
{
    if (!valid) {
        const char *null_str = "null";
        size_t l = strlen(null_str);
        if (pos + l >= maxlen) return -1;
        memcpy(&buf[pos], null_str, l);
        return (int)l;
    }
    int n = snprintf(&buf[pos], maxlen - pos, fmt, value);
    return (n < 0) ? -1 : n;
}

static int build(const FS_Record_t *rec, char *buf, uint16_t maxlen)
{
    int v[FS_CH_NUM];
    for (int i = 0; i < FS_CH_NUM; i++) v[i] = (rec->valid >> i) & 1;
    uint32_t light_u32 = (uint32_t)rec->data[FS_CH_LIGHT];

    size_t pos = 0;
    int n = snprintf(buf, maxlen, "{\"data\":{");
    if (n < 0) return -1;
    pos = (size_t)n;

    struct { const char *key; int valid; const char *fmt; double val; } f[] = {
        { "\"temperature\":",     v[FS_CH_TEMPERATURE],    "%.1f", (double)((int32_t)rec->data[FS_CH_TEMPERATURE])/10.0 },
        { "\"humidity\":",        v[FS_CH_HUMIDITY],       "%.1f", (double)((int32_t)rec->data[FS_CH_HUMIDITY])/10.0 },
        { "\"co2\":",             v[FS_CH_CO2],            "%.0f", (double)((int32_t)rec->data[FS_CH_CO2]) },
        { "\"light\":",           v[FS_CH_LIGHT],          "%.0f", (double)light_u32 },
        { "\"pressure\":",        v[FS_CH_PRESSURE],       "%.1f", (double)((int32_t)rec->data[FS_CH_PRESSURE])/10.0 },
        { "\"soil_temperature\":",v[FS_CH_SOIL_TEMP],      "%.1f", (double)((int32_t)rec->data[FS_CH_SOIL_TEMP])/10.0 },
        { "\"soil_moisture\":",   v[FS_CH_SOIL_MOISTURE],  "%.1f", (double)((int32_t)rec->data[FS_CH_SOIL_MOISTURE])/10.0 },
        { "\"soil_ec\":",         v[FS_CH_SOIL_EC],        "%.0f", (double)((int32_t)rec->data[FS_CH_SOIL_EC]) },
    };
    for (size_t i = 0; i < sizeof(f)/sizeof(f[0]); i++) {
        size_t kl = strlen(f[i].key);
        memcpy(&buf[pos], f[i].key, kl); pos += kl;
        int w = Dtu_PutValue(buf, pos, maxlen, f[i].valid, f[i].fmt, f[i].val);
        if (w < 0) return -1;
        pos += (size_t)w;
        buf[pos++] = ',';
    }
    n = snprintf(&buf[pos], maxlen - pos, "\"valid\":[%d,%d,%d,%d,%d,%d,%d,%d]}}\n",
                 v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);
    if (n < 0) return -1;
    pos += (size_t)n;
    buf[pos] = 0;
    return (int)pos;
}

static int g_fail = 0;
#define CHECK(c,m) do{ if(!(c)){printf("  [FAIL] %s\n",m); g_fail++;} else printf("  [ok]   %s\n",m);}while(0)

int main(void)
{
    char buf[512];
    printf("=== JSON 生成验证（P0-2 光照 / P0-3 null）===\n\n");

    /* 1. 强光：4000000000 lux（>2^31，旧实现转 int32 会变负） */
    printf("[1] 强光 4000000000 lux\n");
    FS_Record_t r; memset(&r, 0, sizeof(r));
    r.valid = 0xFF;
    r.data[FS_CH_TEMPERATURE] = 215;
    r.data[FS_CH_HUMIDITY] = 545;
    r.data[FS_CH_CO2] = 603;
    r.data[FS_CH_LIGHT] = 4000000000u;
    r.data[FS_CH_PRESSURE] = 1016;
    r.data[FS_CH_SOIL_TEMP] = 206;
    r.data[FS_CH_SOIL_MOISTURE] = 450;
    r.data[FS_CH_SOIL_EC] = 1200;
    build(&r, buf, sizeof(buf));
    printf("  %s\n", buf);
    CHECK(strstr(buf, "\"light\":4000000000") != NULL, "光照正确输出 4000000000（无溢出/无精度损失）");
    CHECK(strstr(buf, "\"light\":-") == NULL, "光照不是负数");

    /* 2. 部分无效：百叶箱全失效（bit0-4=0），土壤有效 */
    printf("\n[2] 百叶箱失效、土壤有效（valid=0xE0）\n");
    FS_Record_t r2 = r; r2.valid = 0xE0;
    build(&r2, buf, sizeof(buf));
    printf("  %s\n", buf);
    CHECK(strstr(buf, "\"temperature\":null") != NULL, "温度输出 null");
    CHECK(strstr(buf, "\"humidity\":null") != NULL, "湿度输出 null");
    CHECK(strstr(buf, "\"co2\":null") != NULL, "CO2 输出 null");
    CHECK(strstr(buf, "\"light\":null") != NULL, "光照输出 null");
    CHECK(strstr(buf, "\"soil_ec\":1200") != NULL, "土壤 EC 仍输出数值");
    CHECK(strstr(buf, "\"valid\":[0,0,0,0,0,1,1,1]") != NULL, "valid 数组正确");

    /* 3. 合法零值：夜间光照 0、CO2 0 —— 必须输出 0，不能被当成无效 */
    printf("\n[3] 合法零值（光照=0、CO2=0，全部有效）\n");
    FS_Record_t r3 = r; r3.valid = 0xFF;
    r3.data[FS_CH_LIGHT] = 0; r3.data[FS_CH_CO2] = 0;
    build(&r3, buf, sizeof(buf));
    printf("  %s\n", buf);
    CHECK(strstr(buf, "\"light\":0,") != NULL, "光照 0 输出为 0（不是 null）");
    CHECK(strstr(buf, "\"co2\":0,") != NULL, "CO2 0 输出为 0（不是 null）");

    /* 4. 全量无效 */
    printf("\n[4] 全量无效（在线但采集异常）\n");
    FS_Record_t r4 = r; r4.valid = 0x00;
    build(&r4, buf, sizeof(buf));
    printf("  %s\n", buf);
    CHECK(strstr(buf, "\"valid\":[0,0,0,0,0,0,0,0]") != NULL, "valid 全 0");
    {
        int cnt = 0; const char *p = buf;
        while ((p = strstr(p, "null")) != NULL) { cnt++; p += 4; }
        CHECK(cnt == 8, "8 个通道全部输出 null");
    }

    printf("\n=== 结果：%s ===\n", g_fail == 0 ? "全部通过" : "存在失败");
    return g_fail ? 1 : 0;
}
