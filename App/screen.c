/**
  ******************************************************************************
  * @file    screen.c
  * @brief   BX-5K1 LED 大屏驱动：左中文名称 + 右上数值 + 右下单位（三区域单帧）
  *
 * 布局（物理 64×16 像素，每次整屏只显示一项指标，8 秒切换）：
 *   名称区  X=0 , Y=2 , W=24, H=12   两个 12×12 汉字（CO2 用英文小字）
 *   数值区  X=26, Y=0 , W=38, H=8    E000 7×8，最多 5 字符
 *   单位区  X=26, Y=8 , W=38, H=8    E000 7×8，最多 5 字符
 *   数值与单位都水平居中于同一竖线（X 中心 = 26 + 38/2 = 45），视觉上成一列。
 *   左右间隔 2 像素；名称区 Y=2 使 12px 字落在 16px 屏正中。
 *   精度：温度/湿度/气压固定 1 位小数；土湿、CO2、EC 为整数。
  *
 * 温度单位 = "°C"：同一单位区内先切中文字库输出 °(A1E3)，再切英文字库输出 C。
 *   已知问题（本次不解决）：° 用 12×12 中文、C 用 8 高英文，同一区域混用不同高度
 *   字体，协议并不允许；单位区仅高 8，实际效果需单独验证。
  *
  * 协议要点（BX-5K/6K 通讯协议）：
  *   帧头 8×0xA5，帧尾 0x5A，包校验 CRC16(初值0, poly=0xA001)
  *   命令 A3 06 = 发送实时显示区域数据，数据域结构：
  *     A3 06 Resp ProcessMode 00 DeleteAreaNum AreaNum
  *     [ AreaDataLen(2字节, 小端) + AreaBlock ] × AreaNum
  *   AreaBlock = 区域头(23字节) + DataLen(4字节, 小端) + Data(含 \F 字库转义)
  *   注意：外层 AreaDataLen 是 2 字节，区域末尾 DataLen 是 4 字节，二者不同。
  *
  * 字库（协议 §11.1：内容开始处必须用 "\F"+字库名 指定字库）：
  *   中文 O000 = 12×12 宋体；英文/数字 E000 = 7×8 Arial。
  *   程序按区域分别调用字库，中英文分区域/分段，不把不同高度的字混在同一段。
  ******************************************************************************
  */
#include "screen.h"
#include <string.h>
#include <stdio.h>

/* ===== 字库尺寸（须与仰邦字库工具里生成的字模一致） ===== */
#define SCREEN_CN_W        12
#define SCREEN_CN_H        12
#define SCREEN_EN_W         7
#define SCREEN_EN_H         8

/* ===== 三区域几何（像素） ===== */
#define AREA_NAME_X         0
#define AREA_NAME_Y         2
#define AREA_NAME_W        24
#define AREA_NAME_H        12

#define AREA_VAL_X         26
#define AREA_VAL_Y          0
#define AREA_VAL_W         38
#define AREA_VAL_H          8

#define AREA_UNI_X         26
#define AREA_UNI_Y          8
#define AREA_UNI_W         38
#define AREA_UNI_H          8

/* ===== 区域显示参数 ===== */
/* DisplayMode：协议规定 0x01 = 静止显示；实屏已验证 0x01 正常。
 * 早期注释“实测 0x00 为静止”已作废，不要改回 0x00。 */
#define SCREEN_AREA_MODE_FIXED  0x01
/* ProcessMode：1=收到动态信息后清区域并初始化（每帧重画三个区域，避免残影） */
#define SCREEN_PROC_MODE        0x01
/* TextAlignment 是位域：bit[1:0] 00左/01右/10居中，bit[3:2] 00上/01下/10居中 */
#define SCREEN_ALIGN_CENTER     0x0A   /* 水平居中 + 垂直居中 */

/* ===== 字库名（须与控制器内实际下载的字库名一致） ===== */
#define SCREEN_FONT_NAME_CN  "O000"   /* 12×12 中文（字号 9） */
#define SCREEN_FONT_NAME_EN  "E000"   /* 7×8  英文/单位 */
#define SCREEN_FONT_ESC_LEN   9U      /* "\F" + 4 位字库名 + "\C1" 红色 */

/* ===== 数值 / 单位字库与每区最大字符数 =====
 * 右区宽 38px，E000 为 7×8，字符步进 = 字框宽度（协议支持字间距扩展，本次不启用）：
 *   38 / 7 = 5 字符。数值与单位分别按各自上限检查。
 * 不再使用 E001；数值与单位统一 E000 7×8。 */
#define SCREEN_FONT_NAME_NUM  SCREEN_FONT_NAME_EN
#define SCREEN_NUM_FONT_W     SCREEN_EN_W
#define SCREEN_VAL_MAX        (AREA_VAL_W / SCREEN_NUM_FONT_W)   /* 5，如 101.4 */
#define SCREEN_UNIT_MAX       (AREA_UNI_W / SCREEN_EN_W)         /* 5，如 uS/cm */
#define SCREEN_LUX_KLX_MIN    100000UL   /* 5 字符放不下 6 位，≥1e5 lux 转 klx */

/* ===== 单位 ===== */
/* 温度单位 = °(中文字库) + C(英文字库) 拼接，同一单位区内中途切换 \F：
 *   ° 在 GB2312 是 A1E3，只有中文字库有；C 走 7×8 英文字库。
 *   注意：° 来自 12×12 中文字模，而单位区仅 8px 高——° 字形位于字模上部，
 *   正常情况不会落到被裁的底部。若实测 ° 被切或与 C 基线不齐，
 *   把 SCREEN_TEMP_UNIT_DEG_C 置 0，退回单一 ASCII "C"。 */
#define SCREEN_TEMP_UNIT_DEG_C   1

static const uint8_t U_DEG[] = {0xA1, 0xE3};   /* ° GB2312 A1E3（中文字库） */
static const uint8_t U_C  [] = {'C'};          /* C ASCII（英文字库） */

#define U_RH     "%RH"
#define U_PPM    "ppm"
#define U_LX     "lx"
#define U_KLX    "klx"
#define U_KPA    "kPa"
#define U_PCT    "%"

/* 电导单位：正确物理量是 µS/cm（不能缩成 µS / uS，否则物理量变了）。
 * µ(0xB5) 不在 ASCII 里，需在 7×8 英文字库里做该字形；未做时用兼容写法 uS/cm。
 * 7×8 下 5 字符 = 35px，38px 区余 3px，不会裁切。 */
#define SCREEN_EC_USE_MICRO   0
#if SCREEN_EC_USE_MICRO
  #define U_EC  "\xB5" "S/cm"   /* 拆成两个字面量，避免 \xB5S 被当成一个转义 */
#else
  #define U_EC  "uS/cm"
#endif

/* ===== 缓冲尺寸 ===== */
#define SCREEN_AREA_DATA_MAX  32U   /* 每区域数据上限：\F 各段 + \C1 颜色之和 */
#define SCREEN_REGION_LEN     23U   /* 区域头固定 23 字节 */
#define SCREEN_BLOCK_MAX      (2U + SCREEN_REGION_LEN + 4U + SCREEN_AREA_DATA_MAX)  /* 61 */
#define SCREEN_DOMAIN_MAX     (7U + 3U * SCREEN_BLOCK_MAX)                          /* 190 */
#define SCREEN_RAW_MAX        (14U + SCREEN_DOMAIN_MAX + 2U)                        /* 206 */
#define SCREEN_WIRE_MAX       (8U + 2U * SCREEN_RAW_MAX + 1U)                       /* 421 */

/* 一帧内的一个显示区域：data 已是含 \F 字库转义的完整显示数据 */
typedef struct {
    uint16_t x, y, w, h;
    uint8_t  data[SCREEN_AREA_DATA_MAX];
    uint16_t len;
} Screen_Area_t;

/* ===== 名称（GB2312 / ASCII） ===== */
static const uint8_t NM_TEMP[] = {0xCE, 0xC2, 0xB6, 0xC8};  /* 温度 */
static const uint8_t NM_HUMI[] = {0xCA, 0xAA, 0xB6, 0xC8};  /* 湿度 */
static const uint8_t NM_CO2 [] = {'C',  'O',  '2'      };   /* CO2  */
static const uint8_t NM_LUX [] = {0xB9, 0xE2, 0xD5, 0xD5};  /* 光照 */
static const uint8_t NM_PRES[] = {0xC6, 0xF8, 0xD1, 0xB9};  /* 气压 */
static const uint8_t NM_STMP[] = {0xCD, 0xC1, 0xCE, 0xC2};  /* 土温 */
static const uint8_t NM_SHUM[] = {0xCD, 0xC1, 0xCA, 0xAA};  /* 土湿 */
static const uint8_t NM_EC  [] = {0xB5, 0xE7, 0xB5, 0xBC};  /* 电导 */

/* LED CRC16: 初值 0, poly=0xA001 */
static uint16_t Screen_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++) {
        uint8_t j;
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (uint16_t)((crc >> 1) ^ 0xA001);
            else         crc >>= 1;
        }
    }
    return crc;
}

/* 一位小数定点格式化（整数运算，避免浮点 printf）：25.6 / -10.5 */
static uint16_t fmt_f1(char *buf, float v)
{
    int neg = (v < 0.0f);
    float a = neg ? -v : v;
    uint32_t x = (uint32_t)(a * 10.0f + 0.5f);   /* 四舍五入到 0.1 */
    int n;
    if (x == 0) neg = 0;                          /* 吃掉 -0.0 */
    if (neg) n = sprintf(buf, "-%lu.%lu", (unsigned long)(x / 10U), (unsigned long)(x % 10U));
    else     n = sprintf(buf,  "%lu.%lu", (unsigned long)(x / 10U), (unsigned long)(x % 10U));
    return (uint16_t)n;
}

/* 无效读数占位：-- */
static uint16_t fmt_na(char *buf)
{
    buf[0] = '-'; buf[1] = '-'; buf[2] = '\0';
    return 2;
}

/* 向区域数据追加一段：\F + 字库名(4) + \C1(红色) + 文本；返回写入字节数 */
static uint16_t Screen_Seg(uint8_t *dst, const char *font, const uint8_t *text, uint16_t n)
{
    dst[0] = 0x5C;                 /* '\' */
    dst[1] = 'F';
    memcpy(dst + 2, font, 4);
    dst[6] = 0x5C;                 /* '\' */
    dst[7] = 'C';
    dst[8] = '1';                  /* 显式红色 */
    if (n) memcpy(dst + SCREEN_FONT_ESC_LEN, text, n);
    return (uint16_t)(SCREEN_FONT_ESC_LEN + n);
}

/* 组一帧（N 个区域），返回转义后的线上字节数 */
static uint16_t Screen_BuildFrame(const Screen_Area_t *areas, uint8_t n, uint8_t *out)
{
    uint8_t  raw[SCREEN_RAW_MAX];
    uint8_t *p = raw;
    uint8_t *dlenpos;
    uint16_t crc, nraw, wl, i;
    static const uint8_t phy1[12] = {
        0x01, 0x00,              /* DstAddr = 1 */
        0x00, 0x80,              /* SrcAddr = 0x8000 */
        0x00, 0x00, 0x00,        /* Reserved */
        0x00,                    /* BarCodeOption = 0 */
        0x00,                    /* CheckMode = 0 (CRC16) */
        0x01,                    /* DisplayMode = 1 (动态) */
        0xFE,                    /* DeviceType = 通配 */
        0x02                     /* ProtocolVersion */
    };

    if (out == NULL || areas == NULL || n == 0 || n > 3) return 0;

    memcpy(p, phy1, sizeof(phy1)); p += sizeof(phy1);
    dlenpos = p; p += 2;                 /* DataLen 稍后回填 */

    /* 命令 A3 06 */
    *p++ = 0xA3;
    *p++ = 0x06;
    *p++ = 0x00;                         /* Response：0 不需回 */
    *p++ = SCREEN_PROC_MODE;             /* ProcessMode */
    *p++ = 0x00;                         /* Reserved（本命令仅 1 字节） */
    *p++ = 0x00;                         /* DeleteAreaNum = 0 */
    *p++ = n;                            /* AreaNum */

    for (i = 0; i < n; i++) {
        const Screen_Area_t *a = &areas[i];
        uint16_t dataLen = a->len;
        uint16_t areaLen = (uint16_t)(SCREEN_REGION_LEN + 4U + dataLen);

        /* 区域数据长度（2 字节，小端） */
        *p++ = (uint8_t)areaLen;
        *p++ = (uint8_t)(areaLen >> 8);

        /* ---- 区域头 23 字节 ---- */
        *p++ = 0x00;                                     /* AreaType = 文本 */
        *p++ = (uint8_t)(a->x & 0xFF);
        *p++ = (uint8_t)(0x80 | (a->x >> 8));            /* X：像素单位 */
        *p++ = (uint8_t)(a->y & 0xFF);
        *p++ = (uint8_t)(a->y >> 8);
        *p++ = (uint8_t)(a->w & 0xFF);
        *p++ = (uint8_t)(0x80 | (a->w >> 8));            /* W：像素单位 */
        *p++ = (uint8_t)(a->h & 0xFF);
        *p++ = (uint8_t)(a->h >> 8);
        *p++ = (uint8_t)i;                               /* DynamicAreaLoc 0/1/2 */
        *p++ = 0x00;                                     /* Lines_sizes 行间距 */
        *p++ = 0x01;                                     /* RunMode 停末页 */
        *p++ = 0x00; *p++ = 0x00;                        /* Timeout */
        *p++ = 0x00;                                     /* SoundMode 不语音 */
        *p++ = 0x00;                                     /* ExtendParaLen */
        *p++ = SCREEN_ALIGN_CENTER;                      /* TextAlignment 居中 */
        *p++ = 0x01;                                     /* SingleLine 单行 */
        *p++ = 0x01;                                     /* NewLine 不自动换行 */
        *p++ = SCREEN_AREA_MODE_FIXED;                   /* DisplayMode */
        *p++ = 0x00;                                     /* ExitMode */
        *p++ = 0x00;                                     /* Speed */
        *p++ = 0x00;                                     /* StayTime */

        /* ---- 显示数据长度（4 字节，小端，含 \F 转义） ---- */
        *p++ = (uint8_t)dataLen;
        *p++ = (uint8_t)(dataLen >> 8);
        *p++ = 0x00;
        *p++ = 0x00;

        /* ---- 显示数据（已含 \F 字库转义，可直接多段拼接） ---- */
        if (dataLen) { memcpy(p, a->data, dataLen); p += dataLen; }
    }

    /* 回填数据域长度（不含包头/包校验/帧尾/帧头） */
    {
        uint16_t domain = (uint16_t)(p - (raw + 14));
        dlenpos[0] = (uint8_t)domain;
        dlenpos[1] = (uint8_t)(domain >> 8);
    }

    crc = Screen_CRC16(raw, (uint16_t)(p - raw));
    *p++ = (uint8_t)crc;
    *p++ = (uint8_t)(crc >> 8);
    nraw = (uint16_t)(p - raw);

    /* PHY0：8×0xA5 + 字符转义 + 0x5A */
    memset(out, 0xA5, 8);
    wl = 8;
    for (i = 0; i < nraw; i++) {
        switch (raw[i]) {
        case 0xA5: out[wl++] = 0xA6; out[wl++] = 0x02; break;
        case 0xA6: out[wl++] = 0xA6; out[wl++] = 0x01; break;
        case 0x5A: out[wl++] = 0x5B; out[wl++] = 0x02; break;
        case 0x5B: out[wl++] = 0x5B; out[wl++] = 0x01; break;
        default:   out[wl++] = raw[i]; break;
        }
    }
    out[wl++] = 0x5A;
    return wl;
}

/* HAL_UART_Transmit 成功会等到 TC；超时只用于界定失败。三区域帧在 9600bps
   下约 150~320ms，故给 1000ms。 */
static void Screen_Send(const uint8_t *data, uint16_t len)
{
    if (len == 0) return;
    HAL_GPIO_WritePin(T3_EN_GPIO_Port, T3_EN_Pin, GPIO_PIN_SET);
    (void)HAL_UART_Transmit(&huart5, (uint8_t *)data, len, 1000);
    HAL_GPIO_WritePin(T3_EN_GPIO_Port, T3_EN_Pin, GPIO_PIN_RESET);
}

/* 显示第 idx 项（0..7）：名称 / 数值 / 单位 三区域一次下发 */
void Screen_ShowItem(uint8_t idx)
{
    static uint8_t frame[SCREEN_WIRE_MAX];
    Screen_Area_t areas[3];
    const uint8_t *name = NM_TEMP;
    const char *name_font = SCREEN_FONT_NAME_CN;
    uint16_t nlen = 4;
    char val[16];
    char uni[8];
    uint16_t vlen = 0, ulen = 0;
    uint8_t valid = 0;
    uint8_t temp_unit = 0;   /* 1 = 温度类，单位由 °(中文) + C(英文) 拼接 */

    val[0] = '\0';
    uni[0] = '\0';

    switch (idx) {
    case 0: /* 空气温度 */
        name = NM_TEMP; nlen = 4;
        valid = g_env.ble_box_valid;
        vlen = valid ? fmt_f1(val, g_env.air_temp) : fmt_na(val);
        temp_unit = 1;
        break;

    case 1: /* 空气湿度 */
        name = NM_HUMI; nlen = 4;
        valid = g_env.ble_box_valid;
        vlen = valid ? fmt_f1(val, g_env.air_humi) : fmt_na(val);
        strcpy(uni, U_RH); ulen = (uint16_t)strlen(uni);
        break;

    case 2: /* CO2 */
        name = NM_CO2; nlen = 3;
        valid = g_env.ble_box_valid;
        if (valid) vlen = (uint16_t)sprintf(val, "%lu", (unsigned long)(uint32_t)(g_env.co2 + 0.5f));
        else       vlen = fmt_na(val);
        strcpy(uni, U_PPM); ulen = (uint16_t)strlen(uni);
        break;

    case 3: /* 光照 */
        name = NM_LUX; nlen = 4;
        valid = g_env.ble_box_valid;
        if (!valid) {
            vlen = fmt_na(val);
            strcpy(uni, U_LX);
        } else if (g_env.lux >= SCREEN_LUX_KLX_MIN) {
            /* 超 6 位 → 换 klx 并保留一位小数；仅影响显示，不改变采集/上传原值 */
            vlen = fmt_f1(val, (float)g_env.lux / 1000.0f);
            strcpy(uni, U_KLX);
        } else {
            vlen = (uint16_t)sprintf(val, "%lu", (unsigned long)g_env.lux);
            strcpy(uni, U_LX);
        }
        ulen = (uint16_t)strlen(uni);
        break;

    case 4: /* 大气压 */
        name = NM_PRES; nlen = 4;
        valid = g_env.ble_box_valid;
        vlen = valid ? fmt_f1(val, g_env.pressure) : fmt_na(val);
        strcpy(uni, U_KPA); ulen = (uint16_t)strlen(uni);
        break;

    case 5: /* 土壤温度 */
        name = NM_STMP; nlen = 4;
        valid = g_env.soil_valid;
        vlen = valid ? fmt_f1(val, g_env.soil_temp) : fmt_na(val);
        temp_unit = 1;
        break;

    case 6: /* 土壤湿度：传感器本身就是整数百分比，不再补 .0 造成假精度 */
        name = NM_SHUM; nlen = 4;
        valid = g_env.soil_valid;
        if (valid) vlen = (uint16_t)sprintf(val, "%lu", (unsigned long)(uint32_t)(g_env.soil_humi + 0.5f));
        else       vlen = fmt_na(val);
        strcpy(uni, U_PCT); ulen = (uint16_t)strlen(uni);
        break;

    case 7: /* 土壤电导 */
        name = NM_EC; nlen = 4;
        valid = g_env.soil_valid;
        if (valid) vlen = (uint16_t)sprintf(val, "%lu", (unsigned long)(uint32_t)(g_env.soil_ec + 0.5f));
        else       vlen = fmt_na(val);
        strcpy(uni, U_EC); ulen = (uint16_t)strlen(uni);
        break;

    default:
        return;
    }

    /* 超宽保护：超过各自上限显示 ERR，不截断前后数字 */
    if (vlen > SCREEN_VAL_MAX)  { strcpy(val, "ERR"); vlen = 3; }
    if (ulen > SCREEN_UNIT_MAX) { strcpy(uni, "ERR"); ulen = 3; }

    /* ---- 名称区：中文用中文字库，纯 ASCII（CO2）用英文字库 ---- */
    if (name[0] < 0x80) name_font = SCREEN_FONT_NAME_EN;
    areas[0].x = AREA_NAME_X; areas[0].y = AREA_NAME_Y;
    areas[0].w = AREA_NAME_W; areas[0].h = AREA_NAME_H;
    areas[0].len = Screen_Seg(areas[0].data, name_font, name, nlen);

    /* ---- 数值区：E000 7×8 ---- */
    areas[1].x = AREA_VAL_X; areas[1].y = AREA_VAL_Y;
    areas[1].w = AREA_VAL_W; areas[1].h = AREA_VAL_H;
    areas[1].len = Screen_Seg(areas[1].data, SCREEN_FONT_NAME_NUM, (const uint8_t *)val, vlen);

    /* ---- 单位区 ---- */
    areas[2].x = AREA_UNI_X; areas[2].y = AREA_UNI_Y;
    areas[2].w = AREA_UNI_W; areas[2].h = AREA_UNI_H;

    if (temp_unit) {
#if SCREEN_TEMP_UNIT_DEG_C
        uint16_t o = 0;
        /* 同一区域内：\F中文字库 + °，再 \F英文字库 + C */
        o = Screen_Seg(areas[2].data, SCREEN_FONT_NAME_CN, U_DEG, 2);
        o = (uint16_t)(o + Screen_Seg(areas[2].data + o, SCREEN_FONT_NAME_EN, U_C, 1));
        areas[2].len = o;
#else
        areas[2].len = Screen_Seg(areas[2].data, SCREEN_FONT_NAME_EN, U_C, 1);
#endif
    } else {
        areas[2].len = Screen_Seg(areas[2].data, SCREEN_FONT_NAME_EN, (const uint8_t *)uni, ulen);
    }

    {
        uint16_t flen = Screen_BuildFrame(areas, 3, frame);
        Screen_Send(frame, flen);
    }
}

/* 通用一帧：中文名称(GB2312, ≤2 字) + 数值(ASCII, ≤5) + 单位(ASCII, ≤5)。
   供 app.c 的诊断页使用（待发/丢弃/阶段/喂狗间隔…），不改动上面 8 个指标页的布局。
   val / uni 传 NULL 或空串表示该区域留空。 */
void Screen_ShowText(const uint8_t *name, uint16_t name_len,
                     const char *val, const char *uni)
{
    static uint8_t frame[SCREEN_WIRE_MAX];
    Screen_Area_t areas[3];
    const char *name_font = SCREEN_FONT_NAME_CN;
    uint16_t vlen = 0, ulen = 0;

    if (name == NULL) return;
    if (name_len > 4) name_len = 4;                 /* 名称区 24px / 12px = 2 个汉字 */

    if (val) { vlen = (uint16_t)strlen(val); if (vlen > SCREEN_VAL_MAX)  vlen = SCREEN_VAL_MAX; }
    if (uni) { ulen = (uint16_t)strlen(uni); if (ulen > SCREEN_UNIT_MAX) ulen = SCREEN_UNIT_MAX; }

    if (name_len && name[0] < 0x80) name_font = SCREEN_FONT_NAME_EN;   /* 纯 ASCII 用英文字库 */

    areas[0].x = AREA_NAME_X; areas[0].y = AREA_NAME_Y;
    areas[0].w = AREA_NAME_W; areas[0].h = AREA_NAME_H;
    areas[0].len = name_len ? Screen_Seg(areas[0].data, name_font, name, name_len) : 0;

    areas[1].x = AREA_VAL_X;  areas[1].y = AREA_VAL_Y;
    areas[1].w = AREA_VAL_W;  areas[1].h = AREA_VAL_H;
    areas[1].len = vlen ? Screen_Seg(areas[1].data, SCREEN_FONT_NAME_NUM,
                                     (const uint8_t *)val, vlen) : 0;

    areas[2].x = AREA_UNI_X;  areas[2].y = AREA_UNI_Y;
    areas[2].w = AREA_UNI_W;  areas[2].h = AREA_UNI_H;
    areas[2].len = ulen ? Screen_Seg(areas[2].data, SCREEN_FONT_NAME_EN,
                                     (const uint8_t *)uni, ulen) : 0;

    {
        uint16_t flen = Screen_BuildFrame(areas, 3, frame);
        Screen_Send(frame, flen);
    }
}
