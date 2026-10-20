/**
  ******************************************************************************
  * @file    screen.c
  * @brief   BX-5K1 LED 大屏驱动：左中文名称 + 右上数值 + 右下单位（三区域单帧）
  *
  * 布局（物理 64×16 像素，每次整屏只显示一项指标，8 秒切换）：
  *   名称区  X=0 , Y=2 , W=24, H=12   两个 12×12 汉字（CO2 用 6×8 英文）
  *   数值区  X=26, Y=0 , W=38, H=8    最多 6 个 6×8 ASCII 字符
  *   单位区  X=26, Y=8 , W=38, H=8    最多 6 个 6×8 ASCII 字符
  *   左右间隔 2 像素；名称区 Y=2 使 12px 字落在 16px 屏正中。
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
  *   中文 O000 = 12×12 宋体；英文/数字 E000 = 6×8 Arial。
  *   程序按区域分别调用字库，不把不同高度的字混在同一区域。
  ******************************************************************************
  */
#include "screen.h"
#include <string.h>
#include <stdio.h>

/* ===== 字库尺寸（须与仰邦字库工具里生成的字模一致） ===== */
#define SCREEN_CN_W        12
#define SCREEN_CN_H        12
#define SCREEN_EN_W         6
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

/* 单个区域最多字符数：38px / 6px = 6 */
#define SCREEN_VAL_MAX      6

/* ===== 区域显示参数 ===== */
/* DisplayMode：本机实测 0x00 为静止（协议标称 0x01=静止，如需可改 0x01） */
#define SCREEN_AREA_MODE_FIXED  0x00
/* ProcessMode：1=收到动态信息后清区域并初始化（每帧重画三个区域，避免残影） */
#define SCREEN_PROC_MODE        0x01
/* TextAlignment 是位域：bit[1:0] 00左/01右/10居中，bit[3:2] 00上/01下/10居中 */
#define SCREEN_ALIGN_CENTER     0x0A   /* 水平居中 + 垂直居中 */

/* ===== 字库名（须与控制器内实际下载的字库名一致） ===== */
#define SCREEN_FONT_NAME_CN  "O000"   /* 12×12 中文 */
#define SCREEN_FONT_NAME_EN  "E000"   /* 6×8  英文/数字 */
#define SCREEN_FONT_ESC_LEN   6U      /* "\F" + 4 位字库名 */

/* ===== 单位（走英文字库；单位区高 8px） ===== */
/* 温度单位：方案 C —— 单独的 ≤8px 高 ℃ 小字库。
   ℃ 在 GB2312 里是 A1E6（全角），12×12 中文字库塞不进 8px 高的单位区，
   所以另做一个只含 ℃ 的 8px 高小字库（默认名 O001），温度项的单位区改走它。
   若 O001 尚未做好，把 SCREEN_TEMP_UNIT_CJK8 置 0，可退回 ASCII "C"（走 E000）。 */
#define SCREEN_TEMP_UNIT_CJK8   1
#define SCREEN_FONT_NAME_CJK8   "O001"   /* 8px 高中文小字库，至少含 ℃ */

#if SCREEN_TEMP_UNIT_CJK8
  #define U_TEMP  "\xA1\xE6"   /* ℃（GB2312 A1E6），由 O001 渲染 */
#else
  #define U_TEMP  "C"
#endif
#define U_RH     "%RH"
#define U_PPM    "ppm"
#define U_LX     "lx"
#define U_KLX    "klx"
#define U_KPA    "kPa"
#define U_PCT    "%"
#define U_EC     "uS/cm"

/* ===== 缓冲尺寸 ===== */
#define SCREEN_DATA_MAX    16U    /* 每区域数据上限：\F(6) + 文本(<=6) 余量 */
#define SCREEN_REGION_LEN  23U    /* 区域头固定 23 字节 */
#define SCREEN_BLOCK_MAX   (2U + SCREEN_REGION_LEN + 4U + SCREEN_DATA_MAX)  /* 45 */
#define SCREEN_DOMAIN_MAX  (7U + 3U * SCREEN_BLOCK_MAX)                     /* 142 */
#define SCREEN_RAW_MAX     (14U + SCREEN_DOMAIN_MAX + 2U)                   /* 158，含 CRC2 */
#define SCREEN_WIRE_MAX    (8U + 2U * SCREEN_RAW_MAX + 1U)                  /* 325 */

/* 一帧内的一个显示区域 */
typedef struct {
    uint16_t x, y, w, h;
    const char    *font;   /* 4 位字库名 */
    const uint8_t *text;   /* 文本字节（中文 GB2312 / 英文 ASCII） */
    uint16_t       len;    /* 文本字节数 */
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
        uint16_t dataLen = (uint16_t)(SCREEN_FONT_ESC_LEN + a->len);
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

        /* ---- 显示数据：\F + 字库名(4) + 文本 ---- */
        *p++ = 0x5C;                                     /* '\' */
        *p++ = 'F';
        memcpy(p, a->font, 4); p += 4;
        if (a->len) { memcpy(p, a->text, a->len); p += a->len; }
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
   下约 150~310ms，故给 1000ms。 */
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
    const uint8_t *name = NM_TEMP;
    const char *uni_font = SCREEN_FONT_NAME_EN;
    uint16_t nlen = 4;
    char val[16];
    char uni[8];
    uint16_t vlen = 0, ulen = 0;
    uint8_t valid;

    val[0] = '\0';
    uni[0] = '\0';

    switch (idx) {
    case 0: /* 空气温度 */
        name = NM_TEMP; nlen = 4;
        valid = g_env.ble_box_valid;
        vlen = valid ? fmt_f1(val, g_env.air_temp) : fmt_na(val);
        strcpy(uni, U_TEMP); ulen = (uint16_t)strlen(uni);
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
        } else if (g_env.lux >= 1000000UL) {
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
        strcpy(uni, U_TEMP); ulen = (uint16_t)strlen(uni);
        break;

    case 6: /* 土壤湿度 */
        name = NM_SHUM; nlen = 4;
        valid = g_env.soil_valid;
        vlen = valid ? fmt_f1(val, g_env.soil_humi) : fmt_na(val);
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

#if SCREEN_TEMP_UNIT_CJK8
    /* 温度/土温的 ℃ 走 8px 小字库 O001，其余单位仍走英文字库 E000 */
    if (idx == 0 || idx == 5) uni_font = SCREEN_FONT_NAME_CJK8;
#endif

    /* 超宽保护：>6 字符显示 ERR，不截断前后数字 */
    if (vlen > SCREEN_VAL_MAX) { strcpy(val, "ERR"); vlen = 3; }
    if (ulen > SCREEN_VAL_MAX) { strcpy(uni, "ERR"); ulen = 3; }

    {
        Screen_Area_t areas[3];
        uint16_t flen;

        /* 名称区：中文用中文字库，纯 ASCII（CO2）用英文字库 */
        areas[0].x = AREA_NAME_X; areas[0].y = AREA_NAME_Y;
        areas[0].w = AREA_NAME_W; areas[0].h = AREA_NAME_H;
        areas[0].font = (name[0] < 0x80) ? SCREEN_FONT_NAME_EN : SCREEN_FONT_NAME_CN;
        areas[0].text = name; areas[0].len = nlen;

        /* 数值区 */
        areas[1].x = AREA_VAL_X; areas[1].y = AREA_VAL_Y;
        areas[1].w = AREA_VAL_W; areas[1].h = AREA_VAL_H;
        areas[1].font = SCREEN_FONT_NAME_EN;
        areas[1].text = (const uint8_t *)val; areas[1].len = vlen;

        /* 单位区 */
        areas[2].x = AREA_UNI_X; areas[2].y = AREA_UNI_Y;
        areas[2].w = AREA_UNI_W; areas[2].h = AREA_UNI_H;
        areas[2].font = uni_font;
        areas[2].text = (const uint8_t *)uni; areas[2].len = ulen;

        flen = Screen_BuildFrame(areas, 3, frame);
        Screen_Send(frame, flen);
    }
}
