/**
  ******************************************************************************
  * @file    screen.c
  * @brief   BX-5K1 LED大屏驱动：组滚动显示帧并发送
  *
  * 协议要点（调试记录验证通过）：
  *   帧头8×0xA5，帧尾0x5A
  *   包头14字节：DstAddr(2)+SrcAddr(2)+保留(3)+条码(1)+校验(1)+
  *              动态优先(1)+通配(1)+版本(1)+DataLen(2)
  *   CRC16: 初值0x0000, poly=0xA001, 低字节先发
  *   区域DisplayMode=03左移, 速度09, ProcessMode=01清区域重发
  ******************************************************************************
  */
#include "screen.h"
#include <string.h>
#include <stdio.h>

/* LED CRC16: 初值0, poly=0xA001 */
static uint16_t Screen_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        uint8_t j;
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

/*
 * 组滚动显示帧
 *   text     : GB2312编码文本
 *   text_len : 文本字节数
 *   speed    : 滚动速度(01最快~0F最慢, 用09)
 *   out      : 输出缓冲区
 * 返回: 帧总长度
 */
/* Maximum text supplied by the application; allow worst-case wire escaping. */
#define SCREEN_TEXT_MAX 32U
#define SCREEN_RAW_MAX  (14U + 36U + SCREEN_TEXT_MAX + 2U)
#define SCREEN_WIRE_MAX (8U + 2U * SCREEN_RAW_MAX + 1U)

static uint16_t Build_ScrollFrame(const uint8_t *text, uint16_t text_len,
                                  uint8_t speed, uint8_t *out)
{
    uint8_t raw[SCREEN_RAW_MAX];
    uint8_t *p = raw;
    uint16_t crc, n, i, wire_len = 0;
    /* AreaType, X, Y, W, H, ID, spacing, RunMode, timeout, sound,
       extension, alignment, SingleLine, NewLine, effect, exit, speed, stay. */
    uint8_t region[] = {
        0x00, 0x00, 0x80, 0x00, 0x00, 0x40, 0x80, 0x10, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0x03, 0x00, 0x09, 0x00
    };
    const uint8_t header[] = {
        0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0xFE, 0x02
    };
    const uint8_t command[] = {0xA3, 0x06, 0x00, 0x01, 0x00, 0x00, 0x01};

    if (out == NULL || text == NULL || text_len == 0 || text_len > SCREEN_TEXT_MAX)
        return 0;

    memcpy(p, header, sizeof(header)); p += sizeof(header);
    *p++ = (uint8_t)(36U + text_len); *p++ = 0;
    memcpy(p, command, sizeof(command)); p += sizeof(command);
    *p++ = (uint8_t)(27U + text_len); *p++ = 0;
    region[21] = speed;
    memcpy(p, region, sizeof(region)); p += sizeof(region);
    *p++ = (uint8_t)text_len; *p++ = 0; *p++ = 0; *p++ = 0;
    memcpy(p, text, text_len); p += text_len;

    crc = Screen_CRC16(raw, (uint16_t)(p - raw));
    *p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
    n = (uint16_t)(p - raw);

    memset(out, 0xA5, 8); wire_len = 8;
    for (i = 0; i < n; ++i) {
        switch (raw[i]) {
        case 0xA5: out[wire_len++] = 0xA6; out[wire_len++] = 0x02; break;
        case 0xA6: out[wire_len++] = 0xA6; out[wire_len++] = 0x01; break;
        case 0x5A: out[wire_len++] = 0x5B; out[wire_len++] = 0x02; break;
        case 0x5B: out[wire_len++] = 0x5B; out[wire_len++] = 0x01; break;
        default: out[wire_len++] = raw[i]; break;
        }
    }
    out[wire_len++] = 0x5A;
    return wire_len;
}

/* HAL_UART_Transmit waits for TC on success; its timeout bounds failure. */
static void Screen_Send(const uint8_t *data, uint16_t len)
{
    if (len == 0) return;
    HAL_GPIO_WritePin(T3_EN_GPIO_Port, T3_EN_Pin, GPIO_PIN_SET);
    (void)HAL_UART_Transmit(&huart5, (uint8_t *)data, len, 300);
    HAL_GPIO_WritePin(T3_EN_GPIO_Port, T3_EN_Pin, GPIO_PIN_RESET);
}

void Screen_ShowScroll(const uint8_t *text, uint16_t len)
{
    static uint8_t frame[SCREEN_WIRE_MAX];
    uint16_t flen = Build_ScrollFrame(text, len, 0x09, frame);
    Screen_Send(frame, flen);
}

/* 格式化温度文本: "温度:21.5℃" */
void Screen_FormatTemp(uint8_t *out, uint16_t *out_len)
{
    /* "温度" GB2312 = CE C2 B6 C8 */
    out[0] = 0xCE; out[1] = 0xC2;
    out[2] = 0xB6; out[3] = 0xC8;
    out[4] = 0x3A;  /* ':' */
    if (g_env.ble_box_valid)
        *out_len = (uint16_t)sprintf((char*)out + 5, "%d.%d",
                                     (int)(g_env.air_temp),
                                     (int)(g_env.air_temp * 10) % 10) + 5;
    else
        *out_len = (uint16_t)sprintf((char*)out + 5, "--.-") + 5;
    /* "℃" = A1 E6 */
    out[(*out_len)++] = 0xA1;
    out[(*out_len)++] = 0xE6;
}

/* 格式化湿度文本: "湿度:54.5%" */
void Screen_FormatHumi(uint8_t *out, uint16_t *out_len)
{
    out[0] = 0xCA; out[1] = 0xAA;
    out[2] = 0xB6; out[3] = 0xC8;
    out[4] = 0x3A;
    if (g_env.ble_box_valid)
        *out_len = (uint16_t)sprintf((char*)out + 5, "%d.%d",
                                     (int)(g_env.air_humi),
                                     (int)(g_env.air_humi * 10) % 10) + 5;
    else
        *out_len = (uint16_t)sprintf((char*)out + 5, "--.-") + 5;
    out[(*out_len)++] = 0x25;  /* '%' */
}

/* 格式化CO2文本: "CO2:603ppm" */
void Screen_FormatCO2(uint8_t *out, uint16_t *out_len)
{
    out[0] = 0x43; out[1] = 0x4F; out[2] = 0x32;  /* "CO2" */
    out[3] = 0x3A;  /* ':' */
    if (g_env.ble_box_valid)
        *out_len = (uint16_t)sprintf((char*)out + 4, "%d", (int)g_env.co2) + 4;
    else
        *out_len = (uint16_t)sprintf((char*)out + 4, "---") + 4;
    out[(*out_len)++] = 0x70;  /* p */
    out[(*out_len)++] = 0x70;  /* p */
    out[(*out_len)++] = 0x6D;  /* m */
}

/* 格式化光照文本: "光照:1375lx" */
void Screen_FormatLux(uint8_t *out, uint16_t *out_len)
{
    out[0] = 0xB9; out[1] = 0xE2;  /* 光 */
    out[2] = 0xD5; out[3] = 0xD5;  /* 照 */
    out[4] = 0x3A;
    if (g_env.ble_box_valid)
        /* lux 为 uint32，按无符号打印，避免大光照时转 int 变负 */
        *out_len = (uint16_t)sprintf((char*)out + 5, "%lu", (unsigned long)g_env.lux) + 5;
    else
        *out_len = (uint16_t)sprintf((char*)out + 5, "----") + 5;
    out[(*out_len)++] = 0x4C;  /* L */
    out[(*out_len)++] = 0x75;  /* u */
    out[(*out_len)++] = 0x78;  /* x */
}

/* 格式化气压文本: "气压:101.6kPa" */
void Screen_FormatPressure(uint8_t *out, uint16_t *out_len)
{
    out[0] = 0xC6; out[1] = 0xF8;  /* 气 */
    out[2] = 0xD1; out[3] = 0xB9;  /* 压 */
    out[4] = 0x3A;
    if (g_env.ble_box_valid)
        *out_len = (uint16_t)sprintf((char*)out + 5, "%d.%d",
                                     (int)(g_env.pressure),
                                     (int)(g_env.pressure * 10) % 10) + 5;
    else
        *out_len = (uint16_t)sprintf((char*)out + 5, "--.-") + 5;
    out[(*out_len)++] = 0x6B;  /* k */
    out[(*out_len)++] = 0x50;  /* P */
    out[(*out_len)++] = 0x61;  /* a */
}

/* 格式化土壤温度: "土温:20.6℃" */
void Screen_FormatSoilTemp(uint8_t *out, uint16_t *out_len)
{
    out[0] = 0xCD; out[1] = 0xC1;  /* 土 */
    out[2] = 0xCE; out[3] = 0xC2;  /* 温 */
    out[4] = 0x3A;
    if (g_env.soil_valid)
        *out_len = (uint16_t)sprintf((char*)out + 5, "%d.%d",
                                     (int)(g_env.soil_temp),
                                     (int)(g_env.soil_temp * 10) % 10) + 5;
    else
        *out_len = (uint16_t)sprintf((char*)out + 5, "--.-") + 5;
    out[(*out_len)++] = 0xA1;
    out[(*out_len)++] = 0xE6;  /* ℃ */
}

/* 格式化土壤湿度: "土湿:45%" */
void Screen_FormatSoilHumi(uint8_t *out, uint16_t *out_len)
{
    out[0] = 0xCD; out[1] = 0xC1;  /* 土 */
    out[2] = 0xCA; out[3] = 0xAA;  /* 湿 */
    out[4] = 0x3A;
    if (g_env.soil_valid)
        *out_len = (uint16_t)sprintf((char*)out + 5, "%d", (int)g_env.soil_humi) + 5;
    else
        *out_len = (uint16_t)sprintf((char*)out + 5, "--") + 5;
    out[(*out_len)++] = 0x25;  /* % */
}

/* 格式化土壤EC: "EC:1200us" */
void Screen_FormatEC(uint8_t *out, uint16_t *out_len)
{
    out[0] = 0x45; out[1] = 0x43;  /* "EC" */
    out[2] = 0x3A;  /* ':' */
    if (g_env.soil_valid)
        *out_len = (uint16_t)sprintf((char*)out + 3, "%d", (int)g_env.soil_ec) + 3;
    else
        *out_len = (uint16_t)sprintf((char*)out + 3, "----") + 3;
    out[(*out_len)++] = 0x75;  /* u */
    out[(*out_len)++] = 0x73;  /* s */
    out[(*out_len)++] = 0x2F;  /* / */
    out[(*out_len)++] = 0x63;  /* c */
    out[(*out_len)++] = 0x6D;  /* m */
}
