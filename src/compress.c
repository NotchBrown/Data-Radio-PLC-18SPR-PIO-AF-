/*
 * compress.c - A-13 折线压缩实现 (标准 G.711 A-law)
 *
 * 移植自经典 linear2alaw/alaw2linear (业界广泛验证), 已在 Python 验证:
 *   13bit 带符号往返最大误差 64; 10bit 通路最大误差 16
 */
#include "compress.h"

#define ALAW_SEG_SHIFT  4
#define ALAW_QUANT_MASK 0x0F
#define ALAW_SEG_MASK   0x70
#define ALAW_SIGN_BIT   0x80

/* 各段幅度上界 (段 0=最小, 段 7=最大) */
static const int16_t ALAW_SEG_AEND[8] = {
    0x1F, 0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF
};

/* 找幅度所在段 (返回 0..8, 8=超出最大段) */
static int8_t alaw_search(int16_t val)
{
    int8_t i;
    for (i = 0; i < 8; i++)
        if (val <= ALAW_SEG_AEND[i])
            return i;
    return 8;
}

/* ==================== 标准 A-law ==================== */
uint8_t alaw_encode(int16_t pcm)
{
    int16_t seg;
    uint8_t aval, mask;

    pcm >>= 3;   /* 13bit -> 10bit (A-law 量化起始) */

    if (pcm >= 0) {
        mask = 0xD5;   /* 正: 符号位取反后为 0 */
    } else {
        mask = 0x55;
        pcm = (int16_t)(-pcm - 1);
    }

    seg = alaw_search(pcm);
    if (seg >= 8)
        return (uint8_t)(0x7F ^ mask);   /* 溢出饱和 */

    aval = (uint8_t)(seg << ALAW_SEG_SHIFT);
    if (seg < 2)
        aval |= (uint8_t)((pcm >> 1) & ALAW_QUANT_MASK);
    else
        aval |= (uint8_t)((pcm >> seg) & ALAW_QUANT_MASK);
    return (uint8_t)(aval ^ mask);
}

int16_t alaw_decode(uint8_t code)
{
    uint8_t seg;
    int16_t t;

    code ^= 0x55;
    t = (int16_t)((code & ALAW_QUANT_MASK) << 4);
    seg = (code & ALAW_SEG_MASK) >> ALAW_SEG_SHIFT;

    if (seg == 0)
        t += 8;
    else if (seg == 1)
        t += 0x108;
    else {
        t += 0x108;
        t <<= seg - 1;
    }
    return (code & ALAW_SIGN_BIT) ? t : (int16_t)-t;
}

/* ==================== 帧通路 (10bit 单极性) ==================== */
uint8_t a13_compress(uint16_t s10)
{
    /* 10bit 映射到 13bit 正半轴满量程 */
    return alaw_encode((int16_t)((s10 << 2) & 0x0FFF));
}

uint16_t a13_decompress(uint8_t code)
{
    return (uint16_t)(alaw_decode(code) >> 2) & 0x03FF;
}
