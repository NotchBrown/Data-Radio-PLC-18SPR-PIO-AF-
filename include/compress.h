/*
 * compress.h - A-13 折线压缩 (标准 G.711 A-law) 编解码
 *
 * 帧协议: (字长0, 压缩1) = 8bit A-13 折线压缩 (标准 A-law)
 *
 * 接口分两层:
 *   alaw_encode / alaw_decode : 标准 A-law, 13bit 带符号线性 <-> 8bit 码
 *   a13_compress/decompress   : 帧通路用, 10bit 单极性模拟量 <-> 8bit 码
 *
 * 10bit 映射: s13 = (s10 << 2) & 0x0FFF  (映射到 13bit 正半轴满量程)
 *   恢复: s10 = 解码值 >> 2
 *   往返误差最大 16 LSB (10bit 的 1.5%)
 *   (注: 记忆文件曾提 sample<<3, 但 10bit<<3 最大 8184 超出 A-law 正半轴
 *    0x0FFF, 高端解码会失真, 故改用 <<2 映射; 收发两端同用本模块即可)
 */
#ifndef __COMPRESS_H
#define __COMPRESS_H

#include <stdint.h>

/* 标准 A-law 编码: 13bit 带符号线性 (-4096..4095) -> 8bit 码 */
uint8_t alaw_encode(int16_t pcm13);

/* 标准 A-law 解码: 8bit 码 -> 13bit 带符号线性 */
int16_t alaw_decode(uint8_t code);

/* 帧通路: 10bit 单极性 (0..1023) -> 8bit A-law 码 */
uint8_t a13_compress(uint16_t s10);

/* 帧通路: 8bit A-law 码 -> 10bit 单极性 (0..1023) */
uint16_t a13_decompress(uint8_t code);

#endif /* __COMPRESS_H */
