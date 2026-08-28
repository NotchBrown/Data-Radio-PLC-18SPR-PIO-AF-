/*
 * dbg.h - UART3 调试打印 (仅 RF_DEBUG 构建启用调用点)
 *
 * 用 UART3 TX (115200) 输出调试文本, 供诊断 RF 状态 (SYS 灯/发送/接收)。
 * 打印函数本身常驻, 但调用点用 #ifdef RF_DEBUG 包裹, 非 debug 构建零开销。
 * 注意: 打印会与 UART3 协议回复共享 TX, 诊断时请用串口监视器观察,
 *       不要同时跑协议自动化测试 (或脚本需按帧解析忽略文本行)。
 */
#ifndef __DBG_H
#define __DBG_H

#include <stdint.h>

/* 初始化 (须在 uart3_init 后调用); 之前打印无效 */
void dbg_init(void);

/* 打印字符串 (遇 '\0' 结束) */
void dbg_puts(const char *s);

/* 打印 8bit 十六进制 (2 位) */
void dbg_hex8(uint8_t v);

/* 打印 16bit 十进制 (无符号) */
void dbg_dec(uint16_t v);

/* 换行 CRLF */
void dbg_newline(void);

/* ===== 卡死位置码 (存 EEPROM 空闲区) =====
 * 各关键函数写入, 看门狗复位后 boot 打印定位卡死点。
 * 仅 RF_DEBUG 构建写 EEPROM (定位是调试期的事, release 靠看门狗防卡死即可) */
void dbg_pos(uint8_t p);
uint8_t dbg_pos_get(void);

/* ===== 统一调试宏 =====
 * RF_DEBUG 构建: 展开为实际打印/位置码; release(无 RF_DEBUG): 全部展开为空,
 * 编译期自动移除调试代码, 无需每处写 #ifdef。 */
#ifdef RF_DEBUG
  #define DBG_STR(s)   dbg_puts(s)
  #define DBG_DEC(v)   dbg_dec(v)
  #define DBG_HEX8(v)  dbg_hex8(v)
  #define DBG_NL()     dbg_newline()
  #define DBG_POS(p)   dbg_pos(p)
#else
  #define DBG_STR(s)
  #define DBG_DEC(v)
  #define DBG_HEX8(v)
  #define DBG_NL()
  #define DBG_POS(p)
#endif

#endif /* __DBG_H */
