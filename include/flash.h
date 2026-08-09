/*
 * flash.h - GD25Q64 SPI NOR Flash 驱动 (64Mbit / 8MB)
 *
 * 硬件: CS=PF0 (spi 模块管理), SPI Mode0 / 6MHz (见 spi.h)
 * 指令集兼容标准 SPI NOR Flash (JEDEC), 与 Winbond 通用
 */
#ifndef __FLASH_H
#define __FLASH_H

#include <stdint.h>

/* 初始化: 读 JEDEC ID 校验 (返回 0 = 连接正常) */
uint8_t flash_init(void);

/* 读 JEDEC ID (3 字节: 厂商 / 型号 / 容量) -> id[3] */
void flash_read_id(uint8_t *id);

/* 读数据: addr(24bit) -> buf, 共 len 字节 */
void flash_read(uint32_t addr, uint8_t *buf, uint16_t len);

/* 页编程: len <= 256, addr 建议按 256B 对齐, 写入前目标区须已擦除 */
void flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len);

/* 扇区擦除: 4KB, addr 按 4KB 对齐 */
void flash_sector_erase(uint32_t addr);

/* 整片擦除 (64Mbit, 较慢, 谨慎使用) */
void flash_chip_erase(void);

/* 等待写/擦除完成 (轮询状态寄存器 WIP 位) */
void flash_wait_busy(void);

#endif /* __FLASH_H */
