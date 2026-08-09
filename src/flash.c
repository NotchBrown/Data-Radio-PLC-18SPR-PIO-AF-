/*
 * flash.c - GD25Q64 SPI NOR Flash 驱动实现
 *
 * 指令: 读 0x03 / JEDEC ID 0x9F / 写使能 0x06 / 读状态 0x05 /
 *       页编程 0x02 / 扇区擦除(4KB) 0x20 / 整片擦除 0xC7
 *
 * 注意: flash 大块读写不开中断(ISR 不得访问 SPI); 模式/CS 由 spi 模块管理
 */
#include "flash.h"
#include "spi.h"
#include <stddef.h>   /* NULL */

/* ---- 指令码 ---- */
#define CMD_READ_DATA      0x03   /* 读数据 */
#define CMD_READ_ID        0x9F   /* 读 JEDEC ID */
#define CMD_WRITE_ENABLE   0x06   /* 写使能 */
#define CMD_WRITE_DISABLE  0x04   /* 写禁止 */
#define CMD_READ_STATUS    0x05   /* 读状态寄存器1 */
#define CMD_PAGE_PROGRAM   0x02   /* 页编程 */
#define CMD_SECTOR_ERASE   0x20   /* 扇区擦除 4KB */
#define CMD_CHIP_ERASE     0xC7   /* 整片擦除 */

#define STATUS_WIP         0x01   /* 状态寄存器 bit0: 写/擦除忙 */

/* 厂商码 (GigaDevice) */
#define MFR_GIGADEVICE     0xC8

/* 内部: 发送 3 字节 24bit 地址 */
static void flash_send_addr(uint32_t addr)
{
    spi_transfer((addr >> 16) & 0xFF);
    spi_transfer((addr >> 8) & 0xFF);
    spi_transfer(addr & 0xFF);
}

/* ==================== 初始化 ==================== */
uint8_t flash_init(void)
{
    uint8_t id[3];
    flash_read_id(id);
    /* GD25Q64: 厂商 0xC8; 型号/容量由 id[1..2] 判断 (64Mbit 常见 0x40/0x17) */
    return (id[0] == MFR_GIGADEVICE) ? 0 : 1;
}

void flash_read_id(uint8_t *id)
{
    spi_begin(SPI_SLAVE_FLASH);
    spi_transfer(CMD_READ_ID);
    id[0] = spi_transfer(0xFF);
    id[1] = spi_transfer(0xFF);
    id[2] = spi_transfer(0xFF);
    spi_end(SPI_SLAVE_FLASH);
}

/* ==================== 状态/使能 ==================== */
void flash_wait_busy(void)
{
    uint8_t st;
    do {
        spi_begin(SPI_SLAVE_FLASH);
        spi_transfer(CMD_READ_STATUS);
        st = spi_transfer(0xFF);
        spi_end(SPI_SLAVE_FLASH);
    } while (st & STATUS_WIP);
}

static void flash_write_enable(void)
{
    spi_begin(SPI_SLAVE_FLASH);
    spi_transfer(CMD_WRITE_ENABLE);
    spi_end(SPI_SLAVE_FLASH);
}

/* ==================== 读 ==================== */
void flash_read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    spi_begin(SPI_SLAVE_FLASH);
    spi_transfer(CMD_READ_DATA);
    flash_send_addr(addr);
    spi_transfer_n(NULL, buf, len);
    spi_end(SPI_SLAVE_FLASH);
}

/* ==================== 写/擦除 ==================== */
void flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    flash_write_enable();
    spi_begin(SPI_SLAVE_FLASH);
    spi_transfer(CMD_PAGE_PROGRAM);
    flash_send_addr(addr);
    spi_transfer_n(data, NULL, len);
    spi_end(SPI_SLAVE_FLASH);
    flash_wait_busy();
}

void flash_sector_erase(uint32_t addr)
{
    flash_write_enable();
    spi_begin(SPI_SLAVE_FLASH);
    spi_transfer(CMD_SECTOR_ERASE);
    flash_send_addr(addr);
    spi_end(SPI_SLAVE_FLASH);
    flash_wait_busy();
}

void flash_chip_erase(void)
{
    flash_write_enable();
    spi_begin(SPI_SLAVE_FLASH);
    spi_transfer(CMD_CHIP_ERASE);
    spi_end(SPI_SLAVE_FLASH);
    flash_wait_busy();
}
