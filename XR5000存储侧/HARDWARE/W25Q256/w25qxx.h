/**
 * @file    w25qxx.h
 * @brief   W25Q256 SPI Flash 驱动头文件 (适配STM32F10x标准外设库)
 * @details 硬件连接: SPI2
 *          CS  -> PB12 (软件控制)
 *          CLK -> PB13 (SPI2_SCK)
 *          MISO-> PB14 (SPI2_MISO)
 *          MOSI-> PB15 (SPI2_MOSI)
 *          W25Q256容量32MB, 4字节地址模式
 */
#ifndef __W25QXX_H
#define __W25QXX_H

#include "sys.h"

/*==============================================================
 * 芯片型号定义 (厂商代号EF)
 *============================================================*/
#define W25Q80    0XEF13
#define W25Q16    0XEF14
#define W25Q32    0XEF15
#define W25Q64    0XEF16
#define W25Q128   0XEF17
#define W25Q256   0XEF18
#define W25Q512   0xEF19

/* W25Q256 起始地址 */
#define EX_FLASH_ADD 0x000000

/* 外部全局变量 */
extern uint16_t W25QXX_TYPE;  /* 定义W25QXX芯片型号 */

/*==============================================================
 * W25X 指令表
 *============================================================*/
#define W25X_WriteEnable         0x06
#define W25X_WriteDisable        0x04
#define W25X_ReadStatusReg1      0x05
#define W25X_ReadStatusReg2      0x35
#define W25X_ReadStatusReg3      0x15
#define W25X_WriteStatusReg1     0x01
#define W25X_WriteStatusReg2     0x31
#define W25X_WriteStatusReg3     0x11
#define W25X_ReadData            0x03
#define W25X_FastReadData        0x0B
#define W25X_FastReadDual        0x3B
#define W25X_PageProgram         0x02
#define W25X_BlockErase          0xD8
#define W25X_SectorErase         0x20
#define W25X_ChipErase           0xC7
#define W25X_PowerDown           0xB9
#define W25X_ReleasePowerDown    0xAB
#define W25X_DeviceID            0xAB
#define W25X_ManufactDeviceID    0x90
#define W25X_JedecDeviceID       0x9F
#define W25X_Enable4ByteAddr     0xB7
#define W25X_Exit4ByteAddr       0xE9

/*==============================================================
 * API声明
 *============================================================*/

/* SPI2总线底层读写一个字节 */
uint8_t SPI2_ReadWriteByte(uint8_t TxData);

/* W25QXX片选引脚控制 (0=选中, 其他=释放) */
void W25QXX_CS(uint8_t a);

/* 初始化W25QXX (含SPI2和GPIO配置) 返回0=成功, 1=失败 */
uint8_t W25QXX_Init(void);

/* 读取FLASH ID */
uint16_t W25QXX_ReadID(void);

/* 读取状态寄存器 regno:1~3 */
uint8_t W25QXX_ReadSR(uint8_t regno);

/* 写状态寄存器 */
void W25QXX_Write_SR(uint8_t regno, uint8_t sr);

/* 写使能 */
void W25QXX_Write_Enable(void);

/* 写禁止 */
void W25QXX_Write_Disable(void);

/* 无校验写SPI FLASH (具有自动换页功能, 确保地址不越界) */
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/* 读取SPI FLASH */
void W25QXX_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

/* 写SPI FLASH (带擦除操作) */
void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/* 擦除整个芯片 */
void W25QXX_Erase_Chip(void);

/* 擦除一个扇区 (Dst_Addr: 扇区号) */
void W25QXX_Erase_Sector(uint32_t Dst_Addr);

/* 等待空闲 */
void W25QXX_Wait_Busy(void);

/* 进入掉电模式 */
void W25QXX_PowerDown(void);

/* 唤醒 */
void W25QXX_WAKEUP(void);

#endif /* __W25QXX_H */
