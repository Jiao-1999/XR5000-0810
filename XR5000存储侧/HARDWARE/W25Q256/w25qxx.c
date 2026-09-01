/**
 * @file    w25qxx.c
 * @brief   W25Q256 SPI Flash 驱动实现 (适配STM32F10x标准外设库)
 * @details 移植自XR5000_H7_NEW_V2.0.0.3/BSP/w25qxx.c
 *          适配要点:
 *          - HAL库 -> STM32F10x标准外设库
 *          - SPI2初始化使用SPI_Init/SPI_Cmd
 *          - GPIO使用GPIO_Init/GPIO_SetBits/GPIO_ResetBits
 *          - 片选PB12软件控制
 *          W25Q256容量32MB, 4字节地址模式
 */
#include "w25qxx.h"
#include "delay.h"

/*==============================================================
 * 全局变量
 *============================================================*/
uint16_t W25QXX_TYPE = W25Q256;  /* 默认是W25Q256 */

/*==============================================================
 * SPI2 底层读写一个字节
 *============================================================*/
uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{
    uint8_t retry = 0;
    /* 等待发送缓冲区空 */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
        retry++;
        if (retry > 200) return 0;
    }
    /* 写入发送数据 */
    SPI_I2S_SendData(SPI2, TxData);

    retry = 0;
    /* 等待接收缓冲区非空 */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
        retry++;
        if (retry > 200) return 0;
    }
    /* 返回收到的数据 */
    return SPI_I2S_ReceiveData(SPI2);
}

/*==============================================================
 * 片选控制
 *============================================================*/
void W25QXX_CS(uint8_t a)
{
    if (a == 0)
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);  /* 选中(低电平) */
    else
        GPIO_SetBits(GPIOB, GPIO_Pin_12);    /* 释放(高电平) */
}

/*==============================================================
 * 初始化SPI2和W25QXX
 * 硬件连接:
 *   PB12 -> CS  (软件控制, 推挽输出)
 *   PB13 -> SCK (SPI2_SCK, 复用推挽)
 *   PB14 -> MISO(SPI2_MISO, 浮空输入)
 *   PB15 -> MOSI(SPI2_MOSI, 复用推挽)
 *============================================================*/
uint8_t W25QXX_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    uint8_t temp;

    /* 使能GPIOB和SPI2时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* PB12 CS - 推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);  /* 默认释放片选 */

    /* PB13 SCK, PB15 MOSI - 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* PB14 MISO - 浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* SPI2配置 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode      = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize  = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL      = SPI_CPOL_High;      /* 时钟极性: 空闲高 */
    SPI_InitStructure.SPI_CPHA      = SPI_CPHA_2Edge;     /* 时钟相位: 第二个跳变沿采样 */
    SPI_InitStructure.SPI_NSS       = SPI_NSS_Soft;       /* 软件NSS */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  /* 72MHz/2=36MHz */
    SPI_InitStructure.SPI_FirstBit  = SPI_FirstBit_MSB;   /* 高位在前 */
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    /* 使能SPI2 */
    SPI_Cmd(SPI2, ENABLE);

    /* 片选默认释放 */
    W25QXX_CS(1);

    /* 读取芯片ID */
    W25QXX_TYPE = W25QXX_ReadID();

    /* W25Q256需要设置为4字节地址模式 */
    if (W25QXX_TYPE == W25Q256)
    {
        temp = W25QXX_ReadSR(3);  /* 读取状态寄存器3, 判断地址模式 */
        if ((temp & 0x01) == 0)   /* 如果不是4字节地址模式 */
        {
            W25QXX_CS(0);
            SPI2_ReadWriteByte(W25X_Enable4ByteAddr);  /* 发送进入4字节地址模式指令 */
            W25QXX_CS(1);
        }
    }

    /* 检查ID是否在已知型号列表中 */
    if (W25QXX_TYPE == W25Q256 || W25QXX_TYPE == W25Q128 || W25QXX_TYPE == W25Q64
     || W25QXX_TYPE == W25Q32  || W25QXX_TYPE == W25Q16 || W25QXX_TYPE == W25Q80)
        return 0;  /* 识别成功 */
    else
        return 1;  /* 识别失败 */
}

/*==============================================================
 * 读取状态寄存器
 * regno: 1~3
 *============================================================*/
uint8_t W25QXX_ReadSR(uint8_t regno)
{
    uint8_t byte = 0, command = 0;

    switch (regno)
    {
        case 1:  command = W25X_ReadStatusReg1; break;
        case 2:  command = W25X_ReadStatusReg2; break;
        case 3:  command = W25X_ReadStatusReg3; break;
        default: command = W25X_ReadStatusReg1; break;
    }

    W25QXX_CS(0);
    SPI2_ReadWriteByte(command);        /* 发送读取状态寄存器命令 */
    byte = SPI2_ReadWriteByte(0XFF);    /* 读取一个字节 */
    W25QXX_CS(1);

    return byte;
}

/*==============================================================
 * 写状态寄存器
 *============================================================*/
void W25QXX_Write_SR(uint8_t regno, uint8_t sr)
{
    uint8_t command = 0;

    switch (regno)
    {
        case 1:  command = W25X_WriteStatusReg1; break;
        case 2:  command = W25X_WriteStatusReg2; break;
        case 3:  command = W25X_WriteStatusReg3; break;
        default: command = W25X_WriteStatusReg1; break;
    }

    W25QXX_CS(0);
    SPI2_ReadWriteByte(command);  /* 发送写状态寄存器命令 */
    SPI2_ReadWriteByte(sr);      /* 写入一个字节 */
    W25QXX_CS(1);
}

/*==============================================================
 * 写使能
 *============================================================*/
void W25QXX_Write_Enable(void)
{
    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_WriteEnable);
    W25QXX_CS(1);
}

/*==============================================================
 * 写禁止
 *============================================================*/
void W25QXX_Write_Disable(void)
{
    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_WriteDisable);
    W25QXX_CS(1);
}

/*==============================================================
 * 读取芯片ID
 * 高8位是厂商代号, 低8位是容量大小
 * 0XEF18 = W25Q256
 *============================================================*/
uint16_t W25QXX_ReadID(void)
{
    uint16_t Temp = 0;

    W25QXX_CS(0);
    SPI2_ReadWriteByte(0x90);  /* 发送读取ID命令 */
    SPI2_ReadWriteByte(0x00);
    SPI2_ReadWriteByte(0x00);
    SPI2_ReadWriteByte(0x00);
    Temp |= SPI2_ReadWriteByte(0xFF) << 8;
    Temp |= SPI2_ReadWriteByte(0xFF);
    W25QXX_CS(1);

    return Temp;
}

/*==============================================================
 * 读取SPI FLASH
 * pBuffer: 数据存储区
 * ReadAddr: 开始读取的地址
 * NumByteToRead: 要读取的字节数(最大65535)
 *============================================================*/
void W25QXX_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint16_t i;

    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_ReadData);  /* 发送读取命令 */

    if (W25QXX_TYPE == W25Q256)  /* W25Q256使用4字节地址 */
    {
        SPI2_ReadWriteByte((uint8_t)(ReadAddr >> 24));
    }
    SPI2_ReadWriteByte((uint8_t)(ReadAddr >> 16));  /* 发送24bit地址 */
    SPI2_ReadWriteByte((uint8_t)(ReadAddr >> 8));
    SPI2_ReadWriteByte((uint8_t)ReadAddr);

    for (i = 0; i < NumByteToRead; i++)
    {
        pBuffer[i] = SPI2_ReadWriteByte(0XFF);  /* 循环读数 */
    }
    W25QXX_CS(1);
}

/*==============================================================
 * 写一页(最多256字节)
 * pBuffer: 数据存储区
 * WriteAddr: 开始写入的地址
 * NumByteToWrite: 要写入的字节数(最大256), 不应超过该页剩余字节数
 *============================================================*/
void W25QXX_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t i;

    W25QXX_Write_Enable();
    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_PageProgram);  /* 发送写页命令 */

    if (W25QXX_TYPE == W25Q256)  /* W25Q256使用4字节地址 */
    {
        SPI2_ReadWriteByte((uint8_t)(WriteAddr >> 24));
    }
    SPI2_ReadWriteByte((uint8_t)(WriteAddr >> 16));
    SPI2_ReadWriteByte((uint8_t)(WriteAddr >> 8));
    SPI2_ReadWriteByte((uint8_t)WriteAddr);

    for (i = 0; i < NumByteToWrite; i++)
    {
        SPI2_ReadWriteByte(pBuffer[i]);  /* 循环写数 */
    }
    W25QXX_CS(1);
    W25QXX_Wait_Busy();  /* 等待写入结束 */
}

/*==============================================================
 * 无校验写SPI FLASH (具有自动换页功能)
 * 必须确保所写地址范围内的数据全为0XFF
 *============================================================*/
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;

    pageremain = 256 - WriteAddr % 256;  /* 单页剩余字节数 */

    if (NumByteToWrite <= pageremain)
    {
        pageremain = NumByteToWrite;
    }

    while (1)
    {
        W25QXX_Write_Page(pBuffer, WriteAddr, pageremain);

        if (NumByteToWrite == pageremain)
        {
            break;  /* 写入结束 */
        }
        else  /* NumByteToWrite > pageremain */
        {
            pBuffer        += pageremain;
            WriteAddr      += pageremain;
            NumByteToWrite -= pageremain;

            if (NumByteToWrite > 256)
            {
                pageremain = 256;
            }
            else
            {
                pageremain = NumByteToWrite;
            }
        }
    }
}

/*==============================================================
 * 写SPI FLASH (带擦除操作)
 * pBuffer: 数据存储区
 * WriteAddr: 开始写入的地址
 * NumByteToWrite: 要写入的字节数(最大65535)
 *============================================================*/
uint8_t W25QXX_BUFFER[4096];  /* 扇区缓存 */

void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t *W25QXX_BUF = W25QXX_BUFFER;

    secpos    = WriteAddr / 4096;  /* 扇区号 */
    secoff    = WriteAddr % 4096;  /* 扇区内偏移 */
    secremain = 4096 - secoff;     /* 扇区剩余空间 */

    if (NumByteToWrite <= secremain)
    {
        secremain = NumByteToWrite;
    }

    while (1)
    {
        W25QXX_Read(W25QXX_BUF, secpos * 4096, 4096);  /* 读出整个扇区 */

        for (i = 0; i < secremain; i++)  /* 校验数据 */
        {
            if (W25QXX_BUF[secoff + i] != 0XFF)  /* 有非0xFF数据,需要擦除 */
            {
                break;
            }
        }

        if (i < secremain)  /* 需要擦除 */
        {
            W25QXX_Erase_Sector(secpos);  /* 擦除这个扇区 */
            for (i = 0; i < secremain; i++)  /* 复制新数据 */
            {
                W25QXX_BUF[i + secoff] = pBuffer[i];
            }
            W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096);  /* 写入整个扇区 */
        }
        else
        {
            W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain);  /* 直接写入 */
        }

        if (NumByteToWrite == secremain)
        {
            break;  /* 写入结束 */
        }
        else  /* 写入未结束 */
        {
            secpos++;
            secoff = 0;
            pBuffer        += secremain;
            WriteAddr      += secremain;
            NumByteToWrite -= secremain;

            if (NumByteToWrite > 4096)
            {
                secremain = 4096;
            }
            else
            {
                secremain = NumByteToWrite;
            }
        }
    }
}

/*==============================================================
 * 擦除整个芯片
 *============================================================*/
void W25QXX_Erase_Chip(void)
{
    W25QXX_Write_Enable();
    W25QXX_Wait_Busy();
    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_ChipErase);
    W25QXX_CS(1);
    W25QXX_Wait_Busy();
}

/*==============================================================
 * 擦除一个扇区
 * Dst_Addr: 扇区号 (实际地址 = Dst_Addr * 4096)
 *============================================================*/
void W25QXX_Erase_Sector(uint32_t Dst_Addr)
{
    Dst_Addr *= 4096;
    W25QXX_Write_Enable();
    W25QXX_Wait_Busy();
    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_SectorErase);

    if (W25QXX_TYPE == W25Q256)  /* W25Q256使用4字节地址 */
    {
        SPI2_ReadWriteByte((uint8_t)(Dst_Addr >> 24));
    }
    SPI2_ReadWriteByte((uint8_t)(Dst_Addr >> 16));
    SPI2_ReadWriteByte((uint8_t)(Dst_Addr >> 8));
    SPI2_ReadWriteByte((uint8_t)Dst_Addr);
    W25QXX_CS(1);
    W25QXX_Wait_Busy();
}

/*==============================================================
 * 等待空闲
 *============================================================*/
void W25QXX_Wait_Busy(void)
{
    while ((W25QXX_ReadSR(1) & 0x01) == 0x01);  /* 等待BUSY位清空 */
}

/*==============================================================
 * 进入掉电模式
 *============================================================*/
void W25QXX_PowerDown(void)
{
    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_PowerDown);
    W25QXX_CS(1);
    delay_us(3);
}

/*==============================================================
 * 唤醒
 *============================================================*/
void W25QXX_WAKEUP(void)
{
    W25QXX_CS(0);
    SPI2_ReadWriteByte(W25X_ReleasePowerDown);
    W25QXX_CS(1);
    delay_us(3);
}
