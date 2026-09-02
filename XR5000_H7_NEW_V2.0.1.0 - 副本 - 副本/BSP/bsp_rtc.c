#include "bsp_rtc.h"

BM8563_TimeTypeDef SystemTime = 
{
	.seconds = 30,
	.minutes = 40,
	.hours   = 8,
	.day     = 3,
	.weekday = 1,
	.month   = 1,
	.year    = 2026
};

// 微秒级延迟函数(根据主频调整)
static void SOFT_I2C_Delay(void)
{
    uint32_t i = 20;  // 根据实际时钟频率调整
    while(i--);
}

// 产生I2C起始信号
void SOFT_I2C_Start(void)
{
    // SDA高->低，SCL高
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET);
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
    SOFT_I2C_Delay();
}

// 产生I2C停止信号
void SOFT_I2C_Stop(void)
{
    // SDA低->高，SCL高
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET);
    SOFT_I2C_Delay();
}

// 写一个字节并返回ACK状态
uint8_t SOFT_I2C_WriteByte(uint8_t byte)
{
    uint8_t i, ack;
    
    for(i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, (byte & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        SOFT_I2C_Delay();
        
        HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
        SOFT_I2C_Delay();
        
        HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
        SOFT_I2C_Delay();
        
        byte <<= 1;
    }
    
    // 读取ACK
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET); // 释放SDA
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
    SOFT_I2C_Delay();
    
    ack = HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN);
    
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
    SOFT_I2C_Delay();
    
    return ack ? 1 : 0; // 0表示ACK, 1表示NACK
}

// 读一个字节并发送ACK/NACK
uint8_t SOFT_I2C_ReadByte(uint8_t ack)
{
    uint8_t i, byte = 0;

    for(i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
        SOFT_I2C_Delay();
        
        byte <<= 1;
        if(HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN))
            byte |= 0x01;
            
        HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
        SOFT_I2C_Delay();
    }
    
    // 发送ACK/NACK
    // 恢复SDA为输出模式

    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, ack ? GPIO_PIN_SET : GPIO_PIN_RESET);
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
    SOFT_I2C_Delay();
    
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET); // 释放SDA
    
    return byte;
}

// BCD转十进制
static uint8_t BCD2DEC(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// 十进制转BCD
static uint8_t DEC2BCD(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

// 初始化BM8563
uint8_t BM8563_Soft_I2C_Init(void)
{
    uint8_t data[2];
    
    SOFT_I2C_Start();
    if(SOFT_I2C_WriteByte(BM8563_I2C_ADDR << 1)) // 写地址
    {
        SOFT_I2C_Stop();
        return 1;
    }
    
    // 设置控制寄存器1: 正常模式，24小时制
    data[0] = BM8563_REG_CTRL1;
    data[1] = 0x00;
    
    if(SOFT_I2C_WriteByte(data[0]) || SOFT_I2C_WriteByte(data[1]))
    {
        SOFT_I2C_Stop();
        return 1;
    }
    
    SOFT_I2C_Stop();
    return 0;
}

// 设置时间
void BM8563_Soft_I2C_SetTime(BM8563_TimeTypeDef *time)
{
    SOFT_I2C_Start();
    SOFT_I2C_WriteByte(BM8563_I2C_ADDR << 1); // 写地址
    
    SOFT_I2C_WriteByte(BM8563_REG_SEC);
    SOFT_I2C_WriteByte(DEC2BCD(time->seconds));
    SOFT_I2C_WriteByte(DEC2BCD(time->minutes));
    SOFT_I2C_WriteByte(DEC2BCD(time->hours));
    SOFT_I2C_WriteByte(DEC2BCD(time->day));
    SOFT_I2C_WriteByte(DEC2BCD(time->weekday));
    SOFT_I2C_WriteByte(DEC2BCD(time->month));
    SOFT_I2C_WriteByte(DEC2BCD(time->year % 100));
    
    SOFT_I2C_Stop();
}

// 获取时间
void BM8563_Soft_I2C_GetTime(BM8563_TimeTypeDef *time)
{
    uint8_t data[7];
    
    // 写寄存器地址
    SOFT_I2C_Start();
    SOFT_I2C_WriteByte(BM8563_I2C_ADDR << 1);
    SOFT_I2C_WriteByte(BM8563_REG_SEC);
    SOFT_I2C_Stop();
    
    // 读取7个字节数据
    SOFT_I2C_Start();
    SOFT_I2C_WriteByte((BM8563_I2C_ADDR << 1) | 0x01); // 读地址
    
    for(uint8_t i = 0; i < 6; i++)
    {
        data[i] = SOFT_I2C_ReadByte(0); // 发送ACK
    }
    data[6] = SOFT_I2C_ReadByte(1);     // 最后一个字节发送NACK
    
    SOFT_I2C_Stop();
    
    time->seconds = BCD2DEC(data[0] & 0x7F); // 去掉VL标志位
    time->minutes = BCD2DEC(data[1] & 0x7F);
    time->hours = BCD2DEC(data[2] & 0x3F);    // 24小时制
    time->day = BCD2DEC(data[3] & 0x3F);
    time->weekday = BCD2DEC(data[4] & 0x07);
    time->month = BCD2DEC(data[5] & 0x1F);
    
    // 处理年份(假设2000-2099)
//    uint8_t year = BCD2DEC(data[6]);
//    time->year = year >= 70 ? 1900 + year : 2000 + year;
		// 直接赋值后两位
    time->year = BCD2DEC(data[6]);
}

extern uint8_t secs,years,months,weeks,days,hours,minutes;
void getBM8563TimeToSystemTime(void)
{
	uint8_t data[7];

	// 写寄存器地址
	SOFT_I2C_Start();
	SOFT_I2C_WriteByte(BM8563_I2C_ADDR << 1);
	SOFT_I2C_WriteByte(BM8563_REG_SEC);
	SOFT_I2C_Stop();

	// 读取7个字节数据
	SOFT_I2C_Start();
	SOFT_I2C_WriteByte((BM8563_I2C_ADDR << 1) | 0x01); // 读地址

	for(uint8_t i = 0; i < 6; i++)
	{
		data[i] = SOFT_I2C_ReadByte(0); // 发送ACK
	}
	data[6] = SOFT_I2C_ReadByte(1);     // 最后一个字节发送NACK

	SOFT_I2C_Stop();

	secs    = BCD2DEC(data[0] & 0x7F); // 去掉VL标志位
	minutes = BCD2DEC(data[1] & 0x7F);
	hours   = BCD2DEC(data[2] & 0x3F);    // 24小时制
	days    = BCD2DEC(data[3] & 0x3F);
	weeks   = BCD2DEC(data[4] & 0x07);
	months  = BCD2DEC(data[5] & 0x1F);

	// 处理年份(假设2000-2099)
	
	years = BCD2DEC(data[6]);
//	uint8_t year = BCD2DEC(data[6]);
//	year = year >= 70 ? 1900 + year : 2000 + year;
}
