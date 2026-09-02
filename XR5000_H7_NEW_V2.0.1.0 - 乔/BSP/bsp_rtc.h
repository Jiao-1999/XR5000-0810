#ifndef __BSP_RTC_H
#define __BSP_RTC_H

#include "main.h"

// 用户需要根据实际连接修改这些宏定义
#define SOFT_I2C_SDA_PORT  RTCSDA_GPIO_Port
#define SOFT_I2C_SDA_PIN   RTCSDA_Pin
#define SOFT_I2C_SCL_PORT  RTCSCL_GPIO_Port
#define SOFT_I2C_SCL_PIN   RTCSCL_Pin

#define BM8563_I2C_ADDR    0x51  // 7位地址

// 寄存器地址
#define BM8563_REG_CTRL1      0x00
#define BM8563_REG_CTRL2      0x01
#define BM8563_REG_SEC        0x02
#define BM8563_REG_MIN        0x03
#define BM8563_REG_HOUR       0x04
#define BM8563_REG_DAY        0x05
#define BM8563_REG_WEEKDAY    0x06
#define BM8563_REG_MONTH      0x07
#define BM8563_REG_YEAR       0x08

// 时间结构体
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t weekday;
    uint8_t month;
    uint16_t year;
} BM8563_TimeTypeDef;

extern BM8563_TimeTypeDef SystemTime;

// 函数声明
void SOFT_I2C_Start(void);
void SOFT_I2C_Stop(void);
uint8_t SOFT_I2C_WriteByte(uint8_t byte);
uint8_t SOFT_I2C_ReadByte(uint8_t ack);
uint8_t BM8563_Soft_I2C_Init(void);
void BM8563_Soft_I2C_SetTime(BM8563_TimeTypeDef *time);
void BM8563_Soft_I2C_GetTime(BM8563_TimeTypeDef *time);

// 从RTC芯片中读取时间 并赋值到系统定义的时间变量中
void getBM8563TimeToSystemTime(void);

#endif
