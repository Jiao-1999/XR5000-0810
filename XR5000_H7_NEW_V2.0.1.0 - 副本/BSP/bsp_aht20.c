#include "bsp_aht20.h"

#define AHT20_I2C_ADDR        0x38U
#define AHT20_CMD_INIT        0xBEU
#define AHT20_CMD_MEASURE     0xACU
#define AHT20_CMD_SOFT_RESET  0xBAU

static int16_t g_aht20_temperature = 0;
static uint16_t g_aht20_humidity = 0;
static uint8_t g_aht20_valid = 0;

static void AHT20_Delay(void)
{
    for(volatile uint16_t i = 0; i < 80; i++)
    {
        __NOP();
    }
}

static void AHT20_SDA_Out(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AHTSDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AHTSDA_GPIO_Port, &GPIO_InitStruct);
}

static void AHT20_SDA_In(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AHTSDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AHTSDA_GPIO_Port, &GPIO_InitStruct);
}

static void AHT20_SCL(uint8_t state)
{
    HAL_GPIO_WritePin(AHTSCL_GPIO_Port, AHTSCL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void AHT20_SDA(uint8_t state)
{
    HAL_GPIO_WritePin(AHTSDA_GPIO_Port, AHTSDA_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t AHT20_SDA_Read(void)
{
    return (HAL_GPIO_ReadPin(AHTSDA_GPIO_Port, AHTSDA_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void AHT20_Start(void)
{
    AHT20_SDA_Out();
    AHT20_SDA(1);
    AHT20_SCL(1);
    AHT20_Delay();
    AHT20_SDA(0);
    AHT20_Delay();
    AHT20_SCL(0);
    AHT20_Delay();
}

static void AHT20_Stop(void)
{
    AHT20_SDA_Out();
    AHT20_SDA(0);
    AHT20_SCL(1);
    AHT20_Delay();
    AHT20_SDA(1);
    AHT20_Delay();
}

static uint8_t AHT20_WriteByte(uint8_t byte)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        AHT20_SDA((byte & 0x80U) ? 1U : 0U);
        AHT20_Delay();
        AHT20_SCL(1);
        AHT20_Delay();
        AHT20_SCL(0);
        AHT20_Delay();
        byte <<= 1;
    }

    AHT20_SDA(1);
    AHT20_SDA_In();
    AHT20_Delay();
    AHT20_SCL(1);
    AHT20_Delay();
    uint8_t nack = AHT20_SDA_Read();
    AHT20_SCL(0);
    AHT20_SDA_Out();
    AHT20_Delay();

    return nack;
}

static uint8_t AHT20_ReadByte(uint8_t nack)
{
    uint8_t byte = 0;

    AHT20_SDA(1);
    AHT20_SDA_In();
    for(uint8_t i = 0; i < 8; i++)
    {
        byte <<= 1;
        AHT20_SCL(1);
        AHT20_Delay();
        if(AHT20_SDA_Read())
        {
            byte |= 1U;
        }
        AHT20_SCL(0);
        AHT20_Delay();
    }

    AHT20_SDA_Out();
    AHT20_SDA(nack ? 1U : 0U);
    AHT20_Delay();
    AHT20_SCL(1);
    AHT20_Delay();
    AHT20_SCL(0);
    AHT20_SDA(1);

    return byte;
}

static uint8_t AHT20_Crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFU;

    for(uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(uint8_t bit = 0; bit < 8; bit++)
        {
            if(crc & 0x80U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x31U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static uint8_t AHT20_WriteCmd(uint8_t cmd, uint8_t data0, uint8_t data1)
{
    AHT20_Start();
    if(AHT20_WriteByte((AHT20_I2C_ADDR << 1) | 0U))
    {
        AHT20_Stop();
        return 1;
    }
    if(AHT20_WriteByte(cmd) || AHT20_WriteByte(data0) || AHT20_WriteByte(data1))
    {
        AHT20_Stop();
        return 1;
    }
    AHT20_Stop();
    return 0;
}

void AHT20_Init(void)
{
    g_aht20_valid = 0;
    AHT20_SDA_Out();
    AHT20_SCL(1);
    AHT20_SDA(1);
    HAL_Delay(40);

    AHT20_Start();
    if(AHT20_WriteByte((AHT20_I2C_ADDR << 1) | 0U) == 0)
    {
        AHT20_WriteByte(AHT20_CMD_SOFT_RESET);
    }
    AHT20_Stop();
    HAL_Delay(20);
    (void)AHT20_WriteCmd(AHT20_CMD_INIT, 0x08U, 0x00U);
    HAL_Delay(10);
}

uint8_t AHT20_Update(void)
{
    uint8_t data[7] = {0};

    if(AHT20_WriteCmd(AHT20_CMD_MEASURE, 0x33U, 0x00U))
    {
        return 1;
    }

    HAL_Delay(80);

    AHT20_Start();
    if(AHT20_WriteByte((AHT20_I2C_ADDR << 1) | 1U))
    {
        AHT20_Stop();
        return 1;
    }

    for(uint8_t i = 0; i < 6; i++)
    {
        data[i] = AHT20_ReadByte(0);
    }
    data[6] = AHT20_ReadByte(1);
    AHT20_Stop();

    if((data[0] & 0x80U) || AHT20_Crc8(data, 6) != data[6])
    {
        return 1;
    }

    uint32_t raw_humi = (((uint32_t)data[1]) << 12) | (((uint32_t)data[2]) << 4) | (((uint32_t)data[3]) >> 4);
    uint32_t raw_temp = ((((uint32_t)data[3]) & 0x0FU) << 16) | (((uint32_t)data[4]) << 8) | data[5];

    g_aht20_humidity = (uint16_t)((raw_humi * 100U + 0x80000U) >> 20);
    g_aht20_temperature = (int16_t)(((int32_t)(raw_temp * 200U + 0x80000U) >> 20) - 50);
    g_aht20_valid = 1;

    return 0;
}

uint8_t AHT20_IsValid(void)
{
    return g_aht20_valid;
}

int16_t AHT20_GetTemperature(void)
{
    return g_aht20_temperature;
}

uint16_t AHT20_GetHumidity(void)
{
    return g_aht20_humidity;
}
