/**
 * @file    soft_uart.c
 * @brief   软件串口模块 - PA12(TX)/PA11(RX), 9600 8N1, TIM3 3倍过采样
 * @details 使用TIM3定时中断实现全双工软件串口, 提供额外的串口通信通道.
 *          本模块为通用软件串口驱动, 上层可调用Send/Read接口收发数据.
 *
 *   时钟与采样:
 *     TIM3时钟 = 72MHz (APB1=36MHz, 定时器倍频后72MHz)
 *     3倍过采样频率 = 3 * 9600 = 28800Hz
 *     ARR = 72MHz / 28800 - 1 = 2499
 *
 *   TX策略: 每个比特持续3次中断采样, 第1次中断改变电平, 之后2次保持(保证电平稳定)
 *   RX策略: 检测到起始位下降沿后, 每3次中断采样1次数据位
 *
 *   引脚分配:
 *     PA12 = TX (推挽输出, 与USB的USBDP复用, USB启用时不可用)
 *     PA11 = RX (上拉输入, 与USB的USBDM复用, USB启用时不可用)
 *   说明: PA11/PA12与USB D-/D+复用, 当USB CDC未启用时本软件串口可作为备用串口通道使用.
 */
#include "soft_uart.h"
#include "delay.h"

/*==============================================================
 * GPIO位操作宏 (直接操作PA11/PA12, 避免HAL开销)
 *============================================================*/
#define SU_TX_HIGH()   GPIO_SetBits(GPIOA, GPIO_Pin_12)   /* PA12置高(TX输出1/空闲/停止位) */
#define SU_TX_LOW()    GPIO_ResetBits(GPIOA, GPIO_Pin_12) /* PA12置低(TX输出0/起始位) */
#define SU_TX_READ()   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) /* 读取PA12输出状态 */
#define SU_RX_READ()   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) /* 读取PA11输入电平(RX) */

/*==============================================================
 * TX发送变量
 *============================================================*/
static volatile uint8_t  su_tx_busy = 0;      /* TX发送忙标志(1=正在发送) */
static volatile uint8_t  su_tx_shift = 0;     /* TX移位寄存器(存放待发送数据,LSB先发) */
static volatile uint8_t  su_tx_bit_idx = 0;   /* TX当前比特序号 (0=起始位, 1-8=数据位, 9=停止位) */
static volatile uint8_t  su_tx_sub = 0;       /* TX比特内子计数(0,1,2; 3次中断构成1个比特) */

/*==============================================================
 * RX接收变量
 *============================================================*/
static volatile uint8_t  su_rx_buf[SOFT_UART_RX_BUF_SIZE]; /* 接收环形缓冲区 */
static volatile uint16_t su_rx_head = 0;      /* 写入位置(head, ISR写入) */
static volatile uint16_t su_rx_tail = 0;      /* 读出位置(tail, 主程序读出) */
static volatile uint16_t su_rx_count = 0;     /* 缓冲中当前未读字节数 */

static volatile uint8_t  su_rx_active = 0;    /* RX正在接收标志(检测到起始位后置1) */
static volatile uint8_t  su_rx_shift = 0;     /* RX移位寄存器(存放接收数据,LSB先收) */
static volatile uint8_t  su_rx_bit_idx = 0;   /* RX当前比特序号 (0=起始位采样, 1-8=数据位, 9=停止位) */
static volatile uint8_t  su_rx_sub = 0;       /* RX比特内子计数(0,1,2; 3次中断构成1个比特) */

/*==============================================================
 * 初始化软件串口 (配置GPIO/TIM3/NVIC)
 *============================================================*/
/**
 * @brief  初始化软件串口模块
 * @note   完成: GPIO配置(PA12推挽输出/PA11上拉输入)、TIM3时基配置(28800Hz)、
 *         NVIC中断使能、变量复位、启动TIM3.
 */
void SoftUART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能时钟: GPIOA + AFIO + TIM3 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* PA12 = TX, 推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    SU_TX_HIGH();  /* 空闲状态拉高 */

    /* PA11 = RX, 上拉输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM3时基配置: 3倍过采样频率 = 28800Hz */
    /* TIM3时钟 = 72MHz, ARR = 72MHz/28800 - 1 = 2499 */
    TIM_TimeBaseStructure.TIM_Period        = 2499;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* NVIC配置: 抢占优先级2, 低于USART1(保证主控收发不被打断) */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 复位接收/发送变量 */
    su_rx_head = 0;
    su_rx_tail = 0;
    su_rx_count = 0;
    su_rx_active = 0;
    su_tx_busy = 0;

    /* 启动TIM3开始采样 */
    TIM_Cmd(TIM3, ENABLE);
}

/*==============================================================
 * 发送一个字节 (阻塞)
 *============================================================*/
/**
 * @brief  阻塞发送一个字节
 * @param  data: 待发送字节
 * @note   帧格式: 起始位(0) + 8数据位(LSB先发) + 停止位(1), 无校验位
 *         本函数将数据装入移位寄存器后等待ISR完成发送.
 */
void SoftUART_SendByte(uint8_t data)
{
    /* 等待上一次发送完成 */
    while (su_tx_busy);

    /* 装载TX移位寄存器 */
    su_tx_shift = data;
    su_tx_bit_idx = 0;
    su_tx_sub = 0;
    su_tx_busy = 1;

    /* 阻塞等待本次发送完成(ISR中清除su_tx_busy) */
    while (su_tx_busy);
}

/*==============================================================
 * 发送多字节数据
 *============================================================*/
/**
 * @brief  发送多字节
 * @param  data: 数据指针
 * @param  len:  字节数
 */
void SoftUART_SendData(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        SoftUART_SendByte(data[i]);
    }
}

/*==============================================================
 * 查询接收缓冲中可读取字节数
 *============================================================*/
/**
 * @brief  查询接收缓冲中可用字节数
 * @retval 可读字节数
 */
uint16_t SoftUART_Available(void)
{
    return su_rx_count;
}

/*==============================================================
 * 读取一个字节
 *============================================================*/
/**
 * @brief  从接收缓冲读取一个字节
 * @retval 读取到的字节; 0xFFFF表示缓冲为空
 */
uint16_t SoftUART_ReadByte(void)
{
    uint16_t ret;

    if (su_rx_count == 0)
    {
        return 0xFFFF;  /* 无数据可读 */
    }

    ret = su_rx_buf[su_rx_tail];
    su_rx_tail = (su_rx_tail + 1) % SOFT_UART_RX_BUF_SIZE;
    su_rx_count--;

    return ret;
}

/*==============================================================
 * TIM3中断服务函数 (3倍过采样, 同时处理TX和RX)
 *============================================================*/
/**
 * @brief  TIM3中断服务函数 (由stm32f10x_it.c的TIM3_IRQHandler调用)
 * @note   每次中断(28800Hz)同时执行:
 *         1) TX发送: su_tx_sub在0,1,2三个子计数内维持当前比特电平;
 *            su_tx_sub=0时按比特序号输出电平(起始位/数据位/停止位), sub=0时改变电平.
 *            比特序号: 0=起始位(拉低), 1-8=数据位(LSB先发), 9=停止位(拉高).
 *         2) RX接收: 每次中断采样PA11电平.
 *            空闲时检测下降沿(起始位), 命中后每3次中断取1个采样点,
 *            比特序号: 0=起始位校验(必须为0), 1-8=数据位(LSB先收), 9=停止位(必须为1).
 *            停止位有效后将移位寄存器存入环形缓冲.
 */
void SoftUART_TIM3_ISR(void)
{
    /*=========== TX: 发送移位处理 ===========*/
    if (su_tx_busy)
    {
        if (su_tx_sub == 0)
        {
            /* 子计数0: 输出当前比特电平 */
            if (su_tx_bit_idx == 0)
            {
                SU_TX_LOW();  /* 起始位: 拉低 */
            }
            else if (su_tx_bit_idx <= 8)
            {
                /* 数据位: LSB先发 */
                if (su_tx_shift & 0x01)
                    SU_TX_HIGH();
                else
                    SU_TX_LOW();
                su_tx_shift >>= 1;
            }
            else
            {
                SU_TX_HIGH();  /* 停止位: 拉高 */
            }
        }

        su_tx_sub++;
        if (su_tx_sub >= 3)
        {
            /* 3次子计数完成, 进入下一比特 */
            su_tx_sub = 0;
            su_tx_bit_idx++;
            if (su_tx_bit_idx > 9)  /* 0=起始, 1-8=数据, 9=停止 */
            {
                su_tx_busy = 0;
                SU_TX_HIGH();  /* 发送完成, 恢复空闲电平 */
            }
        }
    }

    /*=========== RX: 接收采样处理 (3倍过采样) ===========*/
    {
        uint8_t bit = SU_RX_READ();

        if (!su_rx_active)
        {
            /* 空闲态: 检测起始位下降沿(电平拉低) */
            if (bit == 0)
            {
                su_rx_active = 1;
                su_rx_bit_idx = 0;  /* 0=起始位采样阶段 */
                su_rx_sub = 0;
                su_rx_shift = 0;
            }
        }
        else
        {
            su_rx_sub++;
            if (su_rx_sub >= 3)
            {
                /* 每3次中断取1个有效采样点(对齐到比特中央) */
                su_rx_sub = 0;

                if (su_rx_bit_idx == 0)
                {
                    /* 起始位校验: 必须为低电平 */
                    if (bit != 0)
                    {
                        su_rx_active = 0;  /* 非起始位, 误触发, 退出 */
                    }
                    else
                    {
                        su_rx_bit_idx = 1;  /* 进入数据位接收 */
                    }
                }
                else if (su_rx_bit_idx <= 8)
                {
                    /* 数据位: LSB先收, 右移入移位寄存器 */
                    su_rx_shift >>= 1;
                    if (bit)
                        su_rx_shift |= 0x80;
                    su_rx_bit_idx++;
                }
                else
                {
                    /* 停止位: 必须为高电平, 有效则存入缓冲 */
                    if (bit == 1 && su_rx_count < SOFT_UART_RX_BUF_SIZE)
                    {
                        su_rx_buf[su_rx_head] = su_rx_shift;
                        su_rx_head = (su_rx_head + 1) % SOFT_UART_RX_BUF_SIZE;
                        su_rx_count++;
                    }
                    su_rx_active = 0;  /* 结束本字节接收 */
                }
            }
        }
    }
}
