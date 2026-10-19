#ifndef __BSP_PRINTER_H
#define __BSP_PRINTER_H

/*
 * 文件名称：bsp_printer.h
 * 功能说明：回路设备报警、故障信息的打印服务接口。
 *
 * 设计原则：
 * 1. 报警/故障检测代码只负责将打印事件放入消息队列，不等待打印完成。
 * 2. 本模块不创建独立的UART发送任务，UART10仍由HostUploadDataTask统一发送。
 * 3. 打印帧采用非标准Modbus格式：地址 + 0x06 + 文本长度 + GB2312文本
 *    + CRC16低字节 + CRC16高字节。
 * 4. 当前只处理具有明确“回路号+设备地址”的回路设备报警和故障。
 * 5. 单条文本由时间、设备信息、分隔线三行组成，每行以0D 0A结束。
 */

#include <stdint.h>

/* 打印帧最大缓冲区长度，包含地址、功能码、长度、文本和CRC。 */
#define BSP_PRINTER_FRAME_MAX_LEN  104U

/*
 * 打印使用的设备类型。
 * 枚举值只用于选择打印名称，不改变设备原有产品码、属性和控制逻辑。
 * 所有中文名称均限制在5个汉字以内，保证第二行不超过打印宽度。
 */
typedef enum
{
    BSP_PRINTER_DEVICE_UNKNOWN = 0,       /* 未知设备 */
    BSP_PRINTER_DEVICE_POINT_TEMP,        /* 点型温感 */
    BSP_PRINTER_DEVICE_POINT_SMOKE,       /* 点型烟感 */
    BSP_PRINTER_DEVICE_COMPOUND,          /* 复合探测器 */
    BSP_PRINTER_DEVICE_ISOLATOR,          /* 短路隔离器 */
    BSP_PRINTER_DEVICE_SOUND_LIGHT,       /* 声光报警器 */
    BSP_PRINTER_DEVICE_MANUAL_CALL,       /* 手动报警器 */
    BSP_PRINTER_DEVICE_FIRE_DISPLAY,      /* 火灾显示盘 */
    BSP_PRINTER_DEVICE_GAS_MODULE,        /* 气体模块 */
    BSP_PRINTER_DEVICE_INPUT_OUTPUT,      /* FCM-1011输入输出模块 */
    BSP_PRINTER_DEVICE_DUAL_OUTPUT,       /* FCM-1012双路输出模块 */
    BSP_PRINTER_DEVICE_FAN_BUTTON,        /* 风机启停按钮 */
    BSP_PRINTER_DEVICE_SPRAY_BUTTON,      /* 灭火装置启停按钮 */
    BSP_PRINTER_DEVICE_FAN_CONTROLLER     /* IG3302-DC风机控制器 */
}BspPrinterDeviceType;

/*
 * 打印使用的报警/故障类型。
 * 枚举值只负责映射打印短名称，不参与报警判断、故障判断或联动控制。
 * 事件名称限制在4个汉字的显示宽度以内；CO、H2使用英文字符。
 */
typedef enum
{
    BSP_PRINTER_EVENT_NONE = 0,          /* 无有效打印事件 */
    BSP_PRINTER_EVENT_TEMP_ALARM,        /* 温度报警 */
    BSP_PRINTER_EVENT_SMOKE_ALARM,       /* 烟雾报警 */
    BSP_PRINTER_EVENT_MANUAL_ALARM,      /* 手动报警 */
    BSP_PRINTER_EVENT_CO_LOW,            /* CO低报 */
    BSP_PRINTER_EVENT_CO_HIGH,           /* CO高报 */
    BSP_PRINTER_EVENT_H2_LOW,            /* H2低报 */
    BSP_PRINTER_EVENT_H2_HIGH,           /* H2高报 */
    BSP_PRINTER_EVENT_COMM_FAULT,        /* 通信故障/设备掉线 */
    BSP_PRINTER_EVENT_TEMP_FAULT,        /* 温感故障 */
    BSP_PRINTER_EVENT_SMOKE_FAULT,       /* 烟感故障 */
    BSP_PRINTER_EVENT_INPUT_FAULT,       /* 输入故障 */
    BSP_PRINTER_EVENT_OUTPUT_FAULT,      /* 输出故障 */
    BSP_PRINTER_EVENT_IDENTIFY_FAULT,    /* 设备识别故障 */
    BSP_PRINTER_EVENT_BRANCH1_SHORT,     /* 隔离器支路1短路 */
    BSP_PRINTER_EVENT_BRANCH2_SHORT,     /* 隔离器支路2短路 */
    BSP_PRINTER_EVENT_BRANCH3_SHORT,     /* 隔离器支路3短路 */
    BSP_PRINTER_EVENT_FAN1_FAULT,        /* 风机1故障 */
    BSP_PRINTER_EVENT_FAN2_FAULT,        /* 风机2故障 */
    BSP_PRINTER_EVENT_DEVICE_FAULT       /* 未细分的设备故障 */
}BspPrinterEventType;

/*
 * 主机自身故障打印类型。
 * 主机故障没有回路号和设备地址，使用“主机|部件|故障状态”的独立格式。
 */
typedef enum
{
    BSP_PRINTER_HOST_FAULT_NONE = 0,     /* 无有效主机故障 */
    BSP_PRINTER_HOST_MAIN_POWER,         /* 主电断电故障 */
    BSP_PRINTER_HOST_BACK_POWER_OPEN,    /* 备用电池开路故障 */
    BSP_PRINTER_HOST_BACK_POWER_SHORT,   /* 备用电池短路故障 */
    BSP_PRINTER_HOST_STORAGE,            /* 系统存储故障 */
    BSP_PRINTER_HOST_RESET_SEND          /* 总复位指令发送异常 */
}BspPrinterHostFaultType;

/*
 * 打印队列中的单条回路事件。
 * 这里只保存结构化数据，最终GB2312文本和CRC在实际发送前统一生成。
 */
typedef struct
{
    uint16_t year;       /* 事件年份，例如2026 */
    uint8_t month;       /* 事件月份：1～12 */
    uint8_t day;         /* 事件日期：1～31 */
    uint8_t hour;        /* 事件小时：0～23 */
    uint8_t minute;      /* 事件分钟：0～59 */
    uint8_t second;      /* 事件秒：0～59 */
    uint8_t loop;        /* 回路号：当前支持1～4 */
    uint8_t address;     /* 设备真实物理地址 */
    uint8_t device_type; /* BspPrinterDeviceType，仅用于打印名称映射 */
    uint8_t event_type;  /* BspPrinterEventType，仅用于事件名称映射 */
}BspPrinterLoopEvent;

/* 主机自身故障消息，不包含回路号和设备地址。 */
typedef struct
{
    uint16_t year;       /* 故障年份，例如2026 */
    uint8_t month;       /* 故障月份：1～12 */
    uint8_t day;         /* 故障日期：1～31 */
    uint8_t hour;        /* 故障小时：0～23 */
    uint8_t minute;      /* 故障分钟：0～59 */
    uint8_t second;      /* 故障秒：0～59 */
    uint8_t fault_type;  /* BspPrinterHostFaultType */
}BspPrinterHostFault;

/* 创建512条静态打印消息队列；应在RTOS任务启动前调用一次。 */
void BspPrinter_Init(void);

/* 清空尚未发送的打印事件和队列溢出计数；系统复位时调用。 */
void BspPrinter_Reset(void);

/*
 * 将一条回路报警/故障加入打印队列。
 * 此接口不阻塞、不发送UART，也不等待打印机；成功返回1，失败返回0。
 */
uint8_t BspPrinter_QueueLoopEvent(const BspPrinterLoopEvent *event);

/* 将主电、备电、存储或复位发送故障加入同一个非阻塞打印队列。 */
uint8_t BspPrinter_QueueHostFault(const BspPrinterHostFault *fault);

/* 查询队列中是否还有待打印事件；有事件返回1，否则返回0。 */
uint8_t BspPrinter_HasPending(void);

/*
 * 取出最早的一条事件并生成完整0x06打印帧。
 * 该接口只能由现有UART10发送任务调用，成功返回1并填写frame_length。
 */
uint8_t BspPrinter_BuildNextFrame(uint8_t *frame, uint8_t capacity,
                                  uint8_t *frame_length);

/* 获取因队列已满而未能入队的事件数量，计数最大为65535。 */
uint16_t BspPrinter_GetDroppedCount(void);

#endif
