#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "bsp_storage_rx.h"
#include "w25qxx.h"
#include "usb_cdc.h"
#include "gb4717_export.h"
#include "led.h"
#include <stdio.h>
#include <string.h>
/* P1-B修复: 测试源码总开关 (0=关闭dump/DBG/R/C命令, 1=开启) */
#define STX_DEBUG_BUILD  1

extern volatile uint32_t g_usb_isr_count;
extern volatile uint32_t g_usb_reset_count;

/*==============================================================
 * 存储侧主程序 (通信通道: USART1 + GB4717导出/USB CDC)
 * 通信接口说明:
 *   W25Q256:  CS-PB12, CLK-PB13, MISO-PB14, MOSI-PB15 (SPI2)
 *   USART1:   PA9(TX), PA10(RX) - 与主机侧串口通信 (115200 8N1)
 *   USB CDC:  PA11(D-), PA12(D+) - 与PC通信 (虚拟COM口, GB4717导出)
 *             PA11/PA12 为 USB 差分信号引脚, 需配置 GPIO 复用
 *             PC 端可用 pyserial 打开对应 COM 口, 用于调试导出
 *============================================================*/
int main(void)
{
    uint8_t flash_ok;
const uint8_t test_msg[] = "USB CDC Ready (ST USB-FS-Device_Driver OK)\r\n";
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


    /* 初始化USART1 (PA9/PA10, 115200 8N1) - 与主机侧串口通信 */
    uart_init(115200);

    /* 初始化存储Flash驱动 (使用W25Q256外部存储) */
    flash_ok = StorageRx_Init();

    delay_ms(100);

    /* W25Q256初始化失败: 死循环 */
    if (flash_ok != 0)
    {
        while (1) { delay_ms(1000); }
    }

    /* 初始化GB4717导出模块 (复位状态机, 清空读取游标) */
    GB4717_ExportInit();

    /* 初始化USB CDC虚拟串口 (PA11/PA12)
     * 枚举完成后 PC 端会出现对应 COM 口, 可并行 SWD 调试
     * USB 占用的引脚复用不影响 USART1, 串口通信正常并行 */
    USB_CDC_Init();

    /* USB CDC 自检: 发送初始化完成提示信息, 便于判断通信链路 */
    {
        
        USB_CDC_SendData(test_msg, sizeof(test_msg) - 1);
    }

    /* 开机自动dump: 在主循环中用计数器延迟dump, 避免阻塞USB中断 */

    while (1)
    {
        StorageRx_Process();
        StorageRx_MetaPrepare();  /* P0-A修复(v2): 后台分段预擦meta bank, 每圈上电等1扇区 */
        GB4717_ExportProcess();
        USB_CDC_Poll();
        //USB_CDC_SendData(test_msg, sizeof(test_msg) - 1);
#if STX_DEBUG_BUILD
        /* 开机自动dump: 计数器延迟后dump全部记录(一次性) */
        {
            static uint8_t dumped = 0;
            static uint32_t dump_delay = 0;
            if (!dumped)
            {
                if (++dump_delay >= 100000)
                {
                    uint32_t cnt = StorageRx_GetTotalCount();
                    uint32_t i;
                    uint8_t z;
                    char msg[128];
                    dumped = 1;
                    sprintf(msg, "[STX_DUMP] total=%lu (Z0_first=%lu Z1_fire=%lu Z2_fault=%lu Z3_gen=%lu)\r\n",
                        (unsigned long)cnt,
                        (unsigned long)StorageRx_GetRecordCount(STX_ZONE_FIRST),
                        (unsigned long)StorageRx_GetRecordCount(STX_ZONE_FIRE),
                        (unsigned long)StorageRx_GetRecordCount(STX_ZONE_FAULT),
                        (unsigned long)StorageRx_GetRecordCount(STX_ZONE_GENERAL));
                    USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                    for (z = 0; z < STX_ZONE_COUNT; z++)
                    {
                        uint32_t zcnt = StorageRx_GetRecordCount(z);
                        for (i = 0; i < zcnt; i++)
                        {
                            EventRecord_t rec;
                            if (StorageRx_ReadRecord(z, i, &rec) == 0)
                            {
                                sprintf(msg, "[STX_REC Z%u#%lu] ctl=%u unit=%u dev=%u ch=%u type=%u evt=%u st=%u %02u-%02u-%02u %02u:%02u:%02u\r\n",
                                    (unsigned)z, (unsigned long)i,
                                    rec.controller_no, rec.unit_no, rec.device_no, rec.channel_no,
                                    rec.dev_type, rec.event_code, rec.state_code,
                                    rec.year, rec.month, rec.day, rec.hour, rec.minute, rec.second);
                                USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                            }
                        }
                    }
                    sprintf(msg, "[STX_DUMP] end (%lu records)\r\n", (unsigned long)cnt);
                    USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                }
            }
        }

        /* USB_CDC 命令处理: PC 通过 COM17 发送字符命令,
         * 'R' = dump全部记录
         * 'C' = 清空存储 (EraseAll, 谨慎) */
        /* P1-6: 0x40-start frames belong to GB4717 parser, peek to route; the
         * IsIdle() gate prevents stealing mid-frame payload bytes (USB packets
         * may split a frame across two drains - theft there poisons the parser
         * state machine and it never resyncs) */
        if (USB_CDC_Available() > 0 && GB4717_IsIdle() && USB_CDC_PeekByte() != 0x40)
        {
            uint16_t cmd = USB_CDC_ReadByte();
            if (cmd == 'R')
            {
                uint32_t count = StorageRx_GetTotalCount();
                uint32_t i;
                uint8_t z;
                char msg[128];
                sprintf(msg, "[STX_DUMP] total=%lu (Z0=%lu Z1=%lu Z2=%lu Z3=%lu)\r\n",
                    (unsigned long)count,
                    (unsigned long)StorageRx_GetRecordCount(STX_ZONE_FIRST),
                    (unsigned long)StorageRx_GetRecordCount(STX_ZONE_FIRE),
                    (unsigned long)StorageRx_GetRecordCount(STX_ZONE_FAULT),
                    (unsigned long)StorageRx_GetRecordCount(STX_ZONE_GENERAL));
                USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                for (z = 0; z < STX_ZONE_COUNT; z++)
                {
                    uint32_t zcnt = StorageRx_GetRecordCount(z);
                    for (i = 0; i < zcnt; i++)
                    {
                        EventRecord_t rec;
                        if (StorageRx_ReadRecord(z, i, &rec) == 0)
                        {
                            sprintf(msg, "[STX_REC Z%u#%lu] ctl=%u unit=%u dev=%u ch=%u type=%u evt=%u st=%u %02u-%02u-%02u %02u:%02u:%02u\r\n",
                                (unsigned)z, (unsigned long)i,
                                rec.controller_no, rec.unit_no, rec.device_no, rec.channel_no,
                                rec.dev_type, rec.event_code, rec.state_code,
                                rec.year, rec.month, rec.day, rec.hour, rec.minute, rec.second);
                            USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                        }
                    }
                }
                sprintf(msg, "[STX_DUMP] end (%lu records)\r\n", (unsigned long)count);
                USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
            }
            else if (cmd == 'C')
            {
                /* 存储复位确认: 需要两次发送'C'才执行, 防误触 */
                static uint8_t erase_armed = 0;
                if (erase_armed == 0)
                {
                    erase_armed = 1;
                    USB_CDC_SendData((const uint8_t *)"[STX_ERASE] send 'C' again to confirm\r\n", 39);
                }
                else
                {
                    erase_armed = 0;
                    StorageRx_EraseAll();
                    USB_CDC_SendData((const uint8_t *)"[STX_ERASE] done\r\n", 18);
                }
            }
        }

        /* 测试参数: 通过 USB CDC 输入参数设置, 见 USB_CDC 命令处理分支
         * bytes=本次开机累计接收字节数 last=最后一字节值 idx=帧重组游标位置 ready=帧就绪标志
         * 判断: bytes递增0值 ? 无帧接收; bytes不变且idx不是0 ? 重组中;
         *       idx递增ready=0 ? 帧未完整; ready=1 ? 帧接收完成待处理 */
#endif
    }
}
