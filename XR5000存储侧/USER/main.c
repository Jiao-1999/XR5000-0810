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

extern volatile uint32_t g_usb_isr_count;
extern volatile uint32_t g_usb_reset_count;

/*==============================================================
 * ?????? (?????: ??????USART1, GB4717?????????USB CDC)
 * ???????:
 *   W25Q256:  CS-PB12, CLK-PB13, MISO-PB14, MOSI-PB15 (SPI2)
 *   USART1:   PA9(TX), PA10(RX) - ????????????? (115200 8N1)
 *   USB CDC:  PA11(D-), PA12(D+) - ??PC??? (????COM??, GB4717????)
 *             PA11/PA12 ?? USB ???????????, ??????? GPIO ????
 *             PC ???? pyserial ?????? COM ??, ??????????
 *============================================================*/
int main(void)
{
    uint8_t flash_ok;
const uint8_t test_msg[] = "USB CDC Ready (ST???USB-FS-Device_Driver??????)\r\n";
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


    /* ?????USART1 (PA9/PA10, 115200 8N1) - ????????????? */
    uart_init(115200);

    /* ??????????????? (??W25Q256?????) */
    flash_ok = StorageRx_Init();

    delay_ms(100);

    /* W25Q256????????: ????? */
    if (flash_ok != 0)
    {
        while (1) { delay_ms(1000); }
    }

    /* ?????GB4717??????? (????????????, ????p???) */
    GB4717_ExportInit();

    /* ?????USB CDC?????? (PA11/PA12)
     * ?????? PC ????????? COM ??, ????? SWD ????
     * USB????????????? USART1, ????????????????? */
    USB_CDC_Init();

    /* USB CDC ??????: ??????????????????????, ???????·?? */
    {
        
        USB_CDC_SendData(test_msg, sizeof(test_msg) - 1);
    }

    /* 开机自动dump: 在主循环中用计数器延迟dump, 避免阻塞USB中断 */

    while (1)
    {
        StorageRx_Process();
        GB4717_ExportProcess();
        USB_CDC_Poll();
        //USB_CDC_SendData(test_msg, sizeof(test_msg) - 1);
        /* 开机自动dump: 计数器延迟后dump全部记录(一次性) */
        {
            static uint8_t dumped = 0;
            static uint32_t dump_delay = 0;
            if (!dumped)
            {
                if (++dump_delay >= 100000)
                {
                    uint32_t cnt = StorageRx_GetRecordCount();
                    uint32_t i;
                    char msg[128];
                    dumped = 1;
                    sprintf(msg, "[STX_DUMP] count=%lu\r\n", (unsigned long)cnt);
                    USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                    for (i = 0; i < cnt; i++)
                    {
                        EventRecord_t rec;
                        if (StorageRx_ReadRecord(i, &rec) == 0)
                        {
                            sprintf(msg, "[STX_REC#%lu] ctl=%u unit=%u dev=%u ch=%u type=%u evt=%u st=%u %02u-%02u-%02u %02u:%02u:%02u\r\n",
                                (unsigned long)i,
                                rec.controller_no, rec.unit_no, rec.device_no, rec.channel_no,
                                rec.dev_type, rec.event_code, rec.state_code,
                                rec.year, rec.month, rec.day, rec.hour, rec.minute, rec.second);
                            USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                        }
                        else
                        {
                            sprintf(msg, "[STX_REC#%lu] read fail\r\n", (unsigned long)i);
                            USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                        }
                    }
                    sprintf(msg, "[STX_DUMP] end (%lu records)\r\n", (unsigned long)cnt);
                    USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                }
            }
        }

        /* USB_CDC 命令处理: PC 通过 COM17 发送字符触发
         * 'R' = dump全部记录
         * 'C' = 清空存储 (EraseAll, 谨慎) */
        if (USB_CDC_Available() > 0)
        {
            uint16_t cmd = USB_CDC_ReadByte();
            if (cmd == 'R')
            {
                uint32_t count = StorageRx_GetRecordCount();
                uint32_t i;
                char msg[128];
                sprintf(msg, "[STX_DUMP] count=%lu\r\n", (unsigned long)count);
                USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                for (i = 0; i < count; i++)
                {
                    EventRecord_t rec;
                    if (StorageRx_ReadRecord(i, &rec) == 0)
                    {
                        sprintf(msg, "[STX_REC#%lu] ctl=%u unit=%u dev=%u ch=%u type=%u evt=%u st=%u %02u-%02u-%02u %02u:%02u:%02u\r\n",
                            (unsigned long)i,
                            rec.controller_no, rec.unit_no, rec.device_no, rec.channel_no,
                            rec.dev_type, rec.event_code, rec.state_code,
                            rec.year, rec.month, rec.day, rec.hour, rec.minute, rec.second);
                        USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                    }
                    else
                    {
                        sprintf(msg, "[STX_REC#%lu] read fail\r\n", (unsigned long)i);
                        USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
                    }
                }
                sprintf(msg, "[STX_DUMP] end (%lu records)\r\n", (unsigned long)count);
                USB_CDC_SendData((const uint8_t *)msg, (uint16_t)strlen(msg));
            }
            else if (cmd == 'C')
            {
                StorageRx_EraseAll();
                USB_CDC_SendData((const uint8_t *)"[STX_ERASE] done\r\n", 18);
            }
        }

        /* ????: ???????USB_CDC????洢???????, ??λ??????洢????·????
         * bytes=??????????? last=?????? idx=????????? ready=????????
         * ?ж?: bytes???0 ?? ????????????????; bytes??????idx????0 ?? ???????;
         *      idx??????ready=0 ?? ?β????; ready=1 ?? ???????? */
        {
            static uint16_t dbg_div = 0;
            if (++dbg_div >= 50000)
            {
                dbg_div = 0;
                char dbg[96];
                sprintf(dbg, "[STX_DBG] bytes=%lu last=%02X idx=%u ready=%u proc=%lu\r\n",
                    (unsigned long)g_stx_rx_byte_count, g_stx_last_byte,
                    g_stx_rx_idx_snap, g_stx_frame_ready_snap,
                    (unsigned long)g_stx_process_count);
                USB_CDC_SendData((const uint8_t *)dbg, (uint16_t)strlen(dbg));
            }
        }
    }
}
