#include "bsp_test_inject.h"
#include "bsp_debug.h"
#include "bsp_mbus.h"
#include "bsp_mbus_control.h"    /* loop2 MBus control inject */
#include "bsp_rs485_detect.h"    /* loop3 RS485 inject */
#include "cmd_process.h"         /* zhu_state/bei_state */
#include "bsp_key.h"             /* hand_paper_state */
#include "bsp_itcallback.h"  /* uartbuff[] + BUFF_MAX */
#include "bsp_logic_screen.h" /* 逻辑设定界面按键转发(画面43) */
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* XR5000_TEST_INJECT_20260811: COM4串口注入调试
 * UART4(PC10/PC11) RX挂接于main.c中HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uartbuff[3], ...)
 * 由DMA中断自动填充数据到uartbuff[3]中供本任务轮询: 检测recepetion_flag, 按行解析命令,
 * 根据ONL/TF/SF等命令直接写公共缓冲区并触发报警判断, 经PointTypeDetectorDataDeal()产生
 * 变化, 报警/故障事件经: StorageEvent_LogFire/LogFault -> LPUART1发送给存储端
 *
 * 命令格式(区分大小写, \r\n结尾):
 *   ONL<addr>  -> 设备上线, 清除断连计数并写探测器名称(默认TF/SF使用的地址)
 *   TF<value>  -> 温度传感器+报警判断, 直接写入Temper[2]=value
 *   SF<value>  -> 烟雾传感器+报警判断, 直接写入Smoke[2]=value
 *   (XR5000_INJECT_FIX_20260811: 修复写入 DetecteName/DisconnectCount 导致判断失效,
 *    使PointTypeDetectorDataDeal按类型5/6正常判断, 避免地址被误判为类型3的误报)
 *
 * 温度状态: 0=正常 1=预警 2=火警 3=温度传感器故障
 * 烟雾状态: 0=正常 1=预警 2=火警 8=烟雾污染故障 9=烟雾传感器故障
 */

#define TEST_INJECT_BUF_LEN        32u   /* 串口命令行缓冲区长度 */
#define TEST_INJECT_TASK_PERIOD_MS 100u  /* 任务轮询周期(ms) */
#define TEST_INJECT_DEFAULT_ADDR   2u    /* 默认注入地址(TF/SF不带地址参数时使用的addr) */
#define TEST_INJECT_UART_IDX       3u    /* uartbuff[3]对应UART4 */

/* 温度/烟雾接收状态数组: 定义于bsp_mbus.c内部, 经extern声明使用 */
extern uint8_t PointTypeMixtureReceiveStateTemper[MIXTURE_DEVICE_SUM];
extern uint8_t PointTypeMixtureReceiveStateSmoke[MIXTURE_DEVICE_SUM];
/* XR5000_INJECT_FIX_20260811: 修复探测器名称与断连计数
 * - PointTypeMixtureDetecteName: 探测器名称数组, 注入写错名称会干扰MBus类型确认(6=温度 5=烟雾),
 *   否则默认全为0, PointTypeDetectorDataDeal按type判断5/6分支, 会导致报警/故障判断失效
 * - PointTypeMixtureDisconnectCount: 断连计数数组, MBus巡检间隔80ms轮询一次+1, 超400ms
 *   连续3次未响应则判为离线, 注入时需同步清零防止误判离线 */
extern uint8_t PointTypeMixtureDetecteName[MIXTURE_DEVICE_SUM];
extern uint8_t PointTypeMixtureDisconnectCount[MIXTURE_DEVICE_SUM];

/* XR5000_INJECT_EXT_20260818: hand paper state global (bsp_key.c, non-static) */
extern uint8_t hand_paper_state;

/* 解析一行注入命令
 * 参数: line 指向以\r\n结尾的命令字符串, len 为有效长度
 * 返回: 0=解析成功, 1=未识别 */
static uint8_t TestInject_ParseLine(const char *line, uint8_t len)
{
    uint8_t addr = TEST_INJECT_DEFAULT_ADDR;
    uint8_t val;

    /* XR5000_INJECT_EXT_20260818: K<ctrl> 转发屏幕按键到逻辑设定界面(画面43)
     * 直接驱动 LogicScreen_OnButton(43, ctrl, 1) 走真实设定代码路径
     * (数字输入->传感器确认->运算符->等号->动作->确认保存->Flash持久化)
     * 键值: 1-9数字 10=0 11=# 12=| 13=& 14=取消 15=删除 16=确认
     *       18=( 19=压力 20== 21=) 24=VOC 25=CH4 26=H2 27=CO 28=烟雾 29=温度 72=新建 */
    if (line[0] == 'K' && len >= 2u && len <= 3u)
    {
        uint8_t kctrl = 0u;
        uint8_t ki;
        for (ki = 1u; ki < len; ki++)
        {
            if (line[ki] < '0' || line[ki] > '9')
            {
                kctrl = 0u;
                break;
            }
            kctrl = (uint8_t)(kctrl * 10u + (uint8_t)(line[ki] - '0'));
        }
        if (kctrl != 0u)
        {
            LogicScreen_OnButton(43u, kctrl, 1u);
            DebugPrintf("[INJECT] KEY%d -> LogicScreen_OnButton(43,%d)\r\n", kctrl, kctrl);
            return 0u;
        }
    }

    if (len < 3u) return 1u;

    /* 命令末尾必须是数字, 取最后一位作为数值 */
    if (line[len - 1u] < '0' || line[len - 1u] > '9') return 1u;
    val = (uint8_t)(line[len - 1u] - '0');

    /* ONL<addr>: 4个字符, 最后一位addr上线 */
    if (len == 4u && line[0] == 'O' && line[1] == 'N' && line[2] == 'L')
    {
        PointTypeMixtureOnlieStateSingleSetting(val, 1u);
        PointTypeMixtureDisconnectCount[val] = 0u; /* 清0: 清除断连计数, 避免MBus巡检将其误判为离线 */
        PointTypeMixtureDetecteName[val] = 6u;     /* 置6: 标记为温度探测器(默认TF/SF使用的地址) */
        DebugPrintf("[INJECT] ONL%d -> addr=%d online\r\n", val, val);
        return 0u;
    }

    /* TF<value>: 3个字符, 温度值写入, addr默认=2 */
    if (len == 3u && line[0] == 'T' && line[1] == 'F')
    {
        PointTypeMixtureDetecteName[addr] = 6u;     /* 置6: 标记为温度探测器, 让PointTypeDetectorDataDeal走温度分支 */
        PointTypeMixtureDisconnectCount[addr] = 0u; /* 清0: 清除断连计数, 避免巡检判定断连而离线 */
        if (getPointTypeMixtureSettingOnlieState(addr) == 0u)
        {
            DebugPrintf("[INJECT] TF%d SKIP (addr=%d offline)\r\n", val, addr);
            return 0u;
        }
        PointTypeMixtureReceiveStateTemper[addr] = val;
        DebugPrintf("[INJECT] TF%d -> Temper[%d]=%d\r\n", val, addr, val);
        return 0u;
    }

    /* SF<value>: 3个字符, 烟雾值写入, addr默认=2 */
    if (len == 3u && line[0] == 'S' && line[1] == 'F')
    {
        PointTypeMixtureDetecteName[addr] = 5u;     /* 置5: 标记为烟雾探测器, 让PointTypeDetectorDataDeal走烟雾分支 */
        PointTypeMixtureDisconnectCount[addr] = 0u; /* 清0: 清除断连计数, 避免巡检判定断连而离线 */
        if (getPointTypeMixtureSettingOnlieState(addr) == 0u)
        {
            DebugPrintf("[INJECT] SF%d SKIP (addr=%d offline)\r\n", val, addr);
            return 0u;
        }
        PointTypeMixtureReceiveStateSmoke[addr] = val;
        DebugPrintf("[INJECT] SF%d -> Smoke[%d]=%d\r\n", val, addr, val);
        return 0u;
    }

    /* XR5000_INJECT_EXT_20260818: loop2/loop3/hand/power inject commands */

    /* O2<addr>: loop2 MBus control device online (len==3, e.g. O23) */
    if (len == 3u && line[0] == 'O' && line[1] == '2')
    {
        MBusCtrl_SetOnline(val, 1u);
        DebugPrintf("[INJECT] O2%d -> loop2 addr=%d online\r\n", val, val);
        return 0u;
    }

    /* S2<addr><state>: loop2 device sensor state inject (len==4, e.g. S212) */
    if (len == 4u && line[0] == 'S' && line[1] == '2')
    {
        uint8_t d2 = (uint8_t)(line[2] - '0');
        MBusCtrl_InjectSensorState(d2, val);
        DebugPrintf("[INJECT] S2%d%d -> loop2 addr=%d state=%d\r\n", d2, val, d2, val);
        return 0u;
    }

    /* O3<addr>: loop3 RS485 detector online (len==3, e.g. O31) */
    if (len == 3u && line[0] == 'O' && line[1] == '3')
    {
        RS485Detect_SetOnline(val, 1u);
        DebugPrintf("[INJECT] O3%d -> loop3 addr=%d online\r\n", val, val);
        return 0u;
    }

    /* R3<addr><type><state>: loop3 sensor state inject (len==5, e.g. R3122) */
    if (len == 5u && line[0] == 'R' && line[1] == '3')
    {
        uint8_t d3 = (uint8_t)(line[2] - '0');
        uint8_t t3 = (uint8_t)(line[3] - '0');
        RS485Detect_InjectSensorState(d3, t3, val);
        DebugPrintf("[INJECT] R3%d%d%d -> loop3 addr=%d type=%d state=%d\r\n", d3, t3, val, d3, t3, val);
        return 0u;
    }

    /* HAND1/HAND0: hand report press/release (len==5) */
    if (len == 5u && line[0] == 'H' && line[1] == 'A' && line[2] == 'N' && line[3] == 'D')
    {
        hand_paper_state = (val != 0u) ? 0xF0u : 0u;
        DebugPrintf("[INJECT] HAND%d -> hand_paper_state=0x%02X\r\n", val, hand_paper_state);
        return 0u;
    }

    /* PWR<dev><state>: power fault inject (len==5, PWR11 -> zhu_state=1) */
    if (len == 5u && line[0] == 'P' && line[1] == 'W' && line[2] == 'R')
    {
        uint8_t pdev = (uint8_t)(line[3] - '0');
        if (pdev == 1u)
        {
            zhu_state = val;
        }
        else if (pdev == 2u)
        {
            bei_state = val;
        }
        DebugPrintf("[INJECT] PWR%d%d -> power state=%d\r\n", pdev, val, val);
        return 0u;
    }

    return 1u;
}

/* 将DMA接收到的字节按行缓存, 遇到\n触发整行解析 */
static void TestInject_AppendBytes(const uint8_t *src, uint16_t n, char *rx_buf, uint8_t *rx_len_in)
{
    uint16_t i;
    uint8_t rx_len = *rx_len_in;

    for (i = 0u; i < n; i++)
    {
        uint8_t b = src[i];

        if (rx_len < (TEST_INJECT_BUF_LEN - 1u))
        {
            rx_buf[rx_len++] = (char)b;
        }
        else
        {
            /* 缓冲溢出: 清空已有数据, 从当前字节重新开始接收 */
            rx_len = 0u;
            rx_buf[rx_len++] = (char)b;
        }

        /* 换行判定: \n (兼容 \r\n) */
        if (b == '\n')
        {
            uint8_t cmd_len = rx_len;
            /* 去掉末尾多余的 \r 或 \n */
            while (cmd_len > 0u && (rx_buf[cmd_len - 1u] == '\n' || rx_buf[cmd_len - 1u] == '\r'))
            {
                cmd_len--;
            }
            rx_buf[cmd_len] = '\0';

            if (cmd_len > 0u)
            {
                if (TestInject_ParseLine(rx_buf, cmd_len) != 0u)
                {
                    DebugPrintf("[INJECT] UNKNOWN: '%s'\r\n", rx_buf);
                }
            }
            rx_len = 0u;
        }
    }

    *rx_len_in = rx_len;
}

void TestInjectTask(void *argument)
{
    char rx_buf[TEST_INJECT_BUF_LEN];
    uint8_t rx_len = 0u;

    (void)argument;

    DebugPrintf("TestInjectTask started\r\n");

    for (;;)
    {
        /* 轮询uartbuff[3] (UART4 DMA RX) 是否有新数据到达 */
        if (uartbuff[TEST_INJECT_UART_IDX].recepetion_flag != 0u)
        {
            uint16_t n = uartbuff[TEST_INJECT_UART_IDX].recepetion_len;
            if (n > BUFF_MAX) n = BUFF_MAX;
            /* 读取后立即清空flag, 防止DMA IRQ期间重复处理 */
            TestInject_AppendBytes(uartbuff[TEST_INJECT_UART_IDX].recepetion_buff, n, rx_buf, &rx_len);
            uartbuff[TEST_INJECT_UART_IDX].recepetion_flag = 0u;
        }

        vTaskDelay(TEST_INJECT_TASK_PERIOD_MS);
    }
}
