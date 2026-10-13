/*
 * 专用项目喷洒联动策略。
 *
 * 设备码19启动后非阻塞延时30秒，延时内停止可取消；延时结束后通过
 * MBusCtrl_Request()控制回路2地址1的FCM-1011输出1和地址2的FCM-1012两路输出。
 * 输出目标在总复位前保持，设备暂时掉线时按限速节拍重试，不直接操作USART2。
 */

#include "bsp_spray_output_policy.h"

#include "bsp_mbus_control.h"
#include "bsp_save_ctrl.h"
#include "cmsis_os.h"

#define SPRAY_POLICY_RETRY_INTERVAL_MS        1000U

#if XR5000_PROJECT_SPRAY_DELAY_ENABLE
typedef enum
{
    SPRAY_POLICY_IDLE = 0,
    SPRAY_POLICY_DELAYING,
    SPRAY_POLICY_LATCHED
} SprayPolicyState;

static SprayPolicyState g_spray_policy_state;
static uint32_t g_spray_delay_start_tick;
static uint32_t g_spray_retry_tick_1011;
static uint32_t g_spray_retry_tick_1012;
static uint8_t g_spray_1011_request_accepted;
static uint8_t g_spray_1012_request_accepted;
static uint8_t g_spray_start_recorded;
static uint8_t SprayOutputPolicy_CanRetry(uint8_t address, uint32_t now, uint32_t last_tick)
{
    MBusCtrlStatus status = MBusCtrl_GetStatus(address);
    if(status == MBUS_CTRL_STATUS_PENDING || status == MBUS_CTRL_STATUS_SENDING) return 0U;
    return (last_tick == 0U || (uint32_t)(now - last_tick) >= SPRAY_POLICY_RETRY_INTERVAL_MS) ? 1U : 0U;
}

static uint8_t SprayOutputPolicy_Request(uint8_t address, uint32_t outputs)
{
    MBusCtrlRequest request = {0};
    request.addr = address;
    request.operation = MBUS_OPERATION_SET_OUTPUT;
    request.target_mask = outputs;
    request.target_value = outputs;
    return MBusCtrl_Request(&request) == MBUS_CTRL_ACCEPTED ? 1U : 0U;
}

static uint8_t SprayOutputPolicy_IsStarted(uint8_t address, uint32_t outputs)
{
    uint8_t state;
    if((outputs & DEVICE_OUTPUT_1) != 0U)
    {
        if(MBusCtrl_GetOutputChannelState(address, 1U, &state) == 0U || state != 1U) return 0U;
    }
    if((outputs & DEVICE_OUTPUT_2) != 0U)
    {
        if(MBusCtrl_GetOutputChannelState(address, 2U, &state) == 0U || state != 1U) return 0U;
    }
    return 1U;
}
#endif

void SprayOutputPolicy_Reset(void)
{
#if XR5000_PROJECT_SPRAY_DELAY_ENABLE
    g_spray_policy_state = SPRAY_POLICY_IDLE;
    g_spray_delay_start_tick = 0U;
    g_spray_retry_tick_1011 = 0U;
    g_spray_retry_tick_1012 = 0U;
    g_spray_1011_request_accepted = 0U;
    g_spray_1012_request_accepted = 0U;
    g_spray_start_recorded = 0U;
#endif
}

void SprayOutputPolicy_Init(void)
{
    SprayOutputPolicy_Reset();
}

void SprayOutputPolicy_OnButtonState(uint8_t address, uint8_t state)
{
#if XR5000_PROJECT_SPRAY_DELAY_ENABLE
    if(state != 0U)
    {
        if(g_spray_policy_state == SPRAY_POLICY_IDLE)
        {
            g_spray_policy_state = SPRAY_POLICY_DELAYING;
            g_spray_delay_start_tick = osKernelGetTickCount();
            BspCommonDataSaveApp(GASER_FLASH_SAVE, SPRAY_EMERGENCY_START,
                                 MBUS_CONTROL_FLASH_ID, address);
        }
    }
    else if(g_spray_policy_state == SPRAY_POLICY_DELAYING)
    {
        uint32_t now = osKernelGetTickCount();
        if((uint32_t)(now - g_spray_delay_start_tick) >= XR5000_PROJECT_SPRAY_DELAY_MS)
        {
            g_spray_policy_state = SPRAY_POLICY_LATCHED;
            g_spray_retry_tick_1011 = 0U;
            g_spray_retry_tick_1012 = 0U;
            return;
        }
        g_spray_policy_state = SPRAY_POLICY_IDLE;
        g_spray_delay_start_tick = 0U;
        BspCommonDataSaveApp(GASER_FLASH_SAVE, SPRAY_EMERGENCY_STOP,
                             MBUS_CONTROL_FLASH_ID, address);
    }
#else
    (void)address;
    (void)state;
#endif
}

void SprayOutputPolicy_Process(void)
{
#if XR5000_PROJECT_SPRAY_DELAY_ENABLE
    uint32_t now = osKernelGetTickCount();

    if(g_spray_policy_state == SPRAY_POLICY_DELAYING &&
       (uint32_t)(now - g_spray_delay_start_tick) >= XR5000_PROJECT_SPRAY_DELAY_MS)
    {
        g_spray_policy_state = SPRAY_POLICY_LATCHED;
        g_spray_retry_tick_1011 = 0U;
        g_spray_retry_tick_1012 = 0U;
    }
    if(g_spray_policy_state != SPRAY_POLICY_LATCHED) return;

    if(SprayOutputPolicy_IsStarted(XR5000_PROJECT_FCM1011_ADDRESS, DEVICE_OUTPUT_1) != 0U)
        g_spray_1011_request_accepted = 1U;
    else if(SprayOutputPolicy_CanRetry(XR5000_PROJECT_FCM1011_ADDRESS,
                                      now, g_spray_retry_tick_1011) != 0U)
    {
        g_spray_retry_tick_1011 = now;
        if(SprayOutputPolicy_Request(XR5000_PROJECT_FCM1011_ADDRESS,
                                     DEVICE_OUTPUT_1) != 0U)
            g_spray_1011_request_accepted = 1U;
    }

    if(SprayOutputPolicy_IsStarted(XR5000_PROJECT_FCM1012_ADDRESS,
                                   DEVICE_OUTPUT_1 | DEVICE_OUTPUT_2) != 0U)
        g_spray_1012_request_accepted = 1U;
    else if(SprayOutputPolicy_CanRetry(XR5000_PROJECT_FCM1012_ADDRESS,
                                      now, g_spray_retry_tick_1012) != 0U)
    {
        g_spray_retry_tick_1012 = now;
        if(SprayOutputPolicy_Request(XR5000_PROJECT_FCM1012_ADDRESS,
                                     DEVICE_OUTPUT_1 | DEVICE_OUTPUT_2) != 0U)
            g_spray_1012_request_accepted = 1U;
    }

    if(g_spray_start_recorded == 0U &&
       g_spray_1011_request_accepted != 0U &&
       g_spray_1012_request_accepted != 0U)
    {
        g_spray_start_recorded = 1U;
        BspCommonDataSaveApp(GASER_FLASH_SAVE, SPRAY_OUTPUT_START,
                             MBUS_CONTROL_FLASH_ID, XR5000_PROJECT_FCM1011_ADDRESS);
    }
#endif
}
