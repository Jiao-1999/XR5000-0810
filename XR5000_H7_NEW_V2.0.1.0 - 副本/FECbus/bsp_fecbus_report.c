/**
 * @file    bsp_fecbus_report.c
 * @brief   FECbus 事件上报接入层实现 - 封装火警/故障/反馈等事件的 FECbus 上报
 *
 * @details
 *   本文件为 cmd_process.c 等业务代码提供统一的 FECbus 上报接入接口,
 *   避免在各业务点直接调用 Fecbus_QueueEvent 底层 API, 实现接入逻辑与业务代码解耦.
 *
 *   接入点清单与 bsp_storage_event.c 21个接入点完全一致:
 *     火警: cmd_process.c L2861/L5916/L11889/L11914 (4点)
 *     故障: cmd_process.c L11880/L11897/L11904/L11905/L11922/L11927 (6点)
 *     反馈: cmd_process.c L7147 (1点)
 *     复位: cmd_process.c BspCmdProcessInit 末尾 (1点)
 *     手自动: bsp_internal_board.c L705/L714/L727/L736/L749/L758 (6点)
 *     启动: bsp_internal_board.c L694 (1点)
 *     屏蔽: bsp_device_disable.c DeviceDisableSet/Clear (2点)
 *
 *   修改指引:
 *     1. 火警接入: 在触发点 StorageEvent_LogFire 旁追加
 *        FecbusReport_Fire(dev_no, DEV_TYPE_xxx, 1, 0);
 *     2. 故障接入: 在触发点 StorageEvent_LogFault 旁追加
 *        FecbusReport_Fault(dev_no, DEV_TYPE_xxx, 1, 0, is_recover);
 *     3. 反馈接入: 在触发点 StorageEvent_LogFeedback 旁追加
 *        FecbusReport_Feedback(dev_no, DEV_TYPE_xxx, state_code);
 *     4. 手自动接入: 在切换点 StorageEvent_LogManualAuto 旁追加
 *        FecbusReport_ManualAuto(dev_no, is_manual);
 *     5. 屏蔽接入: 在屏蔽/解除成功后 StorageEvent_LogShield 旁追加
 *        FecbusReport_Shield(dev_no, dev_type, is_release);
 *     6. 启动接入: 在启动触发点 StorageEvent_LogStart 旁追加
 *        FecbusReport_Start(dev_no, dev_type);
 *     7. 复位接入: 在 BspCmdProcessInit 末尾 StorageEvent_LogReset 旁追加
 *        FecbusReport_Reset();
 *     8. 设备类型代码: 见 bsp_storage_tx.h 的 DEV_TYPE_xxx 宏
 *     9. 事件代码:     见 bsp_storage_tx.h 的 EVT_xxx 宏
 */
#include "bsp_fecbus_report.h"
#include "bsp_fecbus.h"

/*==============================================================
 * 内部辅助
 *============================================================*/

/**
 * @brief  构造事件项并异步入队
 * @param  func_code: 功能码 (5/6/7/1/2/3)
 * @param  pa:        优先级 (FECBUS_PA_URGENT/IMPORTANT/NORMAL)
 * @param  dev_no:    设备号
 * @param  dev_type:  设备类型代码
 * @param  unit_no:  单元号
 * @param  channel_no: 通道号
 * @param  event_code: 事件代码
 * @param  state_code: 状态代码
 * @note   默认 da=0 (广播), 由 Fecbus_QueueEvent 异步入队(非阻塞).
 */
static void FecbusReport_Enqueue(uint8_t func_code, uint8_t pa,
                                  uint8_t dev_no, uint16_t dev_type,
                                  uint8_t unit_no, uint8_t channel_no,
                                  uint16_t event_code, uint16_t state_code)
{
    FecbusEventItem_t item;
    item.func_code = func_code;
    item.da        = FECBUS_DA_BROADCAST;  /* 默认广播 */
    item.pa        = pa;
    item.dev_no    = dev_no;
    item.unit_no   = unit_no;
    item.channel_no = channel_no;
    item.dev_type  = dev_type;
    item.event_code = event_code;
    item.state_code = state_code;

    (void)Fecbus_QueueEvent(&item);
}

/*==============================================================
 * 公开 API
 *============================================================*/

/**
 * @brief  上报火警事件 (功能码5, 紧急, PA=1)
 */
void FecbusReport_Fire(uint8_t dev_no, uint16_t dev_type,
                       uint8_t unit_no, uint8_t channel_no)
{
    FecbusReport_Enqueue(FECBUS_FUNC_URGENT_EVT, FECBUS_PA_URGENT,
                         dev_no, dev_type, unit_no, channel_no,
                         EVT_FIRE, 0);
}

/**
 * @brief  上报故障事件 (功能码6, 一般, PA=3)
 */
void FecbusReport_Fault(uint8_t dev_no, uint16_t dev_type,
                        uint8_t unit_no, uint8_t channel_no,
                        uint8_t is_recover)
{
    uint16_t evt = (is_recover != 0) ? EVT_FAULT_RECOVER : EVT_FAULT;
    FecbusReport_Enqueue(FECBUS_FUNC_NORMAL_EVT, FECBUS_PA_NORMAL,
                         dev_no, dev_type, unit_no, channel_no,
                         evt, 0);
}

/**
 * @brief  上报反馈事件 (功能码5, 紧急, PA=1)
 */
void FecbusReport_Feedback(uint8_t dev_no, uint16_t dev_type,
                            uint16_t state_code)
{
    FecbusReport_Enqueue(FECBUS_FUNC_URGENT_EVT, FECBUS_PA_URGENT,
                         dev_no, dev_type, 1, 0,
                         EVT_FEEDBACK, state_code);
}

/**
 * @brief  上报手动/自动切换事件 (功能码5, 紧急, PA=1)
 */
void FecbusReport_ManualAuto(uint8_t dev_no, uint8_t is_manual)
{
    uint16_t evt = (is_manual != 0) ? EVT_MANUAL : EVT_AUTO;
    FecbusReport_Enqueue(FECBUS_FUNC_URGENT_EVT, FECBUS_PA_URGENT,
                         dev_no, DEV_TYPE_CONTROL_DEV, 1, 0,
                         evt, 0);
}

/**
 * @brief  上报屏蔽/解除屏蔽事件 (功能码6, 一般, PA=3)
 */
void FecbusReport_Shield(uint8_t dev_no, uint16_t dev_type, uint8_t is_release)
{
    uint16_t evt = (is_release != 0) ? EVT_SHIELD_RELEASE : EVT_SHIELD;
    FecbusReport_Enqueue(FECBUS_FUNC_NORMAL_EVT, FECBUS_PA_NORMAL,
                         dev_no, dev_type, 1, 0,
                         evt, 0);
}

/**
 * @brief  上报启动事件 (功能码5, 紧急, PA=1)
 */
void FecbusReport_Start(uint8_t dev_no, uint16_t dev_type)
{
    FecbusReport_Enqueue(FECBUS_FUNC_URGENT_EVT, FECBUS_PA_URGENT,
                         dev_no, dev_type, 1, 0,
                         EVT_START, 0);
}

/**
 * @brief  上报系统复位事件 (功能码1, 广播, PA=1)
 */
void FecbusReport_Reset(void)
{
    FecbusReport_Enqueue(FECBUS_FUNC_RESET, FECBUS_PA_URGENT,
                         1, DEV_TYPE_CONTROLLER, 1, 0,
                         EVT_NORMAL, 0);
}

/**
 * @brief  上报系统消音事件 (功能码2, 广播, PA=1)
 */
void FecbusReport_Silence(void)
{
    FecbusReport_Enqueue(FECBUS_FUNC_SILENCE, FECBUS_PA_URGENT,
                         1, DEV_TYPE_CONTROLLER, 1, 0,
                         EVT_NORMAL, 0);
}

/**
 * @brief  上报系统自检事件 (功能码3, 广播, PA=1)
 */
void FecbusReport_SelfCheck(void)
{
    FecbusReport_Enqueue(FECBUS_FUNC_SELFTEST, FECBUS_PA_URGENT,
                         1, DEV_TYPE_CONTROLLER, 1, 0,
                         EVT_NORMAL, 0);
}
