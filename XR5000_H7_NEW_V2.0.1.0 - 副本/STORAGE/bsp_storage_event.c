/**
 * @file    bsp_storage_event.c
 * @brief   黑匣子存储事件接入层 - 封装火警/故障/反馈事件记录
 *
 * @details
 *   本文件为 cmd_process.c 等业务代码提供统一的黑匣子存储接入接口,
 *   避免在各业务点直接调用 StorageTx 底层API, 实现接入逻辑与业务代码解耦.
 *
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │                    接入点清单                                │
 *   ├──────────────────────────────────────────────────────────────┤
 *   │ 火警 (StorageEvent_LogFire, 0x02首警+0x03火警):              │
 *   │   1. cmd_process.c ~2861行  手报按下                         │
 *   │      dev_no=HANDPOT_Package_ID, dev_type=DEV_TYPE_HAND_REPORT│
 *   │   2. cmd_process.c ~5916行  舱内复合火警(新增舱记录后)       │
 *   │      dev_no=jsz(舱号), dev_type=DEV_TYPE_FIRE_ALARM          │
 *   │   3. cmd_process.c ~11889行 Loop1温度火警                    │
 *   │      dev_no=addr, dev_type=DEV_TYPE_TEMPERATURE              │
 *   │   4. cmd_process.c ~11914行 Loop1烟雾火警                    │
 *   │      dev_no=addr, dev_type=DEV_TYPE_SMOKE                    │
 *   │                                                              │
 *   │ 故障 (StorageEvent_LogFault, 0x04故障独立区段):              │
 *   │   5. cmd_process.c ~11880行 温度传感器故障恢复 is_recover=1  │
 *   │      dev_no=addr, dev_type=DEV_TYPE_TEMPERATURE              │
 *   │   6. cmd_process.c ~11897行 温度传感器故障     is_recover=0  │
 *   │      dev_no=addr, dev_type=DEV_TYPE_TEMPERATURE              │
 *   │   7. cmd_process.c ~11904行 烟雾污染故障恢复   is_recover=1  │
 *   │      dev_no=addr, dev_type=DEV_TYPE_SMOKE                    │
 *   │   8. cmd_process.c ~11905行 烟雾传感器故障恢复 is_recover=1  │
 *   │      dev_no=addr, dev_type=DEV_TYPE_SMOKE                    │
 *   │   9. cmd_process.c ~11922行 烟雾污染故障       is_recover=0  │
 *   │      dev_no=addr, dev_type=DEV_TYPE_SMOKE                    │
 *   │  10. cmd_process.c ~11927行 烟雾传感器故障     is_recover=0  │
 *   │      dev_no=addr, dev_type=DEV_TYPE_SMOKE                    │
 *   │                                                              │
 *   │ 反馈 (StorageEvent_LogFeedback, 0x01普通区段):              │
 *   │  11. cmd_process.c ~7147行 反馈1触发(getFeedBack1State==0x0F)│
 *   │      dev_no=FEEDBK1_Package_ID, dev_type=DEV_TYPE_CONTROL_DEV│
 *   │                                                              │
 *   │ 复位 (StorageEvent_ResetFirstFire + LogReset):               │
 *   │  12. cmd_process.c BspCmdProcessInit() 末尾 (~1071行)        │
 *   │      ResetFirstFire() 清首警标志, LogReset() 记录复位事件    │
 *   │                                                              │
 *   │ 手动/自动切换 (StorageEvent_LogManualAuto, 0x01普通区段):    │
 *   │  13. bsp_internal_board.c ~705行 系统切换手动                │
 *   │      dev_no=SYS_HAND_AUTO_Package_ID, is_manual=1            │
 *   │  14. bsp_internal_board.c ~714行 系统切换自动                │
 *   │      dev_no=SYS_HAND_AUTO_Package_ID, is_manual=0            │
 *   │  15. bsp_internal_board.c ~727行 分区1切换手动               │
 *   │      dev_no=PART1_HAND_AUTO_Package_ID, is_manual=1          │
 *   │  16. bsp_internal_board.c ~736行 分区1切换自动               │
 *   │      dev_no=PART1_HAND_AUTO_Package_ID, is_manual=0          │
 *   │  17. bsp_internal_board.c ~749行 分区2切换手动               │
 *   │      dev_no=PART2_HAND_AUTO_Package_ID, is_manual=1          │
 *   │  18. bsp_internal_board.c ~758行 分区2切换自动               │
 *   │      dev_no=PART2_HAND_AUTO_Package_ID, is_manual=0          │
 *   │                                                              │
 *   │ 联动启动按键 (StorageEvent_LogStart, 0x01普通区段):          │
 *   │  19. bsp_internal_board.c ~694行 KEY_SYSTEM_LINKAGE_S按下    │
 *   │      dev_no=LINKAGE_CLUSTER_ID, dev_type=DEV_TYPE_CONTROL_DEV│
 *   │                                                              │
 *   │ 屏蔽/解除屏蔽 (StorageEvent_LogShield, 0x01普通区段):        │
 *   │  20. bsp_device_disable.c DeviceDisableSet() 返回OK前        │
 *   │      dev_no=identity->address, dev_type=按DeviceRegistryType │
 *   │      映射, is_release=0                                      │
 *   │  21. bsp_device_disable.c DeviceDisableClear() 返回OK前      │
 *   │      dev_no=identity->address, dev_type=按DeviceRegistryType │
 *   │      映射, is_release=1                                      │
 *   └──────────────────────────────────────────────────────────────┘
 *
 *   修改指引:
 *     1. 新增火警接入: 在触发点调用
 *        StorageEvent_LogFire(dev_no, DEV_TYPE_xxx, 1, 0);
 *     2. 新增故障接入: 在触发点调用
 *        StorageEvent_LogFault(dev_no, DEV_TYPE_xxx, 1, 0, is_recover);
 *     3. 新增反馈接入: 在触发点调用
 *        StorageEvent_LogFeedback(dev_no, DEV_TYPE_xxx, state_code);
 *     4. 新增手自动接入: 在切换点调用
 *        StorageEvent_LogManualAuto(dev_no, is_manual);
 *     5. 新增屏蔽接入: 在屏蔽/解除成功后调用
 *        StorageEvent_LogShield(dev_no, dev_type, is_release);
 *     6. 新增启动接入: 在启动触发点调用
 *        StorageEvent_LogStart(dev_no, dev_type);
 *     7. 设备类型代码: 见 bsp_storage_tx.h 的 DEV_TYPE_xxx 宏
 *     8. 事件代码:     见 bsp_storage_tx.h 的 EVT_xxx 宏
 *     9. 命令码分配:   0x01普通/0x02首警/0x03火警/0x04故障
 */
#include "bsp_storage_event.h"
#include <string.h>

/*==============================================================
 * 内部状态
 *============================================================*/
/* 首警已记录标志: 0=未记录(下次火警将先发0x02首警), 1=已记录 */
static uint8_t s_first_fire_recorded = 0;

/*==============================================================
 * 内部函数
 *============================================================*/
/**
 * @brief  构造一条事件记录并异步入队
 * @param  cmd:         命令码(STX_CMD_STORE_xxx)
 * @param  dev_no:      设备号
 * @param  dev_type:    设备类型代码
 * @param  unit_no:     单元号
 * @param  channel_no:  通道号
 * @param  event_code:  事件代码(EVT_xxx)
 * @param  state_code:  状态代码
 * @note   填充 controller_no=1, 自动调 FillTimestamp 填入RTC时间,
 *         调用 QueueRecord 异步入队(非阻塞, 队列满则丢最旧).
 */
static void StorageEvent_Enqueue(uint8_t cmd,
                                 uint8_t dev_no, uint16_t dev_type,
                                 uint8_t unit_no, uint8_t channel_no,
                                 uint16_t event_code, uint16_t state_code)
{
    EventRecord_t rec;

    memset(&rec, 0, sizeof(EventRecord_t));
    rec.controller_no = 1;          /* 控制器号固定1 */
    rec.unit_no       = unit_no;
    rec.device_no     = dev_no;
    rec.channel_no    = channel_no;
    rec.dev_type      = dev_type;
    rec.event_code    = event_code;
    rec.state_code    = state_code;
    StorageTx_FillTimestamp(&rec);  /* 填充RTC时间戳 */
    if (state_code == 0U)
    {
        /* P1-2整改: 调用方未指定状态位图时, 按事件码自动填充(表C.18)
         * (调用方显式传入非0值则保留, 如LogFeedback的外部state_code) */
        StorageTx_FillStateMask(&rec);
    }

    (void)StorageTx_QueueRecord(cmd, &rec);  /* 异步入队, 忽略返回值 */
}

/*==============================================================
 * 公开API
 *============================================================*/

void StorageEvent_LogFire(uint8_t dev_no, uint16_t dev_type,
                          uint8_t unit_no, uint8_t channel_no)
{
    /* 首警: 首次火警先发0x02(首警独立区段), 再发0x03(火警独立区段) */
    if (s_first_fire_recorded == 0U) {
        s_first_fire_recorded = 1U;
        StorageEvent_Enqueue(STX_CMD_STORE_FIRST_ALARM,
                             dev_no, dev_type, unit_no, channel_no,
                             EVT_FIRST_FIRE, 0U);
    }
    /* 火警: 每次都发0x03(火警独立区段) */
    StorageEvent_Enqueue(STX_CMD_STORE_FIRE_ALARM,
                         dev_no, dev_type, unit_no, channel_no,
                         EVT_FIRE, 0U);
}

void StorageEvent_LogFault(uint8_t dev_no, uint16_t dev_type,
                           uint8_t unit_no, uint8_t channel_no,
                           uint8_t is_recover)
{
    uint16_t event_code = (is_recover != 0U) ? EVT_FAULT_RECOVER : EVT_FAULT;

    /* 故障发生/恢复均存入0x04故障独立区段, 不被普通事件覆盖 */
    StorageEvent_Enqueue(STX_CMD_STORE_FAULT,
                         dev_no, dev_type, unit_no, channel_no,
                         event_code, 0U);
}

void StorageEvent_LogFeedback(uint8_t dev_no, uint16_t dev_type,
                              uint16_t state_code)
{
    /* 反馈存入0x01普通区段(先进先出覆盖) */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         dev_no, dev_type, 1U, 0U,
                         EVT_FEEDBACK, state_code);
}

void StorageEvent_ResetFirstFire(void)
{
    s_first_fire_recorded = 0U;
}

void StorageEvent_LogManualAuto(uint8_t dev_no, uint8_t is_manual)
{
    uint16_t event_code = (is_manual != 0U) ? EVT_MANUAL : EVT_AUTO;

    /* 手自动切换存入0x01普通区段, dev_type固定为控制设备 */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         dev_no, DEV_TYPE_CONTROL_DEV, 1U, 0U,
                         event_code, 0U);
}

void StorageEvent_LogShield(uint8_t dev_no, uint16_t dev_type, uint8_t is_release)
{
    uint16_t event_code = (is_release != 0U) ? EVT_SHIELD_RELEASE : EVT_SHIELD;

    /* 屏蔽/解除屏蔽存入0x01普通区段 */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         dev_no, dev_type, 1U, 0U,
                         event_code, 0U);
}

void StorageEvent_LogStart(uint8_t dev_no, uint16_t dev_type)
{
    /* 联动启动按键存入0x01普通区段 */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         dev_no, dev_type, 1U, 0U,
                         EVT_START, 0U);
}

void StorageEvent_LogReset(void)
{
    /* 系统复位: 控制器号1, EVT_RESET(122), state=0, 存入0x01普通区段
     * (规范B.1.2.2仅要求首警/火警/故障独立记录, 复位属其他运行状态信息)
     * 注意: 复位时StorageTx可能未就绪, 入队失败可接受 */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         1U, DEV_TYPE_CONTROLLER, 1U, 0U,
                         EVT_RESET, 0U);
}

/*==============================================================
 * 新增API(GB4717-2024附录B.1.1.1整改, 2026-08-24)
 *============================================================*/

void StorageEvent_LogPowerOn(void)
{
    /* 控制器开机: EVT_POWER_ON(120), 存入0x01普通区段
     * 调用点: freertos.c StartDefaultTask() StorageTx_Init()之后 */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         1U, DEV_TYPE_CONTROLLER, 1U, 0U,
                         EVT_POWER_ON, 0U);
}

void StorageEvent_LogPowerOff(void)
{
    /* 控制器关机: EVT_POWER_OFF(121), 存入0x01普通区段
     * 调用点: bsp_adc.c备电耗尽处 + cmd_process.c PowerManageCtrl主备全失处
     * 关机瞬间异步入队可能来不及发出, 尽力而为 */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         1U, DEV_TYPE_CONTROLLER, 1U, 0U,
                         EVT_POWER_OFF, 0U);
}

void StorageEvent_LogConfirmButton(void)
{
    /* 信息确认按钮动作: EVT_CONFIRM_BUTTON(128), 存入0x01普通区段
     * 调用点: bsp_internal_board.c KEY1_INFORM_CERTAIN case */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         1U, DEV_TYPE_CONTROLLER, 1U, 0U,
                         EVT_CONFIRM_BUTTON, 0U);
}

void StorageEvent_LogCheckButton(void)
{
    /* 检查功能按钮动作: EVT_CHECK_BUTTON(129), 存入0x01普通区段
     * 调用点: cmd_process.c UpdateUI() check_record_pending消费点(复用去重) */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         1U, DEV_TYPE_CONTROLLER, 1U, 0U,
                         EVT_CHECK_BUTTON, 0U);
}

void StorageEvent_LogLinkageStartButton(uint8_t dev_no, uint16_t dev_type)
{
    /* 联动启动按钮动作: EVT_LINKAGE_START_BUTTON(130), 存入0x01普通区段
     * 与StorageEvent_LogStart(19=设备已启动)并存: 130=按下动作, 19=启动结果
     * 调用点: bsp_internal_board.c KEY_SYSTEM_LINKAGE_S case */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         dev_no, dev_type, 1U, 0U,
                         EVT_LINKAGE_START_BUTTON, 0U);
}

void StorageEvent_LogClockAdjust(void)
{
    /* 时钟调整: EVT_CLOCK_ADJUST(131), 存入0x01普通区段
     * 调用点: bsp_screen.c InternalScreenRTCSetting() 画面41六个控件,
     * 每修改一个字段记录一条; 时间戳由FillTimestamp实时读RTC(即调整后新值) */
    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         1U, DEV_TYPE_CONTROLLER, 1U, 0U,
                         EVT_CLOCK_ADJUST, 0U);
}

void StorageEvent_LogSelfCheck(uint8_t is_fail)
{
    /* 自检/自检失败: EVT_SELF_CHECK(123)/EVT_SELF_CHECK_FAIL(124), 存入0x01普通区段
     * 调用点: cmd_process.c 密码页53 SELFCHECK_KEY case */
    uint16_t event_code = (is_fail != 0U) ? EVT_SELF_CHECK_FAIL : EVT_SELF_CHECK;

    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         1U, DEV_TYPE_CONTROLLER, 1U, 0U,
                         event_code, 0U);
}

void StorageEvent_LogSupervise(uint8_t dev_no, uint16_t dev_type,
                               uint8_t is_release)
{
    /* 监管/监管解除: EVT_SUPERVISED(70)/EVT_SUPERVISED_RELEASE(71),
     * 存入0x01普通区段. 预留不接线: XR5000三回路均无监管类信号源,
     * 本API仅补齐供型式试验查表, 待监管设备协议接入后挂到raw_state解析处 */
    uint16_t event_code = (is_release != 0U) ? EVT_SUPERVISED_RELEASE : EVT_SUPERVISED;

    StorageEvent_Enqueue(STX_CMD_STORE_EVENT,
                         dev_no, dev_type, 1U, 0U,
                         event_code, 0U);
}
