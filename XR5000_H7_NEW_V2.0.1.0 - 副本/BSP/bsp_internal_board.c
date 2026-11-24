#include "bsp_internal_board.h"

#include "cmsis_os.h"
#include "cmd_process.h"
#include "bsp_debug.h"

#include "bsp_rs485_01.h"

#include "bsp_relay.h"

#include "system.h"

#include "bsp_adc.h"

#include "bsp_save_ctrl.h"
#include "bsp_printer.h"

#include "bsp_storage_event.h"  /* 黑匣子存储事件接入层 */
#include "bsp_fecbus_report.h" /* FECbus RS485 上报接入层 (GB4717 附录C) */

#define InternalBoardAddr 1U

/* 检修开关状态(V6新增): 1=检修 0=运行; 取自按键板上报Bit4, 不落盘, 上电默认运行 */
uint8_t g_maintenance_state = 0U;

#define getMainPowerState()   (zhu_state           ? 1 : 0)

#define getSparePowerState() \
    ((bei_state == open_circuit || bei_state == short_circuit) ? 0 : \
    ((bei_state == normal_charge || bei_state == discharge) ? 1 : \
    (bei_state ? 1 : 0)))


#define getFireAlarmState()   (fire_alarm_state    ? 1 : 0)
#define getDisconnectState()  ((disconnect_state || linkage_disconnect_state)    ? 1 : 0)
#define getSilencersState()   (silencers_state     ? 1 : 0)
#define getSystemFaultState() (system_fault_state  ? 1 : 0)
#define getInitiationState()  (sys_start_state     ? 1 : 0)
#define getStartDelayState()  (start_delay_state   ? 1 : 0)
#define getFeedbackedState()  ((feedbacked_state || system_feedback_source_mask) ? 1 : 0)
#define getRegulAlarmState()  (regul_alarm_state   ? 1 : 0)
#define getShieldingState()   (shielding_state     ? 1 : 0)
#define getSirenStartState()  (siren_start_state   ? 1 : 0)
#define getSirenFaultState()  (siren_fault_state   ? 1 : 0)
#define getSelfCheckState()   (self_check_state    ? 1 : 0)
// 分区1 状态
#define getPart1StartState()  (part_1_start_state  ? 1 : 0)
#define getPart1DelayState()  (part_1_start_delay  ? 1 : 0)
#define getPart1SprayState()  (part_1_spray_state  ? 1 : 0)
#define getPart1FeedbackState()  (part_1_feedback  ? 1 : 0)
#define getPart1SoundState()  (part_1_sound_light  ? 1 : 0)
#define getPart1FaultState()  (part_1_fault_state  ? 1 : 0)
// 分区2 状态
#define getPart2StartState()  (part_2_start_state  ? 1 : 0)
#define getPart2DelayState()  (part_2_start_delay  ? 1 : 0)
#define getPart2SprayState()  (part_2_spray_state  ? 1 : 0)
#define getPart2FeedbackState()  (part_2_feedback  ? 1 : 0)
#define getPart2SoundState()  (part_2_sound_light  ? 1 : 0)
#define getPart2FaultState()  (part_2_fault_state  ? 1 : 0)
// 备用/特殊 LED状态
#define getSpareGasCheck()    (spare_gas_selfcheck ? 1 : 0)
#define getSpareGasAlarm()    (spare_gas_alarm     ? 1 : 0)
#define getSpareGasFault()    (spare_gas_fault     ? 1 : 0)
#define getSpareGasDelay()    (spare_gas_delay     ? 1 : 0)
#define getSpare_1()          (spare_1_led         ? 1 : 0)
#define getSpare_2()          (spare_2_led         ? 1 : 0)
#define getSpare_3()          (spare_3_led         ? 1 : 0)

#define getSpecialCommun()    (special_communic    ? 1 : 0)
#define getSpecialRuning()    (special_running     ? 1 : 0)

//#define getBuzzerWorkState() (beep_fire_ctrl      ? 1 : \
//                             beep_fault_ctrl      ? 2 : \
//                             main_power_beep_ctrl ? 3 : \
//                             linkage_beep_ctrl    ? 3 : 0)
														 
#define getBuzzerWorkState() (beep_spray_feedback_ctrl ? 1 :  \
															beep_fire_ctrl           ? 3 :  \
                              beep_fault_ctrl          ? 2 :  \
                              main_power_beep_ctrl     ? 2 :  \
                              linkage_beep_ctrl        ? 2 :  \
															beep_general_io_ctrl     ? 2 : 0)

// LED 类型枚举
typedef enum {
  LED_MAIN_POWER,  // 寄存器0    主电
	LED_BACKUP_POWER,  // 寄存器1    备电
	LED_FIRE_ALARM,  // 寄存器2    火警
	LED_DISCONNECT_FAULT,  // 寄存器3
	LED_SILENCE,  // 寄存器4
	LED_SYSTEM_FAULT,  // 寄存器5
	LED_SYSTEM_START_UP,  // 寄存器6
	LED_SYSTEM_START_DELAY,  // 寄存器7
	LED_SYSTEM_FEEDBACK,  // 寄存器8
	LED_SYSTEM_REGULAR_ALARM,  // 寄存器9
	LED_SYSTEM_SHIELD,  // 寄存器10
	LED_SIREN_STARTUP,  // 寄存器11
	LED_SIREN_FAULT,  // 寄存器12
	LED_SYSTEM_SELF_CHECK,  // 寄存器13
	LED_PART_STARTUP_1,  // 寄存器14
	LED_PART_STARTUP_DELAY_1,  // 寄存器15
	LED_PART_SPRAY_STARTUP_1,  // 寄存器16
	LED_PART_SPRAY_FEEDBACK_1,  // 寄存器17
	LED_PART_SOUND_ALARM_1,  // 寄存器18
	LED_PART_FAULT_1,  // 寄存器19
	LED_PART_STARTUP_2,  // 寄存器20
	LED_PART_STARTUP_DELAY_2,  // 寄存器21
	LED_PART_SPRAY_STARTUP_2,  // 寄存器22
	LED_PART_SPRAY_FEEDBACK_2,  // 寄存器23
	LED_PART_SOUND_ALARM_2,  // 寄存器24
	LED_PART_FAULT_2,  // 寄存器25
	
	// 新增LED寄存器 2025/10/10
	LED_COMBUSTIBLE_GAS_SELF_CHECK,  // 可燃气体自检 // 寄存器26
	LED_COMBUSTIBLE_GAS_ALARM,  // 可燃气体报警 // 寄存器27
	LED_COMBUSTIBLE_GAS_FAULT,  // 可燃气体故障 // 寄存器28
	LED_COMBUSTIBLE_GAS_DELAY,  // 可燃气体延时 // 寄存器29
	LED_STANDBY_1,  // 预留LED1 // 寄存器30
	LED_STANDBY_2,  // 预留LED2 // 寄存器31
	LED_STANDBY_3,  // 预留LED3 // 寄存器32
	// 特殊LED寄存器 2025/10/10
	LED_COMMUNICATION,  // 通信LED // 寄存器33
	LED_WORKING,  // 运行指示灯 // 寄存器34
    
  LED_COUNT  // LED总数
} LED_REGISTER_ID;

// led_val灯的状态值 n第n位 x状态
#define setLedxRegisterBit(led_val, n, x) ( (led_val) |= ((x) << (n)) )

// 独立按键键值定义
typedef enum
{
	KEY_SYSTEM_MANUAL = 0x01,
	KEY_SYSTEM_AUTO   = 0x02,
	
	KEY_PART1_MANUAL  = 0x04,  // 分区1切换手动
	KEY_PART1_AUTO    = 0x08,  // 分区1切换自动
	
	KEY_PART2_MANUAL  = 0x10,  // 分区1切换手动
	KEY_PART2_AUTO    = 0x20,  // 分区1切换自动
	
}BspHandAutoState;

uint8_t fire_alarm_state    = 0;  // 火警状态
uint8_t disconnect_state    = 0;  // 掉线状态
uint8_t silencers_state     = 0;  // 消音状态
uint8_t system_fault_state  = 0;  // 系统故障状态
uint8_t sys_start_state = 0;  // 启动状态
uint8_t start_delay_state   = 0;  // 启动延时状态
uint8_t feedbacked_state    = 0;  // 反馈状态
static uint32_t system_feedback_source_mask = 0U;
uint8_t regul_alarm_state   = 0;  // 监管报警状态
uint8_t shielding_state     = 0;  // 屏蔽状态
uint8_t siren_start_state   = 0;  // 报警器启动状态
uint8_t siren_fault_state   = 0;  // 报警器故障状态
uint8_t self_check_state    = 0;  // 自检状态
// 分区1 状态控制变量
uint8_t part_1_start_state  = 0;
uint8_t part_1_start_delay  = 0;
uint8_t part_1_spray_state  = 0;
uint8_t part_1_feedback     = 0;
uint8_t part_1_sound_light  = 0;
uint8_t part_1_fault_state  = 0;
// 分区2 状态控制变量
uint8_t part_2_start_state  = 0;
uint8_t part_2_start_delay  = 0;
uint8_t part_2_spray_state  = 0;
uint8_t part_2_feedback     = 0;
uint8_t part_2_sound_light  = 0;
uint8_t part_2_fault_state  = 0;

/* 分区延时/喷洒/反馈灯的多来源掩码, 定义见 bsp_internal_board.h */
static uint32_t part_1_delay_src_mask    = 0U;
static uint32_t part_1_spray_src_mask    = 0U;
static uint32_t part_1_feedback_src_mask = 0U;
static uint32_t part_2_delay_src_mask    = 0U;
static uint32_t part_2_spray_src_mask    = 0U;
static uint32_t part_2_feedback_src_mask = 0U;
// 备用LED状态控制变量
uint8_t spare_gas_selfcheck = 0;  // 备用 可燃气体自检
uint8_t spare_gas_alarm     = 0;  // 备用 可燃气体报警
uint8_t spare_gas_fault     = 0;  // 备用 可燃气体故障
uint8_t spare_gas_delay     = 0;  // 备用 可燃气体延时
uint8_t spare_1_led         = 0;  // 备用
uint8_t spare_2_led         = 0;  // 备用
uint8_t spare_3_led         = 0;  // 备用

uint8_t special_communic    = 0;  // 通信LED
uint8_t special_running     = 0;  // 运行LED

// 系统复位显示控制
uint8_t system_reset_flag   = 0;
// [7:4] 历史状态 [3:0] 当前状态
uint8_t screen_show_siren_information = 0;

void LedStateInit(void)
{
	// 主面板LED
	fire_alarm_state    = 0;  // 火警状态
	disconnect_state    = 0;  // 掉线状态
	silencers_state     = 0;  // 消音状态
	system_fault_state  = 0;  // 系统故障状态
	sys_start_state     = 0;  // 启动状态
	start_delay_state   = 0;  // 启动延时状态
	feedbacked_state    = 0;  // 反馈状态
	system_feedback_source_mask = 0U;
	regul_alarm_state   = 0;  // 监管报警状态
	shielding_state     = 0;  // 屏蔽状态
	siren_start_state   = 0;  // 报警器启动状态
	siren_fault_state   = 0;  // 报警器故障状态
	self_check_state    = 0;  // 自检状态
	// 分区1 状态控制变量
	part_1_start_state  = 0;
	part_1_start_delay  = 0;
	part_1_spray_state  = 0;
	part_1_feedback     = 0;
	part_1_sound_light  = 0;
	part_1_fault_state  = 0;
	// 分区2 状态控制变量
	part_2_start_state  = 0;
	part_2_start_delay  = 0;
	part_2_spray_state  = 0;
	part_2_feedback     = 0;
	part_2_sound_light  = 0;
	part_2_fault_state  = 0;

	/* 清分区灯的多来源掩码, 否则复位后残留来源会让灯常亮 */
	part_1_delay_src_mask    = 0U;
	part_1_spray_src_mask    = 0U;
	part_1_feedback_src_mask = 0U;
	part_2_delay_src_mask    = 0U;
	part_2_spray_src_mask    = 0U;
	part_2_feedback_src_mask = 0U;
	
	// 备用LED状态控制变量
	spare_gas_selfcheck = 0;  // 备用 可燃气体自检
	spare_gas_alarm     = 0;  // 备用 可燃气体报警
	spare_gas_fault     = 0;  // 备用 可燃气体故障
	spare_gas_delay     = 0;  // 备用 可燃气体延时
	spare_1_led         = 0;  // 备用
	spare_2_led         = 0;  // 备用
	spare_3_led         = 0;  // 备用
	// 特殊LED
	special_communic    = 0;  // 通信LED
	special_running     = 0;  // 运行LED
	
	// 将手报状态初始化
	screen_show_siren_information = 0;
}

void SysSirenStartLedCtrl(LED_STATE state)
{
	siren_start_state = state;
}

void SysSirenFaultLedCtrl(LED_STATE state)
{
	siren_fault_state = state;
}

void SysStartStateLedCtrl(LED_STATE state)
{
	sys_start_state = state;
}

/* 只更新指定来源，避免一个业务恢复时误熄灭其他反馈来源。 */
void SysFeedbackLedSourceCtrl(uint32_t source_mask, LED_STATE state)
{
	if(state == LED_ON)
		system_feedback_source_mask |= source_mask;
	else
		system_feedback_source_mask &= ~source_mask;
}

// 分区1
void Part1StartLedCtrl(LED_STATE state)
{
	part_1_start_state = state;
}

/* 延时灯: 旧状态机与联动引擎各置自己的源位, 任一置位则亮 */
void Part1StartDelayLedSource(uint32_t source_mask, LED_STATE state)
{
	if(state == LED_ON)
		part_1_delay_src_mask |= source_mask;
	else
		part_1_delay_src_mask &= ~source_mask;
	part_1_start_delay = (part_1_delay_src_mask != 0U) ? 1U : 0U;
}

void Part1StartDelayLedCtrl(LED_STATE state)
{
	Part1StartDelayLedSource(PART_LED_SRC_LEGACY, state);
}

void Part1SprayLedSource(uint32_t source_mask, LED_STATE state)
{
	if(state == LED_ON)
		part_1_spray_src_mask |= source_mask;
	else
		part_1_spray_src_mask &= ~source_mask;
	part_1_spray_state = (part_1_spray_src_mask != 0U) ? 1U : 0U;
}

void Part1SprayLedCtrl(LED_STATE state)
{
	Part1SprayLedSource(PART_LED_SRC_LEGACY, state);
}

void Part1FeedbackLedSource(uint32_t source_mask, LED_STATE state)
{
	if(state == LED_ON)
		part_1_feedback_src_mask |= source_mask;
	else
		part_1_feedback_src_mask &= ~source_mask;
	part_1_feedback = (part_1_feedback_src_mask != 0U) ? 1U : 0U;
}

void Part1FeedbackLedCtrl(LED_STATE state)
{
	Part1FeedbackLedSource(PART_LED_SRC_LEGACY, state);
}

void Part1SoundLightLedCtrl(LED_STATE state)
{
	part_1_sound_light = state;
}

void Part1FaultLedCtrl(LED_STATE state)
{
	part_1_fault_state = state;
}

// 2025/10/28 10:10 添加分区2控制
void Part2StartLedCtrl(LED_STATE state)
{
	part_2_start_state = state;
}

void Part2StartDelayLedSource(uint32_t source_mask, LED_STATE state)
{
	if(state == LED_ON)
		part_2_delay_src_mask |= source_mask;
	else
		part_2_delay_src_mask &= ~source_mask;
	part_2_start_delay = (part_2_delay_src_mask != 0U) ? 1U : 0U;
}

void Part2StartDelayLedCtrl(LED_STATE state)
{
	Part2StartDelayLedSource(PART_LED_SRC_LEGACY, state);
}

void Part2SprayLedSource(uint32_t source_mask, LED_STATE state)
{
	if(state == LED_ON)
		part_2_spray_src_mask |= source_mask;
	else
		part_2_spray_src_mask &= ~source_mask;
	part_2_spray_state = (part_2_spray_src_mask != 0U) ? 1U : 0U;
}

void Part2SprayLedCtrl(LED_STATE state)
{
	Part2SprayLedSource(PART_LED_SRC_LEGACY, state);
}

void Part2FeedbackLedSource(uint32_t source_mask, LED_STATE state)
{
	if(state == LED_ON)
		part_2_feedback_src_mask |= source_mask;
	else
		part_2_feedback_src_mask &= ~source_mask;
	part_2_feedback = (part_2_feedback_src_mask != 0U) ? 1U : 0U;
}

void Part2FeedbackLedCtrl(LED_STATE state)
{
	Part2FeedbackLedSource(PART_LED_SRC_LEGACY, state);
}

void Part2SoundLightLedCtrl(LED_STATE state)
{
	part_2_sound_light = state;
}

void Part2FaultLedCtrl(LED_STATE state)
{
	part_2_fault_state = state;
}
/*******************************/


void SpareGasAlarmLedCtrl(LED_STATE state)
{
	spare_gas_alarm = state;
}

void SpecialCommunicLedCtrl(LED_STATE state)
{
	special_communic = state;
}

void SpecialSelfCheckLedCtrl(LED_STATE state)
{
	spare_gas_selfcheck = state;
}

uint8_t getControllorSelfCheckState(void)
{
	return spare_gas_selfcheck;
}

extern uint16_t linkage_disconnect_state;
extern uint8_t main_power_beep_ctrl;
extern uint8_t self_check_show_content;
extern uint16_t linkage_beep_ctrl;

extern uint8_t beep_fire_ctrl ;
extern uint8_t beep_fault_ctrl;
extern uint8_t beep_spray_feedback_ctrl;
extern uint32_t beep_general_io_ctrl;

void InternalBoardSendString(uint8_t* buf, uint8_t len) {
	HAL_UART_Transmit(&huart10, buf, len, 0xff);
}

void getLedStateReg(uint8_t *led_state_buff, uint8_t len)
{
	if(led_state_buff == NULL || len < 5)
	{
		return;
	}
	
	// 根据主电状态修改LED值的状态
	setLedxRegisterBit(led_state_buff[LED_MAIN_POWER/8]          , LED_MAIN_POWER            %8, getMainPowerState()  );  // 主电LED
	setLedxRegisterBit(led_state_buff[LED_BACKUP_POWER/8]        , LED_BACKUP_POWER          %8, getSparePowerState() );  // 备电
	setLedxRegisterBit(led_state_buff[LED_FIRE_ALARM/8]          , LED_FIRE_ALARM            %8, getFireAlarmState()  );  // 火警
	setLedxRegisterBit(led_state_buff[LED_DISCONNECT_FAULT/8]    , LED_DISCONNECT_FAULT      %8, getDisconnectState() );  // 掉线故障
	setLedxRegisterBit(led_state_buff[LED_SILENCE/8]             , LED_SILENCE               %8, getSilencersState()  );  // 消音LED
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_FAULT/8]        , LED_SYSTEM_FAULT          %8, getSystemFaultState());  // 系统故障
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_START_UP/8]     , LED_SYSTEM_START_UP       %8, getInitiationState() );  // 启动
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_START_DELAY/8]  , LED_SYSTEM_START_DELAY    %8, getStartDelayState() );  // 启动延时
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_FEEDBACK/8]     , LED_SYSTEM_FEEDBACK       %8, getFeedbackedState() );  // 反馈
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_REGULAR_ALARM/8], LED_SYSTEM_REGULAR_ALARM  %8, getRegulAlarmState() );  // 监管报警
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_SHIELD/8]       , LED_SYSTEM_SHIELD         %8, getShieldingState()  );  // 屏蔽状态
	setLedxRegisterBit(led_state_buff[LED_SIREN_STARTUP/8]       , LED_SIREN_STARTUP         %8, getSirenStartState() );  // 报警器启动LED
	setLedxRegisterBit(led_state_buff[LED_SIREN_FAULT/8]         , LED_SIREN_FAULT           %8, getSirenFaultState() );  // 报警器故障LED
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_SELF_CHECK/8]   , LED_SYSTEM_SELF_CHECK     %8, getSelfCheckState()  );  // 自检开启LED 结束后自动关闭
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_1/8]      , LED_PART_STARTUP_1        %8, getPart1StartState() );  // 分区启动1启动LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_DELAY_1/8], LED_PART_STARTUP_DELAY_1  %8, getPart1DelayState() );  // 分区1启动延时LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_1/8], LED_PART_SPRAY_STARTUP_1  %8, getPart1SprayState() );  // 分区1喷放启动状态LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_FEEDBACK_1/8], LED_PART_SPRAY_FEEDBACK_1 %8, getPart1FeedbackState());  // 分区1喷放反馈LED
	setLedxRegisterBit(led_state_buff[LED_PART_SOUND_ALARM_1/8]  , LED_PART_SOUND_ALARM_1    %8, getPart1SoundState() );  // 分区1声光状态
	setLedxRegisterBit(led_state_buff[LED_PART_FAULT_1/8]        , LED_PART_FAULT_1          %8, getPart1FaultState() );  // 分区1故障状态LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_2/8]      , LED_PART_STARTUP_2        %8, getPart2StartState() );  // 分区启动2启动LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_DELAY_2/8], LED_PART_STARTUP_DELAY_2  %8, getPart2DelayState() );  // 分区2启动延时LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_2/8], LED_PART_SPRAY_STARTUP_2  %8, getPart2SprayState() );  // 分区2喷放启动状态LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_FEEDBACK_2/8], LED_PART_SPRAY_FEEDBACK_2%8, getPart2FeedbackState());  // 分区2喷放反馈LED
	setLedxRegisterBit(led_state_buff[LED_PART_SOUND_ALARM_2/8]   , LED_PART_SOUND_ALARM_2   %8, getPart2SoundState() );  // 分区2声光状态
	setLedxRegisterBit(led_state_buff[LED_PART_FAULT_2/8]         , LED_PART_FAULT_2         %8, getPart2FaultState() );  // 分区2故障状态LED
	// 新增备用LED控制 2025/10/10 16:56
	setLedxRegisterBit(led_state_buff[LED_COMBUSTIBLE_GAS_SELF_CHECK/8], LED_COMBUSTIBLE_GAS_SELF_CHECK%8, getSpareGasCheck() ); // 
	setLedxRegisterBit(led_state_buff[LED_COMBUSTIBLE_GAS_ALARM     /8], LED_COMBUSTIBLE_GAS_ALARM     %8, getSpareGasAlarm() ); // 
	setLedxRegisterBit(led_state_buff[LED_COMBUSTIBLE_GAS_FAULT     /8], LED_COMBUSTIBLE_GAS_FAULT     %8, getSpareGasFault() ); // 
	setLedxRegisterBit(led_state_buff[LED_COMBUSTIBLE_GAS_DELAY     /8], LED_COMBUSTIBLE_GAS_DELAY     %8, getSpareGasDelay() ); // 
	setLedxRegisterBit(led_state_buff[LED_STANDBY_1                 /8], LED_STANDBY_1                 %8, getSpare_1() ); // 
	setLedxRegisterBit(led_state_buff[LED_STANDBY_2                 /8], LED_STANDBY_2                 %8, getSpare_2() ); // 
	setLedxRegisterBit(led_state_buff[LED_STANDBY_3                 /8], LED_STANDBY_3                 %8, getSpare_3() ); // 
	setLedxRegisterBit(led_state_buff[LED_COMMUNICATION             /8], LED_COMMUNICATION             %8, getSpecialCommun() ); // 
	setLedxRegisterBit(led_state_buff[LED_WORKING                   /8], LED_WORKING                   %8, getSpecialRuning() ); // 
	
}

/* V1.2: 把三路手自动状态填入寄存器2的高/低字节。
 * R2H bit3=总手动 bit4=总自动 bit5=分区1手动 bit6=分区1自动 bit7=分区2手动;
 * R2L bit0=分区2自动。原 bit0~2(预留3/通信/运行)保持不变。 */
static void ApplyHandAutoLedBits(uint8_t *reg2_high, uint8_t *reg2_low)
{
	uint8_t hi = (*reg2_high) & 0x07U;
	uint8_t lo = (*reg2_low) & 0xFEU;

	hi |= (getSysHandAutoState()   == KEY_MANUAL) ? (1U << 3) : (1U << 4);
	hi |= (getPart1HandAutoState() == KEY_MANUAL) ? (1U << 5) : (1U << 6);
	if (getPart2HandAutoState() == KEY_MANUAL)
	{
		hi |= (1U << 7);
	}
	else
	{
		lo |= 0x01U;
	}
	*reg2_high = hi;
	*reg2_low  = lo;
}

void HostUploadDataTask(void *parameter)
{
	uint8_t modbusbuf[24];  // 发送缓冲区
	static uint8_t printer_frame[BSP_PRINTER_FRAME_MAX_LEN];
	uint16_t crc16 = 0x0000;  // CRC校验码
	uint8_t printer_turn = 0U;
	for(;;)
	{
		if(system_reset_flag == 1)
		{
			printer_turn = 0U;
			modbusbuf[0] = 0xFF;  // 从机地址取值
			modbusbuf[1] = 0x05;  // 05功能码
			modbusbuf[2] = 0x00;
			modbusbuf[3] = 20;
			modbusbuf[4] = 0xFF;
			modbusbuf[5] = 0x00; // 
			crc16 = CalcCrc16(modbusbuf, 6);  // 计算六字节数据的CRC校验码
			modbusbuf[6] = crc16 >> 0;
			modbusbuf[7] = crc16 >> 8;
			
			InternalBoardSendString(modbusbuf, 8);
			
		}
		else
		{
			if(self_check_show_content == 1)  // LED指示灯自检
			{
				uint8_t data_count = 0;
				printer_turn = 0U;
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // 使用04功能码 发送全部状态
				modbusbuf[data_count++] = 10;  // 返回的字节数(V1.2: 5个寄存器共10字节)
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = 0xFF;  // 全部点亮(含寄存器2高字节的手自动灯)
				}
				modbusbuf[data_count++] = 0xFF;  // 寄存器2低字节: 分区2自动点亮
				modbusbuf[data_count++] = 0;     // 寄存器3高字节: 关闭蜂鸣器
				modbusbuf[data_count++] = 0;     // 寄存器3低字节: 预留
				modbusbuf[data_count++] = 0;     // 寄存器4高字节: 预留
				modbusbuf[data_count++] = 0;     // 寄存器4低字节: 工作(不触发复位/自检)

				crc16 = CalcCrc16(modbusbuf, data_count);  // CRC校验
				modbusbuf[data_count++] = (uint8_t)(crc16);
				modbusbuf[data_count++] = (crc16 >> 8);
				
				InternalBoardSendString(modbusbuf, data_count);  // 发送数据到3303
				
				crc16 = 0;
				
				osDelay(2000);
				self_check_show_content = 2;
			}
			else if(self_check_show_content == 2)  // 蜂鸣器自检
			{
				uint8_t data_count = 0;
				uint8_t led_reg_state[5] = { 0 };  // 默认关闭
				uint8_t reg2_low = 0U;             // 寄存器2低字节
				printer_turn = 0U;
				
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // 使用04功能码 发送全部状态
				modbusbuf[data_count++] = 10;  // 返回的字节数(V1.2)
				
				getLedStateReg(led_reg_state, 5);
				ApplyHandAutoLedBits(&led_reg_state[4], &reg2_low);
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = led_reg_state[i];
				}
				modbusbuf[data_count++] = reg2_low;  // 寄存器2低字节: 分区2自动
				modbusbuf[data_count++] = 1;  // 寄存器3高字节: 打开蜂鸣器
				modbusbuf[data_count++] = 0;  // 寄存器3低字节: 预留
				modbusbuf[data_count++] = 0;  // 寄存器4高字节: 预留
				modbusbuf[data_count++] = 0;  // 寄存器4低字节: 工作

				crc16 = CalcCrc16(modbusbuf, data_count);  // CRC校验
				modbusbuf[data_count++] = (uint8_t)(crc16);
				modbusbuf[data_count++] = (crc16 >> 8);
				
				InternalBoardSendString(modbusbuf, data_count);  // 发送数据到3303
				
				crc16 = 0;
				
				osDelay(2000);
				self_check_show_content = 3;
			}
			else
			{
				uint8_t printer_frame_length = 0U;
				if(printer_turn != 0U &&
				   BspPrinter_BuildNextFrame(printer_frame,
				                                 sizeof(printer_frame),
				                                 &printer_frame_length) != 0U)
				{
					InternalBoardSendString(printer_frame, printer_frame_length);
					printer_turn = 0U;
				}
				else
				{
					uint8_t led_reg_state[5] = { 0 };  // 默认关闭
					uint8_t reg2_low = 0U;             // 寄存器2低字节
					uint8_t data_count = 0;

					getLedStateReg(led_reg_state, 5);
					ApplyHandAutoLedBits(&led_reg_state[4], &reg2_low);
					modbusbuf[data_count++] = InternalBoardAddr;
					modbusbuf[data_count++] = 0x04;  // 使用04功能码 发送全部状态
					modbusbuf[data_count++] = 10;  // 返回的字节数(V1.2: 5个寄存器共10字节)

					for(uint8_t i = 0; i < 5; i++)
					{
						modbusbuf[data_count++] = led_reg_state[i];
					}
					modbusbuf[data_count++] = reg2_low;  // 寄存器2低字节: 分区2自动
					modbusbuf[data_count++] = getBuzzerWorkState();  // 寄存器3高字节: 蜂鸣器
					modbusbuf[data_count++] = 0;  // 寄存器3低字节: 预留
					modbusbuf[data_count++] = 0;  // 寄存器4高字节: 预留
					modbusbuf[data_count++] = 0;  // 寄存器4低字节: 工作(不触发复位/自检)

					crc16 = CalcCrc16(modbusbuf, data_count);  // CRC校验
					modbusbuf[data_count++] = (uint8_t)(crc16);
					modbusbuf[data_count++] = (crc16 >> 8);

					InternalBoardSendString(modbusbuf, data_count);
//				DebugSendString(modbusbuf, data_count);
					crc16 = 0;
					printer_turn = BspPrinter_HasPending();
				}
			}

		}
		
		osDelay(500);
	}
}

static uint8_t key_value_storage = NONE_KEY;  // 默认没有按键按下

uint8_t getKeyPressValue(void)
{
	return key_value_storage;
}

void setKeyValue(InternalKeyPressValue val)
{
	key_value_storage = val;
}

uint8_t matrix_key_val;

uint8_t getDirectionKeyValue(void)
{
	return matrix_key_val;
}

void clearMatrixKeyValue(void)
{
	matrix_key_val = NO_MATRIX_KEY_PRESS;
}

uint8_t outfire_key_val;

uint8_t getOutFireKeyalue(void)
{
	return outfire_key_val;
}

void clearOutFireKeyValue(void)
{
	outfire_key_val = NO_MATRIX_KEY_PRESS;
}

uint8_t getPart1HandAutoState(void)
{
	return (SystemSaveInfo.part1_hand_or_auto_state == KEY_MANUAL) ? KEY_MANUAL : KEY_AUTO;
}

uint8_t getSysHandAutoState(void)
{
	return (SystemSaveInfo.system_hand_or_auto_state == KEY_MANUAL) ? KEY_MANUAL : KEY_AUTO;
}

uint8_t getPart2HandAutoState(void)
{
	return (SystemSaveInfo.part2_hand_or_auto_state == KEY_MANUAL) ? KEY_MANUAL : KEY_AUTO;
}

/* 需求5: 系统级为手动时, 不管分区级设置为何, 本分区一律判为手动。
 * 只在"是否允许自动联动/是否允许手动喷洒"的业务判断点使用;
 * LED 显示仍用上面三个原始状态函数, 保留设置层语义。 */
uint8_t getPart1EffectiveHandAuto(void)
{
	if(getSysHandAutoState() == KEY_MANUAL)
	{
		return KEY_MANUAL;
	}
	return getPart1HandAutoState();
}

uint8_t getPart2EffectiveHandAuto(void)
{
	if(getSysHandAutoState() == KEY_MANUAL)
	{
		return KEY_MANUAL;
	}
	return getPart2HandAutoState();
}

/* V1.2: 三路手自动按键为两档自复位开关, 板上报瞬时按压位;
 * 此处按上升沿翻转本层状态, 保持原有落盘/黑匣子/FECbus上报序列不变。 */
static void InternalBoard_ToggleHandAuto(uint8_t level)
{
	uint8_t *state;
	FlashSaveType turn_hand;
	FlashSaveType turn_auto;
	uint8_t  package_id;
	uint8_t  to_manual;

	if(level == 0U)
	{
		state = &SystemSaveInfo.system_hand_or_auto_state;
		turn_hand = OTHER_SYS_TURN_HAND;
		turn_auto = OTHER_SYS_TURN_AUTO;
		package_id = SYS_HAND_AUTO_Package_ID;
	}
	else if(level == 1U)
	{
		state = &SystemSaveInfo.part1_hand_or_auto_state;
		turn_hand = OTHER_PART1_TURN_HAND;
		turn_auto = OTHER_PART1_TURN_AUTO;
		package_id = PART1_HAND_AUTO_Package_ID;
	}
	else
	{
		state = &SystemSaveInfo.part2_hand_or_auto_state;
		turn_hand = OTHER_PART2_TURN_HAND;
		turn_auto = OTHER_PART2_TURN_AUTO;
		package_id = PART2_HAND_AUTO_Package_ID;
	}

	/* 当前手动 -> 切自动; 其余(自动或异常值) -> 切手动 */
	to_manual = (*state == KEY_MANUAL) ? 0U : 1U;
	*state = (to_manual != 0U) ? KEY_MANUAL : KEY_AUTO;
	SystemInfoSave();
	BspCommonDataSaveApp(GASER_FLASH_SAVE,
	                     (to_manual != 0U) ? turn_hand : turn_auto,
	                     LINKAGE_CLUSTER_ID, package_id);
	StorageEvent_LogManualAuto(package_id, to_manual);
	FecbusReport_ManualAuto(package_id, to_manual);
}

void InternalScreenBoradRecvDealTask(void * parameter)
{
	static uint8_t last_matrix_key_code = NO_MATRIX_KEY_PRESS;  /* V1.2: 矩阵键去重 */
	static uint8_t last_separate_key_bits = 0U;                 /* V1.2: 独立键上升沿检测 */
	uint8_t temp_key_val;
	uint8_t key_bits;      /* 寄存器1 高字节: 报警器/联动启动/分区复位/检修/三路手自动 */
	uint8_t key_edges;     /* 本次帧中新按下的独立键位 */
	uint8_t matrix_pressed;/* 本次帧是否为矩阵键新按下 */
	uint16_t crc16 = 0x0000;  // CRC校验码
	
	for(;;)
	{
		if(uartbuff[INSCREENSITE].recepetion_flag == 1)
		{
			uartbuff[INSCREENSITE].recepetion_flag = 0;
			/* IG3303 固定回复 9 字节:
			 * [addr][04][04][reg0_hi][reg0_lo][reg1_hi][reg1_lo][crc_lo][crc_hi]
			 * 帧长不符直接丢弃本帧(不能用 return, 那会退出本任务) */
			if(uartbuff[INSCREENSITE].recepetion_len != 9U)
			{
				continue;
			}
			if(uartbuff[INSCREENSITE].recepetion_buff[2] != 0x04U)
			{
				continue;
			}
			crc16 = ( uartbuff[INSCREENSITE].recepetion_buff[uartbuff[INSCREENSITE].recepetion_len-1] << 8)| 
								uartbuff[INSCREENSITE].recepetion_buff[uartbuff[INSCREENSITE].recepetion_len-2];
			
			if(CalcCrc16(uartbuff[INSCREENSITE].recepetion_buff, uartbuff[INSCREENSITE].recepetion_len - 2) == crc16)
			{
				if(uartbuff[INSCREENSITE].recepetion_buff[0] == InternalBoardAddr)
				{
					if(uartbuff[INSCREENSITE].recepetion_buff[1] == 0x04)
					{
						temp_key_val = uartbuff[INSCREENSITE].recepetion_buff[4];
						matrix_key_val = temp_key_val;
						outfire_key_val = temp_key_val;
						/* V1.2: 按住不放时按键板会周期性重发同一键值, 只在“新按下”时执行一次动作 */
						if((temp_key_val != NO_MATRIX_KEY_PRESS) && (temp_key_val != last_matrix_key_code))
						{
							matrix_pressed = 1U;
						}
						else
						{
							matrix_pressed = 0U;
						}
						last_matrix_key_code = temp_key_val;
						if(matrix_pressed == 0U)
						{
							temp_key_val = NO_MATRIX_KEY_PRESS;  /* 走 default, 不重复触发 */
						}
						switch(temp_key_val)
						{
							/* 信息确认键只上报键值；画面86/87成功确认未确认事件后，
							 * 由画面任务写入EVT 128，避免空行或重复确认产生无效记录。 */
							case KEY1_INFORM_CERTAIN  :
								break;
							case KEY2_SELF_INSPECTION :  // 系统自检
								HmiRequestInternalProtectedAction(SELFCHECK_KEY);
								break;
							case KEY3_SYSTEM_SILENCE  : {  // 系统消音
//								key_value_storage = SILENSE_KEY;
								if( main_power_beep_ctrl     != 0 || 
										linkage_beep_ctrl        != 0 ||
										beep_fire_ctrl           != 0 ||
										beep_fault_ctrl          != 0 || 
										beep_spray_feedback_ctrl != 0 || 
										beep_general_io_ctrl     != 0    
								) 
								{		
									silencers_state = 1;  // 消音标志
								}
								
								beep_spray_feedback_ctrl = 0;
								
								beep_fire_ctrl = 0;
								beep_fault_ctrl = 0;

								main_power_beep_ctrl = 0;
								linkage_beep_ctrl = 0;
								beep_general_io_ctrl = 0;
								break;
							}
							case KEY4_SYSTEM_RESET    :  // 系统复位
								HmiRequestInternalProtectedAction(RESET_KEY);
								break;
							case KEY5_SYSTEM_CHECK    : //
								/* XR5000_CHECK_CHANGE_20260804: check is not password protected. */
								key_value_storage = NONE_KEY;
								if(self_check_state == 0U) SystemCheckRequestRecord(); /* XR5000_CHECK_FLASH_FIX_20260804 */
								self_check_state = 1U;
								taskENTER_CRITICAL();
								SetScreen(72);
								taskEXIT_CRITICAL();
								osDelay(5);
								GetScreen();
								break;
							case KEY6_DIRECTION_UP    :  // 方向键上
								
								break;
							case KEY7_DIRECTION_RIGHT :  // 方向键右
								
								break;
							case KEY8_DIRECTION_DOWN  :  // 方向键下
								
								break;
							case KEY9_DIRECTION_LEFT  :  // 方向键左
								
								break;
							case KEY10_DIRECTION_OK   :  // 方向键ok
								
								break;
							case KEY11_PART1_SOUNDLT  :  // 分区1声光
								FireZoneSirenCtrl(1U, (FireZoneGetSirenManual(1U) != 0U) ? 0U : 1U);
								break;
							case KEY12_PART1_STOP     :  // 分区1停止: 只停本区联动动作设备, 不动声光/显示盘
								FireZoneOutputCtrl(1U, 0U);
								break;
							case KEY13_PART2_SOUNDLT  :  // 分区2声光
								FireZoneSirenCtrl(2U, (FireZoneGetSirenManual(2U) != 0U) ? 0U : 1U);
								break;
							case KEY14_PART2_STOP     :  // 分区2停止: 只停本区联动动作设备, 不动声光/显示盘
								FireZoneOutputCtrl(2U, 0U);
								break;
							case KEY15_PART1_SPRAY_ST : {  // 分区1启动: 只启动本区输出设备
								FireZoneOutputCtrl(1U, 1U);   /* 启动本区所有条件成立的联动输出 */
								break;
							}
							case KEY16_PART2_SPRAY_ST : {  // 分区2启动: 只启动本区输出设备
								FireZoneOutputCtrl(2U, 1U);   /* 启动本区所有条件成立的联动输出 */
								break;
							}
							default:
								break;
						}

						/* V1.2: 寄存器1低字节(buf[6])的2位手自动状态编码已取消,
					 * 改为寄存器1高字节(buf[5])的 bit5/6/7 瞬时按键位, 上升沿翻转本层状态。 */
					key_bits = uartbuff[INSCREENSITE].recepetion_buff[5];
					key_edges = key_bits & (uint8_t)(~last_separate_key_bits);
					last_separate_key_bits = key_bits;

					/* 寄存器1高字节: Bit0 报警器 / Bit1 联动启动 / Bit2 分区1复位 /
					 * Bit3 分区2复位 / Bit4 检修开关(保持型) / Bit5~7 三路手自动瞬时按键 */
					g_maintenance_state = ((key_bits & 0x10U) != 0U) ? 1U : 0U;
					if((key_edges & 0x01U) != 0U)
					{
						FireZoneSysSirenToggle();   /* 报警器键: 系统级声光人工开关 */
					}
					if((key_edges & 0x02U) != 0U)
					{
						HmiRequestInternalProtectedAction(LINKAGE_START_KEY);  /* 系统级联动启动 */
					}
					if((key_edges & 0x04U) != 0U)
					{
						FireZoneReset(1U);          /* 分区1复位 */
					}
					if((key_edges & 0x08U) != 0U)
					{
						FireZoneReset(2U);          /* 分区2复位 */
					}
					if((key_edges & 0x20U) != 0U)
					{
						InternalBoard_ToggleHandAuto(0U);  /* 总手自动 */
					}
					if((key_edges & 0x40U) != 0U)
					{
						InternalBoard_ToggleHandAuto(1U);  /* 分区1手自动 */
					}
					if((key_edges & 0x80U) != 0U)
					{
						InternalBoard_ToggleHandAuto(2U);  /* 分区2手自动 */
					}


					}
				}

			}
			crc16 = 0;
			//DebugSendString(uartbuff[INSCREENSITE].recepetion_buff, uartbuff[INSCREENSITE].recepetion_len);
		}
		osDelay(10);
	}
}


