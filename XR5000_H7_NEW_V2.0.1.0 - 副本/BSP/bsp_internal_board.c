#include "bsp_internal_board.h"

#include "cmsis_os.h"
#include "cmd_process.h"
#include "bsp_debug.h"

#include "bsp_rs485_01.h"

#include "bsp_relay.h"

#include "system.h"

#include "bsp_adc.h"

#include "bsp_save_ctrl.h"

#include "bsp_storage_event.h"  /* 黑匣子存储事件接入层 */
#include "bsp_fecbus_report.h" /* FECbus RS485 上报接入层 (GB4717 附录C) */

#define InternalBoardAddr 1U

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
#define getFeedbackedState()  (feedbacked_state    ? 1 : 0)
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
  LED_MAIN_POWER,           // 寄存器0    主电
	LED_BACKUP_POWER,         // 寄存器1    备电
	LED_FIRE_ALARM,           // 寄存器2    火警
	LED_DISCONNECT_FAULT,     // 寄存器3
	LED_SILENCE,              // 寄存器4
	LED_SYSTEM_FAULT,         // 寄存器5
	LED_SYSTEM_START_UP,      // 寄存器6
	LED_SYSTEM_START_DELAY,   // 寄存器7
	LED_SYSTEM_FEEDBACK,      // 寄存器8
	LED_SYSTEM_REGULAR_ALARM, // 寄存器9
	LED_SYSTEM_SHIELD,        // 寄存器10
	LED_SIREN_STARTUP,        // 寄存器11
	LED_SIREN_FAULT,          // 寄存器12
	LED_SYSTEM_SELF_CHECK,    // 寄存器13
	LED_PART_STARTUP_1,       // 寄存器14
	LED_PART_STARTUP_DELAY_1, // 寄存器15
	LED_PART_SPRAY_STARTUP_1, // 寄存器16
	LED_PART_SPRAY_FEEDBACK_1,// 寄存器17
	LED_PART_SOUND_ALARM_1,   // 寄存器18
	LED_PART_FAULT_1,         // 寄存器19
	LED_PART_STARTUP_2,       // 寄存器20
	LED_PART_STARTUP_DELAY_2, // 寄存器21
	LED_PART_SPRAY_STARTUP_2, // 寄存器22
	LED_PART_SPRAY_FEEDBACK_2,// 寄存器23
	LED_PART_SOUND_ALARM_2,   // 寄存器24
	LED_PART_FAULT_2,         // 寄存器25
	
	// 新增LED寄存器 2025/10/10
	LED_COMBUSTIBLE_GAS_SELF_CHECK, // 可燃气体自检 // 寄存器26
	LED_COMBUSTIBLE_GAS_ALARM,  // 可燃气体报警 // 寄存器27
	LED_COMBUSTIBLE_GAS_FAULT,  // 可燃气体故障 // 寄存器28
	LED_COMBUSTIBLE_GAS_DELAY,  // 可燃气体延时 // 寄存器29
	LED_STANDBY_1,              // 预留LED1 // 寄存器30
	LED_STANDBY_2,              // 预留LED2 // 寄存器31
	LED_STANDBY_3,              // 预留LED3 // 寄存器32
	// 特殊LED寄存器 2025/10/10
	LED_COMMUNICATION,          // 通信LED // 寄存器33
	LED_WORKING,                // 运行指示灯 // 寄存器34
    
  LED_COUNT  // LED总数
} LED_REGISTER_ID;

// led_val灯的状态值 n第n位 x状态
#define setLedxRegisterBit(led_val, n, x) ( (led_val) |= ((x) << (n)) )

// 独立按键键值定义
typedef enum
{
	KEY_SYSTEM_ANNOUNCIAT = 0x01,  // 系统报警器启动
	KEY_SYSTEM_LINKAGE_S  = 0x02,  // 联动启动按下
	
	SEPERATE_KEY_NO_PRESS = 0X00,  // 独立按键没有按下
}BspSeperateKeyPressValue;

// 手自动状态键值定义
typedef enum
{
	KEY_SYSTEM_MANUAL = 0x01,
	KEY_SYSTEM_AUTO   = 0x02,
	
	KEY_PART1_MANUAL  = 0x04,  // 分区1切换手动
	KEY_PART1_AUTO    = 0x08,  // 分区1切换自动
	
	KEY_PART2_MANUAL  = 0x10,  // 分区1切换手动
	KEY_PART2_AUTO    = 0x20,  // 分区1切换自动
	
}BspHandAutoState;

uint8_t fire_alarm_state    = 0; // 火警状态
uint8_t disconnect_state    = 0; // 掉线状态
uint8_t silencers_state     = 0; // 消音状态
uint8_t system_fault_state  = 0; // 系统故障状态
uint8_t sys_start_state = 0; // 启动状态
uint8_t start_delay_state   = 0; // 启动延时状态
uint8_t feedbacked_state    = 0; // 反馈状态
uint8_t regul_alarm_state   = 0; // 监管报警状态
uint8_t shielding_state     = 0; // 屏蔽状态
uint8_t siren_start_state   = 0; // 报警器启动状态
uint8_t siren_fault_state   = 0; // 报警器故障状态
uint8_t self_check_state    = 0; // 自检状态
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
// 备用LED状态控制变量
uint8_t spare_gas_selfcheck = 0; // 备用 可燃气体自检
uint8_t spare_gas_alarm     = 0; // 备用 可燃气体报警
uint8_t spare_gas_fault     = 0; // 备用 可燃气体故障
uint8_t spare_gas_delay     = 0; // 备用 可燃气体延时
uint8_t spare_1_led         = 0; // 备用 
uint8_t spare_2_led         = 0; // 备用 
uint8_t spare_3_led         = 0; // 备用 

uint8_t special_communic    = 0; // 通信LED
uint8_t special_running     = 0; // 运行LED    

// 系统复位显示控制
uint8_t system_reset_flag   = 0;
// [7:4] 历史状态 [3:0] 当前状态
uint8_t screen_show_siren_information = 0;

void LedStateInit(void)
{
	// 主面板LED
	fire_alarm_state    = 0; // 火警状态
	disconnect_state    = 0; // 掉线状态
	silencers_state     = 0; // 消音状态
	system_fault_state  = 0; // 系统故障状态
	sys_start_state     = 0; // 启动状态
	start_delay_state   = 0; // 启动延时状态
	feedbacked_state    = 0; // 反馈状态
	regul_alarm_state   = 0; // 监管报警状态
	shielding_state     = 0; // 屏蔽状态
	siren_start_state   = 0; // 报警器启动状态
	siren_fault_state   = 0; // 报警器故障状态
	self_check_state    = 0; // 自检状态
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
	
	// 备用LED状态控制变量
	spare_gas_selfcheck = 0; // 备用 可燃气体自检
	spare_gas_alarm     = 0; // 备用 可燃气体报警
	spare_gas_fault     = 0; // 备用 可燃气体故障
	spare_gas_delay     = 0; // 备用 可燃气体延时
	spare_1_led         = 0; // 备用 
	spare_2_led         = 0; // 备用 
	spare_3_led         = 0; // 备用 
	// 特殊LED
	special_communic    = 0; // 通信LED
	special_running     = 0; // 运行LED  
	
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

// 分区1
void Part1StartLedCtrl(LED_STATE state)
{
	part_1_start_state = state;
}

void Part1StartDelayLedCtrl(LED_STATE state)
{
	part_1_start_delay = state;
}

void Part1SprayLedCtrl(LED_STATE state)
{
	part_1_spray_state = state;
}

void Part1FeedbackLedCtrl(LED_STATE state)
{
	part_1_feedback = state;
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

void Part2StartDelayLedCtrl(LED_STATE state)
{
	part_2_start_delay = state;
}

void Part2SprayLedCtrl(LED_STATE state)
{
	part_2_spray_state = state;
}

void Part2FeedbackLedCtrl(LED_STATE state)
{
	part_2_feedback = state;
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
	setLedxRegisterBit(led_state_buff[LED_MAIN_POWER/8]          , LED_MAIN_POWER            %8, getMainPowerState()  ); // 主电LED
	setLedxRegisterBit(led_state_buff[LED_BACKUP_POWER/8]        , LED_BACKUP_POWER          %8, getSparePowerState() ); // 备电
	setLedxRegisterBit(led_state_buff[LED_FIRE_ALARM/8]          , LED_FIRE_ALARM            %8, getFireAlarmState()  ); // 火警
	setLedxRegisterBit(led_state_buff[LED_DISCONNECT_FAULT/8]    , LED_DISCONNECT_FAULT      %8, getDisconnectState() ); // 掉线故障
	setLedxRegisterBit(led_state_buff[LED_SILENCE/8]             , LED_SILENCE               %8, getSilencersState()  ); // 消音LED
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_FAULT/8]        , LED_SYSTEM_FAULT          %8, getSystemFaultState()); // 系统故障
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_START_UP/8]     , LED_SYSTEM_START_UP       %8, getInitiationState() ); // 启动
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_START_DELAY/8]  , LED_SYSTEM_START_DELAY    %8, getStartDelayState() ); // 启动延时
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_FEEDBACK/8]     , LED_SYSTEM_FEEDBACK       %8, getFeedbackedState() ); // 反馈
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_REGULAR_ALARM/8], LED_SYSTEM_REGULAR_ALARM  %8, getRegulAlarmState() ); // 监管报警
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_SHIELD/8]       , LED_SYSTEM_SHIELD         %8, getShieldingState()  ); // 屏蔽状态
	setLedxRegisterBit(led_state_buff[LED_SIREN_STARTUP/8]       , LED_SIREN_STARTUP         %8, getSirenStartState() ); // 报警器启动LED
	setLedxRegisterBit(led_state_buff[LED_SIREN_FAULT/8]         , LED_SIREN_FAULT           %8, getSirenFaultState() ); // 报警器故障LED
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_SELF_CHECK/8]   , LED_SYSTEM_SELF_CHECK     %8, getSelfCheckState()  ); // 自检开启LED 结束后自动关闭
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_1/8]      , LED_PART_STARTUP_1        %8, getPart1StartState() ); // 分区启动1启动LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_DELAY_1/8], LED_PART_STARTUP_DELAY_1  %8, getPart1DelayState() ); // 分区1启动延时LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_1/8], LED_PART_SPRAY_STARTUP_1  %8, getPart1SprayState() ); // 分区1喷放启动状态LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_1/8], LED_PART_SPRAY_STARTUP_1  %8, getPart1FeedbackState()); // 分区1喷放反馈LED
	setLedxRegisterBit(led_state_buff[LED_PART_SOUND_ALARM_1/8]  , LED_PART_SOUND_ALARM_1    %8, getPart1SoundState() ); // 分区1声光状态
	setLedxRegisterBit(led_state_buff[LED_PART_FAULT_1/8]        , LED_PART_FAULT_1          %8, getPart1FaultState() ); // 分区1故障状态LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_2/8]      , LED_PART_STARTUP_2        %8, getPart2StartState() ); // 分区启动2启动LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_DELAY_2/8], LED_PART_STARTUP_DELAY_2  %8, getPart2DelayState() ); // 分区2启动延时LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_2/8], LED_PART_SPRAY_STARTUP_2  %8, getPart2SprayState() ); // 分区2喷放启动状态LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_FEEDBACK_2/8], LED_PART_SPRAY_FEEDBACK_2%8, getPart2FeedbackState()); // 分区2喷放反馈LED
	setLedxRegisterBit(led_state_buff[LED_PART_SOUND_ALARM_2/8]   , LED_PART_SOUND_ALARM_2   %8, getPart2SoundState() ); // 分区2声光状态
	setLedxRegisterBit(led_state_buff[LED_PART_FAULT_2/8]         , LED_PART_FAULT_2         %8, getPart2FaultState() ); // 分区2故障状态LED
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

void HostUploadDataTask(void *parameter)
{
	uint8_t modbusbuf[24]; // 发送缓冲区
	uint16_t crc16 = 0x0000; // CRC校验码
	for(;;)
	{
		if(system_reset_flag == 1)
		{
			modbusbuf[0] = 0xFF; // 从机地址取值
			modbusbuf[1] = 0x05; // 05功能码
			modbusbuf[2] = 0x00;
			modbusbuf[3] = 20;
			modbusbuf[4] = 0xFF;
			modbusbuf[5] = 0x00; // 
			crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
			modbusbuf[6] = crc16 >> 0;
			modbusbuf[7] = crc16 >> 8;
			
			InternalBoardSendString(modbusbuf, 8);
			
		}
		else
		{
			if(self_check_show_content == 1) // LED指示灯自检
			{
				uint8_t data_count = 0;
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // 使用04功能码 发送全部状态
				modbusbuf[data_count++] = 6;  // 返回的字节数
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = 0xFF; // 全部点亮
				}
				modbusbuf[data_count++] = 0;
				modbusbuf[data_count++] = 0; // 关闭蜂鸣器

				crc16 = CalcCrc16(modbusbuf, data_count); // 计算六字节数据的CRC校验码
				modbusbuf[data_count++] = (uint8_t)(crc16);
				modbusbuf[data_count++] = (crc16 >> 8);
				
				InternalBoardSendString(modbusbuf, data_count); // 发送数据到3303
				
				crc16 = 0;
				
				osDelay(2000);
				self_check_show_content = 2;
			}
			else if(self_check_show_content == 2) // 蜂鸣器自检
			{
				uint8_t data_count = 0;
				uint8_t led_reg_state[5] = { 0 }; // 默认关闭
				
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // 使用04功能码 发送全部状态
				modbusbuf[data_count++] = 6;  // 返回的字节数
				
				getLedStateReg(led_reg_state, 5);
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = led_reg_state[i]; // 全部灭掉
				}
				modbusbuf[data_count++] = 0;
				modbusbuf[data_count++] = 1; // 打开蜂鸣器 鸣叫一秒

				crc16 = CalcCrc16(modbusbuf, data_count); // 计算六字节数据的CRC校验码
				modbusbuf[data_count++] = (uint8_t)(crc16);
				modbusbuf[data_count++] = (crc16 >> 8);
				
				InternalBoardSendString(modbusbuf, data_count); // 发送数据到3303
				
				crc16 = 0;
				
				osDelay(2000);
				self_check_show_content = 3;
			}
			else
			{
				uint8_t led_reg_state[5] = { 0 }; // 默认关闭
				
				getLedStateReg(led_reg_state, 5);

				uint8_t data_count = 0;
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // 使用04功能码 发送全部状态
				modbusbuf[data_count++] = 6;  // 返回的字节数
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = led_reg_state[i];
				}
				modbusbuf[data_count++] = 0;
				modbusbuf[data_count++] = getBuzzerWorkState();

				crc16 = CalcCrc16(modbusbuf, data_count); // 计算六字节数据的CRC校验码
				modbusbuf[data_count++] = (uint8_t)(crc16);
				modbusbuf[data_count++] = (crc16 >> 8);
				
				InternalBoardSendString(modbusbuf, data_count);
//				DebugSendString(modbusbuf, data_count);
				crc16 = 0;
			}

		}
		
		osDelay(500);
	}
}

static uint8_t key_value_storage = NONE_KEY; // 默认没有按键按下

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

uint8_t part1_hand_or_auto_state = KEY_AUTO; // 默认自动
uint8_t getPart1HandAutoState(void)
{
	return part1_hand_or_auto_state;
}

uint8_t sys_hand_or_auto_state = KEY_AUTO; // 默认自动
uint8_t getSysHandAutoState(void)
{
	return sys_hand_or_auto_state;
}

uint8_t part2_hand_or_auto_state = KEY_AUTO;
uint8_t getPart2HandAutoState(void)
{
	return part2_hand_or_auto_state;
}

void InternalScreenBoradRecvDealTask(void * parameter)
{
	uint8_t temp_key_val;
	uint16_t crc16 = 0x0000; // CRC校验码
	
	for(;;)
	{
		if(uartbuff[INSCREENSITE].recepetion_flag == 1)
		{
			uartbuff[INSCREENSITE].recepetion_flag = 0;
			if(uartbuff[INSCREENSITE].recepetion_len < 2)
			{
				return;
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
						switch(temp_key_val)
						{
							case KEY1_INFORM_CERTAIN  : // 信息确认
								break;
							case KEY2_SELF_INSPECTION : // 系统自检
								key_value_storage = SELFCHECK_KEY; // 赋值自检
								taskENTER_CRITICAL();
								SetScreen(53);	// 进入二级密码页
								taskEXIT_CRITICAL();	
								osDelay(5);
								GetScreen();
								break;
							case KEY3_SYSTEM_SILENCE  : { // 系统消音
//								key_value_storage = SILENSE_KEY;
								if( main_power_beep_ctrl     != 0 || 
										linkage_beep_ctrl        != 0 ||
										beep_fire_ctrl           != 0 ||
										beep_fault_ctrl          != 0 || 
										beep_spray_feedback_ctrl != 0 || 
										beep_general_io_ctrl     != 0    
								) 
								{		
									silencers_state = 1; // 消音标志
								}
								
								beep_spray_feedback_ctrl = 0;
								
								beep_fire_ctrl = 0;
								beep_fault_ctrl = 0;

								main_power_beep_ctrl = 0;
								linkage_beep_ctrl = 0;
								beep_general_io_ctrl = 0;
								break;
							}
							case KEY4_SYSTEM_RESET    : // 系统复位
								key_value_storage = RESET_KEY;
								taskENTER_CRITICAL();
								SetScreen(53);	// 进入二级密码页
								taskEXIT_CRITICAL();	
								osDelay(5);
								GetScreen();
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
							case KEY6_DIRECTION_UP    : // 方向键上
								
								break;
							case KEY7_DIRECTION_RIGHT : // 方向键右
								
								break;
							case KEY8_DIRECTION_DOWN  : // 方向键下
								
								break;
							case KEY9_DIRECTION_LEFT  : // 方向键左
								
								break;
							case KEY10_DIRECTION_OK   : // 方向键ok
								
								break;
							case KEY11_PART1_SOUNDLT  : // 分区1声光
								break;
							case KEY12_PART1_STOP     : // 分区1停止
								break;
							case KEY13_PART2_SOUNDLT  :  // 分区2声光
								break;
							case KEY14_PART2_STOP     :  // 分区2停止
								break;
							case KEY15_PART1_SPRAY_ST : { // 分区1喷洒启动
								key_value_storage = PART1_SPRY_START;
								SetScreen(53);								
								osDelay(5);
								GetScreen();
								break;
							}
							case KEY16_PART2_SPRAY_ST : { // 分区2喷洒启动
								key_value_storage = PART2_SPRY_START;
								SetScreen(53);								
								osDelay(5);
								GetScreen();
								break;
							}
							default:
								break;
						}

						switch(uartbuff[INSCREENSITE].recepetion_buff[5])
						{
							case KEY_SYSTEM_ANNOUNCIAT : // 系统报警器启动
								key_value_storage = SIREN_KEY; // 
//								taskENTER_CRITICAL();
//								SetScreen(53);	// 进入二级密码页
//								taskEXIT_CRITICAL();
								SetScreen(53);								
								osDelay(5);
								GetScreen();
								break;
							case KEY_SYSTEM_LINKAGE_S  : // 联动启动按下
								key_value_storage = LINKAGE_START_KEY; // 
								StartupLinkageDevice();
								StorageEvent_LogStart(LINKAGE_CLUSTER_ID, DEV_TYPE_CONTROL_DEV); /* 黑匣子:联动启动 */
							FecbusReport_Start(LINKAGE_CLUSTER_ID, DEV_TYPE_CONTROL_DEV); /* FECbus:联动启动 */
								break;
							default:
								break;
						}
						// 判断系统手自动
						switch(uartbuff[INSCREENSITE].recepetion_buff[6] & 0x03)
						{
							case KEY_MANUAL:
								if(SystemSaveInfo.system_hand_or_auto_state != KEY_MANUAL) // 手动
								{
									SystemSaveInfo.system_hand_or_auto_state = KEY_MANUAL; // 手动
									SystemInfoSave();
									// 存入气灭分区 状态切换为手动
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_SYS_TURN_HAND, LINKAGE_CLUSTER_ID, SYS_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(SYS_HAND_AUTO_Package_ID, 1); /* 黑匣子:系统手动 */
								FecbusReport_ManualAuto(SYS_HAND_AUTO_Package_ID, 1); /* FECbus:系统手动 */
								}
								break;
							case KEY_AUTO:
								if(SystemSaveInfo.system_hand_or_auto_state != KEY_AUTO) // 手动
								{
									SystemSaveInfo.system_hand_or_auto_state = KEY_AUTO; // 模式切换为自动
									SystemInfoSave();
									// 存入气灭分区 状态切换为自动
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_SYS_TURN_AUTO, LINKAGE_CLUSTER_ID, SYS_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(SYS_HAND_AUTO_Package_ID, 0); /* 黑匣子:系统自动 */
								FecbusReport_ManualAuto(SYS_HAND_AUTO_Package_ID, 0); /* FECbus:系统自动 */
								}
								break;
						}
						// 判断分区1手自动
						switch((uartbuff[INSCREENSITE].recepetion_buff[6] >> 2) & 0x03)
						{
							case KEY_MANUAL:
								if(SystemSaveInfo.part1_hand_or_auto_state != KEY_MANUAL)
								{
									SystemSaveInfo.part1_hand_or_auto_state = KEY_MANUAL;
									SystemInfoSave();
									// 存入气灭分区 状态切换为手动
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART1_TURN_HAND, LINKAGE_CLUSTER_ID, PART1_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART1_HAND_AUTO_Package_ID, 1); /* 黑匣子:分区1手动 */
								FecbusReport_ManualAuto(PART1_HAND_AUTO_Package_ID, 1); /* FECbus:分区1手动 */
								}
								break;
							case KEY_AUTO:
								if(SystemSaveInfo.part1_hand_or_auto_state != KEY_AUTO)
								{
									SystemSaveInfo.part1_hand_or_auto_state = KEY_AUTO;
									SystemInfoSave();
									// 存入气灭分区 状态切换为自动
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART1_TURN_AUTO, LINKAGE_CLUSTER_ID, PART1_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART1_HAND_AUTO_Package_ID, 0); /* 黑匣子:分区1自动 */
								FecbusReport_ManualAuto(PART1_HAND_AUTO_Package_ID, 0); /* FECbus:分区1自动 */
								}
								break;
						}
						// 判断分区2手自动
						switch((uartbuff[INSCREENSITE].recepetion_buff[6] >> 4) & 0x03)
						{
							case KEY_MANUAL:
								if(SystemSaveInfo.part2_hand_or_auto_state != KEY_MANUAL)
								{
									SystemSaveInfo.part2_hand_or_auto_state = KEY_MANUAL;
									SystemInfoSave();
									// 存入气灭分区 状态切换为手动
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART2_TURN_HAND, LINKAGE_CLUSTER_ID, PART2_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART2_HAND_AUTO_Package_ID, 1); /* 黑匣子:分区2手动 */
								FecbusReport_ManualAuto(PART2_HAND_AUTO_Package_ID, 1); /* FECbus:分区2手动 */
								}
								break;
							case KEY_AUTO:
								if(SystemSaveInfo.part2_hand_or_auto_state != KEY_AUTO)
								{
									SystemSaveInfo.part2_hand_or_auto_state = KEY_AUTO;
									SystemInfoSave();
									// 存入气灭分区 状态切换为自动
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART2_TURN_AUTO, LINKAGE_CLUSTER_ID, PART2_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART2_HAND_AUTO_Package_ID, 0); /* 黑匣子:分区2自动 */
								FecbusReport_ManualAuto(PART2_HAND_AUTO_Package_ID, 0); /* FECbus:分区2自动 */
								}
								break;
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


