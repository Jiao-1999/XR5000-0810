#include "bsp_internal_board.h"

#include "cmsis_os.h"
#include "cmd_process.h"
#include "bsp_debug.h"

#include "bsp_rs485_01.h"

#include "bsp_relay.h"

#include "system.h"

#include "bsp_adc.h"

#include "bsp_save_ctrl.h"

#include "bsp_storage_event.h"  /* ï¿½ï¿½Ï»ï¿½Ó´æ´¢ï¿½Â¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
#include "bsp_fecbus_report.h" /* FECbus RS485 ï¿½Ï±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ (GB4717 ï¿½ï¿½Â¼C) */

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
// ·ÖÇø1 ×´Ì¬
#define getPart1StartState()  (part_1_start_state  ? 1 : 0)
#define getPart1DelayState()  (part_1_start_delay  ? 1 : 0)
#define getPart1SprayState()  (part_1_spray_state  ? 1 : 0)
#define getPart1FeedbackState()  (part_1_feedback  ? 1 : 0)
#define getPart1SoundState()  (part_1_sound_light  ? 1 : 0)
#define getPart1FaultState()  (part_1_fault_state  ? 1 : 0)
// ·ÖÇø2 ×´Ì¬
#define getPart2StartState()  (part_2_start_state  ? 1 : 0)
#define getPart2DelayState()  (part_2_start_delay  ? 1 : 0)
#define getPart2SprayState()  (part_2_spray_state  ? 1 : 0)
#define getPart2FeedbackState()  (part_2_feedback  ? 1 : 0)
#define getPart2SoundState()  (part_2_sound_light  ? 1 : 0)
#define getPart2FaultState()  (part_2_fault_state  ? 1 : 0)
// ±¸ÓÃ/ÌØÊâ LED×´Ì¬
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

// LED ÀàÐÍÃ¶¾Ù
typedef enum {
  LED_MAIN_POWER,           // ¼Ä´æÆ÷0    Ö÷µç
	LED_BACKUP_POWER,         // ¼Ä´æÆ÷1    ±¸µç
	LED_FIRE_ALARM,           // ¼Ä´æÆ÷2    »ð¾¯
	LED_DISCONNECT_FAULT,     // ¼Ä´æÆ÷3
	LED_SILENCE,              // ¼Ä´æÆ÷4
	LED_SYSTEM_FAULT,         // ¼Ä´æÆ÷5
	LED_SYSTEM_START_UP,      // ¼Ä´æÆ÷6
	LED_SYSTEM_START_DELAY,   // ¼Ä´æÆ÷7
	LED_SYSTEM_FEEDBACK,      // ¼Ä´æÆ÷8
	LED_SYSTEM_REGULAR_ALARM, // ¼Ä´æÆ÷9
	LED_SYSTEM_SHIELD,        // ¼Ä´æÆ÷10
	LED_SIREN_STARTUP,        // ¼Ä´æÆ÷11
	LED_SIREN_FAULT,          // ¼Ä´æÆ÷12
	LED_SYSTEM_SELF_CHECK,    // ¼Ä´æÆ÷13
	LED_PART_STARTUP_1,       // ¼Ä´æÆ÷14
	LED_PART_STARTUP_DELAY_1, // ¼Ä´æÆ÷15
	LED_PART_SPRAY_STARTUP_1, // ¼Ä´æÆ÷16
	LED_PART_SPRAY_FEEDBACK_1,// ¼Ä´æÆ÷17
	LED_PART_SOUND_ALARM_1,   // ¼Ä´æÆ÷18
	LED_PART_FAULT_1,         // ¼Ä´æÆ÷19
	LED_PART_STARTUP_2,       // ¼Ä´æÆ÷20
	LED_PART_STARTUP_DELAY_2, // ¼Ä´æÆ÷21
	LED_PART_SPRAY_STARTUP_2, // ¼Ä´æÆ÷22
	LED_PART_SPRAY_FEEDBACK_2,// ¼Ä´æÆ÷23
	LED_PART_SOUND_ALARM_2,   // ¼Ä´æÆ÷24
	LED_PART_FAULT_2,         // ¼Ä´æÆ÷25
	
	// ÐÂÔöLED¼Ä´æÆ÷ 2025/10/10
	LED_COMBUSTIBLE_GAS_SELF_CHECK, // ¿ÉÈ¼ÆøÌå×Ô¼ì // ¼Ä´æÆ÷26
	LED_COMBUSTIBLE_GAS_ALARM,  // ¿ÉÈ¼ÆøÌå±¨¾¯ // ¼Ä´æÆ÷27
	LED_COMBUSTIBLE_GAS_FAULT,  // ¿ÉÈ¼ÆøÌå¹ÊÕÏ // ¼Ä´æÆ÷28
	LED_COMBUSTIBLE_GAS_DELAY,  // ¿ÉÈ¼ÆøÌåÑÓÊ± // ¼Ä´æÆ÷29
	LED_STANDBY_1,              // Ô¤ÁôLED1 // ¼Ä´æÆ÷30
	LED_STANDBY_2,              // Ô¤ÁôLED2 // ¼Ä´æÆ÷31
	LED_STANDBY_3,              // Ô¤ÁôLED3 // ¼Ä´æÆ÷32
	// ÌØÊâLED¼Ä´æÆ÷ 2025/10/10
	LED_COMMUNICATION,          // Í¨ÐÅLED // ¼Ä´æÆ÷33
	LED_WORKING,                // ÔËÐÐÖ¸Ê¾µÆ // ¼Ä´æÆ÷34
    
  LED_COUNT  // LED×ÜÊý
} LED_REGISTER_ID;

// led_valµÆµÄ×´Ì¬Öµ nµÚnÎ» x×´Ì¬
#define setLedxRegisterBit(led_val, n, x) ( (led_val) |= ((x) << (n)) )

// ¶ÀÁ¢°´¼ü¼üÖµ¶¨Òå
typedef enum
{
	KEY_SYSTEM_ANNOUNCIAT = 0x01,  // ÏµÍ³±¨¾¯Æ÷Æô¶¯
	KEY_SYSTEM_LINKAGE_S  = 0x02,  // Áª¶¯Æô¶¯°´ÏÂ
	
	SEPERATE_KEY_NO_PRESS = 0X00,  // ¶ÀÁ¢°´¼üÃ»ÓÐ°´ÏÂ
}BspSeperateKeyPressValue;

// ÊÖ×Ô¶¯×´Ì¬¼üÖµ¶¨Òå
typedef enum
{
	KEY_SYSTEM_MANUAL = 0x01,
	KEY_SYSTEM_AUTO   = 0x02,
	
	KEY_PART1_MANUAL  = 0x04,  // ·ÖÇø1ÇÐ»»ÊÖ¶¯
	KEY_PART1_AUTO    = 0x08,  // ·ÖÇø1ÇÐ»»×Ô¶¯
	
	KEY_PART2_MANUAL  = 0x10,  // ·ÖÇø1ÇÐ»»ÊÖ¶¯
	KEY_PART2_AUTO    = 0x20,  // ·ÖÇø1ÇÐ»»×Ô¶¯
	
}BspHandAutoState;

uint8_t fire_alarm_state    = 0; // »ð¾¯×´Ì¬
uint8_t disconnect_state    = 0; // µôÏß×´Ì¬
uint8_t silencers_state     = 0; // ÏûÒô×´Ì¬
uint8_t system_fault_state  = 0; // ÏµÍ³¹ÊÕÏ×´Ì¬
uint8_t sys_start_state = 0; // Æô¶¯×´Ì¬
uint8_t start_delay_state   = 0; // Æô¶¯ÑÓÊ±×´Ì¬
uint8_t feedbacked_state    = 0; // ·´À¡×´Ì¬
uint8_t regul_alarm_state   = 0; // ¼à¹Ü±¨¾¯×´Ì¬
uint8_t shielding_state     = 0; // ÆÁ±Î×´Ì¬
uint8_t siren_start_state   = 0; // ±¨¾¯Æ÷Æô¶¯×´Ì¬
uint8_t siren_fault_state   = 0; // ±¨¾¯Æ÷¹ÊÕÏ×´Ì¬
uint8_t self_check_state    = 0; // ×Ô¼ì×´Ì¬
// ·ÖÇø1 ×´Ì¬¿ØÖÆ±äÁ¿
uint8_t part_1_start_state  = 0;
uint8_t part_1_start_delay  = 0;
uint8_t part_1_spray_state  = 0;
uint8_t part_1_feedback     = 0;
uint8_t part_1_sound_light  = 0;
uint8_t part_1_fault_state  = 0;
// ·ÖÇø2 ×´Ì¬¿ØÖÆ±äÁ¿
uint8_t part_2_start_state  = 0;
uint8_t part_2_start_delay  = 0;
uint8_t part_2_spray_state  = 0;
uint8_t part_2_feedback     = 0;
uint8_t part_2_sound_light  = 0;
uint8_t part_2_fault_state  = 0;
// ±¸ÓÃLED×´Ì¬¿ØÖÆ±äÁ¿
uint8_t spare_gas_selfcheck = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌå×Ô¼ì
uint8_t spare_gas_alarm     = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌå±¨¾¯
uint8_t spare_gas_fault     = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌå¹ÊÕÏ
uint8_t spare_gas_delay     = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌåÑÓÊ±
uint8_t spare_1_led         = 0; // ±¸ÓÃ 
uint8_t spare_2_led         = 0; // ±¸ÓÃ 
uint8_t spare_3_led         = 0; // ±¸ÓÃ 

uint8_t special_communic    = 0; // Í¨ÐÅLED
uint8_t special_running     = 0; // ÔËÐÐLED    

// ÏµÍ³¸´Î»ÏÔÊ¾¿ØÖÆ
uint8_t system_reset_flag   = 0;
// [7:4] ÀúÊ·×´Ì¬ [3:0] µ±Ç°×´Ì¬
uint8_t screen_show_siren_information = 0;

void LedStateInit(void)
{
	// Ö÷Ãæ°åLED
	fire_alarm_state    = 0; // »ð¾¯×´Ì¬
	disconnect_state    = 0; // µôÏß×´Ì¬
	silencers_state     = 0; // ÏûÒô×´Ì¬
	system_fault_state  = 0; // ÏµÍ³¹ÊÕÏ×´Ì¬
	sys_start_state     = 0; // Æô¶¯×´Ì¬
	start_delay_state   = 0; // Æô¶¯ÑÓÊ±×´Ì¬
	feedbacked_state    = 0; // ·´À¡×´Ì¬
	regul_alarm_state   = 0; // ¼à¹Ü±¨¾¯×´Ì¬
	shielding_state     = 0; // ÆÁ±Î×´Ì¬
	siren_start_state   = 0; // ±¨¾¯Æ÷Æô¶¯×´Ì¬
	siren_fault_state   = 0; // ±¨¾¯Æ÷¹ÊÕÏ×´Ì¬
	self_check_state    = 0; // ×Ô¼ì×´Ì¬
	// ·ÖÇø1 ×´Ì¬¿ØÖÆ±äÁ¿
	part_1_start_state  = 0;
	part_1_start_delay  = 0;
	part_1_spray_state  = 0;
	part_1_feedback     = 0;
	part_1_sound_light  = 0;
	part_1_fault_state  = 0;
	// ·ÖÇø2 ×´Ì¬¿ØÖÆ±äÁ¿
	part_2_start_state  = 0;
	part_2_start_delay  = 0;
	part_2_spray_state  = 0;
	part_2_feedback     = 0;
	part_2_sound_light  = 0;
	part_2_fault_state  = 0;
	
	// ±¸ÓÃLED×´Ì¬¿ØÖÆ±äÁ¿
	spare_gas_selfcheck = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌå×Ô¼ì
	spare_gas_alarm     = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌå±¨¾¯
	spare_gas_fault     = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌå¹ÊÕÏ
	spare_gas_delay     = 0; // ±¸ÓÃ ¿ÉÈ¼ÆøÌåÑÓÊ±
	spare_1_led         = 0; // ±¸ÓÃ 
	spare_2_led         = 0; // ±¸ÓÃ 
	spare_3_led         = 0; // ±¸ÓÃ 
	// ÌØÊâLED
	special_communic    = 0; // Í¨ÐÅLED
	special_running     = 0; // ÔËÐÐLED  
	
	// ½«ÊÖ±¨×´Ì¬³õÊ¼»¯
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

// ·ÖÇø1
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

// 2025/10/28 10:10 Ìí¼Ó·ÖÇø2¿ØÖÆ
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
	
	// ¸ù¾ÝÖ÷µç×´Ì¬ÐÞ¸ÄLEDÖµµÄ×´Ì¬
	setLedxRegisterBit(led_state_buff[LED_MAIN_POWER/8]          , LED_MAIN_POWER            %8, getMainPowerState()  ); // Ö÷µçLED
	setLedxRegisterBit(led_state_buff[LED_BACKUP_POWER/8]        , LED_BACKUP_POWER          %8, getSparePowerState() ); // ±¸µç
	setLedxRegisterBit(led_state_buff[LED_FIRE_ALARM/8]          , LED_FIRE_ALARM            %8, getFireAlarmState()  ); // »ð¾¯
	setLedxRegisterBit(led_state_buff[LED_DISCONNECT_FAULT/8]    , LED_DISCONNECT_FAULT      %8, getDisconnectState() ); // µôÏß¹ÊÕÏ
	setLedxRegisterBit(led_state_buff[LED_SILENCE/8]             , LED_SILENCE               %8, getSilencersState()  ); // ÏûÒôLED
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_FAULT/8]        , LED_SYSTEM_FAULT          %8, getSystemFaultState()); // ÏµÍ³¹ÊÕÏ
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_START_UP/8]     , LED_SYSTEM_START_UP       %8, getInitiationState() ); // Æô¶¯
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_START_DELAY/8]  , LED_SYSTEM_START_DELAY    %8, getStartDelayState() ); // Æô¶¯ÑÓÊ±
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_FEEDBACK/8]     , LED_SYSTEM_FEEDBACK       %8, getFeedbackedState() ); // ·´À¡
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_REGULAR_ALARM/8], LED_SYSTEM_REGULAR_ALARM  %8, getRegulAlarmState() ); // ¼à¹Ü±¨¾¯
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_SHIELD/8]       , LED_SYSTEM_SHIELD         %8, getShieldingState()  ); // ÆÁ±Î×´Ì¬
	setLedxRegisterBit(led_state_buff[LED_SIREN_STARTUP/8]       , LED_SIREN_STARTUP         %8, getSirenStartState() ); // ±¨¾¯Æ÷Æô¶¯LED
	setLedxRegisterBit(led_state_buff[LED_SIREN_FAULT/8]         , LED_SIREN_FAULT           %8, getSirenFaultState() ); // ±¨¾¯Æ÷¹ÊÕÏLED
	setLedxRegisterBit(led_state_buff[LED_SYSTEM_SELF_CHECK/8]   , LED_SYSTEM_SELF_CHECK     %8, getSelfCheckState()  ); // ×Ô¼ì¿ªÆôLED ½áÊøºó×Ô¶¯¹Ø±Õ
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_1/8]      , LED_PART_STARTUP_1        %8, getPart1StartState() ); // ·ÖÇøÆô¶¯1Æô¶¯LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_DELAY_1/8], LED_PART_STARTUP_DELAY_1  %8, getPart1DelayState() ); // ·ÖÇø1Æô¶¯ÑÓÊ±LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_1/8], LED_PART_SPRAY_STARTUP_1  %8, getPart1SprayState() ); // ·ÖÇø1Åç·ÅÆô¶¯×´Ì¬LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_1/8], LED_PART_SPRAY_STARTUP_1  %8, getPart1FeedbackState()); // ·ÖÇø1Åç·Å·´À¡LED
	setLedxRegisterBit(led_state_buff[LED_PART_SOUND_ALARM_1/8]  , LED_PART_SOUND_ALARM_1    %8, getPart1SoundState() ); // ·ÖÇø1Éù¹â×´Ì¬
	setLedxRegisterBit(led_state_buff[LED_PART_FAULT_1/8]        , LED_PART_FAULT_1          %8, getPart1FaultState() ); // ·ÖÇø1¹ÊÕÏ×´Ì¬LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_2/8]      , LED_PART_STARTUP_2        %8, getPart2StartState() ); // ·ÖÇøÆô¶¯2Æô¶¯LED
	setLedxRegisterBit(led_state_buff[LED_PART_STARTUP_DELAY_2/8], LED_PART_STARTUP_DELAY_2  %8, getPart2DelayState() ); // ·ÖÇø2Æô¶¯ÑÓÊ±LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_STARTUP_2/8], LED_PART_SPRAY_STARTUP_2  %8, getPart2SprayState() ); // ·ÖÇø2Åç·ÅÆô¶¯×´Ì¬LED
	setLedxRegisterBit(led_state_buff[LED_PART_SPRAY_FEEDBACK_2/8], LED_PART_SPRAY_FEEDBACK_2%8, getPart2FeedbackState()); // ·ÖÇø2Åç·Å·´À¡LED
	setLedxRegisterBit(led_state_buff[LED_PART_SOUND_ALARM_2/8]   , LED_PART_SOUND_ALARM_2   %8, getPart2SoundState() ); // ·ÖÇø2Éù¹â×´Ì¬
	setLedxRegisterBit(led_state_buff[LED_PART_FAULT_2/8]         , LED_PART_FAULT_2         %8, getPart2FaultState() ); // ·ÖÇø2¹ÊÕÏ×´Ì¬LED
	// ÐÂÔö±¸ÓÃLED¿ØÖÆ 2025/10/10 16:56
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
	uint8_t modbusbuf[24]; // ·¢ËÍ»º³åÇø
	uint16_t crc16 = 0x0000; // CRCÐ£ÑéÂë
	for(;;)
	{
		if(system_reset_flag == 1)
		{
			modbusbuf[0] = 0xFF; // ´Ó»úµØÖ·È¡Öµ
			modbusbuf[1] = 0x05; // 05¹¦ÄÜÂë
			modbusbuf[2] = 0x00;
			modbusbuf[3] = 20;
			modbusbuf[4] = 0xFF;
			modbusbuf[5] = 0x00; // 
			crc16 = CalcCrc16(modbusbuf, 6); // ¼ÆËãÁù×Ö½ÚÊý¾ÝµÄCRCÐ£ÑéÂë
			modbusbuf[6] = crc16 >> 0;
			modbusbuf[7] = crc16 >> 8;
			
			InternalBoardSendString(modbusbuf, 8);
			
		}
		else
		{
			if(self_check_show_content == 1) // LEDÖ¸Ê¾µÆ×Ô¼ì
			{
				uint8_t data_count = 0;
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // Ê¹ÓÃ04¹¦ÄÜÂë ·¢ËÍÈ«²¿×´Ì¬
				modbusbuf[data_count++] = 6;  // ·µ»ØµÄ×Ö½ÚÊý
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = 0xFF; // È«²¿µãÁÁ
				}
				modbusbuf[data_count++] = 0;
				modbusbuf[data_count++] = 0; // ¹Ø±Õ·äÃùÆ÷

				crc16 = CalcCrc16(modbusbuf, data_count); // ¼ÆËãÁù×Ö½ÚÊý¾ÝµÄCRCÐ£ÑéÂë
				modbusbuf[data_count++] = (uint8_t)(crc16);
				modbusbuf[data_count++] = (crc16 >> 8);
				
				InternalBoardSendString(modbusbuf, data_count); // ·¢ËÍÊý¾Ýµ½3303
				
				crc16 = 0;
				
				osDelay(2000);
				self_check_show_content = 2;
			}
			else if(self_check_show_content == 2) // ·äÃùÆ÷×Ô¼ì
			{
				uint8_t data_count = 0;
				uint8_t led_reg_state[5] = { 0 }; // Ä¬ÈÏ¹Ø±Õ
				
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // Ê¹ÓÃ04¹¦ÄÜÂë ·¢ËÍÈ«²¿×´Ì¬
				modbusbuf[data_count++] = 6;  // ·µ»ØµÄ×Ö½ÚÊý
				
				getLedStateReg(led_reg_state, 5);
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = led_reg_state[i]; // È«²¿Ãðµô
				}
				modbusbuf[data_count++] = 0;
				modbusbuf[data_count++] = 1; // ´ò¿ª·äÃùÆ÷ Ãù½ÐÒ»Ãë

				crc16 = CalcCrc16(modbusbuf, data_count); // ¼ÆËãÁù×Ö½ÚÊý¾ÝµÄCRCÐ£ÑéÂë
				modbusbuf[data_count++] = (uint8_t)(crc16);
				modbusbuf[data_count++] = (crc16 >> 8);
				
				InternalBoardSendString(modbusbuf, data_count); // ·¢ËÍÊý¾Ýµ½3303
				
				crc16 = 0;
				
				osDelay(2000);
				self_check_show_content = 3;
			}
			else
			{
				uint8_t led_reg_state[5] = { 0 }; // Ä¬ÈÏ¹Ø±Õ
				
				getLedStateReg(led_reg_state, 5);

				uint8_t data_count = 0;
				modbusbuf[data_count++] = InternalBoardAddr;
				modbusbuf[data_count++] = 0x04;  // Ê¹ÓÃ04¹¦ÄÜÂë ·¢ËÍÈ«²¿×´Ì¬
				modbusbuf[data_count++] = 6;  // ·µ»ØµÄ×Ö½ÚÊý
				
				for(uint8_t i = 0; i < 5; i++)
				{
					modbusbuf[data_count++] = led_reg_state[i];
				}
				modbusbuf[data_count++] = 0;
				modbusbuf[data_count++] = getBuzzerWorkState();

				crc16 = CalcCrc16(modbusbuf, data_count); // ¼ÆËãÁù×Ö½ÚÊý¾ÝµÄCRCÐ£ÑéÂë
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

static uint8_t key_value_storage = NONE_KEY; // Ä¬ÈÏÃ»ÓÐ°´¼ü°´ÏÂ

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

uint8_t part1_hand_or_auto_state = KEY_AUTO; // Ä¬ÈÏ×Ô¶¯
uint8_t getPart1HandAutoState(void)
{
	return part1_hand_or_auto_state;
}

uint8_t sys_hand_or_auto_state = KEY_AUTO; // Ä¬ÈÏ×Ô¶¯
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
	uint16_t crc16 = 0x0000; // CRCÐ£ÑéÂë
	
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
							case KEY1_INFORM_CERTAIN  : // ÐÅÏ¢È·ÈÏ
								break;
							case KEY2_SELF_INSPECTION : // ÏµÍ³×Ô¼ì
								key_value_storage = SELFCHECK_KEY; // ¸³Öµ×Ô¼ì
								taskENTER_CRITICAL();
								SetScreen(53);	// ½øÈë¶þ¼¶ÃÜÂëÒ³
								taskEXIT_CRITICAL();	
								osDelay(5);
								GetScreen();
								break;
							case KEY3_SYSTEM_SILENCE  : { // ÏµÍ³ÏûÒô
//								key_value_storage = SILENSE_KEY;
								if( main_power_beep_ctrl     != 0 || 
										linkage_beep_ctrl        != 0 ||
										beep_fire_ctrl           != 0 ||
										beep_fault_ctrl          != 0 || 
										beep_spray_feedback_ctrl != 0 || 
										beep_general_io_ctrl     != 0    
								) 
								{		
									silencers_state = 1; // ÏûÒô±êÖ¾
								}
								
								beep_spray_feedback_ctrl = 0;
								
								beep_fire_ctrl = 0;
								beep_fault_ctrl = 0;

								main_power_beep_ctrl = 0;
								linkage_beep_ctrl = 0;
								beep_general_io_ctrl = 0;
								break;
							}
							case KEY4_SYSTEM_RESET    : // ÏµÍ³¸´Î»
								key_value_storage = RESET_KEY;
								taskENTER_CRITICAL();
								SetScreen(53);	// ½øÈë¶þ¼¶ÃÜÂëÒ³
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
							case KEY6_DIRECTION_UP    : // ·½Ïò¼üÉÏ
								
								break;
							case KEY7_DIRECTION_RIGHT : // ·½Ïò¼üÓÒ
								
								break;
							case KEY8_DIRECTION_DOWN  : // ·½Ïò¼üÏÂ
								
								break;
							case KEY9_DIRECTION_LEFT  : // ·½Ïò¼ü×ó
								
								break;
							case KEY10_DIRECTION_OK   : // ·½Ïò¼üok
								
								break;
							case KEY11_PART1_SOUNDLT  : // ·ÖÇø1Éù¹â
								break;
							case KEY12_PART1_STOP     : // ·ÖÇø1Í£Ö¹
								break;
							case KEY13_PART2_SOUNDLT  :  // ·ÖÇø2Éù¹â
								break;
							case KEY14_PART2_STOP     :  // ·ÖÇø2Í£Ö¹
								break;
							case KEY15_PART1_SPRAY_ST : { // ·ÖÇø1ÅçÈ÷Æô¶¯
								key_value_storage = PART1_SPRY_START;
								SetScreen(53);								
								osDelay(5);
								GetScreen();
								break;
							}
							case KEY16_PART2_SPRAY_ST : { // ·ÖÇø2ÅçÈ÷Æô¶¯
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
							case KEY_SYSTEM_ANNOUNCIAT : // ÏµÍ³±¨¾¯Æ÷Æô¶¯
								key_value_storage = SIREN_KEY; // 
//								taskENTER_CRITICAL();
//								SetScreen(53);	// ½øÈë¶þ¼¶ÃÜÂëÒ³
//								taskEXIT_CRITICAL();
								SetScreen(53);								
								osDelay(5);
								GetScreen();
								break;
							case KEY_SYSTEM_LINKAGE_S  : // Áª¶¯Æô¶¯°´ÏÂ
								key_value_storage = LINKAGE_START_KEY; // 
								StartupLinkageDevice();
								StorageEvent_LogStart(LINKAGE_CLUSTER_ID, DEV_TYPE_CONTROL_DEV); /* ï¿½ï¿½Ï»ï¿½ï¿½:ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
							FecbusReport_Start(LINKAGE_CLUSTER_ID, DEV_TYPE_CONTROL_DEV); /* FECbus:ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
								break;
							default:
								break;
						}
						// ÅÐ¶ÏÏµÍ³ÊÖ×Ô¶¯
						switch(uartbuff[INSCREENSITE].recepetion_buff[6] & 0x03)
						{
							case KEY_MANUAL:
								if(SystemSaveInfo.system_hand_or_auto_state != KEY_MANUAL) // ÊÖ¶¯
								{
									SystemSaveInfo.system_hand_or_auto_state = KEY_MANUAL; // ÊÖ¶¯
									SystemInfoSave();
									// ´æÈëÆøÃð·ÖÇø ×´Ì¬ÇÐ»»ÎªÊÖ¶¯
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_SYS_TURN_HAND, LINKAGE_CLUSTER_ID, SYS_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(SYS_HAND_AUTO_Package_ID, 1); /* ï¿½ï¿½Ï»ï¿½ï¿½:ÏµÍ³ï¿½Ö¶ï¿½ */
								FecbusReport_ManualAuto(SYS_HAND_AUTO_Package_ID, 1); /* FECbus:ÏµÍ³ï¿½Ö¶ï¿½ */
								}
								break;
							case KEY_AUTO:
								if(SystemSaveInfo.system_hand_or_auto_state != KEY_AUTO) // ÊÖ¶¯
								{
									SystemSaveInfo.system_hand_or_auto_state = KEY_AUTO; // Ä£Ê½ÇÐ»»Îª×Ô¶¯
									SystemInfoSave();
									// ´æÈëÆøÃð·ÖÇø ×´Ì¬ÇÐ»»Îª×Ô¶¯
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_SYS_TURN_AUTO, LINKAGE_CLUSTER_ID, SYS_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(SYS_HAND_AUTO_Package_ID, 0); /* ï¿½ï¿½Ï»ï¿½ï¿½:ÏµÍ³ï¿½Ô¶ï¿½ */
								FecbusReport_ManualAuto(SYS_HAND_AUTO_Package_ID, 0); /* FECbus:ÏµÍ³ï¿½Ô¶ï¿½ */
								}
								break;
						}
						// ÅÐ¶Ï·ÖÇø1ÊÖ×Ô¶¯
						switch((uartbuff[INSCREENSITE].recepetion_buff[6] >> 2) & 0x03)
						{
							case KEY_MANUAL:
								if(SystemSaveInfo.part1_hand_or_auto_state != KEY_MANUAL)
								{
									SystemSaveInfo.part1_hand_or_auto_state = KEY_MANUAL;
									SystemInfoSave();
									// ´æÈëÆøÃð·ÖÇø ×´Ì¬ÇÐ»»ÎªÊÖ¶¯
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART1_TURN_HAND, LINKAGE_CLUSTER_ID, PART1_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART1_HAND_AUTO_Package_ID, 1); /* ï¿½ï¿½Ï»ï¿½ï¿½:ï¿½ï¿½ï¿½ï¿½1ï¿½Ö¶ï¿½ */
								FecbusReport_ManualAuto(PART1_HAND_AUTO_Package_ID, 1); /* FECbus:ï¿½ï¿½ï¿½ï¿½1ï¿½Ö¶ï¿½ */
								}
								break;
							case KEY_AUTO:
								if(SystemSaveInfo.part1_hand_or_auto_state != KEY_AUTO)
								{
									SystemSaveInfo.part1_hand_or_auto_state = KEY_AUTO;
									SystemInfoSave();
									// ´æÈëÆøÃð·ÖÇø ×´Ì¬ÇÐ»»Îª×Ô¶¯
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART1_TURN_AUTO, LINKAGE_CLUSTER_ID, PART1_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART1_HAND_AUTO_Package_ID, 0); /* ï¿½ï¿½Ï»ï¿½ï¿½:ï¿½ï¿½ï¿½ï¿½1ï¿½Ô¶ï¿½ */
								FecbusReport_ManualAuto(PART1_HAND_AUTO_Package_ID, 0); /* FECbus:ï¿½ï¿½ï¿½ï¿½1ï¿½Ô¶ï¿½ */
								}
								break;
						}
						// ÅÐ¶Ï·ÖÇø2ÊÖ×Ô¶¯
						switch((uartbuff[INSCREENSITE].recepetion_buff[6] >> 4) & 0x03)
						{
							case KEY_MANUAL:
								if(SystemSaveInfo.part2_hand_or_auto_state != KEY_MANUAL)
								{
									SystemSaveInfo.part2_hand_or_auto_state = KEY_MANUAL;
									SystemInfoSave();
									// ´æÈëÆøÃð·ÖÇø ×´Ì¬ÇÐ»»ÎªÊÖ¶¯
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART2_TURN_HAND, LINKAGE_CLUSTER_ID, PART2_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART2_HAND_AUTO_Package_ID, 1); /* ï¿½ï¿½Ï»ï¿½ï¿½:ï¿½ï¿½ï¿½ï¿½2ï¿½Ö¶ï¿½ */
								FecbusReport_ManualAuto(PART2_HAND_AUTO_Package_ID, 1); /* FECbus:ï¿½ï¿½ï¿½ï¿½2ï¿½Ö¶ï¿½ */
								}
								break;
							case KEY_AUTO:
								if(SystemSaveInfo.part2_hand_or_auto_state != KEY_AUTO)
								{
									SystemSaveInfo.part2_hand_or_auto_state = KEY_AUTO;
									SystemInfoSave();
									// ´æÈëÆøÃð·ÖÇø ×´Ì¬ÇÐ»»Îª×Ô¶¯
									BspCommonDataSaveApp(GASER_FLASH_SAVE, OTHER_PART2_TURN_AUTO, LINKAGE_CLUSTER_ID, PART2_HAND_AUTO_Package_ID);
									StorageEvent_LogManualAuto(PART2_HAND_AUTO_Package_ID, 0); /* ï¿½ï¿½Ï»ï¿½ï¿½:ï¿½ï¿½ï¿½ï¿½2ï¿½Ô¶ï¿½ */
								FecbusReport_ManualAuto(PART2_HAND_AUTO_Package_ID, 0); /* FECbus:ï¿½ï¿½ï¿½ï¿½2ï¿½Ô¶ï¿½ */
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


