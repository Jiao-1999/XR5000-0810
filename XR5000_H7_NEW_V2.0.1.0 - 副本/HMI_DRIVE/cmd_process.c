#include "cmd_process.h"
#include "hmi_driver.h"
#include "24c04.h"
#include "w25qxx.h"
#include "system.h"
#include "bsp_relay.h"

#include "bsp_adc.h"

#include "bsp_logic_set.h"
#include "bsp_logic_screen.h" /* ??????????????/?????›¨?? */

#include "bsp_debug.h"

#include "bsp_screen.h"

#include "bsp_rs485_01.h"

#include "bsp_internal_board.h"

#include "bsp_super.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"

#include "bsp_linkage_ctrl.h"
#include "bsp_save_ctrl.h"
#include "bsp_rs485_detect.h"
#include "bsp_mbus_control.h"
#include "bsp_device_registry.h"
#include "bsp_device_threshold.h"
#include "bsp_aht20.h"

#include "bsp_key.h"

#include "bsp_ctrl_bus.h"
#include "bsp_mbus.h"
#include "bsp_fdcan1.h"
#include "bsp_can_monitor.h"

#include "bsp_password.h"

#include "bsp_storage_event.h"  /* ?????›¥???????? */
#include "bsp_fecbus_report.h" /* FECbus RS485 ???????? (GB4717 ???C) */

#include <time.h>

#define CANG_USER_NUM 24
#define PACK_USER_NUM 20

#define ONLINE_TIMEOUT 15

/* XR5000_CHECK_CHANGE_20260804: screen 72 check-mode timing and refresh state. */
#define CHECK_SCREEN_ID 72U
#define CHECK_RETURN_SCREEN_ID 1U
#define CHECK_TIMEOUT_TICKS pdMS_TO_TICKS(60000U)
static uint32_t check_last_activity_tick = 0U;
static uint16_t check_last_values[20];
static uint8_t check_values_valid = 0U;
static volatile uint8_t check_record_pending = 0U;

void SystemCheckRequestRecord(void)
{
	/* XR5000_CHECK_FLASH_FIX_20260804: collapse repeated key frames into one deferred record. */
	check_record_pending = 1U;
}

#define Fault_Show_Zone 7
#define getTotalPage(Fault_Sum) (((Fault_Sum) + Fault_Show_Zone - 1) / Fault_Show_Zone)

#define Alarm_Show_Zone 7
#define getScrollSum(Scroll_Num) ( ( (Scroll_Num) > 1 ) ? ((Scroll_Num) -1) : 1)

#define Out_Fire_Show_Zone 7

uint8_t secs,years,months,weeks,days,hours,minutes;
uint8  cmd_buffer[CMD_MAX_SIZE];                                    //?????
uint16 current_screen_id = 0;                                       //???????ID
uint32_t yonghumima=0,yonghumm1=0,yonghumm2=0,yonghumm3=0,mmsdSTA=0;//???????
uint32_t chaojimima=68686668;//????????

uint8_t mimajiyi=0;
uint8_t miehuoqidong=0;

// new
typedef struct
{
	uint16_t years;    // ??????
	
	uint8_t  months;   // ??????
	uint8_t  days;     // ??????
	
	uint8_t  hours;    // ?????
	uint8_t  minute;   // ??????
	
	uint8_t  second;   // ??????
}AlarmTimeRecord;

typedef enum
{
	PackClassID  = 1,
	CabinClassID = 2, 
	LinkageClassID = 3,
}PackCabinClassID; // ???????

typedef enum
{
	Temperature = 1,  // ???
	Smoke       = 2,  // ????
	HandAlarm   = 3,  // ???
	Hydrogen    = 4,  // ????
	Carbon      = 5,  // ??????
	
	// XR5000_LOOP3_CHANGE_20260727: VOC and CH4 use separate forewarn record types.
	Voc         = 9,
	Methane     = 10,
	Loop3TempWarning = 11,
	Loop3CarbonFire = 12,
	Loop3HydrogenFire = 13,
	Loop1TempWarning = 14,
	Loop1SmokeWarning = 15,
	AlarmCtrlKey = 6, // ??????
	
	FeedBack1Press = 7,
	
	SysFlashFault = 8,

}DetectorAlarmType; //?????????????

typedef enum
{
	OUT_FIRE_NO_START  = 1,  // ??????¦Ä???
	OUT_FIRE_SUSPEND   = 2,  // ???????????

	SPRAY_START_DELAY  = 3,  // ???????
	SPRAY_INTERVAL_T1  = 4,  // ??? ?????2?????
	SPRAY_SECOND_DELAY = 5,  // ??2???????????
	SPRAY_INTERVAL_T2  = 6,  // ??? ?????3?????
	SPRAY_THIRD_DELAY  = 7,  // ??3???????????
}OutFireDeviceState; // ????????


typedef struct
{
	uint8_t cluster_id; // ??id
	uint8_t pack_id;    // ?????????¦Ì?pack id
	
	uint8_t cabin_id;   // ??id

	int8_t lunch_state; // ????§Ù????? ????????????? -1:??? -2:???????? 0:¦Ä??? 99:??????????????

	AlarmTimeRecord atr;

}PackAlarmStorage;    // ????y??????????????????????

PackAlarmStorage pas[224] = { 0 };

uint8_t pas_pointer = 0;
uint8_t last_pas_len = 0;
uint8_t pas_fresh_point = 0;// ????????????

uint8_t multiple_alarm_fresh_flag = 0;
uint8_t pas_traverse_pointer = 1;

uint8_t alarm_number = 0; // ????????
uint8_t last_alarm_num = 255; // ?????????

uint8_t last_online_detector_num = 255;
uint8_t last_disconnect_detector_num = 255;
uint8_t home_statistics_force_refresh = 1;

typedef struct
{
	uint8_t cabin_id;       // ??ID
	
	uint8_t cluster_id;     // ??ID
	uint8_t pack_id;        // ??ID
}DetectorAddrAttribute;   // ???????????????????????

typedef struct
{
	uint8_t temperature_type; // ???
	uint8_t carbon_type;      // ??????
	uint8_t smoke_type;       // ????
}CabinAlarmType;     //???????????????????

typedef struct
{
	uint8_t temperature_type; // ???
	uint8_t carbon_type;      // ??????
	uint8_t smoke_type;       // ????
}PackAlarmType;       ///????????????????????

uint8_t fore_alarm_start_index = 0;
uint8_t fire_alarm_start_index = 0; // ???????????????????????
uint8_t fire_alarm_check_new_flag = 0;
uint8_t force_alarm_check_new_flag = 0;
#define getFireAlarmCheckNewKey() fire_alarm_check_new_flag
#define getForceAlarmCheckNewKey() force_alarm_check_new_flag

/* ??? ??????›¥
 * 2025/07/08?????
 * ??????????????????›¥
 */
// ????›¥ ?????? ???§á??????
typedef struct {
	uint8_t self_bottom_point;       // ?????????????
	
	uint8_t point_history_len;       // ???????????

	uint8_t detector_class[224];     // ?????????
	
	uint8_t alarm_type[224];				 // ??????????
	
	DetectorAddrAttribute da[224];   // ?????????

	AlarmTimeRecord atr[224];        // ?????????
	
	uint16_t fresh_time_count;        // ??????????
}PackCabinForeWarnStorage;  // ???????????????›¥

PackCabinForeWarnStorage pcfws = {
	.self_bottom_point = 0,
	.point_history_len = 255,
	.detector_class = {0},
	.da         = {0},
	.alarm_type = {0},
	.atr        = {0}
};

// ???›¥ ???? ???? ???
typedef struct {
	uint8_t self_bottom_point;       // ?????????????
	
	uint8_t point_history_len;       // ???????????
	
	uint8_t detector_class[224];     // ?????????
	
	uint8_t alarm_type[224];				 // ??????????
	
	DetectorAddrAttribute da[224];   // ?????????
	
	AlarmTimeRecord atr[224];        // ?????????
	
	uint16_t fresh_time_count;       // ?????????
}PackCabinFireAlarmStorage;

PackCabinFireAlarmStorage pcfas = {
	.self_bottom_point = 0,
	.point_history_len = 255,
	.detector_class = {0},
	.alarm_type = {0},
	.da = {0},
	.atr = {0}
};


typedef struct
{
	uint8_t detector_class; // ?????????
	
	DetectorAddrAttribute da;   // ?????????
	
	AlarmTimeRecord atr;        // ?????????
	uint8_t fault_type;
	
}PackCabinFaultStorage;   // ?????›¥???? ???????	
uint8_t pcfs_fresh_ctrl = 255;    // ????????? ???????????
uint8_t pcfs_buttom_point = 0;  // ¦Â??? ??????????
PackCabinFaultStorage pcfs[224] = {0}; // ???????§Û??????

uint8_t fault_check_new_flag = 0;
uint8_t fault_current_page = 0;
#define getFaultCheckNewKey() fault_check_new_flag
// end

// new
// ????????????????
typedef struct
{
	uint8_t cabin_alarm_state  ; 
	uint8_t cluster_alarm_state;

}FireAlarmStorage;
FireAlarmStorage fire_alarm_flag = {0}; // ?????????0

typedef struct
{
	uint8_t fire_alarm_id_buff[300]; // ?›¥???????
	uint8_t faib_buttom_point;      // fire_alarm_id_buff???????
	uint8_t storage_pas_len;        // ?›¥????????? ?????????????????
}FireAlarmNumRecord; // ?????????????????
FireAlarmNumRecord fanr = {0};

/*********
* ?????????????/??
*  1 ?????? ???xx??
*  2 ??????????
*  3 ?????????? ????xx??
*  4 ?????????????????
*  5 ?????????????? ???xx??
*  6 ??????????????????
*  7 ?????????? ????xx??
*  8 ?????????????????
*  9 ??????????????? ???xx??
* 10 ???????????????????
* 11 ??????????? ????xx??
* 12 ??????
* 13 ??????????????????? ???-1
* 14 ????¦Ä??????-2 ????????????????
*/
typedef enum
{
	FIRE_EXTINGUISH_FORCE_START    = -4,    // ?????????
	FIRE_EXTINGUISH_RESTART_FINISH = -3,    // ??????????????? ?????¡ä????
	FIRE_EXTINGUISH_CAN_RESTART    = -2,    // ???????????????
	FIRE_EXTINGUISH_FORCE_STOP     = -1,    // ??????
	
	FIRE_EXTINGUISH_MODE_JUDGEMENT       = 0,      // ?§Ø?????
	FIRE_EXTINGUISH_START_SPRAY_DELAY    = 1,      // ??????
	FED_START_SPRAY_DELAY_FINISH_FLAG    = 2,      // ?????????????? ???????
	FIRE_EXTINGUISH_FIRST_SPRAY_START    = 3,      // ??????????
	FIRE_EXTINGUISH_FIRST_SPRAY_FINISH   = 4,      // ???????????
	FIRE_EXTINGUISH_SECOND_SPRAY_DELAY   = 5,      // ??????????????
	FED_SECOND_SPRAY_DELAY_FINISH_FLAG   = 6,      // ?????????????????? ???????
	FIRE_EXTINGUISH_SECOND_SPRAY_START   = 7,      // ??????????
	FIRE_EXTINGUISH_SECOND_SPRAY_FINISH  = 8,      // ???????????
	FIRE_EXTINGUISH_THIRD_SPRAY_DELAY    = 9,      // ???????????????
	FED_THIRD_SPRAY_DELAY_FINISH_FLAG    = 10,     // ????????????????????
	FIRE_EXTINGUISH_THIRD_SPRAY_START    = 11,     // ???????????
	FIRE_EXTINGUISH_THIRD_SPRAY_FINISH   = 12,     // ????????????
	FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE   = 13,     // ?????????

	FIRE_EXTINGUISH_CLUSTER_VALVE_OPEN   = 14, // ???????
	FIRE_EXTINGUISH_CABIN_VALVE_OPEN     = 15, // ???????
	FIRE_EXTINGUISH_CYLINDEF_1_OPENED    = 16, // ???1??????
	FIRE_EXTINGUISH_CYLINDEF_2_OPENED    = 17, // ???2??????
	
	FIRE_EXTINGUISH_STARYUP_FINISH_FLAG  = 18, // ????????????????
	
	FEEDBACK_1_PRESS = 19,
	FEEDBACK_2_PRESS = 20,
	
}FireExtinguishDeviceActionType; // ?????????????

// ??????¨¹??
typedef struct
{
	uint8_t cluster_id[224];   // ??id
	uint8_t pack_id[224];      // ?????????¦Ì?pack id
	
	uint8_t cabin_id[224];     // ??id
	
	AlarmTimeRecord atr[224];  // ?????????
	
	int8_t fed_action[224];   // ????????????
	
	uint8_t self_point_len;    // 
	uint8_t last_point_len;    // ???????
	
	uint16_t countdown_val[224];    // ???????? 
	uint16_t start_cntd_time[224];  // ??????????????
	uint16_t curr_cntd_time[224];   // ???????? ????????????
}FireExtinguishDeviceActionSave; // ????????????

FireExtinguishDeviceActionSave fedas = {
	.cluster_id = {0},
	.pack_id    = {0},
	.cabin_id   = {0},
	.atr        = {0},
	.fed_action = {0},
	
	.self_point_len = 0,
	.last_point_len = 255
};

uint8_t fed_fresh_flag = 0;
#define getFireEDFresh() fed_fresh_flag
uint8_t fedas_fresh_point = 0;
// end

// new ????????? ???????
typedef struct
{
	uint8_t  curr_detector_page; // ???????
	uint8_t  last_detector_page; // ?????????
	
	uint8_t  detector_offline_fresh_flag[11];
	
	uint8_t  detect_online_state[21][11];   // ??????
	uint8_t  detect_shield_state[21][11];   // ??????
	
	uint8_t  last_temperat_state[21][11]; // ??????????
	uint8_t  last_temperature[21][11];   // ????¦Å????
	
	uint8_t  last_smoke_state[21][11];   // ???????????
	
	uint8_t  last_co_state[21][11];      // 
	uint16_t last_co_concentrat[21][11]; // ????¦Ï??????????

}DetectorDataShowCtrl;

DetectorDataShowCtrl ddsc;

typedef struct
{
	uint8_t force_fresh_flag;    // ?????¡À??¦Ë
	
	uint8_t curr_detector_page;  // ????
	uint8_t last_detector_page;  // ???
	
	uint8_t curr_pack_id;
	
	uint8_t last_temperat_state; // ??????????
	uint8_t last_temperature;    // ????¦Å????
	
	uint8_t last_smoke_state;   // ???????????
	
	uint8_t last_co_state;      // 
	
	uint8_t lat_detector_online_num[4]; // ??????????? 
	
	uint8_t last_derector_state[3][32];
	
	uint16_t last_co_concentrat; // ????¦Ï??????????
}DetectorDataShowCtrl_32Pack;

DetectorDataShowCtrl_32Pack ddsc_32p;

typedef struct
{
	uint8_t force_fresh_flag;
	
	uint8_t curr_cabin_id;
	
	uint8_t last_temperat_state; // ??????????
	uint8_t last_temperat_value; // ????¦Å????
	
	uint8_t last_smoke_state;   // ???????????
	
	uint8_t last_co_state;      // 
	uint8_t last_hh_state;
	
	uint8_t last_detector_model;
	
	uint8_t last_sensor_mode;
	
	uint8_t lasr_cabin_state;
	
	uint16_t last_co_value;      // 
	uint16_t last_hh_value;
}CabinDataShowCtrl_t;

CabinDataShowCtrl_t cabin_dsc = {0};

typedef enum{
	ForceAlarmPart = 0,
	FaultPart      = 1,
	FireAlarmPart  = 2,
	OutFirePart    = 3,
}BspPartitionId;

typedef enum
{
	OutMenu = 0, // ????? ????????§Ý?????
	InMenu = 1,  // ????? ??????2????
	
	InitMenu = 0xFF,
}BspMenuState;

#define POINT_SITE_MAX 3U

// ???????? ?????????¡ã???
typedef struct
{
	uint8_t curr_partition; // ????????????
	uint8_t last_partition; // ????????
	uint8_t curr_point_site[4]; // ?????????????
	uint8_t last_point_site[4]; // ????????
	
	uint8_t last_show_len[4]; // ?›¥???????¦Ì?¦Ë?? ??????????????????¦Ë??
	
	uint8_t curr_menu_state; // ????????
	uint8_t last_menu_state; // ????¦Â????
}BspKeyCheckNewCtrl_t;

BspKeyCheckNewCtrl_t bkcnc = {
	.curr_partition = 0, 
	.last_partition = 255, 
	.curr_point_site = {0, 0, 0, 0}, 
	.last_point_site = {255, 255, 255, 255},
	.curr_menu_state = InitMenu, 
};

// ?§Û??? ??? ?? ??????????????
typedef struct 
{
	// ???????????? ????????????? ???????
	uint8_t *curr_pack_alarm_len;
	uint8_t *curr_pc_fire_alarm_len;
	uint8_t *curr_pc_fore_alarm_len;
	uint8_t *curr_pc_fault_len;
	uint8_t *curr_pc_outfire_len;
	
	uint32_t curr_sys_time;
	
	uint8_t last_pack_alarm_len;
	uint8_t last_pc_fire_alarm_len;
	uint8_t last_pc_fore_alarm_len;
	uint8_t screen_light_flag;
	uint8_t last_pc_fault_len;
	uint8_t last_pc_outfire_len;
}SwitchInterfaceCtrl;
// ????????? ????????¦Ì???????????
SwitchInterfaceCtrl switch_ui_ctrl;

// ??????????
extern uint8_t screen_show_siren_information;
extern uint8_t shielding_state;
extern uint8_t self_check_state;

// ????1 ?????????
extern uint8_t part_1_start_state;
extern uint8_t part_1_start_delay;
extern uint8_t part_1_spray_state;
extern uint8_t part_1_feedback   ;
extern uint8_t part_1_sound_light;
extern uint8_t part_1_fault_state;


extern QueueHandle_t xMyRs485QueueHandle;

uint8_t self_check_show_content = 0;
uint8_t show_content_delay = 0;

char *banben="NEW_XR5000_V4.3.1.7";

uint8_t linkage_start_key_press_flag = 0;

uint8_t tim_get = 0;

// new
uint8_t main_power_alarm_flag = 0;
uint8_t main_power_beep_ctrl  = 0;

uint8_t back_power_alarm_flag = 0;
// end

// new

typedef enum
{
	Isolata_Output_Beep_1 = 0, //
	Isolata_Output_Beep_2 = 1,
	Isolata_Output_Beep_3 = 2,
	Isolata_Output_Beep_4 = 3,
	
	Feedback_State_Beep_1 = 4,
	Feedback_State_Beep_2 = 5,
	Feedback_State_Beep_3 = 6,
	Feedback_State_Beep_4 = 7,
	Feedback_State_Beep_5 = 8,
	Feedback_State_Beep_6 = 9,
	
	General_Output_Beep_1 = 10,
	General_Output_Beep_2 = 11,
	General_Output_Beep_3 = 12,
	General_Output_Beep_4 = 13,
	General_Output_Beep_5 = 14,
	General_Output_Beep_6 = 15,
	General_Output_Beep_7 = 16,
	General_Output_Beep_8 = 17,

}eGeneralIoBeepBit; // ???IO??????¦Ë???



// [7:4] ????? [3:0] ??????
uint8_t beep_fire_ctrl = 0;
uint8_t beep_fault_ctrl = 0;

uint8_t beep_spray_feedback_ctrl = 0;

uint32_t beep_general_io_ctrl = 0;
// end

uint8_t zhu_state=1,bei_state=1;


// ??????? ?????????????
#define RECORD_SHOW_ZONE 10

typedef enum
{
	RECORD_FAULT = 0,
	RECORD_ALARM = 1,
	RECORD_GASOF = 2,
	RECORD_OTHER = 3,
	
	RECORD_INIT
}RecordShowTypeId;

typedef struct
{
	uint8_t curr_page[4]; // ???? 
	
	uint16_t record_sum[4]; // ?????????
	
	uint8_t force_fresh_flag; // ?????¡À??
	
	uint8_t curr_show_type; // ??????????
}BspScreenReadRecord_t;

BspScreenReadRecord_t bsrr = {
	.curr_page = {0x01, 0x01, 0x01, 0x01},
	.force_fresh_flag = 0,
	.curr_show_type = RECORD_INIT,
};
// ?????????
FlashReadCache_t read_data[5];
//

uint8_t IG3302_DX=0,IG1102_DX=0,XR803_3301ID=0,XR803_RST=0,IG3301ID=0,IG3301_PCAK=0;

uint8_t UART_zhongduan=0,zhongduanz_YS=0,cang_reset=0;

uint8_t CU_zx_buf[30] = {0};

uint8_t PACK_zx_buf[30][PACK_NUM_BACKUP];
int16_t PACK_wendu_buf[30][PACK_NUM_BACKUP];
uint8_t PACK_WDZT_buf[30][PACK_NUM_BACKUP];
uint8_t PACK_YWZT_buf[30][PACK_NUM_BACKUP];
uint8_t PACK_COZT_buf[30][PACK_NUM_BACKUP];
uint8_t PACK_CH4ZT_buf[30][PACK_NUM_BACKUP];

uint8_t Cang_WDZT_buf[30] = {0};
uint8_t Cang_YWZT_buf[30] = {0};
uint8_t Cang_COZT_buf[30] = {0};
uint8_t Cang_CH4ZT_buf[30] = {0};
uint8_t Cang_VOCZT_buf[30] = {0};
uint8_t Cang_H2ZT_buf[30] = {0};
uint8_t Cang_TCQXH_buf[30] = {0};
uint8_t Cang_CGQQY_buf[30] = {0};

uint8_t Cang_zx_buf[30] = {0};
int16_t Cang_wendu_buf[30] = {0};
uint16_t Cang_H2zhi_buf[30] = {0};
uint16_t Cang_COzhi_buf[30] = {0};

uint8_t BMS_Temp[12]={0,0,0,0,0,0,0,0,0,0,0,0};

// end

uint8_t PCAC_zxwz_buf[12]={0,4,16,28,40,52,64,76,88,100,112,0};//?????????ID
uint8_t PCAC_wdwz_buf[12]={0,9,21,33,45,57,69,81,93,105,117,0};//?????ID
uint8_t PCAC_ywtb_buf[12]={0,12,24,36,48,60,72,84,96,108,120,0};//?????????ID
uint8_t PCAC_cotb_buf[12]={0,11,23,35,47,59,71,83,95,107,119,0};//CO?????ID
uint8_t PCAC_ch4yb_buf[12]={0,13,25,37,49,61,73,85,97,109,121,0};//CH4?????ID

uint8_t cang_zxwz_buf[8]={0,16,20,36,52,72,88,0};//?????????ID

uint8_t cang_wdwz_buf[8]={0,10,17,41,49,77,111,0};//?????ID
//uint8_t cang_ywtb_buf[8]={0,12,19,35,51,71,87,0};//?????????ID
//uint8_t cang_cotb_buf[8]={0,13,25,41,61,77,93,0};//CO?????ID
//uint8_t cang_ch4yb_buf[8]={0,14,26,42,62,78,94,0};//CH4?????ID
//uint8_t cang_vocyb_buf[8]={0,15,27,43,63,79,95,0};//VOC?????ID

uint8_t cang_cowb_buf[8]={0,117,25,122,61,89,138,0};//CO?????ID
uint8_t cang_h2wb_buf[8]={0,118,26,124,62,90,139,0};//H2?????ID
uint8_t cang_ch4wb_buf[8]={0,12,33,127,69,93,142,0};//CH4?????ID
uint8_t cang_ywwb_buf[8]={0,120,35,129,71,95,144,0};//YW?????ID
uint8_t cang_vocwb_buf[8]={0,15,34,128,70,94,143,0};//VOC?????ID

uint8_t cang_XH_buf[8]={0,149,150,151,152,153,154,0};//?????ID


uint8_t kaijiyanshi=0;

uint8_t DX_cangjiyibuf[30];

uint8_t BJ_cangjiyibuf_wd[30];
uint8_t BJ_cangjiyibuf_yw[30];
uint8_t BJ_cangjiyibuf_co[30];
uint8_t BJ_cangjiyibuf_ch4[30];
uint8_t BJ_cangjiyibuf_voc[30];
uint8_t BJ_cangjiyibuf_h2[30];

uint8_t BJ_packjiyibuf_wd[30][PACK_NUM_BACKUP];
uint8_t BJ_packjiyibuf_yw[30][PACK_NUM_BACKUP];//2024-03-09???????????????????????????????
uint8_t BJ_packjiyibuf_co[30][PACK_NUM_BACKUP];//2024-03-09???????????????????????????????
uint8_t BJ_packjiyibuf_ch4[30][PACK_NUM_BACKUP];//2024-03-09???????????????????????????????
// XR5000_LOOP3_CHANGE_20260726: HMI-layer memories for loop 3 one-shot records.
static uint8_t rs485_detect_disconnect_memory[RS485_DETECT_MAX_DEVICES] = {0};
static uint8_t rs485_detect_alarm_memory[RS485_DETECT_MAX_DEVICES][RS485_SENSOR_COUNT] = {0};
static uint8_t rs485_detect_pas_memory[RS485_DETECT_MAX_DEVICES] = {0};
#define RS485_LOOP3_FAULT_OFFLINE        0U
#define RS485_LOOP3_FAULT_TEMPERATURE    1U
#define RS485_LOOP3_FAULT_SMOKE          2U
#define RS485_LOOP3_FAULT_CO             3U
#define RS485_LOOP3_FAULT_H2             4U
#define RS485_LOOP3_FAULT_VOC            5U
#define RS485_LOOP3_FAULT_CH4            6U
#define RS485_LOOP3_FAULT_SMOKE_SENSOR    7U
#define LOOP1_FAULT_OFFLINE               10U
#define LOOP1_FAULT_TEMPERATURE           11U
#define LOOP1_FAULT_SMOKE_POLLUTION       12U
#define LOOP1_FAULT_SMOKE_SENSOR          13U
static uint8_t loop1_raw_state_memory[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0}; /* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: edge-driven state memory. */
static uint8_t mbus2_disconnect_memory[MBUS_CONTROL_MAX_DEVICES] = {0};
static uint8_t mbus2_hand_alarm_memory[MBUS_CONTROL_MAX_DEVICES] = {0};

uint8_t pack_bianhaobuf[30];
uint8_t mhqdbiaozhi = 0;
uint16_t baojingjishi=0;

uint8_t qingchujilu=0;
uint8_t BMS_BJ[12]={0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t zhu_min;

// ?????????
uint8_t cluster_solenoid_valve_start_state = 0;

static uint8_t screen_fresh_num = 0;

extern uint8_t fire_alarm_state;
extern uint8_t disconnect_state;
extern uint8_t silencers_state;
extern uint8_t start_delay_state ;
extern uint8_t linkage_beep_ctrl;

typedef enum
{
	NONE_GAS = 0,
	Hydrogen_Type = 1,
	Carbon_Type = 2,
}RealTimeGasType;

typedef enum
{
	PACK_CO_ID  = 0,
	CABIN_CO_ID = 1,
	CABIN_HH_ID = 2,
	GAS_ID_SUM
}CombustibleGasId;

typedef struct
{
	int16_t co_max_val;
	DetectorAddrAttribute curr_da;
	
	uint8_t gas_type;
}MaxCombustibleGas_t;

MaxCombustibleGas_t mcg[GAS_ID_SUM] = {0};

uint8_t max_combustible_gas_fresh_flag = 0;
uint8_t gas_concentration_summary_fresh_flag = 1; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */

#define GAS_SUMMARY_TYPE_CO       1U
#define GAS_SUMMARY_TYPE_H2       2U
#define GAS_SUMMARY_LEVEL_NORMAL  0U
#define GAS_SUMMARY_LEVEL_WARNING 1U
#define GAS_SUMMARY_LEVEL_FIRE    2U

typedef struct
{
    uint8_t valid;
    uint8_t loop;
    uint8_t addr;
    uint8_t gas_type;
    uint8_t alarm_level;
    uint16_t value;
} GasConcentrationCandidate;

uint8_t creatNewFaultRecordToCache(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id);

// 2025/11/17 18:10 ???? ??????????????????? ????????? ?????¡¤?? ???????§Ó? ????? xx??¡¤xxID?????

typedef struct
{
	uint8_t poll_circuits_id; // ??¡¤ID
	uint8_t poll_detector_id; // ?????ID
	
	// ????????????????? ????????????????????? ??????ID
	uint8_t last_circuits_id; // ??¦Ë?¡¤ID
	uint8_t last_detector_id; // ????????ID
	
	uint8_t poll_temper_value; // ???????
	uint8_t poll_smokes_state; // ?????????
	
	uint16_t poll_carbon_value; // ??????????
	
	uint16_t poll_hydrog_value; // ???????? ???en
	
	uint8_t poll_detect_name;  // ??? ?????????
	
	uint8_t poll_sensor_state; // ???????????
	
	uint8_t key_perss_fresh; // 'n'????? 'p'?????
}PollingShowBase_t; // ????????????

typedef struct
{
	// ????ID????
	uint8_t verb_circuits_id; // ??¡¤ID
	uint8_t verb_detector_id; // ?????ID
	uint8_t last_detector_id; // ????¦Â?????????ID ???????????
	
	uint8_t force_fresh_ctrl; // ????ID???????????
	
	uint8_t verb_temper_value; // ????
	uint8_t last_temper_value; // ????
	
	uint8_t verb_smokes_state; // ??????
	uint8_t last_smokes_state; // ??????
	
	uint16_t verb_carbon_value; // ???????
	uint16_t last_carbon_value; // ???????
	
	uint16_t verb_hydrog_value; // ???????? ???en
	uint16_t lsat_hydrog_value; // ???????? ???en
	
	uint8_t verb_detect_name;  // ??? ?????????
	uint8_t lsat_detect_name;  // ??? ?????????
	
	uint8_t verb_sensor_state; // ???????????
	uint8_t last_sensor_state; // ???????????
	
}InqueryShowBase_t; // ?????????????

typedef struct
{
	PollingShowBase_t poll_show_ctrl;
	
	InqueryShowBase_t verb_show_ctrl;
	
	uint32_t last_fresh_time_ctrl;
	uint32_t curr_fresh_time_ctrl;
	
}PointTypeShowCtrl_t;

typedef struct
{
	PollingShowBase_t poll_show_ctrl;
	
	InqueryShowBase_t verb_show_ctrl;
	
	uint32_t last_fresh_time_ctrl;
	uint32_t curr_fresh_time_ctrl;
	
}CompositeShowCtrl_t;


PointTypeShowCtrl_t ptsc = {0};

// ???????
static void PointTypeDetectorShowApp(PointTypeShowCtrl_t *ptsc_entry);
static void PointTypeDetectorButtonCtrlApp(PointTypeShowCtrl_t *ptsc_entry, uint16 control_id, uint8  state);
static void PointTypeDetectorTextInputCtrlApp(PointTypeShowCtrl_t *ptsc_entry, uint16 control_id, uint8 *str);
static void PointTypeDetectorScreenSwitchShowApp(PointTypeShowCtrl_t *ptsc_entry);


CompositeShowCtrl_t cpsc = {0};
// ???????
static void CompositeDetectorShowApp(CompositeShowCtrl_t *cpsc_entry);
static void CompositeDetectorButtonCtrlApp(CompositeShowCtrl_t *cpsc_entry, uint16 control_id, uint8  state);
static void CompositeDetectorTextInputCtrlApp(CompositeShowCtrl_t *cpsc_entry, uint16 control_id, uint8 *str);
static void CompositeDetectorScreenSwitchShowApp(CompositeShowCtrl_t *cpsc_entry);

// ???????IO??????§Û?????? 2025/11/21 13:49

// ???????????????????
typedef struct
{
	uint8_t serial_port_state; // ???????
	
	uint8_t serial_port_comid; // ?????????ID ??????????????????§Õ???
	
	uint8_t serial_port_send_mode; // 0:????? 1:16????
	uint8_t serial_port_show_offset;
	
	uint8_t serial_port_show_mode; // 0:????? 1:16????    
	
	uint8_t serial_port_send_len; // ???????????§Ý???????????
	
	uint8_t serial_port_send_new_row; // 0:?????????? 1:????????
	
	uint8_t serial_port_send_buff[256]; // ?????????

}SimulationSerialPortAssistant_t;

SimulationSerialPortAssistant_t sspa = {
	.serial_port_state = 0,
	.serial_port_comid = 0xFF,
	.serial_port_send_len = 0,
	.serial_port_show_offset = 0,
	.serial_port_show_mode = 0,
	.serial_port_send_mode = 0,
};

static void SimulationSerialPortFirstFresh(SimulationSerialPortAssistant_t *sspa_entry);
	
static void SimulationSerialPortButtonCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t ctrl_id, uint8_t state);

static void SimulationSerialPortMenuCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t ctrl_id, uint8_t item, uint8_t state);

static void SimulationSerialPortTextCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t control_id, uint8_t *str);

static void SimulationSerialPortScreenShowApp(SimulationSerialPortAssistant_t *sspa_entry);

// ???????????›¥????
void FaultDataInit(PackCabinFaultStorage *pcfs_entry)
{
	memset(pcfs_entry, 0, sizeof(PackCabinFaultStorage) * 224);
	pcfs_fresh_ctrl = 255;    // ????????? ???????????
	pcfs_buttom_point = 0;  // ¦Â??? ??????????
}

// ??????????????
void ForeAlarmDataInit(PackCabinForeWarnStorage *pcfws_entry)
{
	memset(pcfws_entry, 0, sizeof(PackCabinForeWarnStorage));
	pcfws_entry->point_history_len = 255;
}

// ?????????????
void FireAlarmDataInit(PackCabinFireAlarmStorage *pcfas_entry)
{
	memset(pcfas_entry, 0, sizeof(PackCabinFireAlarmStorage));
	pcfas_entry->point_history_len = 255;
}

// ???????????›¥????
void FireExtinguishDataInit(FireExtinguishDeviceActionSave *fedas_entry)
{
	memset(fedas_entry, 0, sizeof(FireExtinguishDeviceActionSave));
	fedas_entry->last_point_len = 255;
}

void PackAndCabinHistoryAlarmInit(PackAlarmStorage *pas_entry)
{
	memset(pas_entry, 0, sizeof(PackAlarmStorage));
	pas_pointer = 0;
	last_pas_len = 0;
	pas_fresh_point = 0;// ????????????
}

static void PointTypeDetectorAllStateInit(void);

static uint8_t getPointDetectorSetUpCount(void);
static uint8_t getPointDetectorSetUpLive(void);
static uint8_t getPointDetectorFaultCount(void);
static uint8_t getPointDetectorAlarmCount(void);

void ClearDetectorHistoryData(void)
{
	// ??????????????
	memset(BJ_packjiyibuf_wd,  0, 30*PACK_NUM_BACKUP);
	memset(BJ_packjiyibuf_yw,  0, 30*PACK_NUM_BACKUP);
	memset(BJ_packjiyibuf_co,  0, 30*PACK_NUM_BACKUP);
	memset(BJ_packjiyibuf_ch4, 0, 30*PACK_NUM_BACKUP);
	memset(rs485_detect_disconnect_memory, 0, sizeof(rs485_detect_disconnect_memory));
	memset(rs485_detect_alarm_memory, 0, sizeof(rs485_detect_alarm_memory));
	memset(rs485_detect_pas_memory, 0, sizeof(rs485_detect_pas_memory));
	memset(mbus2_disconnect_memory, 0, sizeof(mbus2_disconnect_memory));
	
	memset(CU_zx_buf, 0, sizeof(CU_zx_buf));
	
	memset(PACK_zx_buf, 0, 30*PACK_NUM_BACKUP);
	memset(PACK_wendu_buf, 0, 30*PACK_NUM_BACKUP);
	memset(PACK_WDZT_buf, 0, 30*PACK_NUM_BACKUP);
	memset(PACK_YWZT_buf, 0, 30*PACK_NUM_BACKUP);
	memset(PACK_COZT_buf, 0, 30*PACK_NUM_BACKUP);
	memset(PACK_CH4ZT_buf, 0, 30*PACK_NUM_BACKUP);
	
	// ?????????????
	memset(Cang_WDZT_buf, 0, 30);
	memset(Cang_YWZT_buf, 0, 30);
	memset(Cang_COZT_buf, 0, 30);
	memset(Cang_CH4ZT_buf, 0, 30);
	memset(Cang_VOCZT_buf, 0, 30);
	memset(Cang_H2ZT_buf, 0, 30);
	memset(Cang_TCQXH_buf, 0, 30);
	memset(Cang_CGQQY_buf, 0, 30);
	
	memset(Cang_zx_buf, 0, 30);
	
	memset(Cang_wendu_buf, 0, 30);
	memset(Cang_H2zhi_buf, 0, 30);
	memset(Cang_COzhi_buf, 0, 30);
	
	// ?????????
	memset(BJ_cangjiyibuf_wd, 0, 30);
	memset(BJ_cangjiyibuf_yw, 0, 30);
	memset(BJ_cangjiyibuf_co, 0, 30);
	memset(BJ_cangjiyibuf_ch4, 0, 30);
	memset(BJ_cangjiyibuf_voc, 0, 30);
	memset(BJ_cangjiyibuf_h2, 0, 30);
	
	// ???????????
	cluster_solenoid_valve_start_state = 0;
	
	// ??????????????
	PointTypeDetectorAllStateInit();
}
	
void PowerStateInit(void)
{
	main_power_alarm_flag = 0;
	main_power_beep_ctrl  = 0;
	back_power_alarm_flag = 0;
}

typedef struct
{
	uint8_t curr_num;
	uint8_t last_num;
}DetectorSum;

DetectorSum ds = {
	.curr_num = 0,
	.last_num = 255
}; // ??????õô?????0

void ScreenFreshInhibitionInit(void)
{
	alarm_number = 0; // ?????????????
	last_alarm_num = 255; // ?????????????????????
	last_online_detector_num = 255; // ??????????????????????????
	last_disconnect_detector_num = 255;	 // ??????????????????????????
	ds.last_num = 255;
	ds.curr_num = 0;
}

void BspBeepStateClear(void)
{
	beep_fire_ctrl = 0;
	beep_fault_ctrl = 0;
	main_power_beep_ctrl = 0;
	linkage_beep_ctrl = 0;
	
	beep_spray_feedback_ctrl = 0;
	beep_general_io_ctrl = 0; // 
}
// ??¦Ë?????????¦Ë??
void BspScreenArrowSite(BspKeyCheckNewCtrl_t *bkcnc_entry)
{
	bkcnc_entry->curr_menu_state = InitMenu;
	
	bkcnc_entry->curr_point_site[0] = 0;
	bkcnc_entry->curr_point_site[1] = 0;
	bkcnc_entry->curr_point_site[2] = 0;
	bkcnc_entry->curr_point_site[3] = 0;
}
// 
void BspCmdProcessInit(void)
{
	FaultDataInit(pcfs); // ?????????
	ForeAlarmDataInit(&pcfws); // ?????????
	FireAlarmDataInit(&pcfas); // ????????
	FireExtinguishDataInit(&fedas); // ?????????????
	PackAndCabinHistoryAlarmInit(pas);
	ClearDetectorHistoryData(); // ??????????????
	ScreenFreshInhibitionInit(); // ???????????
	
	// ?????? ?????????2?????? ?????¦Ë??????¦Ï??????????
	clearHandPaperState();
	cleareedBack1State();
	cleareedBack2State();
}

uint8_t getCurrentSystemRunState(void)
{
	uint8_t temp_state = 0;
	if(pas_pointer != 0)
	{
		temp_state = 2; // ??
	}
	else if(pcfws.self_bottom_point != 0 || pcfas.self_bottom_point != 0)
	{
		temp_state = 1;
	}
	return temp_state;
}

uint8_t getCurrentSystemFaultState(void)
{
	return pcfs_buttom_point ? 1 : 0;
}

uint8_t start_stop_key_state = 0;

uint8_t getCurrentStartStopKeyState(void)
{
	return start_stop_key_state;
}

void CurrentStartStopKeyStateInit(void)
{
	start_stop_key_state = 0;
}

uint8_t outfire_spray_state = 0;

uint8_t getOutFireSprayState(void)
{
	return outfire_spray_state;
}

void OutFireSprayStateInit(void)
{
	outfire_spray_state = 0;
}


uint8_t getCurrentMainBackupPowerState(void)
{
	uint8_t temp_power_state = 1;
	if(zhu_state == 1 && bei_state != short_circuit && bei_state != open_circuit)
	{
		temp_power_state = 1;
	}
	else if(zhu_state == 1 && (bei_state == short_circuit || bei_state == open_circuit))
	{
		temp_power_state = 3;
	}
	else
	{
		temp_power_state = 2;
	}
	return temp_power_state;
}


// new
void StorageCabinForeWarn(PackCabinForeWarnStorage *pcfws_entry, uint8_t cabin_id, uint8_t alarm_type)
{
	
}

void StoragePackCabinForeWarn(PackCabinForeWarnStorage *pcfws_entry, uint8_t cluster_id, uint8_t pack_id, uint8_t alarm_type);

// ??????????
void DeletPackCabinForeWarn(PackCabinForeWarnStorage *pcfws_entry, uint8_t cluster_id, uint8_t pack_id, uint8_t alarm_type);

void StorageCabinFireAlarm(PackCabinFireAlarmStorage *pcfas_entry, 
	uint8_t cabin_id, 
	uint8_t alarm_type);

void StoragePackFireAlarm(PackCabinFireAlarmStorage *pcfas_entry, 
	uint8_t cluster_id, 
	uint8_t pack_id, 
	uint8_t alarm_type);

static uint8_t getShieldDetectorSum(uint8_t pack_shield[][33], uint8_t cabin_shield[]);

typedef enum 
{
	FaultRelayId = 0,
	ForeWarmRelayId,
	FireAlarmRelayId,
	
	RelayCtrlSum
}RelayCtrlSerial;
RelayCtrlStateRecord rcsr[RelayCtrlSum] = {
	{.curr_relay_state = JDQ_OFF, .call_back_fun = FaultRelayCtrl},
	{.curr_relay_state = JDQ_OFF, .call_back_fun = ForeWarmRelayCtrl},
	{.curr_relay_state = JDQ_OFF, .call_back_fun = FireAlarmRelayCtrl},
};	

void BspRelayInit(void)
{
	for(uint8_t i = 0; i < 3; i++)
	{
		rcsr[i].curr_relay_state = JDQ_OFF;
		rcsr[i].call_back_fun(JDQ_OFF);
	}
	DefauleRelayCtrl(JDQ_OFF);
	SoundLightRelayCtrl(JDQ_OFF);
	SirenRelayCtrl(JDQ_OFF);
	OutFire2RelayCtrl(JDQ_OFF);
	OutFire1RelayCtrl(JDQ_OFF);
	CabinSprayRelayCtrl(JDQ_OFF);
}

void StartupLinkageDevice(void)
{
	linkage_start_key_press_flag = 1;
}

// ????? ?????????????????
static void getDetectorSetUpLiveSum(DetectorSum *ds_entry, uint8_t cabin_setup[], uint8_t cluster_setup[]);


//new
//??????????????????????????2026.7.19 ????
static uint8_t getPointDetectorSetUpLive(void);
// ???????????????????????
static uint8_t getPointDetectorAlarmCount(void);
// ???????????????????????
static uint8_t getPointDetectorFaultCount(void);
//end


// ????? ??????????
static uint8_t ClusterPackDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
// 2025/12/10 15:51 ????
static uint8_t ClusterPackDataDeal_Plus(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);

// ????????????
uint8_t findRecoveryDevice(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id);
// ??????????????
void deletRecoveryRecord(uint8_t recovery_index); 

static uint8_t CabinDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);

static void FaultRelayCtrlAppFun(uint8_t disconnect_num);

static void ForeWarmRelayCtrlAppFun(PackCabinForeWarnStorage *pcfws_entry);

static void FireAlarmRelayCtrlAppFun(uint8_t pas_alarm_num);
// ??????§Û??????
static void InternalScreenShowAllFault(uint8_t fresh_page_flag);
// ????????????? ??????
static void InternalScreenShowAllForceWorn(PackCabinForeWarnStorage *pcfws_entry, uint8_t fresh_page_flag);

static void InternalScreenShowAllForceWorn_Plus(PackCabinForeWarnStorage *pcfws_entry, uint8_t fresh_page_flag);
// ??????§Ý???? ????????
static void InternalScreenShowAllFireAlarm(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag);

static void InternalScreenShowAllFireAlarm_Plus(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag);
// ?????¦Ì??????????
static void CreatNewFireExtinguishRecord(
	FireExtinguishDeviceActionSave *fedas_entry, // ??????????
	FireExtinguishDeviceActionSave *copy_fedas,  // ??????????
	uint8_t copy_dedas_offset,
	FireExtinguishDeviceActionType state, 
	uint16_t state_switch_delay             // ???§Ý???? 
);


static void FireExtinguishDevice1HandStart(FireExtinguishDeviceActionSave *fedas_entry);
static void FireExtinguishDevice2HandStart(FireExtinguishDeviceActionSave *fedas_entry);

// ????????????
static void FireExtinguishDeviceStateUpdate(FireExtinguishDeviceActionSave *fedas_entry, PackAlarmStorage *pas_entry);
// ?????¡Â??????????
static void InternalScreenShowFireExtinguisher(FireExtinguishDeviceActionSave *fedas_entry, uint8_t fresh_page_flag);

static void InternalScreenShowClusterData(DetectorDataShowCtrl *ddsc_entry);
// 1??32pack?·Ú PACK?????
static void InternalScreenShowClusterData_32Pack(uint16_t screen_id_entry, DetectorDataShowCtrl_32Pack *ddsc_32p_entry);
// 2025/12/10 17:22 ???
static void InternalScreenShowClusterData_32Pack_Plus(uint16_t screen_id_entry, DetectorDataShowCtrl_32Pack *ddsc_32p_entry);

// ????????? 2025/10/27 11:27???
static void InternalScreenShowCabinDate(CabinDataShowCtrl_t *cabin_dsc_entry);

static void DetectorDataFreshMenuCtrl(DetectorDataShowCtrl *ddsc_entry, uint16_t ctrl_id, uint8_t item, uint8_t state);
// 1??32pack?·Ú ???????????
static void DetectorDataFreshMenuCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t item, uint8_t state);
// 
static void DetectorFreshPageButtonCtrl(DetectorDataShowCtrl *ddsc_entry, uint16_t ctrl_id, uint8_t state);
// 1??32pack?·Ú PACK??????????
static void DetectorFreshPageButtonCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t state);
// 1??32pack?·Ú ???PACK???
static void DetectorMonitorButtonCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t state);
// ????????? 2025/10/27 11:27???
static void CabinFreshPageButtonCtrl(CabinDataShowCtrl_t *cabin_dsc_entry, uint16_t ctrl_id, uint8_t state);

// ????????????????
static void RefreshGasConcentrationSummary(void);

// ??ö™?? ??????????????
static void BspCheckNewKeyPressDeal(BspKeyCheckNewCtrl_t *bkcnc_entry);
// ?§Ý????????? ?????????
static void InternalScreenMainInterfaceCtrl(SwitchInterfaceCtrl *sic_entry);
static void SyncMonitorSwitchSnapshot(void);
// ??????????????Ë®?????
static void InternalScreenRecordShiftButtonCtrl(BspScreenReadRecord_t *bsrr_entry, uint16_t ctrl_id, uint8_t state);
// ?????????????????
static void InternalScreenShowRecord(BspScreenReadRecord_t *bsrr_entry);
// ????????§Ý?????
static void RecordSwitchButtonCtrl(BspScreenReadRecord_t *bsrr_entry, uint16_t ctrl_id, uint8_t state);
// ??????????§Ø? ?›¥
static void PowerManageCtrl(uint8_t main_power_state, uint8_t back_power_state);
// ?????????????
static void HandForceStartAnyCluster(FireExtinguishDeviceActionSave *fedas_entry, uint16_t ctrl_id, uint8_t state);
// 
static void BspAlarmDataSaveApp(FlashReadCtrlId addr_type, FlashSaveType save_type, uint8_t cluster_id, uint8_t pack_or_cabin, uint16_t val);
// 
static void BspFanOnlineJudgeFaultRecord(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
//
static void BspFanStartCrtlApp(uint8_t fan_sta, uint8_t early_aralm_num, uint8_t fire_alarm_num);

// 2025/11/15 11:07 ????????????????????????????????
static uint8_t PointTypeDetectorDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
// XR5000_LOOP3_CHANGE_20260726: Loop 3 uses RS485Detect data with original alarm logic.
static uint8_t RS485DetectDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
static void RS485Loop3ClearCurrentState(uint8_t addr);
static void Loop1ClearCurrentState(uint8_t addr);
static uint8_t MBus2DataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
static void PointTypeDetectorOnlineButtonCtrl(uint16_t ctrl_id, uint8_t state);

//uint8_t license_allow_use_state = 0; // ??????
uint8_t license_allow_use_state = 1; // ??????????????
static uint32_t remain_use_time = 0;
static void LicenseVerificationCtrl(void);

// 2026/01/21 16:53
static void PointTypeDetectorOnlineIconCtrl(uint16_t ctrl_id, uint8_t state, uint8_t icon_num);
// 2026/01/21 17:16
static void PointTypeDetectorOnlineButtonCtrlPlus(uint16_t ctrl_id, uint8_t state);
// 2026/01/21 17:30
static void PointTypeDetectorOnlineTextCtrlPlus(uint16_t ctrl_id, uint8 *entry_str);
// 2026/ 01/21 17:52
static void PointTypeDetectorOnlineStateShowInit(void);

// 2026/01/07 10:54

typedef struct 
{
	uint8_t last_screen_id; // ???????????????
	
	uint8_t warn_fresh_flag;
	
	uint8_t fire_fresh_flag;
}AlarmPinToTopCtrl_t;

AlarmPinToTopCtrl_t sicj = {0, 0};

static void FirstAlarmInformationShowCtrl(
	uint16_t temp_screen_id,
	AlarmPinToTopCtrl_t *sicj_entry, 
	PackCabinForeWarnStorage *pcfws_entry, 
	PackCabinFireAlarmStorage *pcfas_entry
);

// 2025/11/26 08:50 ?????????????
extern void SuspendTask(uint8_t task_id);
extern void ResumeTask(uint8_t task_id);
// end



//2026/7/22????????
static uint8_t g_screen69_page = 0;
static uint8_t g_screen69_force_redraw = 0;
static uint8_t g_screen69_transition_pending = 0;
static uint8_t screen69_circuit = 1; /* XR5000_SCREEN69_NAVIGATION_FIX_20260729: fixed circuit snapshot for one detail session. */
/* ????????¡¤???????õô????§Ò?????????????? */
static uint8_t GetCircuitOnlineList(uint8_t circuit, uint8_t *list, uint8_t max)
{
    uint8_t count = 0;
    uint8_t i;

    switch (circuit)
    {
        case 1:
            for (i = 1; i <= MIXTURE_DEVICE_MAX_ADDR && count < max; i++)
            {
                if (getPointTypeMixtureSettingOnlieState(i) && getPointTypeMixtureDetectName(i) != 0U && getPointTypeMixtureDisconnectCount(i) < MIXTURE_DEVICE_DISCONNECT_SUM)
                    list[count++] = i;
            }
            break;
        case 2:
            for (i = 1; i < MBUS_CONTROL_MAX_DEVICES && count < max; i++)
            {
                if (MBusCtrl_GetOnline(i) && MBusCtrl_IsIdentified(i) && !MBusCtrl_IsDisconnected(i))
                    list[count++] = i;
            }
            break;
        case 3:
            for (i = 1; i < RS485_DETECT_MAX_DEVICES && count < max; i++)
            {
                if (RS485Detect_IsOnline(i) && RS485Detect_GetType(i) != RS485_DETECT_TYPE_UNKNOWN)
                    list[count++] = i;
            }
            break;
    }
    return count;
}

/* ????????????????????????buf */
static void FormatDetectorText(uint8_t circuit, uint8_t addr, uint8_t *buf)
{
    switch (circuit)
    {
	case 1:
	{
		uint8_t sensor_bits = getPointTypeMixtureDetectType(addr);
		if (sensor_bits & 0x20) /* ???????????? */
		{
			uint8_t val = getPointTypeMixtureReceiveData(PointTypeData_Temper, addr);
			uint8_t mem = getPointTypeMixtureDetectTempertureMemory(addr);
			sprintf((char *)buf, "??%d??¡¤  ????????%d??  ??????%d??  %s",
					circuit, addr, val, mem ? "????" : "????");
		}
		else if (sensor_bits & 0x01) /* ????????????? */
		{
			uint8_t mem = getPointTypeMixtureDetectSmokeMemory(addr);
			sprintf((char *)buf, "??%d??¡¤  ?????????%d??  ????????%s",
					circuit, addr, mem ? "????" : "????");
		}
		else
		{
			sprintf((char *)buf, "??%d??¡¤  ?????%d??  ??????¦Ä????", circuit, addr);
		}
		break;
	}
	case 3:
	{
		uint16_t enable = RS485Detect_GetSensorEnable(addr);
		char *p = (char *)buf;

		p += sprintf(p, "??%d??¡¤  ?????????%d??  ", circuit, addr);

		if (enable & (1 << 5))
		{
			int16_t temp = RS485Detect_GetTemperature(addr);
			p += sprintf(p, "????%d??  ", temp);
		}
		if (enable & (1 << 0))
		{
			uint8_t smoke = RS485Detect_GetSensorState(addr, RS485_SENSOR_SMOKE);
			p += sprintf(p, "?????%s  ", RS485Detect_IsFaultState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "????" : (RS485Detect_IsAlarmState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "????" : "????"));
		}
		if (enable & (1 << 4))
		{
			uint16_t co = RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO);
			p += sprintf(p, "CO??%dppm  ", co);
		}
		if (enable & (1 << 2))
		{
			uint16_t h2 = RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2);
			p += sprintf(p, "H2??%dppm  ", h2);
		}
		if (enable & (1 << 3))
		{
			uint16_t voc = RS485Detect_GetSensorValue(addr, RS485_SENSOR_VOC);
			p += sprintf(p, "VOC??%dppm  ", voc);
		}
		if (enable & (1 << 1))
		{
			uint16_t ch4 = RS485Detect_GetSensorValue(addr, RS485_SENSOR_CH4);
			p += sprintf(p, "CH4??%dppm  ", ch4);
		}
		if (enable & (1 << 6))
		{
			uint16_t pressure = RS485Detect_GetSensorValue(addr, RS485_SENSOR_PRESSURE);
			p += sprintf(p, "?????%dhPa  ", pressure);
		}

		break;
	}
	}
}


// XR5000_LOOP3_CHANGE_20260726: Helpers keep loop 3 display separate from legacy cluster/pack text.
/* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: loop1 screen59 text follows XR800B/XR800C state semantics. */
static const char *Loop1StateName(uint8_t type, uint8_t state)
{
    if(type == 6U)
    {
        if(state == 1U) return "\xCE\xC2\xB6\xC8\xD4\xA4\xBE\xAF";
        if(state == 2U) return "\xCE\xC2\xB6\xC8\xBB\xF0\xBE\xAF";
        if(state == 3U) return "\xCE\xC2\xB6\xC8\xB9\xCA\xD5\xCF";
    }
    else if(type == 5U)
    {
        if(state == 1U) return "\xD1\xCC\xCE\xED\xD4\xA4\xBE\xAF";
        if(state == 2U) return "\xD1\xCC\xCE\xED\xBB\xF0\xBE\xAF";
        if(state == 8U) return "\xD1\xCC\xCE\xED\xCE\xDB\xC8\xBE\xB9\xCA\xD5\xCF";
        if(state == 9U) return "\xD1\xCC\xCE\xED\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
    }
    return "\xD5\xFD\xB3\xA3";
}

static void FormatLoop1WarningLine(uint8_t *buf, uint8_t sequence, PackCabinForeWarnStorage *entry, uint8_t index)
{
    sprintf((char *)buf, "%03d %d/%02d/%02d %02d:%02d:%02d \xB5\xDA" "1\xBB\xD8\xC2\xB7 %d\xBA\xC5 %s", sequence,
        entry->atr[index].years, entry->atr[index].months, entry->atr[index].days,
        entry->atr[index].hours, entry->atr[index].minute, entry->atr[index].second,
        entry->da[index].cabin_id, entry->alarm_type[index] == Loop1TempWarning ? "\xCE\xC2\xB6\xC8\xD4\xA4\xBE\xAF" : "\xD1\xCC\xCE\xED\xD4\xA4\xBE\xAF");
}

static void FormatLoop1FireLine(uint8_t *buf, uint8_t sequence, PackCabinFireAlarmStorage *entry, uint8_t index)
{
    sprintf((char *)buf, "%03d %d/%02d/%02d %02d:%02d:%02d \xB5\xDA" "1\xBB\xD8\xC2\xB7 %d\xBA\xC5 %s", sequence,
        entry->atr[index].years, entry->atr[index].months, entry->atr[index].days,
        entry->atr[index].hours, entry->atr[index].minute, entry->atr[index].second,
        entry->da[index].cabin_id, entry->alarm_type[index] == Temperature ? "\xCE\xC2\xB6\xC8\xBB\xF0\xBE\xAF" : "\xD1\xCC\xCE\xED\xBB\xF0\xBE\xAF");
}

static uint8_t FormatLoop1FaultLine(uint8_t *buf, uint8_t sequence, PackCabinFaultStorage *entry, uint8_t index)
{
    const char *name;
    if(entry[index].detector_class != CabinClassID || entry[index].da.cluster_id != 0U || entry[index].fault_type < LOOP1_FAULT_OFFLINE) return 0U;
    switch(entry[index].fault_type)
    {
        case LOOP1_FAULT_TEMPERATURE: name = "\xCE\xC2\xB6\xC8\xB9\xCA\xD5\xCF"; break;
        case LOOP1_FAULT_SMOKE_POLLUTION: name = "\xD1\xCC\xCE\xED\xCE\xDB\xC8\xBE\xB9\xCA\xD5\xCF"; break;
        case LOOP1_FAULT_SMOKE_SENSOR: name = "\xD1\xCC\xCE\xED\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF"; break;
        default: name = "\xB5\xF4\xCF\xDF"; break;
    }
    sprintf((char *)buf, "%03d %d/%02d/%02d %02d:%02d:%02d \xB5\xDA" "1\xBB\xD8\xC2\xB7 %d\xBA\xC5 %s", sequence,
        entry[index].atr.years, entry[index].atr.months, entry[index].atr.days,
        entry[index].atr.hours, entry[index].atr.minute, entry[index].atr.second,
        entry[index].da.cabin_id, name);
    return 1U;
}
static uint8_t FormatRS485DetectFlashDeviceName(uint8_t cluster_id, uint8_t addr, uint8_t *buf)
{
	if(cluster_id != RS485_DETECT_FLASH_ID)
	{
		return 0;
	}
	sprintf((char *)buf, "??3??¡¤ %d??", addr);
	return 1;
}

static const char *RS485DetectAlarmName(uint8_t alarm_type)
{
	/* XR5000_LOOP3_STATUS_FINALIZE_20260730: names match warning/fire classification. */
	switch(alarm_type)
	{
		case Temperature:
			return "\xCE\xC2\xB6\xC8\xBB\xF0\xBE\xAF";
		case Smoke:
			return "\xD1\xCC\xCE\xED\xB1\xA8\xBE\xAF";
		case Carbon:
			return "CO\xD4\xA4\xBE\xAF";
		case Hydrogen:
			return "H2\xD4\xA4\xBE\xAF";
		case Voc:
			return "VOC\xD4\xA4\xBE\xAF";
		case Loop3TempWarning:
			return "\xCE\xC2\xB6\xC8\xD4\xA4\xBE\xAF";
		case Loop3CarbonFire:
			return "CO\xBB\xF0\xBE\xAF";
		case Loop3HydrogenFire:
			return "H2\xBB\xF0\xBE\xAF";
		default:
			return "\xB1\xA8\xBE\xAF";
	}
}
static void FormatRS485DetectForeWarnLine(uint8_t *buf, uint8_t sequence, PackCabinForeWarnStorage *pcfws_entry, uint8_t data_index)
{
	sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? %s", sequence,
		pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
		pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
		pcfws_entry->da[data_index].pack_id, RS485DetectAlarmName(pcfws_entry->alarm_type[data_index]));
}

static void FormatRS485DetectFireAlarmLine(uint8_t *buf, uint8_t sequence, PackCabinFireAlarmStorage *pcfas_entry, uint8_t data_index)
{
	sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? %s", sequence,
		pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
		pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
		pcfas_entry->da[data_index].pack_id, RS485DetectAlarmName(pcfas_entry->alarm_type[data_index]));
}

static uint8_t FormatRS485DetectFaultLine(uint8_t *buf, uint8_t sequence, PackCabinFaultStorage *pcfs_entry, uint8_t data_index)
{
	const char *fault_name;
	if(pcfs_entry[data_index].da.cluster_id != RS485_DETECT_FLASH_ID)
	{
		return 0;
	}
	switch(pcfs_entry[data_index].fault_type)
	{
		case RS485_LOOP3_FAULT_TEMPERATURE: fault_name = "\xCE\xC2\xB6\xC8\xB9\xCA\xD5\xCF"; break;
		case RS485_LOOP3_FAULT_SMOKE: fault_name = "\xD1\xCC\xCE\xED\xCE\xDB\xC8\xBE\xB9\xCA\xD5\xCF"; break;
		case RS485_LOOP3_FAULT_CO: fault_name = "CO??????????"; break;
		case RS485_LOOP3_FAULT_H2: fault_name = "H2??????????"; break;
		case RS485_LOOP3_FAULT_VOC: fault_name = "VOC??????????"; break;
        case RS485_LOOP3_FAULT_CH4: fault_name = "CH4??????????"; break;
        case RS485_LOOP3_FAULT_SMOKE_SENSOR: fault_name = "?????????????"; break;
		default: fault_name = "\xB5\xF4\xCF\xDF"; break;
	}
	sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d \xB5\xDA\x33\xBB\xD8\xC2\xB7 %d\xBA\xC5 %s", sequence,
		pcfs_entry[data_index].atr.years, pcfs_entry[data_index].atr.months, pcfs_entry[data_index].atr.days,
		pcfs_entry[data_index].atr.hours, pcfs_entry[data_index].atr.minute, pcfs_entry[data_index].atr.second,
		pcfs_entry[data_index].da.pack_id, fault_name);
	return 1;
}
static const char* GetMBusDeviceChineseName(uint8_t addr)
{
	uint8_t type = MBusCtrl_GetDeviceType(addr);
	switch (type)
	{
		case MBUS_CONTROL_DEV_SGBJQ:  return "????????";
		case MBUS_CONTROL_DEV_XR2200: return "?????????";
		case MBUS_CONTROL_DEV_FIRE_DISPLAY: return "?????????";
		default: return "¦Ä??õô";
	}
}

static uint8_t FormatMBus2FaultLine(uint8_t *buf, uint8_t sequence, PackCabinFaultStorage *pcfs_entry, uint8_t data_index)
{
	if (pcfs_entry[data_index].da.cluster_id != MBUS_CONTROL_FLASH_ID)
	{
		return 0;
	}
	const char *name = GetMBusDeviceChineseName(pcfs_entry[data_index].da.pack_id);
	sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??2??¡¤ %s????", sequence,
		pcfs_entry[data_index].atr.years, pcfs_entry[data_index].atr.months, pcfs_entry[data_index].atr.days,
		pcfs_entry[data_index].atr.hours, pcfs_entry[data_index].atr.minute, pcfs_entry[data_index].atr.second,
		name);
	return 1;
}

static uint8_t FormatRS485DetectFireExtinguisherLine(uint8_t *buf, uint8_t sequence, FireExtinguishDeviceActionSave *fedas_entry, uint8_t data_index, uint16_t temp_time)
{
	if(fedas_entry->cluster_id[data_index] != RS485_DETECT_FLASH_ID)
	{
		return 0;
	}

	switch(fedas_entry->fed_action[data_index])
	{
		case FIRE_EXTINGUISH_MODE_JUDGEMENT:
			if(getPart1HandAutoState() == KEY_MANUAL)
			{
				sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ??????????????", sequence,
					fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
					fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
					fedas_entry->pack_id[data_index]);
			}
			else
			{
				buf[0] = 0;
			}
			break;
		case FIRE_EXTINGUISH_START_SPRAY_DELAY:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ??????????????%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FED_START_SPRAY_DELAY_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????1????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_FIRST_SPRAY_START:
		case FIRE_EXTINGUISH_SECOND_SPRAY_START:
		case FIRE_EXTINGUISH_THIRD_SPRAY_START:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????????????%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FIRE_EXTINGUISH_FIRST_SPRAY_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????1????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_SECOND_SPRAY_DELAY:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????2??????????%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FED_SECOND_SPRAY_DELAY_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????2????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_SECOND_SPRAY_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????2????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_THIRD_SPRAY_DELAY:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????3??????????%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FED_THIRD_SPRAY_DELAY_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????3????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_THIRD_SPRAY_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ???????3????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ????????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_STARYUP_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ?????????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_CYLINDEF_1_OPENED:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ??????1???", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_CYLINDEF_2_OPENED:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ??????2???", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_FORCE_STOP:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ??????????????--", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_CAN_RESTART:
		case FIRE_EXTINGUISH_RESTART_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d ??3??¡¤ %d?? ?? ??????????????", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		default:
			buf[0] = 0;
			break;
	}
	return 1;
}
static void FormatScreen69DetectorText(uint8_t circuit, uint8_t addr, uint8_t *buf)
{
	char *p = (char *)buf;
	int remain = 128;
	int n;

	switch (circuit)
	{
		case 1:
		{
            uint8_t type = getPointTypeMixtureDetectName(addr);
            uint8_t state;
            uint16_t value;
            const char *detector_type;

            if(type == 6U)
            {
                detector_type = "\xCE\xC2\xB6\xC8\xCC\xBD\xB2\xE2\xC6\xF7";
                state = getPointTypeMixtureReceiveState(PointTypeData_Temper, addr);
                value = getPointTypeMixtureReceiveData16(PointTypeData_Temper, addr);
                snprintf(p, remain, "%02d-%03d %s \xCE\xC2\xB6\xC8\xD6\xB5:%d\xA1\xE6 %s", circuit, addr, detector_type,
                    (int16_t)value, Loop1StateName(type, state));
            }
            else if(type == 5U)
            {
                detector_type = "\xD1\xCC\xCE\xED\xCC\xBD\xB2\xE2\xC6\xF7";
                state = getPointTypeMixtureReceiveState(PointTypeData_Smoke, addr);
                snprintf(p, remain, "%02d-%03d %s \xD1\xCC\xCE\xED\xD7\xB4\xCC\xAC\xA3\xBA%s", circuit, addr, detector_type,
                    Loop1StateName(type, state));
            }
            else
            {
                snprintf(p, remain, "%02d-%03d \xCC\xBD\xB2\xE2\xC6\xF7 \xC9\xD0\xCE\xB4\xCA\xB6\xB1\xF0", circuit, addr);
            }
			break;
		}
		case 3:
		{
			uint16_t enable = RS485Detect_GetSensorEnable(addr);

			n = snprintf(p, remain, "%02d%03d ?????????", circuit, addr);
			p += n;
			remain -= n;

			if ((enable & (1 << 5)) && remain > 0)
			{
				int16_t temp = RS485Detect_GetTemperature(addr);
				n = snprintf(p, remain, " ???:%d??", temp);
				p += n;
				remain -= n;
			}
			if ((enable & (1 << 0)) && remain > 0)
			{
				uint8_t smoke = RS485Detect_GetSensorState(addr, RS485_SENSOR_SMOKE);
				n = snprintf(p, remain, " ????:%s", RS485Detect_IsFaultState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "????" : (RS485Detect_IsAlarmState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "????" : "????"));
				p += n;
				remain -= n;
			}
			if ((enable & (1 << 4)) && remain > 0)
			{
				uint16_t co = RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO);
				n = snprintf(p, remain, " CO:%dppm", co);
				p += n;
				remain -= n;
			}
			if ((enable & (1 << 2)) && remain > 0)
			{
				uint16_t h2 = RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2);
				n = snprintf(p, remain, " H2:%dppm", h2);
				p += n;
				remain -= n;
			}
			if ((enable & (1 << 3)) && remain > 0)
			{
				uint16_t voc = RS485Detect_GetSensorValue(addr, RS485_SENSOR_VOC);
				n = snprintf(p, remain, " VOC:%dppm", voc);
				p += n;
				remain -= n;
			}
			if ((enable & (1 << 1)) && remain > 0)
			{
				uint16_t ch4 = RS485Detect_GetSensorValue(addr, RS485_SENSOR_CH4);
				n = snprintf(p, remain, " CH4:%dppm", ch4);
				p += n;
				remain -= n;
			}
			if ((enable & (1 << 6)) && remain > 0)
			{
				uint16_t pressure = RS485Detect_GetSensorValue(addr, RS485_SENSOR_PRESSURE);
				snprintf(p, remain, " ???:%dhPa", pressure);
			}
			{
				int cur = (int)(p - (char *)buf);
				int target = 80;
				while (cur < target && remain > 0) { *p++ = ' '; remain--; cur++; }
				*p = '\0';
			}
			break;
		}
		case 2:
		{
			const char *name = GetMBusDeviceChineseName(addr);
			const char *status_str = MBusCtrl_IsAlarmState(addr) ? "????" : "????";
			n = snprintf(p, remain, "%02d-%03d %s %s", circuit, addr, name, status_str);
			p += n;
			remain -= n;
			{
				int cur = (int)(p - (char *)buf);
				int target = 48;
				while (cur < target && remain > 0) { *p++ = ' '; remain--; cur++; }
				*p = '\0';
			}
			break;
		}
		default:
			snprintf((char *)buf, 128, "%02d%03d ?????", circuit, addr);
			break;
	}
}


/*! 
*  \brief  ???????????
*  \param msg ?????????
*  \param size ???????
*/
void ProcessMessage( PCTRL_MSG msg, uint16 size )
{
    uint8 cmd_type = msg->cmd_type;                                                  //???????
    uint8 ctrl_msg = msg->ctrl_msg;                                                  //?????????
    uint8 control_type = msg->control_type;                                          //???????
    uint16 screen_id = PTR2U16(&msg->screen_id);                                     //????ID
    uint16 control_id = PTR2U16(&msg->control_id);                                   //???ID
    uint32 value = PTR2U32(msg->param);                                              //???

    switch(cmd_type)
    {  
    case NOTIFY_TOUCH_PRESS:                                                        //??????????
    case NOTIFY_TOUCH_RELEASE:                                                      //?????????
        NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4)); 
        break;                                                                    
    case NOTIFY_WRITE_FLASH_OK:                                                     //§ÕFLASH???
        NotifyWriteFlash(1);                                                      
        break;                                                                    
    case NOTIFY_WRITE_FLASH_FAILD:                                                  //§ÕFLASH???
        NotifyWriteFlash(0);                                                      
        break;                                                                    
    case NOTIFY_READ_FLASH_OK:                                                      //???FLASH???
        NotifyReadFlash(1,cmd_buffer+2,size-6);                                     //??????¦Â
        break;                                                                    
    case NOTIFY_READ_FLASH_FAILD:                                                   //???FLASH???
        NotifyReadFlash(0,0,0);                                                   
        break;                                                                    
    case NOTIFY_READ_RTC:                                                           //???RTC???
        NotifyReadRTC(cmd_buffer[2],cmd_buffer[3],cmd_buffer[4],cmd_buffer[5],cmd_buffer[6],cmd_buffer[7],cmd_buffer[8]);
        break;
    case NOTIFY_CONTROL:
        {
            if(ctrl_msg==MSG_GET_CURRENT_SCREEN)                                    //????ID?£??
            {
                NotifyScreen(screen_id);                                            //?????§Ý??????????
            }else
						if(ctrl_msg==TUBIAO_shangchuan)                                    //????ID?£??
            {
               TB_sahngchuan(screen_id,control_id,control_type,msg->param[0]);                      //?????????????????
            }else
            {
                switch(control_type)
                {
                case kCtrlButton:                                                   //??????
                    NotifyButton(screen_id,control_id,msg->param[1]);  
										zhu_min=0;//?§Õ????????????????????5???????????? ??????               
                    break;                                                             
                case kCtrlText:                                                     //??????
                    NotifyText(screen_id,control_id,msg->param);                       
                    break;                                                             
                case kCtrlProgress:                                                 //?????????
                    NotifyProgress(screen_id,control_id,value);                        
                    break;                                                             
                case kCtrlSlider:                                                   //?????????
                    NotifySlider(screen_id,control_id,value);                          
                    break;                                                             
                case kCtrlMeter:                                                    //?????
                    NotifyMeter(screen_id,control_id,value);                           
                    break;                                                             
                case kCtrlMenu:                                                     //??????
                    NotifyMenu(screen_id,control_id,msg->param[0],msg->param[1]);      
                    break;                                                              
                case kCtrlSelector:                                                 //?????
                    NotifySelector(screen_id,control_id,msg->param[0]);                
                    break;                                                              
                case kCtrlRTC:                                                      //????????
                    NotifyTimer(screen_id,control_id);
                    break;
                default:
                    break;
                }
            } 
            break;  
        } 
    case NOTIFY_HandShake:                                                          //??????                                                     
//        NOTIFYHandShake();
        break;
    default:
        break;
    }
}
/*! 
*  \brief  ??????
*/
//void NOTIFYHandShake()
//{
//   SetButtonValue(3,2,1);
//}

typedef struct
{
	uint16_t target_screen;
	uint8_t  switch_flag;
}BspScreenSwitchCtl_t;

BspScreenSwitchCtl_t bsp_screen_switch_ctrl = {
	.target_screen = 0,
	.switch_flag   = 0,
};

#define MONITOR_PAGE_SCREEN_ID                 59U
#define MONITOR_PAGE_AUTH_DEFAULT_RETURN_ID    68U
#define MONITOR_PAGE_LOCKED_DEFAULT_RETURN_ID  1U

static uint16_t monitor_page_return_target = 0U;

static uint16_t MonitorPageDefaultReturnTarget(void)
{
	return (license_allow_use_state == 1) ? MONITOR_PAGE_AUTH_DEFAULT_RETURN_ID : MONITOR_PAGE_LOCKED_DEFAULT_RETURN_ID;
}

static void MonitorPageRememberReturnTarget(uint16_t source_screen)
{
	if(source_screen != 0U && source_screen != MONITOR_PAGE_SCREEN_ID)
	{
		monitor_page_return_target = source_screen;
	}
}

static uint16_t MonitorPageGetReturnTarget(void)
{
	if(monitor_page_return_target == 0U || monitor_page_return_target == MONITOR_PAGE_SCREEN_ID)
	{
		return MonitorPageDefaultReturnTarget();
	}
	return monitor_page_return_target;
}

static void SwitchToMonitorPageFrom(uint16_t source_screen)
{
	MonitorPageRememberReturnTarget(source_screen);
	SwitchCurrentScreenId(MONITOR_PAGE_SCREEN_ID);
	bsp_screen_switch_ctrl.target_screen = MONITOR_PAGE_SCREEN_ID;
	bsp_screen_switch_ctrl.switch_flag = 1;
}

static void SetMonitorPageFrom(uint16_t source_screen)
{
	MonitorPageRememberReturnTarget(source_screen);
	SetScreen(MONITOR_PAGE_SCREEN_ID);
	osDelay(5);
	GetScreen();
}
static void EnterTimeDateSettingWithPassword(void)
{
	setKeyValue(MODIFY_TIME_KEY); /* XR5000_TIME_DATE_ENTRY_REUSE_20260802: share the same password-gated time/date entry flow. */
	SwitchCurrentScreenId(53);
	bsp_screen_switch_ctrl.target_screen = 53;
	bsp_screen_switch_ctrl.switch_flag = 1;
}
const uint8_t pack_online_ctrl_button_id[] = {
	0, 5, 8, 11, 14, 17, 20, 25, 28, 31, 34, 
	37, 40, 50, 53, 56, 59, 62, 65, 68, 71
};

const uint8_t point_type_detect_button_online_ctrl_val_map[] = {
	6, 5, 8, 9, 11, 12, 14, 15,
	17, 18, 20, 21, 25, 28, 31, 34,
	37, 40, 43, 46, 51, 52, 53, 54,
	62, 64, 66, 68, 73, 74, 75, 76,
};

// static uint8_t g_screen69_page = 0;

/*! 
*  \brief  ?????§Ý???
*  \details  ??????????(?????GetScreen)????§Õ????
*  \param screen_id ???????ID
*/
void NotifyScreen(uint16 screen_id)
{
	uint16_t prev_screen_id = current_screen_id; /* XR5000_MONITOR_RETURN_NAV_CHANGE_20260802 */
    //TODO: ??????????
    current_screen_id = screen_id;
    DeviceThreshold_NotifyScreen(screen_id); //??????????§á???????§Ý???????????????ID
	if(screen_id == MONITOR_PAGE_SCREEN_ID)
	{
		if(monitor_page_return_target == 0U)
		{
			MonitorPageRememberReturnTarget(prev_screen_id);
		}
	}
	else if(prev_screen_id == MONITOR_PAGE_SCREEN_ID)
	{
		monitor_page_return_target = 0U;
	}
		// 
		if(bsp_screen_switch_ctrl.switch_flag == 1)
		{
			if(bsp_screen_switch_ctrl.target_screen != current_screen_id)
			{
				SwitchCurrentScreenId(bsp_screen_switch_ctrl.target_screen);
				return; // ???????
			}
			else
			{
				bsp_screen_switch_ctrl.switch_flag = 0;
			}
		}
	
		/* XR5000_CHECK_CHANGE_20260804: entering/leaving screen 72 owns the physical check indicator. */
		if(screen_id == CHECK_SCREEN_ID)
		{
			self_check_state = 1U;
			check_last_activity_tick = osKernelGetTickCount();
			check_values_valid = 0U;
		}
		else
		{
			self_check_state = 0U;
		}
		
    //????????1???????????
    if(screen_id == 1)
    {
			SetTextValue(1, 11, (uint8_t *)"FGS-XR5000.??????????????");//???????????
			
			SetTextInt32(1, 30, SystemSaveInfo.slave_addr485_Station,0,1); // 30??????
			
			SetTextInt32(1, 22, SystemSaveInfo.slave_addr485_EMS,0,1); // 22??EMS??
			
			SetTextInt32(1, 25, SystemSaveInfo.slave_addr485_EMS,0,1); // 25??CAN2??ID
			
			SetTextInt32(1, 8, alarm_number,0,1);  //???????????
			home_statistics_force_refresh = 1;
    }
		else if(screen_id == 3)
		{
			SimulationSerialPortFirstFresh(&sspa);
		}
		else if(screen_id == 4)
		{
			if(pack_circuit == 0 || pack_circuit >= 4)
			{
				pack_circuit = 1;
			}
			
			uint8_t temp_buff[8] = {0};
			
			sprintf((char *)temp_buff, "??¡¤%d", pack_circuit);
			SetTextValue(4, 57, temp_buff); // ????¡¤????
			
			for(uint8_t i = 0; i < 32; i++)
			{
				temp_buff[0] = pack_online_buff[pack_circuit][i + 1] ? 1 : 0;
				
				// ?????????
				setkey_Value(4, point_type_detect_button_online_ctrl_val_map[i], temp_buff[0]);
			}
		}
		else if(screen_id == 5)
		{
			PointTypeDetectorOnlineStateShowInit();
		}
		else if(screen_id == 6) // 
		{
			uint8_t temp_buff[32] = {0};
			//??¡¤1
			sprintf((char *)temp_buff, "????????:%d", getPointDetectorSetUpCount());
			SetTextValue(screen_id, 7, temp_buff); 
			sprintf((char *)temp_buff, "?õô????:%d", getPointDetectorSetUpLive());
			SetTextValue(screen_id, 8, temp_buff); 
			sprintf((char *)temp_buff, "?õô????:%d", (getPointDetectorFaultCount() + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP1)));
			SetTextValue(screen_id, 9, temp_buff); 
			sprintf((char *)temp_buff, "?????õô:%d", getPointDetectorAlarmCount());
			SetTextValue(screen_id, 10, temp_buff); 
			sprintf((char *)temp_buff, "?????õô:0");
			SetTextValue(screen_id, 11, temp_buff); 
			//??¡¤2
			{
				uint8_t mbus2_online = MBusCtrl_GetOnlineCount();
				uint8_t mbus2_disconnect = MBusCtrl_GetDisconnectCount();
				sprintf((char *)temp_buff, "?õô????:%d", mbus2_online);
				SetTextValue(screen_id, 13, temp_buff);
				sprintf((char *)temp_buff, "?õô????:%d", MBusCtrl_GetActiveCount());
				SetTextValue(screen_id, 14, temp_buff);
				sprintf((char *)temp_buff, "?õô????:%d", (mbus2_disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP2)));
				SetTextValue(screen_id, 15, temp_buff);
				sprintf((char *)temp_buff, "?????õô:%d", MBusCtrl_GetAlarmCount());
				SetTextValue(screen_id, 16, temp_buff);
				sprintf((char *)temp_buff, "?????õô:0");
				SetTextValue(screen_id, 17, temp_buff);
			}
			//??¡¤3
			uint8_t online = RS485Detect_GetOnlineCount();
			uint8_t disconnect = RS485Detect_GetDisconnectCount();
			sprintf((char *)temp_buff, "????????:%d", online);
			SetTextValue(screen_id, 19, temp_buff);
			sprintf((char *)temp_buff, "?õô????:%d", RS485Detect_GetActiveCount());
			SetTextValue(screen_id, 20, temp_buff);
			sprintf((char *)temp_buff, "?õô????:%d", (disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP3)));
			SetTextValue(screen_id, 21, temp_buff);
			sprintf((char *)temp_buff, "?????õô:%d", RS485Detect_GetAlarmCount());
			SetTextValue(screen_id, 22, temp_buff);
			sprintf((char *)temp_buff, "?????õô:0");
			SetTextValue(screen_id, 24, temp_buff); 
		}
		else if(screen_id == 7) // 
		{
			if(pack_circuit == 0 || pack_circuit >= 4)
			{
				pack_circuit = 1;
			}
			uint8_t temp_buff[8] = {0};
			sprintf((char *)temp_buff, "??¡¤%d", pack_circuit);
			SetTextValue(7, 231, temp_buff); // ????¡¤????
			switch(pack_circuit)
			{
				case 1:
					for (uint8_t i = 1; i < 33; i++)
					{
						if (getPointTypeMixtureDetectOnlineState(i) == 0)
						{
							temp_buff[0] = 0; // ?????¦Ä????
						}
						else if (getPointTypeMixtureDisconnectCount(i) >= MIXTURE_DEVICE_DISCONNECT_SUM)
						{
							temp_buff[0] = 2; // ??????????????
						}
						else
						{
							temp_buff[0] = 1; // ?????????????
						}
						AnimationPlayFrame(7, i, temp_buff[0]);
					}
					break;
				default:
					break;
			}
			// for(uint8_t i = 1; i < 33; i++)
			// {
			// 	temp_buff[0] = pack_online_buff[pack_circuit][i + 1] ? 1 : 0;
			// 	AnimationPlayFrame(7, i, temp_buff[0]);//(????ID,???ID,?ID) 0??1??
			// }
		}
		else if(current_screen_id == 10)
    {
			uint8_t len = 0;
			uint8_t buff[48] = {0};
			if(strlen((char *)SystemSaveInfo.pref_license_store) != 0)
			{
				len = sprintf((char *)buff, "??????????:");
				for(uint8_t i = 0; i < 10; i++)
				{
					buff[len + i] = SystemSaveInfo.pref_license_store[i];
				}
				SetTextValue(10, 9, buff);
			}
			else
			{
				SetTextValue(10, 9, "??????????:??");
			}
			clearTextValue(10, 7);
			
			SetTextValue(10, 17, "?????§¹????:");
			SetTextValue(10, 18, SystemSaveInfo.last_license_store);
			
			SetTextValue(10, 1, "???????????:");
			SetTextValue(10, 10, SystemSaveInfo.curr_license_store);
			
			char slicense_buff[10] = {0};
			for(uint8_t i = 0; i < 6; i++)
			{
				generate_new_license_code((char *)SystemSaveInfo.last_license_store, getGenerationDate(i), slicense_buff);
				SetTextValue(10, 11 + i, (uint8_t *)slicense_buff);
			}
			
			sprintf((char *)buff, "??§¹???:%d/%d/%d %d:%d:%d", SystemSaveInfo.license_year, SystemSaveInfo.license_month, SystemSaveInfo.license_days, 
				SystemSaveInfo.license_hour, SystemSaveInfo.license_minute, SystemSaveInfo.license_second);
			SetTextValue(10, 20, buff);
			
			sprintf((char *)buff, "??????:%d", remain_use_time);
			SetTextValue(10, 21, buff);
		}
		else if(screen_id == 15)
    {
	
		}
		else if(screen_id == 16)
    {
	
		}
		else if(screen_id == 17)
    {
			uint8_t value = 0;
			for(uint8_t i = 1; i < 21; i++)
			{
				value = cu_sxzt[i] ? 1 : 0;
				// ?????????
				setkey_Value(17, pack_online_ctrl_button_id[i], value);
				// ???????????
				SetTextInt32(17, pack_online_ctrl_button_id[i] + 1, cu_tcq_sxzt[i], 0, 1);
			}
		}
		else if(screen_id == 18)
    {
			uint8_t mapping[] = {0, 6, 5, 8, 9, 11, 12, 14, 15, 17, 18, 20, 
				21, 25, 28, 31, 34, 37, 40, 43, 46, 51, 52, 53, 54};
			for(uint8_t i = 1; i < 25; i++)
			{
				setkey_Value(screen_id, mapping[i], cang_sxzt[i] ? 1 : 0);
			}
		}
		else if(screen_id == 19)
    {
			clearTextValue(19,2);//(????ID,???ID??	
			SetTextValue(19,2,"??¦Ë????");
		}
		else if(screen_id == 21)
    {

		}
		else if(screen_id==22)  //                                        
    {
			
		}
		else if(screen_id == 23)
		{
			clearTextValue(23,2);//(????ID,???ID??
		}
		else if(screen_id == 24)
		{
			clearTextValue(24,2);//(????ID,???ID??	
			clearTextValue(24,5);//(????ID,???ID??
			clearTextValue(24,6);//(????ID,???ID??
			clearTextValue(24,7);//(????ID,???ID??
		}
		else if(current_screen_id == 25)
    {
			// ??????????????????ID????
			const uint8_t setkey_controls[] = {5,8,11,14,17,20,25,28,31,34,37,40,50,53,56,59,62,65,68,71};
			const uint8_t clearText_controls[] = {6,9,12,15,18,21,26,29,32,35,38,41,49,52,55,58,61,64,67,70};
			// ???????setkey_Value
			for (int i = 0; i < sizeof(setkey_controls)/sizeof(setkey_controls[0]); i++) {
					setkey_Value(current_screen_id, setkey_controls[i], 0);
			}

			// ??????????
			for (int i = 0; i < sizeof(clearText_controls)/sizeof(clearText_controls[0]); i++) {
					clearTextValue(current_screen_id, clearText_controls[i]);
			}
		}
		else if(screen_id == 26)
    {
			uint8_t mapping[] = {0, 6, 5, 8, 9, 11, 12, 14, 15, 17, 18, 20, 
				21, 25, 28, 31, 34, 37, 40, 43, 46, 51, 52, 53, 54};
			for(uint8_t i = 1; i < 25; i++)
			{
				setkey_Value(screen_id, mapping[i], cang_pbzt[i] ? 1 : 0);
			}
		}
		else if(current_screen_id == 27)
    {
			SetTextInt32(current_screen_id,2,SystemSaveInfo.factory_release_year + 2000,0,1);//??????
			SetTextInt32(current_screen_id,3,SystemSaveInfo.factory_release_month,0,1);//??????
			SetTextInt32(current_screen_id,4,SystemSaveInfo.factory_release_days,0,1);//??????
			SetTextValue(current_screen_id,5,(unsigned char*)banben);
			
			if(strlen((char *)SystemSaveInfo.curr_license_store) != 0)
			{
				uint8_t len = 0;
				uint8_t buff[32] = {0};
				len = sprintf((char *)buff, "??????:");
				for(uint8_t i = 0; i < 10; i++)
				{
					buff[len + i] = SystemSaveInfo.curr_license_store[i];
				}
				SetTextValue(27, 6, buff);
			}
			else
			{
				SetTextValue(27, 6, "??????:??");
			}
			
			if(SystemSaveInfo.license_remain_day == 6666)
			{
				SetTextValue(27, 7, "?????:???????");
			}
			else if(SystemSaveInfo.license_remain_day != 999)
			{
				uint8_t slicense_buff[32] = {0};
				sprintf((char *)slicense_buff, "?????:???%d??", remain_use_time);
				SetTextValue(27, 7, slicense_buff);
			}
			else
			{
				SetTextValue(27, 7, "?????:??????§¹");
			}
			
		}
		else if(screen_id == 41)
		{
			// ??????????????
			clearTextValue(screen_id, 16);
			clearTextValue(screen_id, 17);
			clearTextValue(screen_id, 18);
			clearTextValue(screen_id, 19);
			clearTextValue(screen_id, 20);
			clearTextValue(screen_id, 21);
			// ?????????
			BM8563_Soft_I2C_GetTime(&SystemTime); // ???????? ????????????
			
			SetTextInt32(screen_id,  9, SystemTime.year + 2000, 0, 4);//??????
			SetTextInt32(screen_id, 10, SystemTime.month      , 0, 2);//??????
			SetTextInt32(screen_id, 11, SystemTime.day        , 0, 2);//??????
			
			SetTextInt32(screen_id, 12, SystemTime.hours      , 0, 2);//??????
			SetTextInt32(screen_id, 13, SystemTime.minutes    , 0, 2);//??????
			SetTextInt32(screen_id, 14, SystemTime.seconds    , 0, 2);//??????
		}
		else if(screen_id == 50)
		{
			SetTextInt32(50, 10, SystemSaveInfo.factory_release_year + 2000, 0, 4);//??????
			SetTextInt32(50, 11, SystemSaveInfo.factory_release_month      , 0, 2);//??????
			SetTextInt32(50, 12, SystemSaveInfo.factory_release_days       , 0, 2);//??????
		}
		else if(screen_id == 59)
		{
			for(uint8_t i = 9 ;i<24;i++)
			{
				clearTextValue(59 , i);//(????ID,???ID)
			}
			clearTextValue(59 , 2);//(????ID,???ID)
		}
		else if(screen_id == 66)
		{
			uint8_t temp_value = 0;
			for(uint8_t i = 1; i < 33; i++)
			{
				temp_value = getPointTypeMixtureDetectOnlineState(i) ? 1 : 0;
				// ?????????
				setkey_Value(66, point_type_detect_button_online_ctrl_val_map[i - 1], temp_value);
			}
		}
		else if(screen_id == 67)
		{
			PointTypeDetectorScreenSwitchShowApp(&ptsc);
			CompositeDetectorScreenSwitchShowApp(&cpsc);
		}
		else if(screen_id == 69)
		{
			uint8_t temp_buff[128] = {0}; /* XR5000_SCREEN69_ATOMIC_RENDER_20260729: first list row can require 128 bytes. */
			uint8_t online_list[MIXTURE_DEVICE_MAX_ADDR] = {0};
			uint8_t online_count;
			HmiTxBatchBegin();
			SetScreenUpdateEnable(0);
			sprintf((char *)temp_buff, "?????????? %d ??¡¤", screen69_circuit);
			SetTextValue(current_screen_id, 200, temp_buff);
			sprintf((char *)temp_buff, "?:?????????????????????????");
			SetTextValue(69, 400, temp_buff);
			sprintf((char *)temp_buff, "????");
			SetTextValue(69, 500, temp_buff);
			sprintf((char *)temp_buff, "????");
			SetTextValue(69, 501, temp_buff);
			sprintf((char *)temp_buff, "????");
			SetTextValue(69, 502, temp_buff);
			/* XR5000_SCREEN69_RESIDUAL_FIX_20260729: clear text controls reused by screen 6 summary. */
			for(uint8_t i = 1;i<25;i++)
			{
				clearTextValue(69 , i);//(????ID,???ID)
			}
			online_count = GetCircuitOnlineList(screen69_circuit, online_list, MIXTURE_DEVICE_MAX_ADDR);
			for(uint8_t i = 0; i < online_count && i < 20; i++)
			{
				FormatScreen69DetectorText(screen69_circuit, online_list[i], temp_buff);
				SetTextValue(69, i + 1, temp_buff);
			}
			g_screen69_force_redraw = 1; /* XR5000_SCREEN69_ATOMIC_RENDER_20260729: refresh cache after atomic first draw. */
			SetScreenUpdateEnable(1);
			HmiTxBatchEnd();
			g_screen69_transition_pending = 0;
		}
}


/*! 
*  \brief  ??????????????
*  \param press 1???¡ä???????3?????????
*  \param x x????
*  \param y y????
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y)
{ 
    //TODO: ??????????
}

uint8_t debug_flag = 0;
uint32_t last_time_stamp = -3600000; // ???????????????????
/*! 
*  \brief  ????????
*/ 
/* XR5000_SCREEN69_FLICKER_FIX_20260727: ????69??????ÈÉ???????????????strcmp???????????????? */
typedef struct {
	uint16_t temper_val;
	uint16_t co_val;
	uint16_t h2_val;
	uint16_t voc_val;
	uint16_t ch4_val;
	uint16_t pressure_val;
	uint16_t sensor_enable;
	uint8_t temper_alarm;
	uint8_t smoke_alarm;
	uint8_t active;
	uint8_t addr;
} Screen69CacheEntry;

/* Keep the combustible-gas alarm lamp tied to live gas states, not UI refreshes. */
static uint8_t RefreshCombustibleGasAlarmLed(void)
{
    uint8_t alarm_on = 0U;
    uint8_t index;
    uint8_t addr;

    /* Legacy loop/pack gas warnings are removed from pcfws on recovery. */
    for(index = 0U; index < pcfws.self_bottom_point; index++)
    {
        uint8_t alarm_type = pcfws.alarm_type[index];
        if(alarm_type == Carbon || alarm_type == Hydrogen || alarm_type == Voc || alarm_type == Methane)
        {
            alarm_on = 1U;
            break;
        }
    }

    /* Loop 3 level-1 and level-2 gas states are evaluated directly from the protocol data. */
    for(addr = 1U; alarm_on == 0U && addr < RS485_DETECT_MAX_DEVICES; addr++)
    {
        uint8_t device_type;
        if(RS485Detect_IsOnline(addr) == 0U || RS485Detect_HasSensorData(addr) == 0U)
        {
            continue;
        }
        device_type = RS485Detect_GetType(addr);
        if(RS485Detect_IsAlarmState(device_type, RS485_SENSOR_CO, RS485Detect_GetSensorState(addr, RS485_SENSOR_CO)) != 0U ||
           RS485Detect_IsAlarmState(device_type, RS485_SENSOR_H2, RS485Detect_GetSensorState(addr, RS485_SENSOR_H2)) != 0U ||
           RS485Detect_IsAlarmState(device_type, RS485_SENSOR_VOC, RS485Detect_GetSensorState(addr, RS485_SENSOR_VOC)) != 0U ||
           RS485Detect_IsAlarmState(device_type, RS485_SENSOR_CH4, RS485Detect_GetSensorState(addr, RS485_SENSOR_CH4)) != 0U)
        {
            alarm_on = 1U;
        }
    }

    SpareGasAlarmLedCtrl(alarm_on != 0U ? LED_ON : LED_OFF);
    return alarm_on;
}

typedef struct
{
	uint16_t total;
	uint16_t normal;
	uint16_t fault;
	uint16_t disabled;
} CheckDeviceStats;

static void CheckStatsClassify(CheckDeviceStats *stats, uint8_t disabled, uint8_t fault)
{
	stats->total++;
	if(disabled != 0U) stats->disabled++;
	else if(fault != 0U) stats->fault++;
	else stats->normal++;
}

static void CheckStatsAccumulate(CheckDeviceStats *dst, const CheckDeviceStats *src)
{
	dst->total += src->total;
	dst->normal += src->normal;
	dst->fault += src->fault;
	dst->disabled += src->disabled;
}

static uint8_t CheckLoop3HasFault(uint8_t addr)
{
	uint8_t sensor;
	uint8_t type = RS485Detect_GetType(addr);
	if(RS485Detect_IsDisconnected(addr) != 0U) return 1U;
	for(sensor = 0U; sensor < RS485_SENSOR_COUNT; sensor++)
	{
		if(RS485Detect_IsFaultState(type, sensor, RS485Detect_GetSensorState(addr, sensor)) != 0U) return 1U;
	}
	return 0U;
}

static void CheckScreenCollectStats(CheckDeviceStats stats[5])
{
	uint8_t addr;
	memset(stats, 0, sizeof(CheckDeviceStats) * 5U);
	for(addr = 1U; addr <= MIXTURE_DEVICE_MAX_ADDR; addr++)
	{
		uint8_t type;
		uint8_t fault;
		uint8_t category;
		if(getPointTypeMixtureSettingOnlieState(addr) == 0U) continue;
		type = getPointTypeMixtureDetectName(addr);
		if(type == 6U) category = 1U;
		else if(type == 5U) category = 2U;
		else continue;
		fault = (getPointTypeMixtureDisconnectCount(addr) >= MIXTURE_DEVICE_DISCONNECT_SUM || getPointTypeMixtureStateClass(addr) == 3U) ? 1U : 0U;
		CheckStatsClassify(&stats[category], DeviceDisableIsLoopAddressSet(1U, addr), fault);
	}
	for(addr = 1U; addr < MBUS_CONTROL_MAX_DEVICES; addr++)
	{
		if(MBusCtrl_GetOnline(addr) == 0U) continue;
		CheckStatsClassify(&stats[4], DeviceDisableIsLoopAddressSet(2U, addr), MBusCtrl_IsDisconnected(addr));
	}
	for(addr = 1U; addr < RS485_DETECT_MAX_DEVICES; addr++)
	{
		if(RS485Detect_GetOnline(addr) == 0U) continue;
		CheckStatsClassify(&stats[3], DeviceDisableIsLoopAddressSet(3U, addr), CheckLoop3HasFault(addr));
	}
	/* Loop 4 is reserved and contributes zero until its driver is implemented. */
	CheckStatsAccumulate(&stats[0], &stats[1]);
	CheckStatsAccumulate(&stats[0], &stats[2]);
	CheckStatsAccumulate(&stats[0], &stats[3]);
	CheckStatsAccumulate(&stats[0], &stats[4]);
}

static void CheckScreenRefresh(void)
{
	CheckDeviceStats stats[5];
	uint16_t values[20];
	uint8_t category;
	uint8_t state;
	CheckScreenCollectStats(stats);
	for(category = 0U; category < 5U; category++)
	{
		values[category * 4U] = stats[category].total;
		values[category * 4U + 1U] = stats[category].normal;
		values[category * 4U + 2U] = stats[category].fault;
		values[category * 4U + 3U] = stats[category].disabled;
	}
	for(state = 0U; state < 20U; state++)
	{
		if(check_values_valid == 0U || check_last_values[state] != values[state])
		{
			SetTextInt32(CHECK_SCREEN_ID, state + 1U, values[state], 0, 1);
			check_last_values[state] = values[state];
		}
	}
	check_values_valid = 1U;
}

static void CheckModeExitToHome(void)
{
	self_check_state = 0U;
	check_values_valid = 0U;
	SwitchCurrentScreenId(CHECK_RETURN_SCREEN_ID);
	bsp_screen_switch_ctrl.target_screen = CHECK_RETURN_SCREEN_ID;
	bsp_screen_switch_ctrl.switch_flag = 1U;
}

void UpdateUI(void)
{
	uint8_t pack_disconnect_sum = 0;
	uint8_t cabin_disconnect_sum = 0;
	uint8_t point_type_disconnect_sum = 0;
	uint8_t rs485_detect_disconnect_sum = 0;
	uint8_t mbus2_disconnect_sum = 0;
	uint8_t shield_sum = 0; // ????????
	uint8_t combustible_gas_alarm_active = 0U;
	uint32_t curr_time_stamp = osKernelGetTickCount(); /* system tick */

	/* XR5000_CHECK_FLASH_FIX_20260804: save from the single UI task, never from the key-receive task. */
	if(check_record_pending != 0U)
	{
		check_record_pending = 0U;
		BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_SYS_CHECK, LINKAGE_CLUSTER_ID, SYS_CHECK_Package_ID);
	}

	/* XR5000_CHECK_CHANGE_20260804: refresh screen 72 and enforce non-blocking inactivity exit. */
	if(current_screen_id == CHECK_SCREEN_ID)
	{
		if((uint32_t)(curr_time_stamp - check_last_activity_tick) >= CHECK_TIMEOUT_TICKS)
		{
			CheckModeExitToHome();
			return;
		}
		CheckScreenRefresh();
	}
	// ??????????
	tim_get++;
	if(tim_get == 10)
	{
		if(kaijiyanshi < ONLINE_TIMEOUT)
		{
			kaijiyanshi++;
		}
		if(mimajiyi != 0)
		{
			mimajiyi++;
		}
		multiple_alarm_fresh_flag = 1; // ??¡À??¦Ë???
		fed_fresh_flag = 1; // ?????
		
		max_combustible_gas_fresh_flag = 1;
		gas_concentration_summary_fresh_flag = 1; /* XR5000_GAS_SUMMARY_CHANGE_20260731 */

		tim_get = 0;
	}
	// end

	if(curr_time_stamp - last_time_stamp >= 3600000)
	{
		last_time_stamp = curr_time_stamp;
		if(SystemSaveInfo.license_remain_day != 6666 && SystemSaveInfo.license_remain_day != 999)
		{
			struct tm register_data = { 0 };
			struct tm current_data = { 0 };
			
			time_t register_t1;
			time_t current_t2;
			
			double diff_seconds = 0.0f;
			
			uint32_t total_use_time = 0;
			
			getBM8563TimeToSystemTime(); // ??????RTC???
			
			register_data.tm_year = SystemSaveInfo.license_year + 100;
			register_data.tm_mon  = SystemSaveInfo.license_month - 1;
			register_data.tm_mday = SystemSaveInfo.license_days;
			register_data.tm_hour = SystemSaveInfo.license_hour;
			register_data.tm_min  = SystemSaveInfo.license_minute;
			register_data.tm_sec  = SystemSaveInfo.license_second;

			current_data.tm_year = years + 100;
			current_data.tm_mon  = months - 1;
			current_data.tm_mday = days;
			current_data.tm_hour = hours;
			current_data.tm_min  = minutes;
			current_data.tm_sec  = secs;
			
			register_t1 = mktime(&register_data);
			current_t2  = mktime(&current_data);

			diff_seconds = difftime(current_t2, register_t1);
			
			total_use_time = (int)(fabs(diff_seconds) / (86400)); // 

			if(total_use_time < SystemSaveInfo.license_remain_day)
			{
				license_allow_use_state = 1;
				remain_use_time = SystemSaveInfo.license_remain_day - total_use_time;
			}
			else
			{
//				license_allow_use_state = 0; //???????1?????????????
				remain_use_time = 0;
			}
				
			if(remain_use_time < 4) // 
			{
				uint8_t slicense_buff[64] = {0};
				sprintf((char *)slicense_buff, "????:??????????????????%d??,?????????????????????", remain_use_time);
				SetTextValue(1, 3, slicense_buff);
			}
			else
			{
				clearTextValue(1, 3);
			}
		}
	}
	
	// ????õô?????????????????????Ž¤?
	getDetectorSetUpLiveSum(&ds, cang_sxzt, cu_tcq_sxzt);
		
	screen_fresh_num++;				// ?????????
	if(screen_fresh_num > 60)
	{
		screen_fresh_num = 0;
	}

	if(kaijiyanshi >= ONLINE_TIMEOUT)//???????ONLINE_TIMEOUT??????????????
	{
		// ??????????? ??????????????? ?????Ž¤?
		pack_disconnect_sum = ClusterPackDataDeal_Plus(pcfs, &pcfs_buttom_point); 
		// ?????????? ?????????????? ?›¥??????????? ?????????? ???????????? 2025/07/09
		cabin_disconnect_sum = CabinDataDeal(pcfs, &pcfs_buttom_point); 
		//
		point_type_disconnect_sum = PointTypeDetectorDataDeal(pcfs, &pcfs_buttom_point); // 2025/11/17 10:27 ????????????????
		// XR5000_LOOP3_CHANGE_20260726: Loop 3 realtime fault/alarm bridge.
		rs485_detect_disconnect_sum = RS485DetectDataDeal(pcfs, &pcfs_buttom_point);
		mbus2_disconnect_sum = MBus2DataDeal(pcfs, &pcfs_buttom_point);
		combustible_gas_alarm_active = RefreshCombustibleGasAlarmLed();
		/* ?????????????IG3306??4¡¤????24V?????????2026-08-06 */
		// ?§Ø?????§Ö??? ?????????????????????¡¤????485????¡¤??????§Ø?
		FaultRelayCtrlAppFun(pack_disconnect_sum + cabin_disconnect_sum + point_type_disconnect_sum + rs485_detect_disconnect_sum + mbus2_disconnect_sum);
		// ?§Ø????????? ????????????
		ForeWarmRelayCtrlAppFun(&pcfws);
		// ?§Ø?????§Ý? ??????????
		FireAlarmRelayCtrlAppFun(pas_pointer);
		
		// ????????????	??????01????????
		for(uint8_t sum = 1; sum < PACK_USER_NUM + 1; sum++)
		{
			if(CU_zx_buf[sum] == PackDisconnectCount)
			{
				
			}
		}

		// ????????
//		uint8_t fire_alarm_state;
//		fire_alarm_state = FireAlarmCompoundLogicJudgement(fire_alarm_logic_ctrl, fire_alarm_judge,cabin_detector_state_buff);
//		if(fire_alarm_state == fire_alarm)
//		{
//			SetTextValue(40, 62, "???????");//??¡À???????
//		}
//		else if(fire_alarm_state == normal)
//		{
//			SetTextValue(40, 62, "????????");//??¡À???????
//		}

		// end
			
		alarm_number = pcfas.self_bottom_point + pcfws.self_bottom_point; // ???????????

		/*
		// ?????????? ?? ????????? ??????????????? ????????õô????
		if(pas_pointer != 0) { // ?????????????
			fire_alarm_state = 1; // ??????????
			fire_alarm_flag.cluster_alarm_state = 1;
			
			if (fanr.storage_pas_len != pas_pointer) {
				fanr.storage_pas_len = pas_pointer;
				
				// ??¦Ë???????????? ID ??¦¶?? 0~255??
				uint8_t seen[32] = {0};
				
				for (uint8_t i = 0; i < pas_pointer && fanr.faib_buttom_point < 300; i++) {
					uint8_t id = (pas[i].cabin_id != 0) ? pas[i].cabin_id : pas[i].cluster_id;
					uint8_t idx = id / 8;
					uint8_t bit = id % 8;
						
					if (!(seen[idx] & (1 << bit))) { // ???¦Ä?????
						seen[idx] |= (1 << bit);     // ?????????
						fanr.fire_alarm_id_buff[fanr.faib_buttom_point++] = id;
					}
				}
			}		
		}
		*/
		
		/*
		for(uint8_t i = 0; i < 12; i++)
		{
			if(BMS_Temp[i]>=3 && BMS_BJ[i]==0)
			{
				BMS_BJ[i]=1; // ????????????????
				beiguangkai();//????
				SetControlForeColor(1,4,0xf800);
				SetTextValue(1,4,"BMS??????????1??????¡ê?");//??¡À???????
				SaveSensor(0,32,0,0,0,0,0,0,0); // ???????? BMS???????
				
				// ?????????????
			}
			else if(BMS_Temp[i] < 3 && BMS_BJ[i] == 1)
			{
				BMS_BJ[i] = 0;
			}
		}
		*/
//		// ???????/??????
//		BspFanOnlineJudgeFaultRecord(NULL, NULL);
//		// ?????????? ?????§Þ?????????????

		BspFanStartCrtlApp(fan_state1, combustible_gas_alarm_active, fedas.self_point_len);
		// ????????????
		PowerManageCtrl(zhu_state, bei_state);

		
		// ??????????????????? ????????
		if((screen_show_siren_information&0x0F) == 0x0F && (screen_show_siren_information&0xF0) != 0xF0)
		{
			screen_show_siren_information |= 0xF0; // ?????§Û? 
			// ????????????? ???????? ????FLASH???????????¦Ë??RTC????
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, LINKAGE_PRESS, LINKAGE_CLUSTER_ID, ALARM_ANNUNCIATOR_ID, 0xFFFF);
//			// ????cache??????
//			StoragePackCabinForeWarn(&pcfws, LINKAGE_CLUSTER_ID, ALARM_ANNUNCIATOR_ID, AlarmCtrlKey);
			// ?????????????? 
			StoragePackFireAlarm(&pcfas, LINKAGE_CLUSTER_ID, ALARM_ANNUNCIATOR_ID, AlarmCtrlKey); // ??????????
			
			// ????????
			SoundLightRelayCtrl(JDQ_ON);
			
			SysSirenStartLedCtrl(LED_ON);
			
			silencers_state  = 0;  // ???¦Ì???? ???????????
		}
		
		if(getHandPaperState() == 0x0F) // ?????????
		{
			// ??????????
			setDealHandPaperState();
			// ????FLASH
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, LINKAGE_PRESS, LINKAGE_CLUSTER_ID, HANDPOT_Package_ID, 0xFFFF);
			//
		}
		
		// ?§Ò??????§Ý??????? ??????? 
		InternalScreenMainInterfaceCtrl(&switch_ui_ctrl);
	}
	//????????????????§Ø?RS485Detect_GetAlarmCount();
	uint8_t total_alarm = alarm_number;
	if(last_alarm_num != total_alarm)
	{
		last_alarm_num = total_alarm;
		SetTextInt32(1, 8, total_alarm,0,1);  //???????????
		if(total_alarm != 0)
		{
			beep_fire_ctrl |= 0xF0;  // ?? ?? ????
			silencers_state = 0; // ???¦Ì???? ???????? ?????????¦Ë
			// ????????
			SoundLightRelayCtrl(JDQ_ON);
			
			ForeWarmRelayCtrl(JDQ_ON);
		}
	}
	
	FirstAlarmInformationShowCtrl(current_screen_id, &sicj, &pcfws, &pcfas);

	FireExtinguishDeviceStateUpdate(&fedas, pas); 
	
	//????????
	if(current_screen_id==1)                                              
	{
		if(getControllorSelfCheckState() == 1) // ???????????? //??????????
		{
			// ??????
			SpecialSelfCheckLedCtrl(LED_ON);
			switch(screen_fresh_num)
			{
				case 1:SetTextValue(1,4,"???????.     ");break;   //??¡À???????
				case 2:SetTextValue(1,4,"???????..    ");break;   //??¡À???????
				case 3:SetTextValue(1,4,"???????...   ");break;   //??¡À???????
				case 4:SetTextValue(1,4,"???????....  ");break;   //??¡À???????
				case 5:SetTextValue(1,4,"???????..... ");break;   //??¡À???????
				case 6:SetTextValue(1,4,"???????......");break;   //??¡À???????
			}
			screen_fresh_num++;
			if(screen_fresh_num >= 7)
			{
				screen_fresh_num = 1;
			}
			
			if(show_content_delay < 10) {
				show_content_delay++;
			}
			else if(show_content_delay == 10) 
			{
				if(self_check_show_content == 0) {
					self_check_show_content = 1;
				}else if(self_check_show_content == 3) {
					self_check_show_content = 4;
				}else if(self_check_show_content == 4) {
					self_check_show_content = 5;
				}
				show_content_delay = 0; // ????????????????? 
			}
			
			switch(self_check_show_content)
			{
				case 0: {
					uint16_t temp_flash_read_id = W25QXX_ReadID();
					if(temp_flash_read_id != W25Q512) // ????FLASH ????????
					{
						// ???????????????
						creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_FLASH_FAULT_ID, DISCONNECT);
					}						
					SetTextValue(1, 12, "???›¥???????");//
					break;
				}
				case 1:
					SetTextValue(1, 12, "?????????");//
					break;
				case 2:
					SetTextValue(1, 12, "???????????");//
					break;
				case 3:
					SetTextValue(1, 12, "?????????");//
					SoundLightRelayCtrl(JDQ_ON);
					DefauleRelayCtrl(JDQ_ON);
					break;
				case 4:
					SetTextValue(1, 12, "????????");//
					SoundLightRelayCtrl(JDQ_OFF);
					DefauleRelayCtrl(JDQ_OFF);
					clearTextValue(1 , 12);//(????ID,???ID)
					break;
				default:
					self_check_show_content = 0; // ????????¦Ë
					show_content_delay = 0; // ?????? ???????????
					SpecialSelfCheckLedCtrl(LED_OFF);
					break;
			}
		}
		else 
		{
			if(kaijiyanshi < ONLINE_TIMEOUT)
			{
				switch(screen_fresh_num)
				{
					case 10:SetTextValue(1, 4, "?????????.     ");break;   //??¡À???????
					case 20:SetTextValue(1, 4, "?????????..    ");break;   //??¡À???????
					case 30:SetTextValue(1, 4, "?????????...   ");break;   //??¡À???????
					case 40:SetTextValue(1, 4, "?????????....  ");break;   //??¡À???????
					case 50:SetTextValue(1, 4, "?????????..... ");break;   //??¡À???????
					case 60:SetTextValue(1, 4, "?????????......");break;   //??¡À???????
				}
			}
			else
			{
				switch(screen_fresh_num)
				{
					case 10:SetTextValue(1, 4, "????????????.     ");break;   //??¡À???????
					case 20:SetTextValue(1, 4, "????????????..    ");break;   //??¡À???????
					case 30:SetTextValue(1, 4, "????????????...   ");break;   //??¡À???????
					case 40:SetTextValue(1, 4, "????????????....  ");break;   //??¡À???????
					case 50:SetTextValue(1, 4, "????????????..... ");break;   //??¡À???????
					case 60:SetTextValue(1, 4, "????????????......");break;   //??¡À???????
				}
			}
			//  ?????õô????/????/????????
			uint8_t rs485_online = RS485Detect_GetOnlineCount();
			uint8_t rs485_disconnect = RS485Detect_GetDisconnectCount();
			uint8_t mbus2_online = MBusCtrl_GetOnlineCount();
			uint8_t mbus2_disconnect = MBusCtrl_GetDisconnectCount();

			uint8_t total_devices = ds.curr_num + rs485_online + mbus2_online;
			if (ds.last_num != total_devices || home_statistics_force_refresh)
			{
				ds.last_num = total_devices;
				SetTextInt32(current_screen_id, 5, total_devices, 0, 1);
			}
			uint8_t total_online = (uint8_t)((ds.curr_num - getPointDetectorSetUpCount()) - (pack_disconnect_sum + cabin_disconnect_sum) + getPointDetectorSetUpLive() + RS485Detect_GetActiveCount() + MBusCtrl_GetActiveCount());
			if (last_online_detector_num != total_online || home_statistics_force_refresh)
			{
				last_online_detector_num = total_online;
				SetTextInt32(current_screen_id, 6, total_online, 0, 1);
			}
			uint8_t total_fault = (uint8_t)((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount());
			if (last_disconnect_detector_num != total_fault || home_statistics_force_refresh)
			{
				last_disconnect_detector_num = total_fault;
				SetTextInt32(current_screen_id, 7, total_fault, 0, 1);
			}
			//????AHT20?????????
			// XR5000_AHT20_CHANGE_20260727: display cached AHT20 temperature/humidity on home screen.
			{
				static int16_t last_aht20_temp = 32767;
				static uint16_t last_aht20_humi = 0xFFFF;
				static uint8_t last_aht20_valid = 0xFF;
				uint8_t aht20_valid = AHT20_IsValid();
				uint8_t aht20_buff[24] = {0};

				if(aht20_valid)
				{
					int16_t temp = AHT20_GetTemperature();
					uint16_t humi = AHT20_GetHumidity();
					if(temp != last_aht20_temp || aht20_valid != last_aht20_valid || home_statistics_force_refresh)
					{
						last_aht20_temp = temp;
						sprintf((char *)aht20_buff, "????%d??", temp);
						SetTextValue(current_screen_id, 39, aht20_buff);
					}
					if(humi != last_aht20_humi || aht20_valid != last_aht20_valid || home_statistics_force_refresh)
					{
						last_aht20_humi = humi;
						sprintf((char *)aht20_buff, "????%d%%", humi);
						SetTextValue(current_screen_id, 40, aht20_buff);
					}
				}
				else if(aht20_valid != last_aht20_valid || home_statistics_force_refresh)
				{
					SetTextValue(current_screen_id, 39, "????--??");
					SetTextValue(current_screen_id, 40, "????--%");
				}
				last_aht20_valid = aht20_valid;
			}
			home_statistics_force_refresh = 0;
		}
		
		//??????????????
		zhu_min=0;//?????????????????????????????—¨5????????????
	
		// ????????
		FansStateUpdataUI(current_screen_id, fan_disconnect_count, fan_state1, fan_state2, fan_mode);
		
		OutfirePressureUpdataUI(current_screen_id, outfire1_pressure, outfire2_pressure, fire_alarm_threshold);

		PowerStateUpdataUI(current_screen_id, zhu_state, bei_state);

		// end
	}
	else if(current_screen_id == 3)
	{
		SimulationSerialPortScreenShowApp(&sspa);
	}
	else if (current_screen_id == 6 && !g_screen69_transition_pending)
	/* XR5000_SCREEN59_RESIDUAL_FIX_20260729: page-6 statistics must not follow asynchronous page changes. */
	{
		//??¡¤1
		uint8_t temp_buff[32] = {0};
		sprintf((char *)temp_buff, "?? 1 ??¡¤???????");
		SetTextValue(6, 6, temp_buff);
		sprintf((char *)temp_buff, "????????:%d", getPointDetectorSetUpCount());
		SetTextValue(6, 7, temp_buff);
		sprintf((char *)temp_buff, "?õô????:%d", getPointDetectorSetUpLive());
		SetTextValue(6, 8, temp_buff);
		sprintf((char *)temp_buff, "?õô????:%d", (getPointDetectorFaultCount() + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP1)));
		SetTextValue(6, 9, temp_buff);
		sprintf((char *)temp_buff, "?????õô:%d", getPointDetectorAlarmCount());
		SetTextValue(6, 10, temp_buff);
		sprintf((char *)temp_buff, "?????õô:0"); 
		SetTextValue(6, 11, temp_buff);

		//??¡¤2
		{
			uint8_t mbus2_online = MBusCtrl_GetOnlineCount();
			uint8_t mbus2_disconnect = MBusCtrl_GetDisconnectCount();
			sprintf((char *)temp_buff, "?õô????:%d", mbus2_online);
			SetTextValue(6, 13, temp_buff);
			sprintf((char *)temp_buff, "?õô????:%d", MBusCtrl_GetActiveCount());
			SetTextValue(6, 14, temp_buff);
			sprintf((char *)temp_buff, "?õô????:%d", (mbus2_disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP2)));
			SetTextValue(6, 15, temp_buff);
			sprintf((char *)temp_buff, "?????õô:%d", MBusCtrl_GetAlarmCount());
			SetTextValue(6, 16, temp_buff);
			sprintf((char *)temp_buff, "?????õô:0");
			SetTextValue(6, 17, temp_buff);
		}

		//??¡¤3
		uint8_t online = RS485Detect_GetOnlineCount();
        uint8_t disconnect = RS485Detect_GetDisconnectCount();
        sprintf((char *)temp_buff, "????????:%d", online);
        SetTextValue(6, 19, temp_buff);
        sprintf((char *)temp_buff, "?õô????:%d", RS485Detect_GetActiveCount());
        SetTextValue(6, 20, temp_buff);
        sprintf((char *)temp_buff, "?õô????:%d", (disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP3)));
        SetTextValue(6, 21, temp_buff); 
        sprintf((char *)temp_buff, "?????õô:%d", RS485Detect_GetAlarmCount());
        SetTextValue(6, 22, temp_buff); 
		sprintf((char *)temp_buff, "?????õô:0"); 
		SetTextValue(6, 24, temp_buff);
	}
	else if(current_screen_id == 7)
	{
		uint8_t temp_buff[8] = {0};
		switch (pack_circuit)
		{
		case 1:
			for (uint8_t i = 1; i < 33; i++)
			{
				if (getPointTypeMixtureDetectOnlineState(i) == 0)
				{
					temp_buff[0] = 0; // ?????¦Ä????
				}
				else if (getPointTypeMixtureDisconnectCount(i) >= MIXTURE_DEVICE_DISCONNECT_SUM)
				{
					temp_buff[0] = 2; // ??????????????
				}
				else
				{
					temp_buff[0] = 1; // ?????????????
				}
				AnimationPlayFrame(7, i, temp_buff[0]);
			}
			break;
		default:
			break;
		}
	}
	else if(current_screen_id==20)                                              
	{
//			char* str = "???????????";
//			uint8_t baojingneirong[50];
////			sprintf((char*)baojingneirong,"?õô???? ?õô???:%s ???????:%d",str,shebeizongshu);
//			SetTextValue(52,2,baojingneirong);//??¡À???????
//			
//			sprintf((char*)baojingneirong,"???????? ?õô???:%s ???????:%d",str,zaixianzongshu);
//			SetTextValue(52, 5, baojingneirong);//??¡À???????
//			
//			sprintf((char*)baojingneirong,"???????? ?õô???:%s ???????:%d",str,diaoxianzongshu);
//			SetTextValue(52, 6, baojingneirong);//??¡À???????
//			
//			sprintf((char*)baojingneirong,"???????? ?õô???:%s ???????:%d",str,shield_sum);
//			SetTextValue(52, 7, baojingneirong);//??¡À???????
	}
	if(current_screen_id==52)                                              
	{
		uint8_t baojingneirong[50];
		char* str = "???????????";
		sprintf((char*)baojingneirong,"?õô???? ?õô???:%s ???????:%d",str, ds.curr_num);
		SetTextValue(52,2,baojingneirong);//??¡À???????
		
		sprintf((char*)baojingneirong,"???????? ?õô???:%s ???????:%d",str, ds.curr_num - ( pack_disconnect_sum + cabin_disconnect_sum ));
		SetTextValue(52, 5, baojingneirong);//??¡À???????
		
		sprintf((char*)baojingneirong,"???????? ?õô???:%s ???????:%d",str, ( pack_disconnect_sum + cabin_disconnect_sum ));
		SetTextValue(52, 6, baojingneirong);//??¡À???????
		
		shield_sum = getShieldDetectorSum(pack_pbzt, cang_pbzt);
		sprintf((char*)baojingneirong,"???????? ?õô???:%s ???????:%d",str, shield_sum);
		SetTextValue(52, 7, baojingneirong);//??¡À???????
		
	}
	else if(current_screen_id==24)    //                                          
	{
		if(mmsdSTA!=0)
		{
			mmsdSTA--;
			if(mmsdSTA==0)
			{
				SetScreen(2);	//
				GetScreen();
				current_screen_id=2;
			}
		}
	}
	else if(current_screen_id == 46) 
	{

		if(qingchujilu==1)
		{
			SetTextValue(current_screen_id, 7, "??????????... ??????");
			BspClearFlashData();
			SetTextValue(current_screen_id, 7, "???????");
			qingchujilu=0;
		}
	}
	else if(current_screen_id == 54)
	{
		InternalScreenShowClusterData(&ddsc);
	}
	else if(current_screen_id == 56 || current_screen_id == 57)
	{
		// ???????
		InternalScreenShowRecord(&bsrr);
	}
	else if(current_screen_id == 59)
	{
		// new ??????????
		InternalScreenShowAllFault(getFaultCheckNewKey());
		if(getFaultCheckNewKey() == 1)
		{
			fault_check_new_flag = 0;
		}
		// end
		
		// NEW 
		// 2025/12/09 10:22 ??????????
		if(pcfws.self_bottom_point > Alarm_Show_Zone) // ???????????????????????? 
		{
			if(baojingjishi - pcfws.fresh_time_count >= 5)
			{
				pcfws.fresh_time_count = baojingjishi;
				// ?????? // ????+????????? §³?????? ????????????
				if (fore_alarm_start_index + Alarm_Show_Zone < pcfws.self_bottom_point) 
				{
					fore_alarm_start_index++;
					force_alarm_check_new_flag = 1;
				}
				else
				{
					fore_alarm_start_index = 0;
					force_alarm_check_new_flag = 1;
				}
			}
		}
		// ??????????
		InternalScreenShowAllForceWorn_Plus(&pcfws, getForceAlarmCheckNewKey());
		if(getForceAlarmCheckNewKey() == 1)
			force_alarm_check_new_flag = 0;
		//END
		
		// NEW 
		// ???????????
		// 2025/12/09 10:41 ??????????
		if(pcfas.self_bottom_point > Alarm_Show_Zone)
		{
			if(baojingjishi - pcfas.fresh_time_count >= 5)
			{
				pcfas.fresh_time_count = baojingjishi;
				if (fire_alarm_start_index + Alarm_Show_Zone < pcfas.self_bottom_point) 
				{
					fire_alarm_start_index++;
					fire_alarm_check_new_flag = 1;
				} 
				else
				{
					fire_alarm_start_index = 0;
					fire_alarm_check_new_flag = 1;
				}
			}
			
		}
		// ??????????
		InternalScreenShowAllFireAlarm_Plus(&pcfas, getFireAlarmCheckNewKey());
		if(getFireAlarmCheckNewKey() == 1)
			fire_alarm_check_new_flag = 0;
		//END

		InternalScreenShowFireExtinguisher(&fedas, fed_fresh_flag);
		if(fed_fresh_flag)
		{
			fed_fresh_flag = 0;
		}
		
		// ???????
		BspCheckNewKeyPressDeal(&bkcnc);
		
		RefreshGasConcentrationSummary(); /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
	}		
	else if(current_screen_id == 61 || current_screen_id == 62) // ??2????
	{
		// ????PACK???
//		InternalScreenShowClusterData_32Pack(current_screen_id, &ddsc_32p);
		// ????PACK???
		InternalScreenShowClusterData_32Pack_Plus(current_screen_id, &ddsc_32p);
	}		
	else if(current_screen_id == 64)
	{
		InternalScreenShowCabinDate(&cabin_dsc);
	}
	else if(current_screen_id == 67)
	{
		PointTypeDetectorShowApp(&ptsc);
		
		CompositeDetectorShowApp(&cpsc); // 
	}

	// else if (current_screen_id == 69)
	// {
	// 	static uint8_t last_screen = 0;

	// 	/* ???????69????????? */
	// 	if (last_screen != 69)
	// 	{
	// 		g_screen69_page = 0;
	// 		last_screen = 69;
	// 	}

	// 	uint8_t temp_buff[64] = {0};
	// 	uint8_t online_list[MIXTURE_DEVICE_MAX_ADDR] = {0};
	// 	uint8_t online_count;
	// 	uint8_t total_pages;
	// 	uint8_t start, end;
	// 	uint8_t ctrl_idx, list_idx;

	// 	/* ???200????¡¤???? */
	// 	sprintf((char *)temp_buff, "????????¡¤??%d ??¡¤", pack_circuit);
	// 	SetTextValue(current_screen_id, 200, temp_buff);

	// 	/* ????????õô */
	// 	online_count = GetCircuitOnlineList(pack_circuit, online_list, 64);

	// 	/* ?????????????????? */
	// 	total_pages = (online_count + 19) / 20;
	// 	if (total_pages == 0)
	// 		total_pages = 1;
	// 	if (g_screen69_page >= total_pages)
	// 		g_screen69_page = total_pages - 1;

	// 	/* ??????¦¶ */
	// 	start = g_screen69_page * 20;
	// 	end = start + 20;
	// 	if (end > online_count)
	// 		end = online_count;

	// 	static uint8_t g_screen69_prev[10][128] = {0};

	// 	for (ctrl_idx = 0, list_idx = start; list_idx < end; ctrl_idx++, list_idx++)
	// 	{
	// 		FormatScreen69DetectorText(pack_circuit, online_list[list_idx], temp_buff);
	// 		if (strcmp((char *)temp_buff, (char *)g_screen69_prev[ctrl_idx]) != 0)
	// 		{
	// 			SetTextValue(current_screen_id, ctrl_idx + 1, temp_buff);
	// 			memcpy(g_screen69_prev[ctrl_idx], temp_buff, sizeof(g_screen69_prev[ctrl_idx]));
	// 		}
	// 	}

	// 	/* ????????????½¨?? */
	// 	for (; ctrl_idx < 20; ctrl_idx++)
	// 	{
	// 		if (g_screen69_prev[ctrl_idx][0] != '\0')
	// 		{
	// 			SetTextValue(current_screen_id, ctrl_idx + 1, (uint8_t *)"");
	// 			g_screen69_prev[ctrl_idx][0] = '\0';
	// 		}
	// 	}

	// 	// /* ?????? */??¦Ä???????????????
	// 	// for (ctrl_idx = 0, list_idx = start; list_idx < end; ctrl_idx++, list_idx++)
	// 	// {
	// 	// 	FormatScreen69DetectorText(pack_circuit, online_list[list_idx], temp_buff);
	// 	// 	SetTextValue(current_screen_id, ctrl_idx + 1, temp_buff);
	// 	// }

	// 	/* ???????? */
	// 	for (; ctrl_idx < 20; ctrl_idx++)
	// 	{
	// 		SetTextValue(current_screen_id, ctrl_idx + 1, (uint8_t *)"");
	// 	}
	// }

	else if (current_screen_id == 69)
	{
		static Screen69CacheEntry g_screen69_cache[20] = {0};

		if (g_screen69_force_redraw)
		{
			g_screen69_page = 0;
			g_screen69_force_redraw = 0;
			memset(g_screen69_cache, 0, sizeof(g_screen69_cache)); /* XR5000_SCREEN69_RESIDUAL_FIX_20260729: force redraw after screen entry. */
		}

		uint8_t temp_buff[128] = {0};
		uint8_t online_list[MIXTURE_DEVICE_MAX_ADDR] = {0};
		uint8_t online_count;
		uint8_t total_pages;
		uint8_t start, end;
		uint8_t ctrl_idx, list_idx;

		
		online_count = GetCircuitOnlineList(screen69_circuit, online_list, MIXTURE_DEVICE_MAX_ADDR);

		total_pages = (online_count + 19) / 20;
		if (total_pages == 0)
			total_pages = 1;
		if (g_screen69_page >= total_pages)
			g_screen69_page = total_pages - 1;

		start = g_screen69_page * 20;
		end = start + 20;
		if (end > online_count)
			end = online_count;

		for (ctrl_idx = 0, list_idx = start; list_idx < end; ctrl_idx++, list_idx++)
		{
			uint8_t addr = online_list[list_idx];
			Screen69CacheEntry *cache = &g_screen69_cache[ctrl_idx];
			uint8_t need_refresh = 0;

			if(cache->active != 0U && cache->addr != addr)
			{
				cache->active = 0U;
			}

			/* XR5000_SCREEN69_FLICKER_FIX_20260727: ??????????????????strcmp???????????????? */
			if (screen69_circuit == 1)
			{
                uint8_t type = getPointTypeMixtureDetectName(addr);
                uint8_t state = 0U;
                uint16_t value = 0U;
                if(type == 6U)
                {
                    state = getPointTypeMixtureReceiveState(PointTypeData_Temper, addr);
                    value = getPointTypeMixtureReceiveData16(PointTypeData_Temper, addr);
                }
                else if(type == 5U)
                {
                    state = getPointTypeMixtureReceiveState(PointTypeData_Smoke, addr);

                }
                if(!cache->active || cache->sensor_enable != type || cache->temper_val != value || cache->temper_alarm != state)
                {
                    need_refresh = 1U;
                    cache->sensor_enable = type;
                    cache->temper_val = value;
                    cache->temper_alarm = state;
                    cache->active = 1U;
                }
			}
			else if (screen69_circuit == 2)
			{
				uint8_t new_state = MBusCtrl_GetDeviceState(addr);

				if (!cache->active
					|| cache->temper_val != new_state)
				{
					need_refresh = 1;
					cache->temper_val = new_state;
					cache->active = 1;
				}
			}
			else if (screen69_circuit == 3)
			{
				uint16_t enable = RS485Detect_GetSensorEnable(addr);
				uint16_t new_temper = (enable & (1 << 5)) ? RS485Detect_GetSensorValue(addr, RS485_SENSOR_TEMPERATURE) : 0;
				uint8_t new_temper_state = (enable & (1 << 5)) ? RS485Detect_GetSensorState(addr, RS485_SENSOR_TEMPERATURE) : 0;
				uint8_t new_smoke = (enable & (1 << 0)) ? RS485Detect_GetSensorState(addr, RS485_SENSOR_SMOKE) : 0;
				uint16_t new_co = (enable & (1 << 4)) ? RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO) : 0;
				uint16_t new_h2 = (enable & (1 << 2)) ? RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2) : 0;
				uint16_t new_voc = (enable & (1 << 3)) ? RS485Detect_GetSensorValue(addr, RS485_SENSOR_VOC) : 0;
				uint16_t new_ch4 = (enable & (1 << 1)) ? RS485Detect_GetSensorValue(addr, RS485_SENSOR_CH4) : 0;
				uint16_t new_pressure = (enable & (1 << 6)) ? RS485Detect_GetSensorValue(addr, RS485_SENSOR_PRESSURE) : 0;

				/* XR5000_LOOP3_STATUS_CHANGE_20260730: every rendered field can trigger a redraw. */
				if (!cache->active || cache->sensor_enable != enable || cache->temper_val != new_temper ||
					cache->temper_alarm != new_temper_state || cache->smoke_alarm != new_smoke ||
					cache->co_val != new_co || cache->h2_val != new_h2 || cache->voc_val != new_voc ||
					cache->ch4_val != new_ch4 || cache->pressure_val != new_pressure)
				{
					need_refresh = 1;
					cache->sensor_enable = enable;
					cache->temper_val = new_temper;
					cache->temper_alarm = new_temper_state;
					cache->smoke_alarm = new_smoke;
					cache->co_val = new_co;
					cache->h2_val = new_h2;
					cache->voc_val = new_voc;
					cache->ch4_val = new_ch4;
					cache->pressure_val = new_pressure;
					cache->active = 1;
				}
			}

			if (need_refresh)
			{
				cache->addr = addr;
				FormatScreen69DetectorText(screen69_circuit, addr, temp_buff);
				SetTextValue(current_screen_id, ctrl_idx + 1, temp_buff);
			}
		}

		for (; ctrl_idx < 20; ctrl_idx++)
		{
			if (g_screen69_cache[ctrl_idx].active)
			{
				SetTextValue(current_screen_id, ctrl_idx + 1, (uint8_t *)"");
				g_screen69_cache[ctrl_idx].active = 0;
			}
		}
	}
	// ??????????
	InternalScreenLinkageMonitorUpdataUI(current_screen_id);
	
	OutFireDeviceInternalScreenUpdataUI(current_screen_id, out_fire_start_ctrl);
	
	//FireAlarmTriggerLogicUpdataUI(current_screen_id, fire_alarm_logic_ctrl, fire_alarm_judge); /* ??????UI????bsp_logic_screen?????? */
	LogicScreen_UpdateUI(current_screen_id); /* ???????????????/?§Ò????????? */
	
	FireAlarmThresholdUpdataUI(current_screen_id, fire_alarm_threshold);
	/* ???????FCP-1011??¡¤????‰Ù???2026-08-06 */
	CanMonitorRefreshDisplay(current_screen_id);
	DeviceThreshold_UpdateUI(current_screen_id, getCurrentSystemRunState() == 2U);
}
/*! 
*  \brief  ?????????
*  \details  ??????????(?????GetControlValue)?????§Õ????
*  \param screen_id ????ID
*  \param control_id ???ID
*  \param state ???????0????1????
*  \param tubiaobh ????????0??0???1??1???2??2?...
*/
void TB_sahngchuan(uint16 screen_id, uint16 control_id, uint8  state, uint8  tubiaobh)
{ 
	switch(screen_id)
	{
		case 5: {
			PointTypeDetectorOnlineIconCtrl(control_id, state, tubiaobh);
			break;
		}
		default:
			break;
	}
}
/*! 
*  \brief  ????????
*  \details  ??????????(?????GetControlValue)?????§Õ????
*  \param screen_id ????ID
*  \param control_id ???ID
*  \param state ???????0????1????
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8  state)
{
	DeviceThreshold_NotifyButton(screen_id, control_id, state);
	if(screen_id == 1)
	{
		if(control_id==10 && state == 1)                                            
		{
			// new
			if( main_power_beep_ctrl     != 0 || 
					linkage_beep_ctrl        != 0 ||
					beep_fire_ctrl           != 0 ||
					beep_fault_ctrl          != 0 ||
					beep_spray_feedback_ctrl != 0 || 
					beep_general_io_ctrl     != 0
			) 
			{
				silencers_state = 1; // ??????
			}
			// end
			beep_fire_ctrl = 0;
			beep_fault_ctrl = 0;
			main_power_beep_ctrl = 0;
			linkage_beep_ctrl = 0;
			
			beep_spray_feedback_ctrl = 0;
			beep_general_io_ctrl = 0; // 
		}
		else if(control_id==3 && state == 1)  // ?????????????
		{
			if(mimajiyi == 0)
			{
				SwitchCurrentScreenId(23);
				bsp_screen_switch_ctrl.target_screen = 23;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
			else
			{
				SwitchCurrentScreenId(2);
				bsp_screen_switch_ctrl.target_screen = 2;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
		}
		else if(control_id == 32 && state == 1)  // ???????????????
		{
			EnterTimeDateSettingWithPassword(); /* XR5000_TIME_DATE_ENTRY_REUSE_20260802 */
		}
		else if( (control_id == 31 || control_id == 35) && state == 1 )
		{
			if(license_allow_use_state == 1)
			{
				setKeyValue(DEVICE_CTRL_KEY); // ????????? ?????????????????????
				SwitchCurrentScreenId(53);
				bsp_screen_switch_ctrl.target_screen = 53;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
		}
		else if( (control_id == 37 || control_id == 38) && state == 1 )
		{
			if(license_allow_use_state == 1)
			{
				SwitchToMonitorPageFrom(current_screen_id); /* XR5000_MONITOR_RETURN_NAV_CHANGE_20260802 */
			}
		}
		else if(control_id == 1 && state == 1) // ??????????
		{
			if(license_allow_use_state == 1)
			{
				//?????—¨?§Ö??????ID8
				// SwitchCurrentScreenId(8);
				// bsp_screen_switch_ctrl.target_screen = 8;
				// bsp_screen_switch_ctrl.switch_flag = 1;

				//?????
				SwitchCurrentScreenId(68);
				bsp_screen_switch_ctrl.target_screen = 68;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
		}
	}
	else if(screen_id == 3)
	{
		SimulationSerialPortButtonCtrl(&sspa, control_id, state);
	}
	else if(screen_id == 4) // ??????¦Ì??õô?????????
	{
		if(control_id == 45 && state == 1) // ??????????
		{
			uint8_t modify_flag = 0;
			for(uint8_t i = 0; i < 32; i++)
			{
				if(pack_online_buff[pack_circuit][i + 1] == 0)
				{
					pack_online_buff[pack_circuit][i + 1] = 1;
					modify_flag = 1;
				}
				// ????????????
				setkey_Value(4, point_type_detect_button_online_ctrl_val_map[i], 1);
			}
			if(modify_flag == 1)
			{
				Save_Pack_Set_Online_State();
			}
		}
		else if(control_id == 55 && state == 1) // ??????????
		{
			uint8_t modify_flag = 0;
			for(uint8_t i = 0; i < 32; i++)
			{
				if(pack_online_buff[pack_circuit][i + 1] == 1)
				{
					pack_online_buff[pack_circuit][i + 1] = 0;
					modify_flag = 1;
				}
				
				// ????????????
				setkey_Value(4, point_type_detect_button_online_ctrl_val_map[i], 0);
			}
			if(modify_flag == 1)
			{
				Save_Pack_Set_Online_State();
			}
		}
		else
		{
			for(uint8_t pack_id = 0; pack_id < 32; pack_id++)
			{
				if(point_type_detect_button_online_ctrl_val_map[pack_id] == control_id)
				{
					pack_online_buff[pack_circuit][pack_id + 1] = state;
					Save_Pack_Set_Online_State();
					break;
				}
			}
		}
	}
	else if(screen_id == 5)
	{
		PointTypeDetectorOnlineButtonCtrlPlus(control_id, state);
	}
	else if(screen_id == 6)
	{
		if(state == 1)
		{
			if(control_id == 5)
			{
				screen69_circuit = pack_circuit; /* XR5000_SCREEN69_NAVIGATION_FIX_20260729: lock selected circuit before screen switch. */
				g_screen69_transition_pending = 1; /* XR5000_SCREEN69_ATOMIC_RENDER_20260729: suppress page6 refresh until page69 confirmation. */
				bsp_screen_switch_ctrl.target_screen = 69;
				bsp_screen_switch_ctrl.switch_flag = 1;
				SwitchCurrentScreenId(69);
			}
			else if(control_id == 45)
			{
				switch (pack_circuit)
				{
				case 1:
					/* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: screen6 owns loop1 online configuration; screen7 is retired. */
					for(uint8_t i = 1U; i <= MIXTURE_DEVICE_MAX_ADDR; i++)
					{
						PointTypeMixtureOnlieStateSingleSetting(i, 1U);
					}
					SavePointTypeSetOnlieState();
					ReadPointTypeSetOnlieState();
					break;
				case 2:
					for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
					{
						if (MBusCtrl_GetDeviceType(i) != MBUS_CONTROL_DEV_UNKNOWN)
						{
							MBusCtrl_SetOnline(i, 1);
						}
					}
					MBusCtrl_SaveOnlineState();
					MBusCtrl_LoadOnlineState();
				    break;
				case 3:
					for (uint8_t i = 1; i < RS485_DETECT_MAX_DEVICES; i++)
					{
						RS485Detect_SetOnline(i, 1);
						AnimationPlayFrame(7, i, RS485Detect_GetOnline(i));
						// XR5000_LOOP3_CHANGE_20260726: Online setting is configuration only; do not write false recovery.
					}
					RS485Detect_SaveOnlineState();
					RS485Detect_LoadOnlineState();
					break;
				default:
					break;
				}
				// XR5000_ARROWSITE_FIX_20260726: Batch-online must reset bkcnc the same way batch-offline does, or screen59's cursor state goes stale.
				SyncMonitorSwitchSnapshot();
				BspScreenArrowSite(&bkcnc);
				home_statistics_force_refresh = 1;
			}
			else if(control_id == 55)
			{
				switch (pack_circuit)
				{
				case 1:
					/* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: clear all loop1 runtime state without screen7 writes. */
					for(uint8_t i = 1U; i <= MIXTURE_DEVICE_MAX_ADDR; i++)
					{
						PointTypeMixtureOnlieStateSingleSetting(i, 0U);
						Loop1ClearCurrentState(i);
					}
					clearPointTypeMixtureDisconnectCount();
					SavePointTypeSetOnlieState();
					ReadPointTypeSetOnlieState();
					break;
				case 2:
					for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
					{
						MBusCtrl_SetOnline(i, 0);
						mbus2_disconnect_memory[i] = 0;
						uint8_t index = findRecoveryDevice(MBUS_CONTROL_FLASH_ID, i, 0);
						if (index != 0xFF)
						{
							deletRecoveryRecord(index);
						}
					}
					MBusCtrl_SaveOnlineState();
					MBusCtrl_LoadOnlineState();
					break;
				case 3:
					for (uint8_t i = 1; i < RS485_DETECT_MAX_DEVICES; i++)
					{
						RS485Detect_SetOnline(i, 0);
						rs485_detect_disconnect_memory[i] = 0;
						memset(rs485_detect_alarm_memory[i], 0, sizeof(rs485_detect_alarm_memory[i]));
						// XR5000_LOOP3_CHANGE_20260726: Configured offline devices must not keep HMI runtime fault memory.
						AnimationPlayFrame(7, i, RS485Detect_GetOnline(i));
						// XR5000_LOOP3_STATUS_CHANGE_20260730: clear every current loop-3 fault/warning for this address.
						RS485Loop3ClearCurrentState(i);
					}
					RS485Detect_SaveOnlineState();
					RS485Detect_LoadOnlineState();
					break;
				default:
					break;
				}
				SyncMonitorSwitchSnapshot();
				BspScreenArrowSite(&bkcnc);
				home_statistics_force_refresh = 1;
			}
		}
	}
	else if(screen_id == 8)
	{
		if(state == 1)
		{
			if(control_id == 5)
			{
				setKeyValue(DEVICE_CTRL_KEY); // ????????? ?????????????????????
			
				SwitchCurrentScreenId(53);
				bsp_screen_switch_ctrl.target_screen = 53;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
			else if(control_id == 6)
			{
				
			}
			else if(control_id == 23)
			{
				setKeyValue(SIMU_SERIAL_PORT); // ????????? ?????????????????????
			
				SwitchCurrentScreenId(53);
				bsp_screen_switch_ctrl.target_screen = 53;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
			else if(control_id == 24)
			{
				setKeyValue(LINKAGE_PROGREM); // ????????? ?????????????????????
			
				SwitchCurrentScreenId(53);
				bsp_screen_switch_ctrl.target_screen = 53;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
		}
		RecordSwitchButtonCtrl(&bsrr, control_id, state);
	}
	else if(screen_id ==17)//?õô???????????
  {
			if(state==0)//????
			{
				switch(control_id)
				{
					case 5: cu_sxzt[1]=0;cu_tcq_sxzt[1]=0;CU_zx_buf[1]=0;break;//20240202??????CU_zx_buf[1]=0;????????????????????????????????????????
					case 8: cu_sxzt[2]=0;cu_tcq_sxzt[2]=0;CU_zx_buf[2]=0;break;
					case 11: cu_sxzt[3]=0;cu_tcq_sxzt[3]=0;CU_zx_buf[3]=0;break;
					case 14: cu_sxzt[4]=0;cu_tcq_sxzt[4]=0;CU_zx_buf[4]=0;break;
					case 17: cu_sxzt[5]=0;cu_tcq_sxzt[5]=0;CU_zx_buf[5]=0;break;
					case 20: cu_sxzt[6]=0;cu_tcq_sxzt[6]=0;CU_zx_buf[6]=0;break;
					case 25: cu_sxzt[7]=0;cu_tcq_sxzt[7]=0;CU_zx_buf[7]=0;break;
					case 28: cu_sxzt[8]=0;cu_tcq_sxzt[8]=0;CU_zx_buf[8]=0;break;
					case 31: cu_sxzt[9]=0;cu_tcq_sxzt[9]=0;CU_zx_buf[9]=0;break;
					case 34: cu_sxzt[10]=0;cu_tcq_sxzt[10]=0;CU_zx_buf[10]=0;break;
					case 37: cu_sxzt[11]=0;cu_tcq_sxzt[11]=0;CU_zx_buf[11]=0;break;
					case 40: cu_sxzt[12]=0;cu_tcq_sxzt[12]=0;CU_zx_buf[12]=0;break;
					
					case 50: cu_sxzt[13]=0;cu_tcq_sxzt[13]=0;CU_zx_buf[13]=0;break;
					case 53: cu_sxzt[14]=0;cu_tcq_sxzt[14]=0;CU_zx_buf[14]=0;break;
					case 56: cu_sxzt[15]=0;cu_tcq_sxzt[15]=0;CU_zx_buf[15]=0;break;
					case 59: cu_sxzt[16]=0;cu_tcq_sxzt[16]=0;CU_zx_buf[16]=0;break;
					case 62: cu_sxzt[17]=0;cu_tcq_sxzt[17]=0;CU_zx_buf[17]=0;break;
					case 65: cu_sxzt[18]=0;cu_tcq_sxzt[18]=0;CU_zx_buf[18]=0;break;
					case 68: cu_sxzt[19]=0;cu_tcq_sxzt[19]=0;CU_zx_buf[19]=0;break;
					case 71: cu_sxzt[20]=0;cu_tcq_sxzt[20]=0;CU_zx_buf[20]=0;break;
				}
				Save_cu_sxzt();//?›¥????????
				Save_cutcq_sxzt();
				
				SetTextInt32(17,6,cu_tcq_sxzt[1],0,1);//??????????????????—¨??????????0-??????1-?§Ù????????¦Ë??????????????
				SetTextInt32(17,9,cu_tcq_sxzt[2],0,1);
				SetTextInt32(17,12,cu_tcq_sxzt[3],0,1);
				SetTextInt32(17,15,cu_tcq_sxzt[4],0,1);
				SetTextInt32(17,18,cu_tcq_sxzt[5],0,1);
				SetTextInt32(17,21,cu_tcq_sxzt[6],0,1);
				SetTextInt32(17,26,cu_tcq_sxzt[7],0,1);
				SetTextInt32(17,29,cu_tcq_sxzt[8],0,1);
				SetTextInt32(17,32,cu_tcq_sxzt[9],0,1);
				SetTextInt32(17,35,cu_tcq_sxzt[10],0,1);
				SetTextInt32(17,38,cu_tcq_sxzt[11],0,1);
				SetTextInt32(17,41,cu_tcq_sxzt[12],0,1);
				
				SetTextInt32(17,49,cu_tcq_sxzt[13],0,1);
				SetTextInt32(17,52,cu_tcq_sxzt[14],0,1);
				SetTextInt32(17,55,cu_tcq_sxzt[15],0,1);
				SetTextInt32(17,58,cu_tcq_sxzt[16],0,1);
				SetTextInt32(17,61,cu_tcq_sxzt[17],0,1);
				SetTextInt32(17,64,cu_tcq_sxzt[18],0,1);
				SetTextInt32(17,67,cu_tcq_sxzt[19],0,1);
				SetTextInt32(17,70,cu_tcq_sxzt[20],0,1);
			}
			else if(state==1)//????
			{
				switch(control_id)
				{
					case 5: cu_sxzt[1]=1;break;
					case 8: cu_sxzt[2]=1;break;
					case 11: cu_sxzt[3]=1;break;
					case 14: cu_sxzt[4]=1;break;
					case 17: cu_sxzt[5]=1;break;
					case 20: cu_sxzt[6]=1;break;
					case 25: cu_sxzt[7]=1;break;
					case 28: cu_sxzt[8]=1;break;
					case 31: cu_sxzt[9]=1;break;
					case 34: cu_sxzt[10]=1;break;
					case 37: cu_sxzt[11]=1;break;
					case 40: cu_sxzt[12]=1;break;
					
					case 50: cu_sxzt[13]=1;break;
					case 53: cu_sxzt[14]=1;break;
					case 56: cu_sxzt[15]=1;break;
					case 59: cu_sxzt[16]=1;break;
					case 62: cu_sxzt[17]=1;break;
					case 65: cu_sxzt[18]=1;break;
					case 68: cu_sxzt[19]=1;break;
					case 71: cu_sxzt[20]=1;break;
				}
				Save_cu_sxzt();//?›¥????????
			}
	}
	else if(screen_id ==18)//?õô???????????
  {
		if(state==0)//????
		{
			switch(control_id)
			{
				case 6: cang_sxzt[1]=0;Cang_zx_buf[1]=0;break;
				case 5: cang_sxzt[2]=0;Cang_zx_buf[2]=0;break;
				case 8: cang_sxzt[3]=0;Cang_zx_buf[3]=0;break;
				case 9: cang_sxzt[4]=0;Cang_zx_buf[4]=0;break;
				case 11: cang_sxzt[5]=0;Cang_zx_buf[5]=0;break;
				case 12: cang_sxzt[6]=0;Cang_zx_buf[6]=0;break;
				case 14: cang_sxzt[7]=0;Cang_zx_buf[7]=0;break;
				case 15: cang_sxzt[8]=0;Cang_zx_buf[8]=0;break;
				case 17: cang_sxzt[9]=0;Cang_zx_buf[9]=0;break;
				case 18: cang_sxzt[10]=0;Cang_zx_buf[10]=0;break;
				case 20: cang_sxzt[11]=0;Cang_zx_buf[11]=0;break;
				case 21: cang_sxzt[12]=0;Cang_zx_buf[12]=0;break;
				
				case 25: cang_sxzt[13]=0;Cang_zx_buf[13]=0;break;
				case 28: cang_sxzt[14]=0;Cang_zx_buf[14]=0;break;
				case 31: cang_sxzt[15]=0;Cang_zx_buf[15]=0;break;
				case 34: cang_sxzt[16]=0;Cang_zx_buf[16]=0;break;
				case 37: cang_sxzt[17]=0;Cang_zx_buf[17]=0;break;
				case 40: cang_sxzt[18]=0;Cang_zx_buf[18]=0;break;
				case 43: cang_sxzt[19]=0;Cang_zx_buf[19]=0;break;
				case 46: cang_sxzt[20]=0;Cang_zx_buf[20]=0;break;
				// ??????????
				case 51: cang_sxzt[21]=0;Cang_zx_buf[21]=0;break;
				case 52: cang_sxzt[22]=0;Cang_zx_buf[22]=0;break;
				case 53: cang_sxzt[23]=0;Cang_zx_buf[23]=0;break;
				case 54: cang_sxzt[24]=0;Cang_zx_buf[24]=0;break;
			}
			Save_cang_sxzt();//?›¥????????
		}
		else if(state==1)//????
		{
			switch(control_id)
			{
				case 6: cang_sxzt[1]=1;break;
				case 5: cang_sxzt[2]=1;break;
				case 8: cang_sxzt[3]=1;break;
				case 9: cang_sxzt[4]=1;break;
				case 11: cang_sxzt[5]=1;break;
				case 12: cang_sxzt[6]=1;break;
				case 14: cang_sxzt[7]=1;break;
				case 15: cang_sxzt[8]=1;break;
				case 17: cang_sxzt[9]=1;break;
				case 18: cang_sxzt[10]=1;break;
				case 20: cang_sxzt[11]=1;break;
				case 21: cang_sxzt[12]=1;break;
				
				case 25: cang_sxzt[13]=1;break;
				case 28: cang_sxzt[14]=1;break;
				case 31: cang_sxzt[15]=1;break;
				case 34: cang_sxzt[16]=1;break;
				case 37: cang_sxzt[17]=1;break;
				case 40: cang_sxzt[18]=1;break;
				case 43: cang_sxzt[19]=1;break;
				case 46: cang_sxzt[20]=1;break;
				// ??????????
				case 51: cang_sxzt[21]=1;break;
				case 52: cang_sxzt[22]=1;break;
				case 53: cang_sxzt[23]=1;break;
				case 54: cang_sxzt[24]=1;break;
			}
			Save_cang_sxzt();//??????????
		}
	}
	else if(screen_id == 19)
  {
		if(control_id==4 && state == 1)                                            
		{
			SetTextValue(1, 4, "????????¦Ë??...?????...");
			if(ONLINE_TIMEOUT > 6)
			{
				kaijiyanshi = ONLINE_TIMEOUT - 6;
			}
			else
			{
				kaijiyanshi = 0;
			}
			
			ResetAllBusDevice();
		}
	}
	else if(screen_id == 21)
	{

	}
	else if(screen_id == 22)
	{
		
	}
	else if(screen_id == 23)
	{
		if(mimajiyi == 0) // ???¦Ä???????? ???? 4.25??????? ???????????????
		{
			if(control_id==4 && state == 1)                                            
			{
				if(yonghumima == SystemSaveInfo.user_password)                                                       
				{ 
					yonghumima=0;
					mimajiyi++;
					clearTextValue(23,2);//(????ID,???ID??
					SetScreen(2);	//??????ID???§Ý????›Ô???y???
					osDelay(10);
					GetScreen();
					current_screen_id=2;
				}else
				{
					clearTextValue(23,2);//(????ID,???ID??
					SetTextValue(23, 2, "???????");
				}
			}
		}
	}
	//???????????
	else if(screen_id == 24)
	{
		if(control_id==4 && state == 1)                                                            //
		{ 
			if(yonghumm1 == SystemSaveInfo.user_password)
			{
				if(yonghumm2 == yonghumm3)
				{
					SystemSaveInfo.user_password = yonghumm3;
					SystemInfoSave();
					SystemInfoLoad();
					SetTextValue(24,7,"???¨®????");
					mmsdSTA=4;
				}
				else if(yonghumm2 != yonghumm3)
				{
					SetTextValue(24,7,"?????????????????¦Â??????");
				}
			}
			else if(yonghumm1!=SystemSaveInfo.user_password)
			{
				SetTextValue(24,7,"????????????????");
			}				
		}
	}
	else if(screen_id ==26)//
	{
		if(state==0)//????
		{
			switch(control_id)
			{
				case 6: cang_pbzt[1]=0;break;
				case 5: cang_pbzt[2]=0;break;
				case 8: cang_pbzt[3]=0;break;
				case 9: cang_pbzt[4]=0;break;
				case 11: cang_pbzt[5]=0;break;
				case 12: cang_pbzt[6]=0;break;
				case 14: cang_pbzt[7]=0;break;
				case 15: cang_pbzt[8]=0;break;
				case 17: cang_pbzt[9]=0;break;
				case 18: cang_pbzt[10]=0;break;
				case 20: cang_pbzt[11]=0;break;
				case 21: cang_pbzt[12]=0;break;
				
				case 25: cang_pbzt[13]=0;break;
				case 28: cang_pbzt[14]=0;break;
				case 31: cang_pbzt[15]=0;break;
				case 34: cang_pbzt[16]=0;break;
				case 37: cang_pbzt[17]=0;break;
				case 40: cang_pbzt[18]=0;break;
				case 43: cang_pbzt[19]=0;break;
				case 46: cang_pbzt[20]=0;break;
				// ??????????
				case 51: cang_pbzt[21]=0;break;
				case 52: cang_pbzt[22]=0;break;
				case 53: cang_pbzt[23]=0;break;
				case 54: cang_pbzt[24]=0;break;
			}
			Save_cang_pbzt();//?›¥????????
		}else
		if(state==1)//????
		{
			switch(control_id)
			{
				case 6: cang_pbzt[1]=1;break;
				case 5: cang_pbzt[2]=1;break;
				case 8: cang_pbzt[3]=1;break;
				case 9: cang_pbzt[4]=1;break;
				case 11: cang_pbzt[5]=1;break;
				case 12: cang_pbzt[6]=1;break;
				case 14: cang_pbzt[7]=1;break;
				case 15: cang_pbzt[8]=1;break;
				case 17: cang_pbzt[9]=1;break;
				case 18: cang_pbzt[10]=1;break;
				case 20: cang_pbzt[11]=1;break;
				case 21: cang_pbzt[12]=1;break;
				
				case 25: cang_pbzt[13]=1;break;
				case 28: cang_pbzt[14]=1;break;
				case 31: cang_pbzt[15]=1;break;
				case 34: cang_pbzt[16]=1;break;
				case 37: cang_pbzt[17]=1;break;
				case 40: cang_pbzt[18]=1;break;
				case 43: cang_pbzt[19]=1;break;
				case 46: cang_pbzt[20]=1;break;
				// ??????????
				case 51: cang_pbzt[21]=1;break;
				case 52: cang_pbzt[22]=1;break;
				case 53: cang_pbzt[23]=1;break;
				case 54: cang_pbzt[24]=1;break;
			}
			Save_cang_pbzt();//?????????? pack_bianhaobuf
		}
	}
	else if(screen_id == 46) 
	{
		if(control_id == 6 && state == 1)                                            
		{
			qingchujilu = 1;
		}
	}
	else if(screen_id == 50)
	{
		if(control_id == 1 && state == 1)                                            
		{
			SystemInfoSave(); // ?????¨®???????????????????EEPROM
		}
	}
	else if(screen_id == 71U)
	{
		if(control_id == 300U && state == 1U)
		{
			/* ???????FCP-1011??¡¤????‰Ù???2026-08-06 */
			bsp_screen_switch_ctrl.target_screen = 68U;
			bsp_screen_switch_ctrl.switch_flag = 1U;
			SwitchCurrentScreenId(68U);
		}
	}
	else if(screen_id == CHECK_SCREEN_ID)
	{
		if(control_id == 300U && state == 1U)
		{
			/* XR5000_CHECK_CHANGE_20260804: screen 72 return button. */
			check_last_activity_tick = osKernelGetTickCount();
			CheckModeExitToHome();
		}
	}
	else if(screen_id == 53)
	{
		if(control_id == 4 && state == 1) 
		{
			clearTextValue(screen_id, 2);
			if(yonghumima == SystemSaveInfo.user_password)                                                       
			{
				switch(getKeyPressValue())
				{
					case SELFCHECK_KEY: { // ???
						SetScreen(1);	// ??????? ????????
						osDelay(5);
						GetScreen(); 
						// ?????????????? ???????????¦Ì???? 
						BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_SYS_SELF_CHECK, LINKAGE_CLUSTER_ID, SYS_SELFCHECK_Package_ID);
						SpecialSelfCheckLedCtrl(LED_ON);
						
						break;
					}
					case SILENSE_KEY: // ????
						break;
					case RESET_KEY: {  // ??¦Ë 
						// ?????¦Ë????????
						SetScreen(1);	// ??????? ????????
						osDelay(5);
						GetScreen();
						SetTextValue(1, 4, "????????¦Ë??...?????...");
						BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_SYS_RESET, LINKAGE_CLUSTER_ID, SYS_RESET_Package_ID);
						if(ONLINE_TIMEOUT > 6)
						{
							kaijiyanshi = ONLINE_TIMEOUT - 6;
						}
						else
						{
							kaijiyanshi = 0;
						}
						ResetAllBusDevice();
						break;
					}
					case CHECK_KEY:
						/* XR5000_CHECK_CHANGE_20260804: legacy password entry is intentionally retired. */
						setKeyValue(NONE_KEY);
						break;
					case MODIFY_TIME_KEY:  // ????????
						SetScreen(41);	// ????????????
						osDelay(5);
						GetScreen();
						break;
					case DEVICE_CTRL_KEY: {
						SetScreen(6);	//
						osDelay(5);
						GetScreen();
						break;
					}
					case DEVICE_SHIELD_KEY: { // XR5000_DEVICE_SHIELD_ENTRY_20260802: ?õô????
						SetScreen(70);
						osDelay(5);
						GetScreen();
						break;
					}
					case SIMU_SERIAL_PORT: {
						SetScreen(3);	// 
						osDelay(5);
						GetScreen();
						
						break;
					}
					case LINKAGE_PROGREM: {
						SetScreen(43);	//
						osDelay(5);
						GetScreen();
						
						break;
					}
					case SIREN_KEY:    // ?????????
						screen_show_siren_information ^= 0x0F; // ???????¦Ë??
						break;
					case LINKAGE_START_KEY: // ?????õô???
						linkage_start_key_press_flag = 1;
						break;
					case PART1_SPRY_START:
						FireExtinguishDevice1HandStart(&fedas);
						break;
					case PART2_SPRY_START:
						FireExtinguishDevice2HandStart(&fedas);
						break;
					default:
						SetScreen(1);	// ????????????
						osDelay(5);
						GetScreen();
						break;
				}
				setKeyValue(NONE_KEY);
			}
			else if(yonghumima == 114514)
			{
				SetScreen(3);	// ????????????
				osDelay(5);
				GetScreen();
			}
		}

	}		
	else if(screen_id == 54)
	{
		DetectorFreshPageButtonCtrl(&ddsc, control_id, state);
	}
	else if(screen_id == 55)
	{
		RecordSwitchButtonCtrl(&bsrr, control_id, state);
	}
	else if(screen_id == 56 || screen_id == 57)
	{
		InternalScreenRecordShiftButtonCtrl(&bsrr, control_id, state);
	}
	else if(screen_id == 58) // ??????????
	{
		HandForceStartAnyCluster(&fedas, control_id, state);
	}
	else if(screen_id == 59) // 
	{
		uint16_t target_screen = MonitorPageGetReturnTarget(); /* XR5000_MONITOR_RETURN_NAV_CHANGE_20260802 */
		SwitchCurrentScreenId(target_screen);
		bsp_screen_switch_ctrl.target_screen = target_screen;
		bsp_screen_switch_ctrl.switch_flag = 1;
	}
	else if(screen_id == 61)
	{
		DetectorFreshPageButtonCtrl_32Pack(&ddsc_32p, control_id, state);
		DetectorMonitorButtonCtrl_32Pack(&ddsc_32p, control_id, state);
	}
	else if(screen_id == 63)
	{
		CabinFreshPageButtonCtrl(&cabin_dsc, control_id, state);
	}
	else if(screen_id == 66)
	{
		PointTypeDetectorOnlineButtonCtrl(control_id, state);
	}
	else if(screen_id == 67)
	{
		PointTypeDetectorButtonCtrlApp(&ptsc, control_id, state);
		
		CompositeDetectorButtonCtrlApp(&cpsc, control_id, state);
	}

	//2026/7/22????????
	else if (screen_id == 69)
	{
		if (state == 1)
		{
			uint8_t online_list[MIXTURE_DEVICE_MAX_ADDR] = {0};
			uint8_t online_count;
			uint8_t total_pages;

			online_count = GetCircuitOnlineList(screen69_circuit, online_list, MIXTURE_DEVICE_MAX_ADDR);
			total_pages = (online_count + 19) / 20;
			if (total_pages == 0)
				total_pages = 1;

			if (control_id == 300)
			{
				if (g_screen69_page > 0)
					g_screen69_page--;
			}
			else if (control_id == 301)
			{
				if (g_screen69_page < total_pages - 1)
					g_screen69_page++;
			}
		}
	}
	InternalLinkageMonitorButtonDeal(screen_id, control_id, state);
	//FireAlarmTriggerLogicButtonSet(screen_id, control_id, state, &fire_alarm_logic_ctrl); /* ??????????????bsp_logic_screen?????? */
	LogicScreen_OnButton(screen_id, control_id, state); /* ?????????????Ú…???Ë®?????? */
	SuperAdminButtonCtrl(screen_id, control_id, state, &button_ctrl);
	SuperAdminPasswordButtonCtrl(screen_id, control_id, state, &super_admin_password);
	
}


/*! 
*  \brief  ????????
*  \details  ???????????????(?????GetControlValue)?????§Õ????
*  \details  ???????????????????????¡¤???MCU???????????????????????
*  \details  ????????????§ß??¡¤???????????????????
*  \param screen_id ????ID
*  \param control_id ???ID
*  \param str ??????????
*/
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str)
{
   { 
			if(control_id == 25) // ???CAN2ID???
      {
				int32 value=0;  			
				sscanf((const char*)(char*)str,"%ld",&value); 
				SystemSaveInfo.can2_slave_addr = value;//MODBUS???
				SystemInfoSave();
				SystemInfoLoad();
			}
			else if(control_id == 30) // ?????485???
      {
				int32 value=0;  			
				sscanf((const char*)(char*)str,"%ld",&value); 
				SystemSaveInfo.slave_addr485_Station = value;//MODBUS???
				SystemInfoSave();
				SystemInfoLoad();
			}
			else if(control_id == 22)
			{
				int32 value=0;  			
				sscanf((const char*)(char*)str,"%ld",&value); 
				SystemSaveInfo.slave_addr485_EMS = value;//MODBUS???
				SystemInfoSave();
				SystemInfoLoad();
			}
		}
    if(screen_id==2)                                                                 //????ID2????????¨²????
    {                                                                            
			
    }
		else if(screen_id == 3)
		{
			SimulationSerialPortTextCtrl(&sspa, control_id, str);
		}
		else if(screen_id == 4) // 
		{
			uint8_t temp_str_len = strlen((char *)str);
			if(temp_str_len != 0)
			{
				if(control_id == 2)
				{
					int8_t success_len;
					int32_t x,y;
					success_len = sscanf((const char*)str, "%d.%d", &x, &y);  
					if(success_len == 2 && x >0 && y > 0) // ??????? ????x y??????
					{
						uint8_t modify_flag = 0;
						if (x > y)
						{
							x ^= y;
							y ^= x;
							x ^= y;
						}
						for(uint8_t i = x; i < y + 1; i++)
						{
							if(pack_online_buff[pack_circuit][i] == 0)
							{
								pack_online_buff[pack_circuit][i] = 1;
								modify_flag = 1;	
							}
							// ?????????
							setkey_Value(4, point_type_detect_button_online_ctrl_val_map[i - 1], 1);
						}
						if(modify_flag == 1)
						{
							Save_Pack_Set_Online_State(); // ?????¦Â?§ÕFLASH
						}
					}
				}
				SetTextValue(4, 2, "????????");
			}
		}
		else if(screen_id == 5)
		{
			PointTypeDetectorOnlineTextCtrlPlus(control_id, str);
		}
		else if(screen_id == 6) // 
		{
			uint8_t temp_str_len = strlen((char *)str);
			if(temp_str_len != 0)
			{
				if(control_id == 2)
				{
					int8_t success_len;
					int32_t x,y;
					success_len = sscanf((const char*)str, "%d.%d", &x, &y);  
					if(success_len == 2 && x >0 && y > 0) // ??????? ????x y??????
					{
						switch(pack_circuit)
						{
							case 1:
							{
								if (x > MIXTURE_DEVICE_MAX_ADDR || y > MIXTURE_DEVICE_MAX_ADDR) break;
								uint8_t modify_flag = 0;
								if (x > y)
								{
									x ^= y;
									y ^= x;
									x ^= y;
								}
								for (uint8_t i = x; i < y + 1; i++)
								{
									if (getPointTypeMixtureDetectOnlineState(i) == 0)
									{
										PointTypeMixtureOnlieStateSingleSetting(i, 1);  
										modify_flag = 1;
									}
									// ?????????
								}
								if (modify_flag == 1)
								{
									SavePointTypeSetOnlieState();  // ?????¦Â?§ÕFLASH
								}
							}
								break;
							case 2:
							{
								if (x >= MBUS_CONTROL_MAX_DEVICES || y >= MBUS_CONTROL_MAX_DEVICES) break;
								uint8_t modify_flag = 0;
								if (x > y)
								{
									x ^= y;
									y ^= x;
									x ^= y;
								}
								for (uint8_t i = x; i < y + 1; i++)
								{
									if (MBusCtrl_GetOnline(i) == 0)
									{
										MBusCtrl_SetOnline(i, 1);
										modify_flag = 1;
									}
								}
								if (modify_flag == 1)
								{
									MBusCtrl_SaveOnlineState();
								}
							}
								break;
							case 3:
							 {
								 if (x >= RS485_DETECT_MAX_DEVICES || y >= RS485_DETECT_MAX_DEVICES) break;
								 uint8_t modify_flag = 0;
								 if (x > y)
								 {
									 x ^= y;
									 y ^= x;
									 x ^= y;
								 }
								 for (uint8_t i = x; i < y + 1; i++)
								 {
									 if (RS485Detect_GetOnline(i) == 0)
									 {
										 RS485Detect_SetOnline(i, 1);
										 modify_flag = 1;
									 }
									 AnimationPlayFrame(7, i, RS485Detect_GetOnline(i));
								 }
								 if (modify_flag == 1)
								 {
									 RS485Detect_SaveOnlineState();
								 }
							 }
								break;
							case 4:
								break;
							default:
								break;
						}
					}
				}
			}
			SetTextValue(6, 2, "????????");
		}
		else if(screen_id == 10)
		{
			if(control_id == 8)
			{
				uint8_t len = strlen((char *)str);
				
				SetTextValue(10, 8, "?????????????ID");
				if(len != 10)
				{
					return;
				}
				for(uint8_t i = 0; i < 10; i++)
				{
					SystemSaveInfo.pref_license_store[i] = str[i];
				}
				
				SystemSaveInfo.license_remain_day = 6666;
				SystemInfoSave();
				
				if(strlen((char *)SystemSaveInfo.pref_license_store) != 0)
				{
					uint8_t len = 0;
					uint8_t buff[32] = {0};
					len = sprintf((char *)buff, "??????????:");
					for(uint8_t i = 0; i < 10; i++)
					{
						buff[len + i] = SystemSaveInfo.pref_license_store[i];
					}
					SetTextValue(10, 9, buff);
				}
				else
				{
					SetTextValue(10, 9, "??????????:??");
				}

			}
		}
		else if(screen_id==17)  
    {                                                                           
        int32 value=0;  
        sscanf((const char*)str, "%ld", &value);    //??????????????? 
				switch(control_id)
				{
					case 6:if(cu_sxzt[1]==1){cu_tcq_sxzt[1]=value;kaijiyanshi=0;}break;//?????????????????
					case 9:if(cu_sxzt[2]==1){cu_tcq_sxzt[2]=value;kaijiyanshi=0;}break;//?????????????????
					case 12:if(cu_sxzt[3]==1){cu_tcq_sxzt[3]=value;kaijiyanshi=0;}break;//?????????????????
					case 15:if(cu_sxzt[4]==1){cu_tcq_sxzt[4]=value;kaijiyanshi=0;}break;//?????????????????
					case 18:if(cu_sxzt[5]==1){cu_tcq_sxzt[5]=value;kaijiyanshi=0;}break;//?????????????????
					case 21:if(cu_sxzt[6]==1){cu_tcq_sxzt[6]=value;kaijiyanshi=0;}break;//?????????????????
					case 26:if(cu_sxzt[7]==1){cu_tcq_sxzt[7]=value;kaijiyanshi=0;}break;//?????????????????
					case 29:if(cu_sxzt[8]==1){cu_tcq_sxzt[8]=value;kaijiyanshi=0;}break;//?????????????????
					case 32:if(cu_sxzt[9]==1){cu_tcq_sxzt[9]=value;kaijiyanshi=0;}break;//?????????????????
					case 35:if(cu_sxzt[10]==1){cu_tcq_sxzt[10]=value;kaijiyanshi=0;}break;//?????????????????
					case 38:if(cu_sxzt[11]==1){cu_tcq_sxzt[11]=value;kaijiyanshi=0;}break;//?????????????????
					case 41:if(cu_sxzt[12]==1){cu_tcq_sxzt[12]=value;kaijiyanshi=0;}break;//?????????????????
					
					case 49:if(cu_sxzt[13]==1){cu_tcq_sxzt[13]=value;kaijiyanshi=0;}break;//?????????????????
					case 52:if(cu_sxzt[14]==1){cu_tcq_sxzt[14]=value;kaijiyanshi=0;}break;//?????????????????
					case 55:if(cu_sxzt[15]==1){cu_tcq_sxzt[15]=value;kaijiyanshi=0;}break;//?????????????????
					case 58:if(cu_sxzt[16]==1){cu_tcq_sxzt[16]=value;kaijiyanshi=0;}break;//?????????????????
					case 61:if(cu_sxzt[17]==1){cu_tcq_sxzt[17]=value;kaijiyanshi=0;}break;//?????????????????
					case 64:if(cu_sxzt[18]==1){cu_tcq_sxzt[18]=value;kaijiyanshi=0;}break;//?????????????????
					case 67:if(cu_sxzt[19]==1){cu_tcq_sxzt[19]=value;kaijiyanshi=0;}break;//?????????????????
					case 70:if(cu_sxzt[20]==1){cu_tcq_sxzt[20]=value;kaijiyanshi=0;}break;//?????????????????
				}
				Save_cutcq_sxzt();
				SetTextInt32(17,6,cu_tcq_sxzt[1],0,1);//??????????????????—¨??????????0-??????1-?§Ù????????¦Ë??????????????
				SetTextInt32(17,9,cu_tcq_sxzt[2],0,1);
				SetTextInt32(17,12,cu_tcq_sxzt[3],0,1);
				SetTextInt32(17,15,cu_tcq_sxzt[4],0,1);
				SetTextInt32(17,18,cu_tcq_sxzt[5],0,1);
				SetTextInt32(17,21,cu_tcq_sxzt[6],0,1);
				SetTextInt32(17,26,cu_tcq_sxzt[7],0,1);
				SetTextInt32(17,29,cu_tcq_sxzt[8],0,1);
				SetTextInt32(17,32,cu_tcq_sxzt[9],0,1);
				SetTextInt32(17,35,cu_tcq_sxzt[10],0,1);
				SetTextInt32(17,38,cu_tcq_sxzt[11],0,1);
				SetTextInt32(17,41,cu_tcq_sxzt[12],0,1);
				
				SetTextInt32(17,49,cu_tcq_sxzt[13],0,1);
				SetTextInt32(17,52,cu_tcq_sxzt[14],0,1);
				SetTextInt32(17,55,cu_tcq_sxzt[15],0,1);
				SetTextInt32(17,58,cu_tcq_sxzt[16],0,1);
				SetTextInt32(17,61,cu_tcq_sxzt[17],0,1);
				SetTextInt32(17,64,cu_tcq_sxzt[18],0,1);
				SetTextInt32(17,67,cu_tcq_sxzt[19],0,1);
				SetTextInt32(17,70,cu_tcq_sxzt[20],0,1);
	}
		else if(screen_id==23)                                                                 //????ID2????????¨²????
    {
			if(mimajiyi == 0) // ???¦Ä???????? ???? 4.25??????? ???????????????
			{
				if(control_id==2)                                                            //
				{
					int32 value=0;  			
					sscanf((const char*)(char*)str,"%ld",&value);                                                    //??????????????? 					
					yonghumima=value;													//????
				}  
			}                                                                     
    }
		else if(screen_id == 24)                                                                 //????ID2????????¨²????
    {                                                                            
        int32 value=0;  			
        sscanf((const char*)(char*)str,"%ld",&value);                                                    //??????????????? 
				if(control_id==2)                                                            //
        {                                                                         
					if(value == SystemSaveInfo.user_password)                                                       
            { 
							
							SetTextValue(24,7,"?????????");
							yonghumm1=value;//???????????
							value = 0; 
						}else
						{
							SetTextValue(24,7,"???????");
						}						
        } 
				if(control_id==5)                                                            //
        {                                                                         
						yonghumm2=value;//????????????1
						value = 0; 
        }	
				if(control_id==6)                                                            //
        {                                                                         
						yonghumm3=value;//????????????2
						value = 0; 
        }				
    }
		else if(screen_id==25)                                                                 //????ID2????????¨²????
    {                                                                            
 
    }
		else if(screen_id == 27)
		{
			if(control_id == 9) // ????????????
			{
				uint8_t len = strlen((char *)str);
				if(len != 10)
				{
					return;
				}
				for(uint8_t i = 0; i < 10; i++)
				{
					SystemSaveInfo.curr_license_store[i] = str[i];
				}

				if(strlen((char *)SystemSaveInfo.curr_license_store) != 0)
				{
					uint8_t len = 0;
					uint8_t buff[32] = {0};
					len = sprintf((char *)buff, "??????:");
					for(uint8_t i = 0; i < 10; i++)
					{
						buff[len + i] = SystemSaveInfo.curr_license_store[i];
					}
					SetTextValue(27, 6, buff);
				}
				else
				{
					SetTextValue(27, 6, "??????:??");
				}
				
				char slicense_buff[10] = {0};
				
				if(strncmp((char *)SystemSaveInfo.curr_license_store, (char *)SystemSaveInfo.pref_license_store, 10) == 0 && SystemSaveInfo.license_remain_day == 6666) // ??????ID ?? ?????????
				{
					for(uint8_t i = 0; i < 10; i++)
					{
						SystemSaveInfo.last_license_store[i] = SystemSaveInfo.curr_license_store[i];
					}
					
					getBM8563TimeToSystemTime(); // ??????RTC???
							
					SystemSaveInfo.license_year = years;
					SystemSaveInfo.license_month = months;
					SystemSaveInfo.license_days = days;
					SystemSaveInfo.license_hour = hours;
					
					SystemSaveInfo.license_minute = minutes;
					SystemSaveInfo.license_second = secs;
					
					SystemSaveInfo.license_remain_day = 30; // ???????30??
					
					uint8_t slicense_buff[32] = {0};
					sprintf((char *)slicense_buff, "?????:???%d??", SystemSaveInfo.license_remain_day);
					SetTextValue(27, 7, slicense_buff);
				}
				else // ????????????¦Å?????
				{
					uint8_t flag = 0;
					
					for(uint8_t i = 0; i < 6; i++)
					{
						generate_new_license_code((char *)SystemSaveInfo.last_license_store, getGenerationDate(i), slicense_buff);
						if(strncmp((char *)SystemSaveInfo.curr_license_store, slicense_buff, 10) == 0)
						{
							flag = 1; // ??????
							getBM8563TimeToSystemTime(); // ??????RTC???
							
							SystemSaveInfo.license_year = years;
							SystemSaveInfo.license_month = months;
							SystemSaveInfo.license_days = days;
							SystemSaveInfo.license_hour = hours;
							
							SystemSaveInfo.license_minute = minutes;
							SystemSaveInfo.license_second = secs;

							SystemSaveInfo.license_remain_day = getRemainUseDate(i); // ???????
							
							for(uint8_t i = 0; i < 10; i++)
							{
								SystemSaveInfo.last_license_store[i] = SystemSaveInfo.curr_license_store[i];
							}
							if(SystemSaveInfo.license_remain_day != 999)
							{
								uint8_t slicense_buff[32] = {0};
								sprintf((char *)slicense_buff, "?????:???%d??", SystemSaveInfo.license_remain_day);
								SetTextValue(27, 7, slicense_buff);
							}
							else
							{
								SetTextValue(27, 7, "?????:??????§¹");
							}
							
							break;
						}
					}
					if(flag == 0) // ?????????????
					{
						SetTextValue(27, 7, "?????:??§¹");
						SystemSaveInfo.license_remain_day = 0; // ???????????
					}
					
				}
				SystemInfoSave();
				last_time_stamp -=3600000;
				SetTextValue(27, 9, "???????????");
			}
		}
		else if(screen_id == 41) // ?????????
		{
			// ????????
			InternalScreenRTCSetting(screen_id, control_id, str); //RTC???
		}
		else if(screen_id == 50)
		{
			if(control_id == 17)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // ??????????????? 
				SystemSaveInfo.factory_release_year = value - 2000;
				
			}		
			else if(control_id == 18)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // ??????????????? 
				SystemSaveInfo.factory_release_month = value;
			}		
			else if(control_id == 19)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // ??????????????? 
				// ????????????
				SystemSaveInfo.factory_release_days = value;
			}		
		}
		else if(screen_id == 53)
		{
			if(control_id == 2)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // ??????????????? 
				yonghumima=value;									// ???????
			}				
		}
		else if(screen_id == 67)
		{
			PointTypeDetectorTextInputCtrlApp(&ptsc, control_id, str);
			
			CompositeDetectorTextInputCtrlApp(&cpsc, control_id, str);
		}
		
		// ????????
		OutFireDeviceInternalScreenTexttSet(screen_id, control_id, str, &out_fire_start_ctrl); // ?????????
		
		SuperAdminInternalScreenTextCtrl(screen_id, control_id, str, &super_admin_password);
		
		FireAlarmThresholdSettingInternalScreenText(screen_id, control_id, str, &fire_alarm_threshold);
}                                                                                

/*!                                                                              
*  \brief  ???????????                                                       
*  \details  ????GetControlValue?????§Õ????                                  
*  \param screen_id ????ID                                                      
*  \param control_id ???ID                                                     
*  \param value ?                                                              
*/                                                                              
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value)           
{  
//    if(screen_id == 5)
//    {
//        Progress_Value = value;                                  
//        SetTextInt32(5,2,Progress_Value,0,1);                                        //???????????     
//    }    
}                                                                                

/*!                                                                              
*  \brief  ???????????                                                       
*  \details  ???????????(?????GetControlValue)?????§Õ????                  
*  \param screen_id ????ID                                                      
*  \param control_id ???ID                                                     
*  \param value ?                                                              
*/                                                                              
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value)             
{                                                             

}


/*! 
*  \brief  ???????
*  \details  ????GetControlValue?????§Õ????
*  \param screen_id ????ID
*  \param control_id ???ID
*  \param value ?
*/
void NotifyMeter(uint16 screen_id, uint16 control_id, uint32 value)
{
    //TODO: ??????????
}

/*! 
*  \brief  ????????
*  \details  ??????????????????§Õ????
*  \param screen_id ????ID
*  \param control_id ???ID
*  \param item ?????????
*  \param state ???????0?????1????
*/
void NotifyMenu(uint16 screen_id, uint16 control_id, uint8 item, uint8 state)
{
  DeviceThreshold_NotifyMenu(screen_id, control_id, item, state);
  //TODO: ??????????
	// ????????? ??????????Ú… ??????????Ú… ????????
	
	OutFireDeviceInternalScreenButtonSet(screen_id, control_id, item, state, &out_fire_start_ctrl);
	if(screen_id == 3)
	{
		SimulationSerialPortMenuCtrl(&sspa, control_id, item, state);
	}
	else if(screen_id == 4)
	{
		if(control_id == 77 && state == 1)
		{
			uint8_t temp_pack_id = item + 1;
			if(pack_circuit != temp_pack_id)
			{
				uint8_t temp_key_value; 
				for(uint8_t i = 0; i < 32; i++)
				{
					temp_key_value = pack_online_buff[temp_pack_id][i + 1] ? 1 : 0;
					// ?????????
					setkey_Value(4, point_type_detect_button_online_ctrl_val_map[i], temp_key_value);
				}
			}
			pack_circuit = temp_pack_id;
		}
	}
	else if (screen_id == 6)
	{
		if (control_id == 77 && state == 1)
		{
			pack_circuit = item + 1;
		}
	}
	else if(screen_id == 54)
	{
		DetectorDataFreshMenuCtrl(&ddsc, control_id, item, state);
	}
	else if(screen_id == 61)
	{
		DetectorDataFreshMenuCtrl_32Pack(&ddsc_32p, control_id, item, state);
	}
	else if(screen_id == 68)
	{
		if(control_id == 17 && state == 1 && item == 0U)
		{
			bsp_screen_switch_ctrl.target_screen = 75U;
			bsp_screen_switch_ctrl.switch_flag = 1U;
			SwitchCurrentScreenId(75U);
		}
		else if(control_id == 16 && state == 1)
		{
			switch(item)
			{
				case 0:
					break;
				case 1:
					setKeyValue(DEVICE_CTRL_KEY);
					SwitchCurrentScreenId(53);
					bsp_screen_switch_ctrl.target_screen = 53;
					bsp_screen_switch_ctrl.switch_flag = 1;
					break;
				case 2:
					setKeyValue(DEVICE_SHIELD_KEY); // XR5000_DEVICE_SHIELD_ENTRY_20260802: ?õô????
					SwitchCurrentScreenId(53);
					bsp_screen_switch_ctrl.target_screen = 53;
					bsp_screen_switch_ctrl.switch_flag = 1;
					break;
				default:
					break;
			}
		}
		else if(control_id == 19 && state == 1)
		{
			switch(item)
			{
				case 0:
					setKeyValue(LINKAGE_PROGREM); // ????????? ?????????????????????
					SwitchCurrentScreenId(53);
					bsp_screen_switch_ctrl.target_screen = 53;
					bsp_screen_switch_ctrl.switch_flag = 1;
					break;
				case 1:
					break;
				default:
					break;
			}
		}
		else if(control_id == 23 && state == 1)
		{
			RecordSwitchButtonCtrl(&bsrr, item+1, state);
		}
		else if(control_id == 21 && state == 1)
		{
			switch(item)
			{
				case 0:
					EnterTimeDateSettingWithPassword(); /* XR5000_TIME_DATE_ENTRY_REUSE_20260802 */
					break;
				case 1:
					break;
				default:
					break;
			}
		}
		else if(control_id == 25 && state == 1)
		{
			if(item == 0U)
			{
				/* ???????FCP-1011??¡¤????‰Ù???2026-08-06 */
				bsp_screen_switch_ctrl.target_screen = 71U;
				bsp_screen_switch_ctrl.switch_flag = 1U;
				SwitchCurrentScreenId(71U);
			}
		}
		else if(control_id == 26U && state == 1U)
		{
			uint16_t target = (item == 0U) ? 73U : 76U;
			if(item <= 1U)
			{
				bsp_screen_switch_ctrl.target_screen = target;
				bsp_screen_switch_ctrl.switch_flag = 1U;
				SwitchCurrentScreenId(target);
			}
		}
}
	else if(screen_id == 75U && control_id == 350U && state == 1U)
	{
		uint8_t first = (item == 6U) ? 0U : item;
		uint8_t last = (item == 6U) ? 5U : item;
		uint8_t index;
		CanMonitorResetChannelName(item);
		for(index = first; index <= last && index < 6U; index++)
			SetTextValue(75U, (uint16_t)(index + 1U), (uint8_t *)"");
	}
}

/*! 
*  \brief  ???????
*  \details  ????????£?????§Õ????
*  \param screen_id ????ID
*  \param control_id ???ID
*  \param item ??????
*/
void NotifySelector(uint16 screen_id, uint16 control_id, uint8  item)
{

}


/*! 
*  \brief  ??????????????
*  \param screen_id ????ID
*  \param control_id ???ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id)
{
    if(screen_id==8&&control_id == 7)
    {
        SetBuzzer(100);
    } 
}


/*! 
*  \brief  ??????FLASH??????
*  \param status 0????1???
*  \param _data ????????
*  \param length ???????
*/
void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length)
{
    //TODO: ??????????
}


/*! 
*  \brief  §Õ???FLASH??????
*  \param status 0????1???
*/
void NotifyWriteFlash(uint8 status)
{
    //TODO: ??????????
}


/*! 
*  \brief  ???RTC???????????BCD??
*  \param year ??BCD??
*  \param month ?¡ê?BCD??
*  \param week ?????BCD??
*  \param day ???BCD??
*  \param hour ???BCD??
*  \param minute ???BCD??
*  \param second ??BCD??
*/
void NotifyReadRTC(uint8 year,uint8 month,uint8 week,uint8 day,uint8 hour,uint8 minute,uint8 second)
{

       
    secs    =(0xff & (second>>4))*10 +(0xf & second);                                    //BCD????????
    years   =(0xff & (year>>4))*10 +(0xf & year);                                      
    months  =(0xff & (month>>4))*10 +(0xf & month);                                     
    weeks   =(0xff & (week>>4))*10 +(0xf & week);                                      
    days    =(0xff & (day>>4))*10 +(0xf & day);                                      
    hours   =(0xff & (hour>>4))*10 +(0xf & hour);                                       
    minutes =(0xff & (minute>>4))*10 +(0xf & minute);  
//   	uart1_printf("???1?? %d??%d??%d??%d?%d??%d??\r\n",years,months,days,hours,minutes,secs);
//    SetTextInt32(8,1,years,1,1);
//    SetTextInt32(8,2,months,1,1);
//    SetTextInt32(8,3,days,1,1);
//    SetTextInt32(8,4,hours,1,1);
//    SetTextInt32(8,5,minutes,1,1);
//    SetTextInt32(8,6,sec,1,1);

}

// ????? ?????????????????
static void getDetectorSetUpLiveSum(DetectorSum *ds_entry, uint8_t cabin_setup[], uint8_t cluster_setup[])
{
	uint8_t detector_sum = 0;
	// ?????2?????????
	for(uint8_t sum = 1; sum < CANG_USER_NUM + 1; sum++)
	{
		detector_sum = detector_sum + cabin_setup[sum];
	}
			
	// ??????????
//	for(uint8_t sum = 1; sum <= 20; sum++)
//	{
//		detector_sum = detector_sum + cluster_setup[sum];
//	}
	
	for(uint8_t i = 1; i < 4; i++)
	{
		for(uint8_t j = 1; j < 33; j++)
		{
			detector_sum = detector_sum + pack_online_buff[i][j];
		}
	}
	
	// 
	for(uint8_t sum = 1U; sum <= MIXTURE_DEVICE_MAX_ADDR; sum++)
	{
		detector_sum += getPointTypeMixtureSettingOnlieState(sum);
	}
	
	ds_entry->curr_num = detector_sum; // ???õô???????
}
static uint8_t getPointDetectorSetUpCount(void)
{
	uint8_t set_up_sum = 0;
	for(uint8_t sum = 1U; sum <= MIXTURE_DEVICE_MAX_ADDR; sum++)
	{
		if(getPointTypeMixtureSettingOnlieState(sum) == 1)
		{
			set_up_sum++;
		}
	}
	return set_up_sum;
}

static uint8_t getPointDetectorSetUpLive(void)
{
	uint8_t detector_sum = 0;
	for(uint8_t sum = 1U; sum <= MIXTURE_DEVICE_MAX_ADDR; sum++)
	{
		if(getPointTypeMixtureSettingOnlieState(sum) == 1 && getPointTypeMixtureDetectName(sum) != 0U && getPointTypeMixtureDisconnectCount(sum) < MIXTURE_DEVICE_DISCONNECT_SUM)
		{
			detector_sum++;
		}
	}
	return detector_sum;
}
// ???????????????????????
static uint8_t getPointDetectorFaultCount(void)
{
    uint8_t fault_sum = 0;
    for(uint8_t sum = 1U; sum <= MIXTURE_DEVICE_MAX_ADDR; sum++)
    {
        if(getPointTypeMixtureSettingOnlieState(sum) == 1 && (getPointTypeMixtureDisconnectCount(sum) >= MIXTURE_DEVICE_DISCONNECT_SUM || getPointTypeMixtureStateClass(sum) == 3U))
        {
            fault_sum++;
        }
    }
    return fault_sum;
}
// ???????????????????????
static uint8_t getPointDetectorAlarmCount(void)
{
    uint8_t alarm_sum = 0;
    for(uint8_t sum = 1U; sum <= MIXTURE_DEVICE_MAX_ADDR; sum++)
    {
        if(getPointTypeMixtureStateClass(sum) == 1U || getPointTypeMixtureStateClass(sum) == 2U)
        {
            alarm_sum++;
        }
    }
    return alarm_sum;
}

uint8_t creatNewFaultRecordToCache(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id)
{
	uint8_t flag = 0;
	
	for(uint8_t k = 0; k < pcfs_buttom_point; k++)
	{
		if(cluster_id == LINKAGE_CLUSTER_ID) // ?????ID?????????õô???
		{
			// ??????????õô???ID?????? ???????????§Õ??????ID ?????
			if(pcfs[k].da.pack_id == pack_id) 
			{
				flag = 1;
				break;
			}
		}
		else if(cluster_id != 0)
		{
			if(pcfs[k].da.cluster_id == cluster_id && pcfs[k].da.pack_id == pack_id) 
			{
				flag = 1;
				break;
			}
		}
		else // ?????????????
		{
			if(pcfs[k].da.cabin_id == cabin_id)
			{
				flag = 1;
				break;
			}
		}
	}
	
	if(flag != 1) // ???????????
	{
		getBM8563TimeToSystemTime(); // ??????RTC???
		
		// ???????ID ????
		if(cluster_id == LINKAGE_CLUSTER_ID) // ?????ID?????????õô???
		{
			pcfs[pcfs_buttom_point].detector_class = LinkageClassID; // ???????????????????õô
			pcfs[pcfs_buttom_point].da.cabin_id    = cabin_id;       // ????????????????? 
			pcfs[pcfs_buttom_point].da.cluster_id  = cluster_id;
			pcfs[pcfs_buttom_point].da.pack_id     = pack_id;
		}
		else if(cluster_id != 0)
		{
			pcfs[pcfs_buttom_point].detector_class = PackClassID; // ???????????????
			pcfs[pcfs_buttom_point].da.cabin_id    = 0;           // ????ID??? ??????????????
			pcfs[pcfs_buttom_point].da.cluster_id  = cluster_id;
			pcfs[pcfs_buttom_point].da.pack_id     = pack_id;
		}
		else
		{
			pcfs[pcfs_buttom_point].detector_class = CabinClassID; // ???????????????
			pcfs[pcfs_buttom_point].da.cabin_id    = cabin_id; 
			pcfs[pcfs_buttom_point].da.cluster_id  = 0;
			pcfs[pcfs_buttom_point].da.pack_id     = 0;
		}

		// ??????????
		pcfs[pcfs_buttom_point].atr.years  = years + 2000;
		pcfs[pcfs_buttom_point].atr.months = months;
		pcfs[pcfs_buttom_point].atr.days   = days;
		pcfs[pcfs_buttom_point].atr.hours  = hours;
		pcfs[pcfs_buttom_point].atr.minute = minutes;
		
		// 2025/11/19 10:59 ?????????????
		pcfs[pcfs_buttom_point].atr.second = secs;
		pcfs[pcfs_buttom_point].fault_type = RS485_LOOP3_FAULT_OFFLINE;
		
		pcfs_buttom_point++; // ?????????
	}
	return flag;
}

uint8_t findRecoveryDevice(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id)
{
	uint8_t temp_index;
	
	for(temp_index = 0; temp_index < pcfs_buttom_point; temp_index++)
	{
		// XR5000_LOOP3_CHANGE_20260726: Recovery lookup must match the requested device class exactly.
		if(cluster_id == LINKAGE_CLUSTER_ID)
		{
			if(pcfs[temp_index].da.cluster_id == LINKAGE_CLUSTER_ID && pcfs[temp_index].da.pack_id == pack_id)
			{
				return temp_index;
			}
		}
		else if(cluster_id == RS485_DETECT_FLASH_ID)
		{
			if(pcfs[temp_index].da.cluster_id == RS485_DETECT_FLASH_ID && pcfs[temp_index].da.pack_id == pack_id)
			{
				return temp_index;
			}
		}
		else if(cluster_id != 0)
		{
			if(pcfs[temp_index].detector_class == PackClassID &&
			   pcfs[temp_index].da.cluster_id == cluster_id &&
			   pcfs[temp_index].da.pack_id == pack_id)
			{
				return temp_index;
			}
		}
		else
		{
			if(pcfs[temp_index].da.cluster_id == 0 && pcfs[temp_index].da.cabin_id == cabin_id)
			{
				return temp_index;
			}
		}
	}
	
	return 0xFF;
}

//
void deletRecoveryRecord(uint8_t recovery_index)
{
	uint8_t k;
	
	if(pcfs_buttom_point == 0)
		return;
	for(k = recovery_index; k < pcfs_buttom_point - 1; k++)
	{
		pcfs[k].detector_class = pcfs[k + 1].detector_class; // ???????????????
		pcfs[k].da             = pcfs[k + 1].da;
		pcfs[k].atr            = pcfs[k + 1].atr;
		pcfs[k].fault_type     = pcfs[k + 1].fault_type;
	}
	pcfs_buttom_point--;

}

// ????? ??????????
static uint8_t ClusterPackDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	uint8_t disconnect_detector_sum = 0;

	MaxCombustibleGas_t temp_pack_mcg_co = {0};
	temp_pack_mcg_co.co_max_val = -1;
	
	for(uint8_t jsz = 1;jsz <= 20;jsz++) // ????20??
	{
		if(cu_tcq_sxzt[jsz] == 0) // ?????????????????§Õ????????
		{
			continue;
		}
		for(uint8_t i=1;i<=cu_tcq_sxzt[jsz];i++)//????????????????????????? ?????????¦Ì??
		{
			if(getClusterPackDisconnectCount(jsz, i) == PackDisconnectCount) { // ???????????? ?§Ø???????
				// ???????? ?????§Û????????????
	
				disconnect_detector_sum++; // ?????õô+1
				// ????§Õ0????????
				if(creatNewFaultRecordToCache(jsz, i, 0) == 0) // ???????0??????§Õ?? ???????????? ???›¥FLASH
				{
					beep_fault_ctrl  = 2;   // ???????? ????????????¦Ë
					silencers_state  = 0;   // ???????
					disconnect_state = 1;   // ?????????
					// ?????????›¥
					//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
					BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, jsz, i);
				}
			}else if(PACK_zx_buf[jsz][i] == i) { //??????????????ID??? ???? ??????§Ø?
				// new
				if(pack_pbzt[jsz][i]==0 && PACK_WDZT_buf[jsz][i] != 0 && BJ_packjiyibuf_wd[jsz][i] == 0)
				{
					// new
					// ??????????????
					getBM8563TimeToSystemTime(); // ??????RTC??? 
					// ????????????????????????
					StoragePackFireAlarm(&pcfas, jsz, i, Temperature);
					
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, jsz, i, PACK_wendu_buf[jsz][i]);
					// ???¡ä›¥???? ??????
					BJ_packjiyibuf_wd[jsz][i] = PACK_WDZT_buf[jsz][i];
					// end
				}
				else if(PACK_WDZT_buf[jsz][i] == 0 && BJ_packjiyibuf_wd[jsz][i] != 0)
				{
					BJ_packjiyibuf_wd[jsz][i]=0;
				}
					
				if(pack_pbzt[jsz][i] == 0 && PACK_YWZT_buf[jsz][i] != 0 && BJ_packjiyibuf_yw[jsz][i] == 0) 
				{
					// new
					getBM8563TimeToSystemTime(); // ??????RTC???
					StoragePackFireAlarm(&pcfas, jsz, i, Smoke);
					// end
					
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, SMOKE_ALARM, jsz, i, 0xFFFF);
					BJ_packjiyibuf_yw[jsz][i] = PACK_YWZT_buf[jsz][i];
				}
				else if(PACK_YWZT_buf[jsz][i] == 0 && BJ_packjiyibuf_yw[jsz][i] != 0)
				{
					BJ_packjiyibuf_yw[jsz][i]=0;
				}
					
				if(pack_pbzt[jsz][i] == 0 && PACK_COZT_buf[jsz][i] !=0 && BJ_packjiyibuf_co[jsz][i] == 0) {

					// new
					getBM8563TimeToSystemTime(); // ??????RTC???
					StoragePackCabinForeWarn(&pcfws, jsz, i, Carbon);
					// end
					// ?????›¥??? ??????????ä¼??FLASH??
					BJ_packjiyibuf_co[jsz][i] = PACK_COZT_buf[jsz][i];
					
					// ??????????
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM, jsz, i, getPackCoConcenValue(jsz, i));
				}
				else if(BJ_packjiyibuf_co[jsz][i] != 0 && PACK_COZT_buf[jsz][i] == 0) { // ??????›¥??
					// ??????????????????????????????
					// NEW
					// 2025/10/11 10:35 ????????????????????????????? ????????????
//					DeletPackCabinForeWarn(&pcfws, jsz, i, Carbon);
//					// 2025/9/2 17:13
//					getBM8563TimeToSystemTime(); // ??????RTC???
//					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, EAR_RECOVERY, jsz, i, getPackCoConcenValue(jsz, i));

					// END
					BJ_packjiyibuf_co[jsz][i]=0;
				}			
					
				if(PACK_CH4ZT_buf[jsz][i]!=0)
				{
					getBM8563TimeToSystemTime(); // ??????RTC???
				}
				else
				{
					BJ_packjiyibuf_ch4[jsz][i]=0;
				}
				
				if( getPackCoConcenValue(jsz, i) > temp_pack_mcg_co.co_max_val)
				{
					temp_pack_mcg_co.co_max_val = getPackCoConcenValue(jsz, i);
					temp_pack_mcg_co.gas_type = Hydrogen_Type;
					temp_pack_mcg_co.curr_da.cluster_id = jsz;
					temp_pack_mcg_co.curr_da.pack_id    = i;
					temp_pack_mcg_co.curr_da.cabin_id   = 0;
				}
				
			}
			// end
				
			if((( BJ_packjiyibuf_ch4[jsz][i] != 0 || 
						BJ_packjiyibuf_co[jsz][i] != 0  || 
						BJ_packjiyibuf_yw[jsz][i] != 0) && 
						BJ_packjiyibuf_wd[jsz][i] != 0) 
				//		|| BJ_packjiyibuf_wd[jsz][i] == 2 // ??????¦Å???????????????? ?????????
				)
			{
				// ??????§Ø?
				uint8_t flag = 0;
				for(uint8_t j = 0;j < pas_pointer; j++)
				{
					if(pas[j].cabin_id == 0) // ?????ID????0 ??????¦Ë??›¥??????
					{
						if(pas[j].cluster_id == jsz	 && pas[j].pack_id == i)
						{
							flag = 1;
							break; // ???????????????›¥???? ???????
						}
					}
				}
				if(flag != 1) // ?????§Õ›¥??
				{
					getBM8563TimeToSystemTime(); // ??????RTC???
					// ???????
					fire_alarm_flag.cluster_alarm_state = 1;
					
					pas[pas_pointer].cluster_id  = jsz;
					pas[pas_pointer].pack_id     = i;
					pas[pas_pointer].cabin_id    = 0;
					pas[pas_pointer].lunch_state = 0; // ????? ¦Ä???	
					// ??P?
					pas[pas_pointer].atr.years  = years + 2000;
					pas[pas_pointer].atr.months = months;
					pas[pas_pointer].atr.days   = days;
					pas[pas_pointer].atr.hours  = hours;
					pas[pas_pointer].atr.minute = minutes;
					
					// 2025/11/19 10:59 ?????????????
					pas[pas_pointer].atr.second = secs;
					
					pas_pointer++;
				}
			}			
		}
	}
	
	if(*pcfs_point > 0)
	{
		disconnect_state = 1; // ?????????
	}
	
	if(disconnect_detector_sum < *pcfs_point) // ????????õô??§³????????? ??????õô?????
	{
		uint8_t flag = 0; 
		uint8_t k;
		
		for(k = 0; k < *pcfs_point; k++)
		{
			if(pcfs_entry[k].da.cluster_id == LINKAGE_CLUSTER_ID ||
			   pcfs_entry[k].da.cluster_id == 0 ||
			   pcfs_entry[k].da.cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 recovery is handled by RS485DetectDataDeal().
				// ??????????õô??? ????????
				continue;
			}
			// ???????????
			if(getClusterPackDisconnectCount(pcfs_entry[k].da.cluster_id, pcfs_entry[k].da.pack_id) != PackDisconnectCount) 
			{
				flag = 1;
				break;
			}
		}
		if(flag == 1)
		{
			deletRecoveryRecord(k);
			// ?›¥??FLASH
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, pcfs_entry[k].da.cluster_id, pcfs_entry[k].da.pack_id);
			if(*pcfs_point > 0) // ??????§Ò??????????????
			{
				beep_fault_ctrl  = 2; // ???????? ????????????¦Ë
				silencers_state  = 0; // ???????
				disconnect_state = 1; // ?????????
			}		
		}
	}
	
	mcg[PACK_CO_ID] = temp_pack_mcg_co;
	
	return disconnect_detector_sum;
}

// ????? ??????????
static uint8_t ClusterPackDataDeal_Plus(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	uint8_t disconnect_detector_sum = 0;

	MaxCombustibleGas_t temp_pack_mcg_co = {0};
	temp_pack_mcg_co.co_max_val = -1;
	
	for(uint8_t jsz = 1;jsz < 4; jsz++) // ????20??
	{
		for(uint8_t i = 1; i < 33; i++)//????????????????????????? ?????????¦Ì??
		{
			if(pack_online_buff[jsz][i] == 0) // ????????¦Ä????
			{
				continue;
			}
			if(getClusterPackDisconnectCount(jsz, i) == PackDisconnectCount) // ???????????? ?§Ø???????
			{ 
				// ???????? ?????§Û????????????
	
				disconnect_detector_sum++; // ?????õô+1
				// ????§Õ0????????
				if(creatNewFaultRecordToCache(jsz, i, 0) == 0) // ???????0??????§Õ?? ???????????? ???›¥FLASH
				{
					beep_fault_ctrl  = 2;   // ???????? ????????????¦Ë
					silencers_state  = 0;   // ???????
					disconnect_state = 1;   // ?????????
					// ?????????›¥
					//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
					BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, jsz, i);
				}
			}
			else if(PACK_zx_buf[jsz][i] == i)  //??????????????ID??? ???? ??????§Ø?
			{
				// new
				if(pack_pbzt[jsz][i]==0 && PACK_WDZT_buf[jsz][i] != 0 && BJ_packjiyibuf_wd[jsz][i] == 0)
				{
					// new
					// ??????????????
					getBM8563TimeToSystemTime(); // ??????RTC??? 
					// ????????????????????????
					StoragePackFireAlarm(&pcfas, jsz, i, Temperature);
					
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, jsz, i, PACK_wendu_buf[jsz][i]);
					// ???¡ä›¥???? ??????
					BJ_packjiyibuf_wd[jsz][i] = PACK_WDZT_buf[jsz][i];
					// end
				}
				else if(PACK_WDZT_buf[jsz][i] == 0 && BJ_packjiyibuf_wd[jsz][i] != 0)
				{
					BJ_packjiyibuf_wd[jsz][i]=0;
				}
					
				if(pack_pbzt[jsz][i] == 0 && PACK_YWZT_buf[jsz][i] != 0 && BJ_packjiyibuf_yw[jsz][i] == 0) 
				{
					// new
					getBM8563TimeToSystemTime(); // ??????RTC???
					StoragePackFireAlarm(&pcfas, jsz, i, Smoke);
					// end
					
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, SMOKE_ALARM, jsz, i, 0xFFFF);
					BJ_packjiyibuf_yw[jsz][i] = PACK_YWZT_buf[jsz][i];
				}
				else if(PACK_YWZT_buf[jsz][i] == 0 && BJ_packjiyibuf_yw[jsz][i] != 0)
				{
					BJ_packjiyibuf_yw[jsz][i]=0;
				}
					
				if(pack_pbzt[jsz][i] == 0 && PACK_COZT_buf[jsz][i] !=0 && BJ_packjiyibuf_co[jsz][i] == 0) {

					// new
					getBM8563TimeToSystemTime(); // ??????RTC???
					StoragePackCabinForeWarn(&pcfws, jsz, i, Carbon);
					// end
					// ?????›¥??? ??????????ä¼??FLASH??
					BJ_packjiyibuf_co[jsz][i] = PACK_COZT_buf[jsz][i];
					
					// ??????????
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM, jsz, i, getPackCoConcenValue(jsz, i));
				}
				else if(BJ_packjiyibuf_co[jsz][i] != 0 && PACK_COZT_buf[jsz][i] == 0) { // ??????›¥??
					// ??????????????????????????????
					// NEW
					// 2025/10/11 10:35 ????????????????????????????? ????????????
//					DeletPackCabinForeWarn(&pcfws, jsz, i, Carbon);
//					// 2025/9/2 17:13
//					getBM8563TimeToSystemTime(); // ??????RTC???
//					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, EAR_RECOVERY, jsz, i, getPackCoConcenValue(jsz, i));

					// END
					BJ_packjiyibuf_co[jsz][i]=0;
				}			
					
				if(PACK_CH4ZT_buf[jsz][i]!=0)
				{
					getBM8563TimeToSystemTime(); // ??????RTC???
				}
				else
				{
					BJ_packjiyibuf_ch4[jsz][i]=0;
				}
				
				if( getPackCoConcenValue(jsz, i) > temp_pack_mcg_co.co_max_val)
				{
					temp_pack_mcg_co.co_max_val = getPackCoConcenValue(jsz, i);
					temp_pack_mcg_co.gas_type = Hydrogen_Type;
					temp_pack_mcg_co.curr_da.cluster_id = jsz;
					temp_pack_mcg_co.curr_da.pack_id    = i;
					temp_pack_mcg_co.curr_da.cabin_id   = 0;
				}
				
			}
			// end
				
			if((( BJ_packjiyibuf_ch4[jsz][i] != 0 || 
						BJ_packjiyibuf_co[jsz][i] != 0  || 
						BJ_packjiyibuf_yw[jsz][i] != 0) && 
						BJ_packjiyibuf_wd[jsz][i] != 0) 
				//		|| BJ_packjiyibuf_wd[jsz][i] == 2 // ??????¦Å???????????????? ?????????
				)
			{
				// ??????§Ø?
				uint8_t flag = 0;
				for(uint8_t j = 0;j < pas_pointer; j++)
				{
					if(pas[j].cabin_id == 0) // ?????ID????0 ??????¦Ë??›¥??????
					{
						if(pas[j].cluster_id == jsz	 && pas[j].pack_id == i)
						{
							flag = 1;
							break; // ???????????????›¥???? ???????
						}
					}
				}
				if(flag != 1) // ?????§Õ›¥??
				{
					getBM8563TimeToSystemTime(); // ??????RTC???
					// ???????
					fire_alarm_flag.cluster_alarm_state = 1;
					
					pas[pas_pointer].cluster_id  = jsz;
					pas[pas_pointer].pack_id     = i;
					pas[pas_pointer].cabin_id    = 0;
					pas[pas_pointer].lunch_state = 0; // ????? ¦Ä???	
					// ??P?
					pas[pas_pointer].atr.years  = years + 2000;
					pas[pas_pointer].atr.months = months;
					pas[pas_pointer].atr.days   = days;
					pas[pas_pointer].atr.hours  = hours;
					pas[pas_pointer].atr.minute = minutes;
					
					// 2025/11/19 10:59 ?????????????
					pas[pas_pointer].atr.second = secs;
					
					pas_pointer++;
				}
			}			
		}
	}
	
	if(*pcfs_point > 0)
	{
		disconnect_state = 1; // ?????????
	}
	
	if(disconnect_detector_sum < *pcfs_point) // ????????õô??§³????????? ??????õô?????
	{
		uint8_t flag = 0; 
		uint8_t k;
		
		for(k = 0; k < *pcfs_point; k++)
		{
			if(pcfs_entry[k].da.cluster_id == LINKAGE_CLUSTER_ID ||
			   pcfs_entry[k].da.cluster_id == 0 ||
			   pcfs_entry[k].da.cluster_id == RS485_DETECT_FLASH_ID ||
			   pcfs_entry[k].da.cluster_id == MBUS_CONTROL_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 recovery is handled by RS485DetectDataDeal().
				// Loop 2 recovery is handled by MBus2DataDeal().
				continue;
			}
			// ???????????
			if(getClusterPackDisconnectCount(pcfs_entry[k].da.cluster_id, pcfs_entry[k].da.pack_id) != PackDisconnectCount) 
			{
				flag = 1;
				break;
			}
		}
		if(flag == 1)
		{
			uint8_t saved_cluster_id = pcfs_entry[k].da.cluster_id;
			uint8_t saved_pack_id    = pcfs_entry[k].da.pack_id;
			deletRecoveryRecord(k);
			// ?›¥??FLASH
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, saved_cluster_id, saved_pack_id);
			if(*pcfs_point > 0) // ??????§Ò??????????????
			{
				beep_fault_ctrl  = 2; // ???????? ????????????¦Ë
				silencers_state  = 0; // ???????
				disconnect_state = 1; // ?????????
			}		
		}
	}
	
	mcg[PACK_CO_ID] = temp_pack_mcg_co;
	
	return disconnect_detector_sum;
}

static uint8_t CabinDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	uint8_t temp_cabin_disconnect_sum = 0;
	
	MaxCombustibleGas_t temp_mcg_co = {0};
	MaxCombustibleGas_t temp_mcg_hh = {0};
	
	temp_mcg_co.co_max_val = -1;
	temp_mcg_hh.co_max_val = -1;
	
	//????§Ø? 24 ???? ??????? ??????
	for(uint8_t jsz=1; jsz <= 24; jsz++) // ??????????????????????????????????24
	{
		if(cang_sxzt[jsz] != 1) // ?????????????? ????§Ø??????
		{
			continue;
		}
		else if(Cang_zx_buf[jsz] == CabinDisconnectCount) // ????????????? ??????+1 ?§Ø??????
		{
			temp_cabin_disconnect_sum++; // ????????????	

			if(DX_cangjiyibuf[jsz] == 0)
			{
				DX_cangjiyibuf[jsz] = 1;	
				if( creatNewFaultRecordToCache(0, 0, jsz) == 0 ) // ???§Õ????
				{
					beep_fault_ctrl  = 2;   // ???????? ????????????¦Ë
					silencers_state  = 0;   // ???????
					disconnect_state = 1;   // ?????????
					// ?????????›¥
					//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
					BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, 0, jsz); // ?›¥?????
				}
			}
			continue;
		}
		
		if(DX_cangjiyibuf[jsz] == 1)
		{
			uint8_t index = 0xFF;
			index = findRecoveryDevice(0, 0, jsz);
			if(index != 0xFF)
			{
				deletRecoveryRecord(index);
				// ?›¥??FLASH
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, 0, jsz);
				if(*pcfs_point > 0) // ??????§Ò??????????????
				{
					beep_fault_ctrl  = 2; // ???????? ????????????¦Ë
					silencers_state  = 0; // ???????
					disconnect_state = 1; // ?????????
				}		
			}
		}
		
		DX_cangjiyibuf[jsz] = 0;

		//??????§Ø?
		if(Cang_WDZT_buf[jsz] != 0 && BJ_cangjiyibuf_wd[jsz] == 0) 
		{
			// new
			// ??????????????
			getBM8563TimeToSystemTime(); // ??????RTC??? 
			// ????????????????????????
			StoragePackFireAlarm(&pcfas, 0, jsz, Temperature);
			
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, 0, jsz, Cang_wendu_buf[jsz]);
			// ???¡ä›¥???? ??????
			BJ_cangjiyibuf_wd[jsz] = Cang_WDZT_buf[jsz];
			// end

			// ????????
			cabin_detector_state_buff[jsz].temperature_state = 1; // ??????
			// end
		}

		//???????§Ø?
		if(Cang_YWZT_buf[jsz] != 0 && BJ_cangjiyibuf_yw[jsz] == 0) { 
			getBM8563TimeToSystemTime(); // ??????RTC???

			// new
			getBM8563TimeToSystemTime(); // ??????RTC???
			StoragePackFireAlarm(&pcfas, 0, jsz, Smoke);
			// end
			
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, SMOKE_ALARM, 0, jsz, 0xFFFF);
			
			BJ_cangjiyibuf_yw[jsz] = Cang_YWZT_buf[jsz];
			// ????????
			cabin_detector_state_buff[jsz].smoke_state = 1; // ??????????
			// end
		}

		//?????????§Ø?
		if(Cang_COZT_buf[jsz] != 0 && BJ_cangjiyibuf_co[jsz] == 0) 
		{ 
			// new
			getBM8563TimeToSystemTime(); // ??????RTC???
			StoragePackCabinForeWarn(&pcfws, 0, jsz, Carbon);
			// end

			// ??????????
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_CO, 0, jsz, Cang_COzhi_buf[jsz]);
			
			BJ_cangjiyibuf_co[jsz] = Cang_COZT_buf[jsz];
			
			// ????????
			cabin_detector_state_buff[jsz].carbon_state = Cang_COZT_buf[jsz]; // ????????????
			
			// end
		}
		else if(Cang_COZT_buf[jsz] == 0 && BJ_cangjiyibuf_co[jsz] != 0)
		{
			BJ_cangjiyibuf_co[jsz] = 0;
			// ????????
			cabin_detector_state_buff[jsz].carbon_state = 0;      // ?????????
		}
			
//			if(Cang_CH4ZT_buf[jsz]==1 && cang_pbzt[jsz]==0)//???????????
//			{
//				if(BJ_cangjiyibuf_ch4[jsz]!=1)
//				{
//					BJ_cangjiyibuf_ch4[jsz]=1;
//					SaveSensor(jsz,1,Cang_wendu_buf[jsz]+40,Cang_YWZT_buf[jsz],Cang_COZT_buf[jsz],Cang_CH4ZT_buf[jsz],0,0,0);
//				}
//			}else if(Cang_CH4ZT_buf[jsz]==2 && cang_pbzt[jsz]==0) { //???????????
//				if(BJ_cangjiyibuf_ch4[jsz]!=2) {
//					BJ_cangjiyibuf_ch4[jsz]=2;
//					SaveSensor(jsz,1,Cang_wendu_buf[jsz]+40,Cang_YWZT_buf[jsz],Cang_COZT_buf[jsz],Cang_CH4ZT_buf[jsz],0,0,0);
//				}
//			}
			
			
		//??VOC?§Ø?
		if(Cang_VOCZT_buf[jsz] != 0 && BJ_cangjiyibuf_voc[jsz] == 0)
		{ 

			BJ_cangjiyibuf_voc[jsz] = Cang_VOCZT_buf[jsz];
		}
		else if(Cang_VOCZT_buf[jsz] == 0 && BJ_cangjiyibuf_voc[jsz] != 0)
		{
			
			BJ_cangjiyibuf_voc[jsz]=0;
			// ????????
		}
		
		//??H2?§Ø?
		if(Cang_H2ZT_buf[jsz] != 0 && BJ_cangjiyibuf_h2[jsz] == 0) 
		{ 
			// new
			getBM8563TimeToSystemTime(); // ??????RTC???
			StoragePackCabinForeWarn(&pcfws, 0, jsz, Hydrogen);
			// end

			// ??????????
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_HH, 0, jsz, Cang_H2zhi_buf[jsz]);
			
			BJ_cangjiyibuf_h2[jsz] = Cang_H2ZT_buf[jsz];
			
			// ????????
			cabin_detector_state_buff[jsz].hydrogen_state = 1; // ??????????
			
			// end
			
		}
		else if(Cang_H2ZT_buf[jsz] == 0 && BJ_cangjiyibuf_h2[jsz] != 0) 
		{
			
			// ????????
			cabin_detector_state_buff[jsz].hydrogen_state = 0;    // ?????????
			BJ_cangjiyibuf_h2[jsz] = 0;
		}

		// end

		// 2025/10/27 16:42 ???????????????§Ø?
		
		if(Cang_H2zhi_buf[jsz] > temp_mcg_hh.co_max_val)
		{
			temp_mcg_hh.co_max_val = Cang_H2zhi_buf[jsz];
			
			temp_mcg_hh.gas_type = Hydrogen_Type;
			
			temp_mcg_hh.curr_da.cabin_id   = jsz;
			// ?????? ???????pack??0
			temp_mcg_hh.curr_da.cluster_id = 0;
			temp_mcg_hh.curr_da.pack_id    = 0;
		}
		
		if(Cang_COzhi_buf[jsz] > temp_mcg_co.co_max_val)
		{
			temp_mcg_co.co_max_val = Cang_COzhi_buf[jsz];
			
			temp_mcg_co.gas_type = Carbon_Type;
			
			temp_mcg_co.curr_da.cabin_id   = jsz;
			// ?????? ???????pack??0
			temp_mcg_co.curr_da.cluster_id = 0;
			temp_mcg_co.curr_da.pack_id    = 0;
		}
		
		
		// ???????????
		//???????§Ø?
		if( // Cang_WDZT_buf[jsz]==2 || 
			(((BJ_cangjiyibuf_h2[jsz] != 0) || BJ_cangjiyibuf_voc[jsz] != 0 || BJ_cangjiyibuf_co[jsz] != 0 || BJ_cangjiyibuf_yw[jsz] != 0) 
			&& BJ_cangjiyibuf_wd[jsz] != 0 ) )
		{
			
			// new
			fire_alarm_state = 1;  // ????(?????????)
			// end
			
			// ??????§Ö???????????
			uint8_t flag = 0;
			for(uint8_t k = 0;k < pas_pointer; k++)
			{
				if(pas[k].cluster_id == 0 && pas[k].pack_id == 0) // ?????ID???ID????0 ??????¦Ë?????????ID
				{
					if(pas[k].cabin_id == jsz) // ?????ID????›¥??
					{
						flag = 1;
						break; // ???????????????›¥???? ???????
					}
				}
				
			}
			if(flag != 1) // ?????§Õ›¥??
			{
				getBM8563TimeToSystemTime(); // ??????RTC???
				// ???????
				fire_alarm_flag.cabin_alarm_state = 1;
				
				pas[pas_pointer].cabin_id    = jsz;
				pas[pas_pointer].cluster_id  = 0;
				pas[pas_pointer].pack_id     = 0;
				pas[pas_pointer].lunch_state = 0;
				// ??P?
				pas[pas_pointer].atr.years  = years + 2000;
				pas[pas_pointer].atr.months = months;
				pas[pas_pointer].atr.days   = days;
				pas[pas_pointer].atr.hours  = hours;
				pas[pas_pointer].atr.minute = minutes;
				
			}
			// end
		}

	}
	
	mcg[CABIN_CO_ID] = temp_mcg_co;
	mcg[CABIN_HH_ID] = temp_mcg_hh;
	
	return temp_cabin_disconnect_sum; // ????????????
}

static void FaultRelayCtrlAppFun(uint8_t disconnect_num)
{
	if(disconnect_num != 0) // ??????????????0 ??????????
	{
		if(rcsr[FaultRelayId].curr_relay_state == JDQ_OFF)
		{
			rcsr[FaultRelayId].call_back_fun(JDQ_ON);
			rcsr[FaultRelayId].curr_relay_state = JDQ_ON;
		}
	}
	else
	{
		if(rcsr[FaultRelayId].curr_relay_state == JDQ_ON)
		{
			rcsr[FaultRelayId].call_back_fun(JDQ_OFF);
			rcsr[FaultRelayId].curr_relay_state = JDQ_OFF;
		}
	}
}

static void ForeWarmRelayCtrlAppFun(PackCabinForeWarnStorage *pcfws_entry)
{
	if(pcfws_entry->self_bottom_point > 0)
	{
		if(rcsr[ForeWarmRelayId].curr_relay_state == JDQ_OFF)
		{
			rcsr[ForeWarmRelayId].call_back_fun(JDQ_ON);
			rcsr[ForeWarmRelayId].curr_relay_state = JDQ_ON;
		}
	}
	else
	{
		if(rcsr[ForeWarmRelayId].curr_relay_state == JDQ_ON)
		{
			rcsr[ForeWarmRelayId].call_back_fun(JDQ_OFF);
			rcsr[ForeWarmRelayId].curr_relay_state = JDQ_OFF;
		}
	}
}

static void FireAlarmRelayCtrlAppFun(uint8_t pas_alarm_num)
{
	if(pas_alarm_num != 0)
	{
		if(rcsr[FireAlarmRelayId].curr_relay_state == JDQ_OFF)
		{
			rcsr[FireAlarmRelayId].call_back_fun(JDQ_ON);
			rcsr[FireAlarmRelayId].curr_relay_state = JDQ_ON;
		}
	}
	else
	{
		if(rcsr[FireAlarmRelayId].curr_relay_state == JDQ_ON)
		{
			rcsr[FireAlarmRelayId].call_back_fun(JDQ_OFF);
			rcsr[FireAlarmRelayId].curr_relay_state = JDQ_OFF;
		}
	}
}

const uint8_t monitor_inform_screen_id = 59;

static void InternalScreenShowAllFault(uint8_t fresh_page_flag)
{
	static uint16_t last_product_unknown_count = 0U;
	uint16_t product_unknown_count = DeviceRegistry_GetProductUnknownCount();
	uint16_t total_fault_count = (uint16_t)pcfs_buttom_point + product_unknown_count;
	// ?????????
	if(total_fault_count == 0U)
	{
		if(pcfs_fresh_ctrl != 0 || last_product_unknown_count != 0U)
		{
			disconnect_state = 0;  // ????????? 
			beep_fault_ctrl  = 0;  // ???????????
			pcfs_fresh_ctrl = 0;
			last_product_unknown_count = 0U;
			clearTextValue(monitor_inform_screen_id , 43);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 44);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 45);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 46);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 47);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 48);//(????ID,???ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 42,"????????????.     ");break;   //??¡À???????
			case 20:SetTextValue(monitor_inform_screen_id, 42,"????????????..    ");break;   //??¡À???????
			case 30:SetTextValue(monitor_inform_screen_id, 42,"????????????...   ");break;   //??¡À???????
			case 40:SetTextValue(monitor_inform_screen_id, 42,"????????????....  ");break;   //??¡À???????
			case 50:SetTextValue(monitor_inform_screen_id, 42,"????????????..... ");break;   //??¡À???????
			case 60:SetTextValue(monitor_inform_screen_id, 42,"????????????......");break;   //??¡À???????
		}
	}
	else if(pcfs_fresh_ctrl != pcfs_buttom_point || last_product_unknown_count != product_unknown_count || fresh_page_flag == 1)
	{
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.
		pcfs_fresh_ctrl = pcfs_buttom_point;
		last_product_unknown_count = product_unknown_count;
		
		uint8_t temp_sequence_count = 0;
		
		for (uint8_t i = 0; i < Fault_Show_Zone; i++) {
			uint16_t data_index = (uint16_t)fault_current_page + i;
			temp_sequence_count = data_index + 1;
			if(data_index < total_fault_count) {
				if(data_index >= pcfs_buttom_point)
				{
					uint8_t unknown_loop = 0U;
					uint8_t unknown_addr = 0U;
                    DeviceIdentifyError identify_error = DEVICE_IDENTIFY_OK;
                    static const uint8_t unknown_format[] = {0xB5U,0xDAU,'%','d',0xBBU,0xD8U,0xC2U,0xB7U,'%','0','2','d',0xBAU,0xC5U,' ','%','s',0U};
					if(DeviceRegistry_GetIdentifyErrorAt((uint16_t)(data_index - pcfs_buttom_point), &unknown_loop, &unknown_addr, &identify_error) != 0U)
                        sprintf((char*)baojingneirong, (const char*)unknown_format, unknown_loop, unknown_addr, DeviceRegistry_GetIdentifyErrorText(identify_error));
					else baojingneirong[0] = 0;
				}
				else if(FormatLoop1FaultLine(baojingneirong, temp_sequence_count, pcfs, data_index) == 1)
				{
				}
				else if(FormatRS485DetectFaultLine(baojingneirong, temp_sequence_count, pcfs, data_index) == 1)
				{
					// XR5000_LOOP3_CHANGE_20260726: Loop 3 fault display uses "??3??¡¤ X??".
				}
				else if(FormatMBus2FaultLine(baojingneirong, temp_sequence_count, pcfs, data_index) == 1)
				{
					// Loop 2 fault display uses "??¡¤?? XX?õô????".
				}
				else if(pcfs[data_index].detector_class == PackClassID) // ??????????
				{
					// 2025/11/19 10:59 ?????????????
                    static const uint8_t pack_format[] = {'%','0','3','d',' ','%','d','/','%','0','2','d','/','%','0','2','d',' ','%','0','2','d',':','%','0','2','d',':','%','0','2','d',' ',0xB5U,0xDAU,'%','d',0xB4U,0xD8U,' ','P','A','C','K','%','d',' ',0xB5U,0xF4U,0xCFU,0xDFU,0U};
                    sprintf((char*)baojingneirong, (const char*)pack_format, temp_sequence_count,
						pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
						pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second,
						pcfs[data_index].da.cluster_id, pcfs[data_index].da.pack_id);
				}
				else if(pcfs[data_index].detector_class == CabinClassID) // ??????????
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????", temp_sequence_count, // ??????????
						pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
						pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second,
						pcfs[data_index].da.cabin_id );
				}
				else if(pcfs[data_index].detector_class == LinkageClassID) // ??????????õô
				{
					switch(pcfs[data_index].da.pack_id)
					{
						
						case Deflate_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ???????????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days, 
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ?????????¡¤", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days, 
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case SoundLt_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????????????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????????¡¤", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case SirenBk_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ???????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ?????¡¤", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case OutFir1_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????1????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????1??¡¤", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case OutFir2_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????2????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????2??¡¤", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case CabinBK_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ?????????", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ???????¡¤", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case FEEDBK1_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????1????", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????1??¡¤", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case FEEDBK2_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????2????", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????2??¡¤", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case HANDPOT_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ???????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ?????¡¤", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case SYS_FLASH_FAULT_ID: { // ?????›¥????
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ???›¥????", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);

							break;
						}							
						case SYS_MAIN_POWER_KEY_ID : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ???????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case SYS_BACK_POWER_KEY_ID : {
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ???????", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ?????¡¤", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_1 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??¡¤1??¡¤", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_2 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??¡¤2??¡¤", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_3 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??¡¤3??¡¤", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_4 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??¡¤4??¡¤", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
					}
				}
			} 
			else {
				baojingneirong[0] = 0;
				//clearTextValue(1 , 41 + i);//(????ID,???ID??
			}
			SetTextValue(monitor_inform_screen_id, i + 42, baojingneirong);//??¡À???????
		}
	}
	
//	/* DEBUG USE */
//	if(pcfs_buttom_point > 0)
//	{
//		uint8_t test_buff[64] = {0};

//		sprintf((char*)test_buff, "%d/%02d/%02d %02d:%02d:%02d cab:%d clu:%d pac:%d cls:%d\r\n", pcfs[0].atr.years, pcfs[0].atr.months, pcfs[0].atr.days,
//									pcfs[0].atr.hours, pcfs[0].atr.minute, pcfs[0].atr.second, pcfs[0].da.cabin_id, pcfs[0].da.cluster_id, pcfs[0].da.pack_id, pcfs[0].detector_class);
//		DebugSendString(test_buff, sizeof(test_buff));
//	}

}	

static void InternalScreenShowAllForceWorn(PackCabinForeWarnStorage *pcfws_entry, uint8_t fresh_page_flag)
{
	// ??????
	if(pcfws_entry->self_bottom_point == 0) // ????????????0 
	{
		if(pcfws_entry->point_history_len != 0)
		{
			pcfws_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 36);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 37);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 38);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 39);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 40);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 41);//(????ID,???ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 35,"???????????.     ");break;   //??¡À???????
			case 20:SetTextValue(monitor_inform_screen_id, 35,"???????????..    ");break;   //??¡À???????
			case 30:SetTextValue(monitor_inform_screen_id, 35,"???????????...   ");break;   //??¡À???????
			case 40:SetTextValue(monitor_inform_screen_id, 35,"???????????....  ");break;   //??¡À???????
			case 50:SetTextValue(monitor_inform_screen_id, 35,"???????????..... ");break;   //??¡À???????
			case 60:SetTextValue(monitor_inform_screen_id, 35,"???????????......");break;   //??¡À???????
		}
	}
	else if(pcfws_entry->self_bottom_point != pcfws_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfws_entry->point_history_len = pcfws_entry->self_bottom_point;
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.
		

		
		// ??????????????????
		if(pcfws_entry->detector_class[0] == PackClassID && pcfws_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first warning display uses "??3??¡¤ X??".
			FormatRS485DetectForeWarnLine(baojingneirong, 1, pcfws_entry, 0);
		}
		else if(pcfws_entry->detector_class[0] == PackClassID)
		{
			if(pcfws_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cluster_id, pcfws.da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
		}
		else if(pcfws_entry->detector_class[0] == LinkageClassID) // ??????????õô
		{
			if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????????", 1,
					pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
					pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second);
			}
		}
		else if(pcfws_entry->detector_class[0] == CabinClassID)
		{
			if(pcfws_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
		}
		
		SetTextValue(monitor_inform_screen_id, 35, baojingneirong); // ??¦Ì????????????
			
		uint8_t temp_sequence_count = 0;
		
		// ??¦Ì???????????
		for (uint8_t i = 1; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fore_alarm_start_index + i; // ???????
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfws_entry->self_bottom_point &&
				pcfws_entry->detector_class[data_index] == PackClassID &&
				pcfws_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 warning display uses "??3??¡¤ X??".
				FormatRS485DetectForeWarnLine(baojingneirong, temp_sequence_count, pcfws_entry, data_index);
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == PackClassID)
			{
				if(pcfws_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == LinkageClassID)
			{
				if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????????", temp_sequence_count,
						pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
						pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[data_index].second);
				}
				else if(pcfws_entry->alarm_type[data_index] == HandAlarm)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????¡À???", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[0].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second);
				}
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == CabinClassID)
			{
				if(pcfws_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 35 + i); //(????ID,???ID??
			}
			SetTextValue(monitor_inform_screen_id, 35 + i, baojingneirong); // ??¡À???????
		}
	}
}

static void InternalScreenShowAllForceWorn_Plus(PackCabinForeWarnStorage *pcfws_entry, uint8_t fresh_page_flag)
{
	// ??????
	if(pcfws_entry->self_bottom_point == 0) // ????????????0 
	{
		if(pcfws_entry->point_history_len != 0)
		{
			pcfws_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 36);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 37);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 38);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 39);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 40);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 41);//(????ID,???ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 35,"???????????.     ");break;   //??¡À???????
			case 20:SetTextValue(monitor_inform_screen_id, 35,"???????????..    ");break;   //??¡À???????
			case 30:SetTextValue(monitor_inform_screen_id, 35,"???????????...   ");break;   //??¡À???????
			case 40:SetTextValue(monitor_inform_screen_id, 35,"???????????....  ");break;   //??¡À???????
			case 50:SetTextValue(monitor_inform_screen_id, 35,"???????????..... ");break;   //??¡À???????
			case 60:SetTextValue(monitor_inform_screen_id, 35,"???????????......");break;   //??¡À???????
		}
	}
	else if(pcfws_entry->self_bottom_point != pcfws_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfws_entry->point_history_len = pcfws_entry->self_bottom_point;
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.
		


		uint8_t temp_sequence_count = 0;
		
		// ??¦Ì???????????
		for (uint8_t i = 0; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fore_alarm_start_index + i; // ???????
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfws_entry->self_bottom_point &&
				pcfws_entry->detector_class[data_index] == PackClassID &&
				pcfws_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 warning display uses "??3??¡¤ X??".
				FormatRS485DetectForeWarnLine(baojingneirong, temp_sequence_count, pcfws_entry, data_index);
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == PackClassID)
			{
				if(pcfws_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == LinkageClassID)
			{
				if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????????", temp_sequence_count,
						pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
						pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[data_index].second);
				}
				else if(pcfws_entry->alarm_type[data_index] == HandAlarm)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????¡À???", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[0].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second);
				}
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == CabinClassID &&
				(pcfws_entry->alarm_type[data_index] == Loop1TempWarning || pcfws_entry->alarm_type[data_index] == Loop1SmokeWarning))
			{
				FormatLoop1WarningLine(baojingneirong, temp_sequence_count, pcfws_entry, data_index);
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == CabinClassID)
			{
				if(pcfws_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 35 + i); //(????ID,???ID??
			}
			SetTextValue(monitor_inform_screen_id, 35 + i, baojingneirong); // ??¡À???????
		}
	}
}

static void InternalScreenShowAllFireAlarm(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag)
{
	// ?????
	if(pcfas_entry->self_bottom_point == 0) // 
	{
		if(pcfas_entry->point_history_len != 0)
		{
			pcfas_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 50);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 51);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 52);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 53);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 54);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 55);//(????ID,???ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 49,"??????????.     ");break;   //??¡À???????
			case 20:SetTextValue(monitor_inform_screen_id, 49,"??????????..    ");break;   //??¡À???????
			case 30:SetTextValue(monitor_inform_screen_id, 49,"??????????...   ");break;   //??¡À???????
			case 40:SetTextValue(monitor_inform_screen_id, 49,"??????????....  ");break;   //??¡À???????
			case 50:SetTextValue(monitor_inform_screen_id, 49,"??????????..... ");break;   //??¡À???????
			case 60:SetTextValue(monitor_inform_screen_id, 49,"??????????......");break;   //??¡À???????
		}
	}
	else if(pcfas_entry->self_bottom_point != pcfas_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfas_entry->point_history_len = pcfas_entry->self_bottom_point;
		
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.

		fire_alarm_state = 1; // ??????????
		
		// ??????????????????
		if(pcfas_entry->detector_class[0] == PackClassID && pcfas_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first fire display uses "??3??¡¤ X??".
			FormatRS485DetectFireAlarmLine(baojingneirong, 1, pcfas_entry, 0);
		}
		else if(pcfas_entry->detector_class[0] == PackClassID)
		{
			if(pcfas_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
		}
		else if(pcfas_entry->detector_class[0] == LinkageClassID)
		{
			if(pcfas_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second);
			}
			else
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????¡À???", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second);
			}
			
		}
		else if(pcfas_entry->detector_class[0] == CabinClassID)
		{
			if(pcfas_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfws.atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
		}
		
		SetTextValue(monitor_inform_screen_id, 49, baojingneirong); // ??¦Ì????????????
			
		uint8_t temp_sequence_count = 0;
		
		// ????????????????
		for (uint8_t i = 1; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fire_alarm_start_index + i;
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfas_entry->self_bottom_point &&
				pcfas_entry->detector_class[data_index] == PackClassID &&
				pcfas_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 fire display uses "??3??¡¤ X??".
				FormatRS485DetectFireAlarmLine(baojingneirong, temp_sequence_count, pcfas_entry, data_index);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == PackClassID)
			{
				
				if(pcfas_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == LinkageClassID)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????¡À???", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == CabinClassID)
			{
				if(pcfas_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 38 + i); //(????ID,???ID??
			}
			SetTextValue(monitor_inform_screen_id, 49 + i, baojingneirong); // ??¡À???????
		}
	}
}

static void InternalScreenShowAllFireAlarm_Plus(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag)
{
	// ?????
	if(pcfas_entry->self_bottom_point == 0) // 
	{
		if(pcfas_entry->point_history_len != 0)
		{
			pcfas_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 50);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 51);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 52);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 53);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 54);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 55);//(????ID,???ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 49,"??????????.     ");break;   //??¡À???????
			case 20:SetTextValue(monitor_inform_screen_id, 49,"??????????..    ");break;   //??¡À???????
			case 30:SetTextValue(monitor_inform_screen_id, 49,"??????????...   ");break;   //??¡À???????
			case 40:SetTextValue(monitor_inform_screen_id, 49,"??????????....  ");break;   //??¡À???????
			case 50:SetTextValue(monitor_inform_screen_id, 49,"??????????..... ");break;   //??¡À???????
			case 60:SetTextValue(monitor_inform_screen_id, 49,"??????????......");break;   //??¡À???????
		}
	}
	else if(pcfas_entry->self_bottom_point != pcfas_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfas_entry->point_history_len = pcfas_entry->self_bottom_point;
		
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.

		fire_alarm_state = 1; // ??????????
		
		uint8_t temp_sequence_count = 0;
		
		// ????????????????
		for (uint8_t i = 0; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fire_alarm_start_index + i;
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfas_entry->self_bottom_point &&
				pcfas_entry->detector_class[data_index] == PackClassID &&
				pcfas_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 fire display uses "??3??¡¤ X??".
				FormatRS485DetectFireAlarmLine(baojingneirong, temp_sequence_count, pcfas_entry, data_index);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == PackClassID)
			{
				
				if(pcfas_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == LinkageClassID)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??????¡À???", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == CabinClassID &&
				(pcfas_entry->alarm_type[data_index] == Temperature || pcfas_entry->alarm_type[data_index] == Smoke))
			{
				FormatLoop1FireLine(baojingneirong, temp_sequence_count, pcfas_entry, data_index);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == CabinClassID)
			{
				if(pcfas_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 38 + i); //(????ID,???ID??
			}
			SetTextValue(monitor_inform_screen_id, 49 + i, baojingneirong); // ??¡À???????
		}
	}
}

static void CreatNewFireExtinguishRecord(
	FireExtinguishDeviceActionSave *fedas_entry, // ??????????
	FireExtinguishDeviceActionSave *copy_fedas,  // ??????????
	uint8_t copy_dedas_offset,
	FireExtinguishDeviceActionType state, 
	uint16_t state_switch_delay             // ???§Ý???? 
)
{
	// ??????RTC???
	getBM8563TimeToSystemTime();
	// ??????????
	fedas_entry->atr[fedas_entry->self_point_len].years  = years + 2000;
	fedas_entry->atr[fedas_entry->self_point_len].months = months;
	fedas_entry->atr[fedas_entry->self_point_len].days   = days;
	fedas_entry->atr[fedas_entry->self_point_len].hours  = hours;
	fedas_entry->atr[fedas_entry->self_point_len].minute = minutes;
	
	fedas_entry->atr[fedas_entry->self_point_len].second = secs;
	
	// ?????PACK/??ID
	fedas_entry->cabin_id[fedas_entry->self_point_len]   = copy_fedas->cabin_id[copy_dedas_offset];
	fedas_entry->cluster_id[fedas_entry->self_point_len] = copy_fedas->cluster_id[copy_dedas_offset];
	fedas_entry->pack_id[fedas_entry->self_point_len]    = copy_fedas->pack_id[copy_dedas_offset];
	// ?????
	fedas_entry->fed_action[fedas_entry->self_point_len] = state;
	fedas_entry->countdown_val[fedas_entry->self_point_len] = state_switch_delay; // ???????state_switch_delay??
	// ?????????
	fedas_entry->start_cntd_time[fedas_entry->self_point_len] = baojingjishi; 
	// ?????¦Ì?????	
	fedas_entry->curr_cntd_time[fedas_entry->self_point_len]  = fedas_entry->start_cntd_time[fedas_entry->self_point_len]; 
	// ????????¦Ë??
	fedas_entry->self_point_len++; 
}

static void FireExtinguishDevice1HandStart(FireExtinguishDeviceActionSave *fedas_entry)
{
	uint8_t out_fire_start_flag = 0;
	start_stop_key_state = 1; // ??????

	// ?????????????????? 
	BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_ST_PRESS, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
	for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
	{
		if(fedas_entry->cabin_id[j] == 0 && fedas_entry->cluster_id[j] != OUTFIRE_CLUSTER_ID && fedas_entry->cluster_id[j] != 0)
		{
			if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_CAN_RESTART)
			{
				// ???? ?????????? ??????????????????
				fedas_entry->fed_action[j] = FIRE_EXTINGUISH_RESTART_FINISH; // ??????????????
				// ???????? ???????
				CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_START_SPRAY_DELAY, 30);
				// ?????FLASH?? ????????????
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRESTART_AGAIN, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				// ?????FLASH?? ??????1?????????????
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
			else if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_MODE_JUDGEMENT)
			{
				fedas_entry->start_cntd_time[j] = baojingjishi; // ?????????
				fedas_entry->curr_cntd_time[j]  = baojingjishi; // ?????¦Ì?????
				fedas_entry->fed_action[j]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 

				// ?????FLASH?? ??????1?????????????
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
		}
		
	}
	if(out_fire_start_flag == 0) // ?????????????????¦Ê¦Ä?
	{
		uint8_t index = fedas_entry->self_point_len;

		// ??????RTC???
		getBM8563TimeToSystemTime();
		
		// ??????????
		fedas_entry->atr[index].years  = years + 2000;
		fedas_entry->atr[index].months = months;
		fedas_entry->atr[index].days   = days;
		fedas_entry->atr[index].hours  = hours;
		fedas_entry->atr[index].minute = minutes;
		
		fedas_entry->atr[index].second = secs;
		
		// ?????ID
		fedas_entry->cabin_id[index]   = 0;
		fedas_entry->cluster_id[index] = 1;
		fedas_entry->pack_id[index]    = 1;
		
		// ?????????????
		fedas_entry->start_cntd_time[index] = baojingjishi; // ?????????
		fedas_entry->curr_cntd_time[index]  = baojingjishi; // ?????¦Ì?????
		fedas_entry->fed_action[index]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 
		
		fedas_entry->countdown_val[index] = 30; // ???????state_switch_delay??
		
		fedas_entry->self_point_len++;
		
		FireAlarmRelayCtrl(JDQ_ON);
		ForeWarmRelayCtrl(JDQ_ON);
		beep_fire_ctrl |= 0xF0;  // ?? ?? ????
	}
}

static void FireExtinguishDevice2HandStart(FireExtinguishDeviceActionSave *fedas_entry)
{
	uint8_t out_fire_start_flag = 0;
	start_stop_key_state = 1; // ??????
	
	// ?????????????????? 
	BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_ST_PRESS, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
	for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
	{
		if(fedas_entry->cabin_id[j] != 0  && fedas_entry->cluster_id[j] == 0)
		{
			if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_CAN_RESTART)
			{
				// ???? ?????????? ??????????????????
				fedas_entry->fed_action[j] = FIRE_EXTINGUISH_RESTART_FINISH; // ??????????????
				// ???????? ???????
				CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_START_SPRAY_DELAY, 30);
				// ?????FLASH?? ????????????
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRESTART_AGAIN, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				// ?????FLASH?? ??????1?????????????
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
			else if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_MODE_JUDGEMENT)
			{
				fedas_entry->start_cntd_time[j] = baojingjishi; // ?????????
				fedas_entry->curr_cntd_time[j]  = baojingjishi; // ?????¦Ì?????
				fedas_entry->fed_action[j]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 

				// ?????FLASH?? ??????1?????????????
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
		}
	}
	if(out_fire_start_flag == 0) // ??????????????¦Ê¦Â???? ???????????
	{
		uint8_t index = fedas_entry->self_point_len;

		// ??????RTC???
		getBM8563TimeToSystemTime();
		
		// ??????????
		fedas_entry->atr[index].years  = years + 2000;
		fedas_entry->atr[index].months = months;
		fedas_entry->atr[index].days   = days;
		fedas_entry->atr[index].hours  = hours;
		fedas_entry->atr[index].minute = minutes;
		
		fedas_entry->atr[index].second = secs;
		
		// ?????ID
		fedas_entry->cabin_id[index]   = 1;
		fedas_entry->cluster_id[index] = 0;
		fedas_entry->pack_id[index]    = 0;
		
		// ?????????????
		fedas_entry->start_cntd_time[index] = baojingjishi; // ?????????
		fedas_entry->curr_cntd_time[index]  = baojingjishi; // ?????¦Ì?????
		fedas_entry->fed_action[index]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 
		
		fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // ???????state_switch_delay??
		
		fedas_entry->self_point_len++;
		
		
		FireAlarmRelayCtrl(JDQ_ON);
		ForeWarmRelayCtrl(JDQ_ON);
		beep_fire_ctrl |= 0xF0;  // ?? ?? ????
	}

}

static void FireExtinguishDeviceStateUpdate(FireExtinguishDeviceActionSave *fedas_entry, PackAlarmStorage *pas_entry)
{
	// ????????????
	if(pas_pointer != last_pas_len && pas_pointer > 0) // ??????¦Ì????????
	{
		for(uint8_t i = last_pas_len; i < pas_pointer; i++) // ??????¦Ì????¦Ë????????????§Ú??
		{
			if(pas_entry[i].cluster_id == LINKAGE_CLUSTER_ID) // ??????????õô???????§Ø?
			{
				continue;
			}
			fedas_entry->atr[fedas_entry->self_point_len]        = pas_entry[i].atr;
			fedas_entry->cabin_id[fedas_entry->self_point_len]   = pas_entry[i].cabin_id;
			fedas_entry->cluster_id[fedas_entry->self_point_len] = pas_entry[i].cluster_id;
			fedas_entry->pack_id[fedas_entry->self_point_len]    = pas_entry[i].pack_id;
			fedas_entry->fed_action[fedas_entry->self_point_len] = pas_entry[i].lunch_state; // ???????????
			fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // ???????????30??

			fedas_entry->self_point_len++;
		}
		last_pas_len = pas_pointer; 
	}
	// ???????1????
	if(getFeedBack1State() == 0x0F)
	{
		// ?????????
		beep_spray_feedback_ctrl = 1;
		// ????? ???????????
		setDealFeedBack1State();
		// ????????1??????
		Part1FeedbackLedCtrl(LED_ON);
		// ?????FLASH?? ?????????
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_FEEDBACK1, LINKAGE_CLUSTER_ID, FEEDBK1_Package_ID);
		StorageEvent_LogFeedback(FEEDBK1_Package_ID, DEV_TYPE_CONTROL_DEV, 0); /* ?????:????1??? */
		FecbusReport_Feedback(FEEDBK1_Package_ID, DEV_TYPE_CONTROL_DEV, 0);    /* FECbus:????1??? */

		// ???????????
		// ??????????
		fedas_entry->atr[fedas_entry->self_point_len].years  = years + 2000;
		fedas_entry->atr[fedas_entry->self_point_len].months = months;
		fedas_entry->atr[fedas_entry->self_point_len].days   = days;
		fedas_entry->atr[fedas_entry->self_point_len].hours  = hours;
		fedas_entry->atr[fedas_entry->self_point_len].minute = minutes;

		// ?????????
		fedas_entry->atr[fedas_entry->self_point_len].second = secs;
		
		fedas_entry->cabin_id[fedas_entry->self_point_len]   = 0;
		fedas_entry->cluster_id[fedas_entry->self_point_len] = LINKAGE_CLUSTER_ID;
		fedas_entry->pack_id[fedas_entry->self_point_len]    = FEEDBK1_Package_ID;
		fedas_entry->fed_action[fedas_entry->self_point_len] = FEEDBACK_1_PRESS; // ???????????
		fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // 
			
		fedas_entry->self_point_len++;
	}
	
	if(getOutFireKeyalue() == KEY12_PART1_STOP)
	{
		// ?????
		clearOutFireKeyValue();
		start_stop_key_state = 2; // ?????
		// ??????RTC???
		getBM8563TimeToSystemTime();
		// ?????????????????
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SP_PRESS, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
		// ????????????
		for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
		{
			if(fedas_entry->cabin_id[j] == 0  && fedas_entry->cluster_id[j] != OUTFIRE_CLUSTER_ID && fedas_entry->cluster_id[j] != 0)
			{
				// ????????????????????????????????
				if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_START_SPRAY_DELAY)
				{
					// ????????? ??? ??????????????
					fedas_entry->fed_action[j] = FIRE_EXTINGUISH_FORCE_STOP; // ??????????
					// ??????????????????? ???????????? ?????
					CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_CAN_RESTART, 0);  //  
					// ????FLASH ????????? ??????????????
					BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_STOP, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				}
			}
			
		}
	}
	else if(getOutFireKeyalue() == KEY11_PART1_SOUNDLT)
	{
		// ?????
		clearOutFireKeyValue();
		// ????????LED 
		Part1SoundLightLedCtrl(LED_ON);
		// ?????????????
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SL_PRESS, OUTFIRE_CLUSTER_ID, SoundLt_Package_ID);
		// ???????
		SoundLightRelayCtrl(JDQ_ON);
	}
	
	if(getOutFireKeyalue() == KEY14_PART2_STOP)
	{
		start_stop_key_state = 2; // ?????
		
		// ?????
		clearOutFireKeyValue();
		// ??????RTC???
		getBM8563TimeToSystemTime();
		// ?????????????????
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SP_PRESS, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
		// ????????????
		for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
		{
			if(fedas_entry->cabin_id[j] != 0  && fedas_entry->cluster_id[j] == 0)
			{
				// ????????????????????????????????
				if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_START_SPRAY_DELAY)
				{
					// ????????? ??? ??????????????
					fedas_entry->fed_action[j] = FIRE_EXTINGUISH_FORCE_STOP; // ??????????
					// ??????????????????? ???????????? ?????
					CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_CAN_RESTART, 0);  //  
					// ????FLASH ????????? ??????????????
					BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_STOP, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				}
			}
			
		}
		
	}
	else if(getOutFireKeyalue() == KEY13_PART2_SOUNDLT)
	{
		// ????????LED 
		Part2SoundLightLedCtrl(LED_ON);
		// ?????????????
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SL_PRESS, OUTFIRE_CLUSTER_ID, SoundLt_Package_ID);
		// ???????
		SoundLightRelayCtrl(JDQ_ON);
		clearOutFireKeyValue();
	}

	for(uint8_t i = 0; i < fedas_entry->self_point_len; i++) // ???????§Ò????????? 
	{
		if(fedas_entry->cluster_id[i] == LINKAGE_CLUSTER_ID) // ?????ID?????????õô???????????????
		{
			continue;
		}
		fedas_entry->curr_cntd_time[i] = baojingjishi; // ?????¦Ì?????
		switch(fedas_entry->fed_action[i])
		{
			case FIRE_EXTINGUISH_MODE_JUDGEMENT: { // ?§Ø?????????????
				if(fedas_entry->cluster_id[i] == 0)
				{
					if(getPart2HandAutoState() == KEY_AUTO)
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ?????????
						fedas_entry->curr_cntd_time[i]  = baojingjishi; // ?????¦Ì?????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; // ???????????
						
						// ??????? ????????????????????????????
						SoundLightRelayCtrl(JDQ_ON);
						// ????2???????LED
						Part2SoundLightLedCtrl(LED_ON);
						
						// ??? ?????????????? ????FLASH ????: ????›¥???? ????????????? ??ID ??????2ID
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIRE_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					if(getPart1HandAutoState() == KEY_AUTO) // 
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ?????????
						fedas_entry->curr_cntd_time[i]  = baojingjishi; // ?????¦Ì?????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; // ???????????
						
						// ??????? ???????????????????????
						SoundLightRelayCtrl(JDQ_ON);
						
						// ????????LED 
						Part1SoundLightLedCtrl(LED_ON);
						
						// ??? ?????????????? ????FLASH
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIRE_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_START_SPRAY_DELAY: { // ???????1??????????
				// ??BMS???????
				outfire_spray_state = 1;
			
				if(fedas_entry->cluster_id[i] == 0)
				{
					// ????¦Ë??1?????§Ø??????õô????
					mhqdbiaozhi = 1;
					// ???? ????2??????? 
					Part2StartDelayLedCtrl(LED_ON);
					// ???? ????2???LED
					Part2StartLedCtrl(LED_ON);
					// ??? ????2????
					SoundLightRelayCtrl(JDQ_ON);
					// ????2???????LED
					Part2SoundLightLedCtrl(LED_ON);
					
					// ???30???????????
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FED_START_SPRAY_DELAY_FINISH_FLAG; // ????????????
						// ?????????LED
						Part2StartDelayLedCtrl(LED_OFF);
						// ????????LED
						Part2SprayLedCtrl(LED_ON);
						
						// ??????2 ????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_1, OUTFIR1_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
						// ?????????????? ???????2
						OutFire2RelayCtrl(JDQ_ON);
						// ??????????
						DefauleRelayCtrl(JDQ_ON);
						// ?????????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_CYLINDEF_2_OPENED, 0);  // ???1?????? 
						// ????????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_FIRST_SPRAY_START, 15); // ???????15??
						// ?????????
						outfire_spray_state = 2; 
					}
				}
				else
				{
					// ????¦Ë??1?????§Ø??????õô????
					mhqdbiaozhi = 1;
					// ??????????? 
					Part1StartDelayLedCtrl(LED_ON);
					// ???? ???LED
					Part1StartLedCtrl(LED_ON);
					// ???????
					SoundLightRelayCtrl(JDQ_ON);
					// ????????LED 
					Part1SoundLightLedCtrl(LED_ON);
					
					// ???30???????????
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FED_START_SPRAY_DELAY_FINISH_FLAG; // ????????????
						// ?????????LED
						Part1StartDelayLedCtrl(LED_OFF);
						// ????????LED
						Part1SprayLedCtrl(LED_ON);
						// ??????1????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_1, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
						// ?????????????? ???????
						OutFire1RelayCtrl(JDQ_ON);
						// ??????????
						DefauleRelayCtrl(JDQ_ON);
						// ?????????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_CYLINDEF_1_OPENED, 0);  // ???1?????? 
						// ????????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_FIRST_SPRAY_START, 15); // ???????15??
						// ?????????
						outfire_spray_state = 2; 
					}
					// ?????????????????????????/???
					else if(cluster_solenoid_valve_start_state == 0 && fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= (fedas_entry->countdown_val[i] - 2) )
					{
						// ???????????
						cluster_solenoid_valve_start_state = 1; 
						if(fedas_entry->cabin_id[i] == 0)
						{
							ClusterOrCabinCtrlCmd(fedas_entry->cluster_id[i], CLUSTER_VALVE1, 0xFF);
						}
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_FIRST_SPRAY_START: { // ???????????????
				if(fedas_entry->cluster_id[i] == 0)
				{
					Part2StartDelayLedCtrl(LED_ON);
					// ???????????s?? ????????????
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_FIRST_SPRAY_FINISH; // ????¦Ã?????????
						
						// ????????????? ?????????
						OutFire1RelayCtrl(JDQ_OFF);
						
						// 2025/10/28 10:50 ??????????????????????????
//						// ?????????????
//						DefauleRelayCtrl(JDQ_OFF);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_DELAY, 300);
						// ??????1???????? ??? ?????????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_2_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
						
						Part2StartDelayLedCtrl(LED_OFF);
					}
				}
				else
				{
					Part1StartDelayLedCtrl(LED_ON);
					// ???????????s?? ????????????
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_FIRST_SPRAY_FINISH; // ????¦Ã?????????
						
						// ????????????? ?????????
						OutFire1RelayCtrl(JDQ_OFF);
						
						// 2025/10/28 10:50 ??????????????????????????
//						// ?????????????
//						DefauleRelayCtrl(JDQ_OFF);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_DELAY, 300);
						// ??????1???????? ??? ?????????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_2_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
						
						Part1StartDelayLedCtrl(LED_OFF);
					}
				}
				break; 
			}
			case FIRE_EXTINGUISH_SECOND_SPRAY_DELAY: {   // ???????2??????????
				
				if(fedas_entry->cluster_id[i] == 0)
				{
					Part2StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FED_SECOND_SPRAY_DELAY_FINISH_FLAG; // ????????????????
						// ?????????????? ???????
						OutFire2RelayCtrl(JDQ_ON);
						// ??????????
						DefauleRelayCtrl(JDQ_ON);
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_START, 15);
						// ????????????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_2, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
						// ??????????
						Part1StartDelayLedCtrl(LED_OFF);
					}
				}
				else
				{
					Part1StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FED_SECOND_SPRAY_DELAY_FINISH_FLAG; // ????????????????
						// ?????????????? ???????
						OutFire1RelayCtrl(JDQ_ON);
						// ??????????
						DefauleRelayCtrl(JDQ_ON);
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_START, 15);
						// ????????????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_2, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
						// ??????????
						Part1StartDelayLedCtrl(LED_OFF);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_SECOND_SPRAY_START: { // ?????????¦Ã?????? 
				if(fedas_entry->cluster_id[i] == 0)
				{
					// ????????????
					Part2StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_SECOND_SPRAY_FINISH; // ???????????
						// 
						Part2StartDelayLedCtrl(LED_OFF);
						// ????????????? ?????????
						OutFire2RelayCtrl(JDQ_OFF);
						
//						// ?????????????
//						DefauleRelayCtrl(JDQ_OFF);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_DELAY, 300);
						// ??????????????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_3_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					// ????????????
					Part1StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_SECOND_SPRAY_FINISH; // ???????????
						// 
						Part1StartDelayLedCtrl(LED_OFF);
						// ????????????? ?????????
						OutFire1RelayCtrl(JDQ_OFF);
						
//						// ?????????????
//						DefauleRelayCtrl(JDQ_OFF);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_DELAY, 300);
						// ??????????????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_3_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_THIRD_SPRAY_DELAY: {   // ???????3??????????
				if(fedas_entry->cluster_id[i] == 0)
				{
					// ????????????
					Part2StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FED_THIRD_SPRAY_DELAY_FINISH_FLAG; // ???????????
						// ??????????LED
						Part2StartDelayLedCtrl(LED_OFF);
						// ?????????????? ???????
						OutFire2RelayCtrl(JDQ_ON);
						// ??????????
						DefauleRelayCtrl(JDQ_ON);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_START, 999);
						// ??????1 ?????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_3, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					// ????????????
					Part1StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FED_THIRD_SPRAY_DELAY_FINISH_FLAG; // ???????????
						// ??????????LED
						Part1StartDelayLedCtrl(LED_OFF);
						// ?????????????? ???????
						OutFire1RelayCtrl(JDQ_ON);
						// ??????????
						DefauleRelayCtrl(JDQ_ON);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_START, 999);
						// ??????1 ?????????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_3, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_THIRD_SPRAY_START: {  // ???????????
				if(fedas_entry->cluster_id[i] == 0)
				{
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_THIRD_SPRAY_FINISH; // ????????????
						
						// ????????????? ?????????
						OutFire2RelayCtrl(JDQ_OFF);
						
//						// ?????????????
//						DefauleRelayCtrl(JDQ_OFF);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE, 999);
						// ??????1 ??????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_OVER, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // ??????????
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_THIRD_SPRAY_FINISH; // ????????????
						
						// ????????????? ?????????
						OutFire1RelayCtrl(JDQ_OFF);
						// ?????????????
						DefauleRelayCtrl(JDQ_OFF);
						
						// ????????
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE, 999);
						// ??????1 ??????
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_OVER, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_FORCE_STOP : { // ???????????? ????????
				
				
				break;
			}
			case FIRE_EXTINGUISH_CAN_RESTART: { // ?????????????? ?§Ø???????o????????
				break;
			}
			default:
				break;
		}
		
	}
	
	
}

//#define FADAS_DEBUG

#ifdef FADAS_DEBUG
static void printf_fadas_data(FireExtinguishDeviceActionSave *fedas_entry)
{
	uint8_t debug_buff[64] = {0};
	uint8_t send_len = 0;
	for(uint8_t i = 0; i < fedas_entry->self_point_len; i++)
	{
		// ??????
		send_len = sprintf((char *)debug_buff, "sq:%d time:%d/%d/%d/%d/%d\r\n", i, fedas_entry->atr[i].years, 
									fedas_entry->atr[i].months, fedas_entry->atr[i].days,
									fedas_entry->atr[i].hours, fedas_entry->atr[i].minute);
		DebugSendString(debug_buff, send_len);
		// ???????
		send_len = sprintf((char *)debug_buff, "sq:%d cb_id:%d pk_id:%d cl_id:%d\r\n", i, fedas_entry->cabin_id[i], fedas_entry->pack_id[i], fedas_entry->cluster_id[i]);
		DebugSendString(debug_buff, send_len);
		// ???????
		send_len = sprintf((char *)debug_buff, "sq:%d action_id:%d \r\n", i, fedas_entry->fed_action[i]);
		DebugSendString(debug_buff, send_len);
	}
	
}
#endif



// ?????¡Â??????????
static void InternalScreenShowFireExtinguisher(FireExtinguishDeviceActionSave *fedas_entry, uint8_t fresh_page_flag)
{
	if(fedas_entry->self_point_len == 0) // ??§Ý?
	{
		if(fedas_entry->last_point_len == 255)
		{
			clearTextValue(monitor_inform_screen_id , 2);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 3);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 4);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 5);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 6);//(????ID,???ID)
			clearTextValue(monitor_inform_screen_id , 7);//(????ID,???ID)
			fedas_entry->last_point_len = 0;
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 1,"????????????.     ");break;   //??¡À???????
			case 20:SetTextValue(monitor_inform_screen_id, 1,"????????????..    ");break;   //??¡À???????
			case 30:SetTextValue(monitor_inform_screen_id, 1,"????????????...   ");break;   //??¡À???????
			case 40:SetTextValue(monitor_inform_screen_id, 1,"????????????....  ");break;   //??¡À???????
			case 50:SetTextValue(monitor_inform_screen_id, 1,"????????????..... ");break;   //??¡À???????
			case 60:SetTextValue(monitor_inform_screen_id, 1,"????????????......");break;   //??¡À???????
		}
	}
	else if( fresh_page_flag == 1 ) // ????§Ý?????????????????????????????????????
	{
		uint8_t baojingneirong[96] = {0}; // XR5000_LOOP3_CHANGE_20260726: Loop 3 gas extinguish text needs more room.
		
		uint8_t temp_sequence_count = 0;
		
		#ifdef FADAS_DEBUG
		printf_fadas_data(fedas_entry);
		#endif
		// 3????????????
		for (uint8_t i = 0; i < Out_Fire_Show_Zone; i++) {
			uint8_t data_index = fedas_fresh_point + i;
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < fedas_entry->self_point_len) // ????????§Ø????????? ???????§Ø?????????
			{
				if(fedas_entry->cluster_id[data_index] == LINKAGE_CLUSTER_ID) // ?????????ID
				{
					if(fedas_entry->fed_action[data_index] == FEEDBACK_1_PRESS)
					{
						sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????1????", temp_sequence_count,
							fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
							fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second);
					}
					else if(fedas_entry->fed_action[data_index] == FEEDBACK_2_PRESS)
					{
						sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????2????", temp_sequence_count,
							fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
							fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second);
					}
					SetTextValue(59, 1 + i, baojingneirong);
					continue; // ????????????
				}
				uint16_t temp_time = fedas_entry->curr_cntd_time[data_index] - fedas_entry->start_cntd_time[data_index];
				if(FormatRS485DetectFireExtinguisherLine(baojingneirong, temp_sequence_count, fedas_entry, data_index, temp_time) == 1)
				{
					// XR5000_LOOP3_CHANGE_20260726: Loop 3 gas extinguish display uses "??3??¡¤ X??".
					SetTextValue(monitor_inform_screen_id, 1 + i, baojingneirong);
					continue;
				}
				switch(fedas_entry->fed_action[data_index])
				{
					case FIRE_EXTINGUISH_MODE_JUDGEMENT:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							if(getPart2HandAutoState() == KEY_MANUAL) // ???
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ??????????????", temp_sequence_count,
									fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
									fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
									fedas_entry->cabin_id[data_index]);
							}
						}
						else
						{
							if(getPart1HandAutoState() == KEY_MANUAL) // ???
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ??????????????", temp_sequence_count,
									fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
									fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
									fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
							}
						}
						break;
					case FIRE_EXTINGUISH_START_SPRAY_DELAY:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ??????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ??????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FED_START_SPRAY_DELAY_FINISH_FLAG: // ??????????????????????
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????1????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????1????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						break;
					case FIRE_EXTINGUISH_FIRST_SPRAY_START:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FIRE_EXTINGUISH_FIRST_SPRAY_FINISH:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????1????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????1????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						break;
					case FIRE_EXTINGUISH_SECOND_SPRAY_DELAY:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????2??????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????2??????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						break;
					case FED_SECOND_SPRAY_DELAY_FINISH_FLAG:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????2????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????2????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						break;
					case FIRE_EXTINGUISH_SECOND_SPRAY_START:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FIRE_EXTINGUISH_SECOND_SPRAY_FINISH:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????2????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????2????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						

						break;
					case FIRE_EXTINGUISH_THIRD_SPRAY_DELAY:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????3??????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????3??????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						break;
					case FED_THIRD_SPRAY_DELAY_FINISH_FLAG: // ?????????????????
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????3????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????3????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						

						break;
					case FIRE_EXTINGUISH_THIRD_SPRAY_START:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????????????%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FIRE_EXTINGUISH_THIRD_SPRAY_FINISH:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ???????3????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ???????3????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						

						break;
					case FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ????????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ????????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						

						break;
					case FIRE_EXTINGUISH_STARYUP_FINISH_FLAG:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ?????????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ?????????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						

						break;
					case FIRE_EXTINGUISH_CYLINDEF_1_OPENED:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ??????1???", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ??????1???", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						
					
						break;
					case FIRE_EXTINGUISH_FORCE_STOP: {
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ??????????????--", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ??????????????--", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						
						break;
					}
					case FIRE_EXTINGUISH_CAN_RESTART:
					case FIRE_EXTINGUISH_RESTART_FINISH:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ??????????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ??????????????", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						
						break;
					case FEEDBACK_1_PRESS : {
						sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ????1????", temp_sequence_count,
							fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
							fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second);
						break;
					}
					case FIRE_EXTINGUISH_CYLINDEF_2_OPENED: {
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ?? ??????2???", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??pack%d?? ??????1???", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						break;
					}
				}					
				
			}
			else
			{
				baojingneirong[0] = 0;
			}
			SetTextValue(monitor_inform_screen_id, 1 + i,baojingneirong);
		} // ???????
	} // ?§Ò??????????¦Ì?????
}

void InternalScreenShowDetectorDataCtrlInit(DetectorDataShowCtrl *ddsc_entry)
{
	ddsc_entry->curr_detector_page = 1; // ????????
	ddsc_entry->last_detector_page = 0; // ???????

	memset(ddsc_entry->detector_offline_fresh_flag, 0xFF, sizeof(ddsc_entry->detector_offline_fresh_flag));
	
	// ?????????????
	memset(ddsc_entry->detect_online_state, 0xFF, sizeof(ddsc_entry->detect_online_state));
	// ????????
	memset(ddsc_entry->detect_shield_state, 0xFF, sizeof(ddsc_entry->detect_shield_state));
	
	// ???????
	memset(ddsc_entry->last_temperature,    0xFF, sizeof(ddsc_entry->last_temperature));
	// ????????
	memset(ddsc_entry->last_temperat_state, 0xFF, sizeof(ddsc_entry->last_temperat_state));
	
	// ??????
	memset(ddsc_entry->last_smoke_state,    0xFF, sizeof(ddsc_entry->last_smoke_state));
	
	// ???????????
	memset(ddsc_entry->last_co_concentrat,  0xFF, sizeof(ddsc_entry->last_co_concentrat));
	// ????????????
	memset(ddsc_entry->last_co_state,       0xFF, sizeof(ddsc_entry->last_co_state));
	
}

void InternalScreenShowDetectorDataCtrlInit_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry)
{
	ddsc_32p_entry->curr_detector_page = 1; // ????????

	ddsc_32p_entry->last_detector_page = 0xFF; // ????????
	
	// ???PACK??0 ?????????????
	ddsc_32p_entry->curr_pack_id = 0; // 
	// ?????¡À??¦Ë
	ddsc_32p_entry->force_fresh_flag = 1;
	
	ddsc_32p_entry->last_temperature = 0xFF; // ??????
	ddsc_32p_entry->last_temperat_state = 0xFF; // ????????
	ddsc_32p_entry->last_smoke_state = 0xFF; // ?????????
	ddsc_32p_entry->last_co_concentrat = 0xFF; // ????????????
	ddsc_32p_entry->last_co_state = 0xFF; // ???????????
	
	// ???????????????????255
	memset(ddsc_32p_entry->lat_detector_online_num, 0xFF, sizeof(ddsc_32p_entry->lat_detector_online_num));
}

void Bsp_Screen_Buff_Init(void)
{
	DetectorDataShowCtrl *p = &ddsc;
	DetectorDataShowCtrl_32Pack *ddsc_32p_entry = &ddsc_32p;
	// ????·Ú
	InternalScreenShowDetectorDataCtrlInit(p);
	// 1??32???·Ú
	InternalScreenShowDetectorDataCtrlInit_32Pack(ddsc_32p_entry);
}

uint8_t detector_online_ctrl_id[11] = {0, 4, 13, 24, 34, 44, 54, 64, 74, 84, 94};
uint8_t temperature_ctrl_id[11] = {0, 9, 18, 28,  38, 48, 58, 68, 78, 88, 98};
uint8_t co_concentrate_ctrl_id[11] = {0, 7, 21, 31, 41, 51, 61, 71, 81, 91, 101};
uint8_t smoke_show_ctrl_id[11] = {0, 11, 22, 32, 42, 52, 62, 72, 82, 92, 102};

static void InternalScreenShowClusterData(DetectorDataShowCtrl *ddsc_entry)
{
	uint8_t temp_screen_id = 54;
	uint8_t curr_page = ddsc_entry->curr_detector_page; // ???????????
	uint8_t fresh_flag = 0;
	
	if(curr_page !=	ddsc_entry->last_detector_page) // ??¦Ì??????
	{
		uint8_t buff[32] = {0};

		fresh_flag |= 1; // ??¡À??
		sprintf((char *)buff, "??%d?? PACK??????", curr_page);
		SetTextValue(temp_screen_id, 1, buff);
		sprintf((char *)buff, "??%d?? ?????????", curr_page);
		SetTextValue(temp_screen_id, 2, buff);
		
		sprintf((char *)buff, "%d/20", curr_page);
		SetTextValue(temp_screen_id, 105, buff);
	}
	
	if(cu_sxzt[curr_page] == 0 || cu_tcq_sxzt[curr_page] == 0) // ????????????
	{
		fresh_flag |= 1; // 
		if(ddsc_entry->detector_offline_fresh_flag[curr_page] != cu_tcq_sxzt[curr_page] || fresh_flag) // ????????????????????
		{
			ddsc_entry->detector_offline_fresh_flag[curr_page] = cu_tcq_sxzt[curr_page]; // ?????????1???
			for(uint8_t i = cu_tcq_sxzt[curr_page] + 1; i < 11; i++)
			{
				SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "¦Ä????");
				// ????????????
				SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, temperature_ctrl_id[i], "--");
				// ?????????
				SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], "--");
				// ??????
				SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "--");
			}
		}
	}
	if(cu_sxzt[curr_page] != 0) // ???????????
	{
//		if(ddsc_entry->detector_offline_fresh_flag[curr_page] == 0)
//			fresh_flag = 1;
		//ddsc_entry->detector_offline_fresh_flag[curr_page] = 0xFF; // ???????????????????
		for(uint8_t i = 1; i < cu_tcq_sxzt[curr_page] + 1; i++)
		{
			if(getClusterPackDisconnectCount(curr_page, i) != PackDisconnectCount) // ?§Ø???????
			{ 
				// ?????????
				if(ddsc_entry->detect_online_state[curr_page][i] != 1 || fresh_flag)
				{
					fresh_flag |= 2; // ????????? ????????????
					ddsc_entry->detect_online_state[curr_page][i] = 1; // ???curr_page?? ??i??????1
					SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0x0400);
					SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "????");
				} // ???????
				
				// ???????
				if(ddsc_entry->last_temperature[curr_page][i] != PACK_wendu_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_temperature[curr_page][i] = PACK_wendu_buf[curr_page][i];
					SetTextInt32(temp_screen_id, temperature_ctrl_id[i], PACK_wendu_buf[curr_page][i], 1, 2);//??????
				}
				
				// ?????????
				if(ddsc_entry->last_temperat_state[curr_page][i] != PACK_WDZT_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_temperat_state[curr_page][i] = PACK_WDZT_buf[curr_page][i];
					
					if(ddsc_entry->last_temperat_state[curr_page][i] == 0)
					{
						SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x0400); // ????
					}
					else if(ddsc_entry->last_temperat_state[curr_page][i] == 1)
					{
						SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0xFB20); // ???
					}
					else
					{
						SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0xF800); // ??
					}
				}
				
				// ??curr_page?? ??i?? CO????
				uint16_t temp_co_concen = getPackCoConcenValue(curr_page, i);
				if(ddsc_entry->last_smoke_state[curr_page][i] != temp_co_concen || fresh_flag)
				{
					ddsc_entry->last_smoke_state[curr_page][i] = temp_co_concen; // ?›¥?????????
//					SetTextInt32(temp_screen_id, co_concentrate_ctrl_id[i], temp_co_concen, 1, 1); // CO???
					uint8_t buff[16];
					sprintf((char *)buff, "%dPPM", temp_co_concen);
					SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], buff);
				}
				
				// CO??????
				if(ddsc_entry->last_co_state[curr_page][i] != PACK_COZT_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_temperat_state[curr_page][i] = PACK_COZT_buf[curr_page][i];
					
					if(ddsc_entry->last_temperat_state[curr_page][i] == 0)
					{
						SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x0400); // ????
					}
					else if(ddsc_entry->last_temperat_state[curr_page][i] == 1)
					{
						SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0xFB20); // ???
					}
					else if(ddsc_entry->last_temperat_state[curr_page][i] == 2)
					{
						SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0xF800); // ???????
					}
				}
				
				if(ddsc_entry->last_smoke_state[curr_page][i] != PACK_YWZT_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_smoke_state[curr_page][i] = PACK_YWZT_buf[curr_page][i];
					if(PACK_YWZT_buf[curr_page][i] == 0)
					{
						SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x0400); // ????
						SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "????");
					}
					else if(PACK_YWZT_buf[curr_page][i] == 1)
					{
						SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0xFB20); // ???
						SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "????");
					}
					else if(PACK_YWZT_buf[curr_page][i] == 2)
					{
						SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0xF800); // ???????
						SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "????");
					}
					
				}
				
			} // ??????? ????
			else // ?????????
			{ 
				if(ddsc_entry->detect_online_state[i] != 0 || fresh_flag)
				{
					ddsc_entry->detect_online_state[curr_page][i] = 0;
					SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0xFB20);
					SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "????");
					
					// ????????????
					SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x8410);
					SetTextValue(temp_screen_id, temperature_ctrl_id[i], "--");

					SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x8410);
					SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], "--");
					
					SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x8410);
					SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "--");
				} // ???????
			} // ?????§Ø?
		} // for??????? ???????????????????????
		
		if(ddsc_entry->detector_offline_fresh_flag[curr_page] != cu_tcq_sxzt[curr_page])
		{
			for(uint8_t i = cu_tcq_sxzt[curr_page] + 1; i < 11; i++)
			{
				SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "¦Ä????");
				// ????????????
				SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, temperature_ctrl_id[i], "--");
				// ?????????
				SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], "--");
				// ??????
				SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "--");
			}
		}// ?????????????? ??????

		
	} // ?????????????
	if(fresh_flag == 1)
	{
		ddsc_entry->last_detector_page = curr_page; // 
	}
}

const uint8_t pack_state_show_ctrl_id[] = {
	0, 3, 7, 11, 15, 19, 23, 27, 31, 
	35, 39, 43, 47, 51, 55, 59, 63, 
	67, 71, 75, 79, 83, 87, 91, 95, 
	99, 109, 113, 117, 121, 125, 129, 133};

static void InternalScreenShowClusterData_32Pack(uint16_t screen_id_entry, DetectorDataShowCtrl_32Pack *ddsc_32p_entry)
{
	uint8_t curr_page = ddsc_32p_entry->curr_detector_page; // ???????????
	
	uint8_t temp_force_fresh = 0;
//	DebugPrintf("page:%d\r\n", ddsc_32p_entry->curr_detector_page);
//	DebugPrintf("pack:%d\r\n", ddsc_32p_entry->curr_pack_id);
//	DebugPrintf("flag:%d\r\n", ddsc_32p_entry->force_fresh_flag);

	uint8_t page_change_flage = 0;

	// ?????¡À??¦Ë
	if(ddsc_32p_entry->last_detector_page != curr_page) // ??¦Ì??????
	{
		uint8_t buff[32] = {0};
		sprintf((char *)buff, "??%d?? PACK??????", curr_page);
		SetTextValue(screen_id_entry, 1, buff); // ??¡ä???????
		sprintf((char *)buff, "%d/3", curr_page);
		SetTextValue(screen_id_entry, 105, buff); // ??¦Ì????????
		
		page_change_flage = 1;
		ddsc_32p_entry->last_detector_page = curr_page;
	}

	if(screen_id_entry == 61) // ?????????????????????????
	{
		// ??????????0 ?????????????0
		if(cu_sxzt[curr_page] == 0 || cu_tcq_sxzt[curr_page] == 0) // ????????????
		{
			if(ddsc_32p_entry->lat_detector_online_num[curr_page] != cu_tcq_sxzt[curr_page] || page_change_flage == 1) // ????????????????????
			{
				ddsc_32p_entry->lat_detector_online_num[curr_page] = cu_tcq_sxzt[curr_page]; // ?????????1???
				for(uint8_t i = cu_tcq_sxzt[curr_page] + 1; i < 33; i++)
				{
//					SetControlForeColor(screen_id_entry, detector_online_ctrl_id[i], 0x8410);
					SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "¦Ä????");
					
					ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 2; // ????¦Ä????
				}
				
				page_change_flage = 0;
			}
		}
		else
		{
			if(page_change_flage == 1 || ddsc_32p_entry->lat_detector_online_num[curr_page] != cu_tcq_sxzt[curr_page])
			{
				ddsc_32p_entry->lat_detector_online_num[curr_page] = cu_tcq_sxzt[curr_page]; // ?????????1???
				for(uint8_t i = 1; i < cu_tcq_sxzt[curr_page] + 1; i++)
				{
					if(getClusterPackDisconnectCount(curr_page, i) != PackDisconnectCount) // ?§Ø???????
					{
//						SetControlForeColor(screen_id_entry, detector_online_ctrl_id[i], 0xFB20);
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "????");
						
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 0; // ???????
					}
					else
					{
//						SetControlForeColor(screen_id_entry, pack_state_show_ctrl_id[i], 0xFB20);
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "????");
						
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 1; // ????????
					}
				}
				
				page_change_flage = 0; 
			}
			else
			{
				uint8_t temp_index = curr_page - 1;
				
				for(uint8_t i = 0; i < cu_tcq_sxzt[curr_page]; i++)
				{
					if(ddsc_32p_entry->last_derector_state[temp_index][i] == 0 && getClusterPackDisconnectCount(curr_page, i + 1) == PackDisconnectCount)
					{
						ddsc_32p_entry->last_derector_state[temp_index][i] = 1; // ????????
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i + 1], "????");
					}
					else if(ddsc_32p_entry->last_derector_state[temp_index][i] == 1 && getClusterPackDisconnectCount(curr_page, i + 1) != PackDisconnectCount)
					{
						ddsc_32p_entry->last_derector_state[temp_index][i] = 0; // ???????
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i + 1], "????");
					}
					else if(ddsc_32p_entry->last_derector_state[temp_index][i] == 2 && getClusterPackDisconnectCount(curr_page, i + 1) != PackDisconnectCount)
					{
						ddsc_32p_entry->last_derector_state[temp_index][i] = 0; // ???????
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i + 1], "????");
					}
				}
				
				for(uint8_t j = cu_tcq_sxzt[curr_page]; j < 32; j++)
				{
					if(ddsc_32p_entry->last_derector_state[temp_index][j] != 2)
					{
						ddsc_32p_entry->last_derector_state[temp_index][j] = 2;
						
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[j + 1], "¦Ä????");
					}
				}
				
			}
		}
	}
	else if(screen_id_entry == 62) // ??????????????????
	{
		if(ddsc_32p_entry->force_fresh_flag == 1)
		{
			// ??????1???
			ddsc_32p_entry->force_fresh_flag = 0;
			temp_force_fresh = 1;
		}

		if(cu_sxzt[curr_page] != 0 && cu_tcq_sxzt[curr_page] >= ddsc_32p_entry->curr_pack_id) // ??????????? ??????id§³??????????
		{
			if(getClusterPackDisconnectCount(curr_page, ddsc_32p_entry->curr_pack_id) != PackDisconnectCount) // ?§Ø???????
			{
				uint8_t buff[32];
				sprintf((char *)buff, "pack%d ??:????", ddsc_32p_entry->curr_pack_id);
				SetTextValue(screen_id_entry, 5, buff);
				
				// ???????
				if(ddsc_32p_entry->last_temperature != PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_temperature = PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id];
					sprintf((char *)buff, "??        ??:%d??", ddsc_32p_entry->last_temperature);
					//SetTextInt32(screen_id_entry, 9, PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id], 1, 2);//??????
					SetTextValue(screen_id_entry, 9, buff);
				}
				
//				// ?????????
//				if(ddsc_32p_entry->last_temperat_state != PACK_WDZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] || ddsc_32p_entry->force_fresh_flag == 1)
//				{
//					ddsc_32p_entry->last_temperat_state = PACK_WDZT_buf[curr_page][ddsc_32p_entry->curr_pack_id];
//					
//					if(ddsc_32p_entry->last_temperat_state == 0)
//					{
//						SetControlForeColor(screen_id_entry, 9, 0x0400); // ????
//					}
//					else if(ddsc_32p_entry->last_temperat_state == 1)
//					{
//						SetControlForeColor(screen_id_entry, 9, 0xFB20); // ???
//					}
//					else
//					{
//						SetControlForeColor(screen_id_entry, 9, 0xF800); // ??
//					}
//				}
						
				// ??curr_page?? ??i?? CO????
				uint16_t temp_co_concen = getPackCoConcenValue(curr_page, ddsc_32p_entry->curr_pack_id);
				if(ddsc_32p_entry->last_smoke_state != temp_co_concen || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = temp_co_concen; // ?›¥?????????
					sprintf((char *)buff, "??????:%dPPM", temp_co_concen);
					SetTextValue(screen_id_entry, 17, buff);
				}
				
//				// CO??????
//				if(ddsc_32p_entry->last_co_state != PACK_COZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] || ddsc_32p_entry->force_fresh_flag == 1)
//				{
//					ddsc_32p_entry->last_co_state = PACK_COZT_buf[curr_page][ddsc_32p_entry->curr_pack_id];
//					
//					if(ddsc_32p_entry->last_co_state == 0)
//					{
//						SetControlForeColor(screen_id_entry, 17, 0x0400); // ????
//					}
//					else if(ddsc_32p_entry->last_co_state == 1)
//					{
//						SetControlForeColor(screen_id_entry, 17, 0xFB20); // ???
//					}
//					else if(ddsc_32p_entry->last_temperat_state == 2)
//					{
//						SetControlForeColor(screen_id_entry, 17, 0xF800); // ???????
//					}
//				}
						
				if(ddsc_32p_entry->last_smoke_state != PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id];
					if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 0)
					{
//						SetControlForeColor(screen_id_entry, 13, 0x0400); // ????
						SetTextValue(screen_id_entry, 13, "??        ??:????");
					}
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 1)
					{
//						SetControlForeColor(screen_id_entry, 13, 0xFB20); // ???
						SetTextValue(screen_id_entry, 13, "??        ??:????");
					} 
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 2)
					{
//						SetControlForeColor(screen_id_entry, 13, 0xF800); // ???????
						SetTextValue(screen_id_entry, 13, "??        ??:????");
					}
					
				}
			}
			else
			{
				if(temp_force_fresh == 1)
				{
					uint8_t buff[32];
					sprintf((char *)buff, "pack%d ????", ddsc_32p_entry->curr_pack_id);
					
//					SetControlForeColor(screen_id_entry, 5, 0xFB20);
					SetTextValue(screen_id_entry, 5, buff);
					
//					SetControlForeColor(screen_id_entry, 13, 0x8410);
					SetTextValue(screen_id_entry, 13, "??        ??:--");
					
//					SetControlForeColor(screen_id_entry, 17, 0x8410);
					SetTextValue(screen_id_entry, 17, "??????:--");
					
//					SetControlForeColor(screen_id_entry, 17, 0x8410);
					SetTextValue(screen_id_entry, 9, "??        ??:--");
					
				}
//				
//				uint8_t buff[32];
//				sprintf((char *)buff, "pack%d ????", ddsc_32p_entry->curr_pack_id);
//				SetTextValue(screen_id_entry, 5, buff);
//				SetTextValue(screen_id_entry, 13, "??        ??:--");
//				SetTextValue(screen_id_entry, 17, "??????:--");
//				SetTextValue(screen_id_entry, 9, "??        ??:--");
				
			}
		}
		else
		{
			if(temp_force_fresh == 1)
			{
				uint8_t buff[32];
				sprintf((char *)buff, "pack%d ??:¦Ä????", ddsc_32p_entry->curr_pack_id);
//				DebugSendString(buff, sizeof(buff));
				//					SetControlForeColor(screen_id_entry, 5, 0xFB20);
				SetTextValue(screen_id_entry, 5, buff);

				//					SetControlForeColor(screen_id_entry, 13, 0x8410);
				SetTextValue(screen_id_entry, 13, "??        ??:--");

				//					SetControlForeColor(screen_id_entry, 17, 0x8410);
				SetTextValue(screen_id_entry, 17, "??????:--");

				//					SetControlForeColor(screen_id_entry, 17, 0x8410);
				SetTextValue(screen_id_entry, 9, "??        ??:--");

			}
			
//			uint8_t buff[32];
//			sprintf((char *)buff, "pack%d ¦Ä????", ddsc_32p_entry->curr_pack_id);
//			SetTextValue(screen_id_entry, 5, buff);
//			SetTextValue(screen_id_entry, 13, "??        ??:--");
//			SetTextValue(screen_id_entry, 17, "??????:--");
//			SetTextValue(screen_id_entry, 9, "??        ??:--");
		}
		
	}
	
//	uint8_t buff[64];
//	
//	sprintf((char *)buff, "sc_id:%d pg_id:%d pid: %d tc:%d \r\n", screen_id_entry, curr_page, ddsc_32p_entry->curr_pack_id, cu_tcq_sxzt[curr_page]);
//	DebugSendString(buff, sizeof(buff));
}

// ????????????????0xFF
uint8_t last_pack_online_buff_state[33] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF  // ???33??0xFF
};

static void InternalScreenShowClusterData_32Pack_Plus(uint16_t screen_id_entry, DetectorDataShowCtrl_32Pack *ddsc_32p_entry)
{
	uint8_t curr_page = ddsc_32p_entry->curr_detector_page; // ???????????
	
	uint8_t temp_force_fresh = 0;

	uint8_t page_change_flage = 0;

	// ?????¡À??¦Ë
	if(ddsc_32p_entry->last_detector_page != curr_page) // ??¦Ì??????
	{
		uint8_t buff[32] = {0};
		sprintf((char *)buff, "??%d?? PACK??????", curr_page);
		SetTextValue(screen_id_entry, 1, buff); // ??¡ä???????
		sprintf((char *)buff, "%d/3", curr_page);
		SetTextValue(screen_id_entry, 105, buff); // ??¦Ì????????
		
		page_change_flage = 1;
		ddsc_32p_entry->last_detector_page = curr_page;
	}

	if(screen_id_entry == 61) // ?????????????????????????
	{
		for(uint8_t i = 1; i < 33; i++)
		{
			if((pack_online_buff[curr_page][i] != last_pack_online_buff_state[i] && pack_online_buff[curr_page][i] == 0) || 
					page_change_flage == 1) // ??????
			{
				last_pack_online_buff_state[i] = pack_online_buff[curr_page][i]; // ????????
				
				SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "¦Ä????");
				ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 2; // ????¦Ä????
			}
			else if(pack_online_buff[curr_page][i] == 1) // ??????????????
			{
				if(getClusterPackDisconnectCount(curr_page, i) != PackDisconnectCount) // ?§Ø???????
				{
					if(page_change_flage == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] != 0)
					{
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "????");
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 0; // ???????
					}
				}
				else
				{
					if(page_change_flage == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] != 1)
					{
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "????");
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 1; // ????????
					}
					
				}
			}
		} // ????????
	} // screen id 61 
	else if(screen_id_entry == 62) // ??????????????????
	{
		if(ddsc_32p_entry->force_fresh_flag == 1)
		{
			// ??????1???
			ddsc_32p_entry->force_fresh_flag = 0;
			temp_force_fresh = 1;
		}
		
		uint8_t check_pack_id = ddsc_32p_entry->curr_pack_id;
		
		if(pack_online_buff[curr_page][check_pack_id] == 1) // ???????????
		{
			if(getClusterPackDisconnectCount(curr_page, check_pack_id) != PackDisconnectCount)
			{
				uint8_t buff[32];
				if(ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] != 0 || temp_force_fresh == 1)
				{
					sprintf((char *)buff, "pack%d ??:????", check_pack_id);
					SetTextValue(screen_id_entry, 5, buff);
					ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] = 0; // ????0
				}

				// ???????
				if(ddsc_32p_entry->last_temperature != PACK_wendu_buf[curr_page][check_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_temperature = PACK_wendu_buf[curr_page][check_pack_id];
					sprintf((char *)buff, "??        ??:%d??", ddsc_32p_entry->last_temperature);
					//SetTextInt32(screen_id_entry, 9, PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id], 1, 2);//??????
					SetTextValue(screen_id_entry, 9, buff);
				}
						
				// ??curr_page?? ??i?? CO????
				uint16_t temp_co_concen = getPackCoConcenValue(curr_page, check_pack_id);
				if(ddsc_32p_entry->last_smoke_state != temp_co_concen || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = temp_co_concen; // ?›¥?????????
					sprintf((char *)buff, "??????:%dPPM", temp_co_concen);
					SetTextValue(screen_id_entry, 17, buff);
				}

				if(ddsc_32p_entry->last_smoke_state != PACK_YWZT_buf[curr_page][check_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = PACK_YWZT_buf[curr_page][check_pack_id];
					if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 0)
					{
						SetTextValue(screen_id_entry, 13, "??        ??:????");
					}
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 1)
					{
						SetTextValue(screen_id_entry, 13, "??        ??:????");
					} 
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 2)
					{
						SetTextValue(screen_id_entry, 13, "??        ??:????");
					}
				}
			}
			else // ??????
			{
				if(temp_force_fresh == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] != 1)
				{
					uint8_t buff[32];
					sprintf((char *)buff, "pack%d ??:????", ddsc_32p_entry->curr_pack_id);

					SetTextValue(screen_id_entry, 5, buff);

					SetTextValue(screen_id_entry, 13, "??        ??:--");

					SetTextValue(screen_id_entry, 17, "??????:--");

					SetTextValue(screen_id_entry, 9, "??        ??:--");
					
					ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] = 1;
				}
			}
		}
		else // ?????????
		{
			if(temp_force_fresh == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][ddsc_32p_entry->curr_pack_id - 1] != 2)
			{
				uint8_t buff[32];
				sprintf((char *)buff, "pack%d ??:¦Ä????", ddsc_32p_entry->curr_pack_id);

				SetTextValue(screen_id_entry, 5, buff);

				SetTextValue(screen_id_entry, 13, "??        ??:--");

				SetTextValue(screen_id_entry, 17, "??????:--");

				SetTextValue(screen_id_entry, 9, "??        ??:--");
				
				ddsc_32p_entry->last_derector_state[curr_page - 1][ddsc_32p_entry->curr_pack_id - 1] = 2;
			}
		}

	} // screen id 62
}

const char sensor_str1[] = "???????";
const char sensor_str2[] = "????";
const char sensor_str3[] = "????";
const char sensor_str4[] = "VOC";
//const char sensor_str5[] = "??????";
const char sensor_str5[] = "CO";
const char sensor_str6[] = "???";
const char sensor_str7[] = "????????????:";

const char *sensor_str[] = {
	sensor_str1, // ???????
	sensor_str2, // ????  
	sensor_str3, // ????
	sensor_str4, // VOC
	sensor_str5, // ??????
	sensor_str6, // ???
	sensor_str7  // ????????????
};

static void InternalScreenShowCabinDate(CabinDataShowCtrl_t *cabin_dsc_entry)
{
	uint8_t temp_screen_id = 64;
	// ??????????›¥????????????
	uint8_t temp_cabin_id = cabin_dsc_entry->curr_cabin_id;

	if(cang_sxzt[temp_cabin_id] != 0)
	{
		if(cang_sxzt[temp_cabin_id] == 1 || cabin_dsc_entry->force_fresh_flag == 1) // ???????
		{
			if(Cang_zx_buf[temp_cabin_id] == CabinDisconnectCount)
			{
				uint8_t temp_buff[32] = {0};
				sprintf((char *)temp_buff,"?????%d??:????", temp_cabin_id);
				SetTextValue(temp_screen_id, 2, temp_buff);
				SetTextValue(temp_screen_id, 3, "????????:--");
				SetTextValue(temp_screen_id, 4, "????????????:--");
				SetTextValue(temp_screen_id, 5, "???:--");
				SetTextValue(temp_screen_id, 6, "??????:--");
				SetTextValue(temp_screen_id, 7, "?????????:--");
				SetTextValue(temp_screen_id, 8, "???????:--");
			}
			else
			{
				uint8_t temp_buff[32] = {0};
				sprintf((char *)temp_buff,"?????%d??:????", temp_cabin_id);
				SetTextValue(temp_screen_id, 2, temp_buff);
				
				// ???????????
				if(cabin_dsc_entry->last_detector_model != Cang_TCQXH_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					switch(Cang_TCQXH_buf[temp_cabin_id])
					{
						case 1: {
							SetTextValue(temp_screen_id, 3, "????????:XR805-V2.0");
							break;
						}
						case 2: {
							SetTextValue(temp_screen_id, 3, "????????:XR805-EXD");
							break;
						}
						case 3: {
							SetTextValue(temp_screen_id, 3, "????????:XR805-EXi");
							break;
						}
						default:
							SetTextValue(temp_screen_id, 3, "????????:--");
							break;
					}
					
					cabin_dsc_entry->last_detector_model = Cang_TCQXH_buf[temp_cabin_id];
				}
				
				if(cabin_dsc_entry->last_sensor_mode != Cang_CGQQY_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					if(Cang_CGQQY_buf[temp_cabin_id] == 0)
					{
						SetTextValue(temp_screen_id, 4, "????????????:??????????");
					}
					else
					{
						uint8_t temp = Cang_CGQQY_buf[temp_cabin_id];
						uint8_t temp_buff[128] = {0};
						
						uint8_t first_sensor = 1;  // ??????????????????
						
						uint8_t pos = 0;
						
						pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
//						SetTextValue(temp_screen_id, 4, temp_buff);
						for(uint8_t i = 0; i < 6; i++)
						{
							if( (temp >> i) & 0x01 )
							{
								// ???????????????????
								if(!first_sensor)
								{
										pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "/");
								}
								else
								{
										first_sensor = 0;
								}
								pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[i]);
							}
						}
						SetTextValue(temp_screen_id, 4, temp_buff);
					}
					cabin_dsc_entry->last_sensor_mode = Cang_CGQQY_buf[temp_cabin_id];
				} // ?????????????§Ø?
				
				if(cabin_dsc_entry->last_temperat_value != Cang_wendu_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					uint8_t temp_buff[16];
					cabin_dsc_entry->last_temperat_value = Cang_wendu_buf[temp_cabin_id];
					sprintf((char *)temp_buff, "???:%d??", Cang_wendu_buf[temp_cabin_id]);
					SetTextValue(temp_screen_id, 5, temp_buff);
				} // ???????
				
				if(cabin_dsc_entry->last_smoke_state != Cang_YWZT_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					if(Cang_YWZT_buf[temp_cabin_id] == 0)
					{
						SetTextValue(temp_screen_id, 6, "??????:????");
					}
					else
					{
						SetTextValue(temp_screen_id, 6, "??????:????");
					}
					cabin_dsc_entry->last_smoke_state = Cang_YWZT_buf[temp_cabin_id];
				} // ?????????
				
				if(cabin_dsc_entry->last_co_value != Cang_COzhi_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					uint8_t temp_buff[16];
					cabin_dsc_entry->last_temperat_value = Cang_COzhi_buf[temp_cabin_id];
					sprintf((char *)temp_buff, "?????????:%dPPM", Cang_COzhi_buf[temp_cabin_id]);
					SetTextValue(temp_screen_id, 7, temp_buff);
					
					cabin_dsc_entry->last_co_value = Cang_COzhi_buf[temp_cabin_id];
				} // ????????????
				
				if(cabin_dsc_entry->last_hh_value != Cang_H2zhi_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					uint8_t temp_buff[16];
					cabin_dsc_entry->last_temperat_value = Cang_H2zhi_buf[temp_cabin_id];
					sprintf((char *)temp_buff, "???????:%dPPM", Cang_H2zhi_buf[temp_cabin_id]);
					SetTextValue(temp_screen_id, 8, temp_buff);
					cabin_dsc_entry->last_hh_value = Cang_H2zhi_buf[temp_cabin_id];
				} // ??????????
			}
		}

		cabin_dsc_entry->lasr_cabin_state = cang_sxzt[temp_cabin_id];
	}
	else
	{
		if(cabin_dsc_entry->force_fresh_flag == 1)
		{
			uint8_t temp_buff[32] = {0};
			sprintf((char *)temp_buff,"?????%d??:¦Ä????", temp_cabin_id);
			SetTextValue(temp_screen_id, 2, temp_buff);
			SetTextValue(temp_screen_id, 3, "????????:--");
			SetTextValue(temp_screen_id, 4, "????????????:--");
			SetTextValue(temp_screen_id, 5, "???:--");
			SetTextValue(temp_screen_id, 6, "??????:--");
			SetTextValue(temp_screen_id, 7, "?????????:--");
			SetTextValue(temp_screen_id, 8, "???????:--");
		}
		
	}
	
	if(cabin_dsc_entry->force_fresh_flag == 1)
	{
		cabin_dsc_entry->force_fresh_flag = 0;
	}
}

static uint8_t GetGasSummaryAlarmLevel(uint8_t state, uint8_t *level)
{
    if (state == 0U)
    {
        *level = GAS_SUMMARY_LEVEL_NORMAL;
        return 1U;
    }
    if (state == 1U)
    {
        *level = GAS_SUMMARY_LEVEL_WARNING;
        return 1U;
    }
    if (state == 2U)
    {
        *level = GAS_SUMMARY_LEVEL_FIRE;
        return 1U;
    }
    return 0U;
}

static void UpdateGasSummaryCandidate(GasConcentrationCandidate *best, uint8_t loop, uint8_t addr, uint8_t gas_type, uint8_t level, uint16_t value)
{
    if (best->valid == 0U || level > best->alarm_level || (level == best->alarm_level && value > best->value))
    {
        best->valid = 1U;
        best->loop = loop;
        best->addr = addr;
        best->gas_type = gas_type;
        best->alarm_level = level;
        best->value = value;
    }
}

static void CollectLoop1GasConcentrationCandidates(GasConcentrationCandidate *best, uint8_t *enabled_sensor)
{
    (void)best;
    (void)enabled_sensor;
    /* XR5000_GAS_SUMMARY_CHANGE_20260731: loop 1 currently has smoke/temperature devices only. */
}

static void CollectLoop3GasConcentrationCandidates(GasConcentrationCandidate *best, uint8_t *enabled_sensor)
{
    uint8_t addr;

    for (addr = 1U; addr < RS485_DETECT_MAX_DEVICES; addr++)
    {
        uint16_t sensor_enable;
        uint8_t level;

        if (RS485Detect_GetOnline(addr) == 0U || RS485Detect_HasSensorEnableData(addr) == 0U)
        {
            continue;
        }

        sensor_enable = RS485Detect_GetSensorEnable(addr);
        if ((sensor_enable & (1U << 4)) != 0U)
        {
            *enabled_sensor = 1U;
            if (RS485Detect_IsOnline(addr) != 0U && RS485Detect_HasSensorData(addr) != 0U &&
                GetGasSummaryAlarmLevel(RS485Detect_GetSensorState(addr, RS485_SENSOR_CO), &level) != 0U)
            {
                UpdateGasSummaryCandidate(best, 3U, addr, GAS_SUMMARY_TYPE_CO, level, RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO));
            }
        }

        if ((sensor_enable & (1U << 2)) != 0U)
        {
            *enabled_sensor = 1U;
            if (RS485Detect_IsOnline(addr) != 0U && RS485Detect_HasSensorData(addr) != 0U &&
                GetGasSummaryAlarmLevel(RS485Detect_GetSensorState(addr, RS485_SENSOR_H2), &level) != 0U)
            {
                UpdateGasSummaryCandidate(best, 3U, addr, GAS_SUMMARY_TYPE_H2, level, RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2));
            }
        }
    }
}

static void RefreshGasConcentrationSummary(void)
{
    GasConcentrationCandidate best = {0};
    uint8_t enabled_sensor = 0U;
    uint8_t temp_buff[64];

    if (gas_concentration_summary_fresh_flag == 0U)
    {
        return;
    }
    gas_concentration_summary_fresh_flag = 0U;

    CollectLoop1GasConcentrationCandidates(&best, &enabled_sensor);
    CollectLoop3GasConcentrationCandidates(&best, &enabled_sensor);

    if (best.valid == 0U)
    {
        if (enabled_sensor == 0U)
        {
            SetTextValue(59, 26, "\xCE\xB4\xC6\xF4\xD3\xC3" "CO/H2" "\xCC\xBD\xB2\xE2\xC6\xF7");
        }
        else
        {
            SetTextValue(59, 26, "CO/H2" "\xCC\xBD\xB2\xE2\xC6\xF7\xCE\xDE\xD3\xD0\xD0\xA7\xD4\xDA\xCF\xDF\xCA\xFD\xBE\xDD");
        }
        return;
    }

    if (best.gas_type == GAS_SUMMARY_TYPE_H2)
    {
        sprintf((char *)temp_buff, "\xB5\xDA%d\xBB\xD8\xC2\xB7 %03d\xBA\xC5 H2\xC5\xA8\xB6\xC8\xA3\xBA%dPPM", best.loop, best.addr, best.value);
    }
    else
    {
        sprintf((char *)temp_buff, "\xB5\xDA%d\xBB\xD8\xC2\xB7 %03d\xBA\xC5 CO\xC5\xA8\xB6\xC8\xA3\xBA%dPPM", best.loop, best.addr, best.value);
    }
    SetTextValue(59, 26, temp_buff);
}

#if 0 /* XR5000_GAS_SUMMARY_CHANGE_20260731: legacy renderer retained for later cleanup. */
static void DetectorShowMaxCombusteGasValue(MaxCombustibleGas_t *mcg_entry)
{
	uint8_t temp_screen_id = 59;
	if(max_combustible_gas_fresh_flag == 1)
	{
		max_combustible_gas_fresh_flag = 0;
		
		MaxCombustibleGas_t temp_mcg = {0};
		temp_mcg.co_max_val = -1;
		
		for(uint8_t i = 0; i < GAS_ID_SUM; i++)
		{
			if(mcg_entry[i].co_max_val > temp_mcg.co_max_val)
			{
				temp_mcg.co_max_val = mcg_entry[i].co_max_val;
				temp_mcg.curr_da    = mcg_entry[i].curr_da;
				temp_mcg.gas_type   = mcg_entry[i].gas_type;
			}
		}

		// ?????????
		if(temp_mcg.co_max_val == -1) // ??????????????? ??????????????
		{
			SetTextValue(temp_screen_id, 26, "?????¦Ä????/?????/?????????");
		}
		else
		{
			uint8_t temp_buff[48];
			
			if(temp_mcg.curr_da.cluster_id == 0 && temp_mcg.curr_da.pack_id == 0 && temp_mcg.curr_da.cabin_id != 0)
			{
				if(Hydrogen_Type == temp_mcg.gas_type)
				{
					sprintf((char *)temp_buff, "??1??¡¤ %d?? ???????%dPPM", temp_mcg.curr_da.cabin_id, temp_mcg.co_max_val);
					SetTextValue(temp_screen_id, 26, temp_buff);
				}
				else
				{
					sprintf((char *)temp_buff, "??1??¡¤ %d?? ?????????%dPPM", temp_mcg.curr_da.cabin_id, temp_mcg.co_max_val);
					SetTextValue(temp_screen_id, 26, temp_buff);
				}
				
			}
			else if(temp_mcg.curr_da.cluster_id != 0 && temp_mcg.curr_da.pack_id != 0 && temp_mcg.curr_da.cabin_id == 0)
			{
				sprintf((char *)temp_buff, "??%d??pack%d?????????%dPPM", temp_mcg.curr_da.cluster_id, temp_mcg.curr_da.pack_id, temp_mcg.co_max_val);
				SetTextValue(temp_screen_id, 26, temp_buff);
			}
			else
			{
				SetTextValue(temp_screen_id, 26, "??????????????");
			}
		}
		
	}
	
}

#endif /* XR5000_GAS_SUMMARY_CHANGE_20260731 */

static void DetectorDataFreshMenuCtrl(DetectorDataShowCtrl *ddsc_entry, uint16_t ctrl_id, uint8_t item, uint8_t state)
{
	if(ctrl_id == 103)
	{
		if(state == 1) // ????????
		{
			ddsc_entry->curr_detector_page = item + 1;
		}
	}
}

static void DetectorDataFreshMenuCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t item, uint8_t state)
{
	if(ctrl_id == 103)
	{
		if(state == 1) // ????????
		{
			ddsc_32p_entry->curr_detector_page = item + 1;
			ddsc_32p_entry->force_fresh_flag = 1; // ?????¡À??¦Ë
		}
	}
}

static void DetectorFreshPageButtonCtrl(DetectorDataShowCtrl *ddsc_entry, uint16_t ctrl_id, uint8_t state)
{
	if(ctrl_id == 106)
	{
		if(state == 1)
		{
			if(ddsc_entry->curr_detector_page > 1)
				ddsc_entry->curr_detector_page--;
		}
	}
	else if(ctrl_id == 107)
	{
		if(state == 1)
		{
			if(ddsc_entry->curr_detector_page < 20)
				ddsc_entry->curr_detector_page++;
		}
	}
}

static void DetectorFreshPageButtonCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t state)
{
	if(ctrl_id == 106)
	{
		if(state == 1)
		{
			if(ddsc_32p_entry->curr_detector_page > 1)
			{
				ddsc_32p_entry->curr_detector_page--;
				ddsc_32p_entry->force_fresh_flag = 1; // ?????¡À??¦Ë
			}
				
		}
	}
	else if(ctrl_id == 107)
	{
		if(state == 1)
		{
			if(ddsc_32p_entry->curr_detector_page < 3)
			{
				ddsc_32p_entry->curr_detector_page++;
				ddsc_32p_entry->force_fresh_flag = 1; // ?????¡À??¦Ë
			}
				
		}
	}
}

const uint8_t button_value_map[] = {
	  5,   9,  13,  17,  21,  25,  29,  33, 
	 37,  41,  45,  49,  53,  57,  61,  65, 
	 69,  73,  77,  81,  85,  89,  93,  97, 
	101, 111, 115, 119, 123, 127, 131, 136,};
// 1??32pack?·Ú ???PACK???
static void DetectorMonitorButtonCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		for(uint8_t i = 0; i < 32; i++)
		{
			if(button_value_map[i] == ctrl_id)
			{
				ddsc_32p_entry->curr_pack_id = i + 1;
				ddsc_32p_entry->force_fresh_flag = 1; // ?????¡À??¦Ë
				SetScreen(62);	// 
				osDelay(5);
				GetScreen();
				break;
			}
		}
	}
}

static void CabinFreshPageButtonCtrl(CabinDataShowCtrl_t *cabin_dsc_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		cabin_dsc_entry->curr_cabin_id = ctrl_id;
		
		cabin_dsc_entry->force_fresh_flag = 1;
		
		SetScreen(64);	// 
		osDelay(5);
		GetScreen();
	}
}

static void PointTypeDetectorOnlineButtonCtrl(uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		for(uint8_t i = 0; i < 32; i++)
		{
			if(point_type_detect_button_online_ctrl_val_map[i] == ctrl_id)
			{
				PointTypeMixtureOnlieStateSingleSetting(i + 1, 1);
				SavePointTypeSetOnlieState();
				ReadPointTypeSetOnlieState();
				break;
			}
		}
	}
	else if(state == 0)
	{
		for(uint8_t i = 0; i < 32; i++)
		{
			if(point_type_detect_button_online_ctrl_val_map[i] == ctrl_id)
			{
				PointTypeMixtureOnlieStateSingleSetting(i + 1, 0);
				SavePointTypeSetOnlieState();
				ReadPointTypeSetOnlieState();
				break;
			}
		}
	}
}

// 2026/01/21 16:53
static void PointTypeDetectorOnlineIconCtrl(uint16_t ctrl_id, uint8_t state, uint8_t icon_num)
{
	if(ctrl_id == 0 || ctrl_id > 32)
	{
		return;
	}
	
	if(state == 1)
	{
		uint8_t set_online_state = !getPointTypeMixtureDetectOnlineState(ctrl_id);
		PointTypeMixtureOnlieStateSingleSetting(ctrl_id, set_online_state);
		AnimationPlayFrame(5, ctrl_id, set_online_state);//(????ID,???ID,?ID) 0??1??
		
		SavePointTypeSetOnlieState();
		ReadPointTypeSetOnlieState();
	}
}

// 2026/01/21 17:16
static void PointTypeDetectorOnlineButtonCtrlPlus(uint16_t ctrl_id, uint8_t state)
{
	if(state != 1)
	{
		return;
	}
	switch(ctrl_id)
	{
		case 233: {
			for(uint8_t i = 1; i < 33; i++)
			{
				PointTypeMixtureOnlieStateSingleSetting(i, 1);
				AnimationPlayFrame(5, i, 1);//(????ID,???ID,?ID) 0??1??
			}
			SavePointTypeSetOnlieState();
			ReadPointTypeSetOnlieState();
			break;
		}
		case 234: {
			for(uint8_t i = 1; i < 33; i++)
			{
				PointTypeMixtureOnlieStateSingleSetting(i, 0);
				AnimationPlayFrame(5, i, 0);//(????ID,???ID,?ID) 0??1??
			}
			SavePointTypeSetOnlieState();
			ReadPointTypeSetOnlieState();
			break;
		}
	}
}

// 2026/01/21 17:30
static void PointTypeDetectorOnlineTextCtrlPlus(uint16_t ctrl_id, uint8 *entry_str)
{
	switch(ctrl_id)
	{
		case 235:{
			uint8_t temp_str_len = strlen((char *)entry_str);
			if(temp_str_len != 0)
			{
				int8_t success_len;
				int32_t x,y;
				success_len = sscanf((const char*)entry_str, "%d.%d", &x, &y);  
				if(success_len == 2 && x >0 && y > 0) // ??????? ????x y??????
				{
					uint8_t modify_flag = 0;
					if (x > y)
					{
						x ^= y;
						y ^= x;
						x ^= y;
					}
					for(uint8_t i = x; i < y + 1; i++)
					{
						if(getPointTypeMixtureDetectOnlineState(i) == 0)
						{
							PointTypeMixtureOnlieStateSingleSetting(i, 1);
							AnimationPlayFrame(5, i, 1);//(????ID,???ID,?ID) 0??1??
							modify_flag = 1;	
						}
					}
					if(modify_flag == 1)
					{
						SavePointTypeSetOnlieState();
					}
				}
			}
			SetTextValue(5, 235, "????????");
			break;
		}
		case 236:{
			uint8_t temp_str_len = strlen((char *)entry_str);
			if(temp_str_len != 0)
			{
				int8_t success_len;
				int32_t x;
				success_len = sscanf((const char*)entry_str, "%d", &x);  
				if(success_len == 2 && x >0) // ??????? ????x??????
				{
					uint8_t modify_flag = 0;

					if(getPointTypeMixtureDetectOnlineState(x) == 0)
					{
						PointTypeMixtureOnlieStateSingleSetting(x, 1);
						AnimationPlayFrame(5, x, 1);//(????ID,???ID,?ID) 0??1??
						modify_flag = 1;	
					}
					if(modify_flag == 1)
					{
						SavePointTypeSetOnlieState();
					}
				}
			}
			SetTextValue(5, 236, "????????");
			break;
		}
		default:
			break;
	}
}

// 2026/ 01/21 17:52
static void PointTypeDetectorOnlineStateShowInit(void)
{
	for(uint8_t i = 1; i < 33; i++)
	{
		AnimationPlayFrame(5, i, getPointTypeMixtureDetectOnlineState(i));//(????ID,???ID,?ID) 0??1??
	}
}

void StoragePackCabinForeWarn(PackCabinForeWarnStorage *pcfws_entry, uint8_t cluster_id, uint8_t pack_id, uint8_t alarm_type)
{
	uint8_t flag = 0;
	for(uint8_t l = 0;l < pcfws_entry->self_bottom_point; l++)
	{
		// ?????ID???????õôID 
		if(pcfws_entry->da[l].cluster_id == LINKAGE_CLUSTER_ID)
		{
			// ???????????????§Ü??????????? ???????
			if(pcfws_entry->da[l].pack_id == pack_id) // ??????§Ø?????????????? ???????????
			{
				flag = 1;
				break;
			}
		}
		else if(pcfws_entry->da[l].cluster_id != 0)
		{
			// ???????????????§Ü??????????? ???????
			if(pcfws_entry->da[l].cluster_id == cluster_id && pcfws_entry->da[l].pack_id == pack_id && pcfws_entry->alarm_type[l] == alarm_type)
			{
				flag = 1;
				break;
			}
		}
		else if(pcfws_entry->da[l].pack_id != 0)
		{
			if( pcfws_entry->da[l].cabin_id == pack_id &&
					pcfws_entry->alarm_type[l]  == alarm_type) // ???????????pack id
			{
				flag = 1;
				break;
			}
		}
	}
	// ?????????????? ??????¦Ã???
	if(flag != 1)
	{
		if(cluster_id == LINKAGE_CLUSTER_ID)
		{
			pcfws_entry->detector_class[pcfws_entry->self_bottom_point] = LinkageClassID;
			pcfws_entry->da[pcfws_entry->self_bottom_point].cabin_id    = 0;
			pcfws_entry->da[pcfws_entry->self_bottom_point].cluster_id  = cluster_id;
			pcfws_entry->da[pcfws_entry->self_bottom_point].pack_id     = pack_id;
		}
		else if(cluster_id != 0)
		{
			pcfws_entry->detector_class[pcfws_entry->self_bottom_point] = PackClassID;
			pcfws_entry->da[pcfws_entry->self_bottom_point].cabin_id    = 0;
			pcfws_entry->da[pcfws_entry->self_bottom_point].cluster_id  = cluster_id;
			pcfws_entry->da[pcfws_entry->self_bottom_point].pack_id     = pack_id;
		}
		else
		{
			pcfws_entry->detector_class[pcfws_entry->self_bottom_point] = CabinClassID;
			pcfws_entry->da[pcfws_entry->self_bottom_point].cabin_id    = pack_id;
			pcfws_entry->da[pcfws_entry->self_bottom_point].cluster_id  = 0;
			pcfws_entry->da[pcfws_entry->self_bottom_point].pack_id     = 0;
		}

		pcfws_entry->alarm_type[pcfws_entry->self_bottom_point]     = alarm_type;
		
		pcfws_entry->atr[pcfws_entry->self_bottom_point].years  = years + 2000;
		pcfws_entry->atr[pcfws_entry->self_bottom_point].months = months;
		pcfws_entry->atr[pcfws_entry->self_bottom_point].days   = days;
		pcfws_entry->atr[pcfws_entry->self_bottom_point].hours  = hours;
		pcfws_entry->atr[pcfws_entry->self_bottom_point].minute = minutes;
		
		// 2025/11/19 13:53 ?????????
		pcfws_entry->atr[pcfws_entry->self_bottom_point].second = secs;
		
		pcfws_entry->self_bottom_point++;
		
		beep_fire_ctrl |= 0x0F;  // ?????? ????
		silencers_state = 0; // ???¦Ì???? ???????? ?????????¦Ë
	}
}

// ??????????
void DeletPackCabinForeWarn(PackCabinForeWarnStorage *pcfws_entry, uint8_t cluster_id, uint8_t pack_id, uint8_t alarm_type)
{
	uint8_t l;
	uint8_t flag = 0;
	for(l = 0;l < pcfws_entry->self_bottom_point; l++)
	{
		if(pcfws_entry->detector_class[l] == PackClassID)
		{
			if(pcfws_entry->da[l].cluster_id == cluster_id && pcfws_entry->da[l].pack_id == pack_id && pcfws_entry->alarm_type[l] == alarm_type)
			{
				flag = 1;
				break;
			}
		}
	}
	if(flag == 1 && pcfws_entry->self_bottom_point > 0) // ????????????????????????????
	{
		for(;l < pcfws_entry->self_bottom_point - 1; l++)
		{
			pcfws_entry->detector_class[l] = pcfws_entry->detector_class[l + 1];
			pcfws_entry->da[l]         = pcfws_entry->da[l + 1]; 
			pcfws_entry->atr[l]        = pcfws_entry->atr[l + 1];
			pcfws_entry->alarm_type[l] = pcfws_entry->alarm_type[l + 1];
		}
		pcfws_entry->self_bottom_point--;
		
		if(pcfws_entry->self_bottom_point > 0) // ??????????????????
		{
			beep_fire_ctrl |= 0x0F; // ???????????
			silencers_state = 0; // ???¦Ì???? ???????? ?????????¦Ë
		}
	}
}

void StorageCabinFireAlarm(PackCabinFireAlarmStorage *pcfas_entry, 
	uint8_t cabin_id, 
	uint8_t alarm_type)
{
		uint8_t flag = 0;
	for(uint8_t l = 0;l < pcfas_entry->self_bottom_point; l++)
	{
		if(pcfas_entry->detector_class[l] == CabinClassID) // ???????????§Ø?
		{
			if(pcfas_entry->da[l].cabin_id == cabin_id && pcfas_entry->alarm_type[l] == alarm_type)
			{
				flag = 1;
				break;
			}
		}
	}
	
	if(flag != 1)
	{
		pcfas_entry->detector_class[pcfas_entry->self_bottom_point] = PackClassID;
		pcfas_entry->da[pcfas_entry->self_bottom_point].cabin_id    = cabin_id;
		pcfas_entry->da[pcfas_entry->self_bottom_point].cluster_id  = 0;
		pcfas_entry->da[pcfas_entry->self_bottom_point].pack_id     = 0;
		pcfas_entry->alarm_type[pcfas_entry->self_bottom_point] = alarm_type;

		pcfas_entry->atr[pcfas_entry->self_bottom_point].years  = years + 2000;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].months = months;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].days   = days;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].hours  = hours;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].minute = minutes;
		
		//  2025/11/19 13:51
		pcfas_entry->atr[pcfas_entry->self_bottom_point].second = secs;
		
		pcfas_entry->self_bottom_point++;
		
	}
}

void StoragePackFireAlarm(
	PackCabinFireAlarmStorage *pcfas_entry, // ??????????????
	uint8_t cluster_id,                     // ??
	uint8_t pack_id,                        // ??/??
	uint8_t alarm_type                      // ????????
)
{
	uint8_t flag = 0;
	for(uint8_t l = 0;l < pcfas_entry->self_bottom_point; l++)
	{
		if(pcfas_entry->da[l].cluster_id == LINKAGE_CLUSTER_ID)
		{
			if(pcfas_entry->da[l].pack_id == pack_id)
			{
				flag = 1;
				break;
			}
		}
		else if(pcfas_entry->da[l].cluster_id != 0)
		{
			if( pcfas_entry->da[l].cluster_id == cluster_id && 
					pcfas_entry->da[l].pack_id    == pack_id    && 
					pcfas_entry->alarm_type[l]    == alarm_type)
			{
				flag = 1;
				break;
			}
		}
		else // ??
		{
			if( pcfas_entry->da[l].cabin_id == pack_id &&
					pcfas_entry->alarm_type[l]  == alarm_type) // ???????????pack id
			{
				flag = 1;
				break;
			}
		}
	}
	
	if(flag != 1)
	{
		if(cluster_id == LINKAGE_CLUSTER_ID) // ??????????õôID
		{
			pcfas_entry->detector_class[pcfas_entry->self_bottom_point] = LinkageClassID;
			pcfas_entry->da[pcfas_entry->self_bottom_point].cabin_id    = 0;
			pcfas_entry->da[pcfas_entry->self_bottom_point].cluster_id  = cluster_id;
			pcfas_entry->da[pcfas_entry->self_bottom_point].pack_id     = pack_id;
		}
		else if(cluster_id != 0)
		{
			pcfas_entry->detector_class[pcfas_entry->self_bottom_point] = PackClassID;
			pcfas_entry->da[pcfas_entry->self_bottom_point].cabin_id    = 0;
			pcfas_entry->da[pcfas_entry->self_bottom_point].cluster_id  = cluster_id;
			pcfas_entry->da[pcfas_entry->self_bottom_point].pack_id     = pack_id;
		}
		else
		{
			pcfas_entry->detector_class[pcfas_entry->self_bottom_point] = CabinClassID;
			pcfas_entry->da[pcfas_entry->self_bottom_point].cabin_id    = pack_id;
			pcfas_entry->da[pcfas_entry->self_bottom_point].cluster_id  = 0;
			pcfas_entry->da[pcfas_entry->self_bottom_point].pack_id     = 0;
		}

		pcfas_entry->alarm_type[pcfas_entry->self_bottom_point] = alarm_type;
		
		pcfas_entry->atr[pcfas_entry->self_bottom_point].years  = years + 2000;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].months = months;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].days   = days;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].hours  = hours;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].minute = minutes;
		pcfas_entry->atr[pcfas_entry->self_bottom_point].second = secs;
		
		pcfas_entry->self_bottom_point++;
	}
}

static uint8_t getShieldDetectorSum(uint8_t pack_shield[][33], uint8_t cabin_shield[])
{
	uint8_t shield_sum = 0;
	// ????????????
	for(uint8_t i = 1;i < 21;i++)
	{
		for(uint8_t j = 1;j < 11;j++)
		{
			shield_sum = shield_sum + pack_shield[i][j];
		}
	}
	
	for(uint8_t i = 1;i < 25; i++)
	{
		shield_sum  = shield_sum + cabin_shield[i];
	}
	
	shielding_state = shield_sum;
	return shield_sum;
}

uint8_t partition_ctrl_id[4] = {8, 8, 16, 16};
uint8_t point_site_ctrl_id[4][7] = {
	{9, 10, 11, 12, 13, 14, 15},
	{9, 10, 11, 12, 13, 14, 15},
	{17, 18, 19, 20, 21, 22, 23},
	{17, 18, 19, 20, 21, 22, 23}
};

static void BspCheckNewKeyPressDeal(BspKeyCheckNewCtrl_t *bkcnc_entry)
{
	uint8_t temp_screen_id = 59;
	uint8_t temp_partition = bkcnc_entry->curr_partition; // ???????
	uint8_t prev_partition = bkcnc_entry->last_partition; // XR5000_CURSOR_FIX_20260726: capture the pre-mutation partition for correct stale-cursor clearing below.
	
	uint8_t key_val_temp;
	
	// ????????
	if(bkcnc_entry->curr_menu_state == InitMenu) // ???????????
	{
		for(uint8_t i = 0; i < 3; i+=2)
		{
			clearTextValue(temp_screen_id, partition_ctrl_id[i]); // ????????
			for(uint8_t j = 0; j < 7; j++)
			{
				clearTextValue(temp_screen_id, point_site_ctrl_id[i][j]); // ????????
			}
		}
		bkcnc_entry->curr_menu_state = OutMenu; // ?????????
		bkcnc_entry->last_menu_state = OutMenu;
		// ?????????? ??????
		bkcnc_entry->last_show_len[ForceAlarmPart] = pcfws.self_bottom_point;
		bkcnc_entry->last_show_len[FaultPart]      = pcfs_buttom_point;
		bkcnc_entry->last_show_len[FireAlarmPart]  = pcfas.self_bottom_point;
		bkcnc_entry->last_show_len[OutFirePart]    = fedas.self_point_len;
		
		bkcnc_entry->curr_partition = ForceAlarmPart;
		bkcnc_entry->last_partition = ForceAlarmPart;
		
		bkcnc_entry->last_point_site[temp_partition] = bkcnc_entry->curr_point_site[temp_partition];
		SetTextValue(temp_screen_id, partition_ctrl_id[ForceAlarmPart], "<-");
	}
	
	// ???¦Ë?????
	
	// ????????? - ??????
	if(((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) > bkcnc_entry->last_show_len[FaultPart])
	{
		// ????????????????? ???¦Ë?¨°????
		bkcnc_entry->last_show_len[FaultPart] = (uint8_t)((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount());
	}
	else if(((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) < bkcnc_entry->last_show_len[FaultPart])
	{
		// ???????????????????§Ý????
		uint8_t new_count = (uint8_t)((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount());
			
		// ??????????¦Ë??????????????¦¶
		if (bkcnc_entry->curr_point_site[FaultPart] >= new_count) 
		{
			bkcnc_entry->curr_point_site[FaultPart] = (new_count > 0) ? new_count - 1 : 0;
		}
			
		// ?????????????????????????§¹
		if (fault_current_page + Fault_Show_Zone > new_count) 
		{
			if (new_count > Fault_Show_Zone) 
			{
				fault_current_page = new_count - Fault_Show_Zone;
			}
			else 
			{
				fault_current_page = 0;
			}
			fault_check_new_flag = 1;
		}
		
		bkcnc_entry->last_show_len[FaultPart] = new_count;
	}
	
	// ????????? - ??????
	if(pcfws.self_bottom_point > bkcnc_entry->last_show_len[ForceAlarmPart])
	{
		// ???????????????? ???¦Ë?¨°????
		bkcnc_entry->last_show_len[ForceAlarmPart] = pcfws.self_bottom_point;
	}
	else if(pcfws.self_bottom_point < bkcnc_entry->last_show_len[ForceAlarmPart])
	{
		// ??????????????????§Ý????
		uint8_t new_count = pcfws.self_bottom_point;
		
		// ??????????¦Ë??????????????¦¶
		if (bkcnc_entry->curr_point_site[ForceAlarmPart] >= new_count) 
		{
			bkcnc_entry->curr_point_site[ForceAlarmPart] = (new_count > 0) ? new_count - 1 : 0;
		}
			
		// ?????????????????????????§¹
		if (fore_alarm_start_index + Alarm_Show_Zone > new_count) 
		{
			if (new_count > Alarm_Show_Zone) 
			{
				fore_alarm_start_index = new_count - Alarm_Show_Zone;
			}
			else 
			{
				fore_alarm_start_index = 0;
			}
			force_alarm_check_new_flag = 1;
		}
		
		bkcnc_entry->last_show_len[ForceAlarmPart] = new_count;
	}
	
	key_val_temp = getDirectionKeyValue();
	if(key_val_temp != NO_MATRIX_KEY_PRESS)
	{
		clearMatrixKeyValue();
	}
	// ???????? 
	switch(key_val_temp)
	{
		case KEY6_DIRECTION_UP   : // ????? ??
			if(bkcnc_entry->curr_menu_state == OutMenu) // ??????????????
			{
				if(temp_partition == FireAlarmPart) // ?????????????????
				{
					bkcnc_entry->curr_partition = ForceAlarmPart;
					temp_partition = ForceAlarmPart;
				}
				else if(temp_partition == OutFirePart)
				{
					bkcnc_entry->curr_partition = FaultPart;
					temp_partition = FaultPart;
				}
			}
			else // ??????????? 
			{
				if(temp_partition == ForceAlarmPart) // ????????????
				{
					if(pcfws.self_bottom_point > 0)
					{
						// 
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // ??????
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0) // ???????
						{
							if(fore_alarm_start_index > 0)
							{
								fore_alarm_start_index--;
								force_alarm_check_new_flag = 1;
							}
						}
					}
				}
				else if(temp_partition == FaultPart) // ??????????
				{
					// ????§Û??????
					if(pcfs_buttom_point > 0)
					{
						// ?????????????????? ????????????????
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // ??????
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0)
						{
							// ?????? ??????
							if (fault_current_page > 0) 
							{ 
								fault_current_page--;
								fault_check_new_flag = 1; // ??????
							} 
						}
						
						// ?????????????
//						else {
//							// ?????????
//							fault_current_page = pcfs_buttom_point - Fault_Show_Zone;
//							// ?????????????????
//							if (fault_current_page < 0) {
//									fault_current_page = 0; // ?????????????????????????
//							}
//						}
						
					}
					else
					{
						// ?????????????????
						bkcnc_entry->curr_point_site[temp_partition] = 0;
					}
				}
				else if(temp_partition == FireAlarmPart) // ??????
				{
					// ????§Û??????
					if(pcfas.self_bottom_point > 0) // ?§Ý?
					{
						// ?????????????????? ????????????????
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // ??????
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0)
						{
							// ?????? ??????
							if (fire_alarm_start_index > 0) 
							{ 
								fire_alarm_start_index--;
								fire_alarm_check_new_flag = 1; // ??????
							} 
						}
					}
					
				}
				else if(temp_partition == OutFirePart) // ???????
				{
					if(fedas.self_point_len > 0)
					{
						// ?????????????????? ????????????????
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // ??????
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0)
						{
							// ?????? ??????
							if (fedas_fresh_point > 0) 
							{ 
								fedas_fresh_point--;
								fed_fresh_flag = 1; // ??????
							} 
						}
					}
				}
			}
			break;
		case KEY7_DIRECTION_RIGHT: // ???????? ?? ????
			if(temp_partition == ForceAlarmPart) // ???????????????
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // ?????????
				{
					bkcnc_entry->curr_menu_state = OutMenu; // ??????
				}
				else // ????????????
				{
					bkcnc_entry->curr_partition = FaultPart; // ?§Ý?????
					temp_partition = FaultPart;
				}
			}
			else if(temp_partition == FireAlarmPart)
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // ?????????
				{
					bkcnc_entry->curr_menu_state = OutMenu; // ??????
				}
				else 
				{
					bkcnc_entry->curr_partition = OutFirePart;
					temp_partition = OutFirePart;
				}
			}
			else if(temp_partition == FaultPart || temp_partition == OutFirePart)
			{
				if(bkcnc_entry->curr_menu_state == OutMenu) // ?????
				{
					bkcnc_entry->curr_menu_state = InMenu; //  ????????
				}
				else
				{
					bkcnc_entry->curr_menu_state = OutMenu; //  ??????
				}
			}
			break;
		case KEY8_DIRECTION_DOWN : // ???????
			if(bkcnc_entry->curr_menu_state == OutMenu) // ??????????????
			{
				if(temp_partition == ForceAlarmPart) // ???????????????????
				{
					bkcnc_entry->curr_partition = FireAlarmPart;
					temp_partition = FireAlarmPart;
				}
				else if(temp_partition == FaultPart)
				{
					bkcnc_entry->curr_partition = OutFirePart;
					temp_partition = OutFirePart;
				}
			}
			else
			{
				if(temp_partition == ForceAlarmPart) // ????????????
				{
					// ????????
					if(pcfws.self_bottom_point > 0)
					{
						// ??? ??????¦Ë???????????
						if(bkcnc_entry->curr_point_site[temp_partition] < Alarm_Show_Zone - 1) 
						{
							// ????????????????¦¶??????????????????´„
							if (bkcnc_entry->curr_point_site[temp_partition] < pcfws.self_bottom_point - 1) {
									bkcnc_entry->curr_point_site[temp_partition]++;
							}
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == Alarm_Show_Zone - 1)
						{
							// ?????? // ????+????????? §³?????? ????????????
							if (fore_alarm_start_index + Alarm_Show_Zone < pcfws.self_bottom_point) 
							{
								fore_alarm_start_index++;
								force_alarm_check_new_flag = 1;
							}
						}
					}
				}
				else if(temp_partition == FaultPart) // ??????????
				{
					if(pcfs_buttom_point > 0) // ????§Ò???
					{
						// ??? ??????¦Ë???????????
						if(bkcnc_entry->curr_point_site[temp_partition] < Fault_Show_Zone - 1) 
						{
//							bkcnc_entry->curr_point_site[temp_partition]++;
//							if(bkcnc_entry->curr_point_site[temp_partition] > pcfs_buttom_point - 1)
//							{
//								bkcnc_entry->curr_point_site[temp_partition] = pcfs_buttom_point - 1;
//							}
							// ????????????????¦¶??????????????????´„
							if (((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) > 0U && bkcnc_entry->curr_point_site[temp_partition] < ((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) - 1U) {
									bkcnc_entry->curr_point_site[temp_partition]++;
							}
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == Fault_Show_Zone - 1)
						{
							// ?????? // ????+????????? §³?????? ????????????
							if ((uint16_t)fault_current_page + Fault_Show_Zone < (uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) {
								fault_current_page++;
								fault_check_new_flag = 1; // ??????
							} 
					// ???????????
//					else {
//						fault_current_page = 0;
//					}
						}
					}
					else
					{
						bkcnc_entry->curr_point_site[temp_partition] = 0;
					}
				}
				else if(temp_partition == FireAlarmPart) // ??
				{
					if(pcfas.self_bottom_point > 0)
					{
						// ??? ??????¦Ë???????????
						if(bkcnc_entry->curr_point_site[temp_partition] < Alarm_Show_Zone - 1) 
						{
							if (bkcnc_entry->curr_point_site[temp_partition] < pcfas.self_bottom_point - 1) {
								bkcnc_entry->curr_point_site[temp_partition]++;
							}
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == Alarm_Show_Zone - 1)
						{
							if (fire_alarm_start_index + Alarm_Show_Zone < pcfas.self_bottom_point) {
								fire_alarm_start_index++;
								fire_alarm_check_new_flag = 1;
							} 
						}			
					} // ??????›¥?§Þ??
				}
				else if(temp_partition == OutFirePart) // ???????
				{
					if(fedas.self_point_len > 0)
					{
						// ??? ??????¦Ë???????????
						if(bkcnc_entry->curr_point_site[temp_partition] < Out_Fire_Show_Zone - 1) 
						{
							if (bkcnc_entry->curr_point_site[temp_partition] < fedas.self_point_len - 1) {
								bkcnc_entry->curr_point_site[temp_partition]++;
							}
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == Out_Fire_Show_Zone - 1)
						{
							if (fedas_fresh_point + Out_Fire_Show_Zone < fedas.self_point_len) 
							{
								fedas_fresh_point++;
								fed_fresh_flag = 1;
							} 
						}	
					} // ???????????›¥?§Þ??
					
				}
			}
			break;
		case KEY9_DIRECTION_LEFT : // ????? ?? ????
			if(temp_partition == FaultPart)
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // ?????????
				{
					bkcnc_entry->curr_menu_state = OutMenu; // ??????
				}
				else
				{
					bkcnc_entry->curr_partition = ForceAlarmPart;
					temp_partition = ForceAlarmPart;
				}
			}
			else if(temp_partition == OutFirePart)
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // ?????????
				{
					bkcnc_entry->curr_menu_state = OutMenu; // ??????
				}
				else
				{
					bkcnc_entry->curr_partition = FireAlarmPart;
					temp_partition = FireAlarmPart;
				}
			}
			else if(temp_partition == ForceAlarmPart || temp_partition == FireAlarmPart)
			{
				if(bkcnc_entry->curr_menu_state == OutMenu) // ?????
				{
					bkcnc_entry->curr_menu_state = InMenu; // ??????
				}
				else
				{
					bkcnc_entry->curr_menu_state = OutMenu; //  ??????
				}
			}
			break;
		case KEY10_DIRECTION_OK  :
			break;
	}
	
	// ?????????????
	if(temp_partition != bkcnc_entry->last_partition) // 
	{
		// ?????????????
		clearTextValue(temp_screen_id, partition_ctrl_id[bkcnc_entry->last_partition]);
		bkcnc_entry->last_partition = temp_partition; // ???????????

		switch(temp_partition)
		{
			case ForceAlarmPart:
				SetTextValue(temp_screen_id, partition_ctrl_id[temp_partition], "<-");
				break;
			case FaultPart     :
				SetTextValue(temp_screen_id, partition_ctrl_id[temp_partition], "->");
				break;
			case FireAlarmPart :
				SetTextValue(temp_screen_id, partition_ctrl_id[temp_partition], "<-");
				break;
			case OutFirePart   :
				SetTextValue(temp_screen_id, partition_ctrl_id[temp_partition], "->");
				break;
			default:
				break;
		}
	}
	
	// ???????????
	if( bkcnc_entry->last_point_site[temp_partition] != bkcnc_entry->curr_point_site[temp_partition] || // ??????????¦Ë????
			bkcnc_entry->last_menu_state                 != bkcnc_entry->curr_menu_state                    // ????????????????
	)
	{
		// ?????????
		clearTextValue(temp_screen_id, point_site_ctrl_id[ prev_partition % 4 ][ bkcnc_entry->last_point_site[prev_partition] % 7 ]); // XR5000_CURSOR_FIX_20260726: was indexing with the already-updated last_partition/temp_partition, so the stale arrow at the old partition/position never got cleared.
		if(bkcnc_entry->curr_menu_state == InMenu)
		{
			switch(temp_partition)
			{
				case ForceAlarmPart:
					SetTextValue(temp_screen_id, point_site_ctrl_id[temp_partition][bkcnc_entry->curr_point_site[temp_partition]], "<-");
					break;
				case FaultPart     :
					SetTextValue(temp_screen_id, point_site_ctrl_id[temp_partition][bkcnc_entry->curr_point_site[temp_partition]], "->");
					break;
				case FireAlarmPart :
					SetTextValue(temp_screen_id, point_site_ctrl_id[temp_partition][bkcnc_entry->curr_point_site[temp_partition]], "<-");
					break;
				case OutFirePart   :
					SetTextValue(temp_screen_id, point_site_ctrl_id[temp_partition][bkcnc_entry->curr_point_site[temp_partition]], "->");
					break;
				default:
					break;
			}
		}

		bkcnc_entry->last_point_site[temp_partition] = bkcnc_entry->curr_point_site[temp_partition];
		bkcnc_entry->last_menu_state = bkcnc_entry->curr_menu_state;
	}
}

void InternalSwitchInterfaceCtrlInit(void)
{
	switch_ui_ctrl.curr_pack_alarm_len = &pas_pointer; // ??????????
	switch_ui_ctrl.last_pack_alarm_len = 0;
	
	switch_ui_ctrl.curr_pc_fire_alarm_len = &pcfas.self_bottom_point;
	switch_ui_ctrl.last_pc_fire_alarm_len = 0;
	
	switch_ui_ctrl.curr_pc_fore_alarm_len = &pcfws.self_bottom_point;
	switch_ui_ctrl.last_pc_fore_alarm_len = 0;
	
	switch_ui_ctrl.curr_pc_fault_len = &pcfs_buttom_point;
	switch_ui_ctrl.last_pc_fault_len = 0;
	
	switch_ui_ctrl.curr_pc_outfire_len = &fedas.self_point_len;
	switch_ui_ctrl.last_pc_outfire_len = 0;
	
	switch_ui_ctrl.curr_sys_time = 0;
	switch_ui_ctrl.screen_light_flag = 0;
}

static void SyncMonitorSwitchSnapshot(void)
{
	 switch_ui_ctrl.last_pack_alarm_len    = pas_pointer;
	 switch_ui_ctrl.last_pc_fire_alarm_len = pcfas.self_bottom_point;
	 switch_ui_ctrl.last_pc_fore_alarm_len = pcfws.self_bottom_point;
	 switch_ui_ctrl.last_pc_fault_len      = pcfs_buttom_point;
	 switch_ui_ctrl.last_pc_outfire_len    = fedas.self_point_len;
}

static void InternalScreenMainInterfaceCtrl(SwitchInterfaceCtrl *sic_entry)
{
	if( sic_entry->curr_pack_alarm_len    == NULL ||
			sic_entry->curr_pc_fire_alarm_len == NULL || 
			sic_entry->curr_pc_fore_alarm_len == NULL ||
			sic_entry->curr_pc_fault_len      == NULL ||
			sic_entry->curr_pc_outfire_len    == NULL
	) // ????????????? ???????
	{
		return;
	}
	if( *(sic_entry->curr_pack_alarm_len   ) != 0 ||
			*(sic_entry->curr_pc_fire_alarm_len) != 0 || 
			*(sic_entry->curr_pc_fore_alarm_len) != 0 ||
			*(sic_entry->curr_pc_fault_len     ) != 0	||
			*(sic_entry->curr_pc_outfire_len   ) != 0
	) // ????????????? ???????
	{
		uint32_t curr_time = osKernelGetTickCount();
		if(curr_time - sic_entry->curr_sys_time >= 180000)
		{
			sic_entry->curr_sys_time = curr_time;
			// ?????????
			beiguangkai();
		}
	}
	
	if( *(sic_entry->curr_pack_alarm_len   ) != sic_entry->last_pack_alarm_len    || 
			*(sic_entry->curr_pc_fire_alarm_len) != sic_entry->last_pc_fire_alarm_len || 
			*(sic_entry->curr_pc_fore_alarm_len) != sic_entry->last_pc_fore_alarm_len ||
			*(sic_entry->curr_pc_fault_len     ) != sic_entry->last_pc_fault_len      ||
			*(sic_entry->curr_pc_outfire_len   ) != sic_entry->last_pc_outfire_len
	) // ????????????? ?????
	{
		sic_entry->last_pack_alarm_len    = *(sic_entry->curr_pack_alarm_len);
		sic_entry->last_pc_fire_alarm_len = *(sic_entry->curr_pc_fire_alarm_len);
		sic_entry->last_pc_fore_alarm_len = *(sic_entry->curr_pc_fore_alarm_len);
		sic_entry->last_pc_fault_len      = *(sic_entry->curr_pc_fault_len     );
		sic_entry->last_pc_outfire_len    = *(sic_entry->curr_pc_outfire_len   );

		// ?????????
		beiguangkai();
		// ???????????
		SetMonitorPageFrom(current_screen_id); /* XR5000_MONITOR_RETURN_NAV_CHANGE_20260802 */
	}
	
}

// ????????§Ý????—¤????????????????????—¨????????????
static void RecordSwitchButtonCtrl(BspScreenReadRecord_t *bsrr_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		switch(ctrl_id)
		{
			case 1: {
				int8_t temp_sector = 0;
				bsrr_entry->curr_show_type = RECORD_FAULT;
				SetScreen(57);	// ????????????
				// ????????
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(FAULT_FLASH_SAVE);
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //??????????
				bsrr_entry->force_fresh_flag = 1; // ??¦Í???????????????
				
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[0].byte_buff, 0);
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[1].byte_buff, 1);
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[2].byte_buff, 2);
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[3].byte_buff, 3);
				
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=500)
				{
					// ?????????¡À?
					temp_sector = temp_sum/500;
					// ??????????§Ö?????
					BspReadFlashData(FAULT_FLASH_SAVE, read_data[temp_sector].byte_buff, temp_sector);
				}
				osDelay(5);
				GetScreen();
				
				break;
			}
			case 2: {
				int8_t temp_sector = 0;
				bsrr_entry->curr_show_type = RECORD_ALARM;
				SetScreen(56);	// ?§Ý???????????
				
				// ????????
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(FIRE_FLASH_SAVE);
				SetTextInt32(56, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //??????????
				bsrr_entry->force_fresh_flag = 1; // ??¦Í???????????????
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=400)
				{
					// ?????????¡À?
					temp_sector = temp_sum/400;
					// ??????????§Ö?????
					BspReadFlashData(FIRE_FLASH_SAVE, read_data[temp_sector].byte_buff, temp_sector);
				}
				osDelay(5);
				GetScreen();
				
				break;
			}
				
			case 3: {
				int8_t temp_sector = 0;
				bsrr_entry->curr_show_type = RECORD_GASOF;
				SetScreen(57);	// 
				
				// ????????
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(GASER_FLASH_SAVE);
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //??????????
				bsrr_entry->force_fresh_flag = 1; // ??¦Í???????????????
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=500)
				{
					// ?????????¡À?
					temp_sector = temp_sum/500;
					// ??????????§Ö?????
					BspReadFlashData(GASER_FLASH_SAVE, read_data[temp_sector].byte_buff, temp_sector);
				}
				osDelay(5);
				GetScreen();
				
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //??????????
				
				break;
			}
				
			case 4: {
				int8_t temp_sector = 0;
				bsrr_entry->curr_show_type = RECORD_OTHER;
				SetScreen(57);	// 

				// ????????
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(OTHER_FLASH_SAVE);
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //??????????
				bsrr_entry->force_fresh_flag = 1; // ??¦Í???????????????
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=500)
				{
					// ?????????¡À?
					temp_sector = temp_sum/500;
					// ??????????§Ö?????
					BspReadFlashData(OTHER_FLASH_SAVE, read_data[temp_sector].byte_buff, temp_sector);
				}
				osDelay(5);
				GetScreen();
				
				break;
			}	
			default:
				break;
		}
		bsrr_entry->curr_page[bsrr_entry->curr_show_type] = 1; // ??¦Í?????????????
	}
}

// ?????§Ý???????????????
static void InternalScreenRecordShiftButtonCtrl(BspScreenReadRecord_t *bsrr_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		switch(ctrl_id)
		{
			case 1:
				if(bsrr_entry->curr_page[bsrr_entry->curr_show_type] > 1)
				{
					bsrr_entry->curr_page[bsrr_entry->curr_show_type]--;
					bsrr_entry->force_fresh_flag = 1; // ?????¡À??¦Ë
				}
				break;
			case 2:
				if(bsrr_entry->curr_page[bsrr_entry->curr_show_type] < (bsrr_entry->record_sum[bsrr_entry->curr_show_type] + RECORD_SHOW_ZONE - 1)/RECORD_SHOW_ZONE)
				{
					bsrr_entry->curr_page[bsrr_entry->curr_show_type]++;
					bsrr_entry->force_fresh_flag = 1;
				}
				break;
			default:
				break;
		}
	}
}

uint8_t serial_ctrl_id[10] = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
uint8_t device_ctrl_id[10] = {16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
uint8_t retime_ctrl_id[10] = {26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
uint8_t states_ctrl_id[10] = {36, 37, 38, 39, 40, 41, 42, 43, 44, 45};

uint8_t values_ctrl_id[10] = {86, 87, 88, 89, 90, 91, 92, 93, 94, 95};
// ??????,??????????????????????
static void InternalScreenShowRecord(BspScreenReadRecord_t *bsrr_entry)
{
	uint8_t temp_screen_id = 0;
	uint8_t x_sector; // ???????
	uint8_t temp_page = bsrr_entry->curr_page[bsrr_entry->curr_show_type]; // ?????????? ???????????
	
	if(bsrr_entry->force_fresh_flag == 1) // ?????¡À??¦Ë
	{
		uint8_t show_buff[32];
		uint16_t temp_caculate;
		uint16_t data_index;
		uint16_t start = (temp_page - 1)*RECORD_SHOW_ZONE; // ????¡À?
		uint16_t total_records = bsrr_entry->record_sum[bsrr_entry->curr_show_type];
		
		// ???????????????????
		temp_screen_id = (bsrr_entry->curr_show_type == RECORD_ALARM) ? 56 : 57;

		// ???????? ???????
		bsrr_entry->force_fresh_flag = 0;
		
		// ????????
		SetTextInt32(temp_screen_id, 97, temp_page, 0, 1);   
		
		for(uint16_t i = 0; i < RECORD_SHOW_ZONE; i++)
		{
			temp_caculate = start + i;
			if(temp_caculate < total_records) // ??????????§³??????
			{
				// ???????? total_records = 591 ?? ???? temp_page = 10 ???? start = ??10-1??*10 = 90
				// ????????i = 0??temp_caculate = start + i = 90??i= 1??temp_caculate = start + i = 91??...
				// reverse_index = 591 - 1 - 90 = 500??reverse_index = 591 - 1 - 91 = 499 
				uint16_t reverse_index = total_records - 1 - temp_caculate;
				// 
				switch(bsrr_entry->curr_show_type)
				{
					case RECORD_FAULT: {
						// ??????????????
						//DebugSendString(read_data[total_records/500].byte_buff, total_records * 8);
						//x_sector = total_records/500;
						x_sector = reverse_index/500;
						data_index = reverse_index%500;
						
						// ??????
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //

						// ?õô?????§Ø?
						if(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id != 0)
						{
							if(FormatRS485DetectFlashDeviceName(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id,
								read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id, show_buff) == 1)
							{
								// XR5000_LOOP3_CHANGE_20260726: Loop 3 history fault display uses "??3??¡¤ X??".
							}
							else if(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id == MBUS_CONTROL_FLASH_ID)
							{
								sprintf((char *)show_buff, "??2??¡¤ %d??", read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id);
							}
							else if(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
							{
								switch(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id)
								{
									case Deflate_Package_ID: {
										sprintf((char *)show_buff, "????????");
										break;
									}
									case SoundLt_Package_ID: {
										sprintf((char *)show_buff, "????????");
										break;
									}
									case SirenBk_Package_ID: {
										sprintf((char *)show_buff, "????/????");
										break;
									}
									case OutFir1_Package_ID: {
										sprintf((char *)show_buff, "??????1");
										break;
									}
									case OutFir2_Package_ID: {
										sprintf((char *)show_buff, "??????2");
										break;
									}
									case CabinBK_Package_ID: {
										sprintf((char *)show_buff, "??????");
										break;
									}
									case FEEDBK1_Package_ID: {
										sprintf((char *)show_buff, "????1");
										break;
									}
									case FEEDBK2_Package_ID: {
										sprintf((char *)show_buff, "????2");
										break;
									}
									case HANDPOT_Package_ID: {
										sprintf((char *)show_buff, "???");
										break;
									}
									case SYS_MAIN_POWER_KEY_ID: {
										sprintf((char *)show_buff, "?????");
										break;
									}
									case SYS_BACK_POWER_KEY_ID: {
										sprintf((char *)show_buff, "?????");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_1: {
										sprintf((char *)show_buff, "??¡¤1");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_2: {
										sprintf((char *)show_buff, "??¡¤2");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_3: {
										sprintf((char *)show_buff, "??¡¤3");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_4: {
										sprintf((char *)show_buff, "??¡¤4");
										break;
									}
									default:
										break;
								}
							}
							else
							{
								sprintf((char *)show_buff, "??%d??%dpack", 
									read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id,
									read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id);
							}
						}
						else
						{
							sprintf((char *)show_buff, "??1??¡¤ %d??", read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id);
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //????õô????
						// ??????
						
						// ???? ??????? ?????????????
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_sys_fault[data_index].fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d??%02d??%02d?? %02d?%02d??%02d??", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);
						
						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //??????
						// ?????
						if(read_data[x_sector].fs_sys_fault[data_index].state == DISCONNECT)
						{
							sprintf((char *)show_buff, "?õô????");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == DIS_RECOVERY)
						{
							sprintf((char *)show_buff, "??????");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == SHORTCIRCUIT)
						{
							sprintf((char *)show_buff, "?õô??¡¤");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == SHO_RECOVERY)
						{
							sprintf((char *)show_buff, "??¡¤???");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_TEMP_SENSOR_FAULT)
						{
							sprintf((char *)show_buff, "\xCE\xC2\xB6\xC8\xB9\xCA\xD5\xCF");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_TEMP_SENSOR_RECOVERY)
						{
							sprintf((char *)show_buff, "????????");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_SMOKE_POLLUTION_FAULT)
						{
							sprintf((char *)show_buff, "???????????");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_SMOKE_POLLUTION_RECOVERY)
						{
							sprintf((char *)show_buff, "?????????????");
						}
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == LOOP1_TEMP_SENSOR_FAULT)
                        {
                            sprintf((char *)show_buff, "\xCE\xC2\xB6\xC8\xB9\xCA\xD5\xCF");
                        }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == LOOP1_TEMP_SENSOR_RECOVERY)
                        {
                            sprintf((char *)show_buff, "\xCE\xC2\xB6\xC8\xB9\xCA\xD5\xCF\xBB\xD6\xB8\xB4");
                        }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == LOOP1_SMOKE_POLLUTION_FAULT)
                        {
                            sprintf((char *)show_buff, "\xD1\xCC\xCE\xED\xCE\xDB\xC8\xBE\xB9\xCA\xD5\xCF");
                        }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == LOOP1_SMOKE_POLLUTION_RECOVERY)
                        {
                            sprintf((char *)show_buff, "\xD1\xCC\xCE\xED\xCE\xDB\xC8\xBE\xB9\xCA\xD5\xCF\xBB\xD6\xB8\xB4");
                        }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == LOOP1_SMOKE_SENSOR_FAULT)
                        {
                            sprintf((char *)show_buff, "\xD1\xCC\xCE\xED\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF");
                        }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == LOOP1_SMOKE_SENSOR_RECOVERY)
                        {
                            sprintf((char *)show_buff, "\xD1\xCC\xCE\xED\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF\xBB\xD6\xB8\xB4");
                        }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CO_SENSOR_FAULT) { sprintf((char *)show_buff, "CO??????????"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CO_SENSOR_RECOVERY) { sprintf((char *)show_buff, "CO????????????"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_H2_SENSOR_FAULT) { sprintf((char *)show_buff, "H2??????????"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_H2_SENSOR_RECOVERY) { sprintf((char *)show_buff, "H2????????????"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_VOC_SENSOR_FAULT) { sprintf((char *)show_buff, "VOC??????????"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_VOC_SENSOR_RECOVERY) { sprintf((char *)show_buff, "VOC????????????"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CH4_SENSOR_FAULT) { sprintf((char *)show_buff, "CH4??????????"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CH4_SENSOR_RECOVERY) { sprintf((char *)show_buff, "CH4????????????"); }
						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //?????
						
						break;
					}
					case RECORD_ALARM: {
						// ???????
//						x_sector = total_records/400;
//						data_index = (total_records - 1 - temp_caculate)%400; // ????????????
//						
						x_sector = reverse_index/400;
						data_index = reverse_index%400;
						
//						DebugSendString(&read_data[total_records/500].byte_buff[data_index * 10], 10);
//						
//						sprintf((char *)show_buff, "%d", total_records);
//						SetTextValue(1, 37, show_buff); //?????
//						
//						sprintf((char *)show_buff, "%d %d %d %d %d %d %d %d %d %d", 
//							read_data[x_sector].byte_buff[0],
//							read_data[x_sector].byte_buff[1],
//							read_data[x_sector].byte_buff[2],
//							read_data[x_sector].byte_buff[3],
//							read_data[x_sector].byte_buff[4],
//							read_data[x_sector].byte_buff[5],
//							read_data[x_sector].byte_buff[6],
//							read_data[x_sector].byte_buff[7],
//							read_data[x_sector].byte_buff[8],
//							read_data[x_sector].byte_buff[9] );
//						SetTextValue(1, 36, show_buff); //?????
//						
						// ?õô?????§Ø?
						if(FormatRS485DetectFlashDeviceName(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id,
							read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id, show_buff) == 1)
						{
							// XR5000_LOOP3_CHANGE_20260726: Loop 3 history alarm display uses "??3??¡¤ X??".
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == MBUS_CONTROL_FLASH_ID)
					{
						/* XR5000_MBUS2_HAND_ALARM_FIRE_HISTORY_20260729: render loop2 manual alarm device. */
						sprintf((char *)show_buff, "??2??¡¤ %d??", read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id);
					}
					else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id == ALARM_ANNUNCIATOR_ID)
							{
								sprintf((char *)show_buff, "??????");
							}
							else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id == HANDPOT_Package_ID)
							{
								sprintf((char *)show_buff, "?????????");
							}
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id != 0)
						{
							sprintf((char *)show_buff, "??%d??%dpack", read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id,
								read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id);
							
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == 0 && 
										read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id != 0)
						{
							sprintf((char *)show_buff, "??1??¡¤ %d??", read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id);
						}
						else
						{
							sprintf((char *)show_buff, "¦Ä??õô");
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //????õô????
						
						// ??????
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //
						
						// ??????
						// 2025/11/19 15:39 ???
						// ???? ??????? ?????????????
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d??%02d??%02d?? %02d?%02d??%02d??", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);

						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //??????
						
						// ?????
						if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == FIRGAS_ALARM)
						{
							sprintf((char *)show_buff, "??????ŒÕ??");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == EAR_RECOVERY)
						{
							sprintf((char *)show_buff, "?????????");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == SMOKE_ALARM)
						{
							sprintf((char *)show_buff, "???????");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == TEMPRT_ALARM)
						{
							sprintf((char *)show_buff, "????");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == LINKAGE_PRESS)
						{
							sprintf((char *)show_buff, "??????????");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == MBUS2_HAND_ALARM)
						{
							/* XR5000_MBUS2_HAND_ALARM_FIRE_HISTORY_20260729: render manual alarm state. */
							sprintf((char *)show_buff, "???????");
						}
					else if(FIRGAS_ALARM_CO == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							/* XR5000_LOOP3_STATUS_FINALIZE_20260730: loop3 level 1 is a warning. */
							if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == RS485_DETECT_FLASH_ID)
							{
								sprintf((char *)show_buff, "CO\xD4\xA4\xBE\xAF");
							}
							else
							{
								sprintf((char *)show_buff, "?????????????????");
							}
						}
						else if(FIRGAS_ALARM_HH == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							/* XR5000_LOOP3_STATUS_FINALIZE_20260730: loop3 level 1 is a warning. */
							if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == RS485_DETECT_FLASH_ID)
							{
								sprintf((char *)show_buff, "H2\xD4\xA4\xBE\xAF");
							}
							else
							{
								sprintf((char *)show_buff, "???????????????");
							}
						}
						else if(RS485_TEMP_WARNING == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "??????");
						}
						else if(RS485_CO_FIRE == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "CO??");
						}
						else if(RS485_H2_FIRE == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "H2??");
						}
						else if(FIRGAS_ALARM_VOC == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "VOC???");
						}
                        else if(LOOP1_TEMP_WARNING == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
                        {
                            sprintf((char *)show_buff, "\xCE\xC2\xB6\xC8\xD4\xA4\xBE\xAF");
                        }
                        else if(LOOP1_SMOKE_WARNING == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
                        {
                            sprintf((char *)show_buff, "\xD1\xCC\xCE\xED\xD4\xA4\xBE\xAF");
                        }
                        else if(LOOP1_TEMP_WARNING_RECOVERY == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
                        {
                            sprintf((char *)show_buff, "\xCE\xC2\xB6\xC8\xD4\xA4\xBE\xAF\xBB\xD6\xB8\xB4");
                        }
                        else if(LOOP1_SMOKE_WARNING_RECOVERY == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
                        {
                            sprintf((char *)show_buff, "\xD1\xCC\xCE\xED\xD4\xA4\xBE\xAF\xBB\xD6\xB8\xB4");
                        }
						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //?????
						
						if(read_data[x_sector].fs_fire_alarm[data_index].data_high == 0xFFFF)
						{
							sprintf((char *)show_buff, "--");
						}
						else
						{
							sprintf((char *)show_buff, "%d", read_data[x_sector].fs_fire_alarm[data_index].data_high);
						}
						SetTextValue(temp_screen_id, values_ctrl_id[i], show_buff); //?????
						
						break;
					}
					case RECORD_GASOF: {
						// ???????
//						x_sector = total_records/500;
//						data_index = (total_records - 1 - temp_caculate)%500; // ????????????
						x_sector = reverse_index/500;
						data_index = reverse_index%500;

						// ??????
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //
						
						// ?õô?????§Ø?
						// ??????????õô
						if(FormatRS485DetectFlashDeviceName(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id,
							read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id, show_buff) == 1)
						{
							// XR5000_LOOP3_CHANGE_20260726: Loop 3 gas action history display uses "??3??¡¤ X??".
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id == OUTFIRE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id == OUTFIR1_PACKAGE_ID)
							{
								sprintf((char *)show_buff, "??????1");
							}
							else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id == OUTFIR2_PACKAGE_ID)
							{
								sprintf((char *)show_buff, "??????2");
							}
							else
							{
								sprintf((char *)show_buff, "??????");
							}
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id == ALARM_ANNUNCIATOR_ID)
							{
								sprintf((char *)show_buff, "??????");
							}
							else if(PART1_HAND_AUTO_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "????1");
							}
							else if(PART2_HAND_AUTO_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "????2");
							}
							else if(SYS_HAND_AUTO_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "???????");
							}
							else if(FEEDBK1_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "????1");
							}
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id != 0)
						{
							sprintf((char *)show_buff, "??%d??%dpack", 
								read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id,
								read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id);
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id == 0 && 
										read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id != 0)
						{
							sprintf((char *)show_buff, "??1??¡¤ %d??", read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id);
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //????õô????

						// ??????
						// 2025/11/19 15:39 ???
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_gas_outfires[data_index].fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d??%02d??%02d?? %02d?%02d??%02d??", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);
			
						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //??????

						// ?????
						switch(read_data[x_sector].fs_gas_outfires[data_index].state)
						{
							case OUTFIRE1OPEN_1:
								sprintf((char *)show_buff, "?????????????");
								break;
							case OUTFIRE1CLOSE:
								sprintf((char *)show_buff, "?????¨´??");
								break;
							case OUTFIRE1OPEN_2:
								sprintf((char *)show_buff, "?????????????");
								break;
							case OUTFIRE1OPEN_3:
								sprintf((char *)show_buff, "??????????????");
								break;
							case OUTFIRE_1_START_DELAY:
								sprintf((char *)show_buff, "?????????????????????");
								break;
							case OUTFIRE_2_START_DELAY:
								sprintf((char *)show_buff, "?????????????????????");
								break;
							case OUTFIRE_3_START_DELAY:
								sprintf((char *)show_buff, "??????????????????????");
								break;
							case OUTFIRE_STOP  :
								sprintf((char *)show_buff, "???????????");
								break;
							case OUTFIRESTART_AGAIN:
								sprintf((char *)show_buff, "????????????");
								break;
							case OUTFIRE_ST_PRESS:
								sprintf((char *)show_buff, "?????????????????");
								break;
							case OUTFIRE_SP_PRESS:
								sprintf((char *)show_buff, "????????????????");
								break;
							case OUTFIRE_SL_PRESS:
								sprintf((char *)show_buff, "??????????");
								break;
							case OUTFIRE_OVER :
								sprintf((char *)show_buff, "????????????");
								break;
							case OTHER_PART1_TURN_AUTO:
								sprintf((char *)show_buff, "????1?§Ý?????");
								break;
							case OTHER_PART1_TURN_HAND:
								sprintf((char *)show_buff, "????1?§Ý?????");
								break;
							case OTHER_PART2_TURN_AUTO:
								sprintf((char *)show_buff, "????2?§Ý?????");
								break;
							case OTHER_PART2_TURN_HAND:
								sprintf((char *)show_buff, "????2?§Ý?????");
								break;
							case OTHER_SYS_TURN_HAND:
								sprintf((char *)show_buff, "????????§Ý?????");
								break;
							case OTHER_SYS_TURN_AUTO:
								sprintf((char *)show_buff, "????????§Ý?????");
								break;
							case OUTFIRE_FEEDBACK1:
								sprintf((char *)show_buff, "????1????");
								break;
							default:
								sprintf((char *)show_buff,"¦Ä???");
								break;
						}

						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //?????
						
						break;
					}
					case RECORD_OTHER: { // ???????????
						// ???????
//						x_sector = total_records/500;
//						data_index = (total_records - 1 - temp_caculate)%500; // ????????????
						x_sector = reverse_index/500;
						data_index = reverse_index%500;
						
//						DebugPrintf("%d\r\n", total_records);
//						
						//DebugSendString(read_data[x_sector].byte_buff, total_records * 8);
						
						// ??????
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //
					
						// ?õô?????§Ø?
						// ??????????õô
						if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_RESET_Package_ID)
							{
								sprintf((char *)show_buff, "????¦Ë????");
							}
							else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_MAIN_POWER_KEY_ID)
							{
								sprintf((char *)show_buff, "?????????");
							}
							else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_SELFCHECK_Package_ID)
							{
								sprintf((char *)show_buff, "???????");
							}
							else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_CHECK_Package_ID)
						{
							sprintf((char *)show_buff, "\xCF\xB5\xCD\xB3\xBC\xEC\xB2\xE9\xB0\xB4\xBC\xFC"); /* XR5000_CHECK_CHANGE_20260804 */
						}
						else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_TURN_OFF_Package_ID)
							{
								sprintf((char *)show_buff, "?????????");
							}
						}
						else
						{
							sprintf((char *)show_buff, "¦Ä??õô");
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //????õô????
						
						// ??????
						// 2025/11/19 15:39 ???
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_other_record[data_index].fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d??%02d??%02d?? %02d?%02d??%02d??", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);

						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //??????
						
						if(read_data[x_sector].fs_other_record[data_index].state == OTHER_SYS_RESET)
						{
							sprintf((char *)show_buff, "????¦Ë???");
						}
						else if(read_data[x_sector].fs_other_record[data_index].state == OTHER_TURN_ON)
						{
							sprintf((char *)show_buff, "?????????");
						}
						else if(read_data[x_sector].fs_other_record[data_index].state == OTHER_SYS_SELF_CHECK)
						{
							sprintf((char *)show_buff, "?????");
						}
						else if(OTHER_SYS_CHECK == read_data[x_sector].fs_other_record[data_index].state)
						{
							sprintf((char *)show_buff, "\xCF\xB5\xCD\xB3\xBC\xEC\xB2\xE9"); /* XR5000_CHECK_CHANGE_20260804 */
						}
						else if(OTHER_TURN_OFF == read_data[x_sector].fs_other_record[data_index].state )
						{
							sprintf((char *)show_buff, "????????");
						}
						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //?????
						break;
					}
					default:
						break;
				}

			}
			else // ?????????????
			{
				clearTextValue(temp_screen_id , serial_ctrl_id[i]); //(????ID,???ID??
				clearTextValue(temp_screen_id , device_ctrl_id[i]); //(????ID,???ID??
				clearTextValue(temp_screen_id , retime_ctrl_id[i]); //(????ID,???ID??
				clearTextValue(temp_screen_id , states_ctrl_id[i]); //(????ID,???ID??
				if(bsrr_entry->curr_show_type == RECORD_ALARM)
				{
					clearTextValue(temp_screen_id , values_ctrl_id[i]); //(????ID,???ID??
				}
			}
		} // for???????????????
	} // ?§Ø??????¦Ì?????
}

// ?¨²???????????FLASH?ãè???????
static void BspAlarmDataSaveApp(
	FlashReadCtrlId addr_type, // ?›¥???
	FlashSaveType save_type,   // ?›¥????
	uint8_t cluster_id,        // ???
	uint8_t pack_or_cabin,     // ????
	uint16_t val               // ?
)
{
	FlashSaveFireAlarm_t temp_data = {0};

	// ?õô????
	temp_data.fs_base.fs_detect_id.cluster_id = cluster_id; // ??ID
	temp_data.fs_base.fs_detect_id.cabin_or_pack_id = pack_or_cabin; // packID ????0????pack
	// ??P?
	// 2025/11/19 15:39 ???
	
	FlashSaveTimeBuff temp_time = {0};
	// ?????Ô§?
	setFlashTime(temp_time, years, months, days, hours, minutes, secs);

	
	
	for(uint8_t i = 0; i < 5; i++)
	{
		temp_data.fs_base.fs_time_buff[i] = temp_time[i];
	}

	// ????? ??
	temp_data.fs_base.state = save_type;
	// ?›¥????????
	temp_data.data_high = val;
	
	BspSaveDataToFlash(addr_type, save_type, (void *)&temp_data);
}

static void PowerManageCtrl(uint8_t main_power_state, uint8_t back_power_state)
{
	// ?????????? ??????
	if(main_power_state == 0 && (back_power_state == open_circuit || back_power_state == short_circuit))
	{
		BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_TURN_OFF, LINKAGE_CLUSTER_ID, SYS_TURN_OFF_Package_ID);
	}
	else
	{
		// ?§Ø?????
		if(main_power_state != 1 && main_power_alarm_flag == 0)
		{
			main_power_beep_ctrl |= (1U << 0);
			silencers_state  = 0;  // ?????? ??????????? ?????????????
			main_power_alarm_flag = 1;
			
			// ?›¥??FLASH????›¥???? ??????
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID);
			// ??RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID, DISCONNECT);
			
		}
		else if(main_power_state == 1 && main_power_alarm_flag == 1)
		{
			// ???????????
			main_power_alarm_flag = 0;
			// ????¦Ë??????
			main_power_beep_ctrl &= ~(1U << 0);
	//				silencers_state  = 0;  // ??????????? ???????? // ????????????????? ????¦Ì???????????????¦Ë?????
			// ?›¥??FLASH????›¥???? ??????
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID);
			// ?????????????§Õ???????
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
		}
		
		// ?§Ø????
		if(back_power_alarm_flag == 0 && back_power_state == open_circuit)
		{
			// ????????
			main_power_beep_ctrl |= (1U << 1);
			// ????????
			back_power_alarm_flag = 1;
			
			silencers_state  = 0;  // ?????? ??????????? ?????????????
			// ?›¥??FLASH????›¥???? ??????
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			// ??RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID, DISCONNECT);
		} // ?????¡¤?????
		else if(back_power_alarm_flag == 0 && back_power_state == short_circuit)
		{
			// ????????
			main_power_beep_ctrl |= (1U << 1);
			// ???????? ?????
			back_power_alarm_flag = 2;
			silencers_state  = 0;  // ?????? ??????????? ?????????????
			// ?›¥??FLASH????›¥???? ?????¡¤
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			// ??RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID, SHORTCIRCUIT);
		} // ?????¡¤?????
		else if(back_power_alarm_flag != 0 && back_power_state != open_circuit && back_power_state != short_circuit)
		{
			if(back_power_alarm_flag == 1)
			{
				// ?›¥??FLASH????›¥???? ?????¡¤
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			}
			else
			{
				// ?›¥??FLASH????›¥???? ?????¡¤
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			}
			
			main_power_beep_ctrl &= ~(1U << 1);
			
			// ?????????????§Õ???????
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			back_power_alarm_flag = 0;
		} // ???????????
	} // ????????????§Ø?????
}

static void HandForceStartAnyCluster(FireExtinguishDeviceActionSave *fedas_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		if(ctrl_id == 41)
		{
			return;
		}
		getBM8563TimeToSystemTime(); // ??????RTC???
		// ???????????
		// ??????????
		fedas_entry->atr[fedas_entry->self_point_len].years  = years + 2000;
		fedas_entry->atr[fedas_entry->self_point_len].months = months;
		fedas_entry->atr[fedas_entry->self_point_len].days   = days;
		fedas_entry->atr[fedas_entry->self_point_len].hours  = hours;
		fedas_entry->atr[fedas_entry->self_point_len].minute = minutes;
		fedas_entry->atr[fedas_entry->self_point_len].second = secs;

		fedas_entry->cabin_id[fedas_entry->self_point_len]   = 0;
		fedas_entry->cluster_id[fedas_entry->self_point_len] = ctrl_id - 20;
		fedas_entry->pack_id[fedas_entry->self_point_len]    = 1; // ??ID?????1
		fedas_entry->fed_action[fedas_entry->self_point_len] = FIRE_EXTINGUISH_START_SPRAY_DELAY; // ????????
		fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // ??????30??
		
		fedas_entry->curr_cntd_time[fedas_entry->self_point_len] = baojingjishi;
		fedas_entry->start_cntd_time[fedas_entry->self_point_len] = fedas_entry->curr_cntd_time[fedas_entry->self_point_len];
		
		
		fedas_entry->self_point_len++;
		
		SetMonitorPageFrom(current_screen_id); /* XR5000_MONITOR_RETURN_NAV_CHANGE_20260802 */
	}
}

uint8_t fan_disconnect_record_flag = 0;
static void BspFanOnlineJudgeFaultRecord(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	if(fan_disconnect_count == 5)
	{
		// ?§Ø????§Õ??
		if(fan_disconnect_record_flag == 0)
		{
			// ?§Ø??????????? ?????? ?????¦Ì? 
			if(creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, LINKAGE_FAN_Package_ID, 0) == 0)
			{
				// ???§Õ???? ???????? ????????LED 
				beep_fault_ctrl  = 2;   // ???????? ????????????¦Ë
				silencers_state  = 0;   // ???????
				disconnect_state = 1;   // ?????????
				// ?????????›¥
				//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, LINKAGE_FAN_Package_ID);
			}
			// ???§Õ??
			fan_disconnect_record_flag = 1;
		}
	}
	else // ????????§Ö???
	{
		if(fan_disconnect_record_flag == 1)
		{
			// ?????????????§Õ???????
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, LINKAGE_FAN_Package_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, LINKAGE_FAN_Package_ID);
			}
			fan_disconnect_record_flag = 0;
		}
	}
}

uint8_t fan_start_state = 0;
uint8_t fan_start_ticks = 0;

uint8_t fan_send_counts = 0;

void FanSendCountInit(void)
{
	fan_send_counts = 1;
}

static void BspFanStartCrtlApp(uint8_t fan_sta, uint8_t early_aralm_num, uint8_t fire_alarm_num)
{
	// ??????????? ???? ???????? ???¡Â??????
	if(fan_sta == fan_disconnect || fan_sta == fan_break || fan_sta == fan_run)
	{
//		return;
	}
	// ??????? ?§Ø???????????? ??????????
	if(baojingjishi - fan_start_ticks >= 2)
	{
		fan_start_ticks = baojingjishi;
		if(fan_sta == fan_stop && early_aralm_num != 0 && fire_alarm_num == 0)
		{
			if(getSysHandAutoState() == KEY_MANUAL) // ?????????????? ???
			{
				if(linkage_start_key_press_flag == 1)
				{
					// ?????????
					Fan1CtrlOpen();
					Fan2CtrlOpen();
					linkage_start_key_press_flag = 0;
					SysStartStateLedCtrl(LED_ON);
				}
			}
			else
			{
				// ?????????
				Fan1CtrlOpen();
				Fan2CtrlOpen();
				SysStartStateLedCtrl(LED_ON);
			}
			
		}
		// ????????????? ??????? ?????0 ?????
		if(fan_sta == fan_run && early_aralm_num == 0 && fire_alarm_num == 0)
		{
			Fan1CtrlClose();
			Fan2CtrlClose();
		}
		

		// ?????????
		if(early_aralm_num == 0 && fire_alarm_num == 0)
		{
			SysStartStateLedCtrl(LED_OFF);
		}

		if(fan_send_counts < 2 && early_aralm_num == 0 && fire_alarm_num == 0)
		{
			if(fan_sta == fan_stop)
			{
				fan_send_counts = 2;
			}
			else
			{
				fan_send_counts++;
				Fan1CtrlClose();
				Fan2CtrlClose();
			}
			
		}
		if(fan_send_counts < 10 && fire_alarm_num != 0) // ???????????§³??10????
		{
			// ????????????? ???????? ?????
			if(fan_sta == fan_run)
			{
				Fan1CtrlClose();
				Fan2CtrlClose();
				fan_send_counts = 10;
			}
			else
			{
				Fan1CtrlClose();
				Fan2CtrlClose();
				fan_send_counts++;
			}
		}
		
		if(early_aralm_num == 0 && fire_alarm_num == 0 && linkage_start_key_press_flag == 1)
		{
			linkage_start_key_press_flag = 0;
		}
		
	}
}

// 0 ??§Ò??? 1?§Ò??? 2????????????????
uint8_t any_point_temper_alarm = 0;
uint8_t any_point_smoke_alarm = 0;

static void PointTypeDetectorAllStateInit(void)
{
	any_point_temper_alarm = 0;
	any_point_smoke_alarm = 0;
	
	clearPointTypeMixtureDetectAllStateMemory();
	memset(loop1_raw_state_memory, 0, sizeof(loop1_raw_state_memory)); /* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: reset edge memory. */
	clearPointTypeMixtureDisconnectCount();
}

// XR5000_LOOP3_CHANGE_20260726: Loop 3 realtime alarm bridge, using original pack/cabin alarm flow.
static uint8_t RS485Loop3FindFault(uint8_t addr, uint8_t fault_type)
{
	for(uint8_t i = 0; i < pcfs_buttom_point; i++)
	{
		if(pcfs[i].da.cluster_id == RS485_DETECT_FLASH_ID &&
			pcfs[i].da.pack_id == addr && pcfs[i].fault_type == fault_type)
		{
			return i;
		}
	}
	return 0xFF;
}

static void RS485Loop3AddFault(uint8_t addr, uint8_t fault_type, FlashSaveType flash_type)
{
	if(RS485Loop3FindFault(addr, fault_type) != 0xFF || pcfs_buttom_point >= 224U)
		return;
	getBM8563TimeToSystemTime();
	pcfs[pcfs_buttom_point].detector_class = PackClassID;
	pcfs[pcfs_buttom_point].da.cabin_id = 0;
	pcfs[pcfs_buttom_point].da.cluster_id = RS485_DETECT_FLASH_ID;
	pcfs[pcfs_buttom_point].da.pack_id = addr;
	pcfs[pcfs_buttom_point].atr.years = years + 2000;
	pcfs[pcfs_buttom_point].atr.months = months;
	pcfs[pcfs_buttom_point].atr.days = days;
	pcfs[pcfs_buttom_point].atr.hours = hours;
	pcfs[pcfs_buttom_point].atr.minute = minutes;
	pcfs[pcfs_buttom_point].atr.second = secs;
	pcfs[pcfs_buttom_point].fault_type = fault_type;
	pcfs_buttom_point++; fault_check_new_flag = 1;
	beep_fault_ctrl = 2;
	silencers_state = 0;
	disconnect_state = 1;
	BspCommonDataSaveApp(FAULT_FLASH_SAVE, flash_type, RS485_DETECT_FLASH_ID, addr);
}

static void RS485Loop3RemoveFault(uint8_t addr, uint8_t fault_type, FlashSaveType recovery_type)
{
	uint8_t index = RS485Loop3FindFault(addr, fault_type);
	if(index != 0xFF)
	{
		deletRecoveryRecord(index);
		fault_check_new_flag = 1;
		BspCommonDataSaveApp(FAULT_FLASH_SAVE, recovery_type, RS485_DETECT_FLASH_ID, addr);
	}
}

static void RS485Loop3RemoveWarning(uint8_t addr, uint8_t alarm_type)
{
	DeletPackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, alarm_type);
}

static void RS485Loop3ClearCurrentState(uint8_t addr)
{
	for(uint8_t i = pcfs_buttom_point; i > 0U; i--)
	{
		uint8_t index = i - 1U;
		if(pcfs[index].da.cluster_id == RS485_DETECT_FLASH_ID && pcfs[index].da.pack_id == addr)
			deletRecoveryRecord(index);
	}
	RS485Loop3RemoveWarning(addr, Loop3TempWarning); force_alarm_check_new_flag = 1;
	RS485Loop3RemoveWarning(addr, Carbon); force_alarm_check_new_flag = 1;
	RS485Loop3RemoveWarning(addr, Hydrogen); force_alarm_check_new_flag = 1;
	RS485Loop3RemoveWarning(addr, Voc); force_alarm_check_new_flag = 1;
	memset(rs485_detect_alarm_memory[addr], 0, sizeof(rs485_detect_alarm_memory[addr]));
}

static uint8_t RS485DetectDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	uint8_t fault_sum = 0;
	(void)pcfs_entry;
	(void)pcfs_point;

	for(uint8_t addr = 1; addr < RS485_DETECT_MAX_DEVICES; addr++)
	{
		if(RS485Detect_GetOnline(addr) == 0)
		{
			/* XR5000_LOOP3_STATUS_FINALIZE_20260730: avoid repeated screen-59 refresh for inactive addresses. */
			if(rs485_detect_disconnect_memory[addr] != 0U ||
				rs485_detect_alarm_memory[addr][RS485_SENSOR_TEMPERATURE] != 0U ||
				rs485_detect_alarm_memory[addr][RS485_SENSOR_SMOKE] != 0U ||
				rs485_detect_alarm_memory[addr][RS485_SENSOR_CO] != 0U ||
				rs485_detect_alarm_memory[addr][RS485_SENSOR_H2] != 0U ||
				rs485_detect_alarm_memory[addr][RS485_SENSOR_VOC] != 0U ||
				rs485_detect_alarm_memory[addr][RS485_SENSOR_CH4] != 0U)
			{
				RS485Loop3ClearCurrentState(addr);
			}
			rs485_detect_disconnect_memory[addr] = 0;
			continue;
		}

		if(RS485Detect_IsDisconnected(addr))
		{
			fault_sum++;
			if(rs485_detect_disconnect_memory[addr] == 0)
			{
				rs485_detect_disconnect_memory[addr] = 1;
				RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_OFFLINE, DISCONNECT);
			}
			continue;
		}

		if(rs485_detect_disconnect_memory[addr] != 0)
		{
			RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_OFFLINE, DIS_RECOVERY);
			rs485_detect_disconnect_memory[addr] = 0;
		}

		uint8_t temp_state = RS485Detect_GetSensorState(addr, RS485_SENSOR_TEMPERATURE);
		uint8_t smoke_state = RS485Detect_GetSensorState(addr, RS485_SENSOR_SMOKE);
		uint8_t co_state = RS485Detect_GetSensorState(addr, RS485_SENSOR_CO);
		uint8_t h2_state = RS485Detect_GetSensorState(addr, RS485_SENSOR_H2);
		uint8_t voc_state = RS485Detect_GetSensorState(addr, RS485_SENSOR_VOC);
		uint8_t ch4_state = RS485Detect_GetSensorState(addr, RS485_SENSOR_CH4);
		uint8_t old_temp = rs485_detect_alarm_memory[addr][RS485_SENSOR_TEMPERATURE];
		uint8_t old_smoke = rs485_detect_alarm_memory[addr][RS485_SENSOR_SMOKE];
		uint8_t old_co = rs485_detect_alarm_memory[addr][RS485_SENSOR_CO];
		uint8_t old_h2 = rs485_detect_alarm_memory[addr][RS485_SENSOR_H2];
		uint8_t old_voc = rs485_detect_alarm_memory[addr][RS485_SENSOR_VOC];
		uint8_t old_ch4 = rs485_detect_alarm_memory[addr][RS485_SENSOR_CH4];
		uint8_t type = RS485Detect_GetType(addr);

		/* XR5000_LOOP3_STATUS_CHANGE_20260730: XR8303 and XR8305 share protocol-level state handling. */
		if(type == RS485_DETECT_TYPE_XR805 || type == RS485_DETECT_TYPE_XR8303 || type == RS485_DETECT_TYPE_XR8305)
		{
			if(old_temp != temp_state)
			{
				if(old_temp == 1U) RS485Loop3RemoveWarning(addr, Loop3TempWarning); force_alarm_check_new_flag = 1;
				if((type == RS485_DETECT_TYPE_XR805 && old_temp == 9U) || (type != RS485_DETECT_TYPE_XR805 && old_temp == 3U)) RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_TEMPERATURE, RS485_TEMP_SENSOR_RECOVERY);
				if(temp_state == 1U)
				{
					getBM8563TimeToSystemTime();
					StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Loop3TempWarning); force_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, RS485_TEMP_WARNING, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_TEMPERATURE));
				}
				else if(temp_state == 2U)
				{
					getBM8563TimeToSystemTime();
					StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Temperature); fire_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_TEMPERATURE));
				}
				else if((type == RS485_DETECT_TYPE_XR805 && temp_state == 9U) || (type != RS485_DETECT_TYPE_XR805 && temp_state == 3U))
				{
					RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_TEMPERATURE, RS485_TEMP_SENSOR_FAULT);
				}
				rs485_detect_alarm_memory[addr][RS485_SENSOR_TEMPERATURE] = temp_state;
			}

			if(old_smoke != smoke_state)
			{
				if(type == RS485_DETECT_TYPE_XR805 && old_smoke == 1U) RS485Loop3RemoveWarning(addr, Loop1SmokeWarning); force_alarm_check_new_flag = 1;
				if(type == RS485_DETECT_TYPE_XR805 && old_smoke == 9U) RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_SMOKE_SENSOR, LOOP1_SMOKE_SENSOR_RECOVERY);
				if(type != RS485_DETECT_TYPE_XR805 && old_smoke == 8U) RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_SMOKE, RS485_SMOKE_POLLUTION_RECOVERY);
				if(type == RS485_DETECT_TYPE_XR805 && smoke_state == 1U)
				{
					getBM8563TimeToSystemTime();
					StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Loop1SmokeWarning); force_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, LOOP1_SMOKE_WARNING, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_SMOKE));
				}
				else if((type == RS485_DETECT_TYPE_XR805 && smoke_state == 2U) || (type != RS485_DETECT_TYPE_XR805 && smoke_state == 1U))
				{
					getBM8563TimeToSystemTime();
					StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Smoke); fire_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, SMOKE_ALARM, RS485_DETECT_FLASH_ID, addr, 0xFFFF);
				}
				else if((type == RS485_DETECT_TYPE_XR805 && smoke_state == 9U) || (type != RS485_DETECT_TYPE_XR805 && smoke_state == 8U))
				{
					if(type == RS485_DETECT_TYPE_XR805) RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_SMOKE_SENSOR, LOOP1_SMOKE_SENSOR_FAULT);
					else RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_SMOKE, RS485_SMOKE_POLLUTION_FAULT);
				}
				rs485_detect_alarm_memory[addr][RS485_SENSOR_SMOKE] = smoke_state;
			}
			if(old_co != co_state)
			{
				if(old_co == 1U) RS485Loop3RemoveWarning(addr, Carbon); force_alarm_check_new_flag = 1;
				if(type == RS485_DETECT_TYPE_XR805 && old_co == 9U) RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_CO, RS485_CO_SENSOR_RECOVERY);
				if(co_state == 1U)
				{
					getBM8563TimeToSystemTime();
					StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Carbon); force_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_CO, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO));
				}
				else if(co_state == 2U)
				{
					getBM8563TimeToSystemTime();
					StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Loop3CarbonFire); fire_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, RS485_CO_FIRE, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO));
				}
				else if(type == RS485_DETECT_TYPE_XR805 && co_state == 9U) RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_CO, RS485_CO_SENSOR_FAULT);
				rs485_detect_alarm_memory[addr][RS485_SENSOR_CO] = co_state;
			}

			if(old_h2 != h2_state)
			{
				if(old_h2 == 1U) RS485Loop3RemoveWarning(addr, Hydrogen); force_alarm_check_new_flag = 1;
				if(type == RS485_DETECT_TYPE_XR805 && old_h2 == 9U) RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_H2, RS485_H2_SENSOR_RECOVERY);
				if(h2_state == 1U)
				{
					getBM8563TimeToSystemTime();
					StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Hydrogen); force_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_HH, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2));
				}
				else if(h2_state == 2U)
				{
					getBM8563TimeToSystemTime();
					StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Loop3HydrogenFire); fire_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, RS485_H2_FIRE, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2));
				}
				else if(type == RS485_DETECT_TYPE_XR805 && h2_state == 9U) RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_H2, RS485_H2_SENSOR_FAULT);
				rs485_detect_alarm_memory[addr][RS485_SENSOR_H2] = h2_state;
			}

			if(old_voc != voc_state)
			{
				if(old_voc == 1U) RS485Loop3RemoveWarning(addr, Voc); force_alarm_check_new_flag = 1;
				if(type == RS485_DETECT_TYPE_XR805 && old_voc == 9U) RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_VOC, RS485_VOC_SENSOR_RECOVERY);
				if(voc_state == 1U)
				{
					getBM8563TimeToSystemTime();
					StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Voc); force_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_VOC, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_VOC));
				}
				else if(type == RS485_DETECT_TYPE_XR805 && voc_state == 2U)
				{
					getBM8563TimeToSystemTime();
					StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Voc); fire_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_VOC, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_VOC));
				}
				else if(type == RS485_DETECT_TYPE_XR805 && voc_state == 9U) RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_VOC, RS485_VOC_SENSOR_FAULT);
				rs485_detect_alarm_memory[addr][RS485_SENSOR_VOC] = voc_state;
			}

			if(type == RS485_DETECT_TYPE_XR805 && old_ch4 != ch4_state)
			{
				if(old_ch4 == 1U) RS485Loop3RemoveWarning(addr, Methane); force_alarm_check_new_flag = 1;
				if(old_ch4 == 9U) RS485Loop3RemoveFault(addr, RS485_LOOP3_FAULT_CH4, RS485_CH4_SENSOR_RECOVERY);
				if(ch4_state == 1U)
				{
					getBM8563TimeToSystemTime();
					StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Methane); force_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_CH4, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_CH4));
				}
				else if(ch4_state == 2U)
				{
					getBM8563TimeToSystemTime();
					StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Methane); fire_alarm_check_new_flag = 1;
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_CH4, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_CH4));
				}
				else if(ch4_state == 9U) RS485Loop3AddFault(addr, RS485_LOOP3_FAULT_CH4, RS485_CH4_SENSOR_FAULT);
				rs485_detect_alarm_memory[addr][RS485_SENSOR_CH4] = ch4_state;
			}
		}
		else
		{
			if(temp_state != 0U && old_temp == 0U)
			{
				getBM8563TimeToSystemTime();
				StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Temperature); fire_alarm_check_new_flag = 1;
				BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_TEMPERATURE));
			}
			if(smoke_state != 0U && old_smoke == 0U)
			{
				getBM8563TimeToSystemTime();
				StoragePackFireAlarm(&pcfas, RS485_DETECT_FLASH_ID, addr, Smoke); fire_alarm_check_new_flag = 1;
				BspAlarmDataSaveApp(FIRE_FLASH_SAVE, SMOKE_ALARM, RS485_DETECT_FLASH_ID, addr, 0xFFFF);
			}
			if(co_state != 0U && old_co == 0U)
			{
				getBM8563TimeToSystemTime();
				StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Carbon); force_alarm_check_new_flag = 1;
				BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_CO, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO));
			}
			if(h2_state != 0U && old_h2 == 0U)
			{
				getBM8563TimeToSystemTime();
				StoragePackCabinForeWarn(&pcfws, RS485_DETECT_FLASH_ID, addr, Hydrogen); force_alarm_check_new_flag = 1;
				BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_HH, RS485_DETECT_FLASH_ID, addr, RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2));
			}
			rs485_detect_alarm_memory[addr][RS485_SENSOR_TEMPERATURE] = temp_state;
			rs485_detect_alarm_memory[addr][RS485_SENSOR_SMOKE] = smoke_state;
			rs485_detect_alarm_memory[addr][RS485_SENSOR_CO] = co_state;
			rs485_detect_alarm_memory[addr][RS485_SENSOR_H2] = h2_state;
			rs485_detect_alarm_memory[addr][RS485_SENSOR_VOC] = voc_state;
		}
		rs485_detect_alarm_memory[addr][RS485_SENSOR_CH4] = ch4_state;

		if(RS485Loop3FindFault(addr, RS485_LOOP3_FAULT_TEMPERATURE) != 0xFF) fault_sum++;
		if(RS485Loop3FindFault(addr, RS485_LOOP3_FAULT_SMOKE) != 0xFF) fault_sum++;
		if(RS485Loop3FindFault(addr, RS485_LOOP3_FAULT_CO) != 0xFF) fault_sum++;
		if(RS485Loop3FindFault(addr, RS485_LOOP3_FAULT_H2) != 0xFF) fault_sum++;
		if(RS485Loop3FindFault(addr, RS485_LOOP3_FAULT_VOC) != 0xFF) fault_sum++;
		if(RS485Loop3FindFault(addr, RS485_LOOP3_FAULT_CH4) != 0xFF) fault_sum++;
		if(RS485Loop3FindFault(addr, RS485_LOOP3_FAULT_SMOKE_SENSOR) != 0xFF) fault_sum++;

		if(rs485_detect_pas_memory[addr] == 0 &&
			(((type == RS485_DETECT_TYPE_XR805 && (smoke_state == 2U || co_state == 2U || h2_state == 2U || voc_state == 2U || ch4_state == 2U)) || (type != RS485_DETECT_TYPE_XR805 && (smoke_state == 1U || co_state == 2U || h2_state == 2U || voc_state == 1U))) && temp_state == 2U))
		{
			getBM8563TimeToSystemTime();
			fire_alarm_flag.cluster_alarm_state = 1;
			pas[pas_pointer].cluster_id = RS485_DETECT_FLASH_ID;
			pas[pas_pointer].pack_id = addr;
			pas[pas_pointer].cabin_id = 0;
			pas[pas_pointer].lunch_state = 0;
			pas[pas_pointer].atr.years = years + 2000;
			pas[pas_pointer].atr.months = months;
			pas[pas_pointer].atr.days = days;
			pas[pas_pointer].atr.hours = hours;
			pas[pas_pointer].atr.minute = minutes;
			pas[pas_pointer].atr.second = secs;
			pas_pointer++;
			rs485_detect_pas_memory[addr] = 1;
		}
	}

	if(pcfs_buttom_point > 0) disconnect_state = 1;
	return fault_sum;
}
static uint8_t MBus2DataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	uint8_t disconnect_sum = 0;
	(void)pcfs_entry;

	for (uint8_t addr = 1; addr < MBUS_CONTROL_MAX_DEVICES; addr++)
	{
		if (MBusCtrl_GetDeviceType(addr) == MBUS_CONTROL_DEV_UNKNOWN)
			continue;

		if (MBusCtrl_GetOnline(addr) == 0)
		{
			if (mbus2_disconnect_memory[addr] != 0)
			{
				uint8_t index = findRecoveryDevice(MBUS_CONTROL_FLASH_ID, addr, 0);
				if (index != 0xFF)
				{
					deletRecoveryRecord(index);
				}
				mbus2_disconnect_memory[addr] = 0;
			}
			continue;
		}

		if (MBusCtrl_IsDisconnected(addr))
		{
			disconnect_sum++;
			if (mbus2_disconnect_memory[addr] == 0)
			{
				mbus2_disconnect_memory[addr] = 1;
				if (creatNewFaultRecordToCache(MBUS_CONTROL_FLASH_ID, addr, 0) == 0)
				{
					beep_fault_ctrl = 2;
					silencers_state = 0;
					disconnect_state = 1;
					BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, MBUS_CONTROL_FLASH_ID, addr);
				}
			}
			continue;
		}

		if (mbus2_disconnect_memory[addr] != 0)
		{
			uint8_t index = findRecoveryDevice(MBUS_CONTROL_FLASH_ID, addr, 0);
			if (index != 0xFF)
			{
				deletRecoveryRecord(index);
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, MBUS_CONTROL_FLASH_ID, addr);
			}
			mbus2_disconnect_memory[addr] = 0;
		}

		if (MBusCtrl_GetDeviceType(addr) == MBUS_CONTROL_DEV_XR2200)
		{
			if (MBusCtrl_IsAlarmState(addr))
			{
				if (mbus2_hand_alarm_memory[addr] == 0)
				{
					mbus2_hand_alarm_memory[addr] = 1;
					/* XR5000_FIRE_DISPLAY_GENERIC_NAMES_20260729: store XR2200 record in fire history only; do not add pcfas. */
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, MBUS2_HAND_ALARM, MBUS_CONTROL_FLASH_ID, addr, 1);
					MBusCtrl_PostFireDisplayEvent(2, addr, MBUS_FIRE_DISPLAY_DETECT_MANUAL, MBUS_FIRE_DISPLAY_ALARM_FIRE);
				}
			}
			else
			{
				mbus2_hand_alarm_memory[addr] = 0;
			}
		}
	}

	if (*pcfs_point > 0)
	{
		disconnect_state = 1;
	}

	return disconnect_sum;
}
/* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: loop1 typed warning/fault cache helpers. */
static uint8_t Loop1FindFault(uint8_t addr, uint8_t fault_type)
{
    for(uint8_t i = 0U; i < pcfs_buttom_point; i++)
    {
        if(pcfs[i].detector_class == CabinClassID && pcfs[i].da.cluster_id == 0U &&
            pcfs[i].da.cabin_id == addr && pcfs[i].fault_type == fault_type) return i;
    }
    return 0xFFU;
}

static void Loop1AddFault(uint8_t addr, uint8_t fault_type, FlashSaveType flash_type)
{
    if(Loop1FindFault(addr, fault_type) != 0xFFU || pcfs_buttom_point >= 224U) return;
    getBM8563TimeToSystemTime();
    pcfs[pcfs_buttom_point].detector_class = CabinClassID;
    pcfs[pcfs_buttom_point].da.cabin_id = addr;
    pcfs[pcfs_buttom_point].da.cluster_id = 0U;
    pcfs[pcfs_buttom_point].da.pack_id = 0U;
    pcfs[pcfs_buttom_point].atr.years = years + 2000;
    pcfs[pcfs_buttom_point].atr.months = months;
    pcfs[pcfs_buttom_point].atr.days = days;
    pcfs[pcfs_buttom_point].atr.hours = hours;
    pcfs[pcfs_buttom_point].atr.minute = minutes;
    pcfs[pcfs_buttom_point].atr.second = secs;
    pcfs[pcfs_buttom_point].fault_type = fault_type;
    pcfs_buttom_point++;
    fault_check_new_flag = 1U;
    beep_fault_ctrl = 2U;
    silencers_state = 0U;
    disconnect_state = 1U;
    BspCommonDataSaveApp(FAULT_FLASH_SAVE, flash_type, 0U, addr);
}

static void Loop1RemoveFault(uint8_t addr, uint8_t fault_type, FlashSaveType recovery_type)
{
    uint8_t index = Loop1FindFault(addr, fault_type);
    if(index == 0xFFU) return;
    deletRecoveryRecord(index);
    fault_check_new_flag = 1U;
    BspCommonDataSaveApp(FAULT_FLASH_SAVE, recovery_type, 0U, addr);
}

static uint8_t Loop1FindWarning(uint8_t addr, uint8_t alarm_type)
{
    for(uint8_t i = 0U; i < pcfws.self_bottom_point; i++)
    {
        if(pcfws.detector_class[i] == CabinClassID && pcfws.da[i].cluster_id == 0U &&
            pcfws.da[i].cabin_id == addr && pcfws.alarm_type[i] == alarm_type) return i;
    }
    return 0xFFU;
}

static void Loop1AddWarning(uint8_t addr, uint8_t alarm_type, FlashSaveType flash_type, uint16_t value)
{
    uint8_t index;
    if(Loop1FindWarning(addr, alarm_type) != 0xFFU || pcfws.self_bottom_point >= 224U) return;
    getBM8563TimeToSystemTime();
    index = pcfws.self_bottom_point;
    pcfws.detector_class[index] = CabinClassID;
    pcfws.da[index].cabin_id = addr;
    pcfws.da[index].cluster_id = 0U;
    pcfws.da[index].pack_id = 0U;
    pcfws.alarm_type[index] = alarm_type;
    pcfws.atr[index].years = years + 2000;
    pcfws.atr[index].months = months;
    pcfws.atr[index].days = days;
    pcfws.atr[index].hours = hours;
    pcfws.atr[index].minute = minutes;
    pcfws.atr[index].second = secs;
    pcfws.self_bottom_point++;
    force_alarm_check_new_flag = 1U;
    beep_fire_ctrl |= 0x0FU;
    silencers_state = 0U;
    BspAlarmDataSaveApp(FIRE_FLASH_SAVE, flash_type, 0U, addr, value);
}

static void Loop1RemoveWarning(uint8_t addr, uint8_t alarm_type, FlashSaveType recovery_type, uint16_t value)
{
    uint8_t index = Loop1FindWarning(addr, alarm_type);
    if(index == 0xFFU) return;
    for(; index + 1U < pcfws.self_bottom_point; index++)
    {
        pcfws.detector_class[index] = pcfws.detector_class[index + 1U];
        pcfws.da[index] = pcfws.da[index + 1U];
        pcfws.atr[index] = pcfws.atr[index + 1U];
        pcfws.alarm_type[index] = pcfws.alarm_type[index + 1U];
    }
    pcfws.self_bottom_point--;
    force_alarm_check_new_flag = 1U;
    BspAlarmDataSaveApp(FIRE_FLASH_SAVE, recovery_type, 0U, addr, value);
}

static void Loop1ClearCurrentState(uint8_t addr)
{
    for(uint8_t i = pcfs_buttom_point; i > 0U; i--)
    {
        uint8_t index = i - 1U;
        if(pcfs[index].detector_class == CabinClassID && pcfs[index].da.cluster_id == 0U &&
            pcfs[index].da.cabin_id == addr && pcfs[index].fault_type >= LOOP1_FAULT_OFFLINE) deletRecoveryRecord(index);
    }
    {
        uint8_t index = Loop1FindWarning(addr, Loop1TempWarning);
        if(index != 0xFFU)
        {
            for(; index + 1U < pcfws.self_bottom_point; index++)
            {
                pcfws.detector_class[index] = pcfws.detector_class[index + 1U];
                pcfws.da[index] = pcfws.da[index + 1U];
                pcfws.atr[index] = pcfws.atr[index + 1U];
                pcfws.alarm_type[index] = pcfws.alarm_type[index + 1U];
            }
            pcfws.self_bottom_point--;
        }
        index = Loop1FindWarning(addr, Loop1SmokeWarning);
        if(index != 0xFFU)
        {
            for(; index + 1U < pcfws.self_bottom_point; index++)
            {
                pcfws.detector_class[index] = pcfws.detector_class[index + 1U];
                pcfws.da[index] = pcfws.da[index + 1U];
                pcfws.atr[index] = pcfws.atr[index + 1U];
                pcfws.alarm_type[index] = pcfws.alarm_type[index + 1U];
            }
            pcfws.self_bottom_point--;
        }
    }
    loop1_raw_state_memory[addr] = 0U;
    setPointTypeMixtureDetectDisconnectMemory(addr, 0U);
    setPointTypeMixtureDetectTempertureMemory(addr, 0U);
    setPointTypeMixtureDetectSmokeMemory(addr, 0U);
}

static uint8_t PointTypeDetectorDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
    uint8_t fault_sum = 0U;
    (void)pcfs_entry;
    (void)pcfs_point;

    for(uint8_t addr = 1U; addr <= MIXTURE_DEVICE_MAX_ADDR; addr++)
    {
        uint8_t type;
        uint8_t raw_state;
        uint8_t old_state;
        uint16_t value;
        uint8_t display_type;

        if(getPointTypeMixtureSettingOnlieState(addr) == 0U)
        {
            if(loop1_raw_state_memory[addr] != 0U || getPointTypeMixtureDetectDisconnectMemory(addr) != 0U ||
                Loop1FindFault(addr, LOOP1_FAULT_TEMPERATURE) != 0xFFU ||
                Loop1FindFault(addr, LOOP1_FAULT_SMOKE_POLLUTION) != 0xFFU ||
                Loop1FindFault(addr, LOOP1_FAULT_SMOKE_SENSOR) != 0xFFU) Loop1ClearCurrentState(addr);
            continue;
        }

        type = getPointTypeMixtureDetectName(addr);
        display_type = (type == 6U) ? MBUS_FIRE_DISPLAY_DETECT_TEMP : MBUS_FIRE_DISPLAY_DETECT_SMOKE;
        if(getPointTypeMixtureDisconnectCount(addr) >= MIXTURE_DEVICE_DISCONNECT_SUM)
        {
            fault_sum++;
            if(getPointTypeMixtureDetectDisconnectMemory(addr) == 0U)
            {
                setPointTypeMixtureDetectDisconnectMemory(addr, 1U);
                Loop1AddFault(addr, LOOP1_FAULT_OFFLINE, DISCONNECT);
                MBusCtrl_PostFireDisplayEvent(1U, addr, display_type, MBUS_FIRE_DISPLAY_ALARM_FAULT);
            }
            continue;
        }

        if(getPointTypeMixtureDetectDisconnectMemory(addr) != 0U)
        {
            setPointTypeMixtureDetectDisconnectMemory(addr, 0U);
            Loop1RemoveFault(addr, LOOP1_FAULT_OFFLINE, DIS_RECOVERY);
        }

        if(type == 6U)
        {
            raw_state = getPointTypeMixtureReceiveState(PointTypeData_Temper, addr);
            value = getPointTypeMixtureReceiveData16(PointTypeData_Temper, addr);
        }
        else if(type == 5U)
        {
            raw_state = getPointTypeMixtureReceiveState(PointTypeData_Smoke, addr);
            value = getPointTypeMixtureReceiveData16(PointTypeData_Smoke, addr);
        }
        else
        {
            continue;
        }

        old_state = loop1_raw_state_memory[addr];
        if(old_state != raw_state)
        {
            if(type == 6U)
            {
                if(old_state == 1U) Loop1RemoveWarning(addr, Loop1TempWarning, LOOP1_TEMP_WARNING_RECOVERY, value);
                if(old_state == 3U) Loop1RemoveFault(addr, LOOP1_FAULT_TEMPERATURE, LOOP1_TEMP_SENSOR_RECOVERY);
                /* ?????:???????? */
                if(old_state == 3U) StorageEvent_LogFault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 1);
                if(old_state == 3U) FecbusReport_Fault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 1); /* FECbus:???????? */
                setPointTypeMixtureDetectTempertureMemory(addr, raw_state == 2U ? 1U : 0U);

                if(raw_state == 1U) Loop1AddWarning(addr, Loop1TempWarning, LOOP1_TEMP_WARNING, value);
                else if(raw_state == 2U)
                {
                    getBM8563TimeToSystemTime();
                    StoragePackFireAlarm(&pcfas, 0U, addr, Temperature);
                    fire_alarm_check_new_flag = 1U;
                    fire_alarm_state = 1U;
                    beep_fire_ctrl = 1U;
                    silencers_state = 0U;
                    BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, 0U, addr, value);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_TEMP, MBUS_FIRE_DISPLAY_ALARM_FIRE);
                    /* ?????:???Loop1????(????§Ø????) */
                    StorageEvent_LogFire(addr, DEV_TYPE_TEMPERATURE, 1, 0);
                    FecbusReport_Fire(addr, DEV_TYPE_TEMPERATURE, 1, 0); /* FECbus:???? */
                }
                else if(raw_state == 3U)
                {
                    Loop1AddFault(addr, LOOP1_FAULT_TEMPERATURE, LOOP1_TEMP_SENSOR_FAULT);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_TEMP, MBUS_FIRE_DISPLAY_ALARM_FAULT);
                    /* ?????:???????????? */
                    StorageEvent_LogFault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 0);
                    FecbusReport_Fault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 0); /* FECbus:?????? */
                }
            }
            else
            {
                if(old_state == 1U) Loop1RemoveWarning(addr, Loop1SmokeWarning, LOOP1_SMOKE_WARNING_RECOVERY, value);
                if(old_state == 8U) Loop1RemoveFault(addr, LOOP1_FAULT_SMOKE_POLLUTION, LOOP1_SMOKE_POLLUTION_RECOVERY);
                if(old_state == 9U) Loop1RemoveFault(addr, LOOP1_FAULT_SMOKE_SENSOR, LOOP1_SMOKE_SENSOR_RECOVERY);
                /* ?????:???????/???????????? */
                if(old_state == 8U) StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 1);
                if(old_state == 9U) StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 1);
                if(old_state == 8U) FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 1); /* FECbus:?????????? */
                if(old_state == 9U) FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 1); /* FECbus:???????????? */
                setPointTypeMixtureDetectSmokeMemory(addr, raw_state == 2U ? 1U : 0U);

                if(raw_state == 1U) Loop1AddWarning(addr, Loop1SmokeWarning, LOOP1_SMOKE_WARNING, value);
                else if(raw_state == 2U)
                {
                    getBM8563TimeToSystemTime();
                    StoragePackFireAlarm(&pcfas, 0U, addr, Smoke);
                    fire_alarm_check_new_flag = 1U;
                    fire_alarm_state = 1U;
                    beep_fire_ctrl = 1U;
                    silencers_state = 0U;
                    BspAlarmDataSaveApp(FIRE_FLASH_SAVE, SMOKE_ALARM, 0U, addr, value);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_SMOKE, MBUS_FIRE_DISPLAY_ALARM_FIRE);
                    /* ?????:???Loop1?????(????§Ø????) */
                    StorageEvent_LogFire(addr, DEV_TYPE_SMOKE, 1, 0);
                    FecbusReport_Fire(addr, DEV_TYPE_SMOKE, 1, 0); /* FECbus:????? */
                }
                else if(raw_state == 8U)
                {
                    Loop1AddFault(addr, LOOP1_FAULT_SMOKE_POLLUTION, LOOP1_SMOKE_POLLUTION_FAULT);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_SMOKE, MBUS_FIRE_DISPLAY_ALARM_FAULT);
                    /* ?????:??????????? */
                    StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 0);
                    FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 0); /* FECbus:??????????? */
                }
                else if(raw_state == 9U)
                {
                    Loop1AddFault(addr, LOOP1_FAULT_SMOKE_SENSOR, LOOP1_SMOKE_SENSOR_FAULT);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_SMOKE, MBUS_FIRE_DISPLAY_ALARM_FAULT);
                    /* ?????:????????????? */
                    StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 0);
                    FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 0); /* FECbus:????????????? */
                }
            }
            loop1_raw_state_memory[addr] = raw_state;
        }

        if(Loop1FindFault(addr, LOOP1_FAULT_TEMPERATURE) != 0xFFU ||
            Loop1FindFault(addr, LOOP1_FAULT_SMOKE_POLLUTION) != 0xFFU ||
            Loop1FindFault(addr, LOOP1_FAULT_SMOKE_SENSOR) != 0xFFU) fault_sum++;
    }

    if(pcfs_buttom_point > 0U) disconnect_state = 1U;
    return fault_sum;
}
uint8_t last_point_type_found_online_state = 1; // ????????????
static void PointTypeDetectorShowApp(PointTypeShowCtrl_t *ptsc_entry)
{
	uint8_t temp_screen_id = 67;
	
	ptsc_entry->curr_fresh_time_ctrl = osKernelGetTickCount(); // ?????????
	
	if(ptsc_entry->curr_fresh_time_ctrl - ptsc_entry->last_fresh_time_ctrl >= 3000 || ptsc_entry->poll_show_ctrl.key_perss_fresh != 0) // ????????
	{
		uint8_t show_addr;
		
		uint8_t found_online = 0;
		
		ptsc_entry->last_fresh_time_ctrl = ptsc_entry->curr_fresh_time_ctrl; // ?????????
		
		// ????????????????
		ptsc_entry->poll_show_ctrl.last_detector_id = ptsc_entry->poll_show_ctrl.poll_circuits_id;

		if(ptsc_entry->poll_show_ctrl.key_perss_fresh == 'p')
		{
			for(uint8_t test_addr = 0U; test_addr < MIXTURE_DEVICE_MAX_ADDR; test_addr++)
			{
				ptsc_entry->poll_show_ctrl.poll_circuits_id--; // ???????? ???????????????
				
				if(ptsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || ptsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					ptsc_entry->poll_show_ctrl.poll_circuits_id = MIXTURE_DEVICE_MAX_ADDR;
				}
				
				if(getPointTypeMixtureDetectOnlineState( ptsc_entry->poll_show_ctrl.poll_circuits_id ) == 1)
				{
					found_online = 1;
					last_point_type_found_online_state = 1; // ?????????????????1 ?????????¦Æ??????????????????
					show_addr = ptsc_entry->poll_show_ctrl.poll_circuits_id;

					break;
				}
			}
			ptsc_entry->poll_show_ctrl.key_perss_fresh = 0;
		}
		else
		{
			for(uint8_t test_addr = 0U; test_addr < MIXTURE_DEVICE_MAX_ADDR; test_addr++)
			{
				ptsc_entry->poll_show_ctrl.poll_circuits_id++; // ???????? ???????????????
				
				if(ptsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || ptsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					ptsc_entry->poll_show_ctrl.poll_circuits_id = 1;
				}
				
				if(getPointTypeMixtureDetectOnlineState( ptsc_entry->poll_show_ctrl.poll_circuits_id ) == 1)
				{
					found_online = 1;
					last_point_type_found_online_state = 1; // ?????????????????1 ?????????¦Æ??????????????????
					show_addr = ptsc_entry->poll_show_ctrl.poll_circuits_id;

					break;
				}
			}
			
			ptsc_entry->poll_show_ctrl.key_perss_fresh = 0;
		}

		if(found_online == 0 && last_point_type_found_online_state != 0) // ?????§Ù?????????????
		{
			last_point_type_found_online_state = 0; // ????????
			ptsc_entry->poll_show_ctrl.poll_circuits_id = ptsc_entry->poll_show_ctrl.last_detector_id; // ????????????
			
			// ??????????
			SetTextValue(temp_screen_id, 33, "???¦Ê??????????"); //?????
			clearTextValue(temp_screen_id , 34); //(????ID,???ID??
			clearTextValue(temp_screen_id , 35); //(????ID,???ID??
			clearTextValue(temp_screen_id , 36); //(????ID,???ID??
			clearTextValue(temp_screen_id , 37); //(????ID,???ID??
			clearTextValue(temp_screen_id , 38); //(????ID,???ID??
			clearTextValue(temp_screen_id , 39); //(????ID,???ID??
		}
		else if(found_online != 0) // ??????????????? ????????
		{
			uint8_t temp_detect_type;
			
			uint8_t temp_buff[64] = {0};
			
			if(getPointTypeMixtureDisconnectCount(show_addr) < MIXTURE_DEVICE_DISCONNECT_SUM) // ?????????
			{
//				if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ???????? | ?????????", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ???????? | ???????", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ?????? | ?????????", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ?????? | ???????", show_addr);
//				}
				sprintf((char *)temp_buff, "?????%d??:????", show_addr);
				SetTextValue(temp_screen_id, 33, temp_buff); //?????
				
				// ???????????
				temp_detect_type = getPointTypeMixtureDetectName(show_addr);
				switch(temp_detect_type)
				{
					case 1: {
						SetTextValue(temp_screen_id, 34, "????????:XR805-V2.0"); //?????
						break;
					}
					case 2: {
						SetTextValue(temp_screen_id, 34, "????????:XR805-EXD"); //?????
						break;
					}
					case 3: {
						SetTextValue(temp_screen_id, 34, "????????:XR805-EXi"); //?????
						break;
					}
					case 4: {
						SetTextValue(temp_screen_id, 34, "????????:XR-DLYGWG"); //?????
						break;
					}
					case 5: {
						SetTextValue(temp_screen_id, 34, "????????:JTY-XR800B"); //?????
						break;
					}
					case 6: {
						SetTextValue(temp_screen_id, 34, "????????:JTY-ZDM-XR8002/C"); //?????
						break;
					}
					case 7: {
						SetTextValue(temp_screen_id, 34, "????????:JTY-GD-XR8001AI"); //?????
						break;
					}
					default: {
						SetTextValue(temp_screen_id, 34, "????????:--"); //?????
						break;
					}
				}
				
				// ????????????????????
				uint8_t sensor_enable_state = getPointTypeMixtureDetectType(show_addr);
				if(sensor_enable_state == 0)
				{
					SetTextValue(temp_screen_id, 35, "????????????:??????????");
				}
				else
				{
					uint8_t first_sensor = 1;  // ??????????????????
					uint8_t pos = 0;
					
					pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
					for(uint8_t i = 0; i < 6; i++)
					{
						if( (sensor_enable_state >> i) & 0x01 )
						{
							// ???????????????????
							if(!first_sensor)
							{
									pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "/");
							}
							else
							{
									first_sensor = 0;
							}
							pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[i]);
						}
					}
					SetTextValue(temp_screen_id, 35, temp_buff);
				}
				
				clearTextValue(temp_screen_id , 36); //(????ID,???ID??
				clearTextValue(temp_screen_id , 37); //(????ID,???ID??
				clearTextValue(temp_screen_id , 38); //(????ID,???ID??
				clearTextValue(temp_screen_id , 39); //(????ID,???ID??
				
				// ??????????????
				uint8_t screen_show_id_offset = 0;
				if(sensor_enable_state & 0x20) // ?§Ø???????????
				{
					sprintf((char *)temp_buff, "????:%d??", getPointTypeMixtureReceiveData(PointTypeData_Temper, show_addr));
					SetTextValue(temp_screen_id, 36 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // ???????? 
				}
				if(sensor_enable_state & 0x01) // ?§Ø????????????
				{
					if(getPointTypeMixtureReceiveState(PointTypeData_Smoke, show_addr) == 1)
					{
						SetTextValue(temp_screen_id, 36 + screen_show_id_offset, "??????:????");
					}
					else
					{
						SetTextValue(temp_screen_id, 36 + screen_show_id_offset, "??????:????");
					}
					screen_show_id_offset++; // ???????? 
				}
				if(sensor_enable_state & 0x10)
				{
					
					screen_show_id_offset++; // ???????? 
				}
				
				
			}
			else // ???????
			{
				sprintf((char *)temp_buff, "?????%d??:????", show_addr);
				SetTextValue(temp_screen_id, 33, temp_buff); //?????
				SetTextValue(temp_screen_id, 34, "????????:--"); //?????
				SetTextValue(temp_screen_id, 35, "????????????:--");
				clearTextValue(temp_screen_id , 36); //(????ID,???ID??
				clearTextValue(temp_screen_id , 37); //(????ID,???ID??
				clearTextValue(temp_screen_id , 38); //(????ID,???ID??
				clearTextValue(temp_screen_id , 39); //(????ID,???ID??
			}
		} // ???????????????????
	} // ????????? ??????
	
	// ??????
	if(ptsc_entry->verb_show_ctrl.verb_detector_id == 0) // ?????0 ??§Ó??ID
	{
		ptsc_entry->verb_show_ctrl.verb_detector_id = 255; // ????????
		SetTextValue(temp_screen_id, 23, "?????????????ID"); //?????
		clearTextValue(temp_screen_id , 24); //(????ID,???ID??
		clearTextValue(temp_screen_id , 25); //(????ID,???ID??
		clearTextValue(temp_screen_id , 26); //(????ID,???ID??
		clearTextValue(temp_screen_id , 27); //(????ID,???ID??
		clearTextValue(temp_screen_id , 28); //(????ID,???ID??
		clearTextValue(temp_screen_id , 29); //(????ID,???ID??
		SetTextValue(temp_screen_id, 30, "???????ID"); //?????
	}
	else if(ptsc_entry->verb_show_ctrl.verb_detector_id != 255) // ????????ID???? ??????????????
	{
		uint8_t show_addr = ptsc_entry->verb_show_ctrl.verb_detector_id;
		
		if(getPointTypeMixtureDetectOnlineState( show_addr ) == 1) // ?????????????
		{
			uint8_t temp_buff[64] = {0};
			// ?§Ø?????????????
			if(getPointTypeMixtureDisconnectCount(show_addr) < MIXTURE_DEVICE_DISCONNECT_SUM)
			{
				// ???id???
				if(show_addr != ptsc_entry->verb_show_ctrl.last_detector_id || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.last_detector_id = show_addr; // ????????
					
					sprintf((char *)temp_buff, "?????%d??:????", show_addr);
					SetTextValue(temp_screen_id, 23, temp_buff); //?????
				}
				
				ptsc_entry->verb_show_ctrl.verb_detect_name = getPointTypeMixtureDetectName(show_addr);
				
				if(ptsc_entry->verb_show_ctrl.lsat_detect_name != ptsc_entry->verb_show_ctrl.verb_detect_name || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.lsat_detect_name = ptsc_entry->verb_show_ctrl.verb_detect_name;
					
					// ???????????
					switch(ptsc_entry->verb_show_ctrl.verb_detect_name)
					{
						case 1: {
							SetTextValue(temp_screen_id, 24, "????????:XR805-V2.0"); //?????
							break;
						}
						case 2: {
							SetTextValue(temp_screen_id, 24, "????????:XR805-EXD"); //?????
							break;
						}
						case 3: {
							SetTextValue(temp_screen_id, 24, "????????:XR805-EXi"); //?????
							break;
						}
						case 4: {
							SetTextValue(temp_screen_id, 24, "????????:XR-DLYGWG"); //?????
							break;
						}
						case 5: {
							SetTextValue(temp_screen_id, 24, "????????:JTY-XR800B"); //?????
							break;
						}
						case 6: {
							SetTextValue(temp_screen_id, 24, "????????:JTY-ZDM-XR8002/C"); //?????
							break;
						}
						case 7: {
							SetTextValue(temp_screen_id, 24, "????????:JTY-GD-XR8001AI"); //?????
							break;
						}
						default: {
							SetTextValue(temp_screen_id, 24, "????????:--"); //?????
							break;
						}
					}
				}
				
				// ????????????????????
				ptsc_entry->verb_show_ctrl.verb_sensor_state = getPointTypeMixtureDetectType(show_addr);
				
				if(ptsc_entry->verb_show_ctrl.last_sensor_state != ptsc_entry->verb_show_ctrl.verb_sensor_state || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.last_sensor_state = ptsc_entry->verb_show_ctrl.verb_sensor_state;
					
					if(ptsc_entry->verb_show_ctrl.verb_sensor_state == 0)
					{
						SetTextValue(temp_screen_id, 25, "????????????:??????????");
					}
					else
					{
						uint8_t first_sensor = 1;  // ??????????????????
						uint8_t pos = 0;
						
						pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
						for(uint8_t i = 0; i < 6; i++)
						{
							if( (ptsc_entry->verb_show_ctrl.verb_sensor_state >> i) & 0x01 )
							{
								// ???????????????????
								if(!first_sensor)
								{
										pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "/");
								}
								else
								{
										first_sensor = 0;
								}
								pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[i]);
							}
						}
						SetTextValue(temp_screen_id, 25, temp_buff);
					}
					
					clearTextValue(temp_screen_id , 26); //(????ID,???ID??
					clearTextValue(temp_screen_id , 27); //(????ID,???ID??
					clearTextValue(temp_screen_id , 28); //(????ID,???ID??
					clearTextValue(temp_screen_id , 29); //(????ID,???ID??
				}

				// ??????????????
				uint8_t screen_show_id_offset = 0;
				if(ptsc_entry->verb_show_ctrl.verb_sensor_state & 0x20) // ?§Ø???????????
				{
					ptsc_entry->verb_show_ctrl.verb_temper_value = getPointTypeMixtureReceiveData(PointTypeData_Temper, show_addr);
					if(ptsc_entry->verb_show_ctrl.verb_temper_value != ptsc_entry->verb_show_ctrl.last_temper_value || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						ptsc_entry->verb_show_ctrl.last_temper_value = ptsc_entry->verb_show_ctrl.verb_temper_value;
						sprintf((char *)temp_buff, "????:%d??", ptsc_entry->verb_show_ctrl.verb_temper_value);
						SetTextValue(temp_screen_id, 26 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // ???????? 
					}
				}
				if(ptsc_entry->verb_show_ctrl.verb_sensor_state & 0x01) // ?§Ø????????????
				{
					ptsc_entry->verb_show_ctrl.verb_smokes_state = getPointTypeMixtureReceiveState(PointTypeData_Smoke, show_addr);
					
					if(ptsc_entry->verb_show_ctrl.verb_smokes_state != ptsc_entry->verb_show_ctrl.last_smokes_state || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						ptsc_entry->verb_show_ctrl.last_smokes_state = ptsc_entry->verb_show_ctrl.verb_smokes_state;
						
						if(ptsc_entry->verb_show_ctrl.verb_smokes_state == 1)
						{
							SetTextValue(temp_screen_id, 26 + screen_show_id_offset, "??????:????");
						}
						else
						{
							SetTextValue(temp_screen_id, 26 + screen_show_id_offset, "??????:????");
						}
						screen_show_id_offset++; // ???????? 
					}
				}
				if(ptsc_entry->verb_show_ctrl.verb_sensor_state & 0x10)
				{
					
					screen_show_id_offset++; // ???????? 
				}
			}
			else // ???????? ????????
			{
				if(show_addr != ptsc_entry->verb_show_ctrl.last_detector_id || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.last_detector_id = show_addr; // ????????
					
					sprintf((char *)temp_buff, "?????%d??:????", show_addr);
					SetTextValue(temp_screen_id, 23, temp_buff); //?????
					SetTextValue(temp_screen_id, 24, "????????:--"); //?????
					SetTextValue(temp_screen_id, 25, "????????????:--");
					clearTextValue(temp_screen_id , 26); //(????ID,???ID??
					clearTextValue(temp_screen_id , 27); //(????ID,???ID??
					clearTextValue(temp_screen_id , 28); //(????ID,???ID??
					clearTextValue(temp_screen_id , 29); //(????ID,???ID??
				}
			}
			ptsc_entry->verb_show_ctrl.force_fresh_ctrl = 0;
		}
		else // ?????????
		{
			ptsc_entry->verb_show_ctrl.verb_detector_id = 255; // ????????
			SetTextValue(temp_screen_id, 23, "???????¦Ä????"); //?????
			clearTextValue(temp_screen_id , 24); //(????ID,???ID??
			clearTextValue(temp_screen_id , 25); //(????ID,???ID??
			clearTextValue(temp_screen_id , 26); //(????ID,???ID??
			clearTextValue(temp_screen_id , 27); //(????ID,???ID??
			clearTextValue(temp_screen_id , 28); //(????ID,???ID??
			clearTextValue(temp_screen_id , 29); //(????ID,???ID??
		}

	}
}

static void PointTypeDetectorButtonCtrlApp(PointTypeShowCtrl_t *ptsc_entry, uint16 control_id, uint8  state)
{
	if(control_id == 32 && state == 1)
	{
		ptsc_entry->poll_show_ctrl.key_perss_fresh = 'n';
	}
	else if(control_id == 31 && state == 1)
	{
		ptsc_entry->poll_show_ctrl.key_perss_fresh = 'p';
	}
}

static void PointTypeDetectorTextInputCtrlApp(PointTypeShowCtrl_t *ptsc_entry, uint16 control_id, uint8 *str)
{
	int32 value=0;  			
	sscanf((const char*)str,"%ld",&value); //??????????????? 
	if(control_id == 30)    
	{
		ptsc_entry->verb_show_ctrl.verb_detector_id = value;
		
		ptsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // ?????????
	}
}

static void PointTypeDetectorScreenSwitchShowApp(PointTypeShowCtrl_t *ptsc_entry)
{
	ptsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // ?????????????????
}

// ????????????
uint8_t last_composite_found_online_state = 1; // ??????????????????????
void CompositeDetectorPollCtrl(CompositeShowCtrl_t *cpsc_entry)
{
	uint8_t temp_screen_id = 67;
	
	cpsc_entry->curr_fresh_time_ctrl = osKernelGetTickCount(); // ?????????
	
	if(cpsc_entry->curr_fresh_time_ctrl - cpsc_entry->last_fresh_time_ctrl >= 3000 || cpsc_entry->poll_show_ctrl.key_perss_fresh != 0) // ????????
	{
		uint8_t show_addr;
		
		uint8_t found_online = 0;
		
		cpsc_entry->last_fresh_time_ctrl = cpsc_entry->curr_fresh_time_ctrl; // ?????????
		
		// ????????????????
		cpsc_entry->poll_show_ctrl.last_detector_id = cpsc_entry->poll_show_ctrl.poll_circuits_id;

		if(cpsc_entry->poll_show_ctrl.key_perss_fresh == 'p')
		{
			for(uint8_t test_addr = 0U; test_addr < MIXTURE_DEVICE_MAX_ADDR; test_addr++)
			{
				cpsc_entry->poll_show_ctrl.poll_circuits_id--; // ???????? ???????????????
				
				if(cpsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || cpsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					cpsc_entry->poll_show_ctrl.poll_circuits_id = MIXTURE_DEVICE_MAX_ADDR;
				}
				
				// ??????? ??????????§Ø??? ?????????????????§Ø?
				if( cang_sxzt[ cpsc_entry->poll_show_ctrl.poll_circuits_id ] == 1 )
				{
					found_online = 1;
					last_composite_found_online_state = 1; // ?????????????????1 ?????????¦Æ??????????????????
					show_addr = cpsc_entry->poll_show_ctrl.poll_circuits_id;

					break;
				}
			}
			cpsc_entry->poll_show_ctrl.key_perss_fresh = 0;
		}
		else
		{
			for(uint8_t test_addr = 0U; test_addr < MIXTURE_DEVICE_MAX_ADDR; test_addr++)
			{
				cpsc_entry->poll_show_ctrl.poll_circuits_id++; // ???????? ???????????????
				
				if(cpsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || cpsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					cpsc_entry->poll_show_ctrl.poll_circuits_id = 1;
				}
				
				if( cang_sxzt[ cpsc_entry->poll_show_ctrl.poll_circuits_id ] == 1 )
				{
					found_online = 1;
					last_composite_found_online_state = 1; // ?????????????????1 ?????????¦Æ??????????????????
					show_addr = cpsc_entry->poll_show_ctrl.poll_circuits_id;

					break;
				}
			}
			
			cpsc_entry->poll_show_ctrl.key_perss_fresh = 0;
		}

		if(found_online == 0 && last_composite_found_online_state != 0) // ?????§Ù?????????????
		{
			last_composite_found_online_state = 0; // ????????
			cpsc_entry->poll_show_ctrl.poll_circuits_id = cpsc_entry->poll_show_ctrl.last_detector_id; // ????????????
			
			// ??????????
			SetTextValue(temp_screen_id, 15, "???¦Ê??????????"); //?????
			clearTextValue(temp_screen_id , 16); //(????ID,???ID??
			clearTextValue(temp_screen_id , 17); //(????ID,???ID??
			clearTextValue(temp_screen_id , 18); //(????ID,???ID??
			clearTextValue(temp_screen_id , 19); //(????ID,???ID??
			clearTextValue(temp_screen_id , 20); //(????ID,???ID??
			clearTextValue(temp_screen_id , 21); //(????ID,???ID??
		}
		else if(found_online != 0) // ??????????????? ????????
		{
			uint8_t temp_buff[64] = {0};
			
			// ??????? ?§Ø???????????
			if( Cang_zx_buf[ cpsc_entry->poll_show_ctrl.poll_circuits_id ] < CabinDisconnectCount ) // ?????????
			{
//				if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ???????? | ?????????", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ???????? | ???????", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ?????? | ?????????", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "?????%d??:???? | ?????? | ???????", show_addr);
//				}
				sprintf((char *)temp_buff, "?????%d??:????", show_addr);
				SetTextValue(temp_screen_id, 15, temp_buff); //?????
				
				// ??????????? ??????? 805xxx?????????????
				switch( Cang_TCQXH_buf[ show_addr ] )
				{
					case 1: {
						SetTextValue(temp_screen_id, 16, "????????:XR805-V2.0"); //?????
						break;
					}
					case 2: {
						SetTextValue(temp_screen_id, 16, "????????:XR805-EXD"); //?????
						break;
					}
					case 3: {
						SetTextValue(temp_screen_id, 16, "????????:XR805-EXi"); //?????
						break;
					}
					case 4: {
						SetTextValue(temp_screen_id, 16, "????????:XR-DLYGWG"); //?????
						break;
					}
					case 5: {
						SetTextValue(temp_screen_id, 16, "????????:JTY-XR800B"); //?????
						break;
					}
					case 6: {
						SetTextValue(temp_screen_id, 16, "????????:JTY-ZDM-XR8002/C"); //?????
						break;
					}
					case 7: {
						SetTextValue(temp_screen_id, 16, "????????:JTY-GD-XR8001AI"); //?????
						break;
					}
					default: {
						SetTextValue(temp_screen_id, 16, "????????:--"); //?????
						break;
					}
				}

				// ???????????????????? ??????? 805xxx?????????????
				uint8_t sensor_enable_state = Cang_CGQQY_buf[ show_addr ];
				if(sensor_enable_state  == 0 )
				{
					SetTextValue(temp_screen_id, 17, "????????????:??????????");
				}
				else
				{
					uint8_t first_sensor = 1;  // ??????????????????
					uint8_t pos = 0;
					
					pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
					for(uint8_t i = 0; i < 6; i++)
					{
						if( (sensor_enable_state >> i) & 0x01 )
						{
							// ???????????????????
							if(!first_sensor)
							{
									pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "/");
							}
							else
							{
									first_sensor = 0;
							}
							pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[i]);
						}
					}
					SetTextValue(temp_screen_id, 17, temp_buff);
				}
				
				clearTextValue(temp_screen_id , 18); //(????ID,???ID??
				clearTextValue(temp_screen_id , 19); //(????ID,???ID??
				clearTextValue(temp_screen_id , 20); //(????ID,???ID??
				clearTextValue(temp_screen_id , 21); //(????ID,???ID??
	
				// ??????????????
				uint8_t screen_show_id_offset = 0;
				if(sensor_enable_state & 0x20) // ?§Ø???????????
				{
					// ??????? 805xxx?????????????
					sprintf((char *)temp_buff, "????:%d??", Cang_wendu_buf[ show_addr ] );
					SetTextValue(temp_screen_id, 18 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // ???????? 
				}
				if(sensor_enable_state & 0x01) // ?§Ø????????????
				{
					if(Cang_YWZT_buf[ show_addr ] == 1)
					{
						SetTextValue(temp_screen_id, 18 + screen_show_id_offset, "??????:????");
					}
					else
					{
						SetTextValue(temp_screen_id, 18 + screen_show_id_offset, "??????:????");
					}
					screen_show_id_offset++; // ???????? 
				}
				if(sensor_enable_state & 0x10) // ?§Ø???????????????????
				{
					// ??????? 805xxx?????????????
					sprintf((char *)temp_buff, "?????????:%dPPM", Cang_COzhi_buf[ show_addr ] );
					SetTextValue(temp_screen_id, 18 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // ???????? 
				}
				if(sensor_enable_state & 0x04) // ?§Ø????????????
				{
					// ??????? 805xxx?????????????
					sprintf((char *)temp_buff, "???????:%dPPM", Cang_H2zhi_buf[show_addr] );
					SetTextValue(temp_screen_id, 18 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // ???????? 
				}
			}
			else // ???????
			{
				sprintf((char *)temp_buff, "?????%d??:????", show_addr);
				SetTextValue(temp_screen_id, 15, temp_buff); //?????
				SetTextValue(temp_screen_id, 16, "????????:--"); //?????
				SetTextValue(temp_screen_id, 17, "????????????:--");
				clearTextValue(temp_screen_id , 18); //(????ID,???ID??
				clearTextValue(temp_screen_id , 19); //(????ID,???ID??
				clearTextValue(temp_screen_id , 20); //(????ID,???ID??
				clearTextValue(temp_screen_id , 21); //(????ID,???ID??
			}
		} // ???????????????????
	} // ????????? ??????
}

void CompositeDetectorVerbCtrl(CompositeShowCtrl_t *cpsc_entry)
{
	uint8_t temp_screen_id = 67;
	// ??????
	if(cpsc_entry->verb_show_ctrl.verb_detector_id == 0) // ?????0 ??§Ó??ID
	{
		cpsc_entry->verb_show_ctrl.verb_detector_id = 255; // ????????
		SetTextValue(temp_screen_id, 5, "?????????????ID"); //?????
		clearTextValue(temp_screen_id , 6); //(????ID,???ID??
		clearTextValue(temp_screen_id , 7); //(????ID,???ID??
		clearTextValue(temp_screen_id , 8); //(????ID,???ID??
		clearTextValue(temp_screen_id , 9); //(????ID,???ID??
		clearTextValue(temp_screen_id , 10); //(????ID,???ID??
		clearTextValue(temp_screen_id , 11); //(????ID,???ID??
		
		SetTextValue(temp_screen_id, 12, "???????ID"); //?????
	}
	else if(cpsc_entry->verb_show_ctrl.verb_detector_id != 255) // ????????ID???? ??????????????
	{
		uint8_t show_addr = cpsc_entry->verb_show_ctrl.verb_detector_id;
		
		// 1 ???
		if( cang_sxzt[ show_addr ] == 1 ) // ?????????????
		{
			uint8_t temp_buff[64] = {0};
			// ?§Ø????????????? 2 ???
			if( Cang_zx_buf[ show_addr ] < CabinDisconnectCount )
			{
				// ???id???
				if(show_addr != cpsc_entry->verb_show_ctrl.last_detector_id || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.last_detector_id = show_addr; // ????????
					
					sprintf((char *)temp_buff, "?????%d??:????", show_addr);
					SetTextValue(temp_screen_id, 5, temp_buff); //?????
				}
				
				// 3 ???
				cpsc_entry->verb_show_ctrl.verb_detect_name = Cang_TCQXH_buf[ show_addr ];
				
				if(cpsc_entry->verb_show_ctrl.lsat_detect_name != cpsc_entry->verb_show_ctrl.verb_detect_name || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.lsat_detect_name = cpsc_entry->verb_show_ctrl.verb_detect_name;
					
					// ???????????
					switch(cpsc_entry->verb_show_ctrl.verb_detect_name)
					{
						case 1: {
							SetTextValue(temp_screen_id, 6, "????????:XR805-V2.0"); //?????
							break;
						}
						case 2: {
							SetTextValue(temp_screen_id, 6, "????????:XR805-EXD"); //?????
							break;
						}
						case 3: {
							SetTextValue(temp_screen_id, 6, "????????:XR805-EXi"); //?????
							break;
						}
						case 4: {
							SetTextValue(temp_screen_id, 6, "????????:XR-DLYGWG"); //?????
							break;
						}
						case 5: {
							SetTextValue(temp_screen_id, 6, "????????:JTY-XR800B"); //?????
							break;
						}
						case 6: {
							SetTextValue(temp_screen_id, 6, "????????:JTY-ZDM-XR8002/C"); //?????
							break;
						}
						case 7: {
							SetTextValue(temp_screen_id, 6, "????????:JTY-GD-XR8001AI"); //?????
							break;
						}
						default: {
							SetTextValue(temp_screen_id, 6, "????????:--"); //?????
							break;
						}
					}
				}
				
				// ???????????????????? 4 ???
				cpsc_entry->verb_show_ctrl.verb_sensor_state = Cang_CGQQY_buf[ show_addr ];
				
				if(cpsc_entry->verb_show_ctrl.last_sensor_state != cpsc_entry->verb_show_ctrl.verb_sensor_state || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.last_sensor_state = cpsc_entry->verb_show_ctrl.verb_sensor_state;
					
					if(cpsc_entry->verb_show_ctrl.verb_sensor_state == 0)
					{
						SetTextValue(temp_screen_id, 7, "????????????:??????????");
					}
					else
					{
						uint8_t first_sensor = 1;  // ??????????????????
						uint8_t pos = 0;
						
						pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
						for(uint8_t i = 0; i < 6; i++)
						{
							if( (cpsc_entry->verb_show_ctrl.verb_sensor_state >> i) & 0x01 )
							{
								// ???????????????????
								if(!first_sensor)
								{
										pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "/");
								}
								else
								{
										first_sensor = 0;
								}
								pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[i]);
							}
						}
						SetTextValue(temp_screen_id, 7, temp_buff);
					}
					
					clearTextValue(temp_screen_id , 8); //(????ID,???ID??
					clearTextValue(temp_screen_id , 9); //(????ID,???ID??
					clearTextValue(temp_screen_id , 10); //(????ID,???ID??
					clearTextValue(temp_screen_id , 11); //(????ID,???ID??
				}

				// ??????????????
				uint8_t screen_show_id_offset = 0;
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x20) // ?§Ø???????????
				{
					cpsc_entry->verb_show_ctrl.verb_temper_value = Cang_wendu_buf[ show_addr ];
					if(cpsc_entry->verb_show_ctrl.verb_temper_value != cpsc_entry->verb_show_ctrl.last_temper_value || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						cpsc_entry->verb_show_ctrl.last_temper_value = cpsc_entry->verb_show_ctrl.verb_temper_value;
						sprintf((char *)temp_buff, "????:%d??", cpsc_entry->verb_show_ctrl.verb_temper_value);
						SetTextValue(temp_screen_id, 8 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // ???????? 
					}
				}
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x01) // ?§Ø????????????
				{
					cpsc_entry->verb_show_ctrl.verb_smokes_state = Cang_wendu_buf[ show_addr ];
					
					if(cpsc_entry->verb_show_ctrl.verb_smokes_state != cpsc_entry->verb_show_ctrl.last_smokes_state || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						cpsc_entry->verb_show_ctrl.last_smokes_state = cpsc_entry->verb_show_ctrl.verb_smokes_state;
						
						if(cpsc_entry->verb_show_ctrl.verb_smokes_state == 1)
						{
							SetTextValue(temp_screen_id, 8 + screen_show_id_offset, "??????:????");
						}
						else
						{
							SetTextValue(temp_screen_id, 8 + screen_show_id_offset, "??????:????");
						}
						screen_show_id_offset++; // ???????? 
					}
				}
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x10) // ?§Ø???????????????????
				{
					// ??????? 805xxx?????????????
					cpsc_entry->verb_show_ctrl.verb_carbon_value = Cang_COzhi_buf[ show_addr ];
					if(cpsc_entry->verb_show_ctrl.last_carbon_value != cpsc_entry->verb_show_ctrl.verb_carbon_value || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						// ??????????
						cpsc_entry->verb_show_ctrl.last_carbon_value = cpsc_entry->verb_show_ctrl.verb_carbon_value;
						
						sprintf((char *)temp_buff, "?????????:%dPPM", cpsc_entry->verb_show_ctrl.verb_carbon_value );
						SetTextValue(temp_screen_id, 8 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // ???????? 
					}
				}
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x04) // ?§Ø????????????
				{
					
					cpsc_entry->verb_show_ctrl.verb_hydrog_value = Cang_H2zhi_buf[ show_addr ]; 
					
					if(cpsc_entry->verb_show_ctrl.lsat_hydrog_value != cpsc_entry->verb_show_ctrl.verb_hydrog_value || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						// ????????
						cpsc_entry->verb_show_ctrl.lsat_hydrog_value = cpsc_entry->verb_show_ctrl.verb_hydrog_value;
						
						// ??????? 805xxx?????????????
						sprintf((char *)temp_buff, "???????:%dPPM", cpsc_entry->verb_show_ctrl.verb_hydrog_value );
						SetTextValue(temp_screen_id, 8 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // ???????? 
					}
				}
			}
			else // ???????? ????????
			{
				if(show_addr != cpsc_entry->verb_show_ctrl.last_detector_id || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.last_detector_id = show_addr; // ????????
					
					sprintf((char *)temp_buff, "?????%d??:????", show_addr);
					SetTextValue(temp_screen_id, 5, temp_buff); //?????
					SetTextValue(temp_screen_id, 6, "????????:--"); //?????
					SetTextValue(temp_screen_id, 7, "????????????:--");
					clearTextValue(temp_screen_id , 8); //(????ID,???ID??
					clearTextValue(temp_screen_id , 9); //(????ID,???ID??
					clearTextValue(temp_screen_id , 10); //(????ID,???ID??
					clearTextValue(temp_screen_id , 11); //(????ID,???ID??
				}
			}
			cpsc_entry->verb_show_ctrl.force_fresh_ctrl = 0;
		}
		else // ?????????
		{
			cpsc_entry->verb_show_ctrl.verb_detector_id = 255; // ????????
			SetTextValue(temp_screen_id, 5, "???????¦Ä????"); //?????
			clearTextValue(temp_screen_id , 6); //(????ID,???ID??
			clearTextValue(temp_screen_id , 7); //(????ID,???ID??
			clearTextValue(temp_screen_id , 8); //(????ID,???ID??
			clearTextValue(temp_screen_id , 9); //(????ID,???ID??
			clearTextValue(temp_screen_id , 10); //(????ID,???ID??
			clearTextValue(temp_screen_id , 11); //(????ID,???ID??
		}

	}
}

static void CompositeDetectorShowApp(CompositeShowCtrl_t *cpsc_entry)
{
	CompositeDetectorPollCtrl(cpsc_entry);
	CompositeDetectorVerbCtrl(cpsc_entry);
}

static void CompositeDetectorButtonCtrlApp(CompositeShowCtrl_t *cpsc_entry, uint16 control_id, uint8  state)
{
	if(control_id == 14 && state == 1)
	{
		cpsc_entry->poll_show_ctrl.key_perss_fresh = 'n';
	}
	else if(control_id == 13 && state == 1)
	{
		cpsc_entry->poll_show_ctrl.key_perss_fresh = 'p';
	}
}

static void CompositeDetectorTextInputCtrlApp(CompositeShowCtrl_t *cpsc_entry, uint16 control_id, uint8 *str)
{
	int32 value=0;  			
	sscanf((const char*)str,"%ld",&value); //??????????????? 
	if(control_id == 12)    
	{
		cpsc_entry->verb_show_ctrl.verb_detector_id = value;
		
		cpsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // ?????????
	}
}

static void CompositeDetectorScreenSwitchShowApp(CompositeShowCtrl_t *cpsc_entry)
{
	cpsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // ?????????????????
}

UART_HandleTypeDef *getSimulateSirealPortSendHandle(uint8_t port_comid)
{
	UART_HandleTypeDef *uart_handle = NULL;
	
	switch(port_comid)
	{
		case 1:{
			uart_handle = &huart7; // MBUS 1
			break;
		}
		case 2:{
			uart_handle = &huart2; // MBUS 2
			break;
		}
		case 3:{
			uart_handle = &huart9; // PACK A/B
			break;
		}
		case 4:{
			uart_handle = &huart3; // EMS????
			break;
		}
		case 5:{
			uart_handle = NULL; /* FECbus: USART1 freed for master, station port NULL */
			break;
		}
		case 6:{
			uart_handle = NULL; /* XR5000_UART5_EXCLUSIVE_FIX_20260730 */ // ???????????
			break;
		}
		default:{
			uart_handle = NULL;
			break;
		}
	}

	return uart_handle;
}

eUartOrder getSimulateSirealPortReceiveIndex(uint8_t port_comid)
{
	eUartOrder temp_order = ERRORSITE;
	switch(port_comid)
	{
		case 1:{
			temp_order = MBUS1SITE; // MBUS 1
			break;
		}
		case 2:{
			temp_order = MBUS2SITE; // MBUS 2
			break;
		}
		case 3:{
			temp_order = PACKSITE; // PACK A/B
			break;
		}
		case 4:{
			temp_order = EMSSITE; // EMS????
			break;
		}
		case 5:{
			temp_order = STATION_OPTICALFIBER; // ???????
			break;
		}
		case 6:{
			temp_order = ERRORSITE; /* XR5000_UART5_EXCLUSIVE_FIX_20260730 */ // ???????????
			break;
		}
		default:{
			temp_order = ERRORSITE;
			break;
		}
	}
	
	return temp_order;
}

static void SimulationSerialPortFirstFresh(SimulationSerialPortAssistant_t *sspa_entry)
{
	if(sspa_entry->serial_port_state == 0)
	{
		SetTextValue(3, 9, "??????"); //?????
	}
	else
	{
		SetTextValue(3, 9, "??????"); //?????
	}
	
	if(sspa_entry->serial_port_comid == 0 || sspa_entry->serial_port_comid == 0xFF)
	{
		clearTextValue(3, 6); //(????ID,???ID??
	}
	
	if(sspa_entry->serial_port_send_mode == 0)
	{
		SetTextValue(3, 32, "?????????"); //?????
	}
	else
	{
		SetTextValue(3, 32, "16???????"); //?????
	}
	
	if(sspa_entry->serial_port_show_mode == 0)
	{
		SetTextValue(3, 36, "?????????"); //?????
	}
	else
	{
		SetTextValue(3, 36, "16???????"); //?????
	}
	
	if(sspa_entry->serial_port_send_new_row == 0)
	{
		SetTextValue(3, 38, "??????????"); //?????
	}
	else
	{
		SetTextValue(3, 38, "????????"); //?????
	}
	
	if(sspa_entry->serial_port_send_len == 0)
	{
		clearTextValue(3, 28); //(????ID,???ID??
	}

}

static void SimulationSerialPortButtonCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t ctrl_id, uint8_t state)
{
	UART_HandleTypeDef *temp_uart = NULL;
	
	switch(ctrl_id)
	{
		case 10: { // ?????????
			if(sspa_entry->serial_port_state == 0) // ????????????????
			{
				if(sspa_entry->serial_port_comid != 0xFF && sspa_entry->serial_port_comid != 6U) // ??????????
				{
					eUartOrder temp_order = ERRORSITE;
					temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
					if(temp_order != ERRORSITE) // ??????????ID
					{
						uartbuff[temp_order].recepetion_flag = 0;
						memset(uartbuff[temp_order].recepetion_buff, 0, BUFF_MAX);
					}
					sspa_entry->serial_port_state = 1; // ??????
					SuspendTask(sspa_entry->serial_port_comid); // ????????????????????
					SetTextValue(3, 9, "??????"); //?????
				}
			}
			else
			{
				eUartOrder temp_order = ERRORSITE;
				temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
				if(temp_order != ERRORSITE) // ??????????ID
				{
					uartbuff[temp_order].recepetion_flag = 0;
					memset(uartbuff[temp_order].recepetion_buff, 0, BUFF_MAX);
				}
				sspa_entry->serial_port_state = 0; // ??????
				ResumeTask(sspa_entry->serial_port_comid); // ??????? 
				SetTextValue(3, 9, "??????"); //?????
			}
			break;
		}
		case 14: { // ???????
			if(sspa_entry->serial_port_state == 1) // ????????????
			{
				if(sspa_entry->serial_port_send_len != 0) // ?????????????????????????0
				{
					temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
					if(temp_uart != NULL)
					{
						if(sspa_entry->serial_port_send_new_row == 1) // ????????
						{
							uint8_t temp_send_buff[256];
							uint8_t temp_buff_len = sspa_entry->serial_port_send_len;
							
							memcpy(temp_send_buff, sspa_entry->serial_port_send_buff, temp_buff_len);
							
							temp_send_buff[temp_buff_len++] = '\r';
							temp_send_buff[temp_buff_len++] = '\n';
							
							HAL_UART_Transmit(temp_uart, temp_send_buff, temp_buff_len, 0xff); // ????????
						}
						else
						{
							HAL_UART_Transmit(temp_uart, sspa_entry->serial_port_send_buff, sspa_entry->serial_port_send_len, 0xff); // ????????
						}
						
					}
				}
			}
			
			break;
		}
		case 30: { // ?????????e???
			if(sspa_entry->serial_port_state == 1) // ????????????
			{
				temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
				if(temp_uart != NULL)
				{
					char cxpz_buff[] = "##,CXPZ,$$\r\n";
					HAL_UART_Transmit(temp_uart, (uint8_t *)cxpz_buff, strlen(cxpz_buff), 0xff); // ????????
				}
			}
			break;
		}
		case 31: { // ??????????
			if(sspa_entry->serial_port_state == 1) // ????????????
			{
				temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
				if(temp_uart != NULL)
				{
					char cxpz_buff[] = "##,RESETALL,$$\r\n";
					HAL_UART_Transmit(temp_uart, (uint8_t *)cxpz_buff, strlen(cxpz_buff), 0xff); // ????????
				}
			}
			break;
		}
		case 29: {
			eUartOrder temp_order = ERRORSITE;
			temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
			if(temp_order != ERRORSITE) // ??????????ID
			{
				uartbuff[temp_order].recepetion_flag = 0;
				memset(uartbuff[temp_order].recepetion_buff, 0, BUFF_MAX);
			}
			
			sspa_entry->serial_port_state = 0; // ??????
			ResumeTask(sspa_entry->serial_port_comid); // ??????? 
			SetTextValue(3, 9, "??????"); //?????
			break;
		}
		case 35: {
			for(uint8_t i = 15; i < 28; i++)
			{
				clearTextValue(3 , i); //(????ID,???ID??
			}
			sspa_entry->serial_port_show_offset = 0;
			break;
		}
		case 33: { // ?????????
			sspa_entry->serial_port_send_mode = !sspa_entry->serial_port_send_mode;
			if(sspa_entry->serial_port_send_mode == 0)
			{
				SetTextValue(3, 32, "?????????"); //?????
			}
			else
			{
				SetTextValue(3, 32, "16???????"); //?????
			}
			
			
			break;
		}
		case 37: {
			sspa_entry->serial_port_show_mode = !sspa_entry->serial_port_show_mode;
			if(sspa_entry->serial_port_show_mode == 0)
			{
				SetTextValue(3, 36, "?????????"); //?????
			}
			else
			{
				SetTextValue(3, 36, "16???????"); //?????
			}
			break;
		}
		case 39: {
			sspa_entry->serial_port_send_new_row = !sspa_entry->serial_port_send_new_row;
			
			if(sspa_entry->serial_port_send_new_row == 0)
			{
				SetTextValue(3, 38, "??????????"); //?????
			}
			else
			{
				SetTextValue(3, 38, "????????"); //?????
			}
			
			break;
		}
	}
	
}


static void SimulationSerialPortMenuCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t ctrl_id, uint8_t item, uint8_t state)
{
    if (ctrl_id != 8 || state != 1) {
        return;  // ????????????????
    }

    uint8_t new_com_id = item + 1;
    uint8_t current_com_id = sspa_entry->serial_port_comid;

    /* XR5000_UART5_EXCLUSIVE_FIX_20260730: serial assistant cannot own loop-3 UART5. */
    if (new_com_id == 6U) {
        return;
    }
    
    // ????????????????????????¦Ê¦Â???
    if (new_com_id == current_com_id) {
        return;
    }
    
    // ??§Ö?????????????????????????/???????
    if (sspa_entry->serial_port_state == 1 && current_com_id != 0) {
        // ?????????????????
        ResumeTask(current_com_id);
        
        // ?????????????????
        SuspendTask(new_com_id);
    }
    
    // ???????????ID
    sspa_entry->serial_port_comid = new_com_id;
}

static void SimulationSerialPortTextCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t control_id, uint8_t *str)
{
	UART_HandleTypeDef *temp_uart = NULL;
	
	if(control_id == 4) // ?????????????
	{
		int slave_addr;
		sscanf((const char *)str, "%d", &slave_addr);
		
		if(sspa_entry->serial_port_state == 1) // ????????????
		{
			temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
			if(temp_uart != NULL)
			{
				uint8_t buff_len;
				uint8_t cxpz_buff[32];
				buff_len = sprintf((char *)cxpz_buff, "##,ADR=%d,$$\r\n", slave_addr);
				HAL_UART_Transmit(temp_uart, cxpz_buff, buff_len, 0xff); // ????????
			}
		}
		
	}
	else if(control_id == 28)
	{
		if(sspa_entry->serial_port_send_mode == 1)
		{
			uint8_t *temp_str = str;
			uint8_t buff_point = 0;
			uint8_t str_point = 0;
			uint8_t high_nibble = 0;
			uint8_t got_high_nibble = 0;

			while(temp_str[str_point] != '\0' && buff_point < 255)
			{
					uint8_t c = temp_str[str_point];
					uint8_t value;
					
					if(c == ' ') {
							str_point++;
							continue;
					}
					
					// ?????????????????????
					if(c >= '0' && c <= '9') value = c - '0';
					else if(c >= 'A' && c <= 'F') value = c - 'A' + 10;
					else if(c >= 'a' && c <= 'f') value = c - 'a' + 10;
					else {
							str_point++; // ??????§¹???
							continue;
					}
					
					if(!got_high_nibble) {
							high_nibble = value << 4; // ??4¦Ë
							got_high_nibble = 1;
					} else {
							sspa_entry->serial_port_send_buff[buff_point++] = high_nibble | value;
							got_high_nibble = 0;
					}
					
					str_point++;
			}
			sspa_entry->serial_port_send_len = buff_point;
		}
		else
		{
			sspa_entry->serial_port_send_len = strlen((const char *)str);
			memcpy(sspa_entry->serial_port_send_buff, str, sspa_entry->serial_port_send_len);
		}
	}
}

void HexToHexStringLight(uint8_t *data, uint8_t len, uint8_t *output)
{
    const uint8_t hex_chars[] = "0123456789ABCDEF";
    uint8_t pos = 0;
    
    for(uint8_t i = 0; i < len; i++) {
        // ??4¦Ë
        output[pos++] = hex_chars[(data[i] >> 4) & 0x0F];
        // ??4¦Ë
        output[pos++] = hex_chars[data[i] & 0x0F];
        
        // ????????????????????2?????????
        if(i < len - 1) {
            output[pos++] = ' ';
        }
    }
    output[pos] = '\0';
}

static void SimulationSerialPortScreenShowApp(SimulationSerialPortAssistant_t *sspa_entry)
{
	eUartOrder temp_order = ERRORSITE;
	
	if(sspa_entry->serial_port_state == 1) // ????????????
	{
		temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
		if(temp_order != ERRORSITE)
		{
			if(uartbuff[temp_order].recepetion_flag == 1) // ??????????
			{
				uartbuff[temp_order].recepetion_flag = 0; // ??????
				
				if(sspa_entry->serial_port_show_mode == 1) // 16???????
				{
					uint8_t serial_port_receive_buff[256]; // ????????? ????????????16????
					HexToHexStringLight(uartbuff[temp_order].recepetion_buff, uartbuff[temp_order].recepetion_len, serial_port_receive_buff);
					
					SetTextValue(3, 15 + sspa_entry->serial_port_show_offset, serial_port_receive_buff); //?????
					sspa_entry->serial_port_show_offset++;
					sspa_entry->serial_port_show_offset %= 13;
				}
				else
				{
					const char *delimiter = "\r\n";
					char *token = strtok((char *)uartbuff[temp_order].recepetion_buff, delimiter);
					
					while (token != NULL) {
							SetTextValue(3, 15 + sspa_entry->serial_port_show_offset, (uint8_t *)token); //?????
							token = strtok(NULL, delimiter);
							sspa_entry->serial_port_show_offset++;
							sspa_entry->serial_port_show_offset %= 13;
					}
					
				}
			}
		}
	}
}

const uint8_t warn_alarm_fresh_id[4] = {28, 28, 28, 137};
const uint8_t fire_alarm_fresh_id[4] = {29, 29, 29, 138};
const uint8_t screen_top_fresh_id[4] = {1, 59, 60, 61};

static void FirstAlarmInformationShowCtrl(
	uint16_t temp_screen_id,
	AlarmPinToTopCtrl_t *sicj_entry, 
	PackCabinForeWarnStorage *pcfws_entry, 
	PackCabinFireAlarmStorage *pcfas_entry
)
{
	if(pcfws_entry->self_bottom_point != 0 && sicj_entry->warn_fresh_flag == 0)
	{
		uint8_t temp_buff[64];
		
		sicj_entry->warn_fresh_flag = 1; // ???????????????
		// ??????????????????
		if(pcfws_entry->detector_class[0] == PackClassID && pcfws_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first warning display uses "??3??¡¤ X??".
			FormatRS485DetectForeWarnLine(temp_buff, 1, pcfws_entry, 0);
		}
		else if(pcfws_entry->detector_class[0] == PackClassID)
		{
			if(pcfws_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cluster_id, pcfws.da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
		}
		else if(pcfws_entry->detector_class[0] == LinkageClassID) // ??????????õô
		{
			if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??????????", 1,
					pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
					pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second);
			}
		}
		else if(pcfws_entry->detector_class[0] == CabinClassID &&
			(pcfws_entry->alarm_type[0] == Loop1TempWarning || pcfws_entry->alarm_type[0] == Loop1SmokeWarning))
		{
			FormatLoop1WarningLine(temp_buff, 1U, pcfws_entry, 0U);
		}
		else if(pcfws_entry->detector_class[0] == CabinClassID)
		{
			if(pcfws_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
		}
		
		for(uint8_t i = 0; i < 4; i++)
		{
			SetTextValue(screen_top_fresh_id[i], warn_alarm_fresh_id[i], temp_buff); // ??¦Ì????????????
		}
	
	}
	else if(pcfws_entry->self_bottom_point == 0 && sicj_entry->warn_fresh_flag != 0)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			clearTextValue(screen_top_fresh_id[i], warn_alarm_fresh_id[i]); // ??¦Ì????????????
		}
		sicj_entry->warn_fresh_flag = 0;
	}
	
	if(pcfas_entry->self_bottom_point != 0 && sicj_entry->fire_fresh_flag == 0)
	{
		uint8_t temp_buff[64];
		
		sicj_entry->fire_fresh_flag = 1;
		// ??????????????????
		if(pcfas_entry->detector_class[0] == PackClassID && pcfas_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first fire display uses "??3??¡¤ X??".
			FormatRS485DetectFireAlarmLine(temp_buff, 1, pcfas_entry, 0);
		}
		else if(pcfas_entry->detector_class[0] == PackClassID)
		{
			if(pcfas_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK????????????", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??%d??%d??PACK???????????????", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
		}
		else if(pcfas_entry->detector_class[0] == LinkageClassID)
		{
			if(pcfas_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??????????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second);
			}
			else
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??????¡À???", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second);
			}
			
		}
		else if(pcfas_entry->detector_class[0] == CabinClassID &&
			(pcfas_entry->alarm_type[0] == Temperature || pcfas_entry->alarm_type[0] == Smoke))
		{
			FormatLoop1FireLine(temp_buff, 1U, pcfas_entry, 0U);
		}
		else if(pcfas_entry->detector_class[0] == CabinClassID)
		{
			if(pcfas_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ???????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfws.atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ??????????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d ??1??¡¤ %d?? ????????", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
		}
		for(uint8_t i = 0; i < 4; i++)
		{
			SetTextValue(screen_top_fresh_id[i], fire_alarm_fresh_id[i], temp_buff); // ??¦Ì????????????
		}
	}
	else if(pcfas_entry->self_bottom_point == 0 && sicj_entry->fire_fresh_flag != 0)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			clearTextValue(screen_top_fresh_id[i], fire_alarm_fresh_id[i]); // ??¦Ì????????????
		}
		sicj_entry->fire_fresh_flag = 0;
	}
}

static void LicenseVerificationCtrl(void)
{
//	int8_t compare_value_1 = strncmp();
//	int8_t compare_value_2 = strncmp();
//	int8_t compare_value_3 = strncmp();
//	
//	SystemSaveInfo.curr_license_store[0] = '\0';
//	SystemSaveInfo.last_license_store[0] = '\0'; // ???›¥???????
//	SystemSaveInfo.pref_license_store[0] = '\0'; // 
}
