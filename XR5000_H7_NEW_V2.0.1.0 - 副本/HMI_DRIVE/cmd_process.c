#include "cmd_process.h"
#include "hmi_driver.h"
#include "24c04.h"
#include "w25qxx.h"
#include "system.h"
#include "bsp_relay.h"

#include "bsp_adc.h"

#include "bsp_logic_set.h"
#include "bsp_logic_screen.h" /* 联动逻辑：屏幕编辑/查看界面处理 */

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
#include "bsp_aht20.h"

#include "bsp_key.h"

#include "bsp_ctrl_bus.h"
#include "bsp_mbus.h"
#include "bsp_fdcan1.h"
#include "bsp_can_monitor.h"

#include "bsp_password.h"

#include "bsp_storage_event.h"  /* 黑匣子存储事件接入层 */
#include "bsp_fecbus_report.h" /* FECbus RS485 上报接入层 (GB4717 附录C) */

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
uint8  cmd_buffer[CMD_MAX_SIZE];                                    //指令缓存
uint16 current_screen_id = 0;                                       //当前画面ID
uint32_t yonghumima=0,yonghumm1=0,yonghumm2=0,yonghumm3=0,mmsdSTA=0;//用户密码
uint32_t chaojimima=68686668;//超级密码

uint8_t mimajiyi=0;
uint8_t miehuoqidong=0;

// new
typedef struct
{
	uint16_t years;    // 报警年
	
	uint8_t  months;   // 报警月
	uint8_t  days;     // 报警秒
	
	uint8_t  hours;    // 报警时
	uint8_t  minute;   // 报警分
	
	uint8_t  second;   // 报警秒
}AlarmTimeRecord;

typedef enum
{
	PackClassID  = 1,
	CabinClassID = 2, 
	LinkageClassID = 3,
}PackCabinClassID; // 类别标识号

typedef enum
{
	Temperature = 1,  // 温度
	Smoke       = 2,  // 烟雾
	HandAlarm   = 3,  // 手报
	Hydrogen    = 4,  // 氢气
	Carbon      = 5,  // 一氧化碳
	
	// XR5000_LOOP3_CHANGE_20260727: VOC and CH4 use separate forewarn record types.
	Voc         = 9,
	Methane     = 10,
	Loop3TempWarning = 11,
	Loop3CarbonFire = 12,
	Loop3HydrogenFire = 13,
	Loop1TempWarning = 14,
	Loop1SmokeWarning = 15,
	AlarmCtrlKey = 6, // 报警器
	
	FeedBack1Press = 7,
	
	SysFlashFault = 8,

}DetectorAlarmType; //探测器报警类型

typedef enum
{
	OUT_FIRE_NO_START  = 1,  // 灭火装置未启动
	OUT_FIRE_SUSPEND   = 2,  // 灭火装置停止启动

	SPRAY_START_DELAY  = 3,  // 喷放倒计时
	SPRAY_INTERVAL_T1  = 4,  // 间隔 等待第2次喷放
	SPRAY_SECOND_DELAY = 5,  // 第2次喷放倒计时状态
	SPRAY_INTERVAL_T2  = 6,  // 间隔 等待第3次喷放
	SPRAY_THIRD_DELAY  = 7,  // 第3次喷放倒计时状态
}OutFireDeviceState; // 灭火装置状态


typedef struct
{
	uint8_t cluster_id; // 簇id
	uint8_t pack_id;    // 挂在对应簇下的pack id
	
	uint8_t cabin_id;   // 舱id

	int8_t lunch_state; // 改为有符号型 用来记录终止情况 -1:终止 -2:重启后终止 0:未启动 99:启动倒计时期间终止

	AlarmTimeRecord atr;

}PackAlarmStorage;    // 现在该结构体用来做多个报警循环显示

PackAlarmStorage pas[224] = { 0 };

uint8_t pas_pointer = 0;
uint8_t last_pas_len = 0;
uint8_t pas_fresh_point = 0;// 屏幕显示刷新指针

uint8_t multiple_alarm_fresh_flag = 0;
uint8_t pas_traverse_pointer = 1;

uint8_t alarm_number = 0; // 报警总数
uint8_t last_alarm_num = 255; // 默认执行一次

uint8_t last_online_detector_num = 255;
uint8_t last_disconnect_detector_num = 255;
uint8_t home_statistics_force_refresh = 1;

typedef struct
{
	uint8_t cabin_id;       // 仓ID
	
	uint8_t cluster_id;     // 簇ID
	uint8_t pack_id;        // 包ID
}DetectorAddrAttribute;   // 原始数据类型，区分仓，包，簇

typedef struct
{
	uint8_t temperature_type; // 温度
	uint8_t carbon_type;      // 一氧化碳
	uint8_t smoke_type;       // 烟雾
}CabinAlarmType;     //仓探测器的保健那个类型

typedef struct
{
	uint8_t temperature_type; // 温度
	uint8_t carbon_type;      // 一氧化碳
	uint8_t smoke_type;       // 烟雾
}PackAlarmType;       ///包报警器的保健那个类型

uint8_t fore_alarm_start_index = 0;
uint8_t fire_alarm_start_index = 0; // 报警信息滚动数据的起始索引
uint8_t fire_alarm_check_new_flag = 0;
uint8_t force_alarm_check_new_flag = 0;
#define getFireAlarmCheckNewKey() fire_alarm_check_new_flag
#define getForceAlarmCheckNewKey() force_alarm_check_new_flag

/* 预警 火警分开存储
 * 2025/07/08日更新
 * 检测中心要求火警预警分开存储
 */
// 预警存储 此预警指 所有可燃气体
typedef struct {
	uint8_t self_bottom_point;       // 所有数组的底指针
	
	uint8_t point_history_len;       // 记录历史指针长度

	uint8_t detector_class[224];     // 探测器类型
	
	uint8_t alarm_type[224];				 // 包报警类型
	
	DetectorAddrAttribute da[224];   // 探测器属性

	AlarmTimeRecord atr[224];        // 报警时间记录
	
	uint16_t fresh_time_count;        // 用来轮询显示
}PackCabinForeWarnStorage;  // 仓包报警器的预警存储

PackCabinForeWarnStorage pcfws = {
	.self_bottom_point = 0,
	.point_history_len = 255,
	.detector_class = {0},
	.da         = {0},
	.alarm_type = {0},
	.atr        = {0}
};

// 火警存储 此火警指 烟雾 温度
typedef struct {
	uint8_t self_bottom_point;       // 所有数组的底指针
	
	uint8_t point_history_len;       // 记录历史指针长度
	
	uint8_t detector_class[224];     // 探测器类型
	
	uint8_t alarm_type[224];				 // 包报警类型
	
	DetectorAddrAttribute da[224];   // 探测器属性
	
	AlarmTimeRecord atr[224];        // 报警时间记录
	
	uint16_t fresh_time_count;       // 轮询显示计时
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
	uint8_t detector_class; // 探测器类型
	
	DetectorAddrAttribute da;   // 探测器属性
	
	AlarmTimeRecord atr;        // 报警时间记录
	uint8_t fault_type;
	
}PackCabinFaultStorage;   // 用来存储掉线 故障信息	
uint8_t pcfs_fresh_ctrl = 255;    // 刷新控制指针 初始赋值为最大值
uint8_t pcfs_buttom_point = 0;  // 尾指针 记录数据长度
PackCabinFaultStorage pcfs[224] = {0}; // 储存所有故障信息

uint8_t fault_check_new_flag = 0;
uint8_t fault_current_page = 0;
#define getFaultCheckNewKey() fault_check_new_flag
// end

// new
// 用来筛选出报火警的编号
typedef struct
{
	uint8_t cabin_alarm_state  ; 
	uint8_t cluster_alarm_state;

}FireAlarmStorage;
FireAlarmStorage fire_alarm_flag = {0}; // 全部初始化为0

typedef struct
{
	uint8_t fire_alarm_id_buff[300]; // 存储报警编号
	uint8_t faib_buttom_point;      // fire_alarm_id_buff长度指针
	uint8_t storage_pas_len;        // 存储火警记录长度 长度不一致时更新数据
}FireAlarmNumRecord; // 把火警报警的编号筛选出来
FireAlarmNumRecord fanr = {0};

/*********
* 灭火装置启动流程/状态
*  1 启动延时 延时xx秒
*  2 启动延时结束
*  3 第一次喷放开始 持续xx秒
*  4 第一次喷放持续时间结束
*  5 第二次喷放启动延时 延时xx秒
*  6 第二次喷放启动延时结束
*  7 第二次喷放开始 持续xx秒
*  8 第二次喷放持续时间结束
*  9 第三次喷放启动延时 延时xx秒
* 10 第三次喷放启动延时结束
* 11 第三次喷放开始 持续xx秒
* 12 喷放完成
* 13 若在启动延时期间强制结束 状态为-1
* 14 若再次触发状态为-2 并从启动延时重新开始
*/
typedef enum
{
	FIRE_EXTINGUISH_FORCE_START    = -4,    // 手动强制启动
	FIRE_EXTINGUISH_RESTART_FINISH = -3,    // 重新启动后赋值为该状态 避免下次启动
	FIRE_EXTINGUISH_CAN_RESTART    = -2,    // 可以重新启动的状态
	FIRE_EXTINGUISH_FORCE_STOP     = -1,    // 强制结束
	
	FIRE_EXTINGUISH_MODE_JUDGEMENT       = 0,      // 判断当前状态
	FIRE_EXTINGUISH_START_SPRAY_DELAY    = 1,      // 启动延时
	FED_START_SPRAY_DELAY_FINISH_FLAG    = 2,      // 灭火装置启动倒计时 结束标志
	FIRE_EXTINGUISH_FIRST_SPRAY_START    = 3,      // 第一次喷放开始
	FIRE_EXTINGUISH_FIRST_SPRAY_FINISH   = 4,      // 第一次喷放完成
	FIRE_EXTINGUISH_SECOND_SPRAY_DELAY   = 5,      // 第二次喷放启动延时
	FED_SECOND_SPRAY_DELAY_FINISH_FLAG   = 6,      // 灭火装置第二次启动倒计时 结束标志
	FIRE_EXTINGUISH_SECOND_SPRAY_START   = 7,      // 第二次喷放开始
	FIRE_EXTINGUISH_SECOND_SPRAY_FINISH  = 8,      // 第二次喷放完成
	FIRE_EXTINGUISH_THIRD_SPRAY_DELAY    = 9,      // 第三次喷放启动延时
	FED_THIRD_SPRAY_DELAY_FINISH_FLAG    = 10,     // 第三次喷放倒计时结束标志
	FIRE_EXTINGUISH_THIRD_SPRAY_START    = 11,     // 第三次喷放开始
	FIRE_EXTINGUISH_THIRD_SPRAY_FINISH   = 12,     // 第三次喷放完成
	FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE   = 13,     // 全部喷放完成

	FIRE_EXTINGUISH_CLUSTER_VALVE_OPEN   = 14, // 簇电磁阀打开
	FIRE_EXTINGUISH_CABIN_VALVE_OPEN     = 15, // 仓电磁阀打开
	FIRE_EXTINGUISH_CYLINDEF_1_OPENED    = 16, // 钢瓶1电磁阀打开
	FIRE_EXTINGUISH_CYLINDEF_2_OPENED    = 17, // 钢瓶2电磁阀打开
	
	FIRE_EXTINGUISH_STARYUP_FINISH_FLAG  = 18, // 灭火装置启动进入倒计时
	
	FEEDBACK_1_PRESS = 19,
	FEEDBACK_2_PRESS = 20,
	
}FireExtinguishDeviceActionType; // 灭火装置动作类型

// 气灭装置记录
typedef struct
{
	uint8_t cluster_id[224];   // 簇id
	uint8_t pack_id[224];      // 挂在对应簇下的pack id
	
	uint8_t cabin_id[224];     // 舱id
	
	AlarmTimeRecord atr[224];  // 动作时间记录
	
	int8_t fed_action[224];   // 灭火装置动作记录
	
	uint8_t self_point_len;    // 
	uint8_t last_point_len;    // 遍历指针
	
	uint16_t countdown_val[224];    // 倒计时的值 
	uint16_t start_cntd_time[224];  // 开始倒计时的时间戳
	uint16_t curr_cntd_time[224];   // 当前系统时钟 用来显示倒计时
}FireExtinguishDeviceActionSave; // 灭火装置动作记录

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

// new 包数据显示 控制变量
typedef struct
{
	uint8_t  curr_detector_page; // 当前显示页
	uint8_t  last_detector_page; // 上一次显示页
	
	uint8_t  detector_offline_fresh_flag[11];
	
	uint8_t  detect_online_state[21][11];   // 在线状态
	uint8_t  detect_shield_state[21][11];   // 屏蔽状态
	
	uint8_t  last_temperat_state[21][11]; // 上一次温度状态
	uint8_t  last_temperature[21][11];   // 上一次的温度
	
	uint8_t  last_smoke_state[21][11];   // 上一次烟雾状态
	
	uint8_t  last_co_state[21][11];      // 
	uint16_t last_co_concentrat[21][11]; // 上一次可燃气体的浓度

}DetectorDataShowCtrl;

DetectorDataShowCtrl ddsc;

typedef struct
{
	uint8_t force_fresh_flag;    // 强制刷新标志位
	
	uint8_t curr_detector_page;  // 当前页
	uint8_t last_detector_page;  // 之前页
	
	uint8_t curr_pack_id;
	
	uint8_t last_temperat_state; // 上一次温度状态
	uint8_t last_temperature;    // 上一次的温度
	
	uint8_t last_smoke_state;   // 上一次烟雾状态
	
	uint8_t last_co_state;      // 
	
	uint8_t lat_detector_online_num[4]; // 历史上线数量 
	
	uint8_t last_derector_state[3][32];
	
	uint16_t last_co_concentrat; // 上一次可燃气体的浓度
}DetectorDataShowCtrl_32Pack;

DetectorDataShowCtrl_32Pack ddsc_32p;

typedef struct
{
	uint8_t force_fresh_flag;
	
	uint8_t curr_cabin_id;
	
	uint8_t last_temperat_state; // 上一次温度状态
	uint8_t last_temperat_value; // 上一次的温度
	
	uint8_t last_smoke_state;   // 上一次烟雾状态
	
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
	OutMenu = 0, // 菜单外 在该状态下切换分区
	InMenu = 1,  // 菜单内 在该状态下才可翻页
	
	InitMenu = 0xFF,
}BspMenuState;

#define POINT_SITE_MAX 3U

// 按键控制 屏幕硬件查新按键
typedef struct
{
	uint8_t curr_partition; // 当前是哪个分区
	uint8_t last_partition; // 更新抑制
	uint8_t curr_point_site[4]; // 当前箭头在哪一栏
	uint8_t last_point_site[4]; // 更新抑制
	
	uint8_t last_show_len[4]; // 存储上一次刷新的位置 用来在可恢复的地方刷新箭头位置
	
	uint8_t curr_menu_state; // 当前菜单状态
	uint8_t last_menu_state; // 上一次菜单状态
}BspKeyCheckNewCtrl_t;

BspKeyCheckNewCtrl_t bkcnc = {
	.curr_partition = 0, 
	.last_partition = 255, 
	.curr_point_site = {0, 0, 0, 0}, 
	.last_point_site = {255, 255, 255, 255},
	.curr_menu_state = InitMenu, 
};

// 有故障 预警 火警 回主界面控制结构体
typedef struct 
{
	// 指针用来绑定地址 变量用来储存值 抑制更新
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
// 用指针绑定地址 只初始化一次即可不用重复绑定
SwitchInterfaceCtrl switch_ui_ctrl;

// 新增加的变量
extern uint8_t screen_show_siren_information;
extern uint8_t shielding_state;
extern uint8_t self_check_state;

// 分区1 状态控制变量
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

}eGeneralIoBeepBit; // 通用IO蜂鸣器位枚举



// [7:4] 火警标志 [3:0] 预警标志
uint8_t beep_fire_ctrl = 0;
uint8_t beep_fault_ctrl = 0;

uint8_t beep_spray_feedback_ctrl = 0;

uint32_t beep_general_io_ctrl = 0;
// end

uint8_t zhu_state=1,bei_state=1;


// 报警查询 故障查询控制界面
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
	uint8_t curr_page[4]; // 当前页 
	
	uint16_t record_sum[4]; // 记录的数量
	
	uint8_t force_fresh_flag; // 强制刷新标志
	
	uint8_t curr_show_type; // 当前显示类型
}BspScreenReadRecord_t;

BspScreenReadRecord_t bsrr = {
	.curr_page = {0x01, 0x01, 0x01, 0x01},
	.force_fresh_flag = 0,
	.curr_show_type = RECORD_INIT,
};
// 读取缓冲区
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

uint8_t PCAC_zxwz_buf[12]={0,4,16,28,40,52,64,76,88,100,112,0};//在线状态控件ID
uint8_t PCAC_wdwz_buf[12]={0,9,21,33,45,57,69,81,93,105,117,0};//温度控件ID
uint8_t PCAC_ywtb_buf[12]={0,12,24,36,48,60,72,84,96,108,120,0};//烟雾状态控件ID
uint8_t PCAC_cotb_buf[12]={0,11,23,35,47,59,71,83,95,107,119,0};//CO状态控件ID
uint8_t PCAC_ch4yb_buf[12]={0,13,25,37,49,61,73,85,97,109,121,0};//CH4状态控件ID

uint8_t cang_zxwz_buf[8]={0,16,20,36,52,72,88,0};//在线状态控件ID

uint8_t cang_wdwz_buf[8]={0,10,17,41,49,77,111,0};//温度控件ID
//uint8_t cang_ywtb_buf[8]={0,12,19,35,51,71,87,0};//烟雾状态控件ID
//uint8_t cang_cotb_buf[8]={0,13,25,41,61,77,93,0};//CO状态控件ID
//uint8_t cang_ch4yb_buf[8]={0,14,26,42,62,78,94,0};//CH4状态控件ID
//uint8_t cang_vocyb_buf[8]={0,15,27,43,63,79,95,0};//VOC状态控件ID

uint8_t cang_cowb_buf[8]={0,117,25,122,61,89,138,0};//CO状态控件ID
uint8_t cang_h2wb_buf[8]={0,118,26,124,62,90,139,0};//H2状态控件ID
uint8_t cang_ch4wb_buf[8]={0,12,33,127,69,93,142,0};//CH4状态控件ID
uint8_t cang_ywwb_buf[8]={0,120,35,129,71,95,144,0};//YW状态控件ID
uint8_t cang_vocwb_buf[8]={0,15,34,128,70,94,143,0};//VOC状态控件ID

uint8_t cang_XH_buf[8]={0,149,150,151,152,153,154,0};//型号控件ID


uint8_t kaijiyanshi=0;

uint8_t DX_cangjiyibuf[30];

uint8_t BJ_cangjiyibuf_wd[30];
uint8_t BJ_cangjiyibuf_yw[30];
uint8_t BJ_cangjiyibuf_co[30];
uint8_t BJ_cangjiyibuf_ch4[30];
uint8_t BJ_cangjiyibuf_voc[30];
uint8_t BJ_cangjiyibuf_h2[30];

uint8_t BJ_packjiyibuf_wd[30][PACK_NUM_BACKUP];
uint8_t BJ_packjiyibuf_yw[30][PACK_NUM_BACKUP];//2024-03-09增加，防止温度烟雾预警并行时候重复记录
uint8_t BJ_packjiyibuf_co[30][PACK_NUM_BACKUP];//2024-03-09增加，防止温度烟雾预警并行时候重复记录
uint8_t BJ_packjiyibuf_ch4[30][PACK_NUM_BACKUP];//2024-03-09增加，防止温度烟雾预警并行时候重复记录
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

// 簇阀开启状态
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

// 2025/11/17 18:10 新增 循环显示每一仓探测器的状态 可主动查询 可上下翻页 后续不叫仓 统一改为 xx回路xxID探测器

typedef struct
{
	uint8_t poll_circuits_id; // 回路ID
	uint8_t poll_detector_id; // 探测器ID
	
	// 此处不是为了抑制更新 而是为了只显示上线的探测器 用来筛选ID
	uint8_t last_circuits_id; // 上次回路ID
	uint8_t last_detector_id; // 上次探测器ID
	
	uint8_t poll_temper_value; // 轮询温度值
	uint8_t poll_smokes_state; // 轮询烟雾状态
	
	uint16_t poll_carbon_value; // 轮询一氧化碳值
	
	uint16_t poll_hydrog_value; // 轮询氢气值 省略en
	
	uint8_t poll_detect_name;  // 轮询 探测器名称
	
	uint8_t poll_sensor_state; // 探测器启用状态
	
	uint8_t key_perss_fresh; // 'n'下一个 'p'上一个
}PollingShowBase_t; // 轮询显示基结构体

typedef struct
{
	// 基础ID控制
	uint8_t verb_circuits_id; // 回路ID
	uint8_t verb_detector_id; // 探测器ID
	uint8_t last_detector_id; // 上一次查询的探测器ID 用来抑制更新
	
	uint8_t force_fresh_ctrl; // 输入ID后强制刷新一次
	
	uint8_t verb_temper_value; // 温度值
	uint8_t last_temper_value; // 温度值
	
	uint8_t verb_smokes_state; // 烟雾状态
	uint8_t last_smokes_state; // 烟雾状态
	
	uint16_t verb_carbon_value; // 一氧化碳值
	uint16_t last_carbon_value; // 一氧化碳值
	
	uint16_t verb_hydrog_value; // 轮询氢气值 省略en
	uint16_t lsat_hydrog_value; // 轮询氢气值 省略en
	
	uint8_t verb_detect_name;  // 轮询 探测器名称
	uint8_t lsat_detect_name;  // 轮询 探测器名称
	
	uint8_t verb_sensor_state; // 探测器启用状态
	uint8_t last_sensor_state; // 探测器启用状态
	
}InqueryShowBase_t; // 主动查询基结构体

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

// 点型显示
static void PointTypeDetectorShowApp(PointTypeShowCtrl_t *ptsc_entry);
static void PointTypeDetectorButtonCtrlApp(PointTypeShowCtrl_t *ptsc_entry, uint16 control_id, uint8  state);
static void PointTypeDetectorTextInputCtrlApp(PointTypeShowCtrl_t *ptsc_entry, uint16 control_id, uint8 *str);
static void PointTypeDetectorScreenSwitchShowApp(PointTypeShowCtrl_t *ptsc_entry);


CompositeShowCtrl_t cpsc = {0};
// 复合显示
static void CompositeDetectorShowApp(CompositeShowCtrl_t *cpsc_entry);
static void CompositeDetectorButtonCtrlApp(CompositeShowCtrl_t *cpsc_entry, uint16 control_id, uint8  state);
static void CompositeDetectorTextInputCtrlApp(CompositeShowCtrl_t *cpsc_entry, uint16 control_id, uint8 *str);
static void CompositeDetectorScreenSwitchShowApp(CompositeShowCtrl_t *cpsc_entry);

// 新增记录IO板的所有故障信息 2025/11/21 13:49

// 新增模拟串口助手界面控制
typedef struct
{
	uint8_t serial_port_state; // 串口打开状态
	
	uint8_t serial_port_comid; // 用来选择端口ID 选择挂起哪个任务来空闲串口
	
	uint8_t serial_port_send_mode; // 0:字符串 1:16进制
	uint8_t serial_port_show_offset;
	
	uint8_t serial_port_show_mode; // 0:字符串 1:16进制    
	
	uint8_t serial_port_send_len; // 从发送缓冲区中获取的字符长度
	
	uint8_t serial_port_send_new_row; // 0:不发送新行 1:发送新行
	
	uint8_t serial_port_send_buff[256]; // 发送缓冲区

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

// 清空整个故障存储数组
void FaultDataInit(PackCabinFaultStorage *pcfs_entry)
{
	memset(pcfs_entry, 0, sizeof(PackCabinFaultStorage) * 224);
	pcfs_fresh_ctrl = 255;    // 刷新控制指针 初始赋值为最大值
	pcfs_buttom_point = 0;  // 尾指针 记录数据长度
}

// 清空整个预警数组
void ForeAlarmDataInit(PackCabinForeWarnStorage *pcfws_entry)
{
	memset(pcfws_entry, 0, sizeof(PackCabinForeWarnStorage));
	pcfws_entry->point_history_len = 255;
}

// 清空整个火警数组
void FireAlarmDataInit(PackCabinFireAlarmStorage *pcfas_entry)
{
	memset(pcfas_entry, 0, sizeof(PackCabinFireAlarmStorage));
	pcfas_entry->point_history_len = 255;
}

// 清空整个气灭存储数组
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
	pas_fresh_point = 0;// 屏幕显示刷新指针
}

static void PointTypeDetectorAllStateInit(void);

static uint8_t getPointDetectorSetUpCount(void);
static uint8_t getPointDetectorSetUpLive(void);
static uint8_t getPointDetectorFaultCount(void);
static uint8_t getPointDetectorAlarmCount(void);

void ClearDetectorHistoryData(void)
{
	// 清除包的历史数据
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
	
	// 清除仓的历史数据
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
	
	// 清除历史记录
	memset(BJ_cangjiyibuf_wd, 0, 30);
	memset(BJ_cangjiyibuf_yw, 0, 30);
	memset(BJ_cangjiyibuf_co, 0, 30);
	memset(BJ_cangjiyibuf_ch4, 0, 30);
	memset(BJ_cangjiyibuf_voc, 0, 30);
	memset(BJ_cangjiyibuf_h2, 0, 30);
	
	// 清空簇阀开启状态
	cluster_solenoid_valve_start_state = 0;
	
	// 恢复点型探测器状态
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
}; // 初始化设备总数为0

void ScreenFreshInhibitionInit(void)
{
	alarm_number = 0; // 初始化报警总数
	last_alarm_num = 255; // 初始化报警总数更新抑制
	last_online_detector_num = 255; // 初始化在线探测器数量更新抑制
	last_disconnect_detector_num = 255;	 // 初始化掉线探测器数量更新抑制
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
// 复位后初始化箭头位置
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
	FaultDataInit(pcfs); // 清除故障记录
	ForeAlarmDataInit(&pcfws); // 清除预警记录
	FireAlarmDataInit(&pcfas); // 清除火警记录
	FireExtinguishDataInit(&fedas); // 清除气灭分区记录
	PackAndCabinHistoryAlarmInit(pas);
	ClearDetectorHistoryData(); // 清除探测器历史记录
	ScreenFreshInhibitionInit(); // 清除上线总数
	
	// 清除手报 反馈一反馈2的历史值 确保复位后下一次可以正常启动
	clearHandPaperState();
	cleareedBack1State();
	cleareedBack2State();
	PowerStateInit(); // 电池状态初始化
	BspBeepStateClear(); // 蜂鸣器状态初始化
	BspScreenArrowSite(&bkcnc); // 屏幕箭头初始化
	StorageEvent_ResetFirstFire(); /* 黑匣子:复位首警标志,下次火警重新判定首警 */
	StorageEvent_LogReset();       /* 黑匣子:记录系统复位事件 */
	FecbusReport_Reset();         /* FECbus:广播系统复位(功能码1) */
}

uint8_t getCurrentSystemRunState(void)
{
	uint8_t temp_state = 0;
	if(pas_pointer != 0)
	{
		temp_state = 2; // 火警
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

// 预警可以清除
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

// 获取仓 簇所有探测器上线状态
static void getDetectorSetUpLiveSum(DetectorSum *ds_entry, uint8_t cabin_setup[], uint8_t cluster_setup[]);


//new
//获取点型探测器的上线总数量，2026.7.19 新增
static uint8_t getPointDetectorSetUpLive(void);
// 获取点型探测器的报警总数量
static uint8_t getPointDetectorAlarmCount(void);
// 获取点型探测器的故障总数量
static uint8_t getPointDetectorFaultCount(void);
//end


// 返回值 包掉线数量
static uint8_t ClusterPackDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
// 2025/12/10 15:51 新增
static uint8_t ClusterPackDataDeal_Plus(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);

// 返回坐标索引
uint8_t findRecoveryDevice(uint8_t cluster_id, uint8_t pack_id, uint8_t cabin_id);
// 删除索引值的数据
void deletRecoveryRecord(uint8_t recovery_index); 

static uint8_t CabinDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);

static void FaultRelayCtrlAppFun(uint8_t disconnect_num);

static void ForeWarmRelayCtrlAppFun(PackCabinForeWarnStorage *pcfws_entry);

static void FireAlarmRelayCtrlAppFun(uint8_t pas_alarm_num);
// 显示所有故障信息
static void InternalScreenShowAllFault(uint8_t fresh_page_flag);
// 显示所有预警信息 可自恢复
static void InternalScreenShowAllForceWorn(PackCabinForeWarnStorage *pcfws_entry, uint8_t fresh_page_flag);

static void InternalScreenShowAllForceWorn_Plus(PackCabinForeWarnStorage *pcfws_entry, uint8_t fresh_page_flag);
// 显示所有火警信息 不可自恢复
static void InternalScreenShowAllFireAlarm(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag);

static void InternalScreenShowAllFireAlarm_Plus(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag);
// 创建新的气灭动作记录
static void CreatNewFireExtinguishRecord(
	FireExtinguishDeviceActionSave *fedas_entry, // 默认赋值的结构体
	FireExtinguishDeviceActionSave *copy_fedas,  // 默认赋值的结构体
	uint8_t copy_dedas_offset,
	FireExtinguishDeviceActionType state, 
	uint16_t state_switch_delay             // 状态切换延时 
);


static void FireExtinguishDevice1HandStart(FireExtinguishDeviceActionSave *fedas_entry);
static void FireExtinguishDevice2HandStart(FireExtinguishDeviceActionSave *fedas_entry);

// 气灭动作状态更新
static void FireExtinguishDeviceStateUpdate(FireExtinguishDeviceActionSave *fedas_entry, PackAlarmStorage *pas_entry);
// 灭火装置分区显示控制
static void InternalScreenShowFireExtinguisher(FireExtinguishDeviceActionSave *fedas_entry, uint8_t fresh_page_flag);

static void InternalScreenShowClusterData(DetectorDataShowCtrl *ddsc_entry);
// 1簇32pack版本 PACK状态刷新
static void InternalScreenShowClusterData_32Pack(uint16_t screen_id_entry, DetectorDataShowCtrl_32Pack *ddsc_32p_entry);
// 2025/12/10 17:22 添加
static void InternalScreenShowClusterData_32Pack_Plus(uint16_t screen_id_entry, DetectorDataShowCtrl_32Pack *ddsc_32p_entry);

// 显示仓数据 2025/10/27 11:27添加
static void InternalScreenShowCabinDate(CabinDataShowCtrl_t *cabin_dsc_entry);

static void DetectorDataFreshMenuCtrl(DetectorDataShowCtrl *ddsc_entry, uint16_t ctrl_id, uint8_t item, uint8_t state);
// 1簇32pack版本 弹出菜单控制
static void DetectorDataFreshMenuCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t item, uint8_t state);
// 
static void DetectorFreshPageButtonCtrl(DetectorDataShowCtrl *ddsc_entry, uint16_t ctrl_id, uint8_t state);
// 1簇32pack版本 PACK查询翻页按键
static void DetectorFreshPageButtonCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t state);
// 1簇32pack版本 查询PACK按钮
static void DetectorMonitorButtonCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t state);
// 查询仓数据 2025/10/27 11:27添加
static void CabinFreshPageButtonCtrl(CabinDataShowCtrl_t *cabin_dsc_entry, uint16_t ctrl_id, uint8_t state);

// 显示可燃气体最高浓度
static void RefreshGasConcentrationSummary(void);

// 检查按键 并处理屏幕上查新
static void BspCheckNewKeyPressDeal(BspKeyCheckNewCtrl_t *bkcnc_entry);
// 切换到主界面 并点亮屏幕
static void InternalScreenMainInterfaceCtrl(SwitchInterfaceCtrl *sic_entry);
static void SyncMonitorSwitchSnapshot(void);
// 报警内容显示界面按钮控制
static void InternalScreenRecordShiftButtonCtrl(BspScreenReadRecord_t *bsrr_entry, uint16_t ctrl_id, uint8_t state);
// 显示报警内容到屏幕上
static void InternalScreenShowRecord(BspScreenReadRecord_t *bsrr_entry);
// 查询报警切换界面
static void RecordSwitchButtonCtrl(BspScreenReadRecord_t *bsrr_entry, uint16_t ctrl_id, uint8_t state);
// 主备电故障判断 存储
static void PowerManageCtrl(uint8_t main_power_state, uint8_t back_power_state);
// 手动强制启动选择簇
static void HandForceStartAnyCluster(FireExtinguishDeviceActionSave *fedas_entry, uint16_t ctrl_id, uint8_t state);
// 
static void BspAlarmDataSaveApp(FlashReadCtrlId addr_type, FlashSaveType save_type, uint8_t cluster_id, uint8_t pack_or_cabin, uint16_t val);
// 
static void BspFanOnlineJudgeFaultRecord(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
//
static void BspFanStartCrtlApp(uint8_t fan_sta, uint8_t early_aralm_num, uint8_t fire_alarm_num);

// 2025/11/15 11:07 添加二总线点型感温感烟探测器轮询控制函数
static uint8_t PointTypeDetectorDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
// XR5000_LOOP3_CHANGE_20260726: Loop 3 uses RS485Detect data with original alarm logic.
static uint8_t RS485DetectDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
static void RS485Loop3ClearCurrentState(uint8_t addr);
static void Loop1ClearCurrentState(uint8_t addr);
static uint8_t MBus2DataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point);
static void PointTypeDetectorOnlineButtonCtrl(uint16_t ctrl_id, uint8_t state);

//uint8_t license_allow_use_state = 0; // 默认禁用
uint8_t license_allow_use_state = 1; // 始终开启，解除锁定
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
	uint8_t last_screen_id; // 用来抑制屏幕更新
	
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

// 2025/11/26 08:50 新增任务挂起恢复
extern void SuspendTask(uint8_t task_id);
extern void ResumeTask(uint8_t task_id);
// end



//2026/7/22新增任务
static uint8_t g_screen69_page = 0;
static uint8_t g_screen69_force_redraw = 0;
static uint8_t g_screen69_transition_pending = 0;
static uint8_t screen69_circuit = 1; /* XR5000_SCREEN69_NAVIGATION_FIX_20260729: fixed circuit snapshot for one detail session. */
/* 获取指定回路的在线设备地址列表，返回在线数量 */
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

/* 格式化单个探测器的显示文本到buf */
static void FormatDetectorText(uint8_t circuit, uint8_t addr, uint8_t *buf)
{
    switch (circuit)
    {
	case 1:
	{
		uint8_t sensor_bits = getPointTypeMixtureDetectType(addr);
		if (sensor_bits & 0x20) /* 温度传感器启用 */
		{
			uint8_t val = getPointTypeMixtureReceiveData(PointTypeData_Temper, addr);
			uint8_t mem = getPointTypeMixtureDetectTempertureMemory(addr);
			sprintf((char *)buf, "第%d回路  温度探测器%d号  温度值：%d℃  %s",
					circuit, addr, val, mem ? "报警" : "正常");
		}
		else if (sensor_bits & 0x01) /* 烟雾传感器启用 */
		{
			uint8_t mem = getPointTypeMixtureDetectSmokeMemory(addr);
			sprintf((char *)buf, "第%d回路  烟雾探测器%d号  烟雾状态：%s",
					circuit, addr, mem ? "报警" : "正常");
		}
		else
		{
			sprintf((char *)buf, "第%d回路  探测器%d号  传感器未启用", circuit, addr);
		}
		break;
	}
	case 3:
	{
		uint16_t enable = RS485Detect_GetSensorEnable(addr);
		char *p = (char *)buf;

		p += sprintf(p, "第%d回路  复合探测器%d号  ", circuit, addr);

		if (enable & (1 << 5))
		{
			int16_t temp = RS485Detect_GetTemperature(addr);
			p += sprintf(p, "温度：%d℃  ", temp);
		}
		if (enable & (1 << 0))
		{
			uint8_t smoke = RS485Detect_GetSensorState(addr, RS485_SENSOR_SMOKE);
			p += sprintf(p, "烟雾：%s  ", RS485Detect_IsFaultState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "故障" : (RS485Detect_IsAlarmState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "报警" : "正常"));
		}
		if (enable & (1 << 4))
		{
			uint16_t co = RS485Detect_GetSensorValue(addr, RS485_SENSOR_CO);
			p += sprintf(p, "CO：%dppm  ", co);
		}
		if (enable & (1 << 2))
		{
			uint16_t h2 = RS485Detect_GetSensorValue(addr, RS485_SENSOR_H2);
			p += sprintf(p, "H2：%dppm  ", h2);
		}
		if (enable & (1 << 3))
		{
			uint16_t voc = RS485Detect_GetSensorValue(addr, RS485_SENSOR_VOC);
			p += sprintf(p, "VOC：%dppm  ", voc);
		}
		if (enable & (1 << 1))
		{
			uint16_t ch4 = RS485Detect_GetSensorValue(addr, RS485_SENSOR_CH4);
			p += sprintf(p, "CH4：%dppm  ", ch4);
		}
		if (enable & (1 << 6))
		{
			uint16_t pressure = RS485Detect_GetSensorValue(addr, RS485_SENSOR_PRESSURE);
			p += sprintf(p, "压力：%dhPa  ", pressure);
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
	sprintf((char *)buf, "第3回路 %d号", addr);
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
	sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 %s", sequence,
		pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
		pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
		pcfws_entry->da[data_index].pack_id, RS485DetectAlarmName(pcfws_entry->alarm_type[data_index]));
}

static void FormatRS485DetectFireAlarmLine(uint8_t *buf, uint8_t sequence, PackCabinFireAlarmStorage *pcfas_entry, uint8_t data_index)
{
	sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 %s", sequence,
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
		case RS485_LOOP3_FAULT_CO: fault_name = "CO传感器故障"; break;
		case RS485_LOOP3_FAULT_H2: fault_name = "H2传感器故障"; break;
		case RS485_LOOP3_FAULT_VOC: fault_name = "VOC传感器故障"; break;
        case RS485_LOOP3_FAULT_CH4: fault_name = "CH4传感器故障"; break;
        case RS485_LOOP3_FAULT_SMOKE_SENSOR: fault_name = "烟雾传感器故障"; break;
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
		case MBUS_CONTROL_DEV_SGBJQ:  return "声光报警器";
		case MBUS_CONTROL_DEV_XR2200: return "手动报警器";
		case MBUS_CONTROL_DEV_FIRE_DISPLAY: return "火灾显示盘";
		default: return "未知设备";
	}
}

static uint8_t FormatMBus2FaultLine(uint8_t *buf, uint8_t sequence, PackCabinFaultStorage *pcfs_entry, uint8_t data_index)
{
	if (pcfs_entry[data_index].da.cluster_id != MBUS_CONTROL_FLASH_ID)
	{
		return 0;
	}
	const char *name = GetMBusDeviceChineseName(pcfs_entry[data_index].da.pack_id);
	sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第2回路 %s掉线", sequence,
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
				sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 请手动启动灭火装置", sequence,
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
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置启动倒计时%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FED_START_SPRAY_DELAY_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第1次喷放启动", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_FIRST_SPRAY_START:
		case FIRE_EXTINGUISH_SECOND_SPRAY_START:
		case FIRE_EXTINGUISH_THIRD_SPRAY_START:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置喷放剩余时间%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FIRE_EXTINGUISH_FIRST_SPRAY_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第1次喷放完毕", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_SECOND_SPRAY_DELAY:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第2次启动倒计时%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FED_SECOND_SPRAY_DELAY_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第2次喷放启动", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_SECOND_SPRAY_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第2次喷放完毕", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_THIRD_SPRAY_DELAY:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第3次启动倒计时%d", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
			break;
		case FED_THIRD_SPRAY_DELAY_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第3次喷放启动", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_THIRD_SPRAY_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置第3次喷放完毕", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置喷放完毕", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_STARYUP_FINISH_FLAG:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置正在启动", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_CYLINDEF_1_OPENED:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置1启动", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_CYLINDEF_2_OPENED:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置2启动", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_FORCE_STOP:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置启动倒计时--", sequence,
				fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
				fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
				fedas_entry->pack_id[data_index]);
			break;
		case FIRE_EXTINGUISH_CAN_RESTART:
		case FIRE_EXTINGUISH_RESTART_FINISH:
			sprintf((char*)buf, "%03d %d/%02d/%02d %02d:%02d:%02d 第3回路 %d号 火警 灭火装置手动停止启动", sequence,
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

			n = snprintf(p, remain, "%02d%03d 复合探测器", circuit, addr);
			p += n;
			remain -= n;

			if ((enable & (1 << 5)) && remain > 0)
			{
				int16_t temp = RS485Detect_GetTemperature(addr);
				n = snprintf(p, remain, " 温度:%d℃", temp);
				p += n;
				remain -= n;
			}
			if ((enable & (1 << 0)) && remain > 0)
			{
				uint8_t smoke = RS485Detect_GetSensorState(addr, RS485_SENSOR_SMOKE);
				n = snprintf(p, remain, " 烟雾:%s", RS485Detect_IsFaultState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "故障" : (RS485Detect_IsAlarmState(RS485Detect_GetType(addr), RS485_SENSOR_SMOKE, smoke) ? "报警" : "正常"));
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
				snprintf(p, remain, " 压力:%dhPa", pressure);
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
			const char *status_str = MBusCtrl_IsAlarmState(addr) ? "报警" : "正常";
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
			snprintf((char *)buf, 128, "%02d%03d 探测器", circuit, addr);
			break;
	}
}


/*! 
*  \brief  消息处理流程
*  \param msg 待处理消息
*  \param size 消息长度
*/
void ProcessMessage( PCTRL_MSG msg, uint16 size )
{
    uint8 cmd_type = msg->cmd_type;                                                  //指令类型
    uint8 ctrl_msg = msg->ctrl_msg;                                                  //消息的类型
    uint8 control_type = msg->control_type;                                          //控件类型
    uint16 screen_id = PTR2U16(&msg->screen_id);                                     //画面ID
    uint16 control_id = PTR2U16(&msg->control_id);                                   //控件ID
    uint32 value = PTR2U32(msg->param);                                              //数值

    switch(cmd_type)
    {  
    case NOTIFY_TOUCH_PRESS:                                                        //触摸屏按下
    case NOTIFY_TOUCH_RELEASE:                                                      //触摸屏松开
        NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4)); 
        break;                                                                    
    case NOTIFY_WRITE_FLASH_OK:                                                     //写FLASH成功
        NotifyWriteFlash(1);                                                      
        break;                                                                    
    case NOTIFY_WRITE_FLASH_FAILD:                                                  //写FLASH失败
        NotifyWriteFlash(0);                                                      
        break;                                                                    
    case NOTIFY_READ_FLASH_OK:                                                      //读取FLASH成功
        NotifyReadFlash(1,cmd_buffer+2,size-6);                                     //去除帧头帧尾
        break;                                                                    
    case NOTIFY_READ_FLASH_FAILD:                                                   //读取FLASH失败
        NotifyReadFlash(0,0,0);                                                   
        break;                                                                    
    case NOTIFY_READ_RTC:                                                           //读取RTC时间
        NotifyReadRTC(cmd_buffer[2],cmd_buffer[3],cmd_buffer[4],cmd_buffer[5],cmd_buffer[6],cmd_buffer[7],cmd_buffer[8]);
        break;
    case NOTIFY_CONTROL:
        {
            if(ctrl_msg==MSG_GET_CURRENT_SCREEN)                                    //画面ID变化通知
            {
                NotifyScreen(screen_id);                                            //画面切换调动的函数
            }else
						if(ctrl_msg==TUBIAO_shangchuan)                                    //画面ID变化通知
            {
               TB_sahngchuan(screen_id,control_id,control_type,msg->param[0]);                      //图标控件上传调动的函数
            }else
            {
                switch(control_type)
                {
                case kCtrlButton:                                                   //按钮控件
                    NotifyButton(screen_id,control_id,msg->param[1]);  
										zhu_min=0;//有触控操作清零倒计时，无操作5分钟后自动返回 主界面               
                    break;                                                             
                case kCtrlText:                                                     //文本控件
                    NotifyText(screen_id,control_id,msg->param);                       
                    break;                                                             
                case kCtrlProgress:                                                 //进度条控件
                    NotifyProgress(screen_id,control_id,value);                        
                    break;                                                             
                case kCtrlSlider:                                                   //滑动条控件
                    NotifySlider(screen_id,control_id,value);                          
                    break;                                                             
                case kCtrlMeter:                                                    //仪表控件
                    NotifyMeter(screen_id,control_id,value);                           
                    break;                                                             
                case kCtrlMenu:                                                     //菜单控件
                    NotifyMenu(screen_id,control_id,msg->param[0],msg->param[1]);      
                    break;                                                              
                case kCtrlSelector:                                                 //选择控件
                    NotifySelector(screen_id,control_id,msg->param[0]);                
                    break;                                                              
                case kCtrlRTC:                                                      //倒计时控件
                    NotifyTimer(screen_id,control_id);
                    break;
                default:
                    break;
                }
            } 
            break;  
        } 
    case NOTIFY_HandShake:                                                          //握手通知                                                     
//        NOTIFYHandShake();
        break;
    default:
        break;
    }
}
/*! 
*  \brief  握手通知
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
*  \brief  画面切换通知
*  \details  当前画面改变时(或调用GetScreen)，执行此函数
*  \param screen_id 当前画面ID
*/
void NotifyScreen(uint16 screen_id)
{
	uint16_t prev_screen_id = current_screen_id; /* XR5000_MONITOR_RETURN_NAV_CHANGE_20260802 */
    //TODO: 添加用户代码
    current_screen_id = screen_id; //在工程配置中开启画面切换通知，记录当前画面ID
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
				return; // 快速结束
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
		
    //进到画面1刷新主机名称
    if(screen_id == 1)
    {
			SetTextValue(1, 11, (uint8_t *)"FGS-XR5000.火气报警控制器");//刷新主机名称
			
			SetTextInt32(1, 30, SystemSaveInfo.slave_addr485_Station,0,1); // 30是场站的
			
			SetTextInt32(1, 22, SystemSaveInfo.slave_addr485_EMS,0,1); // 22是EMS的
			
			SetTextInt32(1, 25, SystemSaveInfo.slave_addr485_EMS,0,1); // 25是CAN2的ID
			
			SetTextInt32(1, 8, alarm_number,0,1);  //报警总数显示
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
			
			sprintf((char *)temp_buff, "回路%d", pack_circuit);
			SetTextValue(4, 57, temp_buff); // 刷新回路名称
			
			for(uint8_t i = 0; i < 32; i++)
			{
				temp_buff[0] = pack_online_buff[pack_circuit][i + 1] ? 1 : 0;
				
				// 显示启用状态
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
			//回路1
			sprintf((char *)temp_buff, "设置上线:%d", getPointDetectorSetUpCount());
			SetTextValue(screen_id, 7, temp_buff); 
			sprintf((char *)temp_buff, "设备在线:%d", getPointDetectorSetUpLive());
			SetTextValue(screen_id, 8, temp_buff); 
			sprintf((char *)temp_buff, "设备故障:%d", (getPointDetectorFaultCount() + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP1)));
			SetTextValue(screen_id, 9, temp_buff); 
			sprintf((char *)temp_buff, "报警设备:%d", getPointDetectorAlarmCount());
			SetTextValue(screen_id, 10, temp_buff); 
			sprintf((char *)temp_buff, "屏蔽设备:0");
			SetTextValue(screen_id, 11, temp_buff); 
			//回路2
			{
				uint8_t mbus2_online = MBusCtrl_GetOnlineCount();
				uint8_t mbus2_disconnect = MBusCtrl_GetDisconnectCount();
				sprintf((char *)temp_buff, "设备上线:%d", mbus2_online);
				SetTextValue(screen_id, 13, temp_buff);
				sprintf((char *)temp_buff, "设备在线:%d", MBusCtrl_GetActiveCount());
				SetTextValue(screen_id, 14, temp_buff);
				sprintf((char *)temp_buff, "设备故障:%d", (mbus2_disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP2)));
				SetTextValue(screen_id, 15, temp_buff);
				sprintf((char *)temp_buff, "报警设备:%d", MBusCtrl_GetAlarmCount());
				SetTextValue(screen_id, 16, temp_buff);
				sprintf((char *)temp_buff, "屏蔽设备:0");
				SetTextValue(screen_id, 17, temp_buff);
			}
			//回路3
			uint8_t online = RS485Detect_GetOnlineCount();
			uint8_t disconnect = RS485Detect_GetDisconnectCount();
			sprintf((char *)temp_buff, "设置上线:%d", online);
			SetTextValue(screen_id, 19, temp_buff);
			sprintf((char *)temp_buff, "设备在线:%d", RS485Detect_GetActiveCount());
			SetTextValue(screen_id, 20, temp_buff);
			sprintf((char *)temp_buff, "设备故障:%d", (disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP3)));
			SetTextValue(screen_id, 21, temp_buff);
			sprintf((char *)temp_buff, "报警设备:%d", RS485Detect_GetAlarmCount());
			SetTextValue(screen_id, 22, temp_buff);
			sprintf((char *)temp_buff, "屏蔽设备:0");
			SetTextValue(screen_id, 24, temp_buff); 
		}
		else if(screen_id == 7) // 
		{
			if(pack_circuit == 0 || pack_circuit >= 4)
			{
				pack_circuit = 1;
			}
			uint8_t temp_buff[8] = {0};
			sprintf((char *)temp_buff, "回路%d", pack_circuit);
			SetTextValue(7, 231, temp_buff); // 刷新回路名称
			switch(pack_circuit)
			{
				case 1:
					for (uint8_t i = 1; i < 33; i++)
					{
						if (getPointTypeMixtureDetectOnlineState(i) == 0)
						{
							temp_buff[0] = 0; // 红色：未上线
						}
						else if (getPointTypeMixtureDisconnectCount(i) >= MIXTURE_DEVICE_DISCONNECT_SUM)
						{
							temp_buff[0] = 2; // 黄色：上线但掉线
						}
						else
						{
							temp_buff[0] = 1; // 绿色：在线正常
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
			// 	AnimationPlayFrame(7, i, temp_buff[0]);//(画面ID,控件ID,帧ID) 0红，1绿
			// }
		}
		else if(current_screen_id == 10)
    {
			uint8_t len = 0;
			uint8_t buff[48] = {0};
			if(strlen((char *)SystemSaveInfo.pref_license_store) != 0)
			{
				len = sprintf((char *)buff, "当前预置许可证:");
				for(uint8_t i = 0; i < 10; i++)
				{
					buff[len + i] = SystemSaveInfo.pref_license_store[i];
				}
				SetTextValue(10, 9, buff);
			}
			else
			{
				SetTextValue(10, 9, "当前预置许可证:无");
			}
			clearTextValue(10, 7);
			
			SetTextValue(10, 17, "上次生效许可证:");
			SetTextValue(10, 18, SystemSaveInfo.last_license_store);
			
			SetTextValue(10, 1, "当前输入许可证:");
			SetTextValue(10, 10, SystemSaveInfo.curr_license_store);
			
			char slicense_buff[10] = {0};
			for(uint8_t i = 0; i < 6; i++)
			{
				generate_new_license_code((char *)SystemSaveInfo.last_license_store, getGenerationDate(i), slicense_buff);
				SetTextValue(10, 11 + i, (uint8_t *)slicense_buff);
			}
			
			sprintf((char *)buff, "生效时间:%d/%d/%d %d:%d:%d", SystemSaveInfo.license_year, SystemSaveInfo.license_month, SystemSaveInfo.license_days, 
				SystemSaveInfo.license_hour, SystemSaveInfo.license_minute, SystemSaveInfo.license_second);
			SetTextValue(10, 20, buff);
			
			sprintf((char *)buff, "剩余时间:%d", remain_use_time);
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
				// 显示启用状态
				setkey_Value(17, pack_online_ctrl_button_id[i], value);
				// 显示在线数量
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
			clearTextValue(19,2);//(画面ID,控件ID）	
			SetTextValue(19,2,"复位系统！");
		}
		else if(screen_id == 21)
    {

		}
		else if(screen_id==22)  //                                        
    {
			
		}
		else if(screen_id == 23)
		{
			clearTextValue(23,2);//(画面ID,控件ID）
		}
		else if(screen_id == 24)
		{
			clearTextValue(24,2);//(画面ID,控件ID）	
			clearTextValue(24,5);//(画面ID,控件ID）
			clearTextValue(24,6);//(画面ID,控件ID）
			clearTextValue(24,7);//(画面ID,控件ID）
		}
		else if(current_screen_id == 25)
    {
			// 定义所有需要设置的控件ID数组
			const uint8_t setkey_controls[] = {5,8,11,14,17,20,25,28,31,34,37,40,50,53,56,59,62,65,68,71};
			const uint8_t clearText_controls[] = {6,9,12,15,18,21,26,29,32,35,38,41,49,52,55,58,61,64,67,70};
			// 循环设置setkey_Value
			for (int i = 0; i < sizeof(setkey_controls)/sizeof(setkey_controls[0]); i++) {
					setkey_Value(current_screen_id, setkey_controls[i], 0);
			}

			// 循环清除文本值
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
			SetTextInt32(current_screen_id,2,SystemSaveInfo.factory_release_year + 2000,0,1);//出厂年
			SetTextInt32(current_screen_id,3,SystemSaveInfo.factory_release_month,0,1);//出厂月
			SetTextInt32(current_screen_id,4,SystemSaveInfo.factory_release_days,0,1);//出厂日
			SetTextValue(current_screen_id,5,(unsigned char*)banben);
			
			if(strlen((char *)SystemSaveInfo.curr_license_store) != 0)
			{
				uint8_t len = 0;
				uint8_t buff[32] = {0};
				len = sprintf((char *)buff, "许可证书:");
				for(uint8_t i = 0; i < 10; i++)
				{
					buff[len + i] = SystemSaveInfo.curr_license_store[i];
				}
				SetTextValue(27, 6, buff);
			}
			else
			{
				SetTextValue(27, 6, "许可证书:无");
			}
			
			if(SystemSaveInfo.license_remain_day == 6666)
			{
				SetTextValue(27, 7, "许可状态:暂无许可");
			}
			else if(SystemSaveInfo.license_remain_day != 999)
			{
				uint8_t slicense_buff[32] = {0};
				sprintf((char *)slicense_buff, "许可状态:剩余%d天", remain_use_time);
				SetTextValue(27, 7, slicense_buff);
			}
			else
			{
				SetTextValue(27, 7, "许可状态:永久有效");
			}
			
		}
		else if(screen_id == 41)
		{
			// 清空之前输入的内容
			clearTextValue(screen_id, 16);
			clearTextValue(screen_id, 17);
			clearTextValue(screen_id, 18);
			clearTextValue(screen_id, 19);
			clearTextValue(screen_id, 20);
			clearTextValue(screen_id, 21);
			// 获取一次时间
			BM8563_Soft_I2C_GetTime(&SystemTime); // 读一次时间 存放到全局结构体中
			
			SetTextInt32(screen_id,  9, SystemTime.year + 2000, 0, 4);//出厂年
			SetTextInt32(screen_id, 10, SystemTime.month      , 0, 2);//出厂月
			SetTextInt32(screen_id, 11, SystemTime.day        , 0, 2);//出厂日
			
			SetTextInt32(screen_id, 12, SystemTime.hours      , 0, 2);//出厂年
			SetTextInt32(screen_id, 13, SystemTime.minutes    , 0, 2);//出厂月
			SetTextInt32(screen_id, 14, SystemTime.seconds    , 0, 2);//出厂日
		}
		else if(screen_id == 50)
		{
			SetTextInt32(50, 10, SystemSaveInfo.factory_release_year + 2000, 0, 4);//出厂年
			SetTextInt32(50, 11, SystemSaveInfo.factory_release_month      , 0, 2);//出厂月
			SetTextInt32(50, 12, SystemSaveInfo.factory_release_days       , 0, 2);//出厂日
		}
		else if(screen_id == 59)
		{
			for(uint8_t i = 9 ;i<24;i++)
			{
				clearTextValue(59 , i);//(画面ID,控件ID)
			}
			clearTextValue(59 , 2);//(画面ID,控件ID)
		}
		else if(screen_id == 66)
		{
			uint8_t temp_value = 0;
			for(uint8_t i = 1; i < 33; i++)
			{
				temp_value = getPointTypeMixtureDetectOnlineState(i) ? 1 : 0;
				// 显示启用状态
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
			sprintf((char *)temp_buff, "当前显示：第 %d 回路", screen69_circuit);
			SetTextValue(current_screen_id, 200, temp_buff);
			sprintf((char *)temp_buff, "注:此界面仅显示已上线并在线探测器");
			SetTextValue(69, 400, temp_buff);
			sprintf((char *)temp_buff, "上一页");
			SetTextValue(69, 500, temp_buff);
			sprintf((char *)temp_buff, "下一页");
			SetTextValue(69, 501, temp_buff);
			sprintf((char *)temp_buff, "返回");
			SetTextValue(69, 502, temp_buff);
			/* XR5000_SCREEN69_RESIDUAL_FIX_20260729: clear text controls reused by screen 6 summary. */
			for(uint8_t i = 1;i<25;i++)
			{
				clearTextValue(69 , i);//(画面ID,控件ID)
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
*  \brief  触摸坐标事件响应
*  \param press 1按下触摸屏，3松开触摸屏
*  \param x x坐标
*  \param y y坐标
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y)
{ 
    //TODO: 添加用户代码
}

uint8_t debug_flag = 0;
uint32_t last_time_stamp = -3600000; // 上电后更新一次剩余时间时间
/*! 
*  \brief  更新数据
*/ 
/* XR5000_SCREEN69_FLICKER_FIX_20260727: 画面69缓存结构体，用原始数据值比较替代strcmp字符串比较，避免频闪 */
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
	uint8_t shield_sum = 0; // 屏蔽总数
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
	// 新增加内容
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
		multiple_alarm_fresh_flag = 1; // 刷新标志位置一
		fed_fresh_flag = 1; // 每秒一刷
		
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
			
			getBM8563TimeToSystemTime(); // 获取一下RTC时间
			
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
//				license_allow_use_state = 0; //当默认值为1的时候将这里注释掉
				remain_use_time = 0;
			}
				
			if(remain_use_time < 4) // 
			{
				uint8_t slicense_buff[64] = {0};
				sprintf((char *)slicense_buff, "敬告:您的剩余使用天数只剩余%d天,请点击左上角图标获取更多信息", remain_use_time);
				SetTextValue(1, 3, slicense_buff);
			}
			else
			{
				clearTextValue(1, 3);
			}
		}
	}
	
	// 获取设备设置为上线的数量，给结构体赋值
	getDetectorSetUpLiveSum(&ds, cang_sxzt, cu_tcq_sxzt);
		
	screen_fresh_num++;				// 屏幕刷新控制
	if(screen_fresh_num > 60)
	{
		screen_fresh_num = 0;
	}

	if(kaijiyanshi >= ONLINE_TIMEOUT)//开机延时ONLINE_TIMEOUT秒后才计算掉线数量
	{
		// 获取包掉线数 并处理包报警数据 给结构体赋值
		pack_disconnect_sum = ClusterPackDataDeal_Plus(pcfs, &pcfs_buttom_point); 
		// 获取仓掉线数 并处理仓报警数据 存储到报警信息中 目前仓没有送检 没有添加报警信息 2025/07/09
		cabin_disconnect_sum = CabinDataDeal(pcfs, &pcfs_buttom_point); 
		//
		point_type_disconnect_sum = PointTypeDetectorDataDeal(pcfs, &pcfs_buttom_point); // 2025/11/17 10:27 添加点型二总线探测器
		// XR5000_LOOP3_CHANGE_20260726: Loop 3 realtime fault/alarm bridge.
		rs485_detect_disconnect_sum = RS485DetectDataDeal(pcfs, &pcfs_buttom_point);
		mbus2_disconnect_sum = MBus2DataDeal(pcfs, &pcfs_buttom_point);
		combustible_gas_alarm_active = RefreshCombustibleGasAlarmLed();
		/* 功能调整：废弃IG3306及4路独立24V输出监测；时间：2026-08-06 */
		// 判断是否有掉线 吸合故障继电器，新增加对回路三，485探测回路的故障判断
		FaultRelayCtrlAppFun(pack_disconnect_sum + cabin_disconnect_sum + point_type_disconnect_sum + rs485_detect_disconnect_sum + mbus2_disconnect_sum);
		// 判断是否有预警 吸合预警继电器
		ForeWarmRelayCtrlAppFun(&pcfws);
		// 判断是否有火警 吸合火警继电器
		FireAlarmRelayCtrlAppFun(pas_pointer);
		
		// 计算簇掉线数量	本质是01模块掉线了
		for(uint8_t sum = 1; sum < PACK_USER_NUM + 1; sum++)
		{
			if(CU_zx_buf[sum] == PackDisconnectCount)
			{
				
			}
		}

		// 新增内容
//		uint8_t fire_alarm_state;
//		fire_alarm_state = FireAlarmCompoundLogicJudgement(fire_alarm_logic_ctrl, fire_alarm_judge,cabin_detector_state_buff);
//		if(fire_alarm_state == fire_alarm)
//		{
//			SetTextValue(40, 62, "舱内起火");//刷新报警内容
//		}
//		else if(fire_alarm_state == normal)
//		{
//			SetTextValue(40, 62, "舱内正常");//刷新报警内容
//		}

		// end
			
		alarm_number = pcfas.self_bottom_point + pcfws.self_bottom_point; // 将火警数量赋值

		/*
		// 目前不显示预警 火警 统一显示报警 此处只用来做火警标记 执行外联设备动作
		if(pas_pointer != 0) { // 如果火警记录不为零
			fire_alarm_state = 1; // 点亮火警指示灯
			fire_alarm_flag.cluster_alarm_state = 1;
			
			if (fanr.storage_pas_len != pas_pointer) {
				fanr.storage_pas_len = pas_pointer;
				
				// 用位图优化去重（假设 ID 范围是 0~255）
				uint8_t seen[32] = {0};
				
				for (uint8_t i = 0; i < pas_pointer && fanr.faib_buttom_point < 300; i++) {
					uint8_t id = (pas[i].cabin_id != 0) ? pas[i].cabin_id : pas[i].cluster_id;
					uint8_t idx = id / 8;
					uint8_t bit = id % 8;
						
					if (!(seen[idx] & (1 << bit))) { // 如果未出现过
						seen[idx] |= (1 << bit);     // 标记为已出现
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
				BMS_BJ[i]=1; // 将符合要求的报警启动
				beiguangkai();//背光开
				SetControlForeColor(1,4,0xf800);
				SetTextValue(1,4,"BMS高温状态：第1簇电池高温！");//刷新报警内容
				SaveSensor(0,32,0,0,0,0,0,0,0); // 储存下来 BMS报警记录
				
				// 其他内容待补充
			}
			else if(BMS_Temp[i] < 3 && BMS_BJ[i] == 1)
			{
				BMS_BJ[i] = 0;
			}
		}
		*/
//		// 风机在线/掉线记录
//		BspFanOnlineJudgeFaultRecord(NULL, NULL);
//		// 风机启动控制 暂时没有记录风机启动停止时间

		BspFanStartCrtlApp(fan_state1, combustible_gas_alarm_active, fedas.self_point_len);
		// 主备电管理控制
		PowerManageCtrl(zhu_state, bei_state);

		
		// 主面板上报警器控制按键 认为是预警
		if((screen_show_siren_information&0x0F) == 0x0F && (screen_show_siren_information&0xF0) != 0xF0)
		{
			screen_show_siren_information |= 0xF0; // 标记执行过 
			// 记录到火警分区中 按键按下 存入FLASH在前可以少一次获取RTC操作
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, LINKAGE_PRESS, LINKAGE_CLUSTER_ID, ALARM_ANNUNCIATOR_ID, 0xFFFF);
//			// 存入cache缓冲区
//			StoragePackCabinForeWarn(&pcfws, LINKAGE_CLUSTER_ID, ALARM_ANNUNCIATOR_ID, AlarmCtrlKey);
			// 存入火灾报警区域 
			StoragePackFireAlarm(&pcfas, LINKAGE_CLUSTER_ID, ALARM_ANNUNCIATOR_ID, AlarmCtrlKey); // 记录手报按下
			
			// 点亮声光
			SoundLightRelayCtrl(JDQ_ON);
			
			SysSirenStartLedCtrl(LED_ON);
			
			silencers_state  = 0;  // 有新的报警 关闭消音指示灯
		}
		
		if(getHandPaperState() == 0x0F) // 手报认为是火警
		{
			// 确保只执行一次
			setDealHandPaperState();
			// 存入FLASH
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, LINKAGE_PRESS, LINKAGE_CLUSTER_ID, HANDPOT_Package_ID, 0xFFFF);
			//
			StoragePackFireAlarm(&pcfas, LINKAGE_CLUSTER_ID, HANDPOT_Package_ID, HandAlarm); // 记录手报按下

			fire_alarm_state = 1; // 点亮火警指示灯
			silencers_state = 0;  // 有新的报警 关闭消音指示灯
			/* 黑匣子:记录手报火警(自动判定首警) */
			StorageEvent_LogFire(HANDPOT_Package_ID, DEV_TYPE_HAND_REPORT, 1, 0);
			FecbusReport_Fire(HANDPOT_Package_ID, DEV_TYPE_HAND_REPORT, 1, 0); /* FECbus:手报火警 */
		}
		
		// 有报警后切换主界面 点亮屏幕 
		InternalScreenMainInterfaceCtrl(&switch_ui_ctrl);
	}
	//新增加报警总数的判断RS485Detect_GetAlarmCount();
	uint8_t total_alarm = alarm_number;
	if(last_alarm_num != total_alarm)
	{
		last_alarm_num = total_alarm;
		SetTextInt32(1, 8, total_alarm,0,1);  //报警总数显示
		if(total_alarm != 0)
		{
			beep_fire_ctrl |= 0xF0;  // 真 火警 长鸣
			silencers_state = 0; // 有新的报警 蜂鸣器开 清除消音标志位
			// 点亮声光
			SoundLightRelayCtrl(JDQ_ON);
			
			ForeWarmRelayCtrl(JDQ_ON);
		}
	}
	
	FirstAlarmInformationShowCtrl(current_screen_id, &sicj, &pcfws, &pcfas);

	FireExtinguishDeviceStateUpdate(&fedas, pas); 
	
	//各页面刷新
	if(current_screen_id==1)                                              
	{
		if(getControllorSelfCheckState() == 1) // 如果自检按键按下 //显示自检内容
		{
			// 打开自检灯
			SpecialSelfCheckLedCtrl(LED_ON);
			switch(screen_fresh_num)
			{
				case 1:SetTextValue(1,4,"系统自检中.     ");break;   //刷新报警内容
				case 2:SetTextValue(1,4,"系统自检中..    ");break;   //刷新报警内容
				case 3:SetTextValue(1,4,"系统自检中...   ");break;   //刷新报警内容
				case 4:SetTextValue(1,4,"系统自检中....  ");break;   //刷新报警内容
				case 5:SetTextValue(1,4,"系统自检中..... ");break;   //刷新报警内容
				case 6:SetTextValue(1,4,"系统自检中......");break;   //刷新报警内容
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
				show_content_delay = 0; // 目前只用来延时做状态跳转 
			}
			
			switch(self_check_show_content)
			{
				case 0: {
					uint16_t temp_flash_read_id = W25QXX_ReadID();
					if(temp_flash_read_id != W25Q512) // 表明FLASH 出现问题
					{
						// 待创建一条故障记录
						creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_FLASH_FAULT_ID, DISCONNECT);
					}						
					SetTextValue(1, 12, "片上存储系统自检中");//
					break;
				}
				case 1:
					SetTextValue(1, 12, "指示灯自检中");//
					break;
				case 2:
					SetTextValue(1, 12, "蜂鸣器自检中");//
					break;
				case 3:
					SetTextValue(1, 12, "声光自检中");//
					SoundLightRelayCtrl(JDQ_ON);
					DefauleRelayCtrl(JDQ_ON);
					break;
				case 4:
					SetTextValue(1, 12, "系统自检完成");//
					SoundLightRelayCtrl(JDQ_OFF);
					DefauleRelayCtrl(JDQ_OFF);
					clearTextValue(1 , 12);//(画面ID,控件ID)
					break;
				default:
					self_check_show_content = 0; // 显示内容复位
					show_content_delay = 0; // 清空延时 为下一次做准备
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
					case 10:SetTextValue(1, 4, "系统初始化中.     ");break;   //刷新报警内容
					case 20:SetTextValue(1, 4, "系统初始化中..    ");break;   //刷新报警内容
					case 30:SetTextValue(1, 4, "系统初始化中...   ");break;   //刷新报警内容
					case 40:SetTextValue(1, 4, "系统初始化中....  ");break;   //刷新报警内容
					case 50:SetTextValue(1, 4, "系统初始化中..... ");break;   //刷新报警内容
					case 60:SetTextValue(1, 4, "系统初始化中......");break;   //刷新报警内容
				}
			}
			else
			{
				switch(screen_fresh_num)
				{
					case 10:SetTextValue(1, 4, "报警系统运行中.     ");break;   //刷新报警内容
					case 20:SetTextValue(1, 4, "报警系统运行中..    ");break;   //刷新报警内容
					case 30:SetTextValue(1, 4, "报警系统运行中...   ");break;   //刷新报警内容
					case 40:SetTextValue(1, 4, "报警系统运行中....  ");break;   //刷新报警内容
					case 50:SetTextValue(1, 4, "报警系统运行中..... ");break;   //刷新报警内容
					case 60:SetTextValue(1, 4, "报警系统运行中......");break;   //刷新报警内容
				}
			}
			//  新增设备总数/在线/故障的显示
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
			//新增AHT20温度湿度显示
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
						sprintf((char *)aht20_buff, "温度：%d℃", temp);
						SetTextValue(current_screen_id, 39, aht20_buff);
					}
					if(humi != last_aht20_humi || aht20_valid != last_aht20_valid || home_statistics_force_refresh)
					{
						last_aht20_humi = humi;
						sprintf((char *)aht20_buff, "湿度：%d%%", humi);
						SetTextValue(current_screen_id, 40, aht20_buff);
					}
				}
				else if(aht20_valid != last_aht20_valid || home_statistics_force_refresh)
				{
					SetTextValue(current_screen_id, 39, "温度：--℃");
					SetTextValue(current_screen_id, 40, "湿度：--%");
				}
				last_aht20_valid = aht20_valid;
			}
			home_statistics_force_refresh = 0;
		}
		
		//处理其他页面事件
		zhu_min=0;//返回主界面时间清零，如果不在主界面，5分钟后自动返回
	
		// 新增内容
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
		//回路1
		uint8_t temp_buff[32] = {0};
		sprintf((char *)temp_buff, "第 1 回路信息汇总");
		SetTextValue(6, 6, temp_buff);
		sprintf((char *)temp_buff, "设置上线:%d", getPointDetectorSetUpCount());
		SetTextValue(6, 7, temp_buff);
		sprintf((char *)temp_buff, "设备在线:%d", getPointDetectorSetUpLive());
		SetTextValue(6, 8, temp_buff);
		sprintf((char *)temp_buff, "设备故障:%d", (getPointDetectorFaultCount() + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP1)));
		SetTextValue(6, 9, temp_buff);
		sprintf((char *)temp_buff, "报警设备:%d", getPointDetectorAlarmCount());
		SetTextValue(6, 10, temp_buff);
		sprintf((char *)temp_buff, "屏蔽设备:0"); 
		SetTextValue(6, 11, temp_buff);

		//回路2
		{
			uint8_t mbus2_online = MBusCtrl_GetOnlineCount();
			uint8_t mbus2_disconnect = MBusCtrl_GetDisconnectCount();
			sprintf((char *)temp_buff, "设备上线:%d", mbus2_online);
			SetTextValue(6, 13, temp_buff);
			sprintf((char *)temp_buff, "设备在线:%d", MBusCtrl_GetActiveCount());
			SetTextValue(6, 14, temp_buff);
			sprintf((char *)temp_buff, "设备故障:%d", (mbus2_disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP2)));
			SetTextValue(6, 15, temp_buff);
			sprintf((char *)temp_buff, "报警设备:%d", MBusCtrl_GetAlarmCount());
			SetTextValue(6, 16, temp_buff);
			sprintf((char *)temp_buff, "屏蔽设备:0");
			SetTextValue(6, 17, temp_buff);
		}

		//回路3
		uint8_t online = RS485Detect_GetOnlineCount();
        uint8_t disconnect = RS485Detect_GetDisconnectCount();
        sprintf((char *)temp_buff, "设置上线:%d", online);
        SetTextValue(6, 19, temp_buff);
        sprintf((char *)temp_buff, "设备在线:%d", RS485Detect_GetActiveCount());
        SetTextValue(6, 20, temp_buff);
        sprintf((char *)temp_buff, "设备故障:%d", (disconnect + DeviceRegistry_GetProductUnknownCountByLoop(DEVICE_REGISTRY_LOOP3)));
        SetTextValue(6, 21, temp_buff); 
        sprintf((char *)temp_buff, "报警设备:%d", RS485Detect_GetAlarmCount());
        SetTextValue(6, 22, temp_buff); 
		sprintf((char *)temp_buff, "屏蔽设备:0"); 
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
					temp_buff[0] = 0; // 红色：未上线
				}
				else if (getPointTypeMixtureDisconnectCount(i) >= MIXTURE_DEVICE_DISCONNECT_SUM)
				{
					temp_buff[0] = 2; // 黄色：上线但掉线
				}
				else
				{
					temp_buff[0] = 1; // 绿色：在线正常
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
//			char* str = "复合型探测器";
//			uint8_t baojingneirong[50];
////			sprintf((char*)baojingneirong,"设备总数 设备类别:%s 地址总数:%d",str,shebeizongshu);
//			SetTextValue(52,2,baojingneirong);//刷新报警内容
//			
//			sprintf((char*)baojingneirong,"在线总数 设备类别:%s 地址总数:%d",str,zaixianzongshu);
//			SetTextValue(52, 5, baojingneirong);//刷新报警内容
//			
//			sprintf((char*)baojingneirong,"故障总数 设备类别:%s 地址总数:%d",str,diaoxianzongshu);
//			SetTextValue(52, 6, baojingneirong);//刷新报警内容
//			
//			sprintf((char*)baojingneirong,"屏蔽总数 设备类别:%s 地址总数:%d",str,shield_sum);
//			SetTextValue(52, 7, baojingneirong);//刷新报警内容
	}
	if(current_screen_id==52)                                              
	{
		uint8_t baojingneirong[50];
		char* str = "复合型探测器";
		sprintf((char*)baojingneirong,"设备总数 设备类别:%s 地址总数:%d",str, ds.curr_num);
		SetTextValue(52,2,baojingneirong);//刷新报警内容
		
		sprintf((char*)baojingneirong,"在线总数 设备类别:%s 地址总数:%d",str, ds.curr_num - ( pack_disconnect_sum + cabin_disconnect_sum ));
		SetTextValue(52, 5, baojingneirong);//刷新报警内容
		
		sprintf((char*)baojingneirong,"故障总数 设备类别:%s 地址总数:%d",str, ( pack_disconnect_sum + cabin_disconnect_sum ));
		SetTextValue(52, 6, baojingneirong);//刷新报警内容
		
		shield_sum = getShieldDetectorSum(pack_pbzt, cang_pbzt);
		sprintf((char*)baojingneirong,"屏蔽总数 设备类别:%s 地址总数:%d",str, shield_sum);
		SetTextValue(52, 7, baojingneirong);//刷新报警内容
		
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
			SetTextValue(current_screen_id, 7, "正在清除记录... 请等待！");
			BspClearFlashData();
			SetTextValue(current_screen_id, 7, "清除完成！");
			qingchujilu=0;
		}
	}
	else if(current_screen_id == 54)
	{
		InternalScreenShowClusterData(&ddsc);
	}
	else if(current_screen_id == 56 || current_screen_id == 57)
	{
		// 更新显示
		InternalScreenShowRecord(&bsrr);
	}
	else if(current_screen_id == 59)
	{
		// new 故障界面刷新
		InternalScreenShowAllFault(getFaultCheckNewKey());
		if(getFaultCheckNewKey() == 1)
		{
			fault_check_new_flag = 0;
		}
		// end
		
		// NEW 
		// 2025/12/09 10:22 新增循序显示
		if(pcfws.self_bottom_point > Alarm_Show_Zone) // 如果报警数量超过了显示区域 
		{
			if(baojingjishi - pcfws.fresh_time_count >= 5)
			{
				pcfws.fresh_time_count = baojingjishi;
				// 翻页逻辑 // 当前页+显示区域长度 小于总数 则可以往后滚动
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
		// 预警界面刷新
		InternalScreenShowAllForceWorn_Plus(&pcfws, getForceAlarmCheckNewKey());
		if(getForceAlarmCheckNewKey() == 1)
			force_alarm_check_new_flag = 0;
		//END
		
		// NEW 
		// 报警界面刷新
		// 2025/12/09 10:41 新增循序显示
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
		// 报警刷新显示
		InternalScreenShowAllFireAlarm_Plus(&pcfas, getFireAlarmCheckNewKey());
		if(getFireAlarmCheckNewKey() == 1)
			fire_alarm_check_new_flag = 0;
		//END

		InternalScreenShowFireExtinguisher(&fedas, fed_fresh_flag);
		if(fed_fresh_flag)
		{
			fed_fresh_flag = 0;
		}
		
		// 箭头控制
		BspCheckNewKeyPressDeal(&bkcnc);
		
		RefreshGasConcentrationSummary(); /* XR5000_GAS_SUMMARY_CHANGE_20260731 */
	}		
	else if(current_screen_id == 61 || current_screen_id == 62) // 刷新不一样
	{
		// 更新PACK显示
//		InternalScreenShowClusterData_32Pack(current_screen_id, &ddsc_32p);
		// 更新PACK显示
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

	// 	/* 刚进入画面69时，页码归零 */
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

	// 	/* 控件200：回路标题 */
	// 	sprintf((char *)temp_buff, "当前显示回路：%d 回路", pack_circuit);
	// 	SetTextValue(current_screen_id, 200, temp_buff);

	// 	/* 收集在线设备 */
	// 	online_count = GetCircuitOnlineList(pack_circuit, online_list, 64);

	// 	/* 计算总页数，修正页码 */
	// 	total_pages = (online_count + 19) / 20;
	// 	if (total_pages == 0)
	// 		total_pages = 1;
	// 	if (g_screen69_page >= total_pages)
	// 		g_screen69_page = total_pages - 1;

	// 	/* 当前页范围 */
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

	// 	/* 清空剩余控件时也要清缓存 */
	// 	for (; ctrl_idx < 20; ctrl_idx++)
	// 	{
	// 		if (g_screen69_prev[ctrl_idx][0] != '\0')
	// 		{
	// 			SetTextValue(current_screen_id, ctrl_idx + 1, (uint8_t *)"");
	// 			g_screen69_prev[ctrl_idx][0] = '\0';
	// 		}
	// 	}

	// 	// /* 逐个显示 */因未会造成频闪所以删除
	// 	// for (ctrl_idx = 0, list_idx = start; list_idx < end; ctrl_idx++, list_idx++)
	// 	// {
	// 	// 	FormatScreen69DetectorText(pack_circuit, online_list[list_idx], temp_buff);
	// 	// 	SetTextValue(current_screen_id, ctrl_idx + 1, temp_buff);
	// 	// }

	// 	/* 清空剩余控件 */
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

			/* XR5000_SCREEN69_FLICKER_FIX_20260727: 改用原始数据值比较，替代strcmp字符串比较，避免频闪 */
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
	// 新增加内容
	InternalScreenLinkageMonitorUpdataUI(current_screen_id);
	
	OutFireDeviceInternalScreenUpdataUI(current_screen_id, out_fire_start_ctrl);
	
	//FireAlarmTriggerLogicUpdataUI(current_screen_id, fire_alarm_logic_ctrl, fire_alarm_judge); /* 旧火警逻辑UI已由bsp_logic_screen模块取代 */
	LogicScreen_UpdateUI(current_screen_id); /* 联动逻辑：编辑页预览/列表页规则刷新 */
	
	FireAlarmThresholdUpdataUI(current_screen_id, fire_alarm_threshold);
	/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
	CanMonitorRefreshDisplay(current_screen_id);
}
/*! 
*  \brief  图标按钮控件通知
*  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param state 按钮状态：0弹起，1按下
*  \param tubiaobh 图标帧编号：0第0帧，1第1帧，2第2帧...
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
*  \brief  按钮控件通知
*  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param state 按钮状态：0弹起，1按下
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8  state)
{
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
				silencers_state = 1; // 消音标志
			}
			// end
			beep_fire_ctrl = 0;
			beep_fault_ctrl = 0;
			main_power_beep_ctrl = 0;
			linkage_beep_ctrl = 0;
			
			beep_spray_feedback_ctrl = 0;
			beep_general_io_ctrl = 0; // 
		}
		else if(control_id==3 && state == 1)  // 如果按的是设置
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
		else if(control_id == 32 && state == 1)  // 如果按的是调整时间
		{
			EnterTimeDateSettingWithPassword(); /* XR5000_TIME_DATE_ENTRY_REUSE_20260802 */
		}
		else if( (control_id == 31 || control_id == 35) && state == 1 )
		{
			if(license_allow_use_state == 1)
			{
				setKeyValue(DEVICE_CTRL_KEY); // 给按键赋值 表明是修改屏幕的按键按下
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
		else if(control_id == 1 && state == 1) // 如果菜单按下
		{
			if(license_allow_use_state == 1)
			{
				//原始界面，切到了屏幕ID8
				// SwitchCurrentScreenId(8);
				// bsp_screen_switch_ctrl.target_screen = 8;
				// bsp_screen_switch_ctrl.switch_flag = 1;

				//新界面
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
	else if(screen_id == 4) // 如果是新的设备上下线界面
	{
		if(control_id == 45 && state == 1) // 一键上线按下
		{
			uint8_t modify_flag = 0;
			for(uint8_t i = 0; i < 32; i++)
			{
				if(pack_online_buff[pack_circuit][i + 1] == 0)
				{
					pack_online_buff[pack_circuit][i + 1] = 1;
					modify_flag = 1;
				}
				// 将状态变更为开启
				setkey_Value(4, point_type_detect_button_online_ctrl_val_map[i], 1);
			}
			if(modify_flag == 1)
			{
				Save_Pack_Set_Online_State();
			}
		}
		else if(control_id == 55 && state == 1) // 一键下线按下
		{
			uint8_t modify_flag = 0;
			for(uint8_t i = 0; i < 32; i++)
			{
				if(pack_online_buff[pack_circuit][i + 1] == 1)
				{
					pack_online_buff[pack_circuit][i + 1] = 0;
					modify_flag = 1;
				}
				
				// 将状态变更为开启
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
				setKeyValue(DEVICE_CTRL_KEY); // 给按键赋值 表明是修改屏幕的按键按下
			
				SwitchCurrentScreenId(53);
				bsp_screen_switch_ctrl.target_screen = 53;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
			else if(control_id == 6)
			{
				
			}
			else if(control_id == 23)
			{
				setKeyValue(SIMU_SERIAL_PORT); // 给按键赋值 表明是修改屏幕的按键按下
			
				SwitchCurrentScreenId(53);
				bsp_screen_switch_ctrl.target_screen = 53;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
			else if(control_id == 24)
			{
				setKeyValue(LINKAGE_PROGREM); // 给按键赋值 表明是修改屏幕的按键按下
			
				SwitchCurrentScreenId(53);
				bsp_screen_switch_ctrl.target_screen = 53;
				bsp_screen_switch_ctrl.switch_flag = 1;
			}
		}
		RecordSwitchButtonCtrl(&bsrr, control_id, state);
	}
	else if(screen_id ==17)//设备上下线簇级页面
  {
			if(state==0)//弹起
			{
				switch(control_id)
				{
					case 5: cu_sxzt[1]=0;cu_tcq_sxzt[1]=0;CU_zx_buf[1]=0;break;//20240202增加了CU_zx_buf[1]=0;防止整簇掉线后，设置下线，别的簇再报掉线显示编号不对
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
				Save_cu_sxzt();//存储簇上线状态
				Save_cutcq_sxzt();
				
				SetTextInt32(17,6,cu_tcq_sxzt[1],0,1);//设置文本为整数，（页面，控件，数值，0-无符号、1-有符号，数字位数，不足时左侧补零）
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
			else if(state==1)//按下
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
				Save_cu_sxzt();//存储簇上线状态
			}
	}
	else if(screen_id ==18)//设备上下线仓级页面
  {
		if(state==0)//弹起
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
				// 新增加内容
				case 51: cang_sxzt[21]=0;Cang_zx_buf[21]=0;break;
				case 52: cang_sxzt[22]=0;Cang_zx_buf[22]=0;break;
				case 53: cang_sxzt[23]=0;Cang_zx_buf[23]=0;break;
				case 54: cang_sxzt[24]=0;Cang_zx_buf[24]=0;break;
			}
			Save_cang_sxzt();//存储仓上线状态
		}
		else if(state==1)//按下
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
				// 新增加内容
				case 51: cang_sxzt[21]=1;break;
				case 52: cang_sxzt[22]=1;break;
				case 53: cang_sxzt[23]=1;break;
				case 54: cang_sxzt[24]=1;break;
			}
			Save_cang_sxzt();//存仓簇上线状态
		}
	}
	else if(screen_id == 19)
  {
		if(control_id==4 && state == 1)                                            
		{
			SetTextValue(1, 4, "控制器复位中...请稍候...");
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
		if(mimajiyi == 0) // 用户未输入密码 或者 4.25分钟已过 需要重新输入密码
		{
			if(control_id==4 && state == 1)                                            
			{
				if(yonghumima == SystemSaveInfo.user_password)                                                       
				{ 
					yonghumima=0;
					mimajiyi++;
					clearTextValue(23,2);//(画面ID,控件ID）
					SetScreen(2);	//（画面ID）切换画面到设置界面
					osDelay(10);
					GetScreen();
					current_screen_id=2;
				}else
				{
					clearTextValue(23,2);//(画面ID,控件ID）
					SetTextValue(23, 2, "密码错误！");
				}
			}
		}
	}
	//设置用户密码
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
					SetTextValue(24,7,"设置成功！");
					mmsdSTA=4;
				}
				else if(yonghumm2 != yonghumm3)
				{
					SetTextValue(24,7,"设置失败，新密码两次不一样！");
				}
			}
			else if(yonghumm1!=SystemSaveInfo.user_password)
			{
				SetTextValue(24,7,"设置失败，原密码错误！");
			}				
		}
	}
	else if(screen_id ==26)//
	{
		if(state==0)//弹起
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
				// 新增加内容
				case 51: cang_pbzt[21]=0;break;
				case 52: cang_pbzt[22]=0;break;
				case 53: cang_pbzt[23]=0;break;
				case 54: cang_pbzt[24]=0;break;
			}
			Save_cang_pbzt();//存储仓屏蔽状态
		}else
		if(state==1)//按下
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
				// 新增加内容
				case 51: cang_pbzt[21]=1;break;
				case 52: cang_pbzt[22]=1;break;
				case 53: cang_pbzt[23]=1;break;
				case 54: cang_pbzt[24]=1;break;
			}
			Save_cang_pbzt();//存仓簇屏蔽状态 pack_bianhaobuf
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
			SystemInfoSave(); // 从设置出厂日期界面退出再保存进EEPROM
		}
	}
	else if(screen_id == 71U)
	{
		if(control_id == 300U && state == 1U)
		{
			/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
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
					case SELFCHECK_KEY: { // 自检
						SetScreen(1);	// 密码正确 回主界面
						osDelay(5);
						GetScreen(); 
						// 创建一条其它记录 记录自检按键按下的时间 
						BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_SYS_SELF_CHECK, LINKAGE_CLUSTER_ID, SYS_SELFCHECK_Package_ID);
						SpecialSelfCheckLedCtrl(LED_ON);
						
						break;
					}
					case SILENSE_KEY: // 消音
						break;
					case RESET_KEY: {  // 复位 
						// 记录复位按键按下
						SetScreen(1);	// 密码正确 回主界面
						osDelay(5);
						GetScreen();
						SetTextValue(1, 4, "控制器复位中...请稍候...");
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
					case MODIFY_TIME_KEY:  // 修改时间按键
						SetScreen(41);	// 进入二级密码页
						osDelay(5);
						GetScreen();
						break;
					case DEVICE_CTRL_KEY: {
						SetScreen(6);	//
						osDelay(5);
						GetScreen();
						break;
					}
					case DEVICE_SHIELD_KEY: { // XR5000_DEVICE_SHIELD_ENTRY_20260802: 设备屏蔽
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
					case SIREN_KEY:    // 报警器启动
						screen_show_siren_information ^= 0x0F; // 翻转低四位状态
						break;
					case LINKAGE_START_KEY: // 外联设备启动
						linkage_start_key_press_flag = 1;
						break;
					case PART1_SPRY_START:
						FireExtinguishDevice1HandStart(&fedas);
						break;
					case PART2_SPRY_START:
						FireExtinguishDevice2HandStart(&fedas);
						break;
					default:
						SetScreen(1);	// 进入二级密码页
						osDelay(5);
						GetScreen();
						break;
				}
				setKeyValue(NONE_KEY);
			}
			else if(yonghumima == 114514)
			{
				SetScreen(3);	// 进入二级密码页
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
	else if(screen_id == 58) // 手动强启某一簇
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

	//2026/7/22新增内容
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
	//FireAlarmTriggerLogicButtonSet(screen_id, control_id, state, &fire_alarm_logic_ctrl); /* 旧火警逻辑按键已由bsp_logic_screen模块取代 */
	LogicScreen_OnButton(screen_id, control_id, state); /* 联动逻辑：逻辑设定界面按键处理 */
	SuperAdminButtonCtrl(screen_id, control_id, state, &button_ctrl);
	SuperAdminPasswordButtonCtrl(screen_id, control_id, state, &super_admin_password);
	
}


/*! 
*  \brief  文本控件通知
*  \details  当文本通过键盘更新(或调用GetControlValue)时，执行此函数
*  \details  文本控件的内容以字符串形式下发到MCU，如果文本控件内容是浮点值，
*  \details  则需要在此函数中将下发字符串重新转回浮点值。
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param str 文本控件内容
*/
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str)
{
	 LogicScreen_OnText(screen_id, control_id, str); /* 联动逻辑：文本输入事件转发（预留） */
	 if(screen_id == 75U && control_id >= 1U && control_id <= 6U)
	 {
		CanMonitorSetChannelName((uint8_t)control_id, str);
	 }
	 if(screen_id==1)                                                                 //画面ID42：上线文本
   { 
			if(control_id == 25) // 修改CAN2ID地址
      {
				int32 value=0;  			
				sscanf((const char*)(char*)str,"%ld",&value); 
				SystemSaveInfo.can2_slave_addr = value;//MODBUS地址
				SystemInfoSave();
				SystemInfoLoad();
			}
			else if(control_id == 30) // 修改场站485地址
      {
				int32 value=0;  			
				sscanf((const char*)(char*)str,"%ld",&value); 
				SystemSaveInfo.slave_addr485_Station = value;//MODBUS地址
				SystemInfoSave();
				SystemInfoLoad();
			}
			else if(control_id == 22)
			{
				int32 value=0;  			
				sscanf((const char*)(char*)str,"%ld",&value); 
				SystemSaveInfo.slave_addr485_EMS = value;//MODBUS地址
				SystemInfoSave();
				SystemInfoLoad();
			}
		}
    if(screen_id==2)                                                                 //画面ID2：文本设置和显示
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
					if(success_len == 2 && x >0 && y > 0) // 解析成功 并且x y大于零
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
							// 显示启用状态
							setkey_Value(4, point_type_detect_button_online_ctrl_val_map[i - 1], 1);
						}
						if(modify_flag == 1)
						{
							Save_Pack_Set_Online_State(); // 避免多次擦写FLASH
						}
					}
				}
				SetTextValue(4, 2, "批量上线");
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
					if(success_len == 2 && x >0 && y > 0) // 解析成功 并且x y大于零
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
									// 显示启用状态
								}
								if (modify_flag == 1)
								{
									SavePointTypeSetOnlieState();  // 避免多次擦写FLASH
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
			SetTextValue(6, 2, "批量上线");
		}
		else if(screen_id == 10)
		{
			if(control_id == 8)
			{
				uint8_t len = strlen((char *)str);
				
				SetTextValue(10, 8, "请输入预置许可证ID");
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
					len = sprintf((char *)buff, "当前预置许可证:");
					for(uint8_t i = 0; i < 10; i++)
					{
						buff[len + i] = SystemSaveInfo.pref_license_store[i];
					}
					SetTextValue(10, 9, buff);
				}
				else
				{
					SetTextValue(10, 9, "当前预置许可证:无");
				}

			}
		}
		else if(screen_id==17)  
    {                                                                           
        int32 value=0;  
        sscanf((const char*)str, "%ld", &value);    //把字符串转换为整数 
				switch(control_id)
				{
					case 6:if(cu_sxzt[1]==1){cu_tcq_sxzt[1]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 9:if(cu_sxzt[2]==1){cu_tcq_sxzt[2]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 12:if(cu_sxzt[3]==1){cu_tcq_sxzt[3]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 15:if(cu_sxzt[4]==1){cu_tcq_sxzt[4]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 18:if(cu_sxzt[5]==1){cu_tcq_sxzt[5]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 21:if(cu_sxzt[6]==1){cu_tcq_sxzt[6]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 26:if(cu_sxzt[7]==1){cu_tcq_sxzt[7]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 29:if(cu_sxzt[8]==1){cu_tcq_sxzt[8]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 32:if(cu_sxzt[9]==1){cu_tcq_sxzt[9]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 35:if(cu_sxzt[10]==1){cu_tcq_sxzt[10]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 38:if(cu_sxzt[11]==1){cu_tcq_sxzt[11]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 41:if(cu_sxzt[12]==1){cu_tcq_sxzt[12]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					
					case 49:if(cu_sxzt[13]==1){cu_tcq_sxzt[13]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 52:if(cu_sxzt[14]==1){cu_tcq_sxzt[14]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 55:if(cu_sxzt[15]==1){cu_tcq_sxzt[15]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 58:if(cu_sxzt[16]==1){cu_tcq_sxzt[16]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 61:if(cu_sxzt[17]==1){cu_tcq_sxzt[17]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 64:if(cu_sxzt[18]==1){cu_tcq_sxzt[18]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 67:if(cu_sxzt[19]==1){cu_tcq_sxzt[19]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
					case 70:if(cu_sxzt[20]==1){cu_tcq_sxzt[20]=value;kaijiyanshi=0;}break;//获取单个设置文本数值
				}
				Save_cutcq_sxzt();
				SetTextInt32(17,6,cu_tcq_sxzt[1],0,1);//设置文本为整数，（页面，控件，数值，0-无符号、1-有符号，数字位数，不足时左侧补零）
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
		else if(screen_id==23)                                                                 //画面ID2：文本设置和显示
    {
			if(mimajiyi == 0) // 用户未输入密码 或者 4.25分钟已过 需要重新输入密码
			{
				if(control_id==2)                                                            //
				{
					int32 value=0;  			
					sscanf((const char*)(char*)str,"%ld",&value);                                                    //把字符串转换为整数 					
					yonghumima=value;													//更新
				}  
			}                                                                     
    }
		else if(screen_id == 24)                                                                 //画面ID2：文本设置和显示
    {                                                                            
        int32 value=0;  			
        sscanf((const char*)(char*)str,"%ld",&value);                                                    //把字符串转换为整数 
				if(control_id==2)                                                            //
        {                                                                         
					if(value == SystemSaveInfo.user_password)                                                       
            { 
							
							SetTextValue(24,7,"密码正确！");
							yonghumm1=value;//记录用户原密码
							value = 0; 
						}else
						{
							SetTextValue(24,7,"密码错误！");
						}						
        } 
				if(control_id==5)                                                            //
        {                                                                         
						yonghumm2=value;//记录新用户密码1
						value = 0; 
        }	
				if(control_id==6)                                                            //
        {                                                                         
						yonghumm3=value;//记录新用户密码2
						value = 0; 
        }				
    }
		else if(screen_id==25)                                                                 //画面ID2：文本设置和显示
    {                                                                            
 
    }
		else if(screen_id == 27)
		{
			if(control_id == 9) // 更新许可证输入
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
					len = sprintf((char *)buff, "许可证书:");
					for(uint8_t i = 0; i < 10; i++)
					{
						buff[len + i] = SystemSaveInfo.curr_license_store[i];
					}
					SetTextValue(27, 6, buff);
				}
				else
				{
					SetTextValue(27, 6, "许可证书:无");
				}
				
				char slicense_buff[10] = {0};
				
				if(strncmp((char *)SystemSaveInfo.curr_license_store, (char *)SystemSaveInfo.pref_license_store, 10) == 0 && SystemSaveInfo.license_remain_day == 6666) // 比较预置ID 和 初始化天数
				{
					for(uint8_t i = 0; i < 10; i++)
					{
						SystemSaveInfo.last_license_store[i] = SystemSaveInfo.curr_license_store[i];
					}
					
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
							
					SystemSaveInfo.license_year = years;
					SystemSaveInfo.license_month = months;
					SystemSaveInfo.license_days = days;
					SystemSaveInfo.license_hour = hours;
					
					SystemSaveInfo.license_minute = minutes;
					SystemSaveInfo.license_second = secs;
					
					SystemSaveInfo.license_remain_day = 30; // 默认试用30天
					
					uint8_t slicense_buff[32] = {0};
					sprintf((char *)slicense_buff, "许可状态:剩余%d天", SystemSaveInfo.license_remain_day);
					SetTextValue(27, 7, slicense_buff);
				}
				else // 新输入的和上一次的不一样
				{
					uint8_t flag = 0;
					
					for(uint8_t i = 0; i < 6; i++)
					{
						generate_new_license_code((char *)SystemSaveInfo.last_license_store, getGenerationDate(i), slicense_buff);
						if(strncmp((char *)SystemSaveInfo.curr_license_store, slicense_buff, 10) == 0)
						{
							flag = 1; // 标记找到
							getBM8563TimeToSystemTime(); // 获取一下RTC时间
							
							SystemSaveInfo.license_year = years;
							SystemSaveInfo.license_month = months;
							SystemSaveInfo.license_days = days;
							SystemSaveInfo.license_hour = hours;
							
							SystemSaveInfo.license_minute = minutes;
							SystemSaveInfo.license_second = secs;

							SystemSaveInfo.license_remain_day = getRemainUseDate(i); // 记录日期
							
							for(uint8_t i = 0; i < 10; i++)
							{
								SystemSaveInfo.last_license_store[i] = SystemSaveInfo.curr_license_store[i];
							}
							if(SystemSaveInfo.license_remain_day != 999)
							{
								uint8_t slicense_buff[32] = {0};
								sprintf((char *)slicense_buff, "许可状态:剩余%d天", SystemSaveInfo.license_remain_day);
								SetTextValue(27, 7, slicense_buff);
							}
							else
							{
								SetTextValue(27, 7, "许可状态:永久有效");
							}
							
							break;
						}
					}
					if(flag == 0) // 如果没有找到匹配的
					{
						SetTextValue(27, 7, "许可状态:无效");
						SystemSaveInfo.license_remain_day = 0; // 剩余天数清零
					}
					
				}
				SystemInfoSave();
				last_time_stamp -=3600000;
				SetTextValue(27, 9, "更新使用许可证");
			}
		}
		else if(screen_id == 41) // 时间修改界面
		{
			// 新增内容
			InternalScreenRTCSetting(screen_id, control_id, str); //RTC修改
		}
		else if(screen_id == 50)
		{
			if(control_id == 17)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // 把字符串转换为整数 
				SystemSaveInfo.factory_release_year = value - 2000;
				
			}		
			else if(control_id == 18)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // 把字符串转换为整数 
				SystemSaveInfo.factory_release_month = value;
			}		
			else if(control_id == 19)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // 把字符串转换为整数 
				// 出场日期设置
				SystemSaveInfo.factory_release_days = value;
			}		
		}
		else if(screen_id == 53)
		{
			if(control_id == 2)
			{
				int32 value=0;  			
				sscanf((char *)str,"%ld",&value); // 把字符串转换为整数 
				yonghumima=value;									// 给密码赋值
			}				
		}
		else if(screen_id == 67)
		{
			PointTypeDetectorTextInputCtrlApp(&ptsc, control_id, str);
			
			CompositeDetectorTextInputCtrlApp(&cpsc, control_id, str);
		}
		
		// 新增内容
		OutFireDeviceInternalScreenTexttSet(screen_id, control_id, str, &out_fire_start_ctrl); // 喷放逻辑修改
		
		SuperAdminInternalScreenTextCtrl(screen_id, control_id, str, &super_admin_password);
		
		FireAlarmThresholdSettingInternalScreenText(screen_id, control_id, str, &fire_alarm_threshold);
}                                                                                

/*!                                                                              
*  \brief  进度条控件通知                                                       
*  \details  调用GetControlValue时，执行此函数                                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/                                                                              
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value)           
{  
//    if(screen_id == 5)
//    {
//        Progress_Value = value;                                  
//        SetTextInt32(5,2,Progress_Value,0,1);                                        //设置文本框的值     
//    }    
}                                                                                

/*!                                                                              
*  \brief  滑动条控件通知                                                       
*  \details  当滑动条改变(或调用GetControlValue)时，执行此函数                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/                                                                              
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value)             
{                                                             

}


/*! 
*  \brief  仪表控件通知
*  \details  调用GetControlValue时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param value 值
*/
void NotifyMeter(uint16 screen_id, uint16 control_id, uint32 value)
{
    //TODO: 添加用户代码
}

/*! 
*  \brief  菜单控件通知
*  \details  当菜单项按下或松开时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 菜单项索引
*  \param state 按钮状态：0松开，1按下
*/
void NotifyMenu(uint16 screen_id, uint16 control_id, uint8 item, uint8 state)
{
  //TODO: 添加用户代码
	// 菜单更新控件 灭火喷放逻辑设定 火警触发逻辑设定 在此处调用
	
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
					// 显示启用状态
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
					setKeyValue(DEVICE_SHIELD_KEY); // XR5000_DEVICE_SHIELD_ENTRY_20260802: 设备屏蔽
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
					setKeyValue(LINKAGE_PROGREM); // 给按键赋值 表明是修改屏幕的按键按下
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
				/* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
				bsp_screen_switch_ctrl.target_screen = 71U;
				bsp_screen_switch_ctrl.switch_flag = 1U;
				SwitchCurrentScreenId(71U);
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
*  \brief  选择控件通知
*  \details  当选择控件变化时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 当前选项
*/
void NotifySelector(uint16 screen_id, uint16 control_id, uint8  item)
{

}


/*! 
*  \brief  定时器超时通知处理
*  \param screen_id 画面ID
*  \param control_id 控件ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id)
{
    if(screen_id==8&&control_id == 7)
    {
        SetBuzzer(100);
    } 
}


/*! 
*  \brief  读取用户FLASH状态返回
*  \param status 0失败，1成功
*  \param _data 返回数据
*  \param length 数据长度
*/
void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length)
{
    //TODO: 添加用户代码
}


/*! 
*  \brief  写用户FLASH状态返回
*  \param status 0失败，1成功
*/
void NotifyWriteFlash(uint8 status)
{
    //TODO: 添加用户代码
}


/*! 
*  \brief  读取RTC时间，注意返回的是BCD码
*  \param year 年（BCD）
*  \param month 月（BCD）
*  \param week 星期（BCD）
*  \param day 日（BCD）
*  \param hour 时（BCD）
*  \param minute 分（BCD）
*  \param second 秒（BCD）
*/
void NotifyReadRTC(uint8 year,uint8 month,uint8 week,uint8 day,uint8 hour,uint8 minute,uint8 second)
{

       
    secs    =(0xff & (second>>4))*10 +(0xf & second);                                    //BCD码转十进制
    years   =(0xff & (year>>4))*10 +(0xf & year);                                      
    months  =(0xff & (month>>4))*10 +(0xf & month);                                     
    weeks   =(0xff & (week>>4))*10 +(0xf & week);                                      
    days    =(0xff & (day>>4))*10 +(0xf & day);                                      
    hours   =(0xff & (hour>>4))*10 +(0xf & hour);                                       
    minutes =(0xff & (minute>>4))*10 +(0xf & minute);  
//   	uart1_printf("时间1： %d年%d月%d日%d时%d分%d秒\r\n",years,months,days,hours,minutes,secs);
//    SetTextInt32(8,1,years,1,1);
//    SetTextInt32(8,2,months,1,1);
//    SetTextInt32(8,3,days,1,1);
//    SetTextInt32(8,4,hours,1,1);
//    SetTextInt32(8,5,minutes,1,1);
//    SetTextInt32(8,6,sec,1,1);

}

// 获取仓 簇所有探测器上线状态
static void getDetectorSetUpLiveSum(DetectorSum *ds_entry, uint8_t cabin_setup[], uint8_t cluster_setup[])
{
	uint8_t detector_sum = 0;
	// 上电更新舱上线数量
	for(uint8_t sum = 1; sum < CANG_USER_NUM + 1; sum++)
	{
		detector_sum = detector_sum + cabin_setup[sum];
	}
			
	// 包上线数量
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
	
	ds_entry->curr_num = detector_sum; // 给设备总数赋值
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
// 获取点型探测器的故障总数量
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
// 获取点型探测器的报警总数量
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
		if(cluster_id == LINKAGE_CLUSTER_ID) // 如果簇ID等于外联设备编号
		{
			// 如果是外联设备则簇ID一定相同 如果相同类型中存在相同ID 则退出
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
		else // 如果都不是就是仓
		{
			if(pcfs[k].da.cabin_id == cabin_id)
			{
				flag = 1;
				break;
			}
		}
	}
	
	if(flag != 1) // 如果没有相同的
	{
		getBM8563TimeToSystemTime(); // 获取一下RTC时间
		
		// 记录报警ID 类型
		if(cluster_id == LINKAGE_CLUSTER_ID) // 如果簇ID等于外联设备编号
		{
			pcfs[pcfs_buttom_point].detector_class = LinkageClassID; // 明确探测器类型是外联设备
			pcfs[pcfs_buttom_point].da.cabin_id    = cabin_id;       // 将仓号来传输报警类型 
			pcfs[pcfs_buttom_point].da.cluster_id  = cluster_id;
			pcfs[pcfs_buttom_point].da.pack_id     = pack_id;
		}
		else if(cluster_id != 0)
		{
			pcfs[pcfs_buttom_point].detector_class = PackClassID; // 明确探测器类型是包
			pcfs[pcfs_buttom_point].da.cabin_id    = 0;           // 清空仓ID编号 如果是包则默认清空
			pcfs[pcfs_buttom_point].da.cluster_id  = cluster_id;
			pcfs[pcfs_buttom_point].da.pack_id     = pack_id;
		}
		else
		{
			pcfs[pcfs_buttom_point].detector_class = CabinClassID; // 明确探测器类型是仓
			pcfs[pcfs_buttom_point].da.cabin_id    = cabin_id; 
			pcfs[pcfs_buttom_point].da.cluster_id  = 0;
			pcfs[pcfs_buttom_point].da.pack_id     = 0;
		}

		// 记录报警时间
		pcfs[pcfs_buttom_point].atr.years  = years + 2000;
		pcfs[pcfs_buttom_point].atr.months = months;
		pcfs[pcfs_buttom_point].atr.days   = days;
		pcfs[pcfs_buttom_point].atr.hours  = hours;
		pcfs[pcfs_buttom_point].atr.minute = minutes;
		
		// 2025/11/19 10:59 新增记录报警秒
		pcfs[pcfs_buttom_point].atr.second = secs;
		pcfs[pcfs_buttom_point].fault_type = RS485_LOOP3_FAULT_OFFLINE;
		
		pcfs_buttom_point++; // 底指针自增
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
		pcfs[k].detector_class = pcfs[k + 1].detector_class; // 将后一个赋给前一个
		pcfs[k].da             = pcfs[k + 1].da;
		pcfs[k].atr            = pcfs[k + 1].atr;
		pcfs[k].fault_type     = pcfs[k + 1].fault_type;
	}
	pcfs_buttom_point--;

}

// 返回值 包掉线数量
static uint8_t ClusterPackDataDeal(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	uint8_t disconnect_detector_sum = 0;

	MaxCombustibleGas_t temp_pack_mcg_co = {0};
	temp_pack_mcg_co.co_max_val = -1;
	
	for(uint8_t jsz = 1;jsz <= 20;jsz++) // 遍历20簇
	{
		if(cu_tcq_sxzt[jsz] == 0) // 如果没设置上线也就没有处理的必要了
		{
			continue;
		}
		for(uint8_t i=1;i<=cu_tcq_sxzt[jsz];i++)//循环次数由设置上线数量决定 遍历每一簇下的包
		{
			if(getClusterPackDisconnectCount(jsz, i) == PackDisconnectCount) { // 如果设置为上线 判断是否掉线
				// 新增内容 对所有故障信息统一处理
	
				disconnect_detector_sum++; // 掉线设备+1
				// 一定要写0否则会出错
				if(creatNewFaultRecordToCache(jsz, i, 0) == 0) // 如果返回0表示成功写入 需要启动蜂鸣器 并存储FLASH
				{
					beep_fault_ctrl  = 2;   // 蜂鸣器开 故障蜂鸣器标志位
					silencers_state  = 0;   // 消音灯灭
					disconnect_state = 1;   // 点亮故障灯
					// 修改为函数存储
					//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
					BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, jsz, i);
				}
			}else if(PACK_zx_buf[jsz][i] == i) { //如果在线值是对应的ID编号 在线 在进行判断
				// new
				if(pack_pbzt[jsz][i]==0 && PACK_WDZT_buf[jsz][i] != 0 && BJ_packjiyibuf_wd[jsz][i] == 0)
				{
					// new
					// 记录探测器温度报警
					getBM8563TimeToSystemTime(); // 获取一下RTC时间 
					// 存入临时缓冲区供屏幕显示使用
					StoragePackFireAlarm(&pcfas, jsz, i, Temperature);
					
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, jsz, i, PACK_wendu_buf[jsz][i]);
					// 更新存储记忆 只存一次
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
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
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
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
					StoragePackCabinForeWarn(&pcfws, jsz, i, Carbon);
					// end
					// 确保只存储一次 防止出现一直存导致FLASH损坏
					BJ_packjiyibuf_co[jsz][i] = PACK_COZT_buf[jsz][i];
					
					// 保存可燃气体
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM, jsz, i, getPackCoConcenValue(jsz, i));
				}
				else if(BJ_packjiyibuf_co[jsz][i] != 0 && PACK_COZT_buf[jsz][i] == 0) { // 证明之前存储过
					// 一氧化碳可以自恢复，需要从数组中删掉
					// NEW
					// 2025/10/11 10:35 可燃气体自恢复指的是探测器不是主机 所以不需要自恢复
//					DeletPackCabinForeWarn(&pcfws, jsz, i, Carbon);
//					// 2025/9/2 17:13
//					getBM8563TimeToSystemTime(); // 获取一下RTC时间
//					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, EAR_RECOVERY, jsz, i, getPackCoConcenValue(jsz, i));

					// END
					BJ_packjiyibuf_co[jsz][i]=0;
				}			
					
				if(PACK_CH4ZT_buf[jsz][i]!=0)
				{
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
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
				//		|| BJ_packjiyibuf_wd[jsz][i] == 2 // 暂时屏蔽掉二级温度报警启动喷放 以免产生误报
				)
			{
				// 复合火警判断
				uint8_t flag = 0;
				for(uint8_t j = 0;j < pas_pointer; j++)
				{
					if(pas[j].cabin_id == 0) // 如果舱ID等于0 表明该位置存储的不是舱
					{
						if(pas[j].cluster_id == jsz	 && pas[j].pack_id == i)
						{
							flag = 1;
							break; // 如果该报警编号已经存储过了 跳出循环
						}
					}
				}
				if(flag != 1) // 表示没有存储过
				{
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
					// 标记为簇火警
					fire_alarm_flag.cluster_alarm_state = 1;
					
					pas[pas_pointer].cluster_id  = jsz;
					pas[pas_pointer].pack_id     = i;
					pas[pas_pointer].cabin_id    = 0;
					pas[pas_pointer].lunch_state = 0; // 启动状态 未启动	
					// 时间赋值
					pas[pas_pointer].atr.years  = years + 2000;
					pas[pas_pointer].atr.months = months;
					pas[pas_pointer].atr.days   = days;
					pas[pas_pointer].atr.hours  = hours;
					pas[pas_pointer].atr.minute = minutes;
					
					// 2025/11/19 10:59 新增记录报警秒
					pas[pas_pointer].atr.second = secs;
					
					pas_pointer++;
				}
			}			
		}
	}
	
	if(*pcfs_point > 0)
	{
		disconnect_state = 1; // 点亮故障灯
	}
	
	if(disconnect_detector_sum < *pcfs_point) // 如果掉线设备数小于指针总数 证明有设备恢复了
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
				// 如果是外联设备编号 跳过本次
				continue;
			}
			// 如果不是掉线值
			if(getClusterPackDisconnectCount(pcfs_entry[k].da.cluster_id, pcfs_entry[k].da.pack_id) != PackDisconnectCount) 
			{
				flag = 1;
				break;
			}
		}
		if(flag == 1)
		{
			deletRecoveryRecord(k);
			// 存储进FLASH
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, pcfs_entry[k].da.cluster_id, pcfs_entry[k].da.pack_id);
			if(*pcfs_point > 0) // 如果仍有报警存在，继续响
			{
				beep_fault_ctrl  = 2; // 蜂鸣器开 故障蜂鸣器标志位
				silencers_state  = 0; // 消音灯灭
				disconnect_state = 1; // 点亮故障灯
			}		
		}
	}
	
	mcg[PACK_CO_ID] = temp_pack_mcg_co;
	
	return disconnect_detector_sum;
}

// 返回值 包掉线数量
static uint8_t ClusterPackDataDeal_Plus(PackCabinFaultStorage *pcfs_entry, uint8_t *pcfs_point)
{
	uint8_t disconnect_detector_sum = 0;

	MaxCombustibleGas_t temp_pack_mcg_co = {0};
	temp_pack_mcg_co.co_max_val = -1;
	
	for(uint8_t jsz = 1;jsz < 4; jsz++) // 遍历20簇
	{
		for(uint8_t i = 1; i < 33; i++)//循环次数由设置上线数量决定 遍历每一簇下的包
		{
			if(pack_online_buff[jsz][i] == 0) // 如果探测器未上线
			{
				continue;
			}
			if(getClusterPackDisconnectCount(jsz, i) == PackDisconnectCount) // 如果设置为上线 判断是否掉线
			{ 
				// 新增内容 对所有故障信息统一处理
	
				disconnect_detector_sum++; // 掉线设备+1
				// 一定要写0否则会出错
				if(creatNewFaultRecordToCache(jsz, i, 0) == 0) // 如果返回0表示成功写入 需要启动蜂鸣器 并存储FLASH
				{
					beep_fault_ctrl  = 2;   // 蜂鸣器开 故障蜂鸣器标志位
					silencers_state  = 0;   // 消音灯灭
					disconnect_state = 1;   // 点亮故障灯
					// 修改为函数存储
					//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
					BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, jsz, i);
				}
			}
			else if(PACK_zx_buf[jsz][i] == i)  //如果在线值是对应的ID编号 在线 在进行判断
			{
				// new
				if(pack_pbzt[jsz][i]==0 && PACK_WDZT_buf[jsz][i] != 0 && BJ_packjiyibuf_wd[jsz][i] == 0)
				{
					// new
					// 记录探测器温度报警
					getBM8563TimeToSystemTime(); // 获取一下RTC时间 
					// 存入临时缓冲区供屏幕显示使用
					StoragePackFireAlarm(&pcfas, jsz, i, Temperature);
					
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, jsz, i, PACK_wendu_buf[jsz][i]);
					// 更新存储记忆 只存一次
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
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
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
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
					StoragePackCabinForeWarn(&pcfws, jsz, i, Carbon);
					// end
					// 确保只存储一次 防止出现一直存导致FLASH损坏
					BJ_packjiyibuf_co[jsz][i] = PACK_COZT_buf[jsz][i];
					
					// 保存可燃气体
					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM, jsz, i, getPackCoConcenValue(jsz, i));
				}
				else if(BJ_packjiyibuf_co[jsz][i] != 0 && PACK_COZT_buf[jsz][i] == 0) { // 证明之前存储过
					// 一氧化碳可以自恢复，需要从数组中删掉
					// NEW
					// 2025/10/11 10:35 可燃气体自恢复指的是探测器不是主机 所以不需要自恢复
//					DeletPackCabinForeWarn(&pcfws, jsz, i, Carbon);
//					// 2025/9/2 17:13
//					getBM8563TimeToSystemTime(); // 获取一下RTC时间
//					BspAlarmDataSaveApp(FIRE_FLASH_SAVE, EAR_RECOVERY, jsz, i, getPackCoConcenValue(jsz, i));

					// END
					BJ_packjiyibuf_co[jsz][i]=0;
				}			
					
				if(PACK_CH4ZT_buf[jsz][i]!=0)
				{
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
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
				//		|| BJ_packjiyibuf_wd[jsz][i] == 2 // 暂时屏蔽掉二级温度报警启动喷放 以免产生误报
				)
			{
				// 复合火警判断
				uint8_t flag = 0;
				for(uint8_t j = 0;j < pas_pointer; j++)
				{
					if(pas[j].cabin_id == 0) // 如果舱ID等于0 表明该位置存储的不是舱
					{
						if(pas[j].cluster_id == jsz	 && pas[j].pack_id == i)
						{
							flag = 1;
							break; // 如果该报警编号已经存储过了 跳出循环
						}
					}
				}
				if(flag != 1) // 表示没有存储过
				{
					getBM8563TimeToSystemTime(); // 获取一下RTC时间
					// 标记为簇火警
					fire_alarm_flag.cluster_alarm_state = 1;
					
					pas[pas_pointer].cluster_id  = jsz;
					pas[pas_pointer].pack_id     = i;
					pas[pas_pointer].cabin_id    = 0;
					pas[pas_pointer].lunch_state = 0; // 启动状态 未启动	
					// 时间赋值
					pas[pas_pointer].atr.years  = years + 2000;
					pas[pas_pointer].atr.months = months;
					pas[pas_pointer].atr.days   = days;
					pas[pas_pointer].atr.hours  = hours;
					pas[pas_pointer].atr.minute = minutes;
					
					// 2025/11/19 10:59 新增记录报警秒
					pas[pas_pointer].atr.second = secs;
					
					pas_pointer++;
				}
			}			
		}
	}
	
	if(*pcfs_point > 0)
	{
		disconnect_state = 1; // 点亮故障灯
	}
	
	if(disconnect_detector_sum < *pcfs_point) // 如果掉线设备数小于指针总数 证明有设备恢复了
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
			// 如果不是掉线值
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
			// 存储进FLASH
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, saved_cluster_id, saved_pack_id);
			if(*pcfs_point > 0) // 如果仍有报警存在，继续响
			{
				beep_fault_ctrl  = 2; // 蜂鸣器开 故障蜂鸣器标志位
				silencers_state  = 0; // 消音灯灭
				disconnect_state = 1; // 点亮故障灯
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
	
	//循环判断 24 个仓 探测在线 报警状态
	for(uint8_t jsz=1; jsz <= 24; jsz++) // 现在新增了四个仓探测器，循环次数也要改成24
	{
		if(cang_sxzt[jsz] != 1) // 如果没有设置上线 直接判断下一个
		{
			continue;
		}
		else if(Cang_zx_buf[jsz] == CabinDisconnectCount) // 如果掉线计数溢出 掉线数+1 判断下一个
		{
			temp_cabin_disconnect_sum++; // 计算仓掉线数量	

			if(DX_cangjiyibuf[jsz] == 0)
			{
				DX_cangjiyibuf[jsz] = 1;	
				if( creatNewFaultRecordToCache(0, 0, jsz) == 0 ) // 如果写入成功
				{
					beep_fault_ctrl  = 2;   // 蜂鸣器开 故障蜂鸣器标志位
					silencers_state  = 0;   // 消音灯灭
					disconnect_state = 1;   // 点亮故障灯
					// 修改为函数存储
					//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
					BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, 0, jsz); // 存储仓掉线
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
				// 存储进FLASH
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, 0, jsz);
				if(*pcfs_point > 0) // 如果仍有报警存在，继续响
				{
					beep_fault_ctrl  = 2; // 蜂鸣器开 故障蜂鸣器标志位
					silencers_state  = 0; // 消音灯灭
					disconnect_state = 1; // 点亮故障灯
				}		
			}
		}
		
		DX_cangjiyibuf[jsz] = 0;

		//仓温度判断
		if(Cang_WDZT_buf[jsz] != 0 && BJ_cangjiyibuf_wd[jsz] == 0) 
		{
			// new
			// 记录探测器温度报警
			getBM8563TimeToSystemTime(); // 获取一下RTC时间 
			// 存入临时缓冲区供屏幕显示使用
			StoragePackFireAlarm(&pcfas, 0, jsz, Temperature);
			
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, TEMPRT_ALARM, 0, jsz, Cang_wendu_buf[jsz]);
			// 更新存储记忆 只存一次
			BJ_cangjiyibuf_wd[jsz] = Cang_WDZT_buf[jsz];
			// end

			// 新增内容
			cabin_detector_state_buff[jsz].temperature_state = 1; // 温度预警
			// end
		}

		//仓烟雾判断
		if(Cang_YWZT_buf[jsz] != 0 && BJ_cangjiyibuf_yw[jsz] == 0) { 
			getBM8563TimeToSystemTime(); // 获取一下RTC时间

			// new
			getBM8563TimeToSystemTime(); // 获取一下RTC时间
			StoragePackFireAlarm(&pcfas, 0, jsz, Smoke);
			// end
			
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, SMOKE_ALARM, 0, jsz, 0xFFFF);
			
			BJ_cangjiyibuf_yw[jsz] = Cang_YWZT_buf[jsz];
			// 新增内容
			cabin_detector_state_buff[jsz].smoke_state = 1; // 烟雾一级预警
			// end
		}

		//仓一氧化碳判断
		if(Cang_COZT_buf[jsz] != 0 && BJ_cangjiyibuf_co[jsz] == 0) 
		{ 
			// new
			getBM8563TimeToSystemTime(); // 获取一下RTC时间
			StoragePackCabinForeWarn(&pcfws, 0, jsz, Carbon);
			// end

			// 保存可燃气体
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_CO, 0, jsz, Cang_COzhi_buf[jsz]);
			
			BJ_cangjiyibuf_co[jsz] = Cang_COZT_buf[jsz];
			
			// 新增内容
			cabin_detector_state_buff[jsz].carbon_state = Cang_COZT_buf[jsz]; // 一氧化碳一级预警
			
			// end
		}
		else if(Cang_COZT_buf[jsz] == 0 && BJ_cangjiyibuf_co[jsz] != 0)
		{
			BJ_cangjiyibuf_co[jsz] = 0;
			// 新增内容
			cabin_detector_state_buff[jsz].carbon_state = 0;      // 清除报警状态
		}
			
//			if(Cang_CH4ZT_buf[jsz]==1 && cang_pbzt[jsz]==0)//仓甲烷一级预警
//			{
//				if(BJ_cangjiyibuf_ch4[jsz]!=1)
//				{
//					BJ_cangjiyibuf_ch4[jsz]=1;
//					SaveSensor(jsz,1,Cang_wendu_buf[jsz]+40,Cang_YWZT_buf[jsz],Cang_COZT_buf[jsz],Cang_CH4ZT_buf[jsz],0,0,0);
//				}
//			}else if(Cang_CH4ZT_buf[jsz]==2 && cang_pbzt[jsz]==0) { //仓甲烷二级预警
//				if(BJ_cangjiyibuf_ch4[jsz]!=2) {
//					BJ_cangjiyibuf_ch4[jsz]=2;
//					SaveSensor(jsz,1,Cang_wendu_buf[jsz]+40,Cang_YWZT_buf[jsz],Cang_COZT_buf[jsz],Cang_CH4ZT_buf[jsz],0,0,0);
//				}
//			}
			
			
		//仓VOC判断
		if(Cang_VOCZT_buf[jsz] != 0 && BJ_cangjiyibuf_voc[jsz] == 0)
		{ 

			BJ_cangjiyibuf_voc[jsz] = Cang_VOCZT_buf[jsz];
		}
		else if(Cang_VOCZT_buf[jsz] == 0 && BJ_cangjiyibuf_voc[jsz] != 0)
		{
			
			BJ_cangjiyibuf_voc[jsz]=0;
			// 新增内容
		}
		
		//仓H2判断
		if(Cang_H2ZT_buf[jsz] != 0 && BJ_cangjiyibuf_h2[jsz] == 0) 
		{ 
			// new
			getBM8563TimeToSystemTime(); // 获取一下RTC时间
			StoragePackCabinForeWarn(&pcfws, 0, jsz, Hydrogen);
			// end

			// 保存可燃气体
			BspAlarmDataSaveApp(FIRE_FLASH_SAVE, FIRGAS_ALARM_HH, 0, jsz, Cang_H2zhi_buf[jsz]);
			
			BJ_cangjiyibuf_h2[jsz] = Cang_H2ZT_buf[jsz];
			
			// 新增内容
			cabin_detector_state_buff[jsz].hydrogen_state = 1; // 氢气一级预警
			
			// end
			
		}
		else if(Cang_H2ZT_buf[jsz] == 0 && BJ_cangjiyibuf_h2[jsz] != 0) 
		{
			
			// 新增内容
			cabin_detector_state_buff[jsz].hydrogen_state = 0;    // 清除报警状态
			BJ_cangjiyibuf_h2[jsz] = 0;
		}

		// end

		// 2025/10/27 16:42 新增可燃气体浓度判断
		
		if(Cang_H2zhi_buf[jsz] > temp_mcg_hh.co_max_val)
		{
			temp_mcg_hh.co_max_val = Cang_H2zhi_buf[jsz];
			
			temp_mcg_hh.gas_type = Hydrogen_Type;
			
			temp_mcg_hh.curr_da.cabin_id   = jsz;
			// 如果是仓 务必将簇和pack清0
			temp_mcg_hh.curr_da.cluster_id = 0;
			temp_mcg_hh.curr_da.pack_id    = 0;
		}
		
		if(Cang_COzhi_buf[jsz] > temp_mcg_co.co_max_val)
		{
			temp_mcg_co.co_max_val = Cang_COzhi_buf[jsz];
			
			temp_mcg_co.gas_type = Carbon_Type;
			
			temp_mcg_co.curr_da.cabin_id   = jsz;
			// 如果是仓 务必将簇和pack清0
			temp_mcg_co.curr_da.cluster_id = 0;
			temp_mcg_co.curr_da.pack_id    = 0;
		}
		
		
		// 舱多火警报警记录
		//仓复合火警判断
		if( // Cang_WDZT_buf[jsz]==2 || 
			(((BJ_cangjiyibuf_h2[jsz] != 0) || BJ_cangjiyibuf_voc[jsz] != 0 || BJ_cangjiyibuf_co[jsz] != 0 || BJ_cangjiyibuf_yw[jsz] != 0) 
			&& BJ_cangjiyibuf_wd[jsz] != 0 ) )
		{
			
			// new
			fire_alarm_state = 1;  // 标记火警(此处为舱内火警)
			// end
			
			// 记录所有的火警信息并储存
			uint8_t flag = 0;
			for(uint8_t k = 0;k < pas_pointer; k++)
			{
				if(pas[k].cluster_id == 0 && pas[k].pack_id == 0) // 如果簇ID和包ID等于0 表明该位置储存的是舱ID
				{
					if(pas[k].cabin_id == jsz) // 如果舱ID已经存储过
					{
						flag = 1;
						break; // 如果该报警编号已经存储过了 跳出循环
					}
				}
				
			}
			if(flag != 1) // 表示没有存储过
			{
				getBM8563TimeToSystemTime(); // 获取一下RTC时间
				// 标记为仓火警
				fire_alarm_flag.cabin_alarm_state = 1;
				
				pas[pas_pointer].cabin_id    = jsz;
				pas[pas_pointer].cluster_id  = 0;
				pas[pas_pointer].pack_id     = 0;
				pas[pas_pointer].lunch_state = 0;
				// 时间赋值
				pas[pas_pointer].atr.years  = years + 2000;
				pas[pas_pointer].atr.months = months;
				pas[pas_pointer].atr.days   = days;
				pas[pas_pointer].atr.hours  = hours;
				pas[pas_pointer].atr.minute = minutes;
				
				// 2025/11/19 10:59 新增记录报警秒
			pas[pas_pointer].atr.second = secs;

			pas_pointer++;

			/* 黑匣子:记录舱内复合火警(仅新增舱记录时,避免重复) */
			StorageEvent_LogFire(jsz, DEV_TYPE_FIRE_ALARM, 1, 0);
			FecbusReport_Fire(jsz, DEV_TYPE_FIRE_ALARM, 1, 0); /* FECbus:舱内复合火警 */

			beep_fire_ctrl = 1;  // 火警/预警 长鸣
			silencers_state = 0; // 有新的报警 蜂鸣器开 清除消音标志位
			}
			// end
		}

	}
	
	mcg[CABIN_CO_ID] = temp_mcg_co;
	mcg[CABIN_HH_ID] = temp_mcg_hh;
	
	return temp_cabin_disconnect_sum; // 返回仓掉线数量
}

static void FaultRelayCtrlAppFun(uint8_t disconnect_num)
{
	if(disconnect_num != 0) // 如果故障数量不为0 吸合故障干接点
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
	// 故障监控显示
	if(total_fault_count == 0U)
	{
		if(pcfs_fresh_ctrl != 0 || last_product_unknown_count != 0U)
		{
			disconnect_state = 0;  // 掉线状态解除 
			beep_fault_ctrl  = 0;  // 关闭掉线蜂鸣器
			pcfs_fresh_ctrl = 0;
			last_product_unknown_count = 0U;
			clearTextValue(monitor_inform_screen_id , 43);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 44);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 45);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 46);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 47);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 48);//(画面ID,控件ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 42,"故障监测运行中.     ");break;   //刷新报警内容
			case 20:SetTextValue(monitor_inform_screen_id, 42,"故障监测运行中..    ");break;   //刷新报警内容
			case 30:SetTextValue(monitor_inform_screen_id, 42,"故障监测运行中...   ");break;   //刷新报警内容
			case 40:SetTextValue(monitor_inform_screen_id, 42,"故障监测运行中....  ");break;   //刷新报警内容
			case 50:SetTextValue(monitor_inform_screen_id, 42,"故障监测运行中..... ");break;   //刷新报警内容
			case 60:SetTextValue(monitor_inform_screen_id, 42,"故障监测运行中......");break;   //刷新报警内容
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
					// XR5000_LOOP3_CHANGE_20260726: Loop 3 fault display uses "第3回路 X号".
				}
				else if(FormatMBus2FaultLine(baojingneirong, temp_sequence_count, pcfs, data_index) == 1)
				{
					// Loop 2 fault display uses "回路二 XX设备掉线".
				}
				else if(pcfs[data_index].detector_class == PackClassID) // 如果类型是包
				{
					// 2025/11/19 10:59 新增记录报警秒
                    static const uint8_t pack_format[] = {'%','0','3','d',' ','%','d','/','%','0','2','d','/','%','0','2','d',' ','%','0','2','d',':','%','0','2','d',':','%','0','2','d',' ',0xB5U,0xDAU,'%','d',0xB4U,0xD8U,' ','P','A','C','K','%','d',' ',0xB5U,0xF4U,0xCFU,0xDFU,0U};
                    sprintf((char*)baojingneirong, (const char*)pack_format, temp_sequence_count,
						pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
						pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second,
						pcfs[data_index].da.cluster_id, pcfs[data_index].da.pack_id);
				}
				else if(pcfs[data_index].detector_class == CabinClassID) // 如果类型是仓
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 掉线", temp_sequence_count, // 新增显示序号
						pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
						pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second,
						pcfs[data_index].da.cabin_id );
				}
				else if(pcfs[data_index].detector_class == LinkageClassID) // 如果是外联设备
				{
					switch(pcfs[data_index].da.pack_id)
					{
						
						case Deflate_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 放气勿入掉线", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days, 
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 放气勿入短路", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days, 
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case SoundLt_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 声光报警器掉线", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 声光报警器短路", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case SirenBk_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 警笛掉线", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 警笛短路", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case OutFir1_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 灭火装置1掉线", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 灭火装置1短路", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case OutFir2_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 灭火装置2掉线", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 灭火装置2短路", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case CabinBK_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 喷放装置掉线", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 喷放装置短路", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case FEEDBK1_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 反馈1掉线", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 反馈1短路", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case FEEDBK2_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 反馈2掉线", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 反馈2短路", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							
							break;
						case HANDPOT_Package_ID:
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 手报掉线", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 手报短路", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						case SYS_FLASH_FAULT_ID: { // 如果是存储故障
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 系统存储故障", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);

							break;
						}							
						case SYS_MAIN_POWER_KEY_ID : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 主电故障", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case SYS_BACK_POWER_KEY_ID : {
							if(pcfs[data_index].da.cabin_id == DISCONNECT)
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 备电故障", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							else
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 备电短路", temp_sequence_count,
									pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
									pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							}
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_1 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 回路1短路", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_2 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 回路2短路", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_3 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 回路3短路", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
						case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_4 : {
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 回路4短路", temp_sequence_count,
								pcfs[data_index].atr.years, pcfs[data_index].atr.months, pcfs[data_index].atr.days,
								pcfs[data_index].atr.hours, pcfs[data_index].atr.minute, pcfs[data_index].atr.second);
							break;
						}
					}
				}
			} 
			else {
				baojingneirong[0] = 0;
				//clearTextValue(1 , 41 + i);//(画面ID,控件ID）
			}
			SetTextValue(monitor_inform_screen_id, i + 42, baojingneirong);//刷新报警内容
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
	// 预警显示
	if(pcfws_entry->self_bottom_point == 0) // 如果报警数量为0 
	{
		if(pcfws_entry->point_history_len != 0)
		{
			pcfws_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 36);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 37);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 38);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 39);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 40);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 41);//(画面ID,控件ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中.     ");break;   //刷新报警内容
			case 20:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中..    ");break;   //刷新报警内容
			case 30:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中...   ");break;   //刷新报警内容
			case 40:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中....  ");break;   //刷新报警内容
			case 50:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中..... ");break;   //刷新报警内容
			case 60:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中......");break;   //刷新报警内容
		}
	}
	else if(pcfws_entry->self_bottom_point != pcfws_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfws_entry->point_history_len = pcfws_entry->self_bottom_point;
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.
		

		
		// 第一条报警信息置顶显示
		if(pcfws_entry->detector_class[0] == PackClassID && pcfws_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first warning display uses "第3回路 X号".
			FormatRS485DetectForeWarnLine(baojingneirong, 1, pcfws_entry, 0);
		}
		else if(pcfws_entry->detector_class[0] == PackClassID)
		{
			if(pcfws_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cluster_id, pcfws.da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
		}
		else if(pcfws_entry->detector_class[0] == LinkageClassID) // 如果是外联设备
		{
			if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 报警器按下", 1,
					pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
					pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second);
			}
		}
		else if(pcfws_entry->detector_class[0] == CabinClassID)
		{
			if(pcfws_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
		}
		
		SetTextValue(monitor_inform_screen_id, 35, baojingneirong); // 刷新第一条报警内容
			
		uint8_t temp_sequence_count = 0;
		
		// 剩下的区域滚动显示
		for (uint8_t i = 1; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fore_alarm_start_index + i; // 预警更新
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfws_entry->self_bottom_point &&
				pcfws_entry->detector_class[data_index] == PackClassID &&
				pcfws_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 warning display uses "第3回路 X号".
				FormatRS485DetectForeWarnLine(baojingneirong, temp_sequence_count, pcfws_entry, data_index);
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == PackClassID)
			{
				if(pcfws_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == LinkageClassID)
			{
				if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 报警器按下", temp_sequence_count,
						pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
						pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[data_index].second);
				}
				else if(pcfws_entry->alarm_type[data_index] == HandAlarm)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 手报按下报警", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[0].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second);
				}
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == CabinClassID)
			{
				if(pcfws_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 35 + i); //(画面ID,控件ID）
			}
			SetTextValue(monitor_inform_screen_id, 35 + i, baojingneirong); // 刷新报警内容
		}
	}
}

static void InternalScreenShowAllForceWorn_Plus(PackCabinForeWarnStorage *pcfws_entry, uint8_t fresh_page_flag)
{
	// 预警显示
	if(pcfws_entry->self_bottom_point == 0) // 如果报警数量为0 
	{
		if(pcfws_entry->point_history_len != 0)
		{
			pcfws_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 36);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 37);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 38);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 39);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 40);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 41);//(画面ID,控件ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中.     ");break;   //刷新报警内容
			case 20:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中..    ");break;   //刷新报警内容
			case 30:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中...   ");break;   //刷新报警内容
			case 40:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中....  ");break;   //刷新报警内容
			case 50:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中..... ");break;   //刷新报警内容
			case 60:SetTextValue(monitor_inform_screen_id, 35,"预警系统运行中......");break;   //刷新报警内容
		}
	}
	else if(pcfws_entry->self_bottom_point != pcfws_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfws_entry->point_history_len = pcfws_entry->self_bottom_point;
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.
		


		uint8_t temp_sequence_count = 0;
		
		// 剩下的区域滚动显示
		for (uint8_t i = 0; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fore_alarm_start_index + i; // 预警更新
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfws_entry->self_bottom_point &&
				pcfws_entry->detector_class[data_index] == PackClassID &&
				pcfws_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 warning display uses "第3回路 X号".
				FormatRS485DetectForeWarnLine(baojingneirong, temp_sequence_count, pcfws_entry, data_index);
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == PackClassID)
			{
				if(pcfws_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", temp_sequence_count,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cluster_id, pcfws_entry->da[data_index].pack_id);
				}
				
			}
			else if(data_index < pcfws_entry->self_bottom_point && pcfws_entry->detector_class[data_index] == LinkageClassID)
			{
				if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 报警器按下", temp_sequence_count,
						pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
						pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[data_index].second);
				}
				else if(pcfws_entry->alarm_type[data_index] == HandAlarm)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 手报按下报警", temp_sequence_count,
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
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
				else if(pcfws_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
						pcfws_entry->atr[data_index].years, pcfws_entry->atr[data_index].months, pcfws_entry->atr[data_index].days,
						pcfws_entry->atr[data_index].hours, pcfws_entry->atr[data_index].minute, pcfws_entry->atr[data_index].second,
						pcfws_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 35 + i); //(画面ID,控件ID）
			}
			SetTextValue(monitor_inform_screen_id, 35 + i, baojingneirong); // 刷新报警内容
		}
	}
}

static void InternalScreenShowAllFireAlarm(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag)
{
	// 火警监控
	if(pcfas_entry->self_bottom_point == 0) // 
	{
		if(pcfas_entry->point_history_len != 0)
		{
			pcfas_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 50);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 51);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 52);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 53);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 54);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 55);//(画面ID,控件ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中.     ");break;   //刷新报警内容
			case 20:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中..    ");break;   //刷新报警内容
			case 30:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中...   ");break;   //刷新报警内容
			case 40:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中....  ");break;   //刷新报警内容
			case 50:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中..... ");break;   //刷新报警内容
			case 60:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中......");break;   //刷新报警内容
		}
	}
	else if(pcfas_entry->self_bottom_point != pcfas_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfas_entry->point_history_len = pcfas_entry->self_bottom_point;
		
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.

		fire_alarm_state = 1; // 点亮火警指示灯
		
		// 第一条报警信息置顶显示
		if(pcfas_entry->detector_class[0] == PackClassID && pcfas_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first fire display uses "第3回路 X号".
			FormatRS485DetectFireAlarmLine(baojingneirong, 1, pcfas_entry, 0);
		}
		else if(pcfas_entry->detector_class[0] == PackClassID)
		{
			if(pcfas_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
		}
		else if(pcfas_entry->detector_class[0] == LinkageClassID)
		{
			if(pcfas_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 报警器按下", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second);
			}
			else
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 手报按下报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second);
			}
			
		}
		else if(pcfas_entry->detector_class[0] == CabinClassID)
		{
			if(pcfas_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfws.atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
		}
		
		SetTextValue(monitor_inform_screen_id, 49, baojingneirong); // 刷新第一条报警内容
			
		uint8_t temp_sequence_count = 0;
		
		// 剩下五个区域滚动显示
		for (uint8_t i = 1; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fire_alarm_start_index + i;
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfas_entry->self_bottom_point &&
				pcfas_entry->detector_class[data_index] == PackClassID &&
				pcfas_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 fire display uses "第3回路 X号".
				FormatRS485DetectFireAlarmLine(baojingneirong, temp_sequence_count, pcfas_entry, data_index);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == PackClassID)
			{
				
				if(pcfas_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == LinkageClassID)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 手报按下报警", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == CabinClassID)
			{
				if(pcfas_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 38 + i); //(画面ID,控件ID）
			}
			SetTextValue(monitor_inform_screen_id, 49 + i, baojingneirong); // 刷新报警内容
		}
	}
}

static void InternalScreenShowAllFireAlarm_Plus(PackCabinFireAlarmStorage *pcfas_entry, uint8_t fresh_page_flag)
{
	// 火警监控
	if(pcfas_entry->self_bottom_point == 0) // 
	{
		if(pcfas_entry->point_history_len != 0)
		{
			pcfas_entry->point_history_len = 0;
			clearTextValue(monitor_inform_screen_id , 50);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 51);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 52);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 53);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 54);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 55);//(画面ID,控件ID)
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中.     ");break;   //刷新报警内容
			case 20:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中..    ");break;   //刷新报警内容
			case 30:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中...   ");break;   //刷新报警内容
			case 40:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中....  ");break;   //刷新报警内容
			case 50:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中..... ");break;   //刷新报警内容
			case 60:SetTextValue(monitor_inform_screen_id, 49,"火警系统运行中......");break;   //刷新报警内容
		}
	}
	else if(pcfas_entry->self_bottom_point != pcfas_entry->point_history_len || fresh_page_flag == 1)
	{
		pcfas_entry->point_history_len = pcfas_entry->self_bottom_point;
		
		uint8_t baojingneirong[64]; // XR5000_LOOP3_CHANGE_20260726: Loop 3 display text needs more room.

		fire_alarm_state = 1; // 点亮火警指示灯
		
		uint8_t temp_sequence_count = 0;
		
		// 剩下五个区域滚动显示
		for (uint8_t i = 0; i < Alarm_Show_Zone; i++) {
			uint8_t data_index = fire_alarm_start_index + i;
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < pcfas_entry->self_bottom_point &&
				pcfas_entry->detector_class[data_index] == PackClassID &&
				pcfas_entry->da[data_index].cluster_id == RS485_DETECT_FLASH_ID)
			{
				// XR5000_LOOP3_CHANGE_20260726: Loop 3 fire display uses "第3回路 X号".
				FormatRS485DetectFireAlarmLine(baojingneirong, temp_sequence_count, pcfas_entry, data_index);
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == PackClassID)
			{
				
				if(pcfas_entry->alarm_type[data_index] == Temperature)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
				else if(pcfas.alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", temp_sequence_count,
					pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
					pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
					pcfas_entry->da[data_index].cluster_id, pcfas_entry->da[data_index].pack_id);
				}
			}
			else if(data_index < pcfas_entry->self_bottom_point && pcfas_entry->detector_class[data_index] == LinkageClassID)
			{
				sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 手报按下报警", temp_sequence_count,
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
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Smoke)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Carbon)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
				else if(pcfas_entry->alarm_type[data_index] == Hydrogen)
				{
					sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
						pcfas_entry->atr[data_index].years, pcfas_entry->atr[data_index].months, pcfas_entry->atr[data_index].days,
						pcfas_entry->atr[data_index].hours, pcfas_entry->atr[data_index].minute, pcfas_entry->atr[data_index].second,
						pcfas_entry->da[data_index].cabin_id);
				}
			}
			else
			{
				baojingneirong[0] = 0;
//				clearTextValue(1 , 38 + i); //(画面ID,控件ID）
			}
			SetTextValue(monitor_inform_screen_id, 49 + i, baojingneirong); // 刷新报警内容
		}
	}
}

static void CreatNewFireExtinguishRecord(
	FireExtinguishDeviceActionSave *fedas_entry, // 默认赋值的结构体
	FireExtinguishDeviceActionSave *copy_fedas,  // 默认赋值的结构体
	uint8_t copy_dedas_offset,
	FireExtinguishDeviceActionType state, 
	uint16_t state_switch_delay             // 状态切换延时 
)
{
	// 获取一下RTC时间
	getBM8563TimeToSystemTime();
	// 记录创建时间
	fedas_entry->atr[fedas_entry->self_point_len].years  = years + 2000;
	fedas_entry->atr[fedas_entry->self_point_len].months = months;
	fedas_entry->atr[fedas_entry->self_point_len].days   = days;
	fedas_entry->atr[fedas_entry->self_point_len].hours  = hours;
	fedas_entry->atr[fedas_entry->self_point_len].minute = minutes;
	
	fedas_entry->atr[fedas_entry->self_point_len].second = secs;
	
	// 复制簇PACK/仓ID
	fedas_entry->cabin_id[fedas_entry->self_point_len]   = copy_fedas->cabin_id[copy_dedas_offset];
	fedas_entry->cluster_id[fedas_entry->self_point_len] = copy_fedas->cluster_id[copy_dedas_offset];
	fedas_entry->pack_id[fedas_entry->self_point_len]    = copy_fedas->pack_id[copy_dedas_offset];
	// 状态赋值
	fedas_entry->fed_action[fedas_entry->self_point_len] = state;
	fedas_entry->countdown_val[fedas_entry->self_point_len] = state_switch_delay; // 持续时长state_switch_delay秒
	// 记录启动时间
	fedas_entry->start_cntd_time[fedas_entry->self_point_len] = baojingjishi; 
	// 记录一下当前时间	
	fedas_entry->curr_cntd_time[fedas_entry->self_point_len]  = fedas_entry->start_cntd_time[fedas_entry->self_point_len]; 
	// 指向下一个位置
	fedas_entry->self_point_len++; 
}

static void FireExtinguishDevice1HandStart(FireExtinguishDeviceActionSave *fedas_entry)
{
	uint8_t out_fire_start_flag = 0;
	start_stop_key_state = 1; // 手动启动

	// 记录气灭启动按键按下 
	BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_ST_PRESS, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
	for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
	{
		if(fedas_entry->cabin_id[j] == 0 && fedas_entry->cluster_id[j] != OUTFIRE_CLUSTER_ID && fedas_entry->cluster_id[j] != 0)
		{
			if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_CAN_RESTART)
			{
				// 该状态 显示内容不变 还是显示灭火装置启动停止
				fedas_entry->fed_action[j] = FIRE_EXTINGUISH_RESTART_FINISH; // 标记为已经重新启动
				// 创建新记录 重新启动
				CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_START_SPRAY_DELAY, 30);
				// 记录到FLASH中 灭火装置再次启动
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRESTART_AGAIN, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				// 记录到FLASH中 灭火装置1第一次启动倒计时
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
			else if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_MODE_JUDGEMENT)
			{
				fedas_entry->start_cntd_time[j] = baojingjishi; // 记录启动时间
				fedas_entry->curr_cntd_time[j]  = baojingjishi; // 记录一下当前时间
				fedas_entry->fed_action[j]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 

				// 记录到FLASH中 灭火装置1第一次启动倒计时
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
		}
		
	}
	if(out_fire_start_flag == 0) // 如果本次按下没有启动任何簇
	{
		uint8_t index = fedas_entry->self_point_len;

		// 获取一下RTC时间
		getBM8563TimeToSystemTime();
		
		// 记录创建时间
		fedas_entry->atr[index].years  = years + 2000;
		fedas_entry->atr[index].months = months;
		fedas_entry->atr[index].days   = days;
		fedas_entry->atr[index].hours  = hours;
		fedas_entry->atr[index].minute = minutes;
		
		fedas_entry->atr[index].second = secs;
		
		// 记录仓ID
		fedas_entry->cabin_id[index]   = 0;
		fedas_entry->cluster_id[index] = 1;
		fedas_entry->pack_id[index]    = 1;
		
		// 创建一条启动记录
		fedas_entry->start_cntd_time[index] = baojingjishi; // 记录启动时间
		fedas_entry->curr_cntd_time[index]  = baojingjishi; // 记录一下当前时间
		fedas_entry->fed_action[index]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 
		
		fedas_entry->countdown_val[index] = 30; // 持续时长state_switch_delay秒
		
		fedas_entry->self_point_len++;
		
		FireAlarmRelayCtrl(JDQ_ON);
		ForeWarmRelayCtrl(JDQ_ON);
		beep_fire_ctrl |= 0xF0;  // 真 火警 长鸣
	}
}

static void FireExtinguishDevice2HandStart(FireExtinguishDeviceActionSave *fedas_entry)
{
	uint8_t out_fire_start_flag = 0;
	start_stop_key_state = 1; // 手动启动
	
	// 记录气灭启动按键按下 
	BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_ST_PRESS, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
	for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
	{
		if(fedas_entry->cabin_id[j] != 0  && fedas_entry->cluster_id[j] == 0)
		{
			if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_CAN_RESTART)
			{
				// 该状态 显示内容不变 还是显示灭火装置启动停止
				fedas_entry->fed_action[j] = FIRE_EXTINGUISH_RESTART_FINISH; // 标记为已经重新启动
				// 创建新记录 重新启动
				CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_START_SPRAY_DELAY, 30);
				// 记录到FLASH中 灭火装置再次启动
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRESTART_AGAIN, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				// 记录到FLASH中 灭火装置1第一次启动倒计时
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
			else if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_MODE_JUDGEMENT)
			{
				fedas_entry->start_cntd_time[j] = baojingjishi; // 记录启动时间
				fedas_entry->curr_cntd_time[j]  = baojingjishi; // 记录一下当前时间
				fedas_entry->fed_action[j]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 

				// 记录到FLASH中 灭火装置1第一次启动倒计时
				BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
				out_fire_start_flag = 1;
			}
		}
	}
	if(out_fire_start_flag == 0) // 如果本次按下没有任何仓启动 即手动强启仓喷
	{
		uint8_t index = fedas_entry->self_point_len;

		// 获取一下RTC时间
		getBM8563TimeToSystemTime();
		
		// 记录创建时间
		fedas_entry->atr[index].years  = years + 2000;
		fedas_entry->atr[index].months = months;
		fedas_entry->atr[index].days   = days;
		fedas_entry->atr[index].hours  = hours;
		fedas_entry->atr[index].minute = minutes;
		
		fedas_entry->atr[index].second = secs;
		
		// 记录仓ID
		fedas_entry->cabin_id[index]   = 1;
		fedas_entry->cluster_id[index] = 0;
		fedas_entry->pack_id[index]    = 0;
		
		// 创建一条启动记录
		fedas_entry->start_cntd_time[index] = baojingjishi; // 记录启动时间
		fedas_entry->curr_cntd_time[index]  = baojingjishi; // 记录一下当前时间
		fedas_entry->fed_action[index]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; 
		
		fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // 持续时长state_switch_delay秒
		
		fedas_entry->self_point_len++;
		
		
		FireAlarmRelayCtrl(JDQ_ON);
		ForeWarmRelayCtrl(JDQ_ON);
		beep_fire_ctrl |= 0xF0;  // 真 火警 长鸣
	}

}

static void FireExtinguishDeviceStateUpdate(FireExtinguishDeviceActionSave *fedas_entry, PackAlarmStorage *pas_entry)
{
	// 如果有探测器火警
	if(pas_pointer != last_pas_len && pas_pointer > 0) // 如果有新的报警增加
	{
		for(uint8_t i = last_pas_len; i < pas_pointer; i++) // 从上一次记录的位置开始给结构体进行赋值
		{
			if(pas_entry[i].cluster_id == LINKAGE_CLUSTER_ID) // 如果是外联设备不计入判断
			{
				continue;
			}
			fedas_entry->atr[fedas_entry->self_point_len]        = pas_entry[i].atr;
			fedas_entry->cabin_id[fedas_entry->self_point_len]   = pas_entry[i].cabin_id;
			fedas_entry->cluster_id[fedas_entry->self_point_len] = pas_entry[i].cluster_id;
			fedas_entry->pack_id[fedas_entry->self_point_len]    = pas_entry[i].pack_id;
			fedas_entry->fed_action[fedas_entry->self_point_len] = pas_entry[i].lunch_state; // 获取当前启动状态
			fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // 启动延时倒计时30秒

			fedas_entry->self_point_len++;
		}
		last_pas_len = pas_pointer; 
	}
	// 如果反馈1触发
	if(getFeedBack1State() == 0x0F)
	{
		// 喷洒声信号
		beep_spray_feedback_ctrl = 1;
		// 修改状态 确保只运行一次
		setDealFeedBack1State();
		// 点亮分区1反馈灯
		Part1FeedbackLedCtrl(LED_ON);
		// 记录到FLASH中 反馈一动作
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_FEEDBACK1, LINKAGE_CLUSTER_ID, FEEDBK1_Package_ID);
		StorageEvent_LogFeedback(FEEDBK1_Package_ID, DEV_TYPE_CONTROL_DEV, 0); /* 黑匣子:反馈1记录 */
		FecbusReport_Feedback(FEEDBK1_Package_ID, DEV_TYPE_CONTROL_DEV, 0);    /* FECbus:反馈1上报 */

		// 创建一条新纪录
		// 记录创建时间
		fedas_entry->atr[fedas_entry->self_point_len].years  = years + 2000;
		fedas_entry->atr[fedas_entry->self_point_len].months = months;
		fedas_entry->atr[fedas_entry->self_point_len].days   = days;
		fedas_entry->atr[fedas_entry->self_point_len].hours  = hours;
		fedas_entry->atr[fedas_entry->self_point_len].minute = minutes;

		// 新增记录秒
		fedas_entry->atr[fedas_entry->self_point_len].second = secs;
		
		fedas_entry->cabin_id[fedas_entry->self_point_len]   = 0;
		fedas_entry->cluster_id[fedas_entry->self_point_len] = LINKAGE_CLUSTER_ID;
		fedas_entry->pack_id[fedas_entry->self_point_len]    = FEEDBK1_Package_ID;
		fedas_entry->fed_action[fedas_entry->self_point_len] = FEEDBACK_1_PRESS; // 获取当前启动状态
		fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // 
			
		fedas_entry->self_point_len++;
	}
	
	if(getOutFireKeyalue() == KEY12_PART1_STOP)
	{
		// 清空键值
		clearOutFireKeyValue();
		start_stop_key_state = 2; // 手动停止
		// 获取一下RTC时间
		getBM8563TimeToSystemTime();
		// 记录气灭停止按键按下
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SP_PRESS, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
		// 遍历所有数组
		for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
		{
			if(fedas_entry->cabin_id[j] == 0  && fedas_entry->cluster_id[j] != OUTFIRE_CLUSTER_ID && fedas_entry->cluster_id[j] != 0)
			{
				// 将所有正在启动倒计时的灭火装置全部停掉
				if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_START_SPRAY_DELAY)
				{
					// 这个状态显示为 时间 启动倒计时多少秒
					fedas_entry->fed_action[j] = FIRE_EXTINGUISH_FORCE_STOP; // 标记为强制结束
					// 将本状态赋值为可重新启动 显示在屏幕上是 停止启动
					CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_CAN_RESTART, 0);  //  
					// 存入FLASH 新建一条记录 记录灭火装置启动停止
					BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_STOP, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				}
			}
			
		}
	}
	else if(getOutFireKeyalue() == KEY11_PART1_SOUNDLT)
	{
		// 清空键值
		clearOutFireKeyValue();
		// 点亮声光LED 
		Part1SoundLightLedCtrl(LED_ON);
		// 记录声光按键按下
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SL_PRESS, OUTFIRE_CLUSTER_ID, SoundLt_Package_ID);
		// 启动声光
		SoundLightRelayCtrl(JDQ_ON);
	}
	
	if(getOutFireKeyalue() == KEY14_PART2_STOP)
	{
		start_stop_key_state = 2; // 手动停止
		
		// 清空键值
		clearOutFireKeyValue();
		// 获取一下RTC时间
		getBM8563TimeToSystemTime();
		// 记录气灭停止按键按下
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SP_PRESS, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
		// 遍历所有数组
		for(uint8_t j = 0; j < fedas_entry->self_point_len; j++)
		{
			if(fedas_entry->cabin_id[j] != 0  && fedas_entry->cluster_id[j] == 0)
			{
				// 将所有正在启动倒计时的灭火装置全部停掉
				if(fedas_entry->fed_action[j] == FIRE_EXTINGUISH_START_SPRAY_DELAY)
				{
					// 这个状态显示为 时间 启动倒计时多少秒
					fedas_entry->fed_action[j] = FIRE_EXTINGUISH_FORCE_STOP; // 标记为强制结束
					// 将本状态赋值为可重新启动 显示在屏幕上是 停止启动
					CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, j, FIRE_EXTINGUISH_CAN_RESTART, 0);  //  
					// 存入FLASH 新建一条记录 记录灭火装置启动停止
					BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_STOP, OUTFIRE_CLUSTER_ID, OUTFIRE_PACKAGE_ID);
				}
			}
			
		}
		
	}
	else if(getOutFireKeyalue() == KEY13_PART2_SOUNDLT)
	{
		// 点亮声光LED 
		Part2SoundLightLedCtrl(LED_ON);
		// 记录声光按键按下
		BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_SL_PRESS, OUTFIRE_CLUSTER_ID, SoundLt_Package_ID);
		// 启动声光
		SoundLightRelayCtrl(JDQ_ON);
		clearOutFireKeyValue();
	}

	for(uint8_t i = 0; i < fedas_entry->self_point_len; i++) // 更新所有报警的倒计时 
	{
		if(fedas_entry->cluster_id[i] == LINKAGE_CLUSTER_ID) // 如果簇ID等于联动设备动作跳过本次循环
		{
			continue;
		}
		fedas_entry->curr_cntd_time[i] = baojingjishi; // 获取一下当前时间
		switch(fedas_entry->fed_action[i])
		{
			case FIRE_EXTINGUISH_MODE_JUDGEMENT: { // 判断是手动还是自动
				if(fedas_entry->cluster_id[i] == 0)
				{
					if(getPart2HandAutoState() == KEY_AUTO)
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 记录启动时间
						fedas_entry->curr_cntd_time[i]  = baojingjishi; // 记录一下当前时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; // 自动启动倒计时
						
						// 启动声光 后续要修改此处启动分区二的独立声光
						SoundLightRelayCtrl(JDQ_ON);
						// 分区2声光启动LED
						Part2SoundLightLedCtrl(LED_ON);
						
						// 记录 灭火装置启动倒计时 存入FLASH 参数: 气灭存储分区 灭火装置第一次启动 簇ID 灭火装置2ID
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIRE_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					if(getPart1HandAutoState() == KEY_AUTO) // 
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 记录启动时间
						fedas_entry->curr_cntd_time[i]  = baojingjishi; // 记录一下当前时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_START_SPRAY_DELAY; // 自动启动倒计时
						
						// 启动声光 后续须修改为分区一独立声光
						SoundLightRelayCtrl(JDQ_ON);
						
						// 点亮声光LED 
						Part1SoundLightLedCtrl(LED_ON);
						
						// 记录 灭火装置启动倒计时 存入FLASH
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_1_START_DELAY, OUTFIRE_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_START_SPRAY_DELAY: { // 灭火装置第1次启动倒计时
				// 给BMS上传的状态
				outfire_spray_state = 1;
			
				if(fedas_entry->cluster_id[i] == 0)
				{
					// 该状态位置1后不在判断外联设备掉线
					mhqdbiaozhi = 1;
					// 点亮 分区2喷洒延时 
					Part2StartDelayLedCtrl(LED_ON);
					// 点亮 分区2启动LED
					Part2StartLedCtrl(LED_ON);
					// 启动 分区2声光
					SoundLightRelayCtrl(JDQ_ON);
					// 分区2声光启动LED
					Part2SoundLightLedCtrl(LED_ON);
					
					// 如果30秒倒计时结束了
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FED_START_SPRAY_DELAY_FINISH_FLAG; // 标记为第一状态完成
						// 关闭启动延时LED
						Part2StartDelayLedCtrl(LED_OFF);
						// 点亮喷洒LED
						Part2SprayLedCtrl(LED_ON);
						
						// 灭火装置2 第一次启动
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_1, OUTFIR1_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
						// 启动倒计时结束后 打开钢瓶阀2
						OutFire2RelayCtrl(JDQ_ON);
						// 打开放气勿入
						DefauleRelayCtrl(JDQ_ON);
						// 创建钢瓶启动记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_CYLINDEF_2_OPENED, 0);  // 钢瓶1电磁阀打开 
						// 创建开始喷放记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_FIRST_SPRAY_START, 15); // 持续喷放15秒
						// 已经启动喷放
						outfire_spray_state = 2; 
					}
				}
				else
				{
					// 该状态位置1后不在判断外联设备掉线
					mhqdbiaozhi = 1;
					// 点亮喷洒延时 
					Part1StartDelayLedCtrl(LED_ON);
					// 点亮 启动LED
					Part1StartLedCtrl(LED_ON);
					// 启动声光
					SoundLightRelayCtrl(JDQ_ON);
					// 点亮声光LED 
					Part1SoundLightLedCtrl(LED_ON);
					
					// 如果30秒倒计时结束了
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FED_START_SPRAY_DELAY_FINISH_FLAG; // 标记为第一状态完成
						// 关闭启动延时LED
						Part1StartDelayLedCtrl(LED_OFF);
						// 点亮喷洒LED
						Part1SprayLedCtrl(LED_ON);
						// 灭火装置1第一次启动
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_1, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
						// 启动倒计时结束后 打开钢瓶阀
						OutFire1RelayCtrl(JDQ_ON);
						// 打开放气勿入
						DefauleRelayCtrl(JDQ_ON);
						// 创建钢瓶启动记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_CYLINDEF_1_OPENED, 0);  // 钢瓶1电磁阀打开 
						// 创建开始喷放记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_FIRST_SPRAY_START, 15); // 持续喷放15秒
						// 已经启动喷放
						outfire_spray_state = 2; 
					}
					// 在启动倒计时一半的时候发送指令打开簇/仓阀
					else if(cluster_solenoid_valve_start_state == 0 && fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= (fedas_entry->countdown_val[i] - 2) )
					{
						// 记录簇阀门开启
						cluster_solenoid_valve_start_state = 1; 
						if(fedas_entry->cabin_id[i] == 0)
						{
							ClusterOrCabinCtrlCmd(fedas_entry->cluster_id[i], CLUSTER_VALVE1, 0xFF);
						}
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_FIRST_SPRAY_START: { // 灭火装置喷放持续时间
				if(fedas_entry->cluster_id[i] == 0)
				{
					Part2StartDelayLedCtrl(LED_ON);
					// 如果喷放持续时间够了 更新状态保存记录
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_FIRST_SPRAY_FINISH; // 第一次持续喷放结束
						
						// 喷放倒计时结束后 关闭钢瓶电磁阀
						OutFire1RelayCtrl(JDQ_OFF);
						
						// 2025/10/28 10:50 暂时在停止喷放时不关闭放弃勿入灯牌
//						// 关闭放气勿入灯牌
//						DefauleRelayCtrl(JDQ_OFF);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_DELAY, 300);
						// 灭火装置1第一次启动 完成 第二次启动倒计时
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_2_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
						
						Part2StartDelayLedCtrl(LED_OFF);
					}
				}
				else
				{
					Part1StartDelayLedCtrl(LED_ON);
					// 如果喷放持续时间够了 更新状态保存记录
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_FIRST_SPRAY_FINISH; // 第一次持续喷放结束
						
						// 喷放倒计时结束后 关闭钢瓶电磁阀
						OutFire1RelayCtrl(JDQ_OFF);
						
						// 2025/10/28 10:50 暂时在停止喷放时不关闭放弃勿入灯牌
//						// 关闭放气勿入灯牌
//						DefauleRelayCtrl(JDQ_OFF);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_DELAY, 300);
						// 灭火装置1第一次启动 完成 第二次启动倒计时
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_2_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
						
						Part1StartDelayLedCtrl(LED_OFF);
					}
				}
				break; 
			}
			case FIRE_EXTINGUISH_SECOND_SPRAY_DELAY: {   // 灭火装置第2次启动倒计时
				
				if(fedas_entry->cluster_id[i] == 0)
				{
					Part2StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FED_SECOND_SPRAY_DELAY_FINISH_FLAG; // 第二次喷放倒计时结束
						// 启动倒计时结束后 打开钢瓶阀
						OutFire2RelayCtrl(JDQ_ON);
						// 打开放气勿入
						DefauleRelayCtrl(JDQ_ON);
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_START, 15);
						// 灭火装置第二次喷放启动
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_2, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
						// 点亮启动延时
						Part1StartDelayLedCtrl(LED_OFF);
					}
				}
				else
				{
					Part1StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FED_SECOND_SPRAY_DELAY_FINISH_FLAG; // 第二次喷放倒计时结束
						// 启动倒计时结束后 打开钢瓶阀
						OutFire1RelayCtrl(JDQ_ON);
						// 打开放气勿入
						DefauleRelayCtrl(JDQ_ON);
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_SECOND_SPRAY_START, 15);
						// 灭火装置第二次喷放启动
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_2, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
						// 点亮启动延时
						Part1StartDelayLedCtrl(LED_OFF);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_SECOND_SPRAY_START: { // 灭火装置第二次持续喷放 
				if(fedas_entry->cluster_id[i] == 0)
				{
					// 延时期间点亮延时
					Part2StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_SECOND_SPRAY_FINISH; // 第二次喷放结束
						// 
						Part2StartDelayLedCtrl(LED_OFF);
						// 喷放倒计时结束后 关闭钢瓶电磁阀
						OutFire2RelayCtrl(JDQ_OFF);
						
//						// 关闭放气勿入灯牌
//						DefauleRelayCtrl(JDQ_OFF);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_DELAY, 300);
						// 灭火装置第三次喷放倒计时
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_3_START_DELAY, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					// 延时期间点亮延时
					Part1StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_SECOND_SPRAY_FINISH; // 第二次喷放结束
						// 
						Part1StartDelayLedCtrl(LED_OFF);
						// 喷放倒计时结束后 关闭钢瓶电磁阀
						OutFire1RelayCtrl(JDQ_OFF);
						
//						// 关闭放气勿入灯牌
//						DefauleRelayCtrl(JDQ_OFF);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_DELAY, 300);
						// 灭火装置第三次喷放倒计时
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_3_START_DELAY, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_THIRD_SPRAY_DELAY: {   // 灭火装置第3次启动倒计时
				if(fedas_entry->cluster_id[i] == 0)
				{
					// 延时期间点亮延时
					Part2StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FED_THIRD_SPRAY_DELAY_FINISH_FLAG; // 第二次喷放结束
						// 启动后关闭延时LED
						Part2StartDelayLedCtrl(LED_OFF);
						// 启动倒计时结束后 打开钢瓶阀
						OutFire2RelayCtrl(JDQ_ON);
						// 打开放气勿入
						DefauleRelayCtrl(JDQ_ON);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_START, 999);
						// 灭火装置1 第三次喷放
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_3, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					// 延时期间点亮延时
					Part1StartDelayLedCtrl(LED_ON);
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FED_THIRD_SPRAY_DELAY_FINISH_FLAG; // 第二次喷放结束
						// 启动后关闭延时LED
						Part1StartDelayLedCtrl(LED_OFF);
						// 启动倒计时结束后 打开钢瓶阀
						OutFire1RelayCtrl(JDQ_ON);
						// 打开放气勿入
						DefauleRelayCtrl(JDQ_ON);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_THIRD_SPRAY_START, 999);
						// 灭火装置1 第三次喷放
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE1OPEN_3, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_THIRD_SPRAY_START: {  // 第三次喷放开始
				if(fedas_entry->cluster_id[i] == 0)
				{
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_THIRD_SPRAY_FINISH; // 第三次喷放结束
						
						// 喷放倒计时结束后 关闭钢瓶电磁阀
						OutFire2RelayCtrl(JDQ_OFF);
						
//						// 关闭放气勿入灯牌
//						DefauleRelayCtrl(JDQ_OFF);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE, 999);
						// 灭火装置1 喷放完成
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_OVER, OUTFIR2_CLUSTER_ID, OUTFIR2_PACKAGE_ID);
					}
				}
				else
				{
					if(fedas_entry->curr_cntd_time[i] - fedas_entry->start_cntd_time[i] >= fedas_entry->countdown_val[i])
					{
						fedas_entry->start_cntd_time[i] = baojingjishi; // 更新启动时间
						fedas_entry->fed_action[i]      = FIRE_EXTINGUISH_THIRD_SPRAY_FINISH; // 第三次喷放结束
						
						// 喷放倒计时结束后 关闭钢瓶电磁阀
						OutFire1RelayCtrl(JDQ_OFF);
						// 关闭放气勿入灯牌
						DefauleRelayCtrl(JDQ_OFF);
						
						// 创建新记录
						CreatNewFireExtinguishRecord(fedas_entry , fedas_entry, i, FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE, 999);
						// 灭火装置1 喷放完成
						BspCommonDataSaveApp(GASER_FLASH_SAVE, OUTFIRE_OVER, OUTFIR1_CLUSTER_ID, OUTFIR1_PACKAGE_ID);
					}
				}
				break;
			}
			case FIRE_EXTINGUISH_FORCE_STOP : { // 如果是强制停止状态 不做处理
				
				
				break;
			}
			case FIRE_EXTINGUISH_CAN_RESTART: { // 如果是可以重启状态 判断按键按下后重新启动
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
		// 打印时间
		send_len = sprintf((char *)debug_buff, "sq:%d time:%d/%d/%d/%d/%d\r\n", i, fedas_entry->atr[i].years, 
									fedas_entry->atr[i].months, fedas_entry->atr[i].days,
									fedas_entry->atr[i].hours, fedas_entry->atr[i].minute);
		DebugSendString(debug_buff, send_len);
		// 打印簇编号
		send_len = sprintf((char *)debug_buff, "sq:%d cb_id:%d pk_id:%d cl_id:%d\r\n", i, fedas_entry->cabin_id[i], fedas_entry->pack_id[i], fedas_entry->cluster_id[i]);
		DebugSendString(debug_buff, send_len);
		// 打印动作
		send_len = sprintf((char *)debug_buff, "sq:%d action_id:%d \r\n", i, fedas_entry->fed_action[i]);
		DebugSendString(debug_buff, send_len);
	}
	
}
#endif



// 灭火装置分区显示控制
static void InternalScreenShowFireExtinguisher(FireExtinguishDeviceActionSave *fedas_entry, uint8_t fresh_page_flag)
{
	if(fedas_entry->self_point_len == 0) // 没有火警
	{
		if(fedas_entry->last_point_len == 255)
		{
			clearTextValue(monitor_inform_screen_id , 2);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 3);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 4);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 5);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 6);//(画面ID,控件ID)
			clearTextValue(monitor_inform_screen_id , 7);//(画面ID,控件ID)
			fedas_entry->last_point_len = 0;
		}
		switch(screen_fresh_num)
		{
			case 10:SetTextValue(monitor_inform_screen_id, 1,"气灭监测运行中.     ");break;   //刷新报警内容
			case 20:SetTextValue(monitor_inform_screen_id, 1,"气灭监测运行中..    ");break;   //刷新报警内容
			case 30:SetTextValue(monitor_inform_screen_id, 1,"气灭监测运行中...   ");break;   //刷新报警内容
			case 40:SetTextValue(monitor_inform_screen_id, 1,"气灭监测运行中....  ");break;   //刷新报警内容
			case 50:SetTextValue(monitor_inform_screen_id, 1,"气灭监测运行中..... ");break;   //刷新报警内容
			case 60:SetTextValue(monitor_inform_screen_id, 1,"气灭监测运行中......");break;   //刷新报警内容
		}
	}
	else if( fresh_page_flag == 1 ) // 如果有火警（任意气体加温度或温度超过二级预警）存在了
	{
		uint8_t baojingneirong[96] = {0}; // XR5000_LOOP3_CHANGE_20260726: Loop 3 gas extinguish text needs more room.
		
		uint8_t temp_sequence_count = 0;
		
		#ifdef FADAS_DEBUG
		printf_fadas_data(fedas_entry);
		#endif
		// 3个区域滚动显示
		for (uint8_t i = 0; i < Out_Fire_Show_Zone; i++) {
			uint8_t data_index = fedas_fresh_point + i;
			
			temp_sequence_count = data_index + 1;
			
			if(data_index < fedas_entry->self_point_len) // 状态机内部判断是包还是仓 不在外部判断做区分了
			{
				if(fedas_entry->cluster_id[data_index] == LINKAGE_CLUSTER_ID) // 如果是联动ID
				{
					if(fedas_entry->fed_action[data_index] == FEEDBACK_1_PRESS)
					{
						sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 反馈1触发", temp_sequence_count,
							fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
							fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second);
					}
					else if(fedas_entry->fed_action[data_index] == FEEDBACK_2_PRESS)
					{
						sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 反馈2触发", temp_sequence_count,
							fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
							fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second);
					}
					SetTextValue(59, 1 + i, baojingneirong);
					continue; // 进入下一次循环
				}
				uint16_t temp_time = fedas_entry->curr_cntd_time[data_index] - fedas_entry->start_cntd_time[data_index];
				if(FormatRS485DetectFireExtinguisherLine(baojingneirong, temp_sequence_count, fedas_entry, data_index, temp_time) == 1)
				{
					// XR5000_LOOP3_CHANGE_20260726: Loop 3 gas extinguish display uses "第3回路 X号".
					SetTextValue(monitor_inform_screen_id, 1 + i, baojingneirong);
					continue;
				}
				switch(fedas_entry->fed_action[data_index])
				{
					case FIRE_EXTINGUISH_MODE_JUDGEMENT:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							if(getPart2HandAutoState() == KEY_MANUAL) // 手动
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 请手动启动灭火装置", temp_sequence_count,
									fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
									fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
									fedas_entry->cabin_id[data_index]);
							}
						}
						else
						{
							if(getPart1HandAutoState() == KEY_MANUAL) // 手动
							{
								sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 请手动启动灭火装置", temp_sequence_count,
									fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
									fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
									fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
							}
						}
						break;
					case FIRE_EXTINGUISH_START_SPRAY_DELAY:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置启动倒计时%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置启动倒计时%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FED_START_SPRAY_DELAY_FINISH_FLAG: // 灭火装置第一次启动倒计时结束
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第1次喷放启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第1次喷放启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						break;
					case FIRE_EXTINGUISH_FIRST_SPRAY_START:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置喷放剩余时间%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置喷放剩余时间%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FIRE_EXTINGUISH_FIRST_SPRAY_FINISH:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第1次喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第1次喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						break;
					case FIRE_EXTINGUISH_SECOND_SPRAY_DELAY:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第2次启动倒计时%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第2次启动倒计时%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						break;
					case FED_SECOND_SPRAY_DELAY_FINISH_FLAG:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第2次喷放启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第2次喷放启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						break;
					case FIRE_EXTINGUISH_SECOND_SPRAY_START:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置喷放剩余时间%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置喷放剩余时间%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FIRE_EXTINGUISH_SECOND_SPRAY_FINISH:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第2次喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第2次喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						

						break;
					case FIRE_EXTINGUISH_THIRD_SPRAY_DELAY:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第3次启动倒计时%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第3次启动倒计时%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						break;
					case FED_THIRD_SPRAY_DELAY_FINISH_FLAG: // 第三次喷放倒计时结束
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第3次喷放启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第3次喷放启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						

						break;
					case FIRE_EXTINGUISH_THIRD_SPRAY_START:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置喷放剩余时间%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index], fedas_entry->countdown_val[data_index] - temp_time);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置喷放剩余时间%d", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  , fedas_entry->countdown_val[data_index] - temp_time);
						}
						

						break;
					case FIRE_EXTINGUISH_THIRD_SPRAY_FINISH:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置第3次喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置第3次喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						

						break;
					case FIRE_EXTINGUISH_ALL_SPRAY_COMPLETE:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置喷放完毕", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]  );
						}
						

						break;
					case FIRE_EXTINGUISH_STARYUP_FINISH_FLAG:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置正在启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置正在启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						

						break;
					case FIRE_EXTINGUISH_CYLINDEF_1_OPENED:
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置1启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置1启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						
					
						break;
					case FIRE_EXTINGUISH_FORCE_STOP: {
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置启动倒计时--", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置启动倒计时--", temp_sequence_count,
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
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置手动停止启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置手动停止启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second, 
								fedas_entry->cluster_id[data_index], fedas_entry->pack_id[data_index]);
						}
						
						break;
					case FEEDBACK_1_PRESS : {
						sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 反馈1触发", temp_sequence_count,
							fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
							fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second);
						break;
					}
					case FIRE_EXTINGUISH_CYLINDEF_2_OPENED: {
						if(fedas_entry->cluster_id[data_index] == 0)
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 火警 灭火装置2启动", temp_sequence_count,
								fedas_entry->atr[data_index].years, fedas_entry->atr[data_index].months, fedas_entry->atr[data_index].days,
								fedas_entry->atr[data_index].hours, fedas_entry->atr[data_index].minute, fedas_entry->atr[data_index].second,
								fedas_entry->cabin_id[data_index]);
						}
						else
						{
							sprintf((char*)baojingneirong, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇pack%d火警 灭火装置1启动", temp_sequence_count,
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
		} // 循环括号
	} // 有报警后每秒刷新的括号
}

void InternalScreenShowDetectorDataCtrlInit(DetectorDataShowCtrl *ddsc_entry)
{
	ddsc_entry->curr_detector_page = 1; // 默认是第一页
	ddsc_entry->last_detector_page = 0; // 默认不显示

	memset(ddsc_entry->detector_offline_fresh_flag, 0xFF, sizeof(ddsc_entry->detector_offline_fresh_flag));
	
	// 探测器在线数组
	memset(ddsc_entry->detect_online_state, 0xFF, sizeof(ddsc_entry->detect_online_state));
	// 屏蔽数组
	memset(ddsc_entry->detect_shield_state, 0xFF, sizeof(ddsc_entry->detect_shield_state));
	
	// 温度值记录
	memset(ddsc_entry->last_temperature,    0xFF, sizeof(ddsc_entry->last_temperature));
	// 温度状态记录
	memset(ddsc_entry->last_temperat_state, 0xFF, sizeof(ddsc_entry->last_temperat_state));
	
	// 烟雾记录
	memset(ddsc_entry->last_smoke_state,    0xFF, sizeof(ddsc_entry->last_smoke_state));
	
	// 一氧化碳浓度记录
	memset(ddsc_entry->last_co_concentrat,  0xFF, sizeof(ddsc_entry->last_co_concentrat));
	// 一氧化碳报警状态
	memset(ddsc_entry->last_co_state,       0xFF, sizeof(ddsc_entry->last_co_state));
	
}

void InternalScreenShowDetectorDataCtrlInit_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry)
{
	ddsc_32p_entry->curr_detector_page = 1; // 默认是第一页

	ddsc_32p_entry->last_detector_page = 0xFF; // 初始化的值
	
	// 默认PACK是0 不初始化也没问题
	ddsc_32p_entry->curr_pack_id = 0; // 
	// 强制刷新标志位
	ddsc_32p_entry->force_fresh_flag = 1;
	
	ddsc_32p_entry->last_temperature = 0xFF; // 历史温度
	ddsc_32p_entry->last_temperat_state = 0xFF; // 历史温度状态
	ddsc_32p_entry->last_smoke_state = 0xFF; // 历史烟雾状态
	ddsc_32p_entry->last_co_concentrat = 0xFF; // 历史一氧化碳浓度
	ddsc_32p_entry->last_co_state = 0xFF; // 历史一氧化碳状态
	
	// 将历史上线数量初始化为255
	memset(ddsc_32p_entry->lat_detector_online_num, 0xFF, sizeof(ddsc_32p_entry->lat_detector_online_num));
}

void Bsp_Screen_Buff_Init(void)
{
	DetectorDataShowCtrl *p = &ddsc;
	DetectorDataShowCtrl_32Pack *ddsc_32p_entry = &ddsc_32p;
	// 经典版本
	InternalScreenShowDetectorDataCtrlInit(p);
	// 1簇32包版本
	InternalScreenShowDetectorDataCtrlInit_32Pack(ddsc_32p_entry);
}

uint8_t detector_online_ctrl_id[11] = {0, 4, 13, 24, 34, 44, 54, 64, 74, 84, 94};
uint8_t temperature_ctrl_id[11] = {0, 9, 18, 28,  38, 48, 58, 68, 78, 88, 98};
uint8_t co_concentrate_ctrl_id[11] = {0, 7, 21, 31, 41, 51, 61, 71, 81, 91, 101};
uint8_t smoke_show_ctrl_id[11] = {0, 11, 22, 32, 42, 52, 62, 72, 82, 92, 102};

static void InternalScreenShowClusterData(DetectorDataShowCtrl *ddsc_entry)
{
	uint8_t temp_screen_id = 54;
	uint8_t curr_page = ddsc_entry->curr_detector_page; // 先备份一次指针
	uint8_t fresh_flag = 0;
	
	if(curr_page !=	ddsc_entry->last_detector_page) // 刷新当前页显示
	{
		uint8_t buff[32] = {0};

		fresh_flag |= 1; // 刷新标志
		sprintf((char *)buff, "第%d簇 PACK灭火控制", curr_page);
		SetTextValue(temp_screen_id, 1, buff);
		sprintf((char *)buff, "第%d簇 簇级灭火控制", curr_page);
		SetTextValue(temp_screen_id, 2, buff);
		
		sprintf((char *)buff, "%d/20", curr_page);
		SetTextValue(temp_screen_id, 105, buff);
	}
	
	if(cu_sxzt[curr_page] == 0 || cu_tcq_sxzt[curr_page] == 0) // 如果没设置上线
	{
		fresh_flag |= 1; // 
		if(ddsc_entry->detector_offline_fresh_flag[curr_page] != cu_tcq_sxzt[curr_page] || fresh_flag) // 如果是第一次启动或页面刷新
		{
			ddsc_entry->detector_offline_fresh_flag[curr_page] = cu_tcq_sxzt[curr_page]; // 表明已经刷新过了
			for(uint8_t i = cu_tcq_sxzt[curr_page] + 1; i < 11; i++)
			{
				SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "未启用");
				// 温度栏颜色设置
				SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, temperature_ctrl_id[i], "--");
				// 一氧化碳浓度
				SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], "--");
				// 烟雾状态
				SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "--");
			}
		}
	}
	if(cu_sxzt[curr_page] != 0) // 如果设置上线
	{
//		if(ddsc_entry->detector_offline_fresh_flag[curr_page] == 0)
//			fresh_flag = 1;
		//ddsc_entry->detector_offline_fresh_flag[curr_page] = 0xFF; // 表明下线后需要重新刷新
		for(uint8_t i = 1; i < cu_tcq_sxzt[curr_page] + 1; i++)
		{
			if(getClusterPackDisconnectCount(curr_page, i) != PackDisconnectCount) // 判断是否掉线
			{ 
				// 在线状态刷新
				if(ddsc_entry->detect_online_state[curr_page][i] != 1 || fresh_flag)
				{
					fresh_flag |= 2; // 从掉线恢复了 重新刷新一次状态
					ddsc_entry->detect_online_state[curr_page][i] = 1; // 把第curr_page簇 第i包赋值为1
					SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0x0400);
					SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "在线");
				} // 在线刷新
				
				// 温度值刷新
				if(ddsc_entry->last_temperature[curr_page][i] != PACK_wendu_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_temperature[curr_page][i] = PACK_wendu_buf[curr_page][i];
					SetTextInt32(temp_screen_id, temperature_ctrl_id[i], PACK_wendu_buf[curr_page][i], 1, 2);//温度显示
				}
				
				// 温度颜色改变
				if(ddsc_entry->last_temperat_state[curr_page][i] != PACK_WDZT_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_temperat_state[curr_page][i] = PACK_WDZT_buf[curr_page][i];
					
					if(ddsc_entry->last_temperat_state[curr_page][i] == 0)
					{
						SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x0400); // 正常
					}
					else if(ddsc_entry->last_temperat_state[curr_page][i] == 1)
					{
						SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0xFB20); // 预警
					}
					else
					{
						SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0xF800); // 火警
					}
				}
				
				// 第curr_page簇 第i包 CO值显示
				uint16_t temp_co_concen = getPackCoConcenValue(curr_page, i);
				if(ddsc_entry->last_smoke_state[curr_page][i] != temp_co_concen || fresh_flag)
				{
					ddsc_entry->last_smoke_state[curr_page][i] = temp_co_concen; // 存储一氧化碳的值
//					SetTextInt32(temp_screen_id, co_concentrate_ctrl_id[i], temp_co_concen, 1, 1); // CO显示
					uint8_t buff[16];
					sprintf((char *)buff, "%dPPM", temp_co_concen);
					SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], buff);
				}
				
				// CO颜色改变
				if(ddsc_entry->last_co_state[curr_page][i] != PACK_COZT_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_temperat_state[curr_page][i] = PACK_COZT_buf[curr_page][i];
					
					if(ddsc_entry->last_temperat_state[curr_page][i] == 0)
					{
						SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x0400); // 正常
					}
					else if(ddsc_entry->last_temperat_state[curr_page][i] == 1)
					{
						SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0xFB20); // 预警
					}
					else if(ddsc_entry->last_temperat_state[curr_page][i] == 2)
					{
						SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0xF800); // 二级预警
					}
				}
				
				if(ddsc_entry->last_smoke_state[curr_page][i] != PACK_YWZT_buf[curr_page][i] || fresh_flag)
				{
					ddsc_entry->last_smoke_state[curr_page][i] = PACK_YWZT_buf[curr_page][i];
					if(PACK_YWZT_buf[curr_page][i] == 0)
					{
						SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x0400); // 正常
						SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "正常");
					}
					else if(PACK_YWZT_buf[curr_page][i] == 1)
					{
						SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0xFB20); // 预警
						SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "报警");
					}
					else if(PACK_YWZT_buf[curr_page][i] == 2)
					{
						SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0xF800); // 二级预警
						SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "报警");
					}
					
				}
				
			} // 在线处理 括号
			else // 如果掉线了
			{ 
				if(ddsc_entry->detect_online_state[i] != 0 || fresh_flag)
				{
					ddsc_entry->detect_online_state[curr_page][i] = 0;
					SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0xFB20);
					SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "掉线");
					
					// 温度栏颜色设置
					SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x8410);
					SetTextValue(temp_screen_id, temperature_ctrl_id[i], "--");

					SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x8410);
					SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], "--");
					
					SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x8410);
					SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "--");
				} // 在线刷新
			} // 掉线判断
		} // for循环括号 遍历当前页上线数量个探测器
		
		if(ddsc_entry->detector_offline_fresh_flag[curr_page] != cu_tcq_sxzt[curr_page])
		{
			for(uint8_t i = cu_tcq_sxzt[curr_page] + 1; i < 11; i++)
			{
				SetControlForeColor(temp_screen_id, detector_online_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, detector_online_ctrl_id[i], "未启用");
				// 温度栏颜色设置
				SetControlForeColor(temp_screen_id, temperature_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, temperature_ctrl_id[i], "--");
				// 一氧化碳浓度
				SetControlForeColor(temp_screen_id, co_concentrate_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, co_concentrate_ctrl_id[i], "--");
				// 烟雾状态
				SetControlForeColor(temp_screen_id, smoke_show_ctrl_id[i], 0x8410);
				SetTextValue(temp_screen_id, smoke_show_ctrl_id[i], "--");
			}
		}// 如果上线数量改变 刷新显示

		
	} // 设置为上线括号
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
	uint8_t curr_page = ddsc_32p_entry->curr_detector_page; // 先备份一次指针
	
	uint8_t temp_force_fresh = 0;
//	DebugPrintf("page:%d\r\n", ddsc_32p_entry->curr_detector_page);
//	DebugPrintf("pack:%d\r\n", ddsc_32p_entry->curr_pack_id);
//	DebugPrintf("flag:%d\r\n", ddsc_32p_entry->force_fresh_flag);

	uint8_t page_change_flage = 0;

	// 强制刷新标志位
	if(ddsc_32p_entry->last_detector_page != curr_page) // 刷新当前页显示
	{
		uint8_t buff[32] = {0};
		sprintf((char *)buff, "第%d簇 PACK灭火控制", curr_page);
		SetTextValue(screen_id_entry, 1, buff); // 刷新簇灭火控制
		sprintf((char *)buff, "%d/3", curr_page);
		SetTextValue(screen_id_entry, 105, buff); // 刷新当前页面显示
		
		page_change_flage = 1;
		ddsc_32p_entry->last_detector_page = curr_page;
	}

	if(screen_id_entry == 61) // 如果不是探测器具体数值查看界面
	{
		// 如果上线状态为0 或者上线数量为0
		if(cu_sxzt[curr_page] == 0 || cu_tcq_sxzt[curr_page] == 0) // 如果没设置上线
		{
			if(ddsc_32p_entry->lat_detector_online_num[curr_page] != cu_tcq_sxzt[curr_page] || page_change_flage == 1) // 如果是第一次启动或页面刷新
			{
				ddsc_32p_entry->lat_detector_online_num[curr_page] = cu_tcq_sxzt[curr_page]; // 表明已经刷新过了
				for(uint8_t i = cu_tcq_sxzt[curr_page] + 1; i < 33; i++)
				{
//					SetControlForeColor(screen_id_entry, detector_online_ctrl_id[i], 0x8410);
					SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "未启用");
					
					ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 2; // 标记为未启用
				}
				
				page_change_flage = 0;
			}
		}
		else
		{
			if(page_change_flage == 1 || ddsc_32p_entry->lat_detector_online_num[curr_page] != cu_tcq_sxzt[curr_page])
			{
				ddsc_32p_entry->lat_detector_online_num[curr_page] = cu_tcq_sxzt[curr_page]; // 表明已经刷新过了
				for(uint8_t i = 1; i < cu_tcq_sxzt[curr_page] + 1; i++)
				{
					if(getClusterPackDisconnectCount(curr_page, i) != PackDisconnectCount) // 判断是否掉线
					{
//						SetControlForeColor(screen_id_entry, detector_online_ctrl_id[i], 0xFB20);
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "在线");
						
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 0; // 标记为恢复
					}
					else
					{
//						SetControlForeColor(screen_id_entry, pack_state_show_ctrl_id[i], 0xFB20);
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "掉线");
						
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 1; // 标记为掉线
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
						ddsc_32p_entry->last_derector_state[temp_index][i] = 1; // 标记为掉线
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i + 1], "掉线");
					}
					else if(ddsc_32p_entry->last_derector_state[temp_index][i] == 1 && getClusterPackDisconnectCount(curr_page, i + 1) != PackDisconnectCount)
					{
						ddsc_32p_entry->last_derector_state[temp_index][i] = 0; // 标记为恢复
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i + 1], "在线");
					}
					else if(ddsc_32p_entry->last_derector_state[temp_index][i] == 2 && getClusterPackDisconnectCount(curr_page, i + 1) != PackDisconnectCount)
					{
						ddsc_32p_entry->last_derector_state[temp_index][i] = 0; // 标记为恢复
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i + 1], "在线");
					}
				}
				
				for(uint8_t j = cu_tcq_sxzt[curr_page]; j < 32; j++)
				{
					if(ddsc_32p_entry->last_derector_state[temp_index][j] != 2)
					{
						ddsc_32p_entry->last_derector_state[temp_index][j] = 2;
						
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[j + 1], "未启用");
					}
				}
				
			}
		}
	}
	else if(screen_id_entry == 62) // 查看具体探测器数值界面
	{
		if(ddsc_32p_entry->force_fresh_flag == 1)
		{
			// 标记为刷新过了
			ddsc_32p_entry->force_fresh_flag = 0;
			temp_force_fresh = 1;
		}

		if(cu_sxzt[curr_page] != 0 && cu_tcq_sxzt[curr_page] >= ddsc_32p_entry->curr_pack_id) // 如果设置上线 且当前查看id小于上线数量
		{
			if(getClusterPackDisconnectCount(curr_page, ddsc_32p_entry->curr_pack_id) != PackDisconnectCount) // 判断是否掉线
			{
				uint8_t buff[32];
				sprintf((char *)buff, "pack%d 状态:在线", ddsc_32p_entry->curr_pack_id);
				SetTextValue(screen_id_entry, 5, buff);
				
				// 温度值刷新
				if(ddsc_32p_entry->last_temperature != PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_temperature = PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id];
					sprintf((char *)buff, "温        度:%d度", ddsc_32p_entry->last_temperature);
					//SetTextInt32(screen_id_entry, 9, PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id], 1, 2);//温度显示
					SetTextValue(screen_id_entry, 9, buff);
				}
				
//				// 温度颜色改变
//				if(ddsc_32p_entry->last_temperat_state != PACK_WDZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] || ddsc_32p_entry->force_fresh_flag == 1)
//				{
//					ddsc_32p_entry->last_temperat_state = PACK_WDZT_buf[curr_page][ddsc_32p_entry->curr_pack_id];
//					
//					if(ddsc_32p_entry->last_temperat_state == 0)
//					{
//						SetControlForeColor(screen_id_entry, 9, 0x0400); // 正常
//					}
//					else if(ddsc_32p_entry->last_temperat_state == 1)
//					{
//						SetControlForeColor(screen_id_entry, 9, 0xFB20); // 预警
//					}
//					else
//					{
//						SetControlForeColor(screen_id_entry, 9, 0xF800); // 火警
//					}
//				}
						
				// 第curr_page簇 第i包 CO值显示
				uint16_t temp_co_concen = getPackCoConcenValue(curr_page, ddsc_32p_entry->curr_pack_id);
				if(ddsc_32p_entry->last_smoke_state != temp_co_concen || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = temp_co_concen; // 存储一氧化碳的值
					sprintf((char *)buff, "一氧化碳:%dPPM", temp_co_concen);
					SetTextValue(screen_id_entry, 17, buff);
				}
				
//				// CO颜色改变
//				if(ddsc_32p_entry->last_co_state != PACK_COZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] || ddsc_32p_entry->force_fresh_flag == 1)
//				{
//					ddsc_32p_entry->last_co_state = PACK_COZT_buf[curr_page][ddsc_32p_entry->curr_pack_id];
//					
//					if(ddsc_32p_entry->last_co_state == 0)
//					{
//						SetControlForeColor(screen_id_entry, 17, 0x0400); // 正常
//					}
//					else if(ddsc_32p_entry->last_co_state == 1)
//					{
//						SetControlForeColor(screen_id_entry, 17, 0xFB20); // 预警
//					}
//					else if(ddsc_32p_entry->last_temperat_state == 2)
//					{
//						SetControlForeColor(screen_id_entry, 17, 0xF800); // 二级预警
//					}
//				}
						
				if(ddsc_32p_entry->last_smoke_state != PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id];
					if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 0)
					{
//						SetControlForeColor(screen_id_entry, 13, 0x0400); // 正常
						SetTextValue(screen_id_entry, 13, "烟        雾:正常");
					}
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 1)
					{
//						SetControlForeColor(screen_id_entry, 13, 0xFB20); // 预警
						SetTextValue(screen_id_entry, 13, "烟        雾:报警");
					} 
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 2)
					{
//						SetControlForeColor(screen_id_entry, 13, 0xF800); // 二级预警
						SetTextValue(screen_id_entry, 13, "烟        雾:报警");
					}
					
				}
			}
			else
			{
				if(temp_force_fresh == 1)
				{
					uint8_t buff[32];
					sprintf((char *)buff, "pack%d 掉线", ddsc_32p_entry->curr_pack_id);
					
//					SetControlForeColor(screen_id_entry, 5, 0xFB20);
					SetTextValue(screen_id_entry, 5, buff);
					
//					SetControlForeColor(screen_id_entry, 13, 0x8410);
					SetTextValue(screen_id_entry, 13, "烟        雾:--");
					
//					SetControlForeColor(screen_id_entry, 17, 0x8410);
					SetTextValue(screen_id_entry, 17, "一氧化碳:--");
					
//					SetControlForeColor(screen_id_entry, 17, 0x8410);
					SetTextValue(screen_id_entry, 9, "温        度:--");
					
				}
//				
//				uint8_t buff[32];
//				sprintf((char *)buff, "pack%d 掉线", ddsc_32p_entry->curr_pack_id);
//				SetTextValue(screen_id_entry, 5, buff);
//				SetTextValue(screen_id_entry, 13, "烟        雾:--");
//				SetTextValue(screen_id_entry, 17, "一氧化碳:--");
//				SetTextValue(screen_id_entry, 9, "温        度:--");
				
			}
		}
		else
		{
			if(temp_force_fresh == 1)
			{
				uint8_t buff[32];
				sprintf((char *)buff, "pack%d 状态:未启用", ddsc_32p_entry->curr_pack_id);
//				DebugSendString(buff, sizeof(buff));
				//					SetControlForeColor(screen_id_entry, 5, 0xFB20);
				SetTextValue(screen_id_entry, 5, buff);

				//					SetControlForeColor(screen_id_entry, 13, 0x8410);
				SetTextValue(screen_id_entry, 13, "烟        雾:--");

				//					SetControlForeColor(screen_id_entry, 17, 0x8410);
				SetTextValue(screen_id_entry, 17, "一氧化碳:--");

				//					SetControlForeColor(screen_id_entry, 17, 0x8410);
				SetTextValue(screen_id_entry, 9, "温        度:--");

			}
			
//			uint8_t buff[32];
//			sprintf((char *)buff, "pack%d 未启用", ddsc_32p_entry->curr_pack_id);
//			SetTextValue(screen_id_entry, 5, buff);
//			SetTextValue(screen_id_entry, 13, "烟        雾:--");
//			SetTextValue(screen_id_entry, 17, "一氧化碳:--");
//			SetTextValue(screen_id_entry, 9, "温        度:--");
		}
		
	}
	
//	uint8_t buff[64];
//	
//	sprintf((char *)buff, "sc_id:%d pg_id:%d pid: %d tc:%d \r\n", screen_id_entry, curr_page, ddsc_32p_entry->curr_pack_id, cu_tcq_sxzt[curr_page]);
//	DebugSendString(buff, sizeof(buff));
}

// 在声明时直接初始化为全0xFF
uint8_t last_pack_online_buff_state[33] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF  // 总共33个0xFF
};

static void InternalScreenShowClusterData_32Pack_Plus(uint16_t screen_id_entry, DetectorDataShowCtrl_32Pack *ddsc_32p_entry)
{
	uint8_t curr_page = ddsc_32p_entry->curr_detector_page; // 先备份一次指针
	
	uint8_t temp_force_fresh = 0;

	uint8_t page_change_flage = 0;

	// 强制刷新标志位
	if(ddsc_32p_entry->last_detector_page != curr_page) // 刷新当前页显示
	{
		uint8_t buff[32] = {0};
		sprintf((char *)buff, "第%d簇 PACK灭火控制", curr_page);
		SetTextValue(screen_id_entry, 1, buff); // 刷新簇灭火控制
		sprintf((char *)buff, "%d/3", curr_page);
		SetTextValue(screen_id_entry, 105, buff); // 刷新当前页面显示
		
		page_change_flage = 1;
		ddsc_32p_entry->last_detector_page = curr_page;
	}

	if(screen_id_entry == 61) // 如果不是探测器具体数值查看界面
	{
		for(uint8_t i = 1; i < 33; i++)
		{
			if((pack_online_buff[curr_page][i] != last_pack_online_buff_state[i] && pack_online_buff[curr_page][i] == 0) || 
					page_change_flage == 1) // 刷新一次
			{
				last_pack_online_buff_state[i] = pack_online_buff[curr_page][i]; // 更新抑制
				
				SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "未启用");
				ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 2; // 标记为未启用
			}
			else if(pack_online_buff[curr_page][i] == 1) // 如果设置为上线了
			{
				if(getClusterPackDisconnectCount(curr_page, i) != PackDisconnectCount) // 判断是否掉线
				{
					if(page_change_flage == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] != 0)
					{
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "在线");
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 0; // 标记为恢复
					}
				}
				else
				{
					if(page_change_flage == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] != 1)
					{
						SetTextValue(screen_id_entry, pack_state_show_ctrl_id[i], "掉线");
						ddsc_32p_entry->last_derector_state[curr_page - 1][i - 1] = 1; // 标记为掉线
					}
					
				}
			}
		} // 循环刷新状态
	} // screen id 61 
	else if(screen_id_entry == 62) // 查看具体探测器数值界面
	{
		if(ddsc_32p_entry->force_fresh_flag == 1)
		{
			// 标记为刷新过了
			ddsc_32p_entry->force_fresh_flag = 0;
			temp_force_fresh = 1;
		}
		
		uint8_t check_pack_id = ddsc_32p_entry->curr_pack_id;
		
		if(pack_online_buff[curr_page][check_pack_id] == 1) // 如果设置上线
		{
			if(getClusterPackDisconnectCount(curr_page, check_pack_id) != PackDisconnectCount)
			{
				uint8_t buff[32];
				if(ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] != 0 || temp_force_fresh == 1)
				{
					sprintf((char *)buff, "pack%d 状态:在线", check_pack_id);
					SetTextValue(screen_id_entry, 5, buff);
					ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] = 0; // 赋值为0
				}

				// 温度值刷新
				if(ddsc_32p_entry->last_temperature != PACK_wendu_buf[curr_page][check_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_temperature = PACK_wendu_buf[curr_page][check_pack_id];
					sprintf((char *)buff, "温        度:%d度", ddsc_32p_entry->last_temperature);
					//SetTextInt32(screen_id_entry, 9, PACK_wendu_buf[curr_page][ddsc_32p_entry->curr_pack_id], 1, 2);//温度显示
					SetTextValue(screen_id_entry, 9, buff);
				}
						
				// 第curr_page簇 第i包 CO值显示
				uint16_t temp_co_concen = getPackCoConcenValue(curr_page, check_pack_id);
				if(ddsc_32p_entry->last_smoke_state != temp_co_concen || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = temp_co_concen; // 存储一氧化碳的值
					sprintf((char *)buff, "一氧化碳:%dPPM", temp_co_concen);
					SetTextValue(screen_id_entry, 17, buff);
				}

				if(ddsc_32p_entry->last_smoke_state != PACK_YWZT_buf[curr_page][check_pack_id] || temp_force_fresh == 1)
				{
					ddsc_32p_entry->last_smoke_state = PACK_YWZT_buf[curr_page][check_pack_id];
					if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 0)
					{
						SetTextValue(screen_id_entry, 13, "烟        雾:正常");
					}
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 1)
					{
						SetTextValue(screen_id_entry, 13, "烟        雾:报警");
					} 
					else if(PACK_YWZT_buf[curr_page][ddsc_32p_entry->curr_pack_id] == 2)
					{
						SetTextValue(screen_id_entry, 13, "烟        雾:报警");
					}
				}
			}
			else // 掉线了
			{
				if(temp_force_fresh == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] != 1)
				{
					uint8_t buff[32];
					sprintf((char *)buff, "pack%d 状态:掉线", ddsc_32p_entry->curr_pack_id);

					SetTextValue(screen_id_entry, 5, buff);

					SetTextValue(screen_id_entry, 13, "烟        雾:--");

					SetTextValue(screen_id_entry, 17, "一氧化碳:--");

					SetTextValue(screen_id_entry, 9, "温        度:--");
					
					ddsc_32p_entry->last_derector_state[curr_page - 1][check_pack_id - 1] = 1;
				}
			}
		}
		else // 没设置上线
		{
			if(temp_force_fresh == 1 || ddsc_32p_entry->last_derector_state[curr_page - 1][ddsc_32p_entry->curr_pack_id - 1] != 2)
			{
				uint8_t buff[32];
				sprintf((char *)buff, "pack%d 状态:未启用", ddsc_32p_entry->curr_pack_id);

				SetTextValue(screen_id_entry, 5, buff);

				SetTextValue(screen_id_entry, 13, "烟        雾:--");

				SetTextValue(screen_id_entry, 17, "一氧化碳:--");

				SetTextValue(screen_id_entry, 9, "温        度:--");
				
				ddsc_32p_entry->last_derector_state[curr_page - 1][ddsc_32p_entry->curr_pack_id - 1] = 2;
			}
		}

	} // screen id 62
}

const char sensor_str1[] = "光学烟雾";
const char sensor_str2[] = "甲烷";
const char sensor_str3[] = "氢气";
const char sensor_str4[] = "VOC";
//const char sensor_str5[] = "一氧化碳";
const char sensor_str5[] = "CO";
const char sensor_str6[] = "温度";
const char sensor_str7[] = "传感器启用状态:";

const char *sensor_str[] = {
	sensor_str1, // 光学烟雾
	sensor_str2, // 甲烷  
	sensor_str3, // 氢气
	sensor_str4, // VOC
	sensor_str5, // 一氧化碳
	sensor_str6, // 温度
	sensor_str7  // 传感器启用状态
};

static void InternalScreenShowCabinDate(CabinDataShowCtrl_t *cabin_dsc_entry)
{
	uint8_t temp_screen_id = 64;
	// 用一个变量存储值，修改的时候方便
	uint8_t temp_cabin_id = cabin_dsc_entry->curr_cabin_id;

	if(cang_sxzt[temp_cabin_id] != 0)
	{
		if(cang_sxzt[temp_cabin_id] == 1 || cabin_dsc_entry->force_fresh_flag == 1) // 如果启用
		{
			if(Cang_zx_buf[temp_cabin_id] == CabinDisconnectCount)
			{
				uint8_t temp_buff[32] = {0};
				sprintf((char *)temp_buff,"探测器%d状态:掉线", temp_cabin_id);
				SetTextValue(temp_screen_id, 2, temp_buff);
				SetTextValue(temp_screen_id, 3, "探测器型号:--");
				SetTextValue(temp_screen_id, 4, "传感器启用状态:--");
				SetTextValue(temp_screen_id, 5, "温度:--");
				SetTextValue(temp_screen_id, 6, "烟雾状态:--");
				SetTextValue(temp_screen_id, 7, "一氧化碳浓度:--");
				SetTextValue(temp_screen_id, 8, "氢气浓度:--");
			}
			else
			{
				uint8_t temp_buff[32] = {0};
				sprintf((char *)temp_buff,"探测器%d状态:在线", temp_cabin_id);
				SetTextValue(temp_screen_id, 2, temp_buff);
				
				// 显示探测器型号
				if(cabin_dsc_entry->last_detector_model != Cang_TCQXH_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					switch(Cang_TCQXH_buf[temp_cabin_id])
					{
						case 1: {
							SetTextValue(temp_screen_id, 3, "探测器型号:XR805-V2.0");
							break;
						}
						case 2: {
							SetTextValue(temp_screen_id, 3, "探测器型号:XR805-EXD");
							break;
						}
						case 3: {
							SetTextValue(temp_screen_id, 3, "探测器型号:XR805-EXi");
							break;
						}
						default:
							SetTextValue(temp_screen_id, 3, "探测器型号:--");
							break;
					}
					
					cabin_dsc_entry->last_detector_model = Cang_TCQXH_buf[temp_cabin_id];
				}
				
				if(cabin_dsc_entry->last_sensor_mode != Cang_CGQQY_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					if(Cang_CGQQY_buf[temp_cabin_id] == 0)
					{
						SetTextValue(temp_screen_id, 4, "传感器启用状态:无传感器启动");
					}
					else
					{
						uint8_t temp = Cang_CGQQY_buf[temp_cabin_id];
						uint8_t temp_buff[128] = {0};
						
						uint8_t first_sensor = 1;  // 标记是否是第一个传感器
						
						uint8_t pos = 0;
						
						pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
//						SetTextValue(temp_screen_id, 4, temp_buff);
						for(uint8_t i = 0; i < 6; i++)
						{
							if( (temp >> i) & 0x01 )
							{
								// 如果不是第一个，添加分隔符
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
				} // 传感器启用状态判断
				
				if(cabin_dsc_entry->last_temperat_value != Cang_wendu_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					uint8_t temp_buff[16];
					cabin_dsc_entry->last_temperat_value = Cang_wendu_buf[temp_cabin_id];
					sprintf((char *)temp_buff, "温度:%d度", Cang_wendu_buf[temp_cabin_id]);
					SetTextValue(temp_screen_id, 5, temp_buff);
				} // 温度值刷新
				
				if(cabin_dsc_entry->last_smoke_state != Cang_YWZT_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					if(Cang_YWZT_buf[temp_cabin_id] == 0)
					{
						SetTextValue(temp_screen_id, 6, "烟雾状态:正常");
					}
					else
					{
						SetTextValue(temp_screen_id, 6, "烟雾状态:报警");
					}
					cabin_dsc_entry->last_smoke_state = Cang_YWZT_buf[temp_cabin_id];
				} // 烟雾状态刷新
				
				if(cabin_dsc_entry->last_co_value != Cang_COzhi_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					uint8_t temp_buff[16];
					cabin_dsc_entry->last_temperat_value = Cang_COzhi_buf[temp_cabin_id];
					sprintf((char *)temp_buff, "一氧化碳浓度:%dPPM", Cang_COzhi_buf[temp_cabin_id]);
					SetTextValue(temp_screen_id, 7, temp_buff);
					
					cabin_dsc_entry->last_co_value = Cang_COzhi_buf[temp_cabin_id];
				} // 一氧化碳浓度刷新
				
				if(cabin_dsc_entry->last_hh_value != Cang_H2zhi_buf[temp_cabin_id] || cabin_dsc_entry->force_fresh_flag == 1)
				{
					uint8_t temp_buff[16];
					cabin_dsc_entry->last_temperat_value = Cang_H2zhi_buf[temp_cabin_id];
					sprintf((char *)temp_buff, "氢气浓度:%dPPM", Cang_H2zhi_buf[temp_cabin_id]);
					SetTextValue(temp_screen_id, 8, temp_buff);
					cabin_dsc_entry->last_hh_value = Cang_H2zhi_buf[temp_cabin_id];
				} // 氢气浓度刷新
			}
		}

		cabin_dsc_entry->lasr_cabin_state = cang_sxzt[temp_cabin_id];
	}
	else
	{
		if(cabin_dsc_entry->force_fresh_flag == 1)
		{
			uint8_t temp_buff[32] = {0};
			sprintf((char *)temp_buff,"探测器%d状态:未启用", temp_cabin_id);
			SetTextValue(temp_screen_id, 2, temp_buff);
			SetTextValue(temp_screen_id, 3, "探测器型号:--");
			SetTextValue(temp_screen_id, 4, "传感器启用状态:--");
			SetTextValue(temp_screen_id, 5, "温度:--");
			SetTextValue(temp_screen_id, 6, "烟雾状态:--");
			SetTextValue(temp_screen_id, 7, "一氧化碳浓度:--");
			SetTextValue(temp_screen_id, 8, "氢气浓度:--");
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

		// 如果数据异常
		if(temp_mcg.co_max_val == -1) // 表示没有探测器上线 或全部探测器掉线
		{
			SetTextValue(temp_screen_id, 26, "探测器未上线/全掉线/无正确数据");
		}
		else
		{
			uint8_t temp_buff[48];
			
			if(temp_mcg.curr_da.cluster_id == 0 && temp_mcg.curr_da.pack_id == 0 && temp_mcg.curr_da.cabin_id != 0)
			{
				if(Hydrogen_Type == temp_mcg.gas_type)
				{
					sprintf((char *)temp_buff, "第1回路 %d号 氢气浓度%dPPM", temp_mcg.curr_da.cabin_id, temp_mcg.co_max_val);
					SetTextValue(temp_screen_id, 26, temp_buff);
				}
				else
				{
					sprintf((char *)temp_buff, "第1回路 %d号 一氧化碳浓度%dPPM", temp_mcg.curr_da.cabin_id, temp_mcg.co_max_val);
					SetTextValue(temp_screen_id, 26, temp_buff);
				}
				
			}
			else if(temp_mcg.curr_da.cluster_id != 0 && temp_mcg.curr_da.pack_id != 0 && temp_mcg.curr_da.cabin_id == 0)
			{
				sprintf((char *)temp_buff, "第%d簇pack%d一氧化碳浓度%dPPM", temp_mcg.curr_da.cluster_id, temp_mcg.curr_da.pack_id, temp_mcg.co_max_val);
				SetTextValue(temp_screen_id, 26, temp_buff);
			}
			else
			{
				SetTextValue(temp_screen_id, 26, "探测器无正确数据");
			}
		}
		
	}
	
}

#endif /* XR5000_GAS_SUMMARY_CHANGE_20260731 */

static void DetectorDataFreshMenuCtrl(DetectorDataShowCtrl *ddsc_entry, uint16_t ctrl_id, uint8_t item, uint8_t state)
{
	if(ctrl_id == 103)
	{
		if(state == 1) // 按键按下
		{
			ddsc_entry->curr_detector_page = item + 1;
		}
	}
}

static void DetectorDataFreshMenuCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t item, uint8_t state)
{
	if(ctrl_id == 103)
	{
		if(state == 1) // 按键按下
		{
			ddsc_32p_entry->curr_detector_page = item + 1;
			ddsc_32p_entry->force_fresh_flag = 1; // 强制刷新标志位
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
				ddsc_32p_entry->force_fresh_flag = 1; // 强制刷新标志位
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
				ddsc_32p_entry->force_fresh_flag = 1; // 强制刷新标志位
			}
				
		}
	}
}

const uint8_t button_value_map[] = {
	  5,   9,  13,  17,  21,  25,  29,  33, 
	 37,  41,  45,  49,  53,  57,  61,  65, 
	 69,  73,  77,  81,  85,  89,  93,  97, 
	101, 111, 115, 119, 123, 127, 131, 136,};
// 1簇32pack版本 查询PACK按钮
static void DetectorMonitorButtonCtrl_32Pack(DetectorDataShowCtrl_32Pack *ddsc_32p_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		for(uint8_t i = 0; i < 32; i++)
		{
			if(button_value_map[i] == ctrl_id)
			{
				ddsc_32p_entry->curr_pack_id = i + 1;
				ddsc_32p_entry->force_fresh_flag = 1; // 强制刷新标志位
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
		AnimationPlayFrame(5, ctrl_id, set_online_state);//(画面ID,控件ID,帧ID) 0红，1绿
		
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
				AnimationPlayFrame(5, i, 1);//(画面ID,控件ID,帧ID) 0红，1绿
			}
			SavePointTypeSetOnlieState();
			ReadPointTypeSetOnlieState();
			break;
		}
		case 234: {
			for(uint8_t i = 1; i < 33; i++)
			{
				PointTypeMixtureOnlieStateSingleSetting(i, 0);
				AnimationPlayFrame(5, i, 0);//(画面ID,控件ID,帧ID) 0红，1绿
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
				if(success_len == 2 && x >0 && y > 0) // 解析成功 并且x y大于零
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
							AnimationPlayFrame(5, i, 1);//(画面ID,控件ID,帧ID) 0红，1绿
							modify_flag = 1;	
						}
					}
					if(modify_flag == 1)
					{
						SavePointTypeSetOnlieState();
					}
				}
			}
			SetTextValue(5, 235, "批量上线");
			break;
		}
		case 236:{
			uint8_t temp_str_len = strlen((char *)entry_str);
			if(temp_str_len != 0)
			{
				int8_t success_len;
				int32_t x;
				success_len = sscanf((const char*)entry_str, "%d", &x);  
				if(success_len == 2 && x >0) // 解析成功 并且x大于零
				{
					uint8_t modify_flag = 0;

					if(getPointTypeMixtureDetectOnlineState(x) == 0)
					{
						PointTypeMixtureOnlieStateSingleSetting(x, 1);
						AnimationPlayFrame(5, x, 1);//(画面ID,控件ID,帧ID) 0红，1绿
						modify_flag = 1;	
					}
					if(modify_flag == 1)
					{
						SavePointTypeSetOnlieState();
					}
				}
			}
			SetTextValue(5, 236, "单点上线");
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
		AnimationPlayFrame(5, i, getPointTypeMixtureDetectOnlineState(i));//(画面ID,控件ID,帧ID) 0红，1绿
	}
}

void StoragePackCabinForeWarn(PackCabinForeWarnStorage *pcfws_entry, uint8_t cluster_id, uint8_t pack_id, uint8_t alarm_type)
{
	uint8_t flag = 0;
	for(uint8_t l = 0;l < pcfws_entry->self_bottom_point; l++)
	{
		// 如果簇ID是外联设备ID 
		if(pcfws_entry->da[l].cluster_id == LINKAGE_CLUSTER_ID)
		{
			// 如果传进来的参数有和结构体内相同的 结束循环
			if(pcfws_entry->da[l].pack_id == pack_id) // 不需要判断报警类型是否一致 因为只有这一个
			{
				flag = 1;
				break;
			}
		}
		else if(pcfws_entry->da[l].cluster_id != 0)
		{
			// 如果传进来的参数有和结构体内相同的 结束循环
			if(pcfws_entry->da[l].cluster_id == cluster_id && pcfws_entry->da[l].pack_id == pack_id && pcfws_entry->alarm_type[l] == alarm_type)
			{
				flag = 1;
				break;
			}
		}
		else if(pcfws_entry->da[l].pack_id != 0)
		{
			if( pcfws_entry->da[l].cabin_id == pack_id &&
					pcfws_entry->alarm_type[l]  == alarm_type) // 既能当仓也能当pack id
			{
				flag = 1;
				break;
			}
		}
	}
	// 表示没有找到相同的 即第一次出现
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
		
		// 2025/11/19 13:53 新增记录秒
		pcfws_entry->atr[pcfws_entry->self_bottom_point].second = secs;
		
		pcfws_entry->self_bottom_point++;
		
		beep_fire_ctrl |= 0x0F;  // 任意报警 长鸣
		silencers_state = 0; // 有新的报警 蜂鸣器开 清除消音标志位
	}
}

// 预警可以清除
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
	if(flag == 1 && pcfws_entry->self_bottom_point > 0) // 表示恢复正常的探测器在报警数组中
	{
		for(;l < pcfws_entry->self_bottom_point - 1; l++)
		{
			pcfws_entry->detector_class[l] = pcfws_entry->detector_class[l + 1];
			pcfws_entry->da[l]         = pcfws_entry->da[l + 1]; 
			pcfws_entry->atr[l]        = pcfws_entry->atr[l + 1];
			pcfws_entry->alarm_type[l] = pcfws_entry->alarm_type[l + 1];
		}
		pcfws_entry->self_bottom_point--;
		
		if(pcfws_entry->self_bottom_point > 0) // 恢复后还有其他报警的话
		{
			beep_fire_ctrl |= 0x0F; // 打开预警蜂鸣器
			silencers_state = 0; // 有新的报警 蜂鸣器开 清除消音标志位
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
		if(pcfas_entry->detector_class[l] == CabinClassID) // 如果是仓才进行判断
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
	PackCabinFireAlarmStorage *pcfas_entry, // 要存入的缓冲区地址
	uint8_t cluster_id,                     // 簇
	uint8_t pack_id,                        // 包/仓
	uint8_t alarm_type                      // 报警类型
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
		else // 仓
		{
			if( pcfas_entry->da[l].cabin_id == pack_id &&
					pcfas_entry->alarm_type[l]  == alarm_type) // 既能当仓也能当pack id
			{
				flag = 1;
				break;
			}
		}
	}
	
	if(flag != 1)
	{
		if(cluster_id == LINKAGE_CLUSTER_ID) // 如果是外联设备ID
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
	// 计算屏蔽总数
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
	uint8_t temp_partition = bkcnc_entry->curr_partition; // 复制一份
	uint8_t prev_partition = bkcnc_entry->last_partition; // XR5000_CURSOR_FIX_20260726: capture the pre-mutation partition for correct stale-cursor clearing below.
	
	uint8_t key_val_temp;
	
	// 需要初始化
	if(bkcnc_entry->curr_menu_state == InitMenu) // 先初始化菜单状态
	{
		for(uint8_t i = 0; i < 3; i+=2)
		{
			clearTextValue(temp_screen_id, partition_ctrl_id[i]); // 默认清空箭头
			for(uint8_t j = 0; j < 7; j++)
			{
				clearTextValue(temp_screen_id, point_site_ctrl_id[i][j]); // 默认清空箭头
			}
		}
		bkcnc_entry->curr_menu_state = OutMenu; // 默认在菜单外
		bkcnc_entry->last_menu_state = OutMenu;
		// 初始化的时候 赋值一次
		bkcnc_entry->last_show_len[ForceAlarmPart] = pcfws.self_bottom_point;
		bkcnc_entry->last_show_len[FaultPart]      = pcfs_buttom_point;
		bkcnc_entry->last_show_len[FireAlarmPart]  = pcfas.self_bottom_point;
		bkcnc_entry->last_show_len[OutFirePart]    = fedas.self_point_len;
		
		bkcnc_entry->curr_partition = ForceAlarmPart;
		bkcnc_entry->last_partition = ForceAlarmPart;
		
		bkcnc_entry->last_point_site[temp_partition] = bkcnc_entry->curr_point_site[temp_partition];
		SetTextValue(temp_screen_id, partition_ctrl_id[ForceAlarmPart], "<-");
	}
	
	// 箭头位置更新
	
	// 故障箭头刷新 - 修改部分
	if(((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) > bkcnc_entry->last_show_len[FaultPart])
	{
		// 如果故障数量增加了 箭头位置不用动
		bkcnc_entry->last_show_len[FaultPart] = (uint8_t)((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount());
	}
	else if(((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) < bkcnc_entry->last_show_len[FaultPart])
	{
		// 如果故障数量减少了（有恢复）
		uint8_t new_count = (uint8_t)((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount());
			
		// 调整当前箭头位置，确保不超出范围
		if (bkcnc_entry->curr_point_site[FaultPart] >= new_count) 
		{
			bkcnc_entry->curr_point_site[FaultPart] = (new_count > 0) ? new_count - 1 : 0;
		}
			
		// 调整翻页索引，确保显示区域有效
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
	
	// 预警箭头刷新 - 修改部分
	if(pcfws.self_bottom_point > bkcnc_entry->last_show_len[ForceAlarmPart])
	{
		// 如果预警数量增加了 箭头位置不用动
		bkcnc_entry->last_show_len[ForceAlarmPart] = pcfws.self_bottom_point;
	}
	else if(pcfws.self_bottom_point < bkcnc_entry->last_show_len[ForceAlarmPart])
	{
		// 如果预警数量减少了（有恢复）
		uint8_t new_count = pcfws.self_bottom_point;
		
		// 调整当前箭头位置，确保不超出范围
		if (bkcnc_entry->curr_point_site[ForceAlarmPart] >= new_count) 
		{
			bkcnc_entry->curr_point_site[ForceAlarmPart] = (new_count > 0) ? new_count - 1 : 0;
		}
			
		// 调整翻页索引，确保显示区域有效
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
	// 获取按键值 
	switch(key_val_temp)
	{
		case KEY6_DIRECTION_UP   : // 方向键 上
			if(bkcnc_entry->curr_menu_state == OutMenu) // 如果当前是在菜单外
			{
				if(temp_partition == FireAlarmPart) // 如果当前分区是火警分区
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
			else // 如果是在菜单内 
			{
				if(temp_partition == ForceAlarmPart) // 如果是在预警栏
				{
					if(pcfws.self_bottom_point > 0)
					{
						// 
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // 指针偏移
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0) // 向上滚动
						{
							if(fore_alarm_start_index > 0)
							{
								fore_alarm_start_index--;
								force_alarm_check_new_flag = 1;
							}
						}
					}
				}
				else if(temp_partition == FaultPart) // 如果是故障栏
				{
					// 如果有故障信息
					if(pcfs_buttom_point > 0)
					{
						// 如果当前指针没在最顶上面 先移动箭头到最上面
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // 指针偏移
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0)
						{
							// 翻页逻辑 向上翻页
							if (fault_current_page > 0) 
							{ 
								fault_current_page--;
								fault_check_new_flag = 1; // 标记按下
							} 
						}
						
						// 暂时不支持到头回底
//						else {
//							// 跳到最后一页
//							fault_current_page = pcfs_buttom_point - Fault_Show_Zone;
//							// 处理最后一页不足的情况
//							if (fault_current_page < 0) {
//									fault_current_page = 0; // 如果总条目不足一页，保持在首页
//							}
//						}
						
					}
					else
					{
						// 否则指针默认指在第一个
						bkcnc_entry->curr_point_site[temp_partition] = 0;
					}
				}
				else if(temp_partition == FireAlarmPart) // 如果是火警
				{
					// 如果有故障信息
					if(pcfas.self_bottom_point > 0) // 有火警
					{
						// 如果当前指针没在最顶上面 先移动箭头到最上面
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // 指针偏移
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0)
						{
							// 翻页逻辑 向上翻页
							if (fire_alarm_start_index > 0) 
							{ 
								fire_alarm_start_index--;
								fire_alarm_check_new_flag = 1; // 标记按下
							} 
						}
					}
					
				}
				else if(temp_partition == OutFirePart) // 气灭分区
				{
					if(fedas.self_point_len > 0)
					{
						// 如果当前指针没在最顶上面 先移动箭头到最上面
						if(bkcnc_entry->curr_point_site[temp_partition] > 0) 
						{
							bkcnc_entry->curr_point_site[temp_partition]--; // 指针偏移
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == 0)
						{
							// 翻页逻辑 向上翻页
							if (fedas_fresh_point > 0) 
							{ 
								fedas_fresh_point--;
								fed_fresh_flag = 1; // 标记按下
							} 
						}
					}
				}
			}
			break;
		case KEY7_DIRECTION_RIGHT: // 如果方向键 右 按下
			if(temp_partition == ForceAlarmPart) // 如果当前分区是预警
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // 如果在菜单内
				{
					bkcnc_entry->curr_menu_state = OutMenu; // 退出菜单
				}
				else // 本来就在菜单外
				{
					bkcnc_entry->curr_partition = FaultPart; // 切换分区
					temp_partition = FaultPart;
				}
			}
			else if(temp_partition == FireAlarmPart)
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // 如果在菜单内
				{
					bkcnc_entry->curr_menu_state = OutMenu; // 退出菜单
				}
				else 
				{
					bkcnc_entry->curr_partition = OutFirePart;
					temp_partition = OutFirePart;
				}
			}
			else if(temp_partition == FaultPart || temp_partition == OutFirePart)
			{
				if(bkcnc_entry->curr_menu_state == OutMenu) // 菜单外
				{
					bkcnc_entry->curr_menu_state = InMenu; //  进入菜单内
				}
				else
				{
					bkcnc_entry->curr_menu_state = OutMenu; //  退出菜单
				}
			}
			break;
		case KEY8_DIRECTION_DOWN : // 方向键下
			if(bkcnc_entry->curr_menu_state == OutMenu) // 如果当前是在菜单外
			{
				if(temp_partition == ForceAlarmPart) // 如果当前分区是预警分区
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
				if(temp_partition == ForceAlarmPart) // 如果是在预警栏
				{
					// 如果有预警
					if(pcfws.self_bottom_point > 0)
					{
						// 如果 当前箭头位置还没有指到底
						if(bkcnc_entry->curr_point_site[temp_partition] < Alarm_Show_Zone - 1) 
						{
							// 检查是否超过实际数据范围（防止数据量不足时越界）
							if (bkcnc_entry->curr_point_site[temp_partition] < pcfws.self_bottom_point - 1) {
									bkcnc_entry->curr_point_site[temp_partition]++;
							}
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == Alarm_Show_Zone - 1)
						{
							// 翻页逻辑 // 当前页+显示区域长度 小于总数 则可以往后滚动
							if (fore_alarm_start_index + Alarm_Show_Zone < pcfws.self_bottom_point) 
							{
								fore_alarm_start_index++;
								force_alarm_check_new_flag = 1;
							}
						}
					}
				}
				else if(temp_partition == FaultPart) // 如果是故障栏
				{
					if(pcfs_buttom_point > 0) // 如果有报警
					{
						// 如果 当前箭头位置还没有指到底
						if(bkcnc_entry->curr_point_site[temp_partition] < Fault_Show_Zone - 1) 
						{
//							bkcnc_entry->curr_point_site[temp_partition]++;
//							if(bkcnc_entry->curr_point_site[temp_partition] > pcfs_buttom_point - 1)
//							{
//								bkcnc_entry->curr_point_site[temp_partition] = pcfs_buttom_point - 1;
//							}
							// 检查是否超过实际数据范围（防止数据量不足时越界）
							if (((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) > 0U && bkcnc_entry->curr_point_site[temp_partition] < ((uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) - 1U) {
									bkcnc_entry->curr_point_site[temp_partition]++;
							}
						}
						else if(bkcnc_entry->curr_point_site[temp_partition] == Fault_Show_Zone - 1)
						{
							// 翻页逻辑 // 当前页+显示区域长度 小于总数 则可以往后滚动
							if ((uint16_t)fault_current_page + Fault_Show_Zone < (uint16_t)pcfs_buttom_point + DeviceRegistry_GetProductUnknownCount()) {
								fault_current_page++;
								fault_check_new_flag = 1; // 标记按下
							} 
					// 暂时去掉到底回头
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
				else if(temp_partition == FireAlarmPart) // 火警
				{
					if(pcfas.self_bottom_point > 0)
					{
						// 如果 当前箭头位置还没有指到底
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
					} // 如果火警存储有记录
				}
				else if(temp_partition == OutFirePart) // 气灭分区
				{
					if(fedas.self_point_len > 0)
					{
						// 如果 当前箭头位置还没有指到底
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
					} // 如果气灭分区存储有记录
					
				}
			}
			break;
		case KEY9_DIRECTION_LEFT : // 方向键 左 按下
			if(temp_partition == FaultPart)
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // 如果在菜单内
				{
					bkcnc_entry->curr_menu_state = OutMenu; // 退出菜单
				}
				else
				{
					bkcnc_entry->curr_partition = ForceAlarmPart;
					temp_partition = ForceAlarmPart;
				}
			}
			else if(temp_partition == OutFirePart)
			{
				if(bkcnc_entry->curr_menu_state == InMenu) // 如果在菜单内
				{
					bkcnc_entry->curr_menu_state = OutMenu; // 退出菜单
				}
				else
				{
					bkcnc_entry->curr_partition = FireAlarmPart;
					temp_partition = FireAlarmPart;
				}
			}
			else if(temp_partition == ForceAlarmPart || temp_partition == FireAlarmPart)
			{
				if(bkcnc_entry->curr_menu_state == OutMenu) // 菜单外
				{
					bkcnc_entry->curr_menu_state = InMenu; // 进入菜单
				}
				else
				{
					bkcnc_entry->curr_menu_state = OutMenu; //  退出菜单
				}
			}
			break;
		case KEY10_DIRECTION_OK  :
			break;
	}
	
	// 初始化菜单内显示
	if(temp_partition != bkcnc_entry->last_partition) // 
	{
		// 清除历史分区箭头
		clearTextValue(temp_screen_id, partition_ctrl_id[bkcnc_entry->last_partition]);
		bkcnc_entry->last_partition = temp_partition; // 表明已经更新

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
	
	// 菜单内指针控制
	if( bkcnc_entry->last_point_site[temp_partition] != bkcnc_entry->curr_point_site[temp_partition] || // 如果是分区状态位置改变
			bkcnc_entry->last_menu_state                 != bkcnc_entry->curr_menu_state                    // 如果是菜单内外状态改变
	)
	{
		// 清除历史箭头
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
	switch_ui_ctrl.curr_pack_alarm_len = &pas_pointer; // 绑定包火警数量
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
	) // 如果有任意的报警 故障存在
	{
		return;
	}
	if( *(sic_entry->curr_pack_alarm_len   ) != 0 ||
			*(sic_entry->curr_pc_fire_alarm_len) != 0 || 
			*(sic_entry->curr_pc_fore_alarm_len) != 0 ||
			*(sic_entry->curr_pc_fault_len     ) != 0	||
			*(sic_entry->curr_pc_outfire_len   ) != 0
	) // 如果有任意的报警 故障存在
	{
		uint32_t curr_time = osKernelGetTickCount();
		if(curr_time - sic_entry->curr_sys_time >= 180000)
		{
			sic_entry->curr_sys_time = curr_time;
			// 打开屏幕背光
			beiguangkai();
		}
	}
	
	if( *(sic_entry->curr_pack_alarm_len   ) != sic_entry->last_pack_alarm_len    || 
			*(sic_entry->curr_pc_fire_alarm_len) != sic_entry->last_pc_fire_alarm_len || 
			*(sic_entry->curr_pc_fore_alarm_len) != sic_entry->last_pc_fore_alarm_len ||
			*(sic_entry->curr_pc_fault_len     ) != sic_entry->last_pc_fault_len      ||
			*(sic_entry->curr_pc_outfire_len   ) != sic_entry->last_pc_outfire_len
	) // 如果有任意的报警 故障变动
	{
		sic_entry->last_pack_alarm_len    = *(sic_entry->curr_pack_alarm_len);
		sic_entry->last_pc_fire_alarm_len = *(sic_entry->curr_pc_fire_alarm_len);
		sic_entry->last_pc_fore_alarm_len = *(sic_entry->curr_pc_fore_alarm_len);
		sic_entry->last_pc_fault_len      = *(sic_entry->curr_pc_fault_len     );
		sic_entry->last_pc_outfire_len    = *(sic_entry->curr_pc_outfire_len   );

		// 打开屏幕背光
		beiguangkai();
		// 回一次主界面
		SetMonitorPageFrom(current_screen_id); /* XR5000_MONITOR_RETURN_NAV_CHANGE_20260802 */
	}
	
}

// 查询报警切换界面（这是第一步，查询报警界面，从查询报警来）
static void RecordSwitchButtonCtrl(BspScreenReadRecord_t *bsrr_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		switch(ctrl_id)
		{
			case 1: {
				int8_t temp_sector = 0;
				bsrr_entry->curr_show_type = RECORD_FAULT;
				SetScreen(57);	// 进入二级密码页
				// 先读取数量
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(FAULT_FLASH_SAVE);
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //记录总数显示
				bsrr_entry->force_fresh_flag = 1; // 每次进入必须强制刷新一次
				
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[0].byte_buff, 0);
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[1].byte_buff, 1);
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[2].byte_buff, 2);
//				BspReadFlashData(FAULT_FLASH_SAVE, read_data[3].byte_buff, 3);
				
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=500)
				{
					// 计算缓冲区下标
					temp_sector = temp_sum/500;
					// 读取缓冲区中的内容
					BspReadFlashData(FAULT_FLASH_SAVE, read_data[temp_sector].byte_buff, temp_sector);
				}
				osDelay(5);
				GetScreen();
				
				break;
			}
			case 2: {
				int8_t temp_sector = 0;
				bsrr_entry->curr_show_type = RECORD_ALARM;
				SetScreen(56);	// 切换到报警显示页
				
				// 先读取数量
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(FIRE_FLASH_SAVE);
				SetTextInt32(56, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //记录总数显示
				bsrr_entry->force_fresh_flag = 1; // 每次进入必须强制刷新一次
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=400)
				{
					// 计算缓冲区下标
					temp_sector = temp_sum/400;
					// 读取缓冲区中的内容
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
				
				// 先读取数量
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(GASER_FLASH_SAVE);
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //记录总数显示
				bsrr_entry->force_fresh_flag = 1; // 每次进入必须强制刷新一次
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=500)
				{
					// 计算缓冲区下标
					temp_sector = temp_sum/500;
					// 读取缓冲区中的内容
					BspReadFlashData(GASER_FLASH_SAVE, read_data[temp_sector].byte_buff, temp_sector);
				}
				osDelay(5);
				GetScreen();
				
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //记录总数显示
				
				break;
			}
				
			case 4: {
				int8_t temp_sector = 0;
				bsrr_entry->curr_show_type = RECORD_OTHER;
				SetScreen(57);	// 

				// 先读取数量
				bsrr_entry->record_sum[bsrr_entry->curr_show_type] = getFlashSaveDataNummber(OTHER_FLASH_SAVE);
				SetTextInt32(57, 99, bsrr_entry->record_sum[bsrr_entry->curr_show_type], 0, 1);   //记录总数显示
				bsrr_entry->force_fresh_flag = 1; // 每次进入必须强制刷新一次
				for(int16_t temp_sum = bsrr_entry->record_sum[bsrr_entry->curr_show_type]; temp_sum > 0; temp_sum-=500)
				{
					// 计算缓冲区下标
					temp_sector = temp_sum/500;
					// 读取缓冲区中的内容
					BspReadFlashData(OTHER_FLASH_SAVE, read_data[temp_sector].byte_buff, temp_sector);
				}
				osDelay(5);
				GetScreen();
				
				break;
			}	
			default:
				break;
		}
		bsrr_entry->curr_page[bsrr_entry->curr_show_type] = 1; // 每次进入强制显示第一页
	}
}

// 查询时切换页按键，换页动作
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
					bsrr_entry->force_fresh_flag = 1; // 强制刷新标志位
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
// 显示记录,同时换页后也调用这个进行显示
static void InternalScreenShowRecord(BspScreenReadRecord_t *bsrr_entry)
{
	uint8_t temp_screen_id = 0;
	uint8_t x_sector; // 扇区偏移
	uint8_t temp_page = bsrr_entry->curr_page[bsrr_entry->curr_show_type]; // 临时记录一次页 预防再别处被修改
	
	if(bsrr_entry->force_fresh_flag == 1) // 强制刷新标志位
	{
		uint8_t show_buff[32];
		uint16_t temp_caculate;
		uint16_t data_index;
		uint16_t start = (temp_page - 1)*RECORD_SHOW_ZONE; // 开始下标
		uint16_t total_records = bsrr_entry->record_sum[bsrr_entry->curr_show_type];
		
		// 通过一个变量修改显示页面
		temp_screen_id = (bsrr_entry->curr_show_type == RECORD_ALARM) ? 56 : 57;

		// 更新显示页 只刷新一次
		bsrr_entry->force_fresh_flag = 0;
		
		// 更新页显示
		SetTextInt32(temp_screen_id, 97, temp_page, 0, 1);   
		
		for(uint16_t i = 0; i < RECORD_SHOW_ZONE; i++)
		{
			temp_caculate = start + i;
			if(temp_caculate < total_records) // 如果当前索引小于总数
			{
				// 假设总数 total_records = 591 条 现在 temp_page = 10 第十页 start = （10-1）*10 = 90
				// 假设现在i = 0，temp_caculate = start + i = 90；i= 1，temp_caculate = start + i = 91；...
				// reverse_index = 591 - 1 - 90 = 500；reverse_index = 591 - 1 - 91 = 499 
				uint16_t reverse_index = total_records - 1 - temp_caculate;
				// 
				switch(bsrr_entry->curr_show_type)
				{
					case RECORD_FAULT: {
						// 测试数据是否正确
						//DebugSendString(read_data[total_records/500].byte_buff, total_records * 8);
						//x_sector = total_records/500;
						x_sector = reverse_index/500;
						data_index = reverse_index%500;
						
						// 显示序号
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //

						// 设备名称判断
						if(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id != 0)
						{
							if(FormatRS485DetectFlashDeviceName(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id,
								read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id, show_buff) == 1)
							{
								// XR5000_LOOP3_CHANGE_20260726: Loop 3 history fault display uses "第3回路 X号".
							}
							else if(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id == MBUS_CONTROL_FLASH_ID)
							{
								sprintf((char *)show_buff, "第2回路 %d号", read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id);
							}
							else if(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
							{
								switch(read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id)
								{
									case Deflate_Package_ID: {
										sprintf((char *)show_buff, "放气误入");
										break;
									}
									case SoundLt_Package_ID: {
										sprintf((char *)show_buff, "声光报警器");
										break;
									}
									case SirenBk_Package_ID: {
										sprintf((char *)show_buff, "警笛/备用");
										break;
									}
									case OutFir1_Package_ID: {
										sprintf((char *)show_buff, "灭火装置1");
										break;
									}
									case OutFir2_Package_ID: {
										sprintf((char *)show_buff, "灭火装置2");
										break;
									}
									case CabinBK_Package_ID: {
										sprintf((char *)show_buff, "喷放装置");
										break;
									}
									case FEEDBK1_Package_ID: {
										sprintf((char *)show_buff, "反馈1");
										break;
									}
									case FEEDBK2_Package_ID: {
										sprintf((char *)show_buff, "反馈2");
										break;
									}
									case HANDPOT_Package_ID: {
										sprintf((char *)show_buff, "手报");
										break;
									}
									case SYS_MAIN_POWER_KEY_ID: {
										sprintf((char *)show_buff, "主电源");
										break;
									}
									case SYS_BACK_POWER_KEY_ID: {
										sprintf((char *)show_buff, "备电池");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_1: {
										sprintf((char *)show_buff, "回路1");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_2: {
										sprintf((char *)show_buff, "回路2");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_3: {
										sprintf((char *)show_buff, "回路3");
										break;
									}
									case GENERAL_IOPUT_ISOLATE_OUTPUT_ID_4: {
										sprintf((char *)show_buff, "回路4");
										break;
									}
									default:
										break;
								}
							}
							else
							{
								sprintf((char *)show_buff, "第%d簇%dpack", 
									read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cluster_id,
									read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id);
							}
						}
						else
						{
							sprintf((char *)show_buff, "第1回路 %d号", read_data[x_sector].fs_sys_fault[data_index].fs_detect_id.cabin_or_pack_id);
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //刷新设备名称
						// 显示时间
						
						// 申请 临时变量 用来刷新时间显示
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_sys_fault[data_index].fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d年%02d月%02d日 %02d时%02d分%02d秒", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);
						
						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //刷新时间
						// 状态显示
						if(read_data[x_sector].fs_sys_fault[data_index].state == DISCONNECT)
						{
							sprintf((char *)show_buff, "设备掉线");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == DIS_RECOVERY)
						{
							sprintf((char *)show_buff, "掉线恢复");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == SHORTCIRCUIT)
						{
							sprintf((char *)show_buff, "设备短路");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == SHO_RECOVERY)
						{
							sprintf((char *)show_buff, "短路恢复");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_TEMP_SENSOR_FAULT)
						{
							sprintf((char *)show_buff, "\xCE\xC2\xB6\xC8\xB9\xCA\xD5\xCF");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_TEMP_SENSOR_RECOVERY)
						{
							sprintf((char *)show_buff, "温度故障恢复");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_SMOKE_POLLUTION_FAULT)
						{
							sprintf((char *)show_buff, "烟雾污染故障");
						}
						else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_SMOKE_POLLUTION_RECOVERY)
						{
							sprintf((char *)show_buff, "烟雾污染故障恢复");
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
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CO_SENSOR_FAULT) { sprintf((char *)show_buff, "CO传感器故障"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CO_SENSOR_RECOVERY) { sprintf((char *)show_buff, "CO传感器故障恢复"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_H2_SENSOR_FAULT) { sprintf((char *)show_buff, "H2传感器故障"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_H2_SENSOR_RECOVERY) { sprintf((char *)show_buff, "H2传感器故障恢复"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_VOC_SENSOR_FAULT) { sprintf((char *)show_buff, "VOC传感器故障"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_VOC_SENSOR_RECOVERY) { sprintf((char *)show_buff, "VOC传感器故障恢复"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CH4_SENSOR_FAULT) { sprintf((char *)show_buff, "CH4传感器故障"); }
                        else if(read_data[x_sector].fs_sys_fault[data_index].state == RS485_CH4_SENSOR_RECOVERY) { sprintf((char *)show_buff, "CH4传感器故障恢复"); }
						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //刷新状态
						
						break;
					}
					case RECORD_ALARM: {
						// 临时扇区
//						x_sector = total_records/400;
//						data_index = (total_records - 1 - temp_caculate)%400; // 计算数据索引
//						
						x_sector = reverse_index/400;
						data_index = reverse_index%400;
						
//						DebugSendString(&read_data[total_records/500].byte_buff[data_index * 10], 10);
//						
//						sprintf((char *)show_buff, "%d", total_records);
//						SetTextValue(1, 37, show_buff); //刷新状态
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
//						SetTextValue(1, 36, show_buff); //刷新状态
//						
						// 设备名称判断
						if(FormatRS485DetectFlashDeviceName(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id,
							read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id, show_buff) == 1)
						{
							// XR5000_LOOP3_CHANGE_20260726: Loop 3 history alarm display uses "第3回路 X号".
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == MBUS_CONTROL_FLASH_ID)
					{
						/* XR5000_MBUS2_HAND_ALARM_FIRE_HISTORY_20260729: render loop2 manual alarm device. */
						sprintf((char *)show_buff, "第2回路 %d号", read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id);
					}
					else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id == ALARM_ANNUNCIATOR_ID)
							{
								sprintf((char *)show_buff, "报警器");
							}
							else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id == HANDPOT_Package_ID)
							{
								sprintf((char *)show_buff, "手动报警器");
							}
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id != 0)
						{
							sprintf((char *)show_buff, "第%d簇%dpack", read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id,
								read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id);
							
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cluster_id == 0 && 
										read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id != 0)
						{
							sprintf((char *)show_buff, "第1回路 %d号", read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_detect_id.cabin_or_pack_id);
						}
						else
						{
							sprintf((char *)show_buff, "未知设备");
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //刷新设备名称
						
						// 显示序号
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //
						
						// 显示时间
						// 2025/11/19 15:39 修改
						// 申请 临时变量 用来刷新时间显示
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_fire_alarm[data_index].fs_base.fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d年%02d月%02d日 %02d时%02d分%02d秒", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);

						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //刷新时间
						
						// 状态显示
						if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == FIRGAS_ALARM)
						{
							sprintf((char *)show_buff, "可燃气体报警");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == EAR_RECOVERY)
						{
							sprintf((char *)show_buff, "可燃气体恢复");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == SMOKE_ALARM)
						{
							sprintf((char *)show_buff, "烟雾报警");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == TEMPRT_ALARM)
						{
							sprintf((char *)show_buff, "温度火警");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == LINKAGE_PRESS)
						{
							sprintf((char *)show_buff, "报警器报警");
						}
						else if(read_data[x_sector].fs_fire_alarm[data_index].fs_base.state == MBUS2_HAND_ALARM)
						{
							/* XR5000_MBUS2_HAND_ALARM_FIRE_HISTORY_20260729: render manual alarm state. */
							sprintf((char *)show_buff, "手动报警");
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
								sprintf((char *)show_buff, "可燃气体一氧化碳报警");
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
								sprintf((char *)show_buff, "可燃气体氢气报警");
							}
						}
						else if(RS485_TEMP_WARNING == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "温度预警");
						}
						else if(RS485_CO_FIRE == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "CO火警");
						}
						else if(RS485_H2_FIRE == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "H2火警");
						}
						else if(FIRGAS_ALARM_VOC == read_data[x_sector].fs_fire_alarm[data_index].fs_base.state)
						{
							sprintf((char *)show_buff, "VOC预警");
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
						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //刷新状态
						
						if(read_data[x_sector].fs_fire_alarm[data_index].data_high == 0xFFFF)
						{
							sprintf((char *)show_buff, "--");
						}
						else
						{
							sprintf((char *)show_buff, "%d", read_data[x_sector].fs_fire_alarm[data_index].data_high);
						}
						SetTextValue(temp_screen_id, values_ctrl_id[i], show_buff); //刷新状态
						
						break;
					}
					case RECORD_GASOF: {
						// 临时扇区
//						x_sector = total_records/500;
//						data_index = (total_records - 1 - temp_caculate)%500; // 计算数据索引
						x_sector = reverse_index/500;
						data_index = reverse_index%500;

						// 显示序号
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //
						
						// 设备名称判断
						// 如果是外联设备
						if(FormatRS485DetectFlashDeviceName(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id,
							read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id, show_buff) == 1)
						{
							// XR5000_LOOP3_CHANGE_20260726: Loop 3 gas action history display uses "第3回路 X号".
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id == OUTFIRE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id == OUTFIR1_PACKAGE_ID)
							{
								sprintf((char *)show_buff, "灭火装置1");
							}
							else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id == OUTFIR2_PACKAGE_ID)
							{
								sprintf((char *)show_buff, "灭火装置2");
							}
							else
							{
								sprintf((char *)show_buff, "灭火装置");
							}
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id == ALARM_ANNUNCIATOR_ID)
							{
								sprintf((char *)show_buff, "警报器");
							}
							else if(PART1_HAND_AUTO_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "分区1");
							}
							else if(PART2_HAND_AUTO_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "分区2");
							}
							else if(SYS_HAND_AUTO_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "联动启动");
							}
							else if(FEEDBK1_Package_ID == read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id)
							{
								sprintf((char *)show_buff, "反馈1");
							}
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id != 0)
						{
							sprintf((char *)show_buff, "第%d簇%dpack", 
								read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id,
								read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id);
						}
						else if(read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cluster_id == 0 && 
										read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id != 0)
						{
							sprintf((char *)show_buff, "第1回路 %d号", read_data[x_sector].fs_gas_outfires[data_index].fs_detect_id.cabin_or_pack_id);
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //刷新设备名称

						// 显示时间
						// 2025/11/19 15:39 修改
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_gas_outfires[data_index].fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d年%02d月%02d日 %02d时%02d分%02d秒", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);
			
						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //刷新时间

						// 状态显示
						switch(read_data[x_sector].fs_gas_outfires[data_index].state)
						{
							case OUTFIRE1OPEN_1:
								sprintf((char *)show_buff, "灭火装置第一次启动");
								break;
							case OUTFIRE1CLOSE:
								sprintf((char *)show_buff, "灭火装置关闭");
								break;
							case OUTFIRE1OPEN_2:
								sprintf((char *)show_buff, "灭火装置第二次启动");
								break;
							case OUTFIRE1OPEN_3:
								sprintf((char *)show_buff, "灭火装置第三次启动");
								break;
							case OUTFIRE_1_START_DELAY:
								sprintf((char *)show_buff, "灭火装置第一次启动倒计时开始");
								break;
							case OUTFIRE_2_START_DELAY:
								sprintf((char *)show_buff, "灭火装置第二次启动倒计时开始");
								break;
							case OUTFIRE_3_START_DELAY:
								sprintf((char *)show_buff, "灭火装置第三次启动倒计时开始");
								break;
							case OUTFIRE_STOP  :
								sprintf((char *)show_buff, "灭火装置强制停止");
								break;
							case OUTFIRESTART_AGAIN:
								sprintf((char *)show_buff, "灭火装置再次启动");
								break;
							case OUTFIRE_ST_PRESS:
								sprintf((char *)show_buff, "灭火装置启动按键按下");
								break;
							case OUTFIRE_SP_PRESS:
								sprintf((char *)show_buff, "灭火装置停止按键按下");
								break;
							case OUTFIRE_SL_PRESS:
								sprintf((char *)show_buff, "声光按键按下");
								break;
							case OUTFIRE_OVER :
								sprintf((char *)show_buff, "灭火装置喷放完成");
								break;
							case OTHER_PART1_TURN_AUTO:
								sprintf((char *)show_buff, "分区1切换为自动");
								break;
							case OTHER_PART1_TURN_HAND:
								sprintf((char *)show_buff, "分区1切换为手动");
								break;
							case OTHER_PART2_TURN_AUTO:
								sprintf((char *)show_buff, "分区2切换为自动");
								break;
							case OTHER_PART2_TURN_HAND:
								sprintf((char *)show_buff, "分区2切换为手动");
								break;
							case OTHER_SYS_TURN_HAND:
								sprintf((char *)show_buff, "联动启动切换为手动");
								break;
							case OTHER_SYS_TURN_AUTO:
								sprintf((char *)show_buff, "联动启动切换为自动");
								break;
							case OUTFIRE_FEEDBACK1:
								sprintf((char *)show_buff, "反馈1触发");
								break;
							default:
								sprintf((char *)show_buff,"未知状态");
								break;
						}

						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //刷新状态
						
						break;
					}
					case RECORD_OTHER: { // 其它记录分区
						// 临时扇区
//						x_sector = total_records/500;
//						data_index = (total_records - 1 - temp_caculate)%500; // 计算数据索引
						x_sector = reverse_index/500;
						data_index = reverse_index%500;
						
//						DebugPrintf("%d\r\n", total_records);
//						
						//DebugSendString(read_data[x_sector].byte_buff, total_records * 8);
						
						// 显示序号
						SetTextInt32(temp_screen_id, serial_ctrl_id[i], temp_caculate + 1, 0, 1);   //
					
						// 设备名称判断
						// 如果是外联设备
						if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cluster_id == LINKAGE_CLUSTER_ID)
						{
							if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_RESET_Package_ID)
							{
								sprintf((char *)show_buff, "系统复位按键");
							}
							else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_MAIN_POWER_KEY_ID)
							{
								sprintf((char *)show_buff, "系统电源按键");
							}
							else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_SELFCHECK_Package_ID)
							{
								sprintf((char *)show_buff, "系统自检按键");
							}
							else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_CHECK_Package_ID)
						{
							sprintf((char *)show_buff, "\xCF\xB5\xCD\xB3\xBC\xEC\xB2\xE9\xB0\xB4\xBC\xFC"); /* XR5000_CHECK_CHANGE_20260804 */
						}
						else if(read_data[x_sector].fs_other_record[data_index].fs_detect_id.cabin_or_pack_id == SYS_TURN_OFF_Package_ID)
							{
								sprintf((char *)show_buff, "系统关机按键");
							}
						}
						else
						{
							sprintf((char *)show_buff, "未知设备");
						}
						SetTextValue(temp_screen_id, device_ctrl_id[i], show_buff); //刷新设备名称
						
						// 显示时间
						// 2025/11/19 15:39 修改
						FlashSaveTime_t temp_time_save = {0};
						
						getFlashTime_Plus(read_data[x_sector].fs_other_record[data_index].fs_time_buff, &temp_time_save);

						sprintf((char *)show_buff, "%02d年%02d月%02d日 %02d时%02d分%02d秒", temp_time_save.years + 2000, temp_time_save.months, 
							temp_time_save.days, temp_time_save.hours , temp_time_save.minute, temp_time_save.second);

						SetTextValue(temp_screen_id , retime_ctrl_id[i], show_buff); //刷新时间
						
						if(read_data[x_sector].fs_other_record[data_index].state == OTHER_SYS_RESET)
						{
							sprintf((char *)show_buff, "系统复位成功");
						}
						else if(read_data[x_sector].fs_other_record[data_index].state == OTHER_TURN_ON)
						{
							sprintf((char *)show_buff, "系统开机成功");
						}
						else if(read_data[x_sector].fs_other_record[data_index].state == OTHER_SYS_SELF_CHECK)
						{
							sprintf((char *)show_buff, "系统自检");
						}
						else if(OTHER_SYS_CHECK == read_data[x_sector].fs_other_record[data_index].state)
						{
							sprintf((char *)show_buff, "\xCF\xB5\xCD\xB3\xBC\xEC\xB2\xE9"); /* XR5000_CHECK_CHANGE_20260804 */
						}
						else if(OTHER_TURN_OFF == read_data[x_sector].fs_other_record[data_index].state )
						{
							sprintf((char *)show_buff, "系统关机成功");
						}
						SetTextValue(temp_screen_id, states_ctrl_id[i], show_buff); //刷新状态
						break;
					}
					default:
						break;
				}

			}
			else // 否则清空剩余显示
			{
				clearTextValue(temp_screen_id , serial_ctrl_id[i]); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , device_ctrl_id[i]); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , retime_ctrl_id[i]); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , states_ctrl_id[i]); //(画面ID,控件ID）
				if(bsrr_entry->curr_show_type == RECORD_ALARM)
				{
					clearTextValue(temp_screen_id , values_ctrl_id[i]); //(画面ID,控件ID）
				}
			}
		} // for循环刷新显示的括号
	} // 判断是否刷新的括号
}

// 该函数是用来给FLASH存报警记录的
static void BspAlarmDataSaveApp(
	FlashReadCtrlId addr_type, // 存储地址
	FlashSaveType save_type,   // 存储类型
	uint8_t cluster_id,        // 簇号
	uint8_t pack_or_cabin,     // 包号
	uint16_t val               // 值
)
{
	FlashSaveFireAlarm_t temp_data = {0};

	// 设备号赋值
	temp_data.fs_base.fs_detect_id.cluster_id = cluster_id; // 簇ID
	temp_data.fs_base.fs_detect_id.cabin_or_pack_id = pack_or_cabin; // packID 簇不为0就是pack
	// 时间赋值
	// 2025/11/19 15:39 修改
	
	FlashSaveTimeBuff temp_time = {0};
	// 给数组赋值
	setFlashTime(temp_time, years, months, days, hours, minutes, secs);

	
	
	for(uint8_t i = 0; i < 5; i++)
	{
		temp_data.fs_base.fs_time_buff[i] = temp_time[i];
	}

	// 状态赋值 火警
	temp_data.fs_base.state = save_type;
	// 存储报警温度值
	temp_data.data_high = val;
	
	BspSaveDataToFlash(addr_type, save_type, (void *)&temp_data);
}

static void PowerManageCtrl(uint8_t main_power_state, uint8_t back_power_state)
{
	// 如果备电故障 主电异常
	if(main_power_state == 0 && (back_power_state == open_circuit || back_power_state == short_circuit))
	{
		BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_TURN_OFF, LINKAGE_CLUSTER_ID, SYS_TURN_OFF_Package_ID);
	}
	else
	{
		// 判断主电
		if(main_power_state != 1 && main_power_alarm_flag == 0)
		{
			main_power_beep_ctrl |= (1U << 0);
			silencers_state  = 0;  // 主电异常 关闭消音指示灯 蜂鸣器开始报警
			main_power_alarm_flag = 1;
			
			// 存储到FLASH故障存储区中 主电掉电
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID);
			// 存RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID, DISCONNECT);
			
		}
		else if(main_power_state == 1 && main_power_alarm_flag == 1)
		{
			// 清除更新抑制
			main_power_alarm_flag = 0;
			// 关闭本位蜂鸣器
			main_power_beep_ctrl &= ~(1U << 0);
	//				silencers_state  = 0;  // 关闭消音指示灯 关闭蜂鸣器 // 消音指示灯不应主动熄灭 应在新的故障到来或者系统复位后熄灭
			// 存储到FLASH故障存储区中 主电掉电
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID);
			// 从缓冲区中寻找有存在的故障
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
		}
		
		// 判断备电
		if(back_power_alarm_flag == 0 && back_power_state == open_circuit)
		{
			// 打开蜂鸣器
			main_power_beep_ctrl |= (1U << 1);
			// 更新抑制
			back_power_alarm_flag = 1;
			
			silencers_state  = 0;  // 主电异常 关闭消音指示灯 蜂鸣器开始报警
			// 存储到FLASH故障存储区中 备电掉电
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			// 存RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID, DISCONNECT);
		} // 备电断路时括号
		else if(back_power_alarm_flag == 0 && back_power_state == short_circuit)
		{
			// 打开蜂鸣器
			main_power_beep_ctrl |= (1U << 1);
			// 更新抑制 记录状态
			back_power_alarm_flag = 2;
			silencers_state  = 0;  // 主电异常 关闭消音指示灯 蜂鸣器开始报警
			// 存储到FLASH故障存储区中 备电短路
			BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHORTCIRCUIT, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			// 存RAM
			creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID, SHORTCIRCUIT);
		} // 备电短路时括号
		else if(back_power_alarm_flag != 0 && back_power_state != open_circuit && back_power_state != short_circuit)
		{
			if(back_power_alarm_flag == 1)
			{
				// 存储到FLASH故障存储区中 备电短路
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DIS_RECOVERY, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			}
			else
			{
				// 存储到FLASH故障存储区中 备电短路
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, SHO_RECOVERY, LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID);
			}
			
			main_power_beep_ctrl &= ~(1U << 1);
			
			// 从缓冲区中寻找有存在的故障
			uint8_t temp_index_ = findRecoveryDevice(LINKAGE_CLUSTER_ID, SYS_BACK_POWER_KEY_ID, 0);
			if(0xFF != temp_index_)
			{
				deletRecoveryRecord(temp_index_);
			}
			back_power_alarm_flag = 0;
		} // 备电恢复时括号
	} // 只有一个异常时的判断括号
}

static void HandForceStartAnyCluster(FireExtinguishDeviceActionSave *fedas_entry, uint16_t ctrl_id, uint8_t state)
{
	if(state == 1)
	{
		if(ctrl_id == 41)
		{
			return;
		}
		getBM8563TimeToSystemTime(); // 获取一下RTC时间
		// 创建一条新纪录
		// 记录创建时间
		fedas_entry->atr[fedas_entry->self_point_len].years  = years + 2000;
		fedas_entry->atr[fedas_entry->self_point_len].months = months;
		fedas_entry->atr[fedas_entry->self_point_len].days   = days;
		fedas_entry->atr[fedas_entry->self_point_len].hours  = hours;
		fedas_entry->atr[fedas_entry->self_point_len].minute = minutes;
		fedas_entry->atr[fedas_entry->self_point_len].second = secs;

		fedas_entry->cabin_id[fedas_entry->self_point_len]   = 0;
		fedas_entry->cluster_id[fedas_entry->self_point_len] = ctrl_id - 20;
		fedas_entry->pack_id[fedas_entry->self_point_len]    = 1; // 包ID默认是1
		fedas_entry->fed_action[fedas_entry->self_point_len] = FIRE_EXTINGUISH_START_SPRAY_DELAY; // 启动延时状态
		fedas_entry->countdown_val[fedas_entry->self_point_len] = 30; // 延时时长30秒
		
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
		// 判断是否写入
		if(fan_disconnect_record_flag == 0)
		{
			// 判断结构体中是否有 如果没有 创建新的 
			if(creatNewFaultRecordToCache(LINKAGE_CLUSTER_ID, LINKAGE_FAN_Package_ID, 0) == 0)
			{
				// 如果写入成功 开蜂鸣器 点亮故障LED 
				beep_fault_ctrl  = 2;   // 蜂鸣器开 故障蜂鸣器标志位
				silencers_state  = 0;   // 消音灯灭
				disconnect_state = 1;   // 点亮故障灯
				// 修改为函数存储
				//DebugSendString((uint8_t *)&temp_data, sizeof(FlashSaveDetectFault_t));
				BspCommonDataSaveApp(FAULT_FLASH_SAVE, DISCONNECT, LINKAGE_CLUSTER_ID, LINKAGE_FAN_Package_ID);
			}
			// 标记写入
			fan_disconnect_record_flag = 1;
		}
	}
	else // 如果风机没有掉线
	{
		if(fan_disconnect_record_flag == 1)
		{
			// 从缓冲区中寻找有存在的故障
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
	// 如果风机是掉线 故障 正在运行 则不用发送启动
	if(fan_sta == fan_disconnect || fan_sta == fan_break || fan_sta == fan_run)
	{
//		return;
	}
	// 两秒一发 判断风机状态发送指令 避免队列溢出
	if(baojingjishi - fan_start_ticks >= 2)
	{
		fan_start_ticks = baojingjishi;
		if(fan_sta == fan_stop && early_aralm_num != 0 && fire_alarm_num == 0)
		{
			if(getSysHandAutoState() == KEY_MANUAL) // 手动状态需要手动启动 风机
			{
				if(linkage_start_key_press_flag == 1)
				{
					// 发送风机启动
					Fan1CtrlOpen();
					Fan2CtrlOpen();
					linkage_start_key_press_flag = 0;
					SysStartStateLedCtrl(LED_ON);
				}
			}
			else
			{
				// 发送风机启动
				Fan1CtrlOpen();
				Fan2CtrlOpen();
				SysStartStateLedCtrl(LED_ON);
			}
			
		}
		// 如果风机正在工作 并且预警 火警都为0 关闭风机
		if(fan_sta == fan_run && early_aralm_num == 0 && fire_alarm_num == 0)
		{
			Fan1CtrlClose();
			Fan2CtrlClose();
		}
		

		// 发送风机关闭
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
		if(fan_send_counts < 10 && fire_alarm_num != 0) // 如果发送数量小于10并且
		{
			// 如果风机正在工作 并且出现火警 关闭风机
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

// 0 没有报警 1有报警 2已经触发仓气体灭火
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
                /* 黑匣子:温度故障恢复 */
                if(old_state == 3U) StorageEvent_LogFault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 1);
                if(old_state == 3U) FecbusReport_Fault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 1); /* FECbus:温度故障恢复 */
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
                    /* 黑匣子:记录Loop1温度火警(自动判定首警) */
                    StorageEvent_LogFire(addr, DEV_TYPE_TEMPERATURE, 1, 0);
                    FecbusReport_Fire(addr, DEV_TYPE_TEMPERATURE, 1, 0); /* FECbus:温度火警 */
                }
                else if(raw_state == 3U)
                {
                    Loop1AddFault(addr, LOOP1_FAULT_TEMPERATURE, LOOP1_TEMP_SENSOR_FAULT);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_TEMP, MBUS_FIRE_DISPLAY_ALARM_FAULT);
                    /* 黑匣子:温度传感器故障 */
                    StorageEvent_LogFault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 0);
                    FecbusReport_Fault(addr, DEV_TYPE_TEMPERATURE, 1, 0, 0); /* FECbus:温度故障 */
                }
            }
            else
            {
                if(old_state == 1U) Loop1RemoveWarning(addr, Loop1SmokeWarning, LOOP1_SMOKE_WARNING_RECOVERY, value);
                if(old_state == 8U) Loop1RemoveFault(addr, LOOP1_FAULT_SMOKE_POLLUTION, LOOP1_SMOKE_POLLUTION_RECOVERY);
                if(old_state == 9U) Loop1RemoveFault(addr, LOOP1_FAULT_SMOKE_SENSOR, LOOP1_SMOKE_SENSOR_RECOVERY);
                /* 黑匣子:烟雾污染/传感器故障恢复 */
                if(old_state == 8U) StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 1);
                if(old_state == 9U) StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 1);
                if(old_state == 8U) FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 1); /* FECbus:烟雾污染恢复 */
                if(old_state == 9U) FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 1); /* FECbus:烟雾传感器恢复 */
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
                    /* 黑匣子:记录Loop1烟雾火警(自动判定首警) */
                    StorageEvent_LogFire(addr, DEV_TYPE_SMOKE, 1, 0);
                    FecbusReport_Fire(addr, DEV_TYPE_SMOKE, 1, 0); /* FECbus:烟雾火警 */
                }
                else if(raw_state == 8U)
                {
                    Loop1AddFault(addr, LOOP1_FAULT_SMOKE_POLLUTION, LOOP1_SMOKE_POLLUTION_FAULT);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_SMOKE, MBUS_FIRE_DISPLAY_ALARM_FAULT);
                    /* 黑匣子:烟雾污染故障 */
                    StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 0);
                    FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 0); /* FECbus:烟雾污染故障 */
                }
                else if(raw_state == 9U)
                {
                    Loop1AddFault(addr, LOOP1_FAULT_SMOKE_SENSOR, LOOP1_SMOKE_SENSOR_FAULT);
                    MBusCtrl_PostFireDisplayEvent(1U, addr, MBUS_FIRE_DISPLAY_DETECT_SMOKE, MBUS_FIRE_DISPLAY_ALARM_FAULT);
                    /* 黑匣子:烟雾传感器故障 */
                    StorageEvent_LogFault(addr, DEV_TYPE_SMOKE, 1, 0, 0);
                    FecbusReport_Fault(addr, DEV_TYPE_SMOKE, 1, 0, 0); /* FECbus:烟雾传感器故障 */
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
uint8_t last_point_type_found_online_state = 1; // 默认必须刷新一次
static void PointTypeDetectorShowApp(PointTypeShowCtrl_t *ptsc_entry)
{
	uint8_t temp_screen_id = 67;
	
	ptsc_entry->curr_fresh_time_ctrl = osKernelGetTickCount(); // 系统当前时间戳
	
	if(ptsc_entry->curr_fresh_time_ctrl - ptsc_entry->last_fresh_time_ctrl >= 3000 || ptsc_entry->poll_show_ctrl.key_perss_fresh != 0) // 三秒一刷新
	{
		uint8_t show_addr;
		
		uint8_t found_online = 0;
		
		ptsc_entry->last_fresh_time_ctrl = ptsc_entry->curr_fresh_time_ctrl; // 记录刷新时间
		
		// 先将值保存给上次轮询值
		ptsc_entry->poll_show_ctrl.last_detector_id = ptsc_entry->poll_show_ctrl.poll_circuits_id;

		if(ptsc_entry->poll_show_ctrl.key_perss_fresh == 'p')
		{
			for(uint8_t test_addr = 0U; test_addr < MIXTURE_DEVICE_MAX_ADDR; test_addr++)
			{
				ptsc_entry->poll_show_ctrl.poll_circuits_id--; // 遍历数组 直到找到上线的数组
				
				if(ptsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || ptsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					ptsc_entry->poll_show_ctrl.poll_circuits_id = MIXTURE_DEVICE_MAX_ADDR;
				}
				
				if(getPointTypeMixtureDetectOnlineState( ptsc_entry->poll_show_ctrl.poll_circuits_id ) == 1)
				{
					found_online = 1;
					last_point_type_found_online_state = 1; // 如果发现上线该状态设为1 用来防止每次都没有上线时重复刷新屏幕
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
				ptsc_entry->poll_show_ctrl.poll_circuits_id++; // 遍历数组 直到找到上线的数组
				
				if(ptsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || ptsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					ptsc_entry->poll_show_ctrl.poll_circuits_id = 1;
				}
				
				if(getPointTypeMixtureDetectOnlineState( ptsc_entry->poll_show_ctrl.poll_circuits_id ) == 1)
				{
					found_online = 1;
					last_point_type_found_online_state = 1; // 如果发现上线该状态设为1 用来防止每次都没有上线时重复刷新屏幕
					show_addr = ptsc_entry->poll_show_ctrl.poll_circuits_id;

					break;
				}
			}
			
			ptsc_entry->poll_show_ctrl.key_perss_fresh = 0;
		}

		if(found_online == 0 && last_point_type_found_online_state != 0) // 如果没有发现上线的探测器
		{
			last_point_type_found_online_state = 0; // 更新抑制
			ptsc_entry->poll_show_ctrl.poll_circuits_id = ptsc_entry->poll_show_ctrl.last_detector_id; // 从上次轮询值开始
			
			// 更新屏幕显示
			SetTextValue(temp_screen_id, 33, "无任何探测器启用"); //刷新状态
			clearTextValue(temp_screen_id , 34); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 35); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 36); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 37); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 38); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 39); //(画面ID,控件ID）
		}
		else if(found_online != 0) // 发现有探测器上线 才更新显示
		{
			uint8_t temp_detect_type;
			
			uint8_t temp_buff[64] = {0};
			
			if(getPointTypeMixtureDisconnectCount(show_addr) < MIXTURE_DEVICE_DISCONNECT_SUM) // 探测器在线
			{
//				if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 无温度报警 | 无烟雾报警", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 无温度报警 | 烟雾报警", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 温度报警 | 无烟雾报警", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 温度报警 | 烟雾报警", show_addr);
//				}
				sprintf((char *)temp_buff, "探测器%d状态:在线", show_addr);
				SetTextValue(temp_screen_id, 33, temp_buff); //刷新状态
				
				// 显示探测器型号
				temp_detect_type = getPointTypeMixtureDetectName(show_addr);
				switch(temp_detect_type)
				{
					case 1: {
						SetTextValue(temp_screen_id, 34, "探测器型号:XR805-V2.0"); //刷新状态
						break;
					}
					case 2: {
						SetTextValue(temp_screen_id, 34, "探测器型号:XR805-EXD"); //刷新状态
						break;
					}
					case 3: {
						SetTextValue(temp_screen_id, 34, "探测器型号:XR805-EXi"); //刷新状态
						break;
					}
					case 4: {
						SetTextValue(temp_screen_id, 34, "探测器型号:XR-DLYGWG"); //刷新状态
						break;
					}
					case 5: {
						SetTextValue(temp_screen_id, 34, "探测器型号:JTY-XR800B"); //刷新状态
						break;
					}
					case 6: {
						SetTextValue(temp_screen_id, 34, "探测器型号:JTY-ZDM-XR8002/C"); //刷新状态
						break;
					}
					case 7: {
						SetTextValue(temp_screen_id, 34, "探测器型号:JTY-GD-XR8001AI"); //刷新状态
						break;
					}
					default: {
						SetTextValue(temp_screen_id, 34, "探测器型号:--"); //刷新状态
						break;
					}
				}
				
				// 显示探测器传感器启用状态
				uint8_t sensor_enable_state = getPointTypeMixtureDetectType(show_addr);
				if(sensor_enable_state == 0)
				{
					SetTextValue(temp_screen_id, 35, "传感器启用状态:无传感器启动");
				}
				else
				{
					uint8_t first_sensor = 1;  // 标记是否是第一个传感器
					uint8_t pos = 0;
					
					pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
					for(uint8_t i = 0; i < 6; i++)
					{
						if( (sensor_enable_state >> i) & 0x01 )
						{
							// 如果不是第一个，添加分隔符
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
				
				clearTextValue(temp_screen_id , 36); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 37); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 38); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 39); //(画面ID,控件ID）
				
				// 根据启用状态显示值
				uint8_t screen_show_id_offset = 0;
				if(sensor_enable_state & 0x20) // 判断温度是否启用
				{
					sprintf((char *)temp_buff, "温度值:%d度", getPointTypeMixtureReceiveData(PointTypeData_Temper, show_addr));
					SetTextValue(temp_screen_id, 36 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // 修改地址偏移 
				}
				if(sensor_enable_state & 0x01) // 判断烟雾是否启用
				{
					if(getPointTypeMixtureReceiveState(PointTypeData_Smoke, show_addr) == 1)
					{
						SetTextValue(temp_screen_id, 36 + screen_show_id_offset, "烟雾状态:报警");
					}
					else
					{
						SetTextValue(temp_screen_id, 36 + screen_show_id_offset, "烟雾状态:正常");
					}
					screen_show_id_offset++; // 修改地址偏移 
				}
				if(sensor_enable_state & 0x10)
				{
					
					screen_show_id_offset++; // 修改地址偏移 
				}
				
				
			}
			else // 否则掉线
			{
				sprintf((char *)temp_buff, "探测器%d状态:掉线", show_addr);
				SetTextValue(temp_screen_id, 33, temp_buff); //刷新状态
				SetTextValue(temp_screen_id, 34, "探测器型号:--"); //刷新状态
				SetTextValue(temp_screen_id, 35, "传感器启用状态:--");
				clearTextValue(temp_screen_id , 36); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 37); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 38); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 39); //(画面ID,控件ID）
			}
		} // 找到有设置上线的探测器
	} // 刷新计时到了 刷新一次
	
	// 查询刷新
	if(ptsc_entry->verb_show_ctrl.verb_detector_id == 0) // 如果是0 没有查询ID
	{
		ptsc_entry->verb_show_ctrl.verb_detector_id = 255; // 更新抑制
		SetTextValue(temp_screen_id, 23, "请输入查询探测器ID"); //刷新状态
		clearTextValue(temp_screen_id , 24); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 25); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 26); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 27); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 28); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 29); //(画面ID,控件ID）
		SetTextValue(temp_screen_id, 30, "此处输入ID"); //刷新状态
	}
	else if(ptsc_entry->verb_show_ctrl.verb_detector_id != 255) // 如果有正确ID输入 且不是更新抑制的值
	{
		uint8_t show_addr = ptsc_entry->verb_show_ctrl.verb_detector_id;
		
		if(getPointTypeMixtureDetectOnlineState( show_addr ) == 1) // 如果设置上线了
		{
			uint8_t temp_buff[64] = {0};
			// 判断探测器是否在线
			if(getPointTypeMixtureDisconnectCount(show_addr) < MIXTURE_DEVICE_DISCONNECT_SUM)
			{
				// 如果id变更
				if(show_addr != ptsc_entry->verb_show_ctrl.last_detector_id || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.last_detector_id = show_addr; // 更新抑制
					
					sprintf((char *)temp_buff, "探测器%d状态:在线", show_addr);
					SetTextValue(temp_screen_id, 23, temp_buff); //刷新状态
				}
				
				ptsc_entry->verb_show_ctrl.verb_detect_name = getPointTypeMixtureDetectName(show_addr);
				
				if(ptsc_entry->verb_show_ctrl.lsat_detect_name != ptsc_entry->verb_show_ctrl.verb_detect_name || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.lsat_detect_name = ptsc_entry->verb_show_ctrl.verb_detect_name;
					
					// 显示探测器型号
					switch(ptsc_entry->verb_show_ctrl.verb_detect_name)
					{
						case 1: {
							SetTextValue(temp_screen_id, 24, "探测器型号:XR805-V2.0"); //刷新状态
							break;
						}
						case 2: {
							SetTextValue(temp_screen_id, 24, "探测器型号:XR805-EXD"); //刷新状态
							break;
						}
						case 3: {
							SetTextValue(temp_screen_id, 24, "探测器型号:XR805-EXi"); //刷新状态
							break;
						}
						case 4: {
							SetTextValue(temp_screen_id, 24, "探测器型号:XR-DLYGWG"); //刷新状态
							break;
						}
						case 5: {
							SetTextValue(temp_screen_id, 24, "探测器型号:JTY-XR800B"); //刷新状态
							break;
						}
						case 6: {
							SetTextValue(temp_screen_id, 24, "探测器型号:JTY-ZDM-XR8002/C"); //刷新状态
							break;
						}
						case 7: {
							SetTextValue(temp_screen_id, 24, "探测器型号:JTY-GD-XR8001AI"); //刷新状态
							break;
						}
						default: {
							SetTextValue(temp_screen_id, 24, "探测器型号:--"); //刷新状态
							break;
						}
					}
				}
				
				// 显示探测器传感器启用状态
				ptsc_entry->verb_show_ctrl.verb_sensor_state = getPointTypeMixtureDetectType(show_addr);
				
				if(ptsc_entry->verb_show_ctrl.last_sensor_state != ptsc_entry->verb_show_ctrl.verb_sensor_state || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.last_sensor_state = ptsc_entry->verb_show_ctrl.verb_sensor_state;
					
					if(ptsc_entry->verb_show_ctrl.verb_sensor_state == 0)
					{
						SetTextValue(temp_screen_id, 25, "传感器启用状态:无传感器启动");
					}
					else
					{
						uint8_t first_sensor = 1;  // 标记是否是第一个传感器
						uint8_t pos = 0;
						
						pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
						for(uint8_t i = 0; i < 6; i++)
						{
							if( (ptsc_entry->verb_show_ctrl.verb_sensor_state >> i) & 0x01 )
							{
								// 如果不是第一个，添加分隔符
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
					
					clearTextValue(temp_screen_id , 26); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 27); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 28); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 29); //(画面ID,控件ID）
				}

				// 根据启用状态显示值
				uint8_t screen_show_id_offset = 0;
				if(ptsc_entry->verb_show_ctrl.verb_sensor_state & 0x20) // 判断温度是否启用
				{
					ptsc_entry->verb_show_ctrl.verb_temper_value = getPointTypeMixtureReceiveData(PointTypeData_Temper, show_addr);
					if(ptsc_entry->verb_show_ctrl.verb_temper_value != ptsc_entry->verb_show_ctrl.last_temper_value || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						ptsc_entry->verb_show_ctrl.last_temper_value = ptsc_entry->verb_show_ctrl.verb_temper_value;
						sprintf((char *)temp_buff, "温度值:%d度", ptsc_entry->verb_show_ctrl.verb_temper_value);
						SetTextValue(temp_screen_id, 26 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // 修改地址偏移 
					}
				}
				if(ptsc_entry->verb_show_ctrl.verb_sensor_state & 0x01) // 判断烟雾是否启用
				{
					ptsc_entry->verb_show_ctrl.verb_smokes_state = getPointTypeMixtureReceiveState(PointTypeData_Smoke, show_addr);
					
					if(ptsc_entry->verb_show_ctrl.verb_smokes_state != ptsc_entry->verb_show_ctrl.last_smokes_state || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						ptsc_entry->verb_show_ctrl.last_smokes_state = ptsc_entry->verb_show_ctrl.verb_smokes_state;
						
						if(ptsc_entry->verb_show_ctrl.verb_smokes_state == 1)
						{
							SetTextValue(temp_screen_id, 26 + screen_show_id_offset, "烟雾状态:报警");
						}
						else
						{
							SetTextValue(temp_screen_id, 26 + screen_show_id_offset, "烟雾状态:正常");
						}
						screen_show_id_offset++; // 修改地址偏移 
					}
				}
				if(ptsc_entry->verb_show_ctrl.verb_sensor_state & 0x10)
				{
					
					screen_show_id_offset++; // 修改地址偏移 
				}
			}
			else // 设置启用 但掉线了
			{
				if(show_addr != ptsc_entry->verb_show_ctrl.last_detector_id || ptsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					ptsc_entry->verb_show_ctrl.last_detector_id = show_addr; // 更新抑制
					
					sprintf((char *)temp_buff, "探测器%d状态:掉线", show_addr);
					SetTextValue(temp_screen_id, 23, temp_buff); //刷新状态
					SetTextValue(temp_screen_id, 24, "探测器型号:--"); //刷新状态
					SetTextValue(temp_screen_id, 25, "传感器启用状态:--");
					clearTextValue(temp_screen_id , 26); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 27); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 28); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 29); //(画面ID,控件ID）
				}
			}
			ptsc_entry->verb_show_ctrl.force_fresh_ctrl = 0;
		}
		else // 没设置上线
		{
			ptsc_entry->verb_show_ctrl.verb_detector_id = 255; // 更新抑制
			SetTextValue(temp_screen_id, 23, "该探测器未启用"); //刷新状态
			clearTextValue(temp_screen_id , 24); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 25); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 26); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 27); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 28); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 29); //(画面ID,控件ID）
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
	sscanf((const char*)str,"%ld",&value); //把字符串转换为整数 
	if(control_id == 30)    
	{
		ptsc_entry->verb_show_ctrl.verb_detector_id = value;
		
		ptsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // 强制刷新一次
	}
}

static void PointTypeDetectorScreenSwitchShowApp(PointTypeShowCtrl_t *ptsc_entry)
{
	ptsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // 进入界面后强制刷新一次
}

// 复合探测器刷新
uint8_t last_composite_found_online_state = 1; // 如果都没上线开局强制刷新一次
void CompositeDetectorPollCtrl(CompositeShowCtrl_t *cpsc_entry)
{
	uint8_t temp_screen_id = 67;
	
	cpsc_entry->curr_fresh_time_ctrl = osKernelGetTickCount(); // 系统当前时间戳
	
	if(cpsc_entry->curr_fresh_time_ctrl - cpsc_entry->last_fresh_time_ctrl >= 3000 || cpsc_entry->poll_show_ctrl.key_perss_fresh != 0) // 三秒一刷新
	{
		uint8_t show_addr;
		
		uint8_t found_online = 0;
		
		cpsc_entry->last_fresh_time_ctrl = cpsc_entry->curr_fresh_time_ctrl; // 记录刷新时间
		
		// 先将值保存给上次轮询值
		cpsc_entry->poll_show_ctrl.last_detector_id = cpsc_entry->poll_show_ctrl.poll_circuits_id;

		if(cpsc_entry->poll_show_ctrl.key_perss_fresh == 'p')
		{
			for(uint8_t test_addr = 0U; test_addr < MIXTURE_DEVICE_MAX_ADDR; test_addr++)
			{
				cpsc_entry->poll_show_ctrl.poll_circuits_id--; // 遍历数组 直到找到上线的数组
				
				if(cpsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || cpsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					cpsc_entry->poll_show_ctrl.poll_circuits_id = MIXTURE_DEVICE_MAX_ADDR;
				}
				
				// 此处修改为 复合仓上线判断状态 筛选出来上线的探测器判断
				if( cang_sxzt[ cpsc_entry->poll_show_ctrl.poll_circuits_id ] == 1 )
				{
					found_online = 1;
					last_composite_found_online_state = 1; // 如果发现上线该状态设为1 用来防止每次都没有上线时重复刷新屏幕
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
				cpsc_entry->poll_show_ctrl.poll_circuits_id++; // 遍历数组 直到找到上线的数组
				
				if(cpsc_entry->poll_show_ctrl.poll_circuits_id > MIXTURE_DEVICE_MAX_ADDR || cpsc_entry->poll_show_ctrl.poll_circuits_id < 1)
				{
					cpsc_entry->poll_show_ctrl.poll_circuits_id = 1;
				}
				
				if( cang_sxzt[ cpsc_entry->poll_show_ctrl.poll_circuits_id ] == 1 )
				{
					found_online = 1;
					last_composite_found_online_state = 1; // 如果发现上线该状态设为1 用来防止每次都没有上线时重复刷新屏幕
					show_addr = cpsc_entry->poll_show_ctrl.poll_circuits_id;

					break;
				}
			}
			
			cpsc_entry->poll_show_ctrl.key_perss_fresh = 0;
		}

		if(found_online == 0 && last_composite_found_online_state != 0) // 如果没有发现上线的探测器
		{
			last_composite_found_online_state = 0; // 更新抑制
			cpsc_entry->poll_show_ctrl.poll_circuits_id = cpsc_entry->poll_show_ctrl.last_detector_id; // 从上次轮询值开始
			
			// 更新屏幕显示
			SetTextValue(temp_screen_id, 15, "无任何探测器启用"); //刷新状态
			clearTextValue(temp_screen_id , 16); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 17); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 18); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 19); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 20); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 21); //(画面ID,控件ID）
		}
		else if(found_online != 0) // 发现有探测器上线 才更新显示
		{
			uint8_t temp_buff[64] = {0};
			
			// 此处修改为 判断复合仓是否掉线
			if( Cang_zx_buf[ cpsc_entry->poll_show_ctrl.poll_circuits_id ] < CabinDisconnectCount ) // 探测器在线
			{
//				if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 无温度报警 | 无烟雾报警", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) == 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 无温度报警 | 烟雾报警", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) == 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 温度报警 | 无烟雾报警", show_addr);
//				}
//				else if(getPointTypeMixtureDetectSmokeMemory(show_addr) != 0 && getPointTypeMixtureDetectTempertureMemory(show_addr) != 0)
//				{
//					sprintf((char *)temp_buff, "探测器%d状态:在线 | 温度报警 | 烟雾报警", show_addr);
//				}
				sprintf((char *)temp_buff, "探测器%d状态:在线", show_addr);
				SetTextValue(temp_screen_id, 15, temp_buff); //刷新状态
				
				// 显示探测器型号 此处修改为 805xxx复合探测器类型
				switch( Cang_TCQXH_buf[ show_addr ] )
				{
					case 1: {
						SetTextValue(temp_screen_id, 16, "探测器型号:XR805-V2.0"); //刷新状态
						break;
					}
					case 2: {
						SetTextValue(temp_screen_id, 16, "探测器型号:XR805-EXD"); //刷新状态
						break;
					}
					case 3: {
						SetTextValue(temp_screen_id, 16, "探测器型号:XR805-EXi"); //刷新状态
						break;
					}
					case 4: {
						SetTextValue(temp_screen_id, 16, "探测器型号:XR-DLYGWG"); //刷新状态
						break;
					}
					case 5: {
						SetTextValue(temp_screen_id, 16, "探测器型号:JTY-XR800B"); //刷新状态
						break;
					}
					case 6: {
						SetTextValue(temp_screen_id, 16, "探测器型号:JTY-ZDM-XR8002/C"); //刷新状态
						break;
					}
					case 7: {
						SetTextValue(temp_screen_id, 16, "探测器型号:JTY-GD-XR8001AI"); //刷新状态
						break;
					}
					default: {
						SetTextValue(temp_screen_id, 16, "探测器型号:--"); //刷新状态
						break;
					}
				}

				// 显示探测器传感器启用状态 此处修改为 805xxx复合探测器类型
				uint8_t sensor_enable_state = Cang_CGQQY_buf[ show_addr ];
				if(sensor_enable_state  == 0 )
				{
					SetTextValue(temp_screen_id, 17, "传感器启用状态:无传感器启动");
				}
				else
				{
					uint8_t first_sensor = 1;  // 标记是否是第一个传感器
					uint8_t pos = 0;
					
					pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
					for(uint8_t i = 0; i < 6; i++)
					{
						if( (sensor_enable_state >> i) & 0x01 )
						{
							// 如果不是第一个，添加分隔符
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
				
				clearTextValue(temp_screen_id , 18); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 19); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 20); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 21); //(画面ID,控件ID）
	
				// 根据启用状态显示值
				uint8_t screen_show_id_offset = 0;
				if(sensor_enable_state & 0x20) // 判断温度是否启用
				{
					// 此处修改为 805xxx复合探测器类型
					sprintf((char *)temp_buff, "温度值:%d度", Cang_wendu_buf[ show_addr ] );
					SetTextValue(temp_screen_id, 18 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // 修改地址偏移 
				}
				if(sensor_enable_state & 0x01) // 判断烟雾是否启用
				{
					if(Cang_YWZT_buf[ show_addr ] == 1)
					{
						SetTextValue(temp_screen_id, 18 + screen_show_id_offset, "烟雾状态:报警");
					}
					else
					{
						SetTextValue(temp_screen_id, 18 + screen_show_id_offset, "烟雾状态:正常");
					}
					screen_show_id_offset++; // 修改地址偏移 
				}
				if(sensor_enable_state & 0x10) // 判断一氧化碳寄存器是否启用
				{
					// 此处修改为 805xxx复合探测器类型
					sprintf((char *)temp_buff, "一氧化碳浓度:%dPPM", Cang_COzhi_buf[ show_addr ] );
					SetTextValue(temp_screen_id, 18 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // 修改地址偏移 
				}
				if(sensor_enable_state & 0x04) // 判断氢气是否启用
				{
					// 此处修改为 805xxx复合探测器类型
					sprintf((char *)temp_buff, "氢气浓度:%dPPM", Cang_H2zhi_buf[show_addr] );
					SetTextValue(temp_screen_id, 18 + screen_show_id_offset, temp_buff);
					screen_show_id_offset++; // 修改地址偏移 
				}
			}
			else // 否则掉线
			{
				sprintf((char *)temp_buff, "探测器%d状态:掉线", show_addr);
				SetTextValue(temp_screen_id, 15, temp_buff); //刷新状态
				SetTextValue(temp_screen_id, 16, "探测器型号:--"); //刷新状态
				SetTextValue(temp_screen_id, 17, "传感器启用状态:--");
				clearTextValue(temp_screen_id , 18); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 19); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 20); //(画面ID,控件ID）
				clearTextValue(temp_screen_id , 21); //(画面ID,控件ID）
			}
		} // 找到有设置上线的探测器
	} // 刷新计时到了 刷新一次
}

void CompositeDetectorVerbCtrl(CompositeShowCtrl_t *cpsc_entry)
{
	uint8_t temp_screen_id = 67;
	// 查询刷新
	if(cpsc_entry->verb_show_ctrl.verb_detector_id == 0) // 如果是0 没有查询ID
	{
		cpsc_entry->verb_show_ctrl.verb_detector_id = 255; // 更新抑制
		SetTextValue(temp_screen_id, 5, "请输入查询探测器ID"); //刷新状态
		clearTextValue(temp_screen_id , 6); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 7); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 8); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 9); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 10); //(画面ID,控件ID）
		clearTextValue(temp_screen_id , 11); //(画面ID,控件ID）
		
		SetTextValue(temp_screen_id, 12, "此处输入ID"); //刷新状态
	}
	else if(cpsc_entry->verb_show_ctrl.verb_detector_id != 255) // 如果有正确ID输入 且不是更新抑制的值
	{
		uint8_t show_addr = cpsc_entry->verb_show_ctrl.verb_detector_id;
		
		// 1 修改
		if( cang_sxzt[ show_addr ] == 1 ) // 如果设置上线了
		{
			uint8_t temp_buff[64] = {0};
			// 判断探测器是否在线 2 修改
			if( Cang_zx_buf[ show_addr ] < CabinDisconnectCount )
			{
				// 如果id变更
				if(show_addr != cpsc_entry->verb_show_ctrl.last_detector_id || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.last_detector_id = show_addr; // 更新抑制
					
					sprintf((char *)temp_buff, "探测器%d状态:在线", show_addr);
					SetTextValue(temp_screen_id, 5, temp_buff); //刷新状态
				}
				
				// 3 修改
				cpsc_entry->verb_show_ctrl.verb_detect_name = Cang_TCQXH_buf[ show_addr ];
				
				if(cpsc_entry->verb_show_ctrl.lsat_detect_name != cpsc_entry->verb_show_ctrl.verb_detect_name || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.lsat_detect_name = cpsc_entry->verb_show_ctrl.verb_detect_name;
					
					// 显示探测器型号
					switch(cpsc_entry->verb_show_ctrl.verb_detect_name)
					{
						case 1: {
							SetTextValue(temp_screen_id, 6, "探测器型号:XR805-V2.0"); //刷新状态
							break;
						}
						case 2: {
							SetTextValue(temp_screen_id, 6, "探测器型号:XR805-EXD"); //刷新状态
							break;
						}
						case 3: {
							SetTextValue(temp_screen_id, 6, "探测器型号:XR805-EXi"); //刷新状态
							break;
						}
						case 4: {
							SetTextValue(temp_screen_id, 6, "探测器型号:XR-DLYGWG"); //刷新状态
							break;
						}
						case 5: {
							SetTextValue(temp_screen_id, 6, "探测器型号:JTY-XR800B"); //刷新状态
							break;
						}
						case 6: {
							SetTextValue(temp_screen_id, 6, "探测器型号:JTY-ZDM-XR8002/C"); //刷新状态
							break;
						}
						case 7: {
							SetTextValue(temp_screen_id, 6, "探测器型号:JTY-GD-XR8001AI"); //刷新状态
							break;
						}
						default: {
							SetTextValue(temp_screen_id, 6, "探测器型号:--"); //刷新状态
							break;
						}
					}
				}
				
				// 显示探测器传感器启用状态 4 修改
				cpsc_entry->verb_show_ctrl.verb_sensor_state = Cang_CGQQY_buf[ show_addr ];
				
				if(cpsc_entry->verb_show_ctrl.last_sensor_state != cpsc_entry->verb_show_ctrl.verb_sensor_state || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.last_sensor_state = cpsc_entry->verb_show_ctrl.verb_sensor_state;
					
					if(cpsc_entry->verb_show_ctrl.verb_sensor_state == 0)
					{
						SetTextValue(temp_screen_id, 7, "传感器启用状态:无传感器启动");
					}
					else
					{
						uint8_t first_sensor = 1;  // 标记是否是第一个传感器
						uint8_t pos = 0;
						
						pos += snprintf((char *)temp_buff + pos, sizeof(temp_buff) - pos, "%s", sensor_str[6]);
						for(uint8_t i = 0; i < 6; i++)
						{
							if( (cpsc_entry->verb_show_ctrl.verb_sensor_state >> i) & 0x01 )
							{
								// 如果不是第一个，添加分隔符
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
					
					clearTextValue(temp_screen_id , 8); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 9); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 10); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 11); //(画面ID,控件ID）
				}

				// 根据启用状态显示值
				uint8_t screen_show_id_offset = 0;
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x20) // 判断温度是否启用
				{
					cpsc_entry->verb_show_ctrl.verb_temper_value = Cang_wendu_buf[ show_addr ];
					if(cpsc_entry->verb_show_ctrl.verb_temper_value != cpsc_entry->verb_show_ctrl.last_temper_value || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						cpsc_entry->verb_show_ctrl.last_temper_value = cpsc_entry->verb_show_ctrl.verb_temper_value;
						sprintf((char *)temp_buff, "温度值:%d度", cpsc_entry->verb_show_ctrl.verb_temper_value);
						SetTextValue(temp_screen_id, 8 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // 修改地址偏移 
					}
				}
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x01) // 判断烟雾是否启用
				{
					cpsc_entry->verb_show_ctrl.verb_smokes_state = Cang_wendu_buf[ show_addr ];
					
					if(cpsc_entry->verb_show_ctrl.verb_smokes_state != cpsc_entry->verb_show_ctrl.last_smokes_state || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						cpsc_entry->verb_show_ctrl.last_smokes_state = cpsc_entry->verb_show_ctrl.verb_smokes_state;
						
						if(cpsc_entry->verb_show_ctrl.verb_smokes_state == 1)
						{
							SetTextValue(temp_screen_id, 8 + screen_show_id_offset, "烟雾状态:报警");
						}
						else
						{
							SetTextValue(temp_screen_id, 8 + screen_show_id_offset, "烟雾状态:正常");
						}
						screen_show_id_offset++; // 修改地址偏移 
					}
				}
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x10) // 判断一氧化碳寄存器是否启用
				{
					// 此处修改为 805xxx复合探测器类型
					cpsc_entry->verb_show_ctrl.verb_carbon_value = Cang_COzhi_buf[ show_addr ];
					if(cpsc_entry->verb_show_ctrl.last_carbon_value != cpsc_entry->verb_show_ctrl.verb_carbon_value || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						// 添加更新抑制
						cpsc_entry->verb_show_ctrl.last_carbon_value = cpsc_entry->verb_show_ctrl.verb_carbon_value;
						
						sprintf((char *)temp_buff, "一氧化碳浓度:%dPPM", cpsc_entry->verb_show_ctrl.verb_carbon_value );
						SetTextValue(temp_screen_id, 8 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // 修改地址偏移 
					}
				}
				if(cpsc_entry->verb_show_ctrl.verb_sensor_state & 0x04) // 判断氢气是否启用
				{
					
					cpsc_entry->verb_show_ctrl.verb_hydrog_value = Cang_H2zhi_buf[ show_addr ]; 
					
					if(cpsc_entry->verb_show_ctrl.lsat_hydrog_value != cpsc_entry->verb_show_ctrl.verb_hydrog_value || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
					{
						// 更新抑制
						cpsc_entry->verb_show_ctrl.lsat_hydrog_value = cpsc_entry->verb_show_ctrl.verb_hydrog_value;
						
						// 此处修改为 805xxx复合探测器类型
						sprintf((char *)temp_buff, "氢气浓度:%dPPM", cpsc_entry->verb_show_ctrl.verb_hydrog_value );
						SetTextValue(temp_screen_id, 8 + screen_show_id_offset, temp_buff);
						screen_show_id_offset++; // 修改地址偏移 
					}
				}
			}
			else // 设置启用 但掉线了
			{
				if(show_addr != cpsc_entry->verb_show_ctrl.last_detector_id || cpsc_entry->verb_show_ctrl.force_fresh_ctrl == 1)
				{
					cpsc_entry->verb_show_ctrl.last_detector_id = show_addr; // 更新抑制
					
					sprintf((char *)temp_buff, "探测器%d状态:掉线", show_addr);
					SetTextValue(temp_screen_id, 5, temp_buff); //刷新状态
					SetTextValue(temp_screen_id, 6, "探测器型号:--"); //刷新状态
					SetTextValue(temp_screen_id, 7, "传感器启用状态:--");
					clearTextValue(temp_screen_id , 8); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 9); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 10); //(画面ID,控件ID）
					clearTextValue(temp_screen_id , 11); //(画面ID,控件ID）
				}
			}
			cpsc_entry->verb_show_ctrl.force_fresh_ctrl = 0;
		}
		else // 没设置上线
		{
			cpsc_entry->verb_show_ctrl.verb_detector_id = 255; // 更新抑制
			SetTextValue(temp_screen_id, 5, "该探测器未启用"); //刷新状态
			clearTextValue(temp_screen_id , 6); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 7); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 8); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 9); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 10); //(画面ID,控件ID）
			clearTextValue(temp_screen_id , 11); //(画面ID,控件ID）
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
	sscanf((const char*)str,"%ld",&value); //把字符串转换为整数 
	if(control_id == 12)    
	{
		cpsc_entry->verb_show_ctrl.verb_detector_id = value;
		
		cpsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // 强制刷新一次
	}
}

static void CompositeDetectorScreenSwitchShowApp(CompositeShowCtrl_t *cpsc_entry)
{
	cpsc_entry->verb_show_ctrl.force_fresh_ctrl = 1; // 进入界面后强制刷新一次
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
			uart_handle = &huart3; // EMS串口
			break;
		}
		case 5:{
			uart_handle = &huart1; // 场站串口
			break;
		}
		case 6:{
			uart_handle = NULL; /* XR5000_UART5_EXCLUSIVE_FIX_20260730 */ // 控制总线串口
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
			temp_order = EMSSITE; // EMS串口
			break;
		}
		case 5:{
			temp_order = STATION_OPTICALFIBER; // 场站串口
			break;
		}
		case 6:{
			temp_order = ERRORSITE; /* XR5000_UART5_EXCLUSIVE_FIX_20260730 */ // 控制总线串口
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
		SetTextValue(3, 9, "打开串口"); //刷新状态
	}
	else
	{
		SetTextValue(3, 9, "关闭串口"); //刷新状态
	}
	
	if(sspa_entry->serial_port_comid == 0 || sspa_entry->serial_port_comid == 0xFF)
	{
		clearTextValue(3, 6); //(画面ID,控件ID）
	}
	
	if(sspa_entry->serial_port_send_mode == 0)
	{
		SetTextValue(3, 32, "字符串发送"); //刷新状态
	}
	else
	{
		SetTextValue(3, 32, "16进制发送"); //刷新状态
	}
	
	if(sspa_entry->serial_port_show_mode == 0)
	{
		SetTextValue(3, 36, "字符串接收"); //刷新状态
	}
	else
	{
		SetTextValue(3, 36, "16进制接收"); //刷新状态
	}
	
	if(sspa_entry->serial_port_send_new_row == 0)
	{
		SetTextValue(3, 38, "不发送新行"); //刷新状态
	}
	else
	{
		SetTextValue(3, 38, "发送新行"); //刷新状态
	}
	
	if(sspa_entry->serial_port_send_len == 0)
	{
		clearTextValue(3, 28); //(画面ID,控件ID）
	}

}

static void SimulationSerialPortButtonCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t ctrl_id, uint8_t state)
{
	UART_HandleTypeDef *temp_uart = NULL;
	
	switch(ctrl_id)
	{
		case 10: { // 打开串口按键
			if(sspa_entry->serial_port_state == 0) // 如果现在串口是关闭状态
			{
				if(sspa_entry->serial_port_comid != 0xFF && sspa_entry->serial_port_comid != 6U) // 如果端口号正确
				{
					eUartOrder temp_order = ERRORSITE;
					temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
					if(temp_order != ERRORSITE) // 如果是正确的ID
					{
						uartbuff[temp_order].recepetion_flag = 0;
						memset(uartbuff[temp_order].recepetion_buff, 0, BUFF_MAX);
					}
					sspa_entry->serial_port_state = 1; // 打开串口
					SuspendTask(sspa_entry->serial_port_comid); // 挂起对应的任务使串口空闲
					SetTextValue(3, 9, "关闭串口"); //刷新状态
				}
			}
			else
			{
				eUartOrder temp_order = ERRORSITE;
				temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
				if(temp_order != ERRORSITE) // 如果是正确的ID
				{
					uartbuff[temp_order].recepetion_flag = 0;
					memset(uartbuff[temp_order].recepetion_buff, 0, BUFF_MAX);
				}
				sspa_entry->serial_port_state = 0; // 关闭串口
				ResumeTask(sspa_entry->serial_port_comid); // 恢复串口 
				SetTextValue(3, 9, "打开串口"); //刷新状态
			}
			break;
		}
		case 14: { // 发送按键
			if(sspa_entry->serial_port_state == 1) // 如果串口是打开的
			{
				if(sspa_entry->serial_port_send_len != 0) // 如果发送缓冲区提取的长度不等于0
				{
					temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
					if(temp_uart != NULL)
					{
						if(sspa_entry->serial_port_send_new_row == 1) // 发送新行
						{
							uint8_t temp_send_buff[256];
							uint8_t temp_buff_len = sspa_entry->serial_port_send_len;
							
							memcpy(temp_send_buff, sspa_entry->serial_port_send_buff, temp_buff_len);
							
							temp_send_buff[temp_buff_len++] = '\r';
							temp_send_buff[temp_buff_len++] = '\n';
							
							HAL_UART_Transmit(temp_uart, temp_send_buff, temp_buff_len, 0xff); // 发送数据
						}
						else
						{
							HAL_UART_Transmit(temp_uart, sspa_entry->serial_port_send_buff, sspa_entry->serial_port_send_len, 0xff); // 发送数据
						}
						
					}
				}
			}
			
			break;
		}
		case 30: { // 查询全部配置按键
			if(sspa_entry->serial_port_state == 1) // 如果串口是打开的
			{
				temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
				if(temp_uart != NULL)
				{
					char cxpz_buff[] = "##,CXPZ,$$\r\n";
					HAL_UART_Transmit(temp_uart, (uint8_t *)cxpz_buff, strlen(cxpz_buff), 0xff); // 发送数据
				}
			}
			break;
		}
		case 31: { // 恢复默认配置
			if(sspa_entry->serial_port_state == 1) // 如果串口是打开的
			{
				temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
				if(temp_uart != NULL)
				{
					char cxpz_buff[] = "##,RESETALL,$$\r\n";
					HAL_UART_Transmit(temp_uart, (uint8_t *)cxpz_buff, strlen(cxpz_buff), 0xff); // 发送数据
				}
			}
			break;
		}
		case 29: {
			eUartOrder temp_order = ERRORSITE;
			temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
			if(temp_order != ERRORSITE) // 如果是正确的ID
			{
				uartbuff[temp_order].recepetion_flag = 0;
				memset(uartbuff[temp_order].recepetion_buff, 0, BUFF_MAX);
			}
			
			sspa_entry->serial_port_state = 0; // 关闭串口
			ResumeTask(sspa_entry->serial_port_comid); // 恢复串口 
			SetTextValue(3, 9, "打开串口"); //刷新状态
			break;
		}
		case 35: {
			for(uint8_t i = 15; i < 28; i++)
			{
				clearTextValue(3 , i); //(画面ID,控件ID）
			}
			sspa_entry->serial_port_show_offset = 0;
			break;
		}
		case 33: { // 字符串发送
			sspa_entry->serial_port_send_mode = !sspa_entry->serial_port_send_mode;
			if(sspa_entry->serial_port_send_mode == 0)
			{
				SetTextValue(3, 32, "字符串发送"); //刷新状态
			}
			else
			{
				SetTextValue(3, 32, "16进制发送"); //刷新状态
			}
			
			
			break;
		}
		case 37: {
			sspa_entry->serial_port_show_mode = !sspa_entry->serial_port_show_mode;
			if(sspa_entry->serial_port_show_mode == 0)
			{
				SetTextValue(3, 36, "字符串接收"); //刷新状态
			}
			else
			{
				SetTextValue(3, 36, "16进制接收"); //刷新状态
			}
			break;
		}
		case 39: {
			sspa_entry->serial_port_send_new_row = !sspa_entry->serial_port_send_new_row;
			
			if(sspa_entry->serial_port_send_new_row == 0)
			{
				SetTextValue(3, 38, "不发送新行"); //刷新状态
			}
			else
			{
				SetTextValue(3, 38, "发送新行"); //刷新状态
			}
			
			break;
		}
	}
	
}


static void SimulationSerialPortMenuCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t ctrl_id, uint8_t item, uint8_t state)
{
    if (ctrl_id != 8 || state != 1) {
        return;  // 提前返回，减少嵌套层级
    }

    uint8_t new_com_id = item + 1;
    uint8_t current_com_id = sspa_entry->serial_port_comid;

    /* XR5000_UART5_EXCLUSIVE_FIX_20260730: serial assistant cannot own loop-3 UART5. */
    if (new_com_id == 6U) {
        return;
    }
    
    // 如果选择的是同一个串口，不需要任何操作
    if (new_com_id == current_com_id) {
        return;
    }
    
    // 只有当串口当前是打开状态时才需要处理挂起/恢复操作
    if (sspa_entry->serial_port_state == 1 && current_com_id != 0) {
        // 恢复当前使用的串口任务
        ResumeTask(current_com_id);
        
        // 挂起新选择的串口任务
        SuspendTask(new_com_id);
    }
    
    // 更新选择的串口ID
    sspa_entry->serial_port_comid = new_com_id;
}

static void SimulationSerialPortTextCtrl(SimulationSerialPortAssistant_t *sspa_entry, uint16_t control_id, uint8_t *str)
{
	UART_HandleTypeDef *temp_uart = NULL;
	
	if(control_id == 4) // 如果是修改地址控件
	{
		int slave_addr;
		sscanf((const char *)str, "%d", &slave_addr);
		
		if(sspa_entry->serial_port_state == 1) // 如果串口是打开的
		{
			temp_uart = getSimulateSirealPortSendHandle(sspa_entry->serial_port_comid);
			if(temp_uart != NULL)
			{
				uint8_t buff_len;
				uint8_t cxpz_buff[32];
				buff_len = sprintf((char *)cxpz_buff, "##,ADR=%d,$$\r\n", slave_addr);
				HAL_UART_Transmit(temp_uart, cxpz_buff, buff_len, 0xff); // 发送数据
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
					
					// 转换单个十六进制字符为数值
					if(c >= '0' && c <= '9') value = c - '0';
					else if(c >= 'A' && c <= 'F') value = c - 'A' + 10;
					else if(c >= 'a' && c <= 'f') value = c - 'a' + 10;
					else {
							str_point++; // 跳过无效字符
							continue;
					}
					
					if(!got_high_nibble) {
							high_nibble = value << 4; // 高4位
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
        // 高4位
        output[pos++] = hex_chars[(data[i] >> 4) & 0x0F];
        // 低4位
        output[pos++] = hex_chars[data[i] & 0x0F];
        
        // 每个字节后添加空格（当前就是每2个字符加空格）
        if(i < len - 1) {
            output[pos++] = ' ';
        }
    }
    output[pos] = '\0';
}

static void SimulationSerialPortScreenShowApp(SimulationSerialPortAssistant_t *sspa_entry)
{
	eUartOrder temp_order = ERRORSITE;
	
	if(sspa_entry->serial_port_state == 1) // 如果串口是打开的
	{
		temp_order = getSimulateSirealPortReceiveIndex(sspa_entry->serial_port_comid);
		if(temp_order != ERRORSITE)
		{
			if(uartbuff[temp_order].recepetion_flag == 1) // 如果接收到了
			{
				uartbuff[temp_order].recepetion_flag = 0; // 清空接收
				
				if(sspa_entry->serial_port_show_mode == 1) // 16进制显示
				{
					uint8_t serial_port_receive_buff[256]; // 显示缓冲区 用来将数据变成16进制
					HexToHexStringLight(uartbuff[temp_order].recepetion_buff, uartbuff[temp_order].recepetion_len, serial_port_receive_buff);
					
					SetTextValue(3, 15 + sspa_entry->serial_port_show_offset, serial_port_receive_buff); //刷新状态
					sspa_entry->serial_port_show_offset++;
					sspa_entry->serial_port_show_offset %= 13;
				}
				else
				{
					const char *delimiter = "\r\n";
					char *token = strtok((char *)uartbuff[temp_order].recepetion_buff, delimiter);
					
					while (token != NULL) {
							SetTextValue(3, 15 + sspa_entry->serial_port_show_offset, (uint8_t *)token); //刷新状态
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
		
		sicj_entry->warn_fresh_flag = 1; // 标志所有状态刷新完成
		// 第一条报警信息置顶显示
		if(pcfws_entry->detector_class[0] == PackClassID && pcfws_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first warning display uses "第3回路 X号".
			FormatRS485DetectForeWarnLine(temp_buff, 1, pcfws_entry, 0);
		}
		else if(pcfws_entry->detector_class[0] == PackClassID)
		{
			if(pcfws_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cluster_id, pcfws.da[0].pack_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cluster_id, pcfws_entry->da[0].pack_id);
			}
		}
		else if(pcfws_entry->detector_class[0] == LinkageClassID) // 如果是外联设备
		{
			if(pcfws_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 报警器按下", 1,
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
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws.atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws.da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
			else if(pcfws_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
				pcfws_entry->atr[0].years, pcfws_entry->atr[0].months, pcfws_entry->atr[0].days,
				pcfws_entry->atr[0].hours, pcfws_entry->atr[0].minute, pcfws_entry->atr[0].second,
				pcfws_entry->da[0].cabin_id);
			}
		}
		
		for(uint8_t i = 0; i < 4; i++)
		{
			SetTextValue(screen_top_fresh_id[i], warn_alarm_fresh_id[i], temp_buff); // 刷新第一条报警内容
		}
	
	}
	else if(pcfws_entry->self_bottom_point == 0 && sicj_entry->warn_fresh_flag != 0)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			clearTextValue(screen_top_fresh_id[i], warn_alarm_fresh_id[i]); // 刷新第一条报警内容
		}
		sicj_entry->warn_fresh_flag = 0;
	}
	
	if(pcfas_entry->self_bottom_point != 0 && sicj_entry->fire_fresh_flag == 0)
	{
		uint8_t temp_buff[64];
		
		sicj_entry->fire_fresh_flag = 1;
		// 第一条报警信息置顶显示
		if(pcfas_entry->detector_class[0] == PackClassID && pcfas_entry->da[0].cluster_id == RS485_DETECT_FLASH_ID)
		{
			// XR5000_LOOP3_CHANGE_20260726: Loop 3 first fire display uses "第3回路 X号".
			FormatRS485DetectFireAlarmLine(temp_buff, 1, pcfas_entry, 0);
		}
		else if(pcfas_entry->detector_class[0] == PackClassID)
		{
			if(pcfas_entry->alarm_type[0] == Temperature)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器温度报警", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器烟雾报警", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第%d簇%d号PACK探测器一氧化碳报警", 1,
				pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
				pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
				pcfas_entry->da[0].cluster_id, pcfas_entry->da[0].pack_id);
			}
		}
		else if(pcfas_entry->detector_class[0] == LinkageClassID)
		{
			if(pcfas_entry->alarm_type[0] == AlarmCtrlKey)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 报警器按下", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second);
			}
			else
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 手报按下报警", 1,
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
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 温度报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Smoke)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 烟雾报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfws.atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Carbon)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 一氧化碳报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
			else if(pcfas_entry->alarm_type[0] == Hydrogen)
			{
				sprintf((char*)temp_buff, "%03d %d/%02d/%02d %02d:%02d:%02d 第1回路 %d号 氢气报警", 1,
					pcfas_entry->atr[0].years, pcfas_entry->atr[0].months, pcfas_entry->atr[0].days,
					pcfas_entry->atr[0].hours, pcfas_entry->atr[0].minute, pcfas_entry->atr[0].second,
					pcfas_entry->da[0].cabin_id);
			}
		}
		for(uint8_t i = 0; i < 4; i++)
		{
			SetTextValue(screen_top_fresh_id[i], fire_alarm_fresh_id[i], temp_buff); // 刷新第一条报警内容
		}
	}
	else if(pcfas_entry->self_bottom_point == 0 && sicj_entry->fire_fresh_flag != 0)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			clearTextValue(screen_top_fresh_id[i], fire_alarm_fresh_id[i]); // 刷新第一条报警内容
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
//	SystemSaveInfo.last_license_store[0] = '\0'; // 之前存储的授权码
//	SystemSaveInfo.pref_license_store[0] = '\0'; // 
}
