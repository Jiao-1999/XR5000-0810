#include "bsp_logic_set.h"

#include "bsp_debug.h"

#include "hmi_driver.h"
#include "hmi_user_uart.h"
#include "cmd_queue.h"
#include "cmd_process.h"

#include "bsp_adc.h"

#include <stdio.h>
#include <string.h>

OutFireStartLogic_t out_fire_start_ctrl = {
	.out_fire_device_sum     = 1,
	.device_work_method      = independent,
	.device_start_interval   = 3,
	.device1_correspondence  = outfire1_cluster,  // 1默认到簇
	.device2_correspondence  = outfire2_cabin,    // 2默认到舱
	.pressure_detection_type = dry_contact_open,  // 默认常开类型节点
	
	.cluster_start_delay     = 30,                // 默认延时30秒后启动
	.cluster_spray_number    = 3,                 // 默认喷放三次
	.cluster_spray_buff[0]   = 10,                // 第一次喷放持续时长
	.cluster_spray_buff[1]   = 25,                // 第一次喷放间隔时长
	.cluster_spray_buff[2]   = 10,                // 第二次喷放持续时长
	.cluster_spray_buff[3]   = 25,                // 第二次喷放间隔时长
	.cluster_spray_buff[4]   = 10,                // 第三次喷放喷放时长
	
	.cabin_start_delay       = 30,                // 默认延时30秒后启动
	.cabin_spray_number      = 3,                 // 默认喷放三次
	.cabin_spray_buff[0]     = 10,                // 第一次喷放持续时长
	.cabin_spray_buff[1]     = 25,                // 第一次喷放间隔时长
	.cabin_spray_buff[2]     = 10,                // 第二次喷放持续时长
	.cabin_spray_buff[3]     = 25,                // 第二次喷放间隔时长
	.cabin_spray_buff[4]     = 10,                // 第三次喷放喷放时长
}; // 灭火装置启动控制

FireAlarmLogic_t fire_alarm_logic_ctrl = {
	.buff_top = 0,
	.buff_bottom = 0,
	.buff_pointer = 0,
	.logic_modify_flag = 0
};

FireAlarmJudge_t fire_alarm_judge[64][64] = {0};

CabinDetectorState cabin_detector_state_buff[30] = {0};

CabinJudgeState cabin_fire_alarm_state = normal; // 默认正常状态

const char* hydrogen_str    = "氢气";
const char* carbon_str      = "一氧化碳";
const char* smoke_str       = "烟雾";
const char* temperature_str = "温度";

const char* and_str         = "与";
const char* or_str          = "或";
const char* number_str      = "号";

char output_buff[512]       = {0};
char convert_buff[2]        = {0,0};

void OutFireDeviceInternalScreenButtonSet(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state, OutFireStartLogic_t* ofsl)
{
	if(screen_id == 40)
	{
		if(control_id == 48) // 灭火装置数量菜单
		{
			if(item == 0 && state == 1) // 如果是第一个菜单栏选项 且按键按下
			{
				ofsl->out_fire_device_sum = 1;
			}
			else if(item == 1 && state == 1) // 如果是第二个菜单栏选项 且按键按下
			{
				ofsl->out_fire_device_sum = 2;
			}
		}
		else if(control_id == 49) // 工作方式菜单
		{
			if(item == 0 && state == 1) // 如果是第一个菜单栏选项 且按键按下
			{
				ofsl->device_work_method = independent;
			}
			else if(item == 1 && state == 1) // 如果是第二个菜单栏选项 且按键按下
			{
				ofsl->device_work_method = cooperative;
			}
		}
		else if(control_id == 55) // 灭火装置对应关系
		{
			if(item == 0 && state == 1) // 如果是第一个菜单栏选项 且按键按下
			{
				ofsl->device1_correspondence = outfire1_cluster;
			}
			else if(item == 1 && state == 1) // 如果是第二个菜单栏选项 且按键按下
			{
				ofsl->device1_correspondence = outfire1_cabin;
			}
			else if(item == 2 && state == 1) // 如果是第三个菜单栏选项 且按键按下
			{
				ofsl->device2_correspondence = outfire2_cluster;
			}
			else if(item == 3 && state == 1) // 如果是第亖个菜单栏选项 且按键按下
			{
				ofsl->device2_correspondence = outfire2_cabin;
			}
		}
		else if(control_id == 61) // 压力检测类型
		{
			if(item == 0 && state == 1) // 如果是第一个菜单栏选项 且按键按下
			{
				ofsl->pressure_detection_type = dry_contact_open;
			}
			else if(item == 1 && state == 1) // 如果是第二个菜单栏选项 且按键按下
			{
				ofsl->pressure_detection_type = dry_contact_close;
			}
			else if(item == 2 && state == 1) // 如果是第三个菜单栏选项 且按键按下
			{
				ofsl->pressure_detection_type = pressure_sensor;
			}
		}
		else if(control_id == 7) // 簇喷放次数菜单
		{
			if(item == 0 && state == 1) // 如果是第一个菜单栏选项 且按键按下
			{
				ofsl->cluster_spray_number = 1;
			}
			else if(item == 1 && state == 1) // 如果是第二个菜单栏选项 且按键按下
			{
				ofsl->cluster_spray_number = 2;
			}
			else if(item == 2 && state == 1) // 如果是第三个菜单栏选项 且按键按下
			{
				ofsl->cluster_spray_number = 3;
			}
		}
		else if(control_id == 22) // 舱喷放次数菜单
		{
			if(item == 0 && state == 1) // 如果是第一个菜单栏选项 且按键按下
			{
				ofsl->cabin_spray_number = 1;
			}
			else if(item == 1 && state == 1) // 如果是第二个菜单栏选项 且按键按下
			{
				ofsl->cabin_spray_number = 2;
			}
			else if(item == 2 && state == 1) // 如果是第三个菜单栏选项 且按键按下
			{
				ofsl->cabin_spray_number = 3;
			}
		}
	}
}

void OutFireDeviceInternalScreenTexttSet(uint16_t screen_id, uint16_t control_id, uint8_t *str, OutFireStartLogic_t* ofsl)
{
	if(screen_id == 40)
	{
		int32_t value=0;  
		sscanf((const char*)str,"%d",&value); 
		if(control_id == 58) // 启动间隔
		{
			ofsl->device_start_interval = value;
		}
		else if(control_id == 39) //簇 启动延时
		{
			ofsl->cluster_start_delay = value;
		}
		else if(control_id == 10) //簇 第一次喷放持续时长
		{
			ofsl->cluster_spray_buff[0] = value;
		}
		else if(control_id == 12) //簇 第一次喷放间隔时长
		{
			ofsl->cluster_spray_buff[1] = value;
		}
		else if(control_id == 14) //簇 第二次喷放持续时长
		{
			ofsl->cluster_spray_buff[2] = value;
		}
		else if(control_id == 16) //簇 第二次喷放间隔时长
		{
			ofsl->cluster_spray_buff[3] = value;
		}
		else if(control_id == 18) //簇 第三次喷放持续时长
		{
			ofsl->cluster_spray_buff[4] = value;
		}
		else if(control_id == 40) //舱 启动延时
		{
			ofsl->cabin_start_delay = value;
		}
		else if(control_id == 25) //舱 第一次喷放持续时长
		{
			ofsl->cabin_spray_buff[0] = value;
		}
		else if(control_id == 27) //舱 第一次喷间隔续时长
		{
			ofsl->cabin_spray_buff[1] = value;
		}
		else if(control_id == 29) //舱 第二次喷放持续时长
		{
			ofsl->cabin_spray_buff[2] = value;
		}
		else if(control_id == 31) //舱 第二次喷间隔续时长
		{
			ofsl->cabin_spray_buff[3] = value;
		}
		else if(control_id == 33) //舱 第三次喷放持续时长
		{
			ofsl->cabin_spray_buff[4] = value;
		}
	}
}

void OutFireDeviceInternalScreenUpdataUI(uint16_t _screen_id, OutFireStartLogic_t ofsl) // 传入结构体 避免不小心对结构体做修改
{
	if(_screen_id == 40)
	{
		uint8_t temp_buff[32];
		
		if(ofsl.out_fire_device_sum == 1)
		{
			SetTextValue(_screen_id, 53, (uint8_t *)"灭火装置对应回路和簇");//刷新报警内容
		}
		else
		{
//			SetTextValue(_screen_id,4,"取消启动灭火装置！");//刷新报警内容
		}
		
		if(ofsl.pressure_detection_type == pressure_sensor)
		{
			float pressure = 0;
			
			pressure = (outfire1_pressure - 4.0f)/3.2f; // 计算压力值
			sprintf((char *)temp_buff, "灭火装置1压力:%.1fMPa", pressure);
			SetTextValue(_screen_id, 62, temp_buff);//刷新报警内容
			
			pressure = (outfire2_pressure - 4.0f)/3.2f; // 计算压力值
			sprintf((char *)temp_buff, "灭火装置2压力:%.1fMPa", pressure);
			SetTextValue(_screen_id, 70, temp_buff);//刷新报警内容
			
//			if(outfire1_pressure > 4)
//			{
//				pressure = (outfire1_pressure - 4.0f)/3.2f; // 计算压力值
//				sprintf((char *)temp_buff, "灭火装置1压力:%.1fMPa", pressure);
//				SetTextValue(_screen_id, 62, temp_buff);//刷新报警内容
//			}
//			else
//			{
//				SetTextValue(_screen_id, 62, "灭火装置1压力异常");//刷新报警内容
//			}
//			
//			if(outfire2_pressure > 4)
//			{
//				pressure = (outfire2_pressure - 4.0f)/3.2f; // 计算压力值
//				sprintf((char *)temp_buff, "灭火装置2压力:%.1fMPa", pressure);
//				SetTextValue(_screen_id, 70, temp_buff);//刷新报警内容
//			}
//			else
//			{
//				SetTextValue(_screen_id, 70, "灭火装置2压力异常");//刷新报警内容
//			}

		}
		else
		{
			clearTextValue(_screen_id, 62);//(画面ID,控件ID） // 清掉压力传感器显示内容
		}
		
		if(ofsl.cluster_spray_number == 1)
		{
			clearTextValue(_screen_id, 36);//(画面ID,控件ID） // 清掉第一次间隔时长
			clearTextValue(_screen_id, 34);//(画面ID,控件ID） // 清掉第二次喷放时长
			clearTextValue(_screen_id, 37);//(画面ID,控件ID） // 清掉第二次间隔时长
			clearTextValue(_screen_id, 35);//(画面ID,控件ID） // 清掉第三次喷放时长
			
			clearTextValue(_screen_id, 11);//(画面ID,控件ID） // 清掉间隔显示框
			clearTextValue(_screen_id, 12);//(画面ID,控件ID） // 清掉间隔时长
			clearTextValue(_screen_id, 13);//(画面ID,控件ID） // 清掉压力传感器显示内容
			clearTextValue(_screen_id, 14);//(画面ID,控件ID） // 清掉压力传感器显示内容
			clearTextValue(_screen_id, 15);//(画面ID,控件ID） // 清掉清掉间隔显示框
			clearTextValue(_screen_id, 16);//(画面ID,控件ID） // 清掉清掉间隔时长
			clearTextValue(_screen_id, 17);//(画面ID,控件ID） // 清掉第三次喷放显示框
			clearTextValue(_screen_id, 18);//(画面ID,控件ID） // 清掉第三次喷放时长显示
			
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[0]);
			SetTextValue(_screen_id, 10, temp_buff);//刷新报警内容
		}
		else if(ofsl.cluster_spray_number == 2)
		{
			clearTextValue(_screen_id, 37);//(画面ID,控件ID） // 清掉第二次间隔时长
			clearTextValue(_screen_id, 35);//(画面ID,控件ID） // 清掉第三次喷放时长
			
			clearTextValue(_screen_id, 15);//(画面ID,控件ID） // 清掉清掉间隔显示框
			clearTextValue(_screen_id, 16);//(画面ID,控件ID） // 清掉清掉间隔时长
			clearTextValue(_screen_id, 17);//(画面ID,控件ID） // 清掉第三次喷放显示框
			clearTextValue(_screen_id, 18);//(画面ID,控件ID） // 清掉第三次喷放时长显示
			
			SetTextValue(_screen_id, 36, (uint8_t *)"间隔时长");//刷新报警内容
			SetTextValue(_screen_id, 34, (uint8_t *)"第二次喷放");//刷新报警内容
			SetTextValue(_screen_id, 11, (uint8_t *)"间隔/分:");//刷新报警内容
			SetTextValue(_screen_id, 13, (uint8_t *)"时长/秒:");//刷新报警内容
			
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[0]);
			SetTextValue(_screen_id, 10, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[1]);
			SetTextValue(_screen_id, 12, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[2]);
			SetTextValue(_screen_id, 14, temp_buff);//刷新报警内容
			
		}
		else if(ofsl.cluster_spray_number == 3)
		{
			SetTextValue(_screen_id, 36, (uint8_t *)"间隔时长");//刷新报警内容
			SetTextValue(_screen_id, 34, (uint8_t *)"第二次喷放");//刷新报警内容
			SetTextValue(_screen_id, 37, (uint8_t *)"间隔时长");//刷新报警内容
			SetTextValue(_screen_id, 35, (uint8_t *)"第三次喷放");//刷新报警内容
			
			SetTextValue(_screen_id, 11, (uint8_t *)"间隔/分:");//刷新报警内容
			SetTextValue(_screen_id, 13, (uint8_t *)"时长/秒:");//刷新报警内容
			SetTextValue(_screen_id, 15, (uint8_t *)"间隔/分:");//刷新报警内容
			SetTextValue(_screen_id, 17, (uint8_t *)"时长/秒:");//刷新报警内容
			
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[0]);
			SetTextValue(_screen_id, 10, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[1]);
			SetTextValue(_screen_id, 12, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[2]);
			SetTextValue(_screen_id, 14, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[3]);
			SetTextValue(_screen_id, 16, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cluster_spray_buff[4]);
			SetTextValue(_screen_id, 18, temp_buff);//刷新报警内容
		}
		
		if(ofsl.cabin_spray_number == 1)
		{
			clearTextValue(_screen_id, 67);//(画面ID,控件ID） // 清掉第一次间隔时长
			clearTextValue(_screen_id, 65);//(画面ID,控件ID） // 清掉第二次喷放时长
			clearTextValue(_screen_id, 68);//(画面ID,控件ID） // 清掉第二次间隔时长
			clearTextValue(_screen_id, 66);//(画面ID,控件ID） // 清掉第三次喷放时长
			
			clearTextValue(_screen_id, 26);//(画面ID,控件ID） // 清掉间隔显示框
			clearTextValue(_screen_id, 27);//(画面ID,控件ID） // 清掉间隔时长
			clearTextValue(_screen_id, 28);//(画面ID,控件ID） // 清掉压力传感器显示内容
			clearTextValue(_screen_id, 29);//(画面ID,控件ID） // 清掉压力传感器显示内容
			clearTextValue(_screen_id, 30);//(画面ID,控件ID） // 清掉清掉间隔显示框
			clearTextValue(_screen_id, 31);//(画面ID,控件ID） // 清掉清掉间隔时长
			clearTextValue(_screen_id, 32);//(画面ID,控件ID） // 清掉第三次喷放显示框
			clearTextValue(_screen_id, 33);//(画面ID,控件ID） // 清掉第三次喷放时长显示
			
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[0]);
			SetTextValue(_screen_id, 25, temp_buff);//
		}
		else if(ofsl.cabin_spray_number == 2)
		{
			clearTextValue(_screen_id, 68);//(画面ID,控件ID） // 清掉第二次间隔时长
			clearTextValue(_screen_id, 66);//(画面ID,控件ID） // 清掉第三次喷放时长
			
			clearTextValue(_screen_id, 30);//(画面ID,控件ID） // 清掉清掉间隔显示框
			clearTextValue(_screen_id, 31);//(画面ID,控件ID） // 清掉清掉间隔时长
			clearTextValue(_screen_id, 32);//(画面ID,控件ID） // 清掉第三次喷放显示框
			clearTextValue(_screen_id, 33);//(画面ID,控件ID） // 清掉第三次喷放时长显示
			
			SetTextValue(_screen_id, 67, (uint8_t *)"间隔时长");//刷新报警内容
			SetTextValue(_screen_id, 65, (uint8_t *)"第二次喷放");//刷新报警内容
			SetTextValue(_screen_id, 26, (uint8_t *)"间隔/分:");//刷新报警内容
			SetTextValue(_screen_id, 28, (uint8_t *)"时长/秒:");//刷新报警内容
			
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[0]);
			SetTextValue(_screen_id, 25, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[1]);
			SetTextValue(_screen_id, 27, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[2]);
			SetTextValue(_screen_id, 29, temp_buff);//刷新报警内容
			
		}
		else if(ofsl.cabin_spray_number == 3)
		{
			SetTextValue(_screen_id, 67, (uint8_t *)"间隔时长");//刷新报警内容
			SetTextValue(_screen_id, 65, (uint8_t *)"第二次喷放");//刷新报警内容
			SetTextValue(_screen_id, 68, (uint8_t *)"间隔时长");//刷新报警内容
			SetTextValue(_screen_id, 66, (uint8_t *)"第三次喷放");//刷新报警内容
			
			SetTextValue(_screen_id, 26, (uint8_t *)"间隔/分:");//刷新报警内容
			SetTextValue(_screen_id, 28, (uint8_t *)"时长/秒:");//刷新报警内容
			SetTextValue(_screen_id, 30, (uint8_t *)"间隔/分:");//刷新报警内容
			SetTextValue(_screen_id, 32, (uint8_t *)"时长/秒:");//刷新报警内容
			
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[0]);
			SetTextValue(_screen_id, 25, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[1]);
			SetTextValue(_screen_id, 27, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[2]);
			SetTextValue(_screen_id, 29, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[3]);
			SetTextValue(_screen_id, 31, temp_buff);//刷新报警内容
			sprintf((char *)temp_buff, "%d", ofsl.cabin_spray_buff[4]);
			SetTextValue(_screen_id, 33, temp_buff);//刷新报警内容
		}
	}
}


// 火警触发逻辑控制
void FireAlarmTriggerLogicButtonSet(uint16_t screen_id, uint16_t control_id, uint8_t  state, FireAlarmLogic_t* fals)
{
	if(screen_id == 43) // 火警触发逻辑设定页面
	{
		if(control_id == 72 && fals->logic_modify_flag == 0 && state == 1)
		{
			fals->buff_bottom = 0;  // 栈尾指针
			fals->buff_pointer = 0; // 数组指针
			memset(fals->fire_alarm_logic_buff, 0, sizeof(fals->fire_alarm_logic_buff));
			fals->logic_modify_flag = 1;
			clearTextValue(screen_id, 73);//(画面ID,控件ID） // 清掉第二次间隔时长
		}
		if(fals->logic_modify_flag == 1)
		{
			if(control_id >= 1 && control_id <= 9 && state == 1)
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = control_id; // 
			}
			else if(control_id == 10 && state == 1)
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = 0;
			}
			else if(control_id == 11 && state == 1)  // 
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = '#'; // 用来表示号
			}
			else if(control_id == 12 && state == 1) // 逻辑或
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = '|'; // 用来表示或
			}
			else if(control_id == 13 && state == 1) // 逻辑与
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = '&'; // 用来表示与
			}
			else if(control_id == 14 && state == 1)  // 取消修改操作
			{
				
			}
			else if(control_id == 15 && state == 1)  // 删除操作
			{
				if(fals->buff_bottom != 0)
				{
					fals->buff_bottom--; // 让指针回退一位
				}
			}
			else if(control_id == 16 && state == 1)  // 确认操作
			{
				fals->logic_modify_flag = 0;   // 完成修改
				
//				DebugSendString(fals->fire_alarm_logic_buff, fals->buff_bottom);
			}
			else if(control_id == 18 && state == 1)  // 氢气按键
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = hydrogen; // 用来表示氢气
			}
			else if(control_id == 19 && state == 1)  // 一氧化碳按键
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = carbon; // 用来表示一氧化碳
			}
			else if(control_id == 20 && state == 1)  // 烟雾按键
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = smoke; // 用来表示烟雾
			}
			else if(control_id == 21 && state == 1)  // 温度按键
			{
				fals->fire_alarm_logic_buff[fals->buff_bottom++] = temperature; // 用来表示温度
			}
		}
	}
}

void FireAlarmTriggerLogicUpdataUI(uint16_t _screen_id, FireAlarmLogic_t fals, FireAlarmJudge_t faj[][64]) // 传入结构体 避免不小心对结构体做修改
{
	if(_screen_id == 43)
	{
		if(fals.buff_top != fals.buff_bottom) // 如果堆顶不等于堆底 表明有数据
		{
			output_buff[0] = 0;
			for(int i = 0;i < fals.buff_bottom;i++)
			{
        if(fals.fire_alarm_logic_buff[i] == '#')
        {
           strcat(output_buff, number_str);
        }
        else if(fals.fire_alarm_logic_buff[i] == hydrogen)
        {
           strcat(output_buff, hydrogen_str);
        }
        else if(fals.fire_alarm_logic_buff[i] == carbon)
        {
           strcat(output_buff, carbon_str);
        }
				else if(fals.fire_alarm_logic_buff[i] == smoke)
        {
           strcat(output_buff, smoke_str);
        }
        else if(fals.fire_alarm_logic_buff[i] == temperature)
        {
           strcat(output_buff, temperature_str);
        }
        else if(fals.fire_alarm_logic_buff[i] == '&')
        {
           strcat(output_buff, and_str);
        }
        else if(fals.fire_alarm_logic_buff[i] == '|')
        {
					strcat(output_buff, or_str);
        }
        else
        {
					convert_buff[0] = fals.fire_alarm_logic_buff[i] + '0';  // 数字转字符
					strcat(output_buff, convert_buff);
        }
			}
			SetTextValue(_screen_id, 73, (uint8_t *)output_buff);//刷新触发逻辑
		}
	}
	else if(_screen_id == 45)
	{
		uint8_t i; // 循环变量 遍历数组
		uint8_t j; // 循环变量 遍历数组
		
		for(i = 0;faj[i][0].detect_number != 0;i++)
		{
			convert_buff[0] = i + 1 + '0';
			SetTextValue(_screen_id, (i + 1), (uint8_t *)convert_buff);    // 显示序号
			output_buff[0] = 0;
			for(j = 0;faj[i][j].detect_number != 0;j++)
			{	
				convert_buff[0] = faj[i][j].detect_number + '0';  // 数字转字符
				strcat(output_buff, convert_buff);
				strcat(output_buff, number_str);
				if(faj[i][j].alarm_type == hydrogen)
				{
					strcat(output_buff, hydrogen_str);
				}
				else if(faj[i][j].alarm_type == carbon)
				{
					strcat(output_buff, carbon_str);
				}
				else if(faj[i][j].alarm_type == smoke)
				{
					strcat(output_buff, smoke_str);
				}
				else if(faj[i][j].alarm_type == temperature)
				{
					strcat(output_buff, temperature_str);
				}
				
				if(faj[i][j + 1].detect_number != 0)
				{
					strcat(output_buff, and_str);
				}
			}
			SetTextValue(_screen_id, (i + 10), (uint8_t *)output_buff);    // 显示序号
//			memset(output_buff, 0, sizeof(output_buff));
			output_buff[0] = 0;
		}
		
		while(i < 9)
		{
			clearTextValue(_screen_id, i + 1);     //(画面ID,控件ID） // 清掉不存在的逻辑
			clearTextValue(_screen_id, i + 10);     //(画面ID,控件ID） // 清掉不存在的逻辑
			i++;
		}
	}
}

// 入口参数 FireAlarmLogic_t 表示火警逻辑 是用户输入的数组
//          FireAlarmJudge_t 表示提取后的用户逻辑，用来实现最后的火警判断
void FireAlarmJudgeBuffExtract(FireAlarmLogic_t fals, FireAlarmJudge_t faj[][64])
{
	static uint8_t bottom_point = 0;
	
	if(fals.logic_modify_flag == 0)
	{
		if(fals.buff_top != fals.buff_bottom)
		{
			if(fals.buff_bottom != bottom_point)
			{
				uint8_t logic_num = 0; // 一共有几个或逻辑
				uint8_t logic_row_num = 0; // 一行有几个逻辑
				uint8_t i = 0;    // 循环变量
				
				bottom_point = fals.buff_bottom; // 只有尾部指针发生变更时才进行修改
				memset(faj, 0, sizeof(FireAlarmJudge_t) * 64 * 64); // 清空数组内容 避免判定的时候出现问题
				while(i < fals.buff_bottom) // 逻辑存储中有值 进行提取
				{
					while(fals.fire_alarm_logic_buff[i] <= 9 && i < fals.buff_bottom) // 先将数字提取出来
					{
						faj[logic_num][logic_row_num].detect_number = faj[logic_num][logic_row_num].detect_number * 10 + fals.fire_alarm_logic_buff[i]; // 提取数字出来
						i++;
					}
					
					if(i < fals.buff_bottom) // 防止数组越界
					{
						if(fals.fire_alarm_logic_buff[i] == '#') // 如果是编号 说明后面是判定报警的类型类型
						{
							i++; // 向下跳一位
							if(i < fals.buff_bottom) // 防止数组越界
							{
								faj[logic_num][logic_row_num].alarm_type = fals.fire_alarm_logic_buff[i]; // 储存报警判断的逻辑
							}
						}
						i++; // 报警类型之后就是逻辑判断符号
						if(fals.fire_alarm_logic_buff[i] == '&' && i < fals.buff_bottom) // 如果是逻辑符
						{
							logic_row_num++; // 往下记录与逻辑判定
						}
						else if(fals.fire_alarm_logic_buff[i] == '|' && i < fals.buff_bottom)
						{
							logic_num++;
							logic_row_num = 0;
						}
						else if(fals.fire_alarm_logic_buff[i] == 0 && i < fals.buff_bottom) // 如果不是这两个符号 表明结束了
						{
							logic_num++;
						}
					}
					
				}
				
			}
		}
		else if(fals.buff_top == fals.buff_bottom)
		{
			memset(faj, 0, sizeof(FireAlarmJudge_t) * 64 * 64); // 清空数组内容 避免判定的时候出现问题
		}
	}
	
}


CabinJudgeState FireAlarmCompoundLogicJudgement(FireAlarmLogic_t fals, FireAlarmJudge_t faj[][64], CabinDetectorState cds[])
{
	uint8_t i; // 循环变量 遍历数组
	uint8_t j; // 循环变量 遍历数组

	uint8_t and_sum = 0;     // 满足与逻辑数量
	
	CabinJudgeState alarm_state = normal;
	
	FireAlarmJudgeBuffExtract(fals, faj); // 提取报警逻辑
	
	for(i = 0;faj[i][0].detect_number != 0;i++)
	{
		for(j = 0;faj[i][j].detect_number != 0;j++)
		{
			if(faj[i][j].alarm_type == hydrogen && cds[faj[i][j].detect_number].hydrogen_state != 0)
			{
				and_sum++;
			}
			else if(faj[i][j].alarm_type == carbon && cds[faj[i][j].detect_number].carbon_state != 0)
			{
				and_sum++;
			}
			else if(faj[i][j].alarm_type == smoke && cds[faj[i][j].detect_number].smoke_state != 0)
			{
				and_sum++;
			}
			else if(faj[i][j].alarm_type == temperature && cds[faj[i][j].detect_number].temperature_state != 0)
			{
				and_sum++;
			}
//			DebugSendString(&faj[i][j].detect_number, 1);
//			DebugSendString(&faj[i][j].alarm_type   , 1);
		}
		if(j == and_sum)
		{
			alarm_state = fire_alarm;
			break;
		}
		else
		{
			and_sum = 0;
		}
	}
	return alarm_state;
}


