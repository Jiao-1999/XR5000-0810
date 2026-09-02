#include "bsp_super.h"

#include "bsp_debug.h"
#include "cmd_process.h"
#include "cmd_queue.h"
#include "cmsis_os.h"

AdminButtonCtrl_t button_ctrl = {0};

uint32_t super_admin_password = 0;

ThresholdSeting_t fire_alarm_threshold = {
	.hydrogen_warn_1           = 500,
	.hydrogen_warn_2           = 800,
	
	.carbon_mono_warn_1        = 500,
	.carbon_mono_warn_2        = 800,
	
	.temperature_warn_1        = 68,        // 温度一级预警阈值
	.temperature_warn_2        = 78,        // 温度二级预警阈值
	
	.outfire_device1_pressure  = 2.5,
	.device1_pressure_uplimit  = 3.2,
	.device1_pressure_lowlimit = 1.8,

	.outfire_device2_pressure  = 2.5,
	.device2_pressure_uplimit  = 3.2,
	.device2_pressure_lowlimit = 1.8,
	
};

extern uint32_t chaojimima;

void SuperAdminButtonCtrl(uint16_t screen_id, uint16_t control_id, uint8_t  state, AdminButtonCtrl_t *abc)
{
	if(screen_id == 27)
	{
		if(control_id == 8 && state == 1)
		{
			abc->button_count++;
			
			if(abc->press_flag == 0)
			{
				abc->button_count = 0;     // 第一次进入清空计数值
				abc->timout_count = 0;     // 清空计时值
				abc->press_flag = 1;       // 标志位置1 开始计时
			}
			else
			{
				if(abc->timout_count <= 5) // 五秒计数到了
				{
					if(abc->button_count >= 8)
					{
						abc->press_flag = 0;   // 
						
						SetScreen(51);
						GetScreen();
					}
					
				}
				else
				{
					abc->press_flag = 0;   // 
				}
			}
		}
	}

}

void SuperAdminPasswordButtonCtrl(uint16_t screen_id, uint16_t control_id, uint8_t  state, uint32_t *password)
{
	if(screen_id == 51)
	{
		if(control_id == 3 && state == 1)
		{
			if(*password == chaojimima)
			{
				clearTextValue(screen_id,4);//(画面ID,控件ID）
				SetScreen(46);
				GetScreen();
				*password = 0;
			}
			else
			{
				SetTextValue(screen_id, 4, "error password");//刷新报警内容
			}
		}
	}
}

void SuperAdminInternalScreenTextCtrl(uint16_t screen_id, uint16_t control_id, uint8_t *str, uint32_t *password)
{
	if(screen_id == 51)
	{
		if(control_id == 4) 
		{
			sscanf((const char*)str,"%d", password); 
		}
	}
}

void FireAlarmThresholdSettingInternalScreenText(uint16_t screen_id, uint16_t control_id, uint8_t *str, ThresholdSeting_t* ts)
{
	if(screen_id == 47)
	{
		int32_t value=0;  
		sscanf((const char*)str,"%d",&value); 
		switch(control_id)
		{
			case 14: ts->hydrogen_warn_1 = value; break;
			case 15: ts->hydrogen_warn_2 = value; break; // 氢气二级预警
			
			case 16: ts->carbon_mono_warn_1 = value; break;
			case 17: ts->carbon_mono_warn_2 = value; break; // 一氧化碳二级预警
			
			case 18: ts->temperature_warn_1 = value; break;
			case 19: ts->temperature_warn_2 = value; break; // 温度二级预警
			default: break;
		}
	}
	else if(screen_id == 48)
	{
		float value=0;  
		sscanf((const char*)str,"%f",&value); 
		if(control_id == 5)
		{
			if(value == 1.2f)
			{
				ts->outfire_device1_pressure  = value;
				ts->device1_pressure_uplimit  = 1.9f;
				ts->device1_pressure_lowlimit = 0.9f;
			}
			else if(value == 2.5f)
			{
				ts->outfire_device1_pressure  = value;
				ts->device1_pressure_uplimit  = 3.2f;
				ts->device1_pressure_lowlimit = 1.8f;
			}
			else if(value == 4.2f)
			{
				ts->outfire_device1_pressure  = value;
				ts->device1_pressure_uplimit  = 6.0f;
				ts->device1_pressure_lowlimit = 3.1f;
			}
			else if(value == 3.4f)
			{
				ts->outfire_device1_pressure  = value;
				ts->device1_pressure_uplimit  = 3.8f;
				ts->device1_pressure_lowlimit = 3.1f;
			}
		}
		else if(control_id == 8)
		{
			if(value == 1.2f)
			{
				ts->outfire_device2_pressure  = value;
				ts->device2_pressure_uplimit  = 1.9f;
				ts->device2_pressure_lowlimit = 0.9f;
			}
			else if(value == 2.5f)
			{
				ts->outfire_device2_pressure  = value;
				ts->device2_pressure_uplimit  = 3.2f;
				ts->device2_pressure_lowlimit = 1.8f;
			}
			else if(value == 4.2f)
			{
				ts->outfire_device2_pressure  = value;
				ts->device2_pressure_uplimit  = 6.0f;
				ts->device2_pressure_lowlimit = 3.1f;
			}
			else if(value == 3.4f)
			{
				ts->outfire_device2_pressure  = value;
				ts->device2_pressure_uplimit  = 3.8f;
				ts->device2_pressure_lowlimit = 3.1f;
			}
		}
	}
}

void FireAlarmThresholdUpdataUI(uint16_t _screen_id, ThresholdSeting_t ts)
{
	uint8_t temp_buff[32];
	if(_screen_id == 47)
	{
		sprintf((char *)temp_buff,"%d", ts.hydrogen_warn_1);
		SetTextValue(_screen_id, 14, temp_buff); // 氢气一级预警值
		sprintf((char *)temp_buff,"%d", ts.hydrogen_warn_2);
		SetTextValue(_screen_id, 15, temp_buff); // 氢二一级预警值
		
		sprintf((char *)temp_buff,"%d", ts.carbon_mono_warn_1);
		SetTextValue(_screen_id, 16, temp_buff); // 一氧化碳一级预警值
		sprintf((char *)temp_buff,"%d", ts.carbon_mono_warn_2);
		SetTextValue(_screen_id, 17, temp_buff); // 一氧化碳二级预警值
		
		sprintf((char *)temp_buff,"%d", ts.temperature_warn_1);
		SetTextValue(_screen_id, 18, temp_buff); // 温度一级预警值
		sprintf((char *)temp_buff,"%d", ts.temperature_warn_2);
		SetTextValue(_screen_id, 19, temp_buff); // 温度二级预警值
	}
	else if(_screen_id == 48)
	{
		sprintf((char *)temp_buff,"%.1f", ts.outfire_device1_pressure);
		SetTextValue(_screen_id, 5, temp_buff); // 标称压力
		sprintf((char *)temp_buff,"%.1f", ts.device1_pressure_uplimit);
		SetTextValue(_screen_id, 6, temp_buff); // 
		sprintf((char *)temp_buff,"%.1f", ts.device1_pressure_lowlimit);
		SetTextValue(_screen_id, 7, temp_buff); // 
		
		sprintf((char *)temp_buff,"%.1f", ts.outfire_device2_pressure);
		SetTextValue(_screen_id, 8, temp_buff); // 标称压力
		sprintf((char *)temp_buff,"%.1f", ts.device2_pressure_uplimit);
		SetTextValue(_screen_id, 9, temp_buff); // 
		sprintf((char *)temp_buff,"%.1f", ts.device2_pressure_lowlimit);
		SetTextValue(_screen_id, 10, temp_buff); // 
	}
	
}





