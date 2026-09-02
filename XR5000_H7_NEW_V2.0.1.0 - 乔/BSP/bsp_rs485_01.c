#include "bsp_rs485_01.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"

#include "cmd_process.h"
#include "bsp_debug.h"

// XR805从而总线该到485
#include "bsp_mbus.h"

#include "bsp_ctrl_bus.h"

// 此文件为01模块（簇行为管理器）
// 现风机控制模块也接在该总线上（IG3302）

#define CANG_SLAVE_SUM 30
#define CANG_USER_SLAVE_SUM 24

// 定义轮询设备类型
typedef enum {
    DEVICE_PACK = 0,
//  DEVICE_XR805,
    DEVICE_TYPE_COUNT // 用于数组大小计算
} DeviceType;

typedef void (*device_poll_func)(void); 

// 设备轮询配置结构体
typedef struct {
    uint32_t poll_interval;
    uint32_t last_poll_time;
    device_poll_func poll_function;
} DevicePollConfig;

// 全局轮询配置
DevicePollConfig poll_configs[DEVICE_TYPE_COUNT] = {
    [DEVICE_PACK]  = {150,  0, PollPackManager_Ultra},
//    [DEVICE_XR805] = {9999,  0, PollXR805Manager},
};

uint8_t pack_polling = 1; // 轮询初始值（注：为了兼容屏幕显示，下标从1开始）
uint8_t online_buff[PACK_SLAVE_NUM]; // 筛选出设定为上线的探测器 储存下标到该数组中
uint8_t len; // 获取设定为上线的01模块数量

uint8_t PackSlaveAddr[PACK_SLAVE_NUM] = {
	0,1,2,3,4,5,6,7,8,9,
	10,11,12,13,14,15,16,17,18,19,
	20,21,22,23,24,25,26,27,28,29
};

uint8_t CangSlaveAddr[CANG_SLAVE_SUM] = {
	20,21,22,23,24,25,26,27,28,29,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49
};

extern QueueHandle_t xMyRs485QueueHandle;

//extern SemaphoreHandle_t xMutex; // 初始化互斥锁



extern uint8_t cang_polling; // 轮询初始值（注：为了兼容屏幕显示，下标从1开始）
extern uint8_t pollFlag;

// end

// new

uint8_t transimit_addr = 0;
uint8_t recieve_judge  = 0;

uint8_t timeout_led_ctrl_flag = 0;
uint8_t timeout_led_ctrl = 0;

// end


// new

uint8_t cluster_pack_disconnect_count_buff[PACK_SLAVE_NUM][33] = {0};

void ClusterPackDisconnectCountInit(void)
{
	memset(cluster_pack_disconnect_count_buff, 0, PACK_SLAVE_NUM*33);
}

uint8_t getClusterPackDisconnectCount(uint8_t cluster_id, uint8_t pack_id)
{
	return cluster_pack_disconnect_count_buff[cluster_id][pack_id];
}

void clearClusterPackDisconnectCount(uint8_t cluster_id, uint8_t pack_id)
{
	cluster_pack_disconnect_count_buff[cluster_id][pack_id] = 0;
}

// end


void PackBehaviorManageSendString(uint8_t* buf, uint8_t len)
{
	HAL_UART_Transmit(&huart9,buf,len,0xff);
}

extern void BspCmdProcessInit(void);
extern void LinkageDeviceStateInit(void);
extern void BspRelayInit(void);
extern void LedStateInit(void);

extern void DeletUpdataUITask(void);
extern void CreatUpdataUITask(void);

extern void CurrentStartStopKeyStateInit(void);
extern void OutFireSprayStateInit(void);
extern void FanSendCountInit(void);

extern void UartBufferInit(void);


//将二总线上的所有设备进行复位操作
void ResetAllBusDevice(void)
{
	uint8_t modbusbuf[8]; // 发送缓冲区
	uint8_t clear_queue[8];
	
	uint16_t crc16 = 0x0000; // CRC校验码
	
	taskENTER_CRITICAL(); // 进入临界区
	
	DeletUpdataUITask(); // 删除屏幕更新任务
	
	modbusbuf[0] = 0xFF; // 从机地址取值
	modbusbuf[1] = 0x05; // 05功能码
	modbusbuf[2] = 0x00;
	modbusbuf[3] = 20;
	modbusbuf[4] = 0xFF;
	modbusbuf[5] = 0x00; // 
	crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
	modbusbuf[6] = crc16 >> 0;
	modbusbuf[7] = crc16 >> 8;
	
	// 循环读取直到队列为空
	while(xQueueReceive(xMyRs485QueueHandle, clear_queue, 0) == pdTRUE)
	{
			// 什么也不做，只是清空数据
	}
	
	while(ReceiveDataFromFanBusQueue(clear_queue) == 1)
	{
			// 什么也不做，只是清空数据
	}
	
	// 发送三次初始化
	for(crc16 = 0;crc16 < 2;crc16++)
	{
		xQueueSend(xMyRs485QueueHandle, modbusbuf, 0); // 发送到队列
		
	}
	
	Fan1CtrlClose();
	Fan2CtrlClose();
	
	taskEXIT_CRITICAL();
	
	MBus1ResetAllDevices(); /* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: UART7 blocking reset runs outside the RTOS critical section. */
	
	osDelay(1000); // 等待发送任务发送复位命令
	
	taskENTER_CRITICAL();
	
	BspCmdProcessInit();
	ClusterPackDisconnectCountInit(); // 清除包掉线值
	LinkageDeviceStateInit();
	BspRelayInit(); // 继电器初始化
	LedStateInit(); // LED状态初始化
	CurrentStartStopKeyStateInit(); // 清除启动停止状态
	OutFireSprayStateInit();
	FanSendCountInit();
	
	UartBufferInit();
	
	CreatUpdataUITask(); // 重新创建屏幕更新任务
	
//	ReadPointTypeSetOnlieState();
	
	taskEXIT_CRITICAL();
}


// 簇/仓 控制 命令
void ClusterOrCabinCtrlCmd(uint8_t detector_id, uint8_t reg_addr, uint8_t turn_on_off)
{
	uint8_t modbusbuf[8]; // 发送缓冲区
	uint16_t crc16 = 0x0000; // CRC校验码
	
	modbusbuf[0] = detector_id; // 从机地址取值
	modbusbuf[1] = 5; // 05功能码
	modbusbuf[2] = 0;
	modbusbuf[3] = reg_addr;     // 寄存器地址
	modbusbuf[4] = turn_on_off;  // 开或关继电器
	modbusbuf[5] = 0; 
	crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
	modbusbuf[6] = crc16 >> 0;
	modbusbuf[7] = crc16 >> 8;
	
	xQueueSend(xMyRs485QueueHandle, modbusbuf, 0); // 发送到队列
}

uint8_t last_poll_id = 1; //来自于PollPackManager_Plus
/*
下面的结构体和初始化来自于第二次定义的PollPackManager_Ultra(void)
*/
typedef struct {
    uint8_t circuit_id;
    uint8_t package_id;
} PollState;

PollState ultra_poll_state = {1, 1};

void PollPackManager_Ultra(void)//第三次定义
{
	uint8_t found_online = 0;
	uint8_t modbusbuf[8];
	
	// 从当前状态开始尝试
	PollState start_state = ultra_poll_state;
	PollState current_state = start_state;
	
	// 先尝试从当前位置开始找在线package
	while(1)
	{
		// 递增package ID（从下一个package开始）
		current_state.package_id++;
		if(current_state.package_id >= 33)  // 超出范围
		{
			current_state.package_id = 1;
			// package回到1时，切换回路
			current_state.circuit_id++;
			if(current_state.circuit_id >= 4)
			{
					current_state.circuit_id = 1;
			}
		}

		// 检查是否在线
		if(pack_online_buff[current_state.circuit_id][current_state.package_id] == 1)
		{
			found_online = 1;
			break;
		}
		
		// 检查是否已经遍历一圈回到起点
		if(current_state.circuit_id == start_state.circuit_id && 
			 current_state.package_id == start_state.package_id)
		{
			// 已经遍历完所有可能的package，退出
			break;
		}
	}
	
	if(!found_online)
	{
		// 没有找到在线模块，重置状态
		ultra_poll_state.circuit_id = 1;
		ultra_poll_state.package_id = 1;
		return;
	}
	
	// 保存状态供下次使用 - 不要加1！
  ultra_poll_state = current_state;

	// 构建并发送Modbus查询（你的原有代码）
	modbusbuf[0] = current_state.circuit_id;
	modbusbuf[1] = 0x04;
	modbusbuf[2] = 0x00;
	modbusbuf[3] = (current_state.package_id - 1)*6;
	modbusbuf[4] = 0x00;
	modbusbuf[5] = 0x06;
	
	uint16_t crc = CalcCrc16(modbusbuf, 6);
	modbusbuf[6] = crc & 0xFF;
	modbusbuf[7] = crc >> 8;
	
	xQueueSend(xMyRs485QueueHandle, modbusbuf, 0);
	
	// 更新超时计数器
	if(cluster_pack_disconnect_count_buff[PackSlaveAddr[current_state.circuit_id]][current_state.package_id] < PackDisconnectCount)
	{
			cluster_pack_disconnect_count_buff[PackSlaveAddr[current_state.circuit_id]][current_state.package_id]++;
	}
	
	if(CU_zx_buf[current_state.circuit_id] < PackDisconnectCount)
	{
			CU_zx_buf[current_state.circuit_id]++;
	}
}

/*
以下代码：
last_online_index的定义和PollXR805Manager(void)函数首次出现在void BSP_RS485_01_Poll(void* parameter)前面定义
此处只是对其进行了复制，但并未对原代码位置进行修改
*/
static uint8_t last_online_index = 0;
void PollXR805Manager(void)
{
    uint8_t modbusbuf[8];
    uint16_t crc16 = 0;
    
    uint8_t start_index = last_online_index;
    uint8_t found_online = 0;
    
    // 查找下一个在线设备
    for(uint8_t attempt = 0; attempt < CANG_USER_SLAVE_SUM; attempt++)
    {
        last_online_index++;
        if(last_online_index >= CANG_USER_SLAVE_SUM || last_online_index < 1) {
            last_online_index = 1;
        }
        
        if(cang_sxzt[last_online_index] == 1)
        {
            found_online = 1;
            cang_polling = last_online_index; // 更新全局索引
            break;
        }
    }
    
    if(!found_online)
    {
        // 没有在线设备，重置为起始位置
        last_online_index = start_index;
//        printf("[XR805] 无在线设备可用\n");
        return;
    }
    
    // 构建Modbus请求
    modbusbuf[0] = CangSlaveAddr[cang_polling];
    modbusbuf[1] = 0x04;
    modbusbuf[2] = 0x00;
    modbusbuf[3] = 0x00;
    modbusbuf[4] = 0x00;
    modbusbuf[5] = 16;
    
    crc16 = CalcCrc16(modbusbuf, 6);
    modbusbuf[6] = crc16 & 0xFF;
    modbusbuf[7] = crc16 >> 8;
    
    // 发送请求
    if(xQueueSend(xMyRs485QueueHandle, modbusbuf, 0) == pdTRUE)
    {
			// 更新超时计数器（修复原函数中的transimit_addr未定义问题）
			uint8_t device_index = modbusbuf[0] - 20;
			if(Cang_zx_buf[device_index] < CabinDisconnectCount)
			{
					Cang_zx_buf[device_index]++;
			}
    }

}


// pack 一氧化碳浓度值缓存区
uint16_t pack_co_concen_buf[21][33] = {0};

uint16_t getPackCoConcenValue(uint8_t cluster_id, uint8_t pack_id)
{
	return pack_co_concen_buf[cluster_id][pack_id];
}

// 全局定义状态映射表
static const uint8_t STATE_MAP[] = {0, 1, 2, 3};

/*
此函数出现在本文件的最后，此处仅复制了原函数的代码，但并未对原函数位置进行修改
*/
void RS485_01_Receive_Deal_Ultra(void)
{
	uint16_t crc16 = 0x0000; // CRC校验码
	uint8_t subscrib = 0; // 数组下标
	uint8_t	package_subscrib = 0;
	if(uartbuff[PACKSITE].recepetion_flag == 1) // 如果接收到了
	{
		// 计算CRC校验值
		crc16 = (uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-1]<<8) | 
							uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-2];
		if(CalcCrc16(uartbuff[PACKSITE].recepetion_buff, uartbuff[PACKSITE].recepetion_len - 2) == crc16)
		{
			if(uartbuff[PACKSITE].recepetion_buff[0] < 21) // 如果是处理包
			{
				// 回路ID
				subscrib = uartbuff[PACKSITE].recepetion_buff[0];
				// 回复探测器ID
				package_subscrib = (uartbuff[PACKSITE].recepetion_buff[3] << 8) | uartbuff[PACKSITE].recepetion_buff[4];
				
				CU_zx_buf[subscrib] = 0; // 清楚对应编号的掉线计数
				
				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04)
				{
					// 高位是0 直接忽略
					PACK_zx_buf[subscrib][package_subscrib] = package_subscrib;//获取探测器地址
					if(package_subscrib)
					{
						cluster_pack_disconnect_count_buff[subscrib][package_subscrib] = 0; // 清空掉线次数
					}
					PACK_wendu_buf[subscrib][package_subscrib] = (uartbuff[PACKSITE].recepetion_buff[5] << 8) | 
																					uartbuff[PACKSITE].recepetion_buff[6];//获取温度
					
					pack_co_concen_buf[subscrib][package_subscrib] = (uartbuff[PACKSITE].recepetion_buff[9] << 8) | 
																							uartbuff[PACKSITE].recepetion_buff[10]; // 获取一氧化碳浓度值
					
					uint16_t temp_state = (uartbuff[PACKSITE].recepetion_buff[13] << 8) |
																uartbuff[PACKSITE].recepetion_buff[14]; //获取报警状态
					
					PACK_WDZT_buf[subscrib][package_subscrib] = (uint8_t)(temp_state & 0x0003);
					
					// 代码逻辑
					PACK_YWZT_buf[subscrib][package_subscrib] = STATE_MAP[(temp_state & 0x000C) >> 2];
					PACK_COZT_buf[subscrib][package_subscrib] = STATE_MAP[(temp_state & 0x0030) >> 4];
				}
				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06)
				{
					
				}
			}
			else if(uartbuff[PACKSITE].recepetion_buff[0] >= 21 && uartbuff[PACKSITE].recepetion_buff[0] < 45) // 如果是XR805
			{
				uint8_t real_subscrib = uartbuff[PACKSITE].recepetion_buff[0] - 20;
				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04) 
				{
					Cang_wendu_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[3]<<8)|uartbuff[PACKSITE].recepetion_buff[4]; // 取出16位温度数据
					Cang_WDZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[5]<<8)|uartbuff[PACKSITE].recepetion_buff[6]; // 取出温度状态
					
					Cang_YWZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[9]<<8)|uartbuff[PACKSITE].recepetion_buff[10]; // 取出烟雾状态
					
					Cang_CH4ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[13]<<8)|uartbuff[PACKSITE].recepetion_buff[14]; // 取出甲烷状态
					
					Cang_COzhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[15]<<8)|uartbuff[PACKSITE].recepetion_buff[16]; // 取出一氧化碳值
					Cang_COZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[17]<<8)|uartbuff[PACKSITE].recepetion_buff[18]; // 取出一氧化碳状态
					
					Cang_VOCZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[21]<<8)|uartbuff[PACKSITE].recepetion_buff[22]; // 取出VOC状态
					
					Cang_H2zhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[23]<<8)|uartbuff[PACKSITE].recepetion_buff[24]; // 取出氢气值
					Cang_H2ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[25]<<8)|uartbuff[PACKSITE].recepetion_buff[26]; // 取出氢气状态
					
					Cang_TCQXH_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[31]<<8)|uartbuff[PACKSITE].recepetion_buff[32]; // 取出探测器型号
					
					Cang_CGQQY_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[33]<<8)|uartbuff[PACKSITE].recepetion_buff[34]; // 取出传感器启用状态
					
				}
				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06) 
				{
					
				}
				Cang_zx_buf[real_subscrib] = 0; // 将对应地址掉线下标清零
			}
		}
		uartbuff[PACKSITE].recepetion_flag = 0; // 清除标志位

	}
	
	if(timeout_led_ctrl_flag == 1) // 如果为一 则证明有发送
	{
		timeout_led_ctrl_flag = 0;
		if(transimit_addr >= 1 && transimit_addr < 21)
		{
			recieve_judge = CU_zx_buf[transimit_addr];
		}
		else if(transimit_addr >= 21 && transimit_addr < 45)
		{
			recieve_judge = Cang_zx_buf[transimit_addr - 20]; // 获取掉线值 如果不为零 证明本次没有接受到
		}
		if(recieve_judge < 2)
		{
			timeout_led_ctrl = 2; // 发送成功 接收成功
		}
		else
		{
			timeout_led_ctrl = 1; // 发送成功 但是接收失败
		}

	}
}

/*
此函数出现在本文件的最后，此处仅复制了原函数的代码，但并未对原函数位置进行修改
*/
void RS485_01_PollAndRecieve(void * parameter)
{
	uint8_t current_poll_index = 0;  // 当前轮询的设备索引
	uint8_t modbusbuf[8] = {0};      // 发送缓冲区

	uint32_t current_time = osKernelGetTickCount(); // 系统当前时间戳
	// 初始化各设备的上次轮询时间
	for(int i = 0; i < DEVICE_TYPE_COUNT; i++) {
		poll_configs[i].last_poll_time = current_time;
	}
	
	for(;;)
	{
		// 先判断处理
		RS485_01_Receive_Deal_Ultra();

		current_time = osKernelGetTickCount();
		for(int count = 0; count < DEVICE_TYPE_COUNT; count++) 
		{
			uint8_t i = (current_poll_index + count) % DEVICE_TYPE_COUNT; // 尝试从current_poll_index开始找下一个可发送设备
			
			if(current_time - poll_configs[i].last_poll_time >= poll_configs[i].poll_interval) 
			{
				current_poll_index = (i + 1) % DEVICE_TYPE_COUNT; // 确保下个设备开始
	
				poll_configs[i].poll_function();                  // 回调发送函数
				// 执行完发送后才标记时间
				poll_configs[i].last_poll_time = current_time;    // 记录发送时间
			}
		}
		
		if(xQueueReceive(xMyRs485QueueHandle, modbusbuf, 0) == pdTRUE)
		{
			PackBehaviorManageSendString(modbusbuf, sizeof(modbusbuf));
			timeout_led_ctrl_flag = 1;
		}

		osDelay(50);
	}
}

// void PollPackManager(void)
// {
// 	uint8_t modbusbuf[8]; 
// 	uint8_t subscrib = 0; // 数组下标
// 	uint16_t crc16 = 0x0000; // CRC校验码
	
// 	len = SystemQuerySetOnlineNum(cu_sxzt, online_buff, PACK_SLAVE_NUM); // 获取一共有几个探测器
// 	if(len > 1) // 如果长度为1 表示没有需要轮询的 因为为了兼容祖传代码 下标默认是1
// 	{
// 		// 更改轮询值
// 		pack_polling = (pack_polling < len - 1) ? pack_polling + 1 : 1;
// 		// online_buff中取出下标 该下标再通过寄存器地址转换
// 		subscrib = online_buff[pack_polling];

// 		modbusbuf[0] = PackSlaveAddr[subscrib]; // 从机地址取值
// 		modbusbuf[1] = 4; // 04功能码
// 		modbusbuf[2] = 0;
// 		modbusbuf[3] = 0;
// 		modbusbuf[4] = 0;
// 		modbusbuf[5] = cu_tcq_sxzt[subscrib] * 6; // 一个pack探测器下有6个寄存器 根据01模块下pack数量计算发送长度
// 		crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
// 		modbusbuf[6] = crc16 >> 0;
// 		modbusbuf[7] = crc16 >> 8;
// //		crc16 = 0;
		
// //		PackBehaviorManageSendString(modbusbuf, 8); // 现在使用分时发送 不需要使用消息队列
// 		xQueueSend(xMyRs485QueueHandle, modbusbuf, 0); // 发送到队列		
		
// 		for(uint8_t i = 0; i < cu_tcq_sxzt[subscrib]; i++)
// 		{
// 			if(cluster_pack_disconnect_count_buff[PackSlaveAddr[subscrib]][i + 1] < PackDisconnectCount)
// 			{
// 				cluster_pack_disconnect_count_buff[PackSlaveAddr[subscrib]][i + 1]++;
// 			}
// 		}
		
// 		if(CU_zx_buf[modbusbuf[0]] < PackDisconnectCount)
// 		{
// 			CU_zx_buf[modbusbuf[0]] = CU_zx_buf[modbusbuf[0]] + 1; // 超时值+1 最多10次 到10后认为掉线
// 		}
		
// 	}
// }

// 2025/12/10 14:38 新增用来满足送检支持
// uint8_t last_poll_id = 1; // 保留原有格式，所以注释掉，但是已经重新定义该变量，恢复时注意把旧变量名恢复，同时删除重复的变量

// void PollPackManager_Plus(void)
// {
// 	uint8_t curr_poll_id = last_poll_id; // 
// 	uint8_t found_online_flag = 0;
	
// 	uint8_t modbusbuf[8]; 

// 	uint16_t crc16 = 0x0000; // CRC校验码

// 	for(uint8_t i = 1; i < 4; i++)
// 	{
// 		curr_poll_id++;
// 		if(curr_poll_id == 0 || curr_poll_id >= 4)
// 		{
// 			curr_poll_id = 1;
// 		}
// 		for(uint8_t j = 1; j < 33; j++)
// 		{
// 			if(pack_online_buff[curr_poll_id][j] == 1) // 如果有设置上线的
// 			{
// 				found_online_flag = 1;
// 				last_poll_id = curr_poll_id;
// 				break;
// 			}
// 		}
// 		if(found_online_flag == 1)
// 		{
// 			break;
// 		}
// 	}
	
// 	if(found_online_flag == 0)
// 	{
// 		return;
// 	}
	
// 	modbusbuf[0] = curr_poll_id; // 从轮询中取值
// 	modbusbuf[1] = 4; // 04功能码
// 	modbusbuf[2] = 0;
// 	modbusbuf[3] = 0;
// 	modbusbuf[4] = 0;
// 	modbusbuf[5] = 32 * 6; // 一个pack探测器下有6个寄存器 根据01模块下pack数量计算发送长度
// 	crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
// 	modbusbuf[6] = crc16 >> 0;
// 	modbusbuf[7] = crc16 >> 8;
// //		crc16 = 0;
	
// //		PackBehaviorManageSendString(modbusbuf, 8); // 现在使用分时发送 不需要使用消息队列
// 	xQueueSend(xMyRs485QueueHandle, modbusbuf, 0); // 发送到队列		
	
// 	for(uint8_t i = 1; i < 33; i++)
// 	{
// 		if(cluster_pack_disconnect_count_buff[PackSlaveAddr[curr_poll_id]][i] < PackDisconnectCount && 
// 			pack_online_buff[curr_poll_id][i] == 1 ) // 如果小于掉线值并且设置上线才计算掉线值
// 		{
// 			cluster_pack_disconnect_count_buff[PackSlaveAddr[curr_poll_id]][i]++;
// 		}
// 	}
	
// 	if(CU_zx_buf[curr_poll_id] < PackDisconnectCount)
// 	{
// 		CU_zx_buf[curr_poll_id] = CU_zx_buf[curr_poll_id] + 1; // 超时值+1 最多10次 到10后认为掉线
// 	}
// }

//// 2025/12/11 15:25 新增用来满足送检支持
//uint8_t last_ultra_poll_circuit_id = 1; // 
//uint8_t last_ultra_poll_package_id = 1;

//void PollPackManager_Ultra(void)//第一次定义
//{
//	uint8_t curr_ultra_poll_circuit_id = last_ultra_poll_circuit_id; // 
//	uint8_t curr_ultra_poll_package_id = last_ultra_poll_package_id;
//	
//	uint8_t found_online_flag = 0;
//	
//	uint8_t modbusbuf[8]; 

//	uint16_t crc16 = 0x0000; // CRC校验码

//	for(uint8_t i = 1; i < 4; i++)
//	{
//		if(curr_ultra_poll_circuit_id == 0 || curr_ultra_poll_circuit_id >= 4)
//		{
//			curr_ultra_poll_circuit_id = 1;
//		}
//		for(uint8_t j = 1; j < 33; j++)
//		{
//			curr_ultra_poll_package_id++;
//			if(curr_ultra_poll_package_id == 0 || curr_ultra_poll_package_id >= 33)
//			{
//				curr_ultra_poll_package_id = 1;
//			}
//			if(pack_online_buff[curr_ultra_poll_circuit_id][curr_ultra_poll_package_id] == 1) // 如果有设置上线的
//			{
//				found_online_flag = 1;
//				break;
//			}
//			
//		}
//		if(found_online_flag == 1)
//		{
//			last_ultra_poll_package_id = curr_ultra_poll_package_id;
//			last_ultra_poll_circuit_id = curr_ultra_poll_circuit_id;
//			break;
//		}
//		curr_ultra_poll_circuit_id++;
//	}
//	
//	if(found_online_flag == 0)
//	{
//		return;
//	}
//	
//	modbusbuf[0] = curr_ultra_poll_circuit_id; // 从轮询中取值
//	modbusbuf[1] = 4; // 04功能码
//	modbusbuf[2] = 0;
//	modbusbuf[3] = curr_ultra_poll_package_id;
//	modbusbuf[4] = 0;
//	modbusbuf[5] = 1; // 一个pack探测器下有6个寄存器 根据01模块下pack数量计算发送长度
//	crc16 = CalcCrc16(modbusbuf, 6); // 计算六字节数据的CRC校验码
//	modbusbuf[6] = crc16 >> 0;
//	modbusbuf[7] = crc16 >> 8;

//	xQueueSend(xMyRs485QueueHandle, modbusbuf, 0); // 发送到队列		
//	// 如果小于掉线值并且设置上线才计算掉线值
//	if(cluster_pack_disconnect_count_buff[PackSlaveAddr[curr_ultra_poll_circuit_id]][curr_ultra_poll_package_id] < PackDisconnectCount) 
//	{
//		cluster_pack_disconnect_count_buff[PackSlaveAddr[curr_ultra_poll_circuit_id]][curr_ultra_poll_package_id]++;
//	}
//	
//	if(CU_zx_buf[curr_ultra_poll_circuit_id] < PackDisconnectCount)
//	{
//		CU_zx_buf[curr_ultra_poll_circuit_id] = CU_zx_buf[curr_ultra_poll_circuit_id] + 1; // 超时值+1 最多10次 到10后认为掉线
//	}
//}

// 2025/12/11 15:25 新增用来满足送检支持
// typedef struct {
//     uint8_t circuit_id;
//     uint8_t package_id;
// } PollState;

// PollState ultra_poll_state = {1, 1};//保留原有格式，所以注释掉，但是已经重新定义该变量，恢复时注意把旧变量名恢复，同时删除重复的变量

//void PollPackManager_Ultra(void)//第二次定义
//{
//    PollState current_state = ultra_poll_state;
//    uint8_t found_online = 0;
//    uint8_t modbusbuf[8];
//    
//    // 遍历逻辑：先递增package，遍历完当前circuit的所有package后，再切换到下一个circuit
//    for(uint8_t circuit_checked = 0; circuit_checked < 3; circuit_checked++)
//    {
//        // 遍历当前circuit的所有package
//        for(uint8_t package_try = 0; package_try < 32; package_try++)
//        {
//            // 递增package ID
//            current_state.package_id++;
//            if(current_state.package_id >= 33)  // 超出范围
//            {
//                current_state.package_id = 1;
//            }
//            
//            // 检查是否在线
//            if(pack_online_buff[current_state.circuit_id][current_state.package_id] == 1)
//            {
//                found_online = 1;
//                break;
//            }
//        }
//        
//        if(found_online)
//        {
//					break;
//        }
//        
//        // 当前circuit没有在线package，切换到下一个circuit
//        current_state.circuit_id++;
//        if(current_state.circuit_id >= 4)  // 超出范围
//        {
//            current_state.circuit_id = 1;
//        }
//        current_state.package_id = 0;  // 设置为0，内层循环会+1变成1
//    }
//    
//    if(!found_online)
//    {
//        // 没有找到在线模块，重置状态
//        ultra_poll_state.circuit_id = 1;
//        ultra_poll_state.package_id = 1;
//        return;
//    }
//    
//    // 保存状态供下次使用
//    ultra_poll_state = current_state;
//    
//    // 构建并发送Modbus查询
//    modbusbuf[0] = current_state.circuit_id;
//    modbusbuf[1] = 0x04;
//    modbusbuf[2] = 0x00;
//    modbusbuf[3] = (current_state.package_id - 1)*6;  // 起始地址
//    modbusbuf[4] = 0x00;
//    modbusbuf[5] = 0x06;  // 只查询1个寄存器
//    
//    uint16_t crc = CalcCrc16(modbusbuf, 6);
//    modbusbuf[6] = crc & 0xFF;
//    modbusbuf[7] = crc >> 8;
//    
//    xQueueSend(xMyRs485QueueHandle, modbusbuf, 0);
//    
//    // 更新超时计数器
//    if(cluster_pack_disconnect_count_buff[PackSlaveAddr[current_state.circuit_id]][current_state.package_id] < PackDisconnectCount)
//    {
//        cluster_pack_disconnect_count_buff[PackSlaveAddr[current_state.circuit_id]][current_state.package_id]++;
//    }
//    
//    if(CU_zx_buf[current_state.circuit_id] < PackDisconnectCount)
//    {
//        CU_zx_buf[current_state.circuit_id]++;
//    }
//}

// void PollPackManager_Ultra(void)//第三次定义
// {
// 	uint8_t found_online = 0;
// 	uint8_t modbusbuf[8];
	
// 	// 从当前状态开始尝试
// 	PollState start_state = ultra_poll_state;
// 	PollState current_state = start_state;
	
// 	// 先尝试从当前位置开始找在线package
// 	while(1)
// 	{
// 		// 递增package ID（从下一个package开始）
// 		current_state.package_id++;
// 		if(current_state.package_id >= 33)  // 超出范围
// 		{
// 			current_state.package_id = 1;
// 			// package回到1时，切换回路
// 			current_state.circuit_id++;
// 			if(current_state.circuit_id >= 4)
// 			{
// 					current_state.circuit_id = 1;
// 			}
// 		}

// 		// 检查是否在线
// 		if(pack_online_buff[current_state.circuit_id][current_state.package_id] == 1)
// 		{
// 			found_online = 1;
// 			break;
// 		}
		
// 		// 检查是否已经遍历一圈回到起点
// 		if(current_state.circuit_id == start_state.circuit_id && 
// 			 current_state.package_id == start_state.package_id)
// 		{
// 			// 已经遍历完所有可能的package，退出
// 			break;
// 		}
// 	}
	
// 	if(!found_online)
// 	{
// 		// 没有找到在线模块，重置状态
// 		ultra_poll_state.circuit_id = 1;
// 		ultra_poll_state.package_id = 1;
// 		return;
// 	}
	
// 	// 保存状态供下次使用 - 不要加1！
//   ultra_poll_state = current_state;

// 	// 构建并发送Modbus查询（你的原有代码）
// 	modbusbuf[0] = current_state.circuit_id;
// 	modbusbuf[1] = 0x04;
// 	modbusbuf[2] = 0x00;
// 	modbusbuf[3] = (current_state.package_id - 1)*6;
// 	modbusbuf[4] = 0x00;
// 	modbusbuf[5] = 0x06;
	
// 	uint16_t crc = CalcCrc16(modbusbuf, 6);
// 	modbusbuf[6] = crc & 0xFF;
// 	modbusbuf[7] = crc >> 8;
	
// 	xQueueSend(xMyRs485QueueHandle, modbusbuf, 0);
	
// 	// 更新超时计数器
// 	if(cluster_pack_disconnect_count_buff[PackSlaveAddr[current_state.circuit_id]][current_state.package_id] < PackDisconnectCount)
// 	{
// 			cluster_pack_disconnect_count_buff[PackSlaveAddr[current_state.circuit_id]][current_state.package_id]++;
// 	}
	
// 	if(CU_zx_buf[current_state.circuit_id] < PackDisconnectCount)
// 	{
// 			CU_zx_buf[current_state.circuit_id]++;
// 	}
// }

// static uint8_t last_online_index = 0;
// void PollXR805Manager(void)
// {
//     uint8_t modbusbuf[8];
//     uint16_t crc16 = 0;
    
//     uint8_t start_index = last_online_index;
//     uint8_t found_online = 0;
    
//     // 查找下一个在线设备
//     for(uint8_t attempt = 0; attempt < CANG_USER_SLAVE_SUM; attempt++)
//     {
//         last_online_index++;
//         if(last_online_index >= CANG_USER_SLAVE_SUM || last_online_index < 1) {
//             last_online_index = 1;
//         }
        
//         if(cang_sxzt[last_online_index] == 1)
//         {
//             found_online = 1;
//             cang_polling = last_online_index; // 更新全局索引
//             break;
//         }
//     }
    
//     if(!found_online)
//     {
//         // 没有在线设备，重置为起始位置
//         last_online_index = start_index;
// //        printf("[XR805] 无在线设备可用\n");
//         return;
//     }
    
//     // 构建Modbus请求
//     modbusbuf[0] = CangSlaveAddr[cang_polling];
//     modbusbuf[1] = 0x04;
//     modbusbuf[2] = 0x00;
//     modbusbuf[3] = 0x00;
//     modbusbuf[4] = 0x00;
//     modbusbuf[5] = 16;
    
//     crc16 = CalcCrc16(modbusbuf, 6);
//     modbusbuf[6] = crc16 & 0xFF;
//     modbusbuf[7] = crc16 >> 8;
    
//     // 发送请求
//     if(xQueueSend(xMyRs485QueueHandle, modbusbuf, 0) == pdTRUE)
//     {
// 			// 更新超时计数器（修复原函数中的transimit_addr未定义问题）
// 			uint8_t device_index = modbusbuf[0] - 20;
// 			if(Cang_zx_buf[device_index] < CabinDisconnectCount)
// 			{
// 					Cang_zx_buf[device_index]++;
// 			}
//     }

// }
//该函数未使用
//void BSP_RS485_01_Poll(void* parameter)
//{
//	uint8_t current_poll_index = 0;  // 当前轮询的设备索引
//	uint8_t modbusbuf[8] = {0};      // 发送缓冲区

//	uint32_t current_time = osKernelGetTickCount(); // 系统当前时间戳
//	// 初始化各设备的上次轮询时间
//	for(int i = 0; i < DEVICE_TYPE_COUNT; i++) {
//		poll_configs[i].last_poll_time = current_time;
//	}

//	for(;;)
//	{
//		current_time = osKernelGetTickCount();

//		for(int count = 0; count < DEVICE_TYPE_COUNT; count++) 
//		{
//			
//			uint8_t i = (current_poll_index + count) % DEVICE_TYPE_COUNT; // 尝试从current_poll_index开始找下一个可发送设备
//			
//			if(current_time - poll_configs[i].last_poll_time >= poll_configs[i].poll_interval) 
//			{
//				current_poll_index = (i + 1) % DEVICE_TYPE_COUNT; // 确保下个设备开始
//				
//				poll_configs[i].last_poll_time = current_time;    // 记录发送时间
//				poll_configs[i].poll_function();                  // 回调发送函数
//			}
//		}
//		
//		if(xQueueReceive(xMyRs485QueueHandle, modbusbuf, 0) == pdTRUE)
//		{
//			PackBehaviorManageSendString(modbusbuf, sizeof(modbusbuf));
//		}

//		osDelay(200); // 
//	}
//}

// pack 一氧化碳浓度值缓存区
// uint16_t pack_co_concen_buf[21][33] = {0};

// uint16_t getPackCoConcenValue(uint8_t cluster_id, uint8_t pack_id)
// {
// 	return pack_co_concen_buf[cluster_id][pack_id];
// }

// // 全局定义状态映射表
// static const uint8_t STATE_MAP[] = {0, 1, 2, 3};
//该函数未使用，句柄已经被注释  
//void QueuePollRecvDealTask(void *parameter)
//{
//	uint16_t crc16 = 0x0000; // CRC校验码
//	uint8_t subscrib = 0; // 数组下标
//	for(;;)
//	{
//		if(uartbuff[PACKSITE].recepetion_flag == 1) // 如果接收到了
//		{
//			// 计算CRC校验值
//			crc16 = (uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-1]<<8)|uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-2];
//			if(CalcCrc16(uartbuff[PACKSITE].recepetion_buff, uartbuff[PACKSITE].recepetion_len - 2) == crc16)
//			{
//				if(uartbuff[PACKSITE].recepetion_buff[0] < 21) // 如果是处理包
//				{
////					subscrib = online_buff[pack_polling]; // 取出下标
//					subscrib = uartbuff[PACKSITE].recepetion_buff[0];
//					CU_zx_buf[subscrib] = 0; // 清楚对应编号的掉线计数
//					
//					if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04)
//					{
//						// 清空未设置上线的数组或者设置为下线的数组 避免在屏幕上乱显示
//						for(uint8_t i = cu_tcq_sxzt[subscrib];i < 32; i++)
//						{
//							PACK_zx_buf[subscrib][i+1] = 0;
////							cluster_pack_disconnect_count_buff[subscrib][i + 1] = 0; // 将接收到的簇下标pack掉线值清零
//							pack_co_concen_buf[subscrib][i+1] = 0;
//							
//							PACK_wendu_buf[subscrib][i+1]=0;
//							PACK_WDZT_buf[subscrib][i+1]=0;
//							PACK_YWZT_buf[subscrib][i+1]=0;
//							PACK_COZT_buf[subscrib][i+1]=0;
//							PACK_CH4ZT_buf[subscrib][i+1]=0;
//						}
//						for(uint8_t i = 0;i < cu_tcq_sxzt[subscrib]; i++) // 计算一个01模块索引下 有多少个pack探测器
//						{
//							// 高位是0 直接忽略
//							PACK_zx_buf[subscrib][i+1] = uartbuff[PACKSITE].recepetion_buff[(i*12)+4];//获取探测器地址
//							if(PACK_zx_buf[subscrib][i+1])
//							{
//								cluster_pack_disconnect_count_buff[subscrib][i + 1] = 0;
//							}
//							PACK_wendu_buf[subscrib][i+1] = (uartbuff[PACKSITE].recepetion_buff[(i*12)+5] << 8) | 
//																							uartbuff[PACKSITE].recepetion_buff[(i*12)+6];//获取温度
//							
//							pack_co_concen_buf[subscrib][i+1] = (uartbuff[PACKSITE].recepetion_buff[(i*12)+9] << 8) | 
//																									uartbuff[PACKSITE].recepetion_buff[(i*12)+10]; // 获取一氧化碳浓度值
//							
//							uint16_t temp_state = (uartbuff[PACKSITE].recepetion_buff[(i*12)+13] << 8) |
//																		uartbuff[PACKSITE].recepetion_buff[(i*12)+14]; //获取报警状态
//							
//							PACK_WDZT_buf[subscrib][i+1] = (uint8_t)(temp_state & 0x0003);
//							
//							// 代码逻辑
//							PACK_YWZT_buf[subscrib][i+1] = STATE_MAP[(temp_state & 0x000C) >> 2];
//							PACK_COZT_buf[subscrib][i+1] = STATE_MAP[(temp_state & 0x0030) >> 4];
//							// PACK_COZT_buf[subscrib][i+1] = (uint8_t)(temp_state & 0x0030);
//							// PACK_YWZT_buf[subscrib][i+1]=uartbuff[PACKSITE].recepetion_buff[(i*12)+9]*256+uartbuff[PACKSITE].recepetion_buff[(i*12)+10];//获取烟雾状态
//							// PACK_COZT_buf[subscrib][i+1]=uartbuff[PACKSITE].recepetion_buff[(i*12)+11]*256+uartbuff[PACKSITE].recepetion_buff[(i*12)+12];//获取一氧化碳状态
//							// PACK_CH4ZT_buf[subscrib][i+1]=uartbuff[PACKSITE].recepetion_buff[(i*12)+13]*256+uartbuff[PACKSITE].recepetion_buff[(i*12)+14];//获取甲烷状态
//						}
//					}
//					else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06)
//					{
//						
//					}
//				}
//				else if(uartbuff[PACKSITE].recepetion_buff[0] >= 21 && uartbuff[PACKSITE].recepetion_buff[0] < 45) // 如果是XR805
//				{
//					uint8_t real_subscrib = uartbuff[PACKSITE].recepetion_buff[0] - 20;
//					if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04) 
//					{
//						Cang_wendu_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[3]<<8)|uartbuff[PACKSITE].recepetion_buff[4]; // 取出16位温度数据
//						Cang_WDZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[5]<<8)|uartbuff[PACKSITE].recepetion_buff[6]; // 取出温度状态
//						
//						Cang_YWZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[9]<<8)|uartbuff[PACKSITE].recepetion_buff[10]; // 取出烟雾状态
//						
//						Cang_CH4ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[13]<<8)|uartbuff[PACKSITE].recepetion_buff[14]; // 取出甲烷状态
//						
//						Cang_COzhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[15]<<8)|uartbuff[PACKSITE].recepetion_buff[16]; // 取出一氧化碳值
//						Cang_COZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[17]<<8)|uartbuff[PACKSITE].recepetion_buff[18]; // 取出一氧化碳状态
//						
//						Cang_VOCZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[21]<<8)|uartbuff[PACKSITE].recepetion_buff[22]; // 取出VOC状态
//						
//						Cang_H2zhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[23]<<8)|uartbuff[PACKSITE].recepetion_buff[24]; // 取出氢气值
//						Cang_H2ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[25]<<8)|uartbuff[PACKSITE].recepetion_buff[26]; // 取出氢气状态
//						
//						Cang_TCQXH_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[31]<<8)|uartbuff[PACKSITE].recepetion_buff[32]; // 取出探测器型号
//						
//						Cang_CGQQY_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[33]<<8)|uartbuff[PACKSITE].recepetion_buff[34]; // 取出传感器启用状态
//						
//					}
//					else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06) 
//					{
//						
//					}
//					Cang_zx_buf[real_subscrib] = 0; // 将对应地址掉线下标清零
//				}
//			}
//			uartbuff[PACKSITE].recepetion_flag = 0; // 清除标志位

//		}
//		
////		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) // 如果获取互斥锁成功
////		{
////			xSemaphoreGive(xMutex); // 必须释放锁
////		}

//		if(timeout_led_ctrl_flag == 1) // 如果为一 则证明有发送
//		{
//			timeout_led_ctrl_flag = 0;
//			if(transimit_addr >= 1 && transimit_addr < 21)
//			{
//				recieve_judge = CU_zx_buf[transimit_addr];
//			}
//			else if(transimit_addr >= 21 && transimit_addr < 45)
//			{
//				recieve_judge = Cang_zx_buf[transimit_addr - 20]; // 获取掉线值 如果不为零 证明本次没有接受到
//			}
//			if(recieve_judge < 2)
//			{
//				timeout_led_ctrl = 2; // 发送成功 接收成功
//			}
//			else
//			{
//				timeout_led_ctrl = 1; // 发送成功 但是接收失败
//			}

//		}

//		osDelay(100);
//	}
//}

// void RS485_01_Receive_Deal(void)
// {
// 	uint16_t crc16 = 0x0000; // CRC校验码
// 	uint8_t subscrib = 0; // 数组下标
// 	if(uartbuff[PACKSITE].recepetion_flag == 1) // 如果接收到了
// 	{
// 		// 计算CRC校验值
// 		crc16 = (uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-1]<<8)|uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-2];
// 		if(CalcCrc16(uartbuff[PACKSITE].recepetion_buff, uartbuff[PACKSITE].recepetion_len - 2) == crc16)
// 		{
// 			if(uartbuff[PACKSITE].recepetion_buff[0] < 21) // 如果是处理包
// 			{
// //					subscrib = online_buff[pack_polling]; // 取出下标
// 				subscrib = uartbuff[PACKSITE].recepetion_buff[0];
// 				CU_zx_buf[subscrib] = 0; // 清楚对应编号的掉线计数
				
// 				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04)
// 				{
// 					// 清空未设置上线的数组或者设置为下线的数组 避免在屏幕上乱显示
// 					for(uint8_t i = cu_tcq_sxzt[subscrib];i < 32; i++)
// 					{
// 						PACK_zx_buf[subscrib][i+1] = 0;
// //							cluster_pack_disconnect_count_buff[subscrib][i + 1] = 0; // 将接收到的簇下标pack掉线值清零
// 						pack_co_concen_buf[subscrib][i+1] = 0;
						
// 						PACK_wendu_buf[subscrib][i+1]=0;
// 						PACK_WDZT_buf[subscrib][i+1]=0;
// 						PACK_YWZT_buf[subscrib][i+1]=0;
// 						PACK_COZT_buf[subscrib][i+1]=0;
// 						PACK_CH4ZT_buf[subscrib][i+1]=0;
// 					}
// 					for(uint8_t i = 0;i < cu_tcq_sxzt[subscrib]; i++) // 计算一个01模块索引下 有多少个pack探测器
// 					{
// 						// 高位是0 直接忽略
// 						PACK_zx_buf[subscrib][i+1] = uartbuff[PACKSITE].recepetion_buff[(i*12)+4];//获取探测器地址
// 						if(PACK_zx_buf[subscrib][i+1])
// 						{
// 							cluster_pack_disconnect_count_buff[subscrib][i + 1] = 0;
// 						}
// 						PACK_wendu_buf[subscrib][i+1] = (uartbuff[PACKSITE].recepetion_buff[(i*12)+5] << 8) | 
// 																						uartbuff[PACKSITE].recepetion_buff[(i*12)+6];//获取温度
						
// 						pack_co_concen_buf[subscrib][i+1] = (uartbuff[PACKSITE].recepetion_buff[(i*12)+9] << 8) | 
// 																								uartbuff[PACKSITE].recepetion_buff[(i*12)+10]; // 获取一氧化碳浓度值
						
// 						uint16_t temp_state = (uartbuff[PACKSITE].recepetion_buff[(i*12)+13] << 8) |
// 																	uartbuff[PACKSITE].recepetion_buff[(i*12)+14]; //获取报警状态
						
// 						PACK_WDZT_buf[subscrib][i+1] = (uint8_t)(temp_state & 0x0003);
						
// 						// 代码逻辑
// 						PACK_YWZT_buf[subscrib][i+1] = STATE_MAP[(temp_state & 0x000C) >> 2];
// 						PACK_COZT_buf[subscrib][i+1] = STATE_MAP[(temp_state & 0x0030) >> 4];
// 						// PACK_COZT_buf[subscrib][i+1] = (uint8_t)(temp_state & 0x0030);
// 						// PACK_YWZT_buf[subscrib][i+1]=uartbuff[PACKSITE].recepetion_buff[(i*12)+9]*256+uartbuff[PACKSITE].recepetion_buff[(i*12)+10];//获取烟雾状态
// 						// PACK_COZT_buf[subscrib][i+1]=uartbuff[PACKSITE].recepetion_buff[(i*12)+11]*256+uartbuff[PACKSITE].recepetion_buff[(i*12)+12];//获取一氧化碳状态
// 						// PACK_CH4ZT_buf[subscrib][i+1]=uartbuff[PACKSITE].recepetion_buff[(i*12)+13]*256+uartbuff[PACKSITE].recepetion_buff[(i*12)+14];//获取甲烷状态
// 					}
// 				}
// 				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06)
// 				{
					
// 				}
// 			}
// 			else if(uartbuff[PACKSITE].recepetion_buff[0] >= 21 && uartbuff[PACKSITE].recepetion_buff[0] < 45) // 如果是XR805
// 			{
// 				uint8_t real_subscrib = uartbuff[PACKSITE].recepetion_buff[0] - 20;
// 				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04) 
// 				{
// 					Cang_wendu_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[3]<<8)|uartbuff[PACKSITE].recepetion_buff[4]; // 取出16位温度数据
// 					Cang_WDZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[5]<<8)|uartbuff[PACKSITE].recepetion_buff[6]; // 取出温度状态
					
// 					Cang_YWZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[9]<<8)|uartbuff[PACKSITE].recepetion_buff[10]; // 取出烟雾状态
					
// 					Cang_CH4ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[13]<<8)|uartbuff[PACKSITE].recepetion_buff[14]; // 取出甲烷状态
					
// 					Cang_COzhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[15]<<8)|uartbuff[PACKSITE].recepetion_buff[16]; // 取出一氧化碳值
// 					Cang_COZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[17]<<8)|uartbuff[PACKSITE].recepetion_buff[18]; // 取出一氧化碳状态
					
// 					Cang_VOCZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[21]<<8)|uartbuff[PACKSITE].recepetion_buff[22]; // 取出VOC状态
					
// 					Cang_H2zhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[23]<<8)|uartbuff[PACKSITE].recepetion_buff[24]; // 取出氢气值
// 					Cang_H2ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[25]<<8)|uartbuff[PACKSITE].recepetion_buff[26]; // 取出氢气状态
					
// 					Cang_TCQXH_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[31]<<8)|uartbuff[PACKSITE].recepetion_buff[32]; // 取出探测器型号
					
// 					Cang_CGQQY_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[33]<<8)|uartbuff[PACKSITE].recepetion_buff[34]; // 取出传感器启用状态
					
// 				}
// 				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06) 
// 				{
					
// 				}
// 				Cang_zx_buf[real_subscrib] = 0; // 将对应地址掉线下标清零
// 			}
// 		}
// 		uartbuff[PACKSITE].recepetion_flag = 0; // 清除标志位

// 	}
	
// 	if(timeout_led_ctrl_flag == 1) // 如果为一 则证明有发送
// 	{
// 		timeout_led_ctrl_flag = 0;
// 		if(transimit_addr >= 1 && transimit_addr < 21)
// 		{
// 			recieve_judge = CU_zx_buf[transimit_addr];
// 		}
// 		else if(transimit_addr >= 21 && transimit_addr < 45)
// 		{
// 			recieve_judge = Cang_zx_buf[transimit_addr - 20]; // 获取掉线值 如果不为零 证明本次没有接受到
// 		}
// 		if(recieve_judge < 2)
// 		{
// 			timeout_led_ctrl = 2; // 发送成功 接收成功
// 		}
// 		else
// 		{
// 			timeout_led_ctrl = 1; // 发送成功 但是接收失败
// 		}

// 	}
// }

// void RS485_01_Receive_Deal_Plus(void)
// {
// 	uint16_t crc16 = 0x0000; // CRC校验码
// 	uint8_t subscrib = 0; // 数组下标
// 	if(uartbuff[PACKSITE].recepetion_flag == 1) // 如果接收到了
// 	{
// 		// 计算CRC校验值
// 		crc16 = (uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-1]<<8)|uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-2];
// 		if(CalcCrc16(uartbuff[PACKSITE].recepetion_buff, uartbuff[PACKSITE].recepetion_len - 2) == crc16)
// 		{
// 			if(uartbuff[PACKSITE].recepetion_buff[0] < 21) // 如果是处理包
// 			{
// //					subscrib = online_buff[pack_polling]; // 取出下标
// 				subscrib = uartbuff[PACKSITE].recepetion_buff[0];
// 				CU_zx_buf[subscrib] = 0; // 清楚对应编号的掉线计数
				
// 				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04)
// 				{
// 					for(uint8_t i = 0; i < 32; i++) // 计算一个01模块索引下 有多少个pack探测器
// 					{
// 						if(pack_online_buff[subscrib][i + 1] == 0)
// 						{
// 							PACK_zx_buf[subscrib][i+1] = 0;
// //							cluster_pack_disconnect_count_buff[subscrib][i + 1] = 0; // 将接收到的簇下标pack掉线值清零
// 							pack_co_concen_buf[subscrib][i+1] = 0;
							
// 							PACK_wendu_buf[subscrib][i+1]=0;
// 							PACK_WDZT_buf[subscrib][i+1]=0;
// 							PACK_YWZT_buf[subscrib][i+1]=0;
// 							PACK_COZT_buf[subscrib][i+1]=0;
// 							PACK_CH4ZT_buf[subscrib][i+1]=0;
// 						}
// 						else
// 						{
// 							// 高位是0 直接忽略
// 							PACK_zx_buf[subscrib][i+1] = uartbuff[PACKSITE].recepetion_buff[(i*12)+4];//获取探测器地址
// 							if(PACK_zx_buf[subscrib][i+1])
// 							{
// 								cluster_pack_disconnect_count_buff[subscrib][i + 1] = 0;
// 							}
// 							PACK_wendu_buf[subscrib][i+1] = (uartbuff[PACKSITE].recepetion_buff[(i*12)+5] << 8) | 
// 																							uartbuff[PACKSITE].recepetion_buff[(i*12)+6];//获取温度
							
// 							pack_co_concen_buf[subscrib][i+1] = (uartbuff[PACKSITE].recepetion_buff[(i*12)+9] << 8) | 
// 																									uartbuff[PACKSITE].recepetion_buff[(i*12)+10]; // 获取一氧化碳浓度值
							
// 							uint16_t temp_state = (uartbuff[PACKSITE].recepetion_buff[(i*12)+13] << 8) |
// 																		uartbuff[PACKSITE].recepetion_buff[(i*12)+14]; //获取报警状态
							
// 							PACK_WDZT_buf[subscrib][i+1] = (uint8_t)(temp_state & 0x0003);
							
// 							// 代码逻辑
// 							PACK_YWZT_buf[subscrib][i+1] = STATE_MAP[(temp_state & 0x000C) >> 2];
// 							PACK_COZT_buf[subscrib][i+1] = STATE_MAP[(temp_state & 0x0030) >> 4];
// 						}
// 					}
// 				}
// 				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06)
// 				{
					
// 				}
// 			}
// 			else if(uartbuff[PACKSITE].recepetion_buff[0] >= 21 && uartbuff[PACKSITE].recepetion_buff[0] < 45) // 如果是XR805
// 			{
// 				uint8_t real_subscrib = uartbuff[PACKSITE].recepetion_buff[0] - 20;
// 				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04) 
// 				{
// 					Cang_wendu_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[3]<<8)|uartbuff[PACKSITE].recepetion_buff[4]; // 取出16位温度数据
// 					Cang_WDZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[5]<<8)|uartbuff[PACKSITE].recepetion_buff[6]; // 取出温度状态
					
// 					Cang_YWZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[9]<<8)|uartbuff[PACKSITE].recepetion_buff[10]; // 取出烟雾状态
					
// 					Cang_CH4ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[13]<<8)|uartbuff[PACKSITE].recepetion_buff[14]; // 取出甲烷状态
					
// 					Cang_COzhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[15]<<8)|uartbuff[PACKSITE].recepetion_buff[16]; // 取出一氧化碳值
// 					Cang_COZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[17]<<8)|uartbuff[PACKSITE].recepetion_buff[18]; // 取出一氧化碳状态
					
// 					Cang_VOCZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[21]<<8)|uartbuff[PACKSITE].recepetion_buff[22]; // 取出VOC状态
					
// 					Cang_H2zhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[23]<<8)|uartbuff[PACKSITE].recepetion_buff[24]; // 取出氢气值
// 					Cang_H2ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[25]<<8)|uartbuff[PACKSITE].recepetion_buff[26]; // 取出氢气状态
					
// 					Cang_TCQXH_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[31]<<8)|uartbuff[PACKSITE].recepetion_buff[32]; // 取出探测器型号
					
// 					Cang_CGQQY_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[33]<<8)|uartbuff[PACKSITE].recepetion_buff[34]; // 取出传感器启用状态
					
// 				}
// 				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06) 
// 				{
					
// 				}
// 				Cang_zx_buf[real_subscrib] = 0; // 将对应地址掉线下标清零
// 			}
// 		}
// 		uartbuff[PACKSITE].recepetion_flag = 0; // 清除标志位

// 	}
	
// 	if(timeout_led_ctrl_flag == 1) // 如果为一 则证明有发送
// 	{
// 		timeout_led_ctrl_flag = 0;
// 		if(transimit_addr >= 1 && transimit_addr < 21)
// 		{
// 			recieve_judge = CU_zx_buf[transimit_addr];
// 		}
// 		else if(transimit_addr >= 21 && transimit_addr < 45)
// 		{
// 			recieve_judge = Cang_zx_buf[transimit_addr - 20]; // 获取掉线值 如果不为零 证明本次没有接受到
// 		}
// 		if(recieve_judge < 2)
// 		{
// 			timeout_led_ctrl = 2; // 发送成功 接收成功
// 		}
// 		else
// 		{
// 			timeout_led_ctrl = 1; // 发送成功 但是接收失败
// 		}

// 	}
// }


// void RS485_01_Receive_Deal_Ultra(void)
// {
// 	uint16_t crc16 = 0x0000; // CRC校验码
// 	uint8_t subscrib = 0; // 数组下标
// 	uint8_t	package_subscrib = 0;
// 	if(uartbuff[PACKSITE].recepetion_flag == 1) // 如果接收到了
// 	{
// 		// 计算CRC校验值
// 		crc16 = (uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-1]<<8) | 
// 							uartbuff[PACKSITE].recepetion_buff[uartbuff[PACKSITE].recepetion_len-2];
// 		if(CalcCrc16(uartbuff[PACKSITE].recepetion_buff, uartbuff[PACKSITE].recepetion_len - 2) == crc16)
// 		{
// 			if(uartbuff[PACKSITE].recepetion_buff[0] < 21) // 如果是处理包
// 			{
// 				// 回路ID
// 				subscrib = uartbuff[PACKSITE].recepetion_buff[0];
// 				// 回复探测器ID
// 				package_subscrib = (uartbuff[PACKSITE].recepetion_buff[3] << 8) | uartbuff[PACKSITE].recepetion_buff[4];
				
// 				CU_zx_buf[subscrib] = 0; // 清楚对应编号的掉线计数
				
// 				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04)
// 				{
// 					// 高位是0 直接忽略
// 					PACK_zx_buf[subscrib][package_subscrib] = package_subscrib;//获取探测器地址
// 					if(package_subscrib)
// 					{
// 						cluster_pack_disconnect_count_buff[subscrib][package_subscrib] = 0; // 清空掉线次数
// 					}
// 					PACK_wendu_buf[subscrib][package_subscrib] = (uartbuff[PACKSITE].recepetion_buff[5] << 8) | 
// 																					uartbuff[PACKSITE].recepetion_buff[6];//获取温度
					
// 					pack_co_concen_buf[subscrib][package_subscrib] = (uartbuff[PACKSITE].recepetion_buff[9] << 8) | 
// 																							uartbuff[PACKSITE].recepetion_buff[10]; // 获取一氧化碳浓度值
					
// 					uint16_t temp_state = (uartbuff[PACKSITE].recepetion_buff[13] << 8) |
// 																uartbuff[PACKSITE].recepetion_buff[14]; //获取报警状态
					
// 					PACK_WDZT_buf[subscrib][package_subscrib] = (uint8_t)(temp_state & 0x0003);
					
// 					// 代码逻辑
// 					PACK_YWZT_buf[subscrib][package_subscrib] = STATE_MAP[(temp_state & 0x000C) >> 2];
// 					PACK_COZT_buf[subscrib][package_subscrib] = STATE_MAP[(temp_state & 0x0030) >> 4];
// 				}
// 				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06)
// 				{
					
// 				}
// 			}
// 			else if(uartbuff[PACKSITE].recepetion_buff[0] >= 21 && uartbuff[PACKSITE].recepetion_buff[0] < 45) // 如果是XR805
// 			{
// 				uint8_t real_subscrib = uartbuff[PACKSITE].recepetion_buff[0] - 20;
// 				if(uartbuff[PACKSITE].recepetion_buff[1] == 0x04) 
// 				{
// 					Cang_wendu_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[3]<<8)|uartbuff[PACKSITE].recepetion_buff[4]; // 取出16位温度数据
// 					Cang_WDZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[5]<<8)|uartbuff[PACKSITE].recepetion_buff[6]; // 取出温度状态
					
// 					Cang_YWZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[9]<<8)|uartbuff[PACKSITE].recepetion_buff[10]; // 取出烟雾状态
					
// 					Cang_CH4ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[13]<<8)|uartbuff[PACKSITE].recepetion_buff[14]; // 取出甲烷状态
					
// 					Cang_COzhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[15]<<8)|uartbuff[PACKSITE].recepetion_buff[16]; // 取出一氧化碳值
// 					Cang_COZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[17]<<8)|uartbuff[PACKSITE].recepetion_buff[18]; // 取出一氧化碳状态
					
// 					Cang_VOCZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[21]<<8)|uartbuff[PACKSITE].recepetion_buff[22]; // 取出VOC状态
					
// 					Cang_H2zhi_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[23]<<8)|uartbuff[PACKSITE].recepetion_buff[24]; // 取出氢气值
// 					Cang_H2ZT_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[25]<<8)|uartbuff[PACKSITE].recepetion_buff[26]; // 取出氢气状态
					
// 					Cang_TCQXH_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[31]<<8)|uartbuff[PACKSITE].recepetion_buff[32]; // 取出探测器型号
					
// 					Cang_CGQQY_buf[real_subscrib] = (uartbuff[PACKSITE].recepetion_buff[33]<<8)|uartbuff[PACKSITE].recepetion_buff[34]; // 取出传感器启用状态
					
// 				}
// 				else if(uartbuff[PACKSITE].recepetion_buff[1] == 0x06) 
// 				{
					
// 				}
// 				Cang_zx_buf[real_subscrib] = 0; // 将对应地址掉线下标清零
// 			}
// 		}
// 		uartbuff[PACKSITE].recepetion_flag = 0; // 清除标志位

// 	}
	
// 	if(timeout_led_ctrl_flag == 1) // 如果为一 则证明有发送
// 	{
// 		timeout_led_ctrl_flag = 0;
// 		if(transimit_addr >= 1 && transimit_addr < 21)
// 		{
// 			recieve_judge = CU_zx_buf[transimit_addr];
// 		}
// 		else if(transimit_addr >= 21 && transimit_addr < 45)
// 		{
// 			recieve_judge = Cang_zx_buf[transimit_addr - 20]; // 获取掉线值 如果不为零 证明本次没有接受到
// 		}
// 		if(recieve_judge < 2)
// 		{
// 			timeout_led_ctrl = 2; // 发送成功 接收成功
// 		}
// 		else
// 		{
// 			timeout_led_ctrl = 1; // 发送成功 但是接收失败
// 		}

// 	}
// }

// void RS485_01_PollAndRecieve(void * parameter)
// {
// 	uint8_t current_poll_index = 0;  // 当前轮询的设备索引
// 	uint8_t modbusbuf[8] = {0};      // 发送缓冲区

// 	uint32_t current_time = osKernelGetTickCount(); // 系统当前时间戳
// 	// 初始化各设备的上次轮询时间
// 	for(int i = 0; i < DEVICE_TYPE_COUNT; i++) {
// 		poll_configs[i].last_poll_time = current_time;
// 	}
	
// 	for(;;)
// 	{
// 		// 先判断处理
// 		RS485_01_Receive_Deal_Ultra();

// 		current_time = osKernelGetTickCount();
// 		for(int count = 0; count < DEVICE_TYPE_COUNT; count++) 
// 		{
// 			uint8_t i = (current_poll_index + count) % DEVICE_TYPE_COUNT; // 尝试从current_poll_index开始找下一个可发送设备
			
// 			if(current_time - poll_configs[i].last_poll_time >= poll_configs[i].poll_interval) 
// 			{
// 				current_poll_index = (i + 1) % DEVICE_TYPE_COUNT; // 确保下个设备开始
	
// 				poll_configs[i].poll_function();                  // 回调发送函数
// 				// 执行完发送后才标记时间
// 				poll_configs[i].last_poll_time = current_time;    // 记录发送时间
// 			}
// 		}
		
// 		if(xQueueReceive(xMyRs485QueueHandle, modbusbuf, 0) == pdTRUE)
// 		{
// 			PackBehaviorManageSendString(modbusbuf, sizeof(modbusbuf));
// 			timeout_led_ctrl_flag = 1;
// 		}

// 		osDelay(50);
// 	}
// }






