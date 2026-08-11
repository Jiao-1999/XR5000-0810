/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_itcallback.h"
#include "bsp_debug.h"
#include "bsp_screen.h"

#include "hmi_driver.h"
#include "hmi_user_uart.h"
#include "cmd_queue.h"
#include "cmd_process.h"

#include "usart.h"
#include "bsp_mbus.h"
#include "bsp_mbus_control.h"
#include "bsp_rs485_01.h"

#include "bsp_adc.h"
#include "bsp_key.h"

#include "bsp_rtc.h"
#include "bsp_logic_set.h"

#include "bsp_bms.h"

#include "bsp_linkage_ctrl.h"

#include "bsp_super.h"

#include "queue.h"
#include "semphr.h"

#include "bsp_internal_board.h"
#include "bsp_save_ctrl.h"
#include "bsp_station.h"
#include "bsp_ctrl_bus.h"
#include "bsp_rs485_detect.h"
#include "bsp_fdcan1.h"
#include "bsp_can_monitor.h" /* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
#include "bsp_aht20.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// 01模块轮询任务
osThreadId_t PackPollingTaskHandle; // 
const osThreadAttr_t PackPollingTask_attributes = {
	.name = "pack_pollingTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// PACK485 polling task
osThreadId_t PackPollAndReceiveTaskHandle; // 
const osThreadAttr_t PackPollAndReceiveTask_attributes = {
	.name = "PackPollAndReceiveTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// BSP485PACK轮询任务
osThreadId_t BSP485PollTaskHandle; // 任务句柄
const osThreadAttr_t BSP485PollTask_attributes = {
	.name = " BSP485_PollTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};

// 485队列模块轮询任务
osThreadId_t QueuePollingTaskHandle; // ������ѯ̽����������
const osThreadAttr_t QueuePollingTask_attributes = {
	.name = "pack_pollingTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 包接收处理函数
osThreadId_t QueueRcevDealTaskHandle; // ������ѯ̽����������
const osThreadAttr_t QueueRcevDealTask_attributes = {
	.name = "QueueRcevDealTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 内屏
osThreadId_t InternalScreenTaskHandle; // ������ѯ̽����������
const osThreadAttr_t InternalScreen_attributes = {
	.name = "InternalScreenTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal3,
};
// 内屏刷新任务
osThreadId_t InterScreenUpdataTaskHandle; // ������ѯ̽����������
const osThreadAttr_t InterScreenUpdataTask_attributes = {
	.name = "InterScreenUpdataTask",
	.stack_size = 384 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};
// 外联设备状态判断
osThreadId_t LinkageOnlineTaskHandle; // 句柄
const osThreadAttr_t LinkageOnlineTask_attributes = {
	.name = "LinkageOnlineJudgeTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};
// 按键设备状态判断
osThreadId_t KeyOnlineJudgeTaskHandle; // 句柄
const osThreadAttr_t KeyOnlineJudgeTask_attributes = {
	.name = "KeyOnlineJudgeTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};
// 主备电状态判断 及电流监测
osThreadId_t MainStandbyPwrJudgeTaskHandle; // 句柄
const osThreadAttr_t MSPwrJudgeTask_attributes = {
	.name = "MSPJudgeAndCurrTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityAboveNormal,
};
// BMS查询回复任务
osThreadId_t BMSRecvDealTaskHandle; // 句柄
const osThreadAttr_t BMSRecvDealTask_attributes = {
	.name = "BMSRecvDeal",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

// 外联设备控制任务
osThreadId_t LinkageCtrlTaskHandle; // 句柄
const osThreadAttr_t LinkageCtrlTask_attributes = {
	.name = "LinkageCtrl",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

// 主机状态上行任务
osThreadId_t HostUploadTaskHandle; // 句柄
const osThreadAttr_t HostUploadTask_attributes = {
	.name = "HostUploadCtrl",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};
// 内屏板接收处理
osThreadId_t InternalBoardRecvTaskHandle; // 句柄
const osThreadAttr_t InternalBoardRecvTask_attributes = {
	.name = "InternalBoardRecv",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};
//
osThreadId_t StationResponseTaskHandle; // 句柄
const osThreadAttr_t StationResponseTask_attributes = {
	.name = "StationResponse",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 2025/10/29 09:49 创建
//osThreadId_t CtrlBusPollingTaskHandle; // ctrl bus send task
//const osThreadAttr_t CtrlBusPollingTask_attributes = {
//	.name = "CtrlBusPolling",
//	.stack_size = 128 * 4,
//	.priority = (osPriority_t) osPriorityNormal1,
//};

//osThreadId_t CtrlBusReceiveTaskHandle; // ctrl bus receive task
//const osThreadAttr_t CtrlBusReceiveTask_attributes = {
//	.name = "CtrlBusReceive",
//	.stack_size = 128 * 4,
//	.priority = (osPriority_t) osPriorityNormal1,
//};

// 2025/11/19 18:35 创建 优化任务处理 合并两个任务 减少栈区消耗
osThreadId_t CtrlBusPollAndReceiveTaskHandle; // ctrl bus send task
const osThreadAttr_t CtrlBusPollAndReceiveTask_attributes = {
	.name = "CtrlBusPollAndReceiveTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 回路3 RS485探测轮询任务 (XR805 + XR8303)
osThreadId_t RS485DetectPollAndReceiveTaskHandle;
const osThreadAttr_t RS485DetectPollAndReceiveTask_attributes = {
	.name = "RS485DetectPollAndRecv",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 2025/11/17 11:24 创建
//osThreadId_t MBusPollingTaskHandle; // Mbus send task
//const osThreadAttr_t MBusPollingTask_attributes = {
//	.name = "MBusPolling",
//	.stack_size = 128 * 4,
//	.priority = (osPriority_t) osPriorityNormal1,
//};

//osThreadId_t MBusReceiveTaskHandle; // Mbus receive task
//const osThreadAttr_t MBusReceiveTask_attributes = {
//	.name = "MBusReceive",
//	.stack_size = 128 * 4,
//	.priority = (osPriority_t) osPriorityNormal1,
//};

// 2025/11/19 18:24 创建 优化任务处理 合并两个任务 减少栈区消耗
osThreadId_t MBus1PollAndReceiveTaskHandle; // Mbus receive task
const osThreadAttr_t MBus1PollAndReceiveTask_attributes = {
	.name = "MBusPollAndReceiveTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

osThreadId_t MBus2PollAndReceiveTaskHandle;
const osThreadAttr_t MBus2PollAndReceiveTask_attributes = {
	.name = "MBus2PollAndReceiveTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 2025/11/20 18:11 创建 优化任务处理 合并两个任务 减少栈区消耗
osThreadId_t CanMonitorTaskHandle; /* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
const osThreadAttr_t CanMonitorTask_attributes = {
	.name = "CanMonitorTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};


extern uint8_t timeout_led_ctrl;

// 定义485总线发送消息队列
QueueHandle_t xMyRs485QueueHandle;
// XR5000_UART5_EXCLUSIVE_FIX_20260730: legacy fan traffic has its own disabled transport queue.
QueueHandle_t xFanBusQueueHandle;
// 定义MBUS总线发送消息队列
QueueHandle_t xMyMBusQueueHandle;
// 定义MBUS2总线发送消息队列（回路2 二总线控制）
QueueHandle_t xMBus2QueueHandle;


//StaticSemaphore_t xMutexBuffer;
//SemaphoreHandle_t xMutex; // 初始化互斥锁


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led_test */
osThreadId_t led_testHandle;
const osThreadAttr_t led_test_attributes = {
  .name = "led_test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SecondTimer */
osTimerId_t SecondTimerHandle;
const osTimerAttr_t SecondTimer_attributes = {
  .name = "SecondTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void InterScreenFresh(void* parameter);

int8_t SendDataToFanBusQueue(uint8_t *buf, uint8_t buf_len);

int8_t ReceiveDataFromFanBusQueue(uint8_t *buf);

void DeletUpdataUITask(void);
void CreatUpdataUITask(void);

void SuspendTask(uint8_t task_id);
void ResumeTask(uint8_t task_id);




/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LED_TEST(void *argument);
void SecondTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  HmiTxInit(); /* XR5000_HMI_UART_LOCK_FIX_20260729: create UART8 frame mutex before screen tasks. */
  BspSaveCtrlInit(); /* XR5000_W25Q_MUTEX_FIX_20260804: create the shared Flash mutex before tasks. */
  /* add mutexes, ... */
//	xMutex = xSemaphoreCreateMutexStatic(&xMutexBuffer); // 初始化互斥锁
	
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of SecondTimer */
  SecondTimerHandle = osTimerNew(SecondTimerCallback, osTimerPeriodic, NULL, &SecondTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(SecondTimerHandle, 1000);  // 设置1秒周期
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	xMyRs485QueueHandle = xQueueCreate(10, sizeof(char[8])); // 创建一个最多存储10条消息的队列，每条消息是8字节的char数组
	
	xFanBusQueueHandle = xQueueCreate( 5, sizeof(char[8])); // XR5000_UART5_EXCLUSIVE_FIX_20260730: fan-only queue; UART5 is not its transport.
	
	xMyMBusQueueHandle = xQueueCreate( 5, sizeof(char[8])); // 创建一个最多存储5条消息的队列，每条消息是8字节的char数组
	
	xMBus2QueueHandle = xQueueCreate( 5, sizeof(char[8])); // 创建一个最多存储5条消息的队列，每条消息是8字节的char数组（回路2 二总线控制）

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of led_test */
  led_testHandle = osThreadNew(LED_TEST, NULL, &led_test_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	// 内屏接收处理任务
	InternalScreenTaskHandle = osThreadNew(InternalScreenRecvDealTask, NULL, &InternalScreen_attributes);
//	// 屏幕UI更新任务
	InterScreenUpdataTaskHandle = osThreadNew(InterScreenFresh, NULL, &InterScreenUpdataTask_attributes);

//	//BSP485 PACK POLL TASK
//	PackPollingTaskHandle = osThreadNew(BSP_RS485_01_Poll, NULL, &BSP485PollTask_attributes);
//	// 消息队列轮询后 接收处理任务
//	QueueRcevDealTaskHandle = osThreadNew(QueuePollRecvDealTask, NULL, &QueueRcevDealTask_attributes);
	
	// 2025/11/25 11:03 启用新功能
	PackPollAndReceiveTaskHandle = osThreadNew(RS485_01_PollAndRecieve, NULL, &PackPollAndReceiveTask_attributes);
	
	
	// 2025/11/17 17:22 启用MBUS
//	// 
//	MBusPollingTaskHandle = osThreadNew(MBus1PollSlaveTsk, NULL, &MBusPollingTask_attributes);
//	// 
//	MBusReceiveTaskHandle = osThreadNew(MBus1RecvDealTask, NULL, &MBusReceiveTask_attributes);
	
	// // 2025/11/19 18:24 创建 优化任务处理 合并两个任务 减少栈区消耗
	MBus1PollAndReceiveTaskHandle = osThreadNew(MBus1PollSlaveAndReceiveTask, NULL, &MBus1PollAndReceiveTask_attributes);
	
	MBus2PollAndReceiveTaskHandle = osThreadNew(MBusControlPollSlaveAndReceiveTask, NULL, &MBus2PollAndReceiveTask_attributes);
	
	// 主机状态上行
	HostUploadTaskHandle = osThreadNew(HostUploadDataTask, NULL, &HostUploadTask_attributes);
	
	// 外联设备在线掉线状态判断
	LinkageOnlineTaskHandle = osThreadNew(LinkageOnlineJudgeTask, NULL, &LinkageOnlineTask_attributes);
	// 按键设备在线掉线状态判断
	KeyOnlineJudgeTaskHandle = osThreadNew(KeyStateJudgeTask, NULL, &KeyOnlineJudgeTask_attributes);
	// 场站回复任务
	StationResponseTaskHandle = osThreadNew(StationResponseTesk, NULL, &StationResponseTask_attributes);
	
	// 主备电及电流状态判断
	MainStandbyPwrJudgeTaskHandle = osThreadNew(PowerOnlineJudgeTask, NULL, &MSPwrJudgeTask_attributes);
	
	// CtrlBusPollingTask
//	CtrlBusPollingTaskHandle = osThreadNew(CtrlBusPollingTask, NULL, &CtrlBusPollingTask_attributes);
//	//
//	CtrlBusReceiveTaskHandle = osThreadNew(CtrlBusReceiveDealTask, NULL, &CtrlBusReceiveTask_attributes);
	
	// 2025/11/19 18:36 创建 优化任务处理 合并两个任务 减少栈区消耗
//	CtrlBusPollAndReceiveTaskHandle = osThreadNew(CtrlBusPollAndReceiveTask, NULL, &CtrlBusPollAndReceiveTask_attributes);
	// 注：原CtrlBusPollAndReceiveTask已停用，uart5由回路3 RS485探测任务接管

	// 回路3 RS485探测轮询任务 (XR805 + XR8303)
	RS485DetectPollAndReceiveTaskHandle = osThreadNew(RS485DetectPollAndReceiveTask, NULL, &RS485DetectPollAndReceiveTask_attributes);
	
	// 2025/11/20 18:12 
	CanMonitorTaskHandle = osThreadNew(CanMonitorTask, NULL, &CanMonitorTask_attributes); /* 新加功能：FCP-1011六路控制板；时间：2026-08-06 */
	
	// BMS查询回复任务 2025/11/28 11:49 启用该任务
	BMSRecvDealTaskHandle = osThreadNew(BMSRecvDealTask, NULL, &BMSRecvDealTask_attributes);
//	// 外联设备继电器控制
//	LinkageCtrlTaskHandle = osThreadNew(LinkageRelayCtrlTask, NULL, &LinkageCtrlTask_attributes);
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
//	uint8_t sum = 0;
//	uint8_t read_sum;
	size_t freeHeap;
//	uint8_t fresh_screen_timeout = 0;
	
//	uint8_t test_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
	
	osDelay(1000);
	// 存储开机时间
	BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_TURN_ON, LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID);
	
	osDelay(2000);
	SetInternalScreenRTCTime();
	GetScreen();
	// XR5000_AHT20_CHANGE_20260727: initialize PD9/PD10 AHT20 sensor after GPIO is ready.
	AHT20_Init();
	(void)AHT20_Update();
	MBusCtrl_Init();
	// 内屏板子接收处理函数 开机三秒后才创建该任务
	InternalBoardRecvTaskHandle = osThreadNew(InternalScreenBoradRecvDealTask, NULL, &InternalBoardRecvTask_attributes);
  /* Infinite loop */
  for(;;)
  {
		freeHeap = xPortGetFreeHeapSize();
		HAL_IWDG_Refresh(&hiwdg1);
		// XR5000_AHT20_CHANGE_20260727: refresh cached home-screen temperature/humidity.
		(void)AHT20_Update();
//		DebugPrintf("maintain:%d\r\n", freeHeap);

//		FDCAN1_SendStdDataFrame(0x123, test_data, 8);
		osDelay(2000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LED_TEST */
/**
* @brief Function implementing the led_test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_TEST */
void LED_TEST(void *argument)
{
  /* USER CODE BEGIN LED_TEST */
	uint32_t last_tick = osKernelGetTickCount();
	uint8_t led_state = 0; // 0:关闭, 1:红灯闪烁, 2:绿灯闪烁
	
  /* Infinite loop */
  for(;;)
  {
		// 处理外部控制信号
		if(timeout_led_ctrl == 1) { // 故障状态
				led_state = 1;
				timeout_led_ctrl = 0;
				HAL_GPIO_WritePin(PMLED_G_GPIO_Port, PMLED_G_Pin, GPIO_PIN_SET); // 确保绿灯关闭
		} 
		else if(timeout_led_ctrl == 2) { // 正常状态
				led_state = 2;
				timeout_led_ctrl = 0;
				HAL_GPIO_WritePin(PMLED_R_GPIO_Port, PMLED_R_Pin, GPIO_PIN_SET); // 确保红灯关闭
		}

		// 处理LED闪烁（500ms周期）
		if(osKernelGetTickCount() - last_tick >= 500) {
				last_tick = osKernelGetTickCount();
				
				if(led_state == 1) { // 红灯闪烁
						HAL_GPIO_TogglePin(PMLED_R_GPIO_Port, PMLED_R_Pin);
				} 
				else if(led_state == 2) { // 绿灯闪烁
						HAL_GPIO_TogglePin(PMLED_G_GPIO_Port, PMLED_G_Pin);
				}
		}

		// 独立处理的屏幕LED（500ms周期）
		HAL_GPIO_TogglePin(PWLED_G_GPIO_Port, PWLED_G_Pin);
		osDelay(500); // 直接延迟500ms
  }
  /* USER CODE END LED_TEST */
}

/* SecondTimerCallback function */
void SecondTimerCallback(void *argument)
{
  /* USER CODE BEGIN SecondTimerCallback */

	baojingjishi++;
	
	if(button_ctrl.press_flag == 1)
	{
		button_ctrl.timout_count++;
	}
  /* USER CODE END SecondTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void InterScreenFresh(void* parameter)
{
	osDelay(1000);
	for(;;)
	{
		UpdateUI();
    osDelay(100);
	}
}

int8_t SendDataToFanBusQueue(uint8_t *buf, uint8_t buf_len)
{
	BaseType_t send_state = pdFALSE;
	
	if(buf_len == 0 || buf_len > 8)
		return -1;
	send_state = xQueueSend(xFanBusQueueHandle, buf, 0);
	
	return send_state;
}

int8_t ReceiveDataFromFanBusQueue(uint8_t *buf)
{
	BaseType_t send_state = pdFALSE;
	if(buf == NULL)
	{
		return -1;
	}
	send_state = xQueueReceive(xFanBusQueueHandle, buf, 0);
	
	return send_state;
}

int8_t SendDataToMBusQueue(uint8_t *buf, uint8_t buf_len)
{
	if(buf == NULL || buf_len > 8)
	{
		return -1;
	}
	if( xQueueSend(xMyMBusQueueHandle, buf, 0) == pdFALSE )
	{
		return 0;
	}
	return 1;
}

int8_t ReceiveDataFromMBus(uint8_t *buf)
{
	if(buf == NULL)
	{
		return -1;
	}
	if(xQueueReceive(xMyMBusQueueHandle, buf, 0) == pdFALSE)
	{
		return 0;
	}
	return 1;
}

void DeletUpdataUITask(void)
{
	vTaskDelete(InterScreenUpdataTaskHandle);
}
void CreatUpdataUITask(void)
{
	//	// 屏幕UI更新任务
	InterScreenUpdataTaskHandle = osThreadNew(InterScreenFresh, NULL, &InterScreenUpdataTask_attributes);
}

// 
void SuspendTask(uint8_t task_id)
{
	switch(task_id)
	{
		case 1:{
			vTaskSuspend(MBus1PollAndReceiveTaskHandle); // 挂起二总线1
			break;
		}
		case 2:{
			
			break;
		}
		case 3:{
			vTaskSuspend(PackPollAndReceiveTaskHandle); // 挂起PACK 485总线
			break;
		}
		case 4:{
			vTaskSuspend(BMSRecvDealTaskHandle); // 挂起BMS总线
			break;
		}
		case 5:{
			vTaskSuspend(StationResponseTaskHandle); // 挂起 场站 总线
			break;
		}
		case 6:{
//			vTaskSuspend(CtrlBusPollAndReceiveTaskHandle); // 已停用，uart5由回路3接管
			break;
		}
		case 7:{
			vTaskSuspend(RS485DetectPollAndReceiveTaskHandle); // 挂起回路3 RS485探测
			break;
		}
		default:{
			break;
		}
	}
}

// 
void ResumeTask(uint8_t task_id)
{
	switch(task_id)
	{
		case 1:{
			vTaskResume(MBus1PollAndReceiveTaskHandle); // 挂起二总线1
			break;
		}
		case 2:{
			break;
		}
		case 3:{
			vTaskResume(PackPollAndReceiveTaskHandle); // 挂起PACK 485总线
			break;
		}
		case 4:{
			vTaskResume(BMSRecvDealTaskHandle); // 挂起BMS总线
			break;
		}
		case 5:{
			vTaskResume(StationResponseTaskHandle); // 挂起 场站 总线
			break;
		}
		case 6:{
//			vTaskResume(CtrlBusPollAndReceiveTaskHandle); // 已停用，uart5由回路3接管
			break;
		}
		case 7:{
			vTaskResume(RS485DetectPollAndReceiveTaskHandle); // 恢复回路3 RS485探测
			break;
		}
		default:{
			break;
		}
	}
}


/* USER CODE END Application */
