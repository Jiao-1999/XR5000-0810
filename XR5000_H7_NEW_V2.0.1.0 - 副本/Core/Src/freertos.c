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
#include "bsp_can_monitor.h" /* ???????FCP-1011??·????壻???2026-08-06 */
#include "bsp_aht20.h"
#include "iwdg.h"
#include "bsp_storage_tx.h"  /* ?洢???: LPUART1(PB6/PB7) */
#include "bsp_storage_event.h" /* 黑匣子事件记录API: 开机等GB4717-2024事件 */
#include "bsp_fecbus.h"      /* FECbus RS485 ???: USART3(PB10/PB11), GB4717 ???C */
#include "bsp_test_inject.h" /* COM4??????????: UART4 RX?????, ??DMA?ж?????У?? */
#include "bsp_logic_engine.h" /* Linkage logic engine task: rule eval + action exec, 100ms cycle */
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

// 01??????????
osThreadId_t PackPollingTaskHandle; // ???
const osThreadAttr_t PackPollingTask_attributes = {
	.name = "pack_pollingTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// PACK485 ???????
osThreadId_t PackPollAndReceiveTaskHandle; // ???
const osThreadAttr_t PackPollAndReceiveTask_attributes = {
	.name = "PackPollAndReceiveTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// BSP485 PACK???????
osThreadId_t BSP485PollTaskHandle; // ???
const osThreadAttr_t BSP485PollTask_attributes = {
	.name = " BSP485_PollTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};

// 485??????????????
osThreadId_t QueuePollingTaskHandle; // ???
const osThreadAttr_t QueuePollingTask_attributes = {
	.name = "pack_pollingTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// ?????????????
osThreadId_t QueueRcevDealTaskHandle; // ???
const osThreadAttr_t QueueRcevDealTask_attributes = {
	.name = "QueueRcevDealTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// ????
osThreadId_t InternalScreenTaskHandle; // ???
const osThreadAttr_t InternalScreen_attributes = {
	.name = "InternalScreenTask",
	.stack_size = 384 * 4,  /* enlarged: NotifyButton ctx runs DebugPrintf vsnprintf frame */
	.priority = (osPriority_t) osPriorityNormal3,
};

/* Debug log pump task: drain ring queue to UART4 async, never blocks callers */
osThreadId_t DebugPrintTaskHandle;
const osThreadAttr_t DebugPrintTask_attributes = {
	.name = "DebugPrintTask",
	.stack_size = 1024,
	.priority = (osPriority_t) osPriorityBelowNormal,
};
// ???????????
osThreadId_t InterScreenUpdataTaskHandle; // ???
const osThreadAttr_t InterScreenUpdataTask_attributes = {
	.name = "InterScreenUpdataTask",
	.stack_size = 384 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};
// ?????豸???ж?
osThreadId_t LinkageOnlineTaskHandle; // ???
const osThreadAttr_t LinkageOnlineTask_attributes = {
	.name = "LinkageOnlineJudgeTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};
// ?????豸???ж?
osThreadId_t KeyOnlineJudgeTaskHandle; // ???
const osThreadAttr_t KeyOnlineJudgeTask_attributes = {
	.name = "KeyOnlineJudgeTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};
// ?????????ж? ?????????
osThreadId_t MainStandbyPwrJudgeTaskHandle; // ???
const osThreadAttr_t MSPwrJudgeTask_attributes = {
	.name = "MSPJudgeAndCurrTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityAboveNormal,
};
// BMS??????????
osThreadId_t BMSRecvDealTaskHandle; // ???
const osThreadAttr_t BMSRecvDealTask_attributes = {
	.name = "BMSRecvDeal",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

// ?????豸????????
osThreadId_t LinkageCtrlTaskHandle; // ???
const osThreadAttr_t LinkageCtrlTask_attributes = {
	.name = "LinkageCtrl",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

// ??????????????
osThreadId_t HostUploadTaskHandle; // ???
const osThreadAttr_t HostUploadTask_attributes = {
	.name = "HostUploadCtrl",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};
// ????????????
osThreadId_t InternalBoardRecvTaskHandle; // ???
const osThreadAttr_t InternalBoardRecvTask_attributes = {
	.name = "InternalBoardRecv",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};
//
osThreadId_t StationResponseTaskHandle; // ???
const osThreadAttr_t StationResponseTask_attributes = {
	.name = "StationResponse",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 2025/10/29 09:49 ????
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

// 2025/11/19 18:35 ???? ????????? ??????????? ???????????
osThreadId_t CtrlBusPollAndReceiveTaskHandle; // ctrl bus send task
const osThreadAttr_t CtrlBusPollAndReceiveTask_attributes = {
	.name = "CtrlBusPollAndReceiveTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// ??·3 RS485?????????? (XR805 + XR8303)
osThreadId_t RS485DetectPollAndReceiveTaskHandle;
const osThreadAttr_t RS485DetectPollAndReceiveTask_attributes = {
	.name = "RS485DetectPollAndRecv",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

// 2025/11/17 11:24 ????
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

// 2025/11/19 18:24 ???? ????????? ??????????? ???????????
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

// 2025/11/20 18:11 ???? ????????? ??????????? ???????????
osThreadId_t CanMonitorTaskHandle; /* ???????FCP-1011??·????壻???2026-08-06 */
const osThreadAttr_t CanMonitorTask_attributes = {
	.name = "CanMonitorTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal1,
};

/* ?洢???????: ??LPUART1??洢??MCU???, ?????????????? */
osThreadId_t StorageTxTaskHandle;
const osThreadAttr_t StorageTxTask_attributes = {
	.name = "StorageTxTask",
	.stack_size = 512 * 4,  /* ?: DebugPrintf?400??? + StorageTx_SendFrame???buf[260] */
	.priority = (osPriority_t) osPriorityNormal1,
};

/* FECbus ??????????: ??USART3(PB10/PB11) RS485 ???????, GB4717 ???C */
osThreadId_t FecbusTxTaskHandle;
const osThreadAttr_t FecbusTxTask_attributes = {
	.name = "FecbusTxTask",
	.stack_size = 512 * 4,  /* ?: Fecbus_SendEvent???buf[270] + ???????? */
	.priority = (osPriority_t) osPriorityBelowNormal1,  /* ????StorageTx, ????????洢??????? */
};

/* FECbus ???????????: 1s??????/5s????/10s???, ??USART3 */
osThreadId_t FecbusPeriodicTaskHandle;
const osThreadAttr_t FecbusPeriodicTask_attributes = {
	.name = "FecbusPeriodicTask",
	.stack_size = 256 * 4,  /* ?????????С: ????????payload[7] */
	.priority = (osPriority_t) osPriorityLow,  /* ???????????????, ????????????? */
};

/* COM4???????????: UART4 RX?????DMA????, ????ONL/TF/SF???????????????ж? */
osThreadId_t TestInjectTaskHandle;
const osThreadAttr_t TestInjectTask_attributes = {
	.name = "TestInjectTask",
	.stack_size = 512 * 4,  /* ?: DebugPrintf?400??? + rx_buf[32] + ParseLine???? */
	.priority = (osPriority_t) osPriorityNormal1,
};

/* Linkage logic engine task: evaluate rules and execute actions every 100ms */
osThreadId_t LogicEngineTaskHandle;
const osThreadAttr_t LogicEngineTask_attributes = {
	.name = "LogicEngineTask",
	.stack_size = 512 * 4,  /* stack: rule table walk + control callbacks */
	.priority = (osPriority_t) osPriorityNormal,
};


extern uint8_t timeout_led_ctrl;

// ????485??????????????
QueueHandle_t xMyRs485QueueHandle;
// XR5000_UART5_EXCLUSIVE_FIX_20260730: legacy fan traffic has its own disabled transport queue.
QueueHandle_t xFanBusQueueHandle;
// ????MBUS??????????????
QueueHandle_t xMyMBusQueueHandle;
// ????MBUS2?????????????У???·2 ??????????
QueueHandle_t xMBus2QueueHandle;


//StaticSemaphore_t xMutexBuffer;
//SemaphoreHandle_t xMutex; // ???????????


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,  /* ???С: DebugPrintf?400??? + vsnprintf??????? */
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

/* ?洢????????????? */
void StorageTxTask(void *argument);

/* FECbus ?????????????? (GB4717 ???C, USART3 RS485) */
void FecbusTxTask(void *argument);

/* FECbus ???????????? (1s????/5s????/10s???) */
void FecbusPeriodicTask(void *argument);

/* COM4??????????????? */
void TestInjectTask(void *argument);

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
//	xMutex = xSemaphoreCreateMutexStatic(&xMutexBuffer); // ???????????

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of SecondTimer */
  SecondTimerHandle = osTimerNew(SecondTimerCallback, osTimerPeriodic, NULL, &SecondTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(SecondTimerHandle, 1000);  // ????1??????
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	xMyRs485QueueHandle = xQueueCreate(10, sizeof(char[8])); // ??????????洢10?????????У?????????8????char????

	xFanBusQueueHandle = xQueueCreate( 5, sizeof(char[8])); // XR5000_UART5_EXCLUSIVE_FIX_20260730: fan-only queue; UART5 is not its transport.

	xMyMBusQueueHandle = xQueueCreate( 5, sizeof(char[8])); // ??????????洢5?????????У?????????8????char????

	xMBus2QueueHandle = xQueueCreate( 5, sizeof(char[8])); // ??????????洢5?????????У?????????8????char???飨??·2 ???????

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of led_test */
  led_testHandle = osThreadNew(LED_TEST, NULL, &led_test_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

	// ???????????????
	InternalScreenTaskHandle = osThreadNew(InternalScreenRecvDealTask, NULL, &InternalScreen_attributes);
	DebugPrintTaskHandle = osThreadNew(DebugPrintTask, NULL, &DebugPrintTask_attributes);  /* async log pump */
//	// ???????????
	InterScreenUpdataTaskHandle = osThreadNew(InterScreenFresh, NULL, &InterScreenUpdataTask_attributes);

//	//BSP485 PACK POLL TASK
//	PackPollingTaskHandle = osThreadNew(BSP_RS485_01_Poll, NULL, &BSP485PollTask_attributes);
//	// ????????? ??????????
//	QueueRcevDealTaskHandle = osThreadNew(QueuePollRecvDealTask, NULL, &QueueRcevDealTask_attributes);

	// 2025/11/25 11:03 ???? ??????????
	PackPollAndReceiveTaskHandle = osThreadNew(RS485_01_PollAndRecieve, NULL, &PackPollAndReceiveTask_attributes);


	// 2025/11/17 17:22 ????MBUS
//	//
//	MBusPollingTaskHandle = osThreadNew(MBus1PollSlaveTsk, NULL, &MBusPollingTask_attributes);
//	//
//	MBusReceiveTaskHandle = osThreadNew(MBus1RecvDealTask, NULL, &MBusReceiveTask_attributes);

	// // 2025/11/19 18:24 ???? ????????? ??????????? ???????????
	MBus1PollAndReceiveTaskHandle = osThreadNew(MBus1PollSlaveAndReceiveTask, NULL, &MBus1PollAndReceiveTask_attributes);

	MBus2PollAndReceiveTaskHandle = osThreadNew(MBusControlPollSlaveAndReceiveTask, NULL, &MBus2PollAndReceiveTask_attributes);

	// ??????????????
	HostUploadTaskHandle = osThreadNew(HostUploadDataTask, NULL, &HostUploadTask_attributes);

	// ?????豸???ж?????
	LinkageOnlineTaskHandle = osThreadNew(LinkageOnlineJudgeTask, NULL, &LinkageOnlineTask_attributes);
	// ?????豸???ж?????
	KeyOnlineJudgeTaskHandle = osThreadNew(KeyStateJudgeTask, NULL, &KeyOnlineJudgeTask_attributes);
	// ?????????? (FECbus ???? 20260818: USART1 ???? FECbus ???????, ??????????)
//	StationResponseTaskHandle = osThreadNew(StationResponseTesk, NULL, &StationResponseTask_attributes);

	// ?????????ж?????
	MainStandbyPwrJudgeTaskHandle = osThreadNew(PowerOnlineJudgeTask, NULL, &MSPwrJudgeTask_attributes);

	// CtrlBusPollingTask
//	CtrlBusPollingTaskHandle = osThreadNew(CtrlBusPollingTask, NULL, &CtrlBusPollingTask_attributes);
//	//
//	CtrlBusReceiveTaskHandle = osThreadNew(CtrlBusReceiveDealTask, NULL, &CtrlBusReceiveTask_attributes);

	// 2025/11/19 18:36 ???? ????????? ??????????? ???????????
//	CtrlBusPollAndReceiveTaskHandle = osThreadNew(CtrlBusPollAndReceiveTask, NULL, &CtrlBusPollAndReceiveTask_attributes);
	// ???CtrlBusPollAndReceiveTask????: ????uart5????·3 RS485??????

	// ??·3 RS485?????????? (XR805 + XR8303)
	RS485DetectPollAndReceiveTaskHandle = osThreadNew(RS485DetectPollAndReceiveTask, NULL, &RS485DetectPollAndReceiveTask_attributes);

	// 2025/11/20 18:12
	CanMonitorTaskHandle = osThreadNew(CanMonitorTask, NULL, &CanMonitorTask_attributes); /* ???????FCP-1011??·????壻???2026-08-06 */

	// BMS?????????? 2025/11/28 11:49 ??????
	// XR5000_FECBUS_TEST: BMS???USART3(PB10/PB11)????FECbus, ???????BMS????
 // BMSRecvDealTaskHandle = osThreadNew(BMSRecvDealTask, NULL, &BMSRecvDealTask_attributes);
//	// ?????豸????????
//	LinkageCtrlTaskHandle = osThreadNew(LinkageRelayCtrlTask, NULL, &LinkageCtrlTask_attributes);

	// ?洢????????
	StorageTxTaskHandle = osThreadNew(StorageTxTask, NULL, &StorageTxTask_attributes);
	DebugPrintf("StorageTxTask create: %s\r\n", StorageTxTaskHandle ? "OK" : "FAIL");

	// FECbus ???????????: GB4717 ???C, USART3 RS485 Э??
	FecbusTxTaskHandle = osThreadNew(FecbusTxTask, NULL, &FecbusTxTask_attributes);
	DebugPrintf("FecbusTxTask create: %s\r\n", FecbusTxTaskHandle ? "OK" : "FAIL");

	// FECbus ?????????: 1s??????/5s????/10s???
	FecbusPeriodicTaskHandle = osThreadNew(FecbusPeriodicTask, NULL, &FecbusPeriodicTask_attributes);
	DebugPrintf("FecbusPeriodicTask create: %s\r\n", FecbusPeriodicTaskHandle ? "OK" : "FAIL");

	// COM4???????????: PC?·?ONL/TF/SF?????????/??????ж?
	TestInjectTaskHandle = osThreadNew(TestInjectTask, NULL, &TestInjectTask_attributes);
	DebugPrintf("TestInjectTask create: %s\r\n", TestInjectTaskHandle ? "OK" : "FAIL");

	// Linkage logic engine task: rule evaluation + linkage action execution
	LogicEngineTaskHandle = osThreadNew(LogicEngineTask, NULL, &LogicEngineTask_attributes);
	DebugPrintf("LogicEngineTask create: %s\r\n", LogicEngineTaskHandle ? "OK" : "FAIL");

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
	// ?洢????????
	BspCommonDataSaveApp(OTHER_FLASH_SAVE, OTHER_TURN_ON, LINKAGE_CLUSTER_ID, SYS_MAIN_POWER_KEY_ID);

	/* ??osDelay(2000)?ι??IWDG??????????λ */
	HAL_IWDG_Refresh(&hiwdg1);
	osDelay(2000);
	SetInternalScreenRTCTime();
	GetScreen();
	// XR5000_AHT20_CHANGE_20260727: initialize PD9/PD10 AHT20 sensor after GPIO is ready.
	AHT20_Init();
	(void)AHT20_Update();
	MBusCtrl_Init();
	// ????????????? ????????·??????
	InternalBoardRecvTaskHandle = osThreadNew(InternalScreenBoradRecvDealTask, NULL, &InternalBoardRecvTask_attributes);

	/* ?洢?????: ?????LPUART1?????·, ????????ACK=0?????·???? */
	HAL_IWDG_Refresh(&hiwdg1);
	StorageTx_Init();
	StorageEvent_LogPowerOn();  /* GB4717-2024 B.1.1.1d: 控制器开机事件(EVT_POWER_ON=120)记录 */

  /* Infinite loop */
	DebugPrintf("XR5000 Boot OK\r\n");  /* UART4????????? */
  for(;;)
  {
		freeHeap = xPortGetFreeHeapSize();
		HAL_IWDG_Refresh(&hiwdg1);
		// XR5000_AHT20_CHANGE_20260727: refresh cached home-screen temperature/humidity.
		(void)AHT20_Update();
		DebugPrintf("maintain:%d\r\n", freeHeap);  /* ????????С, ????????? */

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
	uint8_t led_state = 0; // 0:???, 1:??????, 2:??????

  /* Infinite loop */
  for(;;)
  {
		// ?????????????
		if(timeout_led_ctrl == 1) { // ??????
				led_state = 1;
				timeout_led_ctrl = 0;
				HAL_GPIO_WritePin(PMLED_G_GPIO_Port, PMLED_G_Pin, GPIO_PIN_SET); // ????????
		}
		else if(timeout_led_ctrl == 2) { // ??????
				led_state = 2;
				timeout_led_ctrl = 0;
				HAL_GPIO_WritePin(PMLED_R_GPIO_Port, PMLED_R_Pin, GPIO_PIN_SET); // ????????
		}

		// ????LED?????500ms?????
		if(osKernelGetTickCount() - last_tick >= 500) {
				last_tick = osKernelGetTickCount();

				if(led_state == 1) { // ??????
						HAL_GPIO_TogglePin(PMLED_R_GPIO_Port, PMLED_R_Pin);
				}
				else if(led_state == 2) { // ??????
						HAL_GPIO_TogglePin(PMLED_G_GPIO_Port, PMLED_G_Pin);
				}
		}

		// ?????????????LED??500ms?????
		HAL_GPIO_TogglePin(PWLED_G_GPIO_Port, PWLED_G_Pin);
		osDelay(500); // ??????500ms
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
	//	// ???UI????????
	InterScreenUpdataTaskHandle = osThreadNew(InterScreenFresh, NULL, &InterScreenUpdataTask_attributes);
}

//
void SuspendTask(uint8_t task_id)
{
	switch(task_id)
	{
		case 1:{
			vTaskSuspend(MBus1PollAndReceiveTaskHandle); // ?????????1
			break;
		}
		case 2:{

			break;
		}
		case 3:{
			vTaskSuspend(PackPollAndReceiveTaskHandle); // ????PACK 485????
			break;
		}
		case 4:{
//                 vTaskSuspend(BMSRecvDealTaskHandle); // ????BMS???? (FECBUS_TEST: BMS????δ????, handle=NULL)
			break;
		}
		case 5:{
//			vTaskSuspend(StationResponseTaskHandle); // ???? ??? ???? (FECbus ????: ???????δ???? 20260818)
			break;
		}
		case 6:{
//			vTaskSuspend(CtrlBusPollAndReceiveTaskHandle); // ??????uart5???·3???
			break;
		}
		case 7:{
			vTaskSuspend(RS485DetectPollAndReceiveTaskHandle); // ?????·3 RS485???
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
			vTaskResume(MBus1PollAndReceiveTaskHandle); // ?????????1
			break;
		}
		case 2:{
			break;
		}
		case 3:{
			vTaskResume(PackPollAndReceiveTaskHandle); // ???PACK 485????
			break;
		}
		case 4:{
//                 vTaskResume(BMSRecvDealTaskHandle); // ???BMS???? (FECBUS_TEST: BMS????δ????, handle=NULL)
			break;
		}
		case 5:{
//			vTaskResume(StationResponseTaskHandle); // ??? ??? ???? (FECbus ????: ???????δ???? 20260818)
			break;
		}
		case 6:{
//			vTaskResume(CtrlBusPollAndReceiveTaskHandle); // ??????uart5???·3???
			break;
		}
		case 7:{
			vTaskResume(RS485DetectPollAndReceiveTaskHandle); // ?????·3 RS485???
			break;
		}
		default:{
			break;
		}
	}
}


/* ?洢???????: ??LPUART1??洢??MCU???, ?????????????? */
void StorageTxTask(void *argument)
{
	/* ??????洢??·(LPUART1 PB6/PB7 + ?ж????) */
	StorageTx_Init();
	DebugPrintf("StorageTxTask started\r\n");

	for(;;)
	{
		/* ??????????????, ??????????(??3?????δ??????????) */
		StorageTx_TaskLoop();
	}
}


/* FECbus ??????????: GB4717 ???C, ??USART3(PB10/PB11) RS485 Э??? */
void FecbusTxTask(void *argument)
{
	/* ?????FECbusЭ???: ?????????/???????? + ????USART3???IT?ж?
	 * ?: Fecbus_Init ???????, ????????? */
	Fecbus_Init();
	DebugPrintf("FecbusTxTask started\r\n");

	for(;;)
	{
		/* ?????????????????, ????????(?1s???3??, ????9s)
		 * ??????????????ιIWDG??????????λ */
		Fecbus_TxTaskLoop();
	}
}


/* FECbus ???????????: 1s?????? + 5s???? + 10s???, ??USART3 */
void FecbusPeriodicTask(void *argument)
{
	/* ?????FECbusЭ???(??FecbusTxTask???ó????, ???????) */
	Fecbus_Init();
	DebugPrintf("FecbusPeriodicTask started\r\n");

	/* ?: XR5000_FECBUS_TEST_EVENT ???????豸??·(???????????):
	 * ?????·(TF/SF???->PointTypeDetectorDataDeal->FecbusReport->???->FecbusTxTask)
	 * ????COM6???: ?????5/??·1/?豸2/???31/?豸????3 ?? 3???????? */

	/* ???????????(?????ιIWDG?????λ) */
	Fecbus_PeriodicTaskLoop();
}


/* USER CODE END Application */
