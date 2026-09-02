#include "bsp_mbus.h"
#include "bsp_device_registry.h"
#include "cmsis_os.h"
#include "cmd_process.h"
#include "bsp_debug.h"
#include "w25qxx.h"

// 此文件为二总线通信

//设置的点型批量上线设备数量
uint8_t setPointDevicelivenumber = 0;

uint8_t cang_polling = 1; // 轮询初始值（注：为了兼容屏幕显示，下标从1开始）

// 上线控制 是否设置为上线
uint8_t PointTypeMixtureOnlieState[MIXTURE_DEVICE_SUM] = {0};
// 掉线计数 判断是否掉线
uint8_t PointTypeMixtureDisconnectCount[MIXTURE_DEVICE_SUM] = {0};

// 接收数据 温度
uint16_t PointTypeMixtureReceiveDataTemper[MIXTURE_DEVICE_SUM] = {0};
// 接收数据 烟雾
uint16_t PointTypeMixtureReceiveDataSmoke[MIXTURE_DEVICE_SUM] = {0};

// 接收状态 温度
uint8_t PointTypeMixtureReceiveStateTemper[MIXTURE_DEVICE_SUM] = {0};
// 接收状态 烟雾
uint8_t PointTypeMixtureReceiveStateSmoke[MIXTURE_DEVICE_SUM] = {0};

//
uint8_t PointTypeMixtureDetecteName[MIXTURE_DEVICE_SUM] = {0};
// 
uint8_t PointTypeMixtureDetecteType[MIXTURE_DEVICE_SUM] = {0};
//
uint8_t PointTypeMixtureAllStateMemory[MIXTURE_DEVICE_SUM] = {0};
static void MBus1ClearIdentification(uint8_t addr);

void SavePointTypeSetOnlieState(void)
{
	W25QXX_Write(PointTypeMixtureOnlieState, MIXTURE_DEVICE_FLASH_ADDR, MIXTURE_DEVICE_FLASH_DATA_LEN);
}

void ReadPointTypeSetOnlieState(void)
{
	memset(PointTypeMixtureOnlieState, 0, sizeof(PointTypeMixtureOnlieState));
	for(uint8_t addr = 1U; addr <= MIXTURE_DEVICE_MAX_ADDR; addr++) MBus1ClearIdentification(addr);
	W25QXX_Read(PointTypeMixtureOnlieState, MIXTURE_DEVICE_FLASH_ADDR, MIXTURE_DEVICE_FLASH_DATA_LEN);
	
	for(uint16_t i = 0; i <= MIXTURE_DEVICE_MAX_ADDR; i++)
	{
		if(PointTypeMixtureOnlieState[i] > 1U)//判断是否为第一次写入
		{
			PointTypeMixtureOnlieState[i] = 0;
		}
	}
	SavePointTypeSetOnlieState();
}

void MBus1SendString(uint8_t* buf, uint8_t len)
{
	HAL_UART_Transmit(&huart7,buf,len,0xff);
}

void MBus2SendString(uint8_t* buf, uint8_t len)
{
	HAL_UART_Transmit(&huart2,buf,len,0xff);
}

// 全部下线点型仓探
void PointTypeMixtureOnlieStateDeInit(void)
{
	memset(PointTypeMixtureOnlieState, 0, sizeof(PointTypeMixtureOnlieState));//清空数组
	for(uint8_t addr = 1U; addr <= MIXTURE_DEVICE_MAX_ADDR; addr++) MBus1ClearIdentification(addr);
}

void PointTypeMixtureOnlieStateBatchSetting(uint8_t *new_online_state, uint8_t update_len)
{
	if(new_online_state == NULL || update_len == 0 || update_len > MIXTURE_DEVICE_MAX_ADDR)
	{
		return;
	}
	PointTypeMixtureOnlieStateDeInit(); // 先清空状态
	for(uint8_t i = 0; i < update_len; i++)
	{
		if(new_online_state[i] != 1)
		{
			continue;
		}
		PointTypeMixtureOnlieState[i] = new_online_state[i];
	}
}

void PointTypeMixtureOnlieStateSingleSetting(uint8_t detector_id, uint8_t online_or_offline)
{
	if(detector_id == 0 || detector_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return;
	}
	PointTypeMixtureOnlieState[detector_id] = online_or_offline ? 1 : 0;
	if(online_or_offline == 0U) MBus1ClearIdentification(detector_id);
}

/*
 * ──────────────────────────────────────────────────────────────
 * 第一类：上线设置 — 数据源 PointTypeMixtureOnlieState[]
 * 含义：用户通过UI设置的"想让该探测器上线"的意图
 *       1 = 上线, 0 = 下线
 * 注意：不等于实际在线！实际在线 = 设置上线 && DisconnectCount < MIXTURE_DEVICE_DISCONNECT_SUM
 * ──────────────────────────────────────────────────────────────
 */
uint8_t getPointTypeMixtureSettingOnlieState(uint8_t detector_id)
{
	if(detector_id == 0 || detector_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 0;
	}
	return PointTypeMixtureOnlieState[detector_id];
}
/*
 * ──────────────────────────────────────────────────────────────
 * 第二类：掉线判断 — 数据源 PointTypeMixtureDisconnectCount[]
 * 含义：一次真实请求超时后 +1，收到合法回复清零
 *       >= MIXTURE_DEVICE_DISCONNECT_SUM 即判定为掉线
 * ──────────────────────────────────────────────────────────────
 */
uint8_t getPointTypeMixtureDisconnectCount(uint8_t point_mix_id)
{
	if(point_mix_id == 0 || point_mix_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 0;
	}
	return PointTypeMixtureDisconnectCount[point_mix_id];
}

void clearPointTypeMixtureDisconnectCount(void)
{
	memset(PointTypeMixtureDisconnectCount, 0, sizeof(PointTypeMixtureDisconnectCount));//清空数组
}
/*
 * ──────────────────────────────────────────────────────────────
 * 第三类：接收数据 — 数据源 PointTypeMixtureReceiveDataTemper/Smoke[]
 * 含义：从探测器回复中解析出的温度值或烟雾值
 *       由 MBus1ReceiveSlaveDataDeal() 在收到合法回复时更新
 * ──────────────────────────────────────────────────────────────
 */
uint8_t getPointTypeMixtureReceiveData(ePointTypeDataOrder detect_data_type, uint8_t detect_id)
{
	/* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: legacy 8-bit compatibility wrapper. */
	return (uint8_t)getPointTypeMixtureReceiveData16(detect_data_type, detect_id);
}
/*
 *  接收状态 — 数据源 PointTypeMixtureReceiveStateTemper/Smoke[]
 *  含义：探测器回复中的报警状态位，!=0 表示正在报警
 */
uint8_t getPointTypeMixtureReceiveState(ePointTypeDataOrder detect_data_type, uint8_t detect_id)
{
	uint8_t *pData = NULL;
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 255;
	}
	
	switch(detect_data_type)
	{
		case PointTypeData_Temper:
			pData = PointTypeMixtureReceiveStateTemper;
			break;
		case PointTypeData_Smoke:
			pData = PointTypeMixtureReceiveStateSmoke;
			break;
		default:
			return 255;
	}
	
	return pData[detect_id];
}

/*
 * ──────────────────────────────────────────────────────────────
 * 第四类：探测器属性
 *  DetectName — 型号名（如 8001_AI=1, 800C=2）
 *  DetectType — 监测类型（Smoke=1烟感, Temper=2温感）
 * ──────────────────────────────────────────────────────────────
 */
uint8_t getPointTypeMixtureDetectName(uint8_t detect_id)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 255;
	}
	return PointTypeMixtureDetecteName[detect_id];
}

// 获取传感器启用状态
uint8_t getPointTypeMixtureDetectType(uint8_t detect_id)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 255;
	}
	return PointTypeMixtureDetecteType[detect_id];
}

uint8_t getPointTypeMixtureDetectOnlineState(uint8_t detect_id)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 255;
	}
	return PointTypeMixtureOnlieState[detect_id];
}

void clearPointTypeMixtureDetectAllStateMemory(void)
{
	memset(PointTypeMixtureAllStateMemory, 0, sizeof(PointTypeMixtureAllStateMemory));//清空数组
}

uint8_t getPointTypeMixtureDetectDisconnectMemory(uint8_t detect_id)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 255;
	}
	return (PointTypeMixtureAllStateMemory[detect_id] >> PointTypeDetectorDisconnectBit)&0x01;
}

void setPointTypeMixtureDetectDisconnectMemory(uint8_t detect_id, uint8_t state)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return;
	}
	if(state == 1)
	{
		PointTypeMixtureAllStateMemory[detect_id] |= (1 << PointTypeDetectorDisconnectBit);
	}
	else
	{
		PointTypeMixtureAllStateMemory[detect_id] &= ~(1 << PointTypeDetectorDisconnectBit);
	}
}

uint8_t getPointTypeMixtureDetectTempertureMemory(uint8_t detect_id)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 255;
	}
	return (PointTypeMixtureAllStateMemory[detect_id] >> PointTypeDetectorTempertureBit)&0x01;
}

void setPointTypeMixtureDetectTempertureMemory(uint8_t detect_id, uint8_t state)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return;
	}
	if(state == 1)
	{
		PointTypeMixtureAllStateMemory[detect_id] |= (1 << PointTypeDetectorTempertureBit);
	}
	else
	{
		PointTypeMixtureAllStateMemory[detect_id] &= ~(1 << PointTypeDetectorTempertureBit);
	}
}

uint8_t getPointTypeMixtureDetectSmokeMemory(uint8_t detect_id)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return 255;
	}
	return (PointTypeMixtureAllStateMemory[detect_id] >> PointTypeDetectorSmokeBit)&0x01;
}

void setPointTypeMixtureDetectSmokeMemory(uint8_t detect_id, uint8_t state)
{
	if(detect_id == 0 || detect_id > MIXTURE_DEVICE_MAX_ADDR)
	{
		return;
	}
	if(state == 1)
	{
		PointTypeMixtureAllStateMemory[detect_id] |= (1 << PointTypeDetectorSmokeBit);
	}
	else
	{
		PointTypeMixtureAllStateMemory[detect_id] &= ~(1 << PointTypeDetectorSmokeBit);
	}
}

static uint8_t MBus1GetStateClassByRaw(uint8_t type, uint8_t state)
{
    if(type == 6U)
    {
        if(state == 1U) return 1U;
        if(state == 2U) return 2U;
        if(state == 3U) return 3U;
    }
    else if(type == 5U)
    {
        if(state == 1U) return 1U;
        if(state == 2U) return 2U;
        if(state == 8U || state == 9U) return 3U;
    }
    return 0U;
}

uint16_t getPointTypeMixtureReceiveData16(ePointTypeDataOrder detect_data_type, uint8_t detect_id)
{
    if(detect_id == 0U || detect_id > MIXTURE_DEVICE_MAX_ADDR) return 0U;
    if(detect_data_type == PointTypeData_Temper) return PointTypeMixtureReceiveDataTemper[detect_id];
    if(detect_data_type == PointTypeData_Smoke) return PointTypeMixtureReceiveDataSmoke[detect_id];
    return 0U;
}

uint8_t getPointTypeMixtureStateClass(uint8_t detect_id)
{
    uint8_t type;
    if(detect_id == 0U || detect_id > MIXTURE_DEVICE_MAX_ADDR) return 0U;
    type = PointTypeMixtureDetecteName[detect_id];
    if(type == 6U) return MBus1GetStateClassByRaw(type, PointTypeMixtureReceiveStateTemper[detect_id]);
    if(type == 5U) return MBus1GetStateClassByRaw(type, PointTypeMixtureReceiveStateSmoke[detect_id]);
    return 0U;
}

static void MBus1BuildReadCommand(uint8_t *buf, uint8_t addr, uint16_t start, uint8_t count)
{
    uint16_t crc16;
    buf[0] = addr; buf[1] = 0x04; buf[2] = (uint8_t)(start >> 8); buf[3] = (uint8_t)start; buf[4] = 0x00; buf[5] = count;
    crc16 = CalcCrc16(buf, 6); buf[6] = crc16 & 0xFF; buf[7] = crc16 >> 8;
}

static uint8_t g_mbus1_type_confirmed[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint8_t g_mbus1_identify_fail_count[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint32_t g_mbus1_last_identify_tick[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint16_t g_mbus1_national_code[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint16_t g_mbus1_product_code[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint8_t g_mbus1_identify_stage[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint16_t g_mbus1_identify_candidate[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint8_t g_mbus1_identify_confirm_count[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
static uint32_t g_mbus1_last_offline_probe_tick[MIXTURE_DEVICE_MAX_ADDR + 1U] = {0};
#define MBUS1_STAGE_NATIONAL 0U
#define MBUS1_STAGE_PRODUCT  1U
#define MBUS1_STAGE_COMPLETE 2U
#define MBUS1_IDENTIFY_FAIL_THRESHOLD 3U
#define MBUS1_IDENTIFY_RETRY_MS 1000U
#define MBUS1_IDENTIFY_RETRY_GAP_MS 150U
#define MBUS1_OFFLINE_PROBE_INTERVAL_MS 1000U

uint16_t MBus1_GetNationalTypeCode(uint8_t addr)
{
    if(addr == 0U || addr > MIXTURE_DEVICE_MAX_ADDR) return 0U;
    return g_mbus1_national_code[addr];
}

uint16_t MBus1_GetProductCode(uint8_t addr)
{
    if(addr == 0U || addr > MIXTURE_DEVICE_MAX_ADDR) return 0U;
    return g_mbus1_product_code[addr];
}

static void MBus1ClearIdentification(uint8_t addr)
{
    if(addr == 0U || addr > MIXTURE_DEVICE_MAX_ADDR) return;
    g_mbus1_type_confirmed[addr] = 0U;
    g_mbus1_identify_fail_count[addr] = 0U;
    g_mbus1_last_identify_tick[addr] = 0U;
    g_mbus1_national_code[addr] = 0U;
    g_mbus1_product_code[addr] = 0U;
    g_mbus1_identify_stage[addr] = MBUS1_STAGE_NATIONAL;
    g_mbus1_identify_candidate[addr] = 0U;
    g_mbus1_identify_confirm_count[addr] = 0U;
    g_mbus1_last_offline_probe_tick[addr] = 0U;
    PointTypeMixtureDetecteName[addr] = 0U;
    PointTypeMixtureDetecteType[addr] = 0U;
    PointTypeMixtureDisconnectCount[addr] = 0U;
    DeviceRegistry_SetProductUnknown(DEVICE_REGISTRY_LOOP1, addr, 0U);
}

/* 获取探测器国标设备类型码(供联动逻辑显示用), 地址无效返回0 */
uint16_t getPointTypeMixtureNationalCode(uint8_t detector_id)
{
    if(detector_id == 0U || detector_id > MIXTURE_DEVICE_MAX_ADDR) return 0U;
    return g_mbus1_national_code[detector_id];
}
static uint8_t g_mbus1_transaction_pending = 0U;
static uint8_t g_mbus1_transaction_addr = 0U;
static uint8_t g_mbus1_transaction_identify_stage = MBUS1_STAGE_COMPLETE;
static uint32_t g_mbus1_transaction_tick = 0U;
static uint8_t g_mbus1_poll_addr = 0U;
static uint8_t g_mbus1_retry_addr = 0U;
static volatile uint8_t g_mbus1_bus_locked = 0U;

static void MBus1FinishTransaction(uint8_t addr)
{
    PointTypeMixtureDisconnectCount[addr] = 0U;
    g_mbus1_transaction_pending = 0U; g_mbus1_transaction_addr = 0U;
    g_mbus1_transaction_identify_stage = MBUS1_STAGE_COMPLETE; g_mbus1_transaction_tick = 0U; g_mbus1_retry_addr = 0U;
}

static void MBus1MarkIdentifyFailure(uint8_t addr, DeviceIdentifyError error)
{
    if(addr == 0U || addr > MIXTURE_DEVICE_MAX_ADDR) return;
    g_mbus1_identify_candidate[addr] = 0U;
    g_mbus1_identify_confirm_count[addr] = 0U;
    if(error == DEVICE_IDENTIFY_NATIONAL_UNKNOWN || error == DEVICE_IDENTIFY_CODE_MISMATCH)
        g_mbus1_identify_stage[addr] = MBUS1_STAGE_NATIONAL;
    if(g_mbus1_identify_fail_count[addr] < MBUS1_IDENTIFY_FAIL_THRESHOLD) g_mbus1_identify_fail_count[addr]++;
    g_mbus1_last_identify_tick[addr] = osKernelGetTickCount();
    if(g_mbus1_identify_fail_count[addr] >= MBUS1_IDENTIFY_FAIL_THRESHOLD)
        DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP1, addr, error);
}

static void MBus1MarkTimeout(void)
{
    uint8_t addr;
    uint8_t identify_stage;
    uint32_t timeout_ms;
    if(g_mbus1_bus_locked != 0U || g_mbus1_transaction_pending == 0U) return;
    timeout_ms = g_mbus1_transaction_identify_stage == MBUS1_STAGE_COMPLETE ?
                 MIXTURE_DEVICE_RESPONSE_TIMEOUT_MS : MIXTURE_DEVICE_IDENTIFY_RESPONSE_TIMEOUT_MS;
    if((osKernelGetTickCount() - g_mbus1_transaction_tick) < timeout_ms) return;
    addr = g_mbus1_transaction_addr;
    identify_stage = g_mbus1_transaction_identify_stage;
    g_mbus1_transaction_pending = 0U; g_mbus1_transaction_addr = 0U;
    g_mbus1_transaction_identify_stage = MBUS1_STAGE_COMPLETE; g_mbus1_transaction_tick = 0U;
    if(addr > 0U && addr <= MIXTURE_DEVICE_MAX_ADDR && PointTypeMixtureOnlieState[addr] != 0U)
    {
        if(identify_stage != MBUS1_STAGE_COMPLETE)
            MBus1MarkIdentifyFailure(addr, identify_stage == MBUS1_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE);
        else
        {
            if(PointTypeMixtureDisconnectCount[addr] < MIXTURE_DEVICE_DISCONNECT_SUM) PointTypeMixtureDisconnectCount[addr]++;
            if(PointTypeMixtureDisconnectCount[addr] >= MIXTURE_DEVICE_DISCONNECT_SUM)
                g_mbus1_last_offline_probe_tick[addr] = osKernelGetTickCount();
        }
    }
}

static uint8_t MBus1FindNextOnlineAddress(void)
{
    uint8_t attempt;
    uint32_t now = osKernelGetTickCount();
    for(attempt = 0U; attempt < MIXTURE_DEVICE_MAX_ADDR; attempt++)
    {
        g_mbus1_poll_addr++;
        if(g_mbus1_poll_addr == 0U || g_mbus1_poll_addr > MIXTURE_DEVICE_MAX_ADDR) g_mbus1_poll_addr = 1U;
        if(PointTypeMixtureOnlieState[g_mbus1_poll_addr] != 0U)
        {
            if(g_mbus1_type_confirmed[g_mbus1_poll_addr] == 0U)
            {
                uint32_t retry_gap = DeviceRegistry_IsProductUnknown(DEVICE_REGISTRY_LOOP1, g_mbus1_poll_addr) != 0U ?
                                     MBUS1_IDENTIFY_RETRY_MS : MBUS1_IDENTIFY_RETRY_GAP_MS;
                if(g_mbus1_last_identify_tick[g_mbus1_poll_addr] == 0U ||
                   (now - g_mbus1_last_identify_tick[g_mbus1_poll_addr]) >= retry_gap)
                    return g_mbus1_poll_addr;
            }
            else if(PointTypeMixtureDisconnectCount[g_mbus1_poll_addr] < MIXTURE_DEVICE_DISCONNECT_SUM ||
                    (now - g_mbus1_last_offline_probe_tick[g_mbus1_poll_addr]) >= MBUS1_OFFLINE_PROBE_INTERVAL_MS)
            {
                if(PointTypeMixtureDisconnectCount[g_mbus1_poll_addr] >= MIXTURE_DEVICE_DISCONNECT_SUM)
                    g_mbus1_last_offline_probe_tick[g_mbus1_poll_addr] = now;
                return g_mbus1_poll_addr;
            }
        }
    }
    return 0U;
}

static void MBus1StartTransaction(uint8_t addr)
{
    uint8_t modbus_buff[8];
    uint8_t identify_stage;
    uint32_t transaction_tick;
    if(addr == 0U || addr > MIXTURE_DEVICE_MAX_ADDR || PointTypeMixtureOnlieState[addr] == 0U) return;
    identify_stage = g_mbus1_type_confirmed[addr] == 0U ? g_mbus1_identify_stage[addr] : MBUS1_STAGE_COMPLETE;
    MBus1BuildReadCommand(modbus_buff, addr, identify_stage == MBUS1_STAGE_NATIONAL ? 0x000DU : identify_stage == MBUS1_STAGE_PRODUCT ? 0x000EU : 0U, identify_stage == MBUS1_STAGE_COMPLETE ? 4U : 1U);
    transaction_tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    if(g_mbus1_bus_locked != 0U || g_mbus1_transaction_pending != 0U)
    {
        taskEXIT_CRITICAL();
        return;
    }
    g_mbus1_transaction_pending = 1U; g_mbus1_transaction_addr = addr;
    g_mbus1_transaction_identify_stage = identify_stage; g_mbus1_transaction_tick = transaction_tick;
    taskEXIT_CRITICAL();

    uartbuff[MBUS1SITE].recepetion_flag = 0U; uartbuff[MBUS1SITE].recepetion_len = 0U;
    if(HAL_UART_Transmit(&huart7, modbus_buff, sizeof(modbus_buff), 30U) != HAL_OK)
    {
        taskENTER_CRITICAL();
        if(g_mbus1_transaction_addr == addr)
        {
            g_mbus1_transaction_pending = 0U; g_mbus1_transaction_addr = 0U;
            g_mbus1_transaction_identify_stage = MBUS1_STAGE_COMPLETE; g_mbus1_transaction_tick = 0U;
        }
        taskEXIT_CRITICAL();
    }
}
void MixtureDevicePollingManage(void)
{
    uint8_t addr;
    if(g_mbus1_transaction_pending != 0U) return;
    addr = g_mbus1_retry_addr;
    if(addr != 0U) g_mbus1_retry_addr = 0U; else addr = MBus1FindNextOnlineAddress();
    MBus1StartTransaction(addr);
}

void MBus1ReceiveSlaveDataDeal(void)
{
    uint8_t *buf = uartbuff[MBUS1SITE].recepetion_buff;
    uint16_t len = uartbuff[MBUS1SITE].recepetion_len;
    uint16_t crc16;
    uint8_t addr, byte_count, expected_count;
    if(uartbuff[MBUS1SITE].recepetion_flag != 1U) return;
    uartbuff[MBUS1SITE].recepetion_flag = 0U;
    if(g_mbus1_transaction_pending == 0U || len < 5U) return;
    crc16 = (buf[len - 1U] << 8) | buf[len - 2U];
    if(CalcCrc16(buf, len - 2U) != crc16) return;
    addr = buf[0];
    if(addr != g_mbus1_transaction_addr || addr == 0U || addr > MIXTURE_DEVICE_MAX_ADDR || buf[1] != 0x04U) return;
    byte_count = buf[2]; expected_count = g_mbus1_transaction_identify_stage == MBUS1_STAGE_COMPLETE ? 8U : 2U;
    if(byte_count != expected_count || len != (uint16_t)(byte_count + 5U)) return;
    if(g_mbus1_transaction_identify_stage == MBUS1_STAGE_NATIONAL)
    {
        uint16_t national_code = ((uint16_t)buf[3] << 8) | buf[4];
        if(g_mbus1_identify_candidate[addr] != national_code)
        {
            g_mbus1_identify_candidate[addr] = national_code;
            g_mbus1_identify_confirm_count[addr] = 1U;
        }
        else if(g_mbus1_identify_confirm_count[addr] < 2U)
        {
            g_mbus1_identify_confirm_count[addr]++;
        }
        if(g_mbus1_identify_confirm_count[addr] >= 2U)
        {
            g_mbus1_national_code[addr] = national_code;
            g_mbus1_identify_stage[addr] = MBUS1_STAGE_PRODUCT;
            g_mbus1_identify_candidate[addr] = 0U;
            g_mbus1_identify_confirm_count[addr] = 0U;
            g_mbus1_identify_fail_count[addr] = 0U;
        }
    }
    else if(g_mbus1_transaction_identify_stage == MBUS1_STAGE_PRODUCT)
    {
        uint16_t product_type = ((uint16_t)buf[3] << 8) | buf[4];
        if(g_mbus1_identify_candidate[addr] != product_type)
        {
            g_mbus1_identify_candidate[addr] = product_type;
            g_mbus1_identify_confirm_count[addr] = 1U;
        }
        else if(g_mbus1_identify_confirm_count[addr] < 2U)
        {
            g_mbus1_identify_confirm_count[addr]++;
        }
        if(g_mbus1_identify_confirm_count[addr] >= 2U)
        {
            if(DeviceRegistry_IsSupportedOnLoop(product_type, DEVICE_REGISTRY_LOOP1) == 0U)
                MBus1MarkIdentifyFailure(addr, DEVICE_IDENTIFY_PRODUCT_UNKNOWN);
            else if(DeviceRegistry_IsNationalProductMatch(g_mbus1_national_code[addr], product_type) == 0U)
                MBus1MarkIdentifyFailure(addr, DeviceRegistry_IsNationalTypeKnown(g_mbus1_national_code[addr]) != 0U ? DEVICE_IDENTIFY_CODE_MISMATCH : DEVICE_IDENTIFY_NATIONAL_UNKNOWN);
            else
            {
                g_mbus1_product_code[addr] = product_type;
                PointTypeMixtureDetecteName[addr] = product_type;
                PointTypeMixtureDetecteType[addr] = (product_type == DEVICE_PRODUCT_XR8002_TEMP) ? 0x20U : 0x01U;
                g_mbus1_type_confirmed[addr] = 1U;
                g_mbus1_identify_stage[addr] = MBUS1_STAGE_COMPLETE;
                g_mbus1_identify_candidate[addr] = 0U;
                g_mbus1_identify_confirm_count[addr] = 0U;
                g_mbus1_identify_fail_count[addr] = 0U;
                DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP1, addr, DEVICE_IDENTIFY_OK);
            }
        }
    }
    else
    {
        PointTypeMixtureReceiveDataTemper[addr] = ((uint16_t)buf[3] << 8) | buf[4];
        if(buf[5] != 0U || buf[9] != 0U) return;
        PointTypeMixtureReceiveStateTemper[addr] = buf[6];
        PointTypeMixtureReceiveDataSmoke[addr] = ((uint16_t)buf[7] << 8) | buf[8];
        PointTypeMixtureReceiveStateSmoke[addr] = buf[10];
    }
    MBus1FinishTransaction(addr);
}

void MBus1ResetAllDevices(void)
{
    uint8_t reset_command[8] = {0xFFU, 0x05U, 0x00U, 0x03U, 0xFFU, 0x00U, 0U, 0U};
    uint16_t crc16 = CalcCrc16(reset_command, 6U);
    uint32_t wait_tick = osKernelGetTickCount();
    reset_command[6] = (uint8_t)(crc16 & 0xFFU);
    reset_command[7] = (uint8_t)(crc16 >> 8);

    /* XR5000_LOOP1_100_DEVICE_TRANSACTION_20260730: serialize reset with the UART7 poll transaction. */
    taskENTER_CRITICAL();
    g_mbus1_bus_locked = 1U;
    taskEXIT_CRITICAL();
    while(g_mbus1_transaction_pending != 0U &&
          (osKernelGetTickCount() - wait_tick) <= (MIXTURE_DEVICE_RESPONSE_TIMEOUT_MS + 40U))
    {
        osDelay(1U);
    }

    taskENTER_CRITICAL();
    g_mbus1_transaction_pending = 0U; g_mbus1_transaction_addr = 0U;
    g_mbus1_transaction_identify_stage = MBUS1_STAGE_COMPLETE; g_mbus1_transaction_tick = 0U; g_mbus1_retry_addr = 0U;
    taskEXIT_CRITICAL();
    uartbuff[MBUS1SITE].recepetion_flag = 0U; uartbuff[MBUS1SITE].recepetion_len = 0U;
    HAL_UART_Transmit(&huart7, reset_command, sizeof(reset_command), 30U);
    HAL_UART_Transmit(&huart7, reset_command, sizeof(reset_command), 30U);

    taskENTER_CRITICAL();
    g_mbus1_bus_locked = 0U;
    taskEXIT_CRITICAL();
}
void MBus1PollSlaveAndReceiveTask(void* parameter)
{
    (void)parameter;
    for(;;)
    {
        MBus1ReceiveSlaveDataDeal();
        MBus1MarkTimeout();
        if(g_mbus1_bus_locked == 0U && g_mbus1_transaction_pending == 0U) MixtureDevicePollingManage();
        osDelay(MIXTURE_DEVICE_TASK_INTERVAL_MS);
    }
}
void MBus2RecvDealTask(void* parameter)
{
	if(uartbuff[MBUS2SITE].recepetion_flag == 1)
	{
		uartbuff[MBUS2SITE].recepetion_flag = 0;
	}
}



