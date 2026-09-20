#include "bsp_ig3302_policy.h"

#include "bsp_device_disable.h"
#include "bsp_mbus.h"
#include "bsp_mbus_control.h"
#include "bsp_rs485_detect.h"

static uint8_t IG3302Policy_HasFireAlarm(void)
{
    uint8_t address;

    /* 与现有声光联动保持一致：回路1正式上线、未屏蔽且状态类别为2即为火警。 */
    for(address = 1U; address <= MIXTURE_DEVICE_MAX_ADDR; address++)
    {
        if(getPointTypeMixtureSettingOnlieState(address) != 0U &&
           DeviceDisableIsLoopAddressSet(1U, address) == 0U &&
           getPointTypeMixtureStateClass(address) == 2U)
            return 1U;
    }

    /* 回路3温度或烟雾状态为1即为火警；保留的火警状态按安全优先继续停风机。 */
    for(address = 1U; address < RS485_DETECT_MAX_DEVICES; address++)
    {
        if(RS485Detect_GetOnline(address) != 0U &&
           DeviceDisableIsLoopAddressSet(3U, address) == 0U &&
           (RS485Detect_GetSensorState(address, RS485_SENSOR_TEMPERATURE) == 1U ||
            RS485Detect_GetSensorState(address, RS485_SENSOR_SMOKE) == 1U))
            return 1U;
    }
    return 0U;
}

static uint8_t IG3302Policy_HasGasAlarm(void)
{
    uint8_t address;

    for(address = 1U; address < RS485_DETECT_MAX_DEVICES; address++)
    {
        if(RS485Detect_IsOnline(address) == 0U ||
           RS485Detect_HasSensorData(address) == 0U ||
           DeviceDisableIsLoopAddressSet(3U, address) != 0U)
            continue;

        if(RS485Detect_GetSensorState(address, RS485_SENSOR_CO) == 2U ||
           RS485Detect_GetSensorState(address, RS485_SENSOR_CO) == 3U ||
           RS485Detect_GetSensorState(address, RS485_SENSOR_H2) == 2U ||
           RS485Detect_GetSensorState(address, RS485_SENSOR_H2) == 3U)
            return 1U;
    }
    return 0U;
}

uint8_t IG3302Policy_GetAllFanTarget(void)
{
    /* 同时存在火警和CO/H2报警时，火警停风机具有绝对优先级。 */
    if(IG3302Policy_HasFireAlarm() != 0U) return 0U;
    return (IG3302Policy_HasGasAlarm() != 0U ||
            MBusCtrl_HasActiveFanButton() != 0U) ? 1U : 0U;
}
