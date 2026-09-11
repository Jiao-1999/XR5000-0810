#include "bsp_device_alias.h"

#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "bsp_mbus.h"
#include "bsp_mbus_control.h"
#include "bsp_rs485_detect.h"
#include "bsp_screen.h"
#include "hmi_driver.h"

#define DEVICE_ALIAS_SCREEN_ID             82U
#define DEVICE_ALIAS_CLEAR_CONFIRM_TICKS   3000U

typedef enum
{
    DEVICE_ALIAS_HMI_IDLE = 0,
    DEVICE_ALIAS_HMI_WAIT_QUERY,
    DEVICE_ALIAS_HMI_WAIT_SAVE
} DeviceAliasHmiWaitState;

static uint8_t g_alias_hmi_active;
static uint8_t g_alias_hmi_loop;
static uint8_t g_alias_hmi_address;
static uint16_t g_alias_hmi_product;
static uint8_t g_alias_hmi_device_valid;
static uint8_t g_alias_hmi_code[16];
static uint8_t g_alias_hmi_name[64];
static volatile uint8_t g_alias_hmi_query_pending;
static volatile uint8_t g_alias_hmi_save_pending;
static volatile uint8_t g_alias_hmi_clear_pending;
static DeviceAliasHmiWaitState g_alias_hmi_wait_state;
static uint8_t g_alias_hmi_clear_confirm;
static uint32_t g_alias_hmi_clear_tick;

static void DeviceAliasHmiCopyText(uint8_t *destination, uint8_t size,
                                   const uint8_t *source)
{
    uint8_t index = 0U;
    if(destination == NULL || size == 0U) return;
    if(source != NULL)
    {
        while(index + 1U < size && source[index] != 0U)
        {
            destination[index] = source[index];
            index++;
        }
    }
    destination[index] = 0U;
}

uint16_t DeviceAliasDevice_GetProductCode(uint8_t loop_id, uint8_t address)
{
    switch(loop_id)
    {
        case 1U: return MBus1_GetProductCode(address);
        case 2U: return MBusCtrl_GetProductCode(address);
        case 3U: return RS485Detect_GetProductCode(address);
        default: return 0U;
    }
}

uint8_t DeviceAliasDevice_IsAvailable(uint8_t loop_id, uint8_t address)
{
    if(address == 0U || address > DEVICE_ALIAS_MAX_ADDRESS) return 0U;
    switch(loop_id)
    {
        case 1U:
            return getPointTypeMixtureSettingOnlieState(address) != 0U &&
                   getPointTypeMixtureDetectName(address) != 0U &&
                   getPointTypeMixtureDisconnectCount(address) < MIXTURE_DEVICE_DISCONNECT_SUM &&
                   MBus1_GetProductCode(address) != 0U;
        case 2U:
            return address < MBUS_CONTROL_MAX_DEVICES &&
                   MBusCtrl_GetOnline(address) != 0U &&
                   MBusCtrl_IsIdentified(address) != 0U &&
                   MBusCtrl_IsDisconnected(address) == 0U &&
                   MBusCtrl_GetProductCode(address) != 0U;
        case 3U:
            return address < RS485_DETECT_MAX_DEVICES &&
                   RS485Detect_IsOnline(address) != 0U &&
                   RS485Detect_GetType(address) != RS485_DETECT_TYPE_UNKNOWN &&
                   RS485Detect_GetProductCode(address) != 0U;
        default:
            return 0U;
    }
}

const char *DeviceAliasDevice_GetTypeText(uint8_t loop_id, uint8_t address)
{
    if(loop_id == 1U)
    {
        uint8_t type = getPointTypeMixtureDetectName(address);
        if(type == 5U) return "烟雾探测器";
        if(type == 6U) return "温度探测器";
        return "未知设备";
    }
    if(loop_id == 2U)
    {
        switch(MBusCtrl_GetDeviceType(address))
        {
            case MBUS_CONTROL_DEV_SGBJQ: return "声光报警器";
            case MBUS_CONTROL_DEV_XR2200: return "手动报警器";
            case MBUS_CONTROL_DEV_FIRE_DISPLAY: return "火灾显示盘";
            case MBUS_CONTROL_DEV_FCM1011: return "输入输出模块";
            default: return "未知设备";
        }
    }
    if(loop_id == 3U) return "复合探测器";
    return "未知设备";
}

static uint8_t DeviceAliasHmiParseCode(const uint8_t *text, uint8_t *loop_id,
                                       uint8_t *address)
{
    uint16_t parsed_address;
    uint8_t index;
    if(text == NULL || strlen((const char *)text) != 5U) return 0U;
    for(index = 0U; index < 5U; index++)
        if(text[index] < '0' || text[index] > '9') return 0U;
    *loop_id = (uint8_t)((text[0] - '0') * 10U + (text[1] - '0'));
    parsed_address = (uint16_t)((text[2] - '0') * 100U +
                                (text[3] - '0') * 10U + (text[4] - '0'));
    if(*loop_id < 1U || *loop_id > 3U || parsed_address == 0U ||
       parsed_address > DEVICE_ALIAS_MAX_ADDRESS) return 0U;
    *address = (uint8_t)parsed_address;
    return 1U;
}

static void DeviceAliasHmiResetClearConfirmation(void)
{
    g_alias_hmi_clear_confirm = 0U;
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 24U, (uint8_t *)"[ 清除名称 ]");
}

static void DeviceAliasHmiShowUnavailable(void)
{
    g_alias_hmi_device_valid = 0U;
    g_alias_hmi_product = 0U;
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 12U, (uint8_t *)"--");
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 17U, (uint8_t *)"--");
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 13U, (uint8_t *)"--");
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 14U,
                 (uint8_t *)"设备未识别，请恢复通信后设置");
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 15U, (uint8_t *)"未命名");
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U,
                 (uint8_t *)"设备未识别，请恢复通信后设置");
}

static void DeviceAliasHmiShowCurrent(uint8_t loop_id, uint8_t address)
{
    uint8_t text[40];
    uint8_t alias[DEVICE_ALIAS_NAME_BYTES];
    DeviceAliasLookupResult lookup;
    g_alias_hmi_loop = loop_id;
    g_alias_hmi_address = address;
    if(DeviceAliasDevice_IsAvailable(loop_id, address) == 0U)
    {
        DeviceAliasHmiShowUnavailable();
        return;
    }
    g_alias_hmi_product = DeviceAliasDevice_GetProductCode(loop_id, address);
    g_alias_hmi_device_valid = 1U;
    snprintf((char *)text, sizeof(text), "回路%u", loop_id);
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 12U, text);
    snprintf((char *)text, sizeof(text), "%03u", address);
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 17U, text);
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 13U,
                 (uint8_t *)DeviceAliasDevice_GetTypeText(loop_id, address));
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 14U, (uint8_t *)"在线");
    lookup = DeviceAlias_Get(loop_id, address, g_alias_hmi_product, alias,
                             sizeof(alias));
    if(lookup == DEVICE_ALIAS_MATCH)
        SetTextValue(DEVICE_ALIAS_SCREEN_ID, 15U, alias);
    else if(lookup == DEVICE_ALIAS_PRODUCT_MISMATCH)
        SetTextValue(DEVICE_ALIAS_SCREEN_ID, 15U,
                     (uint8_t *)"设备已更换，请重新命名");
    else
        SetTextValue(DEVICE_ALIAS_SCREEN_ID, 15U, (uint8_t *)"未命名");
}

static void DeviceAliasHmiProcessQuery(void)
{
    uint8_t loop_id;
    uint8_t address;
    if(DeviceAliasHmiParseCode(g_alias_hmi_code, &loop_id, &address) == 0U)
    {
        DeviceAliasHmiShowUnavailable();
        SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U,
                     (uint8_t *)"请输入5位设备编号，例如01001");
        return;
    }
    DeviceAliasHmiShowCurrent(loop_id, address);
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U,
                 (uint8_t *)"注：最多允许13个字符");
}

static void DeviceAliasHmiStep(int8_t direction)
{
    uint8_t list[DEVICE_ALIAS_MAX_ADDRESS];
    uint8_t count = 0U;
    uint8_t index;
    uint8_t selected = 0U;
    uint8_t loop_id = g_alias_hmi_loop;
    if(loop_id < 1U || loop_id > 3U)
    {
        if(DeviceAliasHmiParseCode(g_alias_hmi_code, &loop_id, &selected) == 0U)
            loop_id = 1U;
    }
    for(index = 1U; index <= DEVICE_ALIAS_MAX_ADDRESS; index++)
        if(DeviceAliasDevice_IsAvailable(loop_id, index) != 0U) list[count++] = index;
    if(count == 0U)
    {
        DeviceAliasHmiShowUnavailable();
        return;
    }
    selected = direction > 0 ? list[0] : list[count - 1U];
    for(index = 0U; index < count; index++)
    {
        if(list[index] == g_alias_hmi_address)
        {
            selected = direction > 0 ? list[(uint8_t)((index + 1U) % count)] :
                       list[index == 0U ? count - 1U : index - 1U];
            break;
        }
    }
    snprintf((char *)g_alias_hmi_code, sizeof(g_alias_hmi_code), "%02u%03u",
             loop_id, selected);
    SetTextValue(DEVICE_ALIAS_SCREEN_ID, 4U, g_alias_hmi_code);
    DeviceAliasHmiShowCurrent(loop_id, selected);
}

static void DeviceAliasHmiShowSaveResult(DeviceAliasResult result)
{
    switch(result)
    {
        case DEVICE_ALIAS_OK:
            SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U, (uint8_t *)"保存成功");
            DeviceAliasHmiShowCurrent(g_alias_hmi_loop, g_alias_hmi_address);
            break;
        case DEVICE_ALIAS_EMPTY_NAME:
            SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U, (uint8_t *)"名称不能为空");
            break;
        case DEVICE_ALIAS_NAME_TOO_LONG:
            SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U, (uint8_t *)"名称超过13个字符");
            break;
        case DEVICE_ALIAS_INVALID_CHARACTER:
            SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U,
                         (uint8_t *)"仅允许中文、字母、数字和空格");
            break;
        case DEVICE_ALIAS_INVALID_DEVICE:
            SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U,
                         (uint8_t *)"设备未识别，请恢复通信后设置");
            break;
        default:
            SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U, (uint8_t *)"保存失败");
            break;
    }
}

void DeviceAliasHmiScreenUpdate(uint16_t screen_id)
{
    if(screen_id != DEVICE_ALIAS_SCREEN_ID)
    {
        g_alias_hmi_active = 0U;
        g_alias_hmi_wait_state = DEVICE_ALIAS_HMI_IDLE;
        g_alias_hmi_clear_confirm = 0U;
        return;
    }
    if(g_alias_hmi_active == 0U)
    {
        g_alias_hmi_active = 1U;
        g_alias_hmi_device_valid = 0U;
        g_alias_hmi_loop = 0U;
        g_alias_hmi_address = 0U;
        g_alias_hmi_product = 0U;
        g_alias_hmi_code[0] = 0U;
        g_alias_hmi_name[0] = 0U;
        clearTextValue(DEVICE_ALIAS_SCREEN_ID, 4U);
        clearTextValue(DEVICE_ALIAS_SCREEN_ID, 20U);
        DeviceAliasHmiShowUnavailable();
        DeviceAliasHmiResetClearConfirmation();
        SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U,
                     (uint8_t *)"注：最多允许13个字符");
    }
    if(g_alias_hmi_clear_confirm != 0U &&
       (uint32_t)(osKernelGetTickCount() - g_alias_hmi_clear_tick) >=
       DEVICE_ALIAS_CLEAR_CONFIRM_TICKS)
        DeviceAliasHmiResetClearConfirmation();
    if(g_alias_hmi_query_pending != 0U)
    {
        g_alias_hmi_query_pending = 0U;
        DeviceAliasHmiProcessQuery();
    }
    if(g_alias_hmi_save_pending != 0U)
    {
        DeviceAliasResult result;
        g_alias_hmi_save_pending = 0U;
        if(g_alias_hmi_device_valid == 0U ||
           DeviceAliasDevice_IsAvailable(g_alias_hmi_loop, g_alias_hmi_address) == 0U ||
           DeviceAliasDevice_GetProductCode(g_alias_hmi_loop, g_alias_hmi_address) !=
           g_alias_hmi_product)
        {
            DeviceAliasHmiShowUnavailable();
            result = DEVICE_ALIAS_INVALID_DEVICE;
        }
        else
            result = DeviceAlias_Set(g_alias_hmi_loop, g_alias_hmi_address,
                                     g_alias_hmi_product, g_alias_hmi_name);
        DeviceAliasHmiShowSaveResult(result);
    }
    if(g_alias_hmi_clear_pending != 0U)
    {
        DeviceAliasResult result;
        g_alias_hmi_clear_pending = 0U;
        if(g_alias_hmi_device_valid == 0U ||
           DeviceAliasDevice_IsAvailable(g_alias_hmi_loop, g_alias_hmi_address) == 0U ||
           DeviceAliasDevice_GetProductCode(g_alias_hmi_loop, g_alias_hmi_address) !=
           g_alias_hmi_product)
        {
            DeviceAliasHmiShowUnavailable();
            result = DEVICE_ALIAS_INVALID_DEVICE;
        }
        else
            result = DeviceAlias_Clear(g_alias_hmi_loop, g_alias_hmi_address);
        if(result == DEVICE_ALIAS_OK)
        {
            DeviceAliasHmiResetClearConfirmation();
            SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U, (uint8_t *)"清除成功");
            DeviceAliasHmiShowCurrent(g_alias_hmi_loop, g_alias_hmi_address);
        }
        else
        {
            DeviceAliasHmiResetClearConfirmation();
            if(result != DEVICE_ALIAS_INVALID_DEVICE)
                SetTextValue(DEVICE_ALIAS_SCREEN_ID, 21U, (uint8_t *)"清除失败");
        }
    }
}

void DeviceAliasHmiButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
    if(screen_id != DEVICE_ALIAS_SCREEN_ID || state != 1U) return;
    if(control_id != 29U) DeviceAliasHmiResetClearConfirmation();
    switch(control_id)
    {
        case 6U:
            g_alias_hmi_wait_state = DEVICE_ALIAS_HMI_WAIT_QUERY;
            GetControlValue(DEVICE_ALIAS_SCREEN_ID, 4U);
            break;
        case 27U:
            DeviceAliasHmiStep(-1);
            break;
        case 28U:
            DeviceAliasHmiStep(1);
            break;
        case 29U:
            if(g_alias_hmi_device_valid == 0U)
            {
                DeviceAliasHmiShowUnavailable();
                break;
            }
            if(g_alias_hmi_clear_confirm == 0U)
            {
                g_alias_hmi_clear_confirm = 1U;
                g_alias_hmi_clear_tick = osKernelGetTickCount();
                SetTextValue(DEVICE_ALIAS_SCREEN_ID, 24U, (uint8_t *)"[ 确认清除 ]");
            }
            else
                g_alias_hmi_clear_pending = 1U;
            break;
        case 30U:
            if(g_alias_hmi_device_valid == 0U)
            {
                DeviceAliasHmiShowUnavailable();
                break;
            }
            g_alias_hmi_wait_state = DEVICE_ALIAS_HMI_WAIT_SAVE;
            GetControlValue(DEVICE_ALIAS_SCREEN_ID, 20U);
            break;
        case 31U:
            SwitchCurrentScreenId(6U);
            break;
        default:
            break;
    }
}

void DeviceAliasHmiText(uint16_t screen_id, uint16_t control_id,
                        const uint8_t *text)
{
    if(screen_id != DEVICE_ALIAS_SCREEN_ID) return;
    if(control_id == 4U)
    {
        DeviceAliasHmiCopyText(g_alias_hmi_code, sizeof(g_alias_hmi_code), text);
        if(g_alias_hmi_wait_state == DEVICE_ALIAS_HMI_WAIT_QUERY)
        {
            g_alias_hmi_wait_state = DEVICE_ALIAS_HMI_IDLE;
            g_alias_hmi_query_pending = 1U;
        }
    }
    else if(control_id == 20U)
    {
        DeviceAliasHmiCopyText(g_alias_hmi_name, sizeof(g_alias_hmi_name), text);
        if(g_alias_hmi_wait_state == DEVICE_ALIAS_HMI_WAIT_SAVE)
        {
            g_alias_hmi_wait_state = DEVICE_ALIAS_HMI_IDLE;
            g_alias_hmi_save_pending = 1U;
        }
    }
}
