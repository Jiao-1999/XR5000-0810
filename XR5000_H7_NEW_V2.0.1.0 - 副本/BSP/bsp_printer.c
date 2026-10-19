#include "bsp_printer.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "system.h"

#include <string.h>

#define BSP_PRINTER_ADDRESS             1U
#define BSP_PRINTER_FUNCTION            0x06U
#define BSP_PRINTER_QUEUE_CAPACITY      512U
#define BSP_PRINTER_PAYLOAD_LIMIT       99U
#define BSP_PRINTER_SEPARATOR_LENGTH    32U
#define BSP_PRINTER_HOST_SOURCE_WIDTH    8U
#define BSP_PRINTER_HOST_COMPONENT_WIDTH 12U
#define BSP_PRINTER_HOST_STATE_WIDTH     10U

typedef enum
{
    BSP_PRINTER_RECORD_LOOP = 0,
    BSP_PRINTER_RECORD_HOST_FAULT
}BspPrinterRecordType;

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t record_type;
    uint8_t loop;
    uint8_t address;
    uint8_t device_type;
    uint8_t event_type;
    uint8_t host_fault_type;
}BspPrinterQueueItem;

static QueueHandle_t g_printer_queue;
static StaticQueue_t g_printer_queue_control;
static uint8_t g_printer_queue_storage[BSP_PRINTER_QUEUE_CAPACITY * sizeof(BspPrinterQueueItem)];
static uint16_t g_printer_dropped_count;

/* GB2312 device labels. Every label is no longer than five Chinese characters. */
static const uint8_t g_device_unknown[]        = "\xCE\xB4\xD6\xAA\xC9\xE8\xB1\xB8";
static const uint8_t g_device_point_temp[]     = "\xB5\xE3\xD0\xCD\xCE\xC2\xB8\xD0";
static const uint8_t g_device_point_smoke[]    = "\xB5\xE3\xD0\xCD\xD1\xCC\xB8\xD0";
static const uint8_t g_device_compound[]       = "\xB8\xB4\xBA\xCF\xCC\xBD\xB2\xE2\xC6\xF7";
static const uint8_t g_device_isolator[]       = "\xB6\xCC\xC2\xB7\xB8\xF4\xC0\xEB\xC6\xF7";
static const uint8_t g_device_sound_light[]    = "\xC9\xF9\xB9\xE2\xB1\xA8\xBE\xAF\xC6\xF7";
static const uint8_t g_device_manual_call[]    = "\xCA\xD6\xB6\xAF\xB1\xA8\xBE\xAF\xC6\xF7";
static const uint8_t g_device_fire_display[]   = "\xBB\xF0\xD4\xD6\xCF\xD4\xCA\xBE\xC5\xCC";
static const uint8_t g_device_gas_module[]     = "\xC6\xF8\xCC\xE5\xC4\xA3\xBF\xE9";
static const uint8_t g_device_input_output[]   = "\xCA\xE4\xC8\xEB\xCA\xE4\xB3\xF6";
static const uint8_t g_device_dual_output[]    = "\xCB\xAB\xC2\xB7\xCA\xE4\xB3\xF6";
static const uint8_t g_device_fan_button[]     = "\xB7\xE7\xBB\xFA\xB0\xB4\xC5\xA5";
static const uint8_t g_device_spray_button[]   = "\xC5\xE7\xC8\xF7\xB0\xB4\xC5\xA5";
static const uint8_t g_device_fan_controller[] = "\xB7\xE7\xBB\xFA\xBF\xD8\xD6\xC6\xC6\xF7";

/* GB2312 event labels. Mixed ASCII labels still fit the reserved eight columns. */
static const uint8_t g_event_temp_alarm[]      = "\xCE\xC2\xB6\xC8\xB1\xA8\xBE\xAF";
static const uint8_t g_event_smoke_alarm[]     = "\xD1\xCC\xCE\xED\xB1\xA8\xBE\xAF";
static const uint8_t g_event_manual_alarm[]    = "\xCA\xD6\xB6\xAF\xB1\xA8\xBE\xAF";
static const uint8_t g_event_co_low[]          = "CO\xB5\xCD\xB1\xA8";
static const uint8_t g_event_co_high[]         = "CO\xB8\xDF\xB1\xA8";
static const uint8_t g_event_h2_low[]          = "H2\xB5\xCD\xB1\xA8";
static const uint8_t g_event_h2_high[]         = "H2\xB8\xDF\xB1\xA8";
static const uint8_t g_event_comm_fault[]      = "\xCD\xA8\xD0\xC5\xB9\xCA\xD5\xCF";
static const uint8_t g_event_temp_fault[]      = "\xCE\xC2\xB8\xD0\xB9\xCA\xD5\xCF";
static const uint8_t g_event_smoke_fault[]     = "\xD1\xCC\xB8\xD0\xB9\xCA\xD5\xCF";
static const uint8_t g_event_input_fault[]     = "\xCA\xE4\xC8\xEB\xB9\xCA\xD5\xCF";
static const uint8_t g_event_output_fault[]    = "\xCA\xE4\xB3\xF6\xB9\xCA\xD5\xCF";
static const uint8_t g_event_identify_fault[]  = "\xCA\xB6\xB1\xF0\xB9\xCA\xD5\xCF";
static const uint8_t g_event_branch1_short[]   = "1\xC2\xB7\xB6\xCC\xC2\xB7";
static const uint8_t g_event_branch2_short[]   = "2\xC2\xB7\xB6\xCC\xC2\xB7";
static const uint8_t g_event_branch3_short[]   = "3\xC2\xB7\xB6\xCC\xC2\xB7";
static const uint8_t g_event_fan1_fault[]      = "\xB7\xE7" "1" "\xB9\xCA\xD5\xCF";
static const uint8_t g_event_fan2_fault[]      = "\xB7\xE7" "2" "\xB9\xCA\xD5\xCF";
static const uint8_t g_event_device_fault[]    = "\xC9\xE8\xB1\xB8\xB9\xCA\xD5\xCF";

/* GB2312 host-fault fields: source | component | state. */
static const uint8_t g_host_source[]           = "\xD6\xF7\xBB\xFA"; /* 主机 */
static const uint8_t g_host_main_power[]       = "\xD6\xF7\xB5\xE7\xD4\xB4"; /* 主电源 */
static const uint8_t g_host_back_power[]       = "\xB1\xB8\xD3\xC3\xB5\xE7\xB3\xD8"; /* 备用电池 */
static const uint8_t g_host_storage[]          = "\xCF\xB5\xCD\xB3\xB4\xE6\xB4\xA2"; /* 系统存储 */
static const uint8_t g_host_reset[]            = "\xCF\xB5\xCD\xB3\xB8\xB4\xCE\xBB"; /* 系统复位 */
static const uint8_t g_host_power_off[]        = "\xB6\xCF\xB5\xE7\xB9\xCA\xD5\xCF"; /* 断电故障 */
static const uint8_t g_host_open_fault[]       = "\xBF\xAA\xC2\xB7\xB9\xCA\xD5\xCF"; /* 开路故障 */
static const uint8_t g_host_short_fault[]      = "\xB6\xCC\xC2\xB7\xB9\xCA\xD5\xCF"; /* 短路故障 */
static const uint8_t g_host_storage_fault[]    = "\xB4\xE6\xB4\xA2\xB9\xCA\xD5\xCF"; /* 存储故障 */
static const uint8_t g_host_send_error[]       = "\xB7\xA2\xCB\xCD\xD2\xEC\xB3\xA3"; /* 发送异常 */

static const uint8_t *PrinterDeviceLabel(uint8_t type)
{
    switch(type)
    {
        case BSP_PRINTER_DEVICE_POINT_TEMP:     return g_device_point_temp;
        case BSP_PRINTER_DEVICE_POINT_SMOKE:    return g_device_point_smoke;
        case BSP_PRINTER_DEVICE_COMPOUND:       return g_device_compound;
        case BSP_PRINTER_DEVICE_ISOLATOR:       return g_device_isolator;
        case BSP_PRINTER_DEVICE_SOUND_LIGHT:    return g_device_sound_light;
        case BSP_PRINTER_DEVICE_MANUAL_CALL:    return g_device_manual_call;
        case BSP_PRINTER_DEVICE_FIRE_DISPLAY:   return g_device_fire_display;
        case BSP_PRINTER_DEVICE_GAS_MODULE:     return g_device_gas_module;
        case BSP_PRINTER_DEVICE_INPUT_OUTPUT:   return g_device_input_output;
        case BSP_PRINTER_DEVICE_DUAL_OUTPUT:    return g_device_dual_output;
        case BSP_PRINTER_DEVICE_FAN_BUTTON:     return g_device_fan_button;
        case BSP_PRINTER_DEVICE_SPRAY_BUTTON:   return g_device_spray_button;
        case BSP_PRINTER_DEVICE_FAN_CONTROLLER: return g_device_fan_controller;
        default:                                return g_device_unknown;
    }
}

static const uint8_t *PrinterEventLabel(uint8_t type)
{
    switch(type)
    {
        case BSP_PRINTER_EVENT_TEMP_ALARM:     return g_event_temp_alarm;
        case BSP_PRINTER_EVENT_SMOKE_ALARM:    return g_event_smoke_alarm;
        case BSP_PRINTER_EVENT_MANUAL_ALARM:   return g_event_manual_alarm;
        case BSP_PRINTER_EVENT_CO_LOW:         return g_event_co_low;
        case BSP_PRINTER_EVENT_CO_HIGH:        return g_event_co_high;
        case BSP_PRINTER_EVENT_H2_LOW:         return g_event_h2_low;
        case BSP_PRINTER_EVENT_H2_HIGH:        return g_event_h2_high;
        case BSP_PRINTER_EVENT_COMM_FAULT:     return g_event_comm_fault;
        case BSP_PRINTER_EVENT_TEMP_FAULT:     return g_event_temp_fault;
        case BSP_PRINTER_EVENT_SMOKE_FAULT:    return g_event_smoke_fault;
        case BSP_PRINTER_EVENT_INPUT_FAULT:    return g_event_input_fault;
        case BSP_PRINTER_EVENT_OUTPUT_FAULT:   return g_event_output_fault;
        case BSP_PRINTER_EVENT_IDENTIFY_FAULT: return g_event_identify_fault;
        case BSP_PRINTER_EVENT_BRANCH1_SHORT:  return g_event_branch1_short;
        case BSP_PRINTER_EVENT_BRANCH2_SHORT:  return g_event_branch2_short;
        case BSP_PRINTER_EVENT_BRANCH3_SHORT:  return g_event_branch3_short;
        case BSP_PRINTER_EVENT_FAN1_FAULT:     return g_event_fan1_fault;
        case BSP_PRINTER_EVENT_FAN2_FAULT:     return g_event_fan2_fault;
        default:                               return g_event_device_fault;
    }
}

static uint8_t PrinterAppendByte(uint8_t *frame, uint8_t capacity,
                                 uint8_t *length, uint8_t value)
{
    if(*length >= capacity) return 0U;
    frame[(*length)++] = value;
    return 1U;
}

static uint8_t PrinterAppendText(uint8_t *frame, uint8_t capacity,
                                 uint8_t *length, const uint8_t *text)
{
    while(*text != 0U)
    {
        if(PrinterAppendByte(frame, capacity, length, *text++) == 0U) return 0U;
    }
    return 1U;
}

static uint8_t PrinterAppendPaddedText(uint8_t *frame, uint8_t capacity,
                                       uint8_t *length, const uint8_t *text,
                                       uint8_t width)
{
    uint8_t text_length = (uint8_t)strlen((const char *)text);
    uint8_t index;
    if(text_length > width || PrinterAppendText(frame, capacity, length, text) == 0U)
        return 0U;
    for(index = text_length; index < width; index++)
    {
        if(PrinterAppendByte(frame, capacity, length, ' ') == 0U) return 0U;
    }
    return 1U;
}

static void PrinterHostFaultLabels(uint8_t fault_type,
                                   const uint8_t **component,
                                   const uint8_t **state)
{
    switch(fault_type)
    {
        case BSP_PRINTER_HOST_MAIN_POWER:
            *component = g_host_main_power;
            *state = g_host_power_off;
            break;
        case BSP_PRINTER_HOST_BACK_POWER_OPEN:
            *component = g_host_back_power;
            *state = g_host_open_fault;
            break;
        case BSP_PRINTER_HOST_BACK_POWER_SHORT:
            *component = g_host_back_power;
            *state = g_host_short_fault;
            break;
        case BSP_PRINTER_HOST_STORAGE:
            *component = g_host_storage;
            *state = g_host_storage_fault;
            break;
        default:
            *component = g_host_reset;
            *state = g_host_send_error;
            break;
    }
}

static uint8_t PrinterAppendFixedNumber(uint8_t *frame, uint8_t capacity,
                                        uint8_t *length, uint16_t value,
                                        uint8_t digits)
{
    uint16_t divisor = 1U;
    uint8_t index;
    for(index = 1U; index < digits; index++) divisor = (uint16_t)(divisor * 10U);
    for(index = 0U; index < digits; index++)
    {
        if(PrinterAppendByte(frame, capacity, length,
                             (uint8_t)('0' + ((value / divisor) % 10U))) == 0U)
            return 0U;
        divisor = (uint16_t)(divisor / 10U);
    }
    return 1U;
}

static uint8_t PrinterAppendNumber(uint8_t *frame, uint8_t capacity,
                                   uint8_t *length, uint16_t value)
{
    uint16_t divisor = 10000U;
    uint8_t started = 0U;
    while(divisor != 0U)
    {
        uint8_t digit = (uint8_t)((value / divisor) % 10U);
        if(digit != 0U || started != 0U || divisor == 1U)
        {
            if(PrinterAppendByte(frame, capacity, length, (uint8_t)('0' + digit)) == 0U)
                return 0U;
            started = 1U;
        }
        divisor = (uint16_t)(divisor / 10U);
    }
    return 1U;
}

void BspPrinter_Init(void)
{
    if(g_printer_queue == NULL)
    {
        g_printer_queue = xQueueCreateStatic(BSP_PRINTER_QUEUE_CAPACITY,
                                              sizeof(BspPrinterQueueItem),
                                              g_printer_queue_storage,
                                              &g_printer_queue_control);
    }
    g_printer_dropped_count = 0U;
}

void BspPrinter_Reset(void)
{
    if(g_printer_queue != NULL) (void)xQueueReset(g_printer_queue);
    g_printer_dropped_count = 0U;
}

uint8_t BspPrinter_QueueLoopEvent(const BspPrinterLoopEvent *event)
{
    BspPrinterQueueItem item;
    if(event == NULL || g_printer_queue == NULL || event->loop < 1U ||
       event->loop > 4U || event->address == 0U ||
       event->event_type == BSP_PRINTER_EVENT_NONE)
        return 0U;

    memset(&item, 0, sizeof(item));
    item.year = event->year < 100U ? (uint16_t)(event->year + 2000U) : event->year;
    item.month = event->month;
    item.day = event->day;
    item.hour = event->hour;
    item.minute = event->minute;
    item.second = event->second;
    item.record_type = BSP_PRINTER_RECORD_LOOP;
    item.loop = event->loop;
    item.address = event->address;
    item.device_type = event->device_type;
    item.event_type = event->event_type;
    if(xQueueSendToBack(g_printer_queue, &item, 0U) != pdPASS)
    {
        if(g_printer_dropped_count != 0xFFFFU) g_printer_dropped_count++;
        return 0U;
    }
    return 1U;
}

uint8_t BspPrinter_QueueHostFault(const BspPrinterHostFault *fault)
{
    BspPrinterQueueItem item;
    if(fault == NULL || g_printer_queue == NULL ||
       fault->fault_type <= BSP_PRINTER_HOST_FAULT_NONE ||
       fault->fault_type > BSP_PRINTER_HOST_RESET_SEND)
        return 0U;

    memset(&item, 0, sizeof(item));
    item.year = fault->year < 100U ? (uint16_t)(fault->year + 2000U) : fault->year;
    item.month = fault->month;
    item.day = fault->day;
    item.hour = fault->hour;
    item.minute = fault->minute;
    item.second = fault->second;
    item.record_type = BSP_PRINTER_RECORD_HOST_FAULT;
    item.host_fault_type = fault->fault_type;
    if(xQueueSendToBack(g_printer_queue, &item, 0U) != pdPASS)
    {
        if(g_printer_dropped_count != 0xFFFFU) g_printer_dropped_count++;
        return 0U;
    }
    return 1U;
}

uint8_t BspPrinter_HasPending(void)
{
    return (g_printer_queue != NULL && uxQueueMessagesWaiting(g_printer_queue) != 0U) ? 1U : 0U;
}

uint8_t BspPrinter_BuildNextFrame(uint8_t *frame, uint8_t capacity,
                                  uint8_t *frame_length)
{
    BspPrinterQueueItem event;
    const uint8_t *device_label;
    const uint8_t *event_label;
    const uint8_t *host_component;
    const uint8_t *host_state;
    uint8_t length = 0U;
    uint8_t payload_length;
    uint8_t index;
    uint16_t crc;
    static const uint8_t loop_prefix[] = {0xB5U, 0xDAU, 0U};             /* 第 */
    static const uint8_t loop_middle[] = {0xBBU, 0xD8U, 0xC2U, 0xB7U, 0U}; /* 回路 */
    static const uint8_t address_suffix[] = {0xBAU, 0xC5U, 0U};          /* 号 */

    if(frame == NULL || frame_length == NULL ||
       capacity < BSP_PRINTER_FRAME_MAX_LEN || g_printer_queue == NULL)
        return 0U;
    if(xQueueReceive(g_printer_queue, &event, 0U) != pdPASS) return 0U;

    if(PrinterAppendByte(frame, capacity, &length, BSP_PRINTER_ADDRESS) == 0U ||
       PrinterAppendByte(frame, capacity, &length, BSP_PRINTER_FUNCTION) == 0U ||
       PrinterAppendByte(frame, capacity, &length, 0U) == 0U ||
       PrinterAppendFixedNumber(frame, capacity, &length, event.year, 4U) == 0U ||
       PrinterAppendByte(frame, capacity, &length, '-') == 0U ||
       PrinterAppendFixedNumber(frame, capacity, &length, event.month, 2U) == 0U ||
       PrinterAppendByte(frame, capacity, &length, '-') == 0U ||
       PrinterAppendFixedNumber(frame, capacity, &length, event.day, 2U) == 0U ||
       PrinterAppendByte(frame, capacity, &length, ' ') == 0U ||
       PrinterAppendFixedNumber(frame, capacity, &length, event.hour, 2U) == 0U ||
       PrinterAppendByte(frame, capacity, &length, ':') == 0U ||
       PrinterAppendFixedNumber(frame, capacity, &length, event.minute, 2U) == 0U ||
       PrinterAppendByte(frame, capacity, &length, ':') == 0U ||
       PrinterAppendFixedNumber(frame, capacity, &length, event.second, 2U) == 0U ||
       PrinterAppendByte(frame, capacity, &length, 0x0DU) == 0U ||
       PrinterAppendByte(frame, capacity, &length, 0x0AU) == 0U)
        return 0U;

    if(event.record_type == BSP_PRINTER_RECORD_HOST_FAULT)
    {
        PrinterHostFaultLabels(event.host_fault_type, &host_component, &host_state);
        if(PrinterAppendPaddedText(frame, capacity, &length, g_host_source,
                                  BSP_PRINTER_HOST_SOURCE_WIDTH) == 0U ||
           PrinterAppendByte(frame, capacity, &length, '|') == 0U ||
           PrinterAppendPaddedText(frame, capacity, &length, host_component,
                                  BSP_PRINTER_HOST_COMPONENT_WIDTH) == 0U ||
           PrinterAppendByte(frame, capacity, &length, '|') == 0U ||
           PrinterAppendPaddedText(frame, capacity, &length, host_state,
                                  BSP_PRINTER_HOST_STATE_WIDTH) == 0U)
            return 0U;
    }
    else
    {
        device_label = PrinterDeviceLabel(event.device_type);
        event_label = PrinterEventLabel(event.event_type);
        if(PrinterAppendText(frame, capacity, &length, loop_prefix) == 0U ||
           PrinterAppendNumber(frame, capacity, &length, event.loop) == 0U ||
           PrinterAppendText(frame, capacity, &length, loop_middle) == 0U ||
           PrinterAppendNumber(frame, capacity, &length, event.address) == 0U ||
           PrinterAppendText(frame, capacity, &length, address_suffix) == 0U ||
           PrinterAppendByte(frame, capacity, &length, '|') == 0U ||
           PrinterAppendText(frame, capacity, &length, device_label) == 0U ||
           PrinterAppendByte(frame, capacity, &length, '|') == 0U ||
           PrinterAppendText(frame, capacity, &length, event_label) == 0U)
            return 0U;
    }
    if(PrinterAppendByte(frame, capacity, &length, 0x0DU) == 0U ||
       PrinterAppendByte(frame, capacity, &length, 0x0AU) == 0U)
        return 0U;

    for(index = 0U; index < BSP_PRINTER_SEPARATOR_LENGTH; index++)
    {
        if(PrinterAppendByte(frame, capacity, &length, '-') == 0U) return 0U;
    }
    if(PrinterAppendByte(frame, capacity, &length, 0x0DU) == 0U ||
       PrinterAppendByte(frame, capacity, &length, 0x0AU) == 0U)
        return 0U;

    payload_length = (uint8_t)(length - 3U);
    if(payload_length > BSP_PRINTER_PAYLOAD_LIMIT) return 0U;
    frame[2] = payload_length;
    crc = CalcCrc16(frame, length);
    if(PrinterAppendByte(frame, capacity, &length, (uint8_t)crc) == 0U ||
       PrinterAppendByte(frame, capacity, &length, (uint8_t)(crc >> 8)) == 0U)
        return 0U;

    *frame_length = length;
    return 1U;
}

uint16_t BspPrinter_GetDroppedCount(void)
{
    return g_printer_dropped_count;
}
