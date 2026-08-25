/* ============================================================================
 * ģ������: MBus/��·2�豸����ģ�� (MBus Control Module)
 * ��������: ʵ�ֻ�·2(UART2) MBus�����豸����ѯ���ȡ�Modbus RTUͨ�š�
 *          ״̬���������ⱨ�������ơ�������ʾ���¼��ϱ���Flash�־û���
 * ͨ��Э��: Modbus RTU, ������04(������Ĵ���)/05(д����Ȧ)/10(д��Ĵ���),
 *          UART2/115200/8N1, MBUS2SITE=1
 * �豸����: ���ⱨ����(XR-SGBJQ,��ַ60)/�ֶ�������(XR2200,��ַ61)/������ʾ��(XR1530,��ַ62)
 * ��ѯ����: ÿ10����������(200ms)��ѯһ�������豸, ����04�������1���Ĵ���
 * ���Ʒ���: ���ⱨ����ͨ��05���������, ������ʾ��ͨ��10�������ϱ��¼�
 * ���߼��: ����ʧ��(������)��ʱ����Ӧ���Ƶ���, ����10���ж�����
 * ��·��ʶ: ��·2, Flash�洢��ַ0x110000, ���ϴ�ID=0x52(82��)
 * ============================================================================ */

#include "bsp_mbus_control.h"
#include "bsp_device_registry.h"
#include "bsp_mbus.h"          
#include "bsp_itcallback.h"     
#include "system.h"             
#include "w25qxx.h"             
#include "cmsis_os.h"           

/* ============================================================
 * �ڲ����������ݽṹ
 * ============================================================ */

static MBusCtrlDevice g_mbus_ctrl_devices[MBUS_CONTROL_MAX_DEVICES]; /* �豸ʵ������(����=��ַ) */
static uint8_t g_mbus_ctrl_polling_addr = 1; /* ��ǰ��ѯ��ַ(1~63ѭ��) */

#define MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN       16U  /* ������ʾ���¼�������󳤶� */
#define MBUS_FIRE_DISPLAY_RESPONSE_WAIT_TICKS   15U  /* ������ʾ����Ӧ�ȴ���ʱ(tick) */
#define MBUS_FIRE_DISPLAY_MAX_RETRY             3U   /* ������ʾ��������Դ��� */
#define MBUS_SOUND_LIGHT_RESPONSE_WAIT_TICKS    15U  /* ���ⱨ������Ӧ�ȴ���ʱ(tick) */
#define MBUS_SOUND_LIGHT_MAX_RETRY              3U   /* ���ⱨ����������Դ��� */

/* ������ʾ���¼�����(���ζ���Ԫ��) */
typedef struct
{
    uint8_t loop;          /* ��·��� */
    uint8_t addr;          /* ̽������ַ */
    uint8_t detector_type; /* ̽��������(MBUS_FIRE_DISPLAY_DETECT_*) */
    uint8_t alarm_type;    /* ��������(MBUS_FIRE_DISPLAY_ALARM_*) */
} MBusFireDisplayEvent;

/* ������ʾ���¼����ζ��� */
static MBusFireDisplayEvent g_fire_display_event_queue[MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN];
static uint8_t g_fire_display_event_head = 0;    /* ����ͷָ��(����λ��) */
static uint8_t g_fire_display_event_tail = 0;    /* ����βָ��(���λ��) */
static uint8_t g_fire_display_event_count = 0;   /* ��ǰ�����е��¼��� */
static uint8_t g_fire_display_wait_response = 0; /* �Ƿ�ȴ���ʾ����Ӧ */
static uint8_t g_fire_display_wait_ticks = 0;    /* �ȴ���Ӧ�Ѽ�tick�� */
static uint8_t g_fire_display_retry_count = 0;   /* ��ǰ�¼����Լ��� */

/* ���ⱨ��������״̬(�������豸����״̬, �������Ŀ��ֵ) */
static uint8_t g_sound_light_target_valid = 0;   /* Ŀ��ֵ�Ƿ���Ч */
static uint8_t g_sound_light_target_state = 0;   /* Ŀ��״̬(0=�ر�, 1=����) */
static uint8_t g_sound_light_confirmed_valid = 0;/* ȷ��ֵ�Ƿ���Ч */
static uint8_t g_sound_light_confirmed_state = 0;/* ��ȷ�ϵĵ�ǰ״̬ */
static uint8_t g_sound_light_request_pending = 0;/* �Ƿ��д����͵Ŀ������� */
static uint8_t g_sound_light_wait_response = 0;  /* �Ƿ�ȴ����ⱨ������Ӧ */
static uint8_t g_sound_light_wait_ticks = 0;     /* �ȴ���Ӧ�Ѽ�tick�� */
static uint8_t g_sound_light_retry_count = 0;    /* ��ǰ�������Լ��� */

/* �̶���ַ�豸����ӳ���(����=��ַ, ֵ=MBusCtrlDevType) */
/* �豸��������ӳ���(����=MBusCtrlDevType) */
#define MBUS2_IDENTIFY_FAIL_THRESHOLD 3U
#define MBUS2_IDENTIFY_RETRY_MS 1000U
#define MBUS2_STAGE_NATIONAL 0U
#define MBUS2_STAGE_PRODUCT  1U
#define MBUS2_STAGE_COMPLETE 2U

static uint8_t MBusCtrl_MapProductType(uint16_t product_code)
{
    uint8_t parser;
    if(DeviceRegistry_IsSupportedOnLoop(product_code, DEVICE_REGISTRY_LOOP2) == 0U) return MBUS_CONTROL_DEV_UNKNOWN;
    parser = DeviceRegistry_GetParserType(product_code);
    if(parser == DEVICE_PARSER_SGBJQ) return MBUS_CONTROL_DEV_SGBJQ;
    if(parser == DEVICE_PARSER_XR2200) return MBUS_CONTROL_DEV_XR2200;
    if(parser == DEVICE_PARSER_XR1503) return MBUS_CONTROL_DEV_FIRE_DISPLAY;
    if(parser == DEVICE_PARSER_GCM1002) return MBUS_CONTROL_DEV_GCM1002;
    if(parser == DEVICE_PARSER_FIM1017) return MBUS_CONTROL_DEV_FIM1017;
    return MBUS_CONTROL_DEV_UNKNOWN;
}

static uint8_t MBusCtrl_FindAddressByType(uint8_t dev_type)
{
    uint8_t addr;
    for(addr = 1U; addr < MBUS_CONTROL_MAX_DEVICES; addr++)
        if(g_mbus_ctrl_devices[addr].online != 0U && g_mbus_ctrl_devices[addr].type_confirmed != 0U &&
           g_mbus_ctrl_devices[addr].dev_type == dev_type) return addr;
    return 0U;
}

static void MBusCtrl_MarkIdentifyFailure(uint8_t addr, DeviceIdentifyError error)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return;
    if(error == DEVICE_IDENTIFY_NATIONAL_UNKNOWN || error == DEVICE_IDENTIFY_CODE_MISMATCH)
        g_mbus_ctrl_devices[addr].identify_stage = MBUS2_STAGE_NATIONAL;
    if(g_mbus_ctrl_devices[addr].identify_fail_count < MBUS2_IDENTIFY_FAIL_THRESHOLD)
        g_mbus_ctrl_devices[addr].identify_fail_count++;
    g_mbus_ctrl_devices[addr].last_identify_tick = osKernelGetTickCount();
    if(g_mbus_ctrl_devices[addr].identify_fail_count >= MBUS2_IDENTIFY_FAIL_THRESHOLD)
        DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP2, addr, error);
}
/* ============================================================
 * ������ʾ���¼�����: 10������д��Ĵ���, ���ζ���+���Ի���
 * ============================================================ */

/* ����һ��������ʾ���¼�֡: 10������, д2���Ĵ�����8�ֽ�(��·/��ַ/̽��������/��������) */
static void MBusCtrl_SendFireDisplayEvent(void)
{
    uint8_t frame[17] = {0};
    uint16_t crc16;
    MBusFireDisplayEvent *event = &g_fire_display_event_queue[g_fire_display_event_head];

    frame[0] = MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_FIRE_DISPLAY);
    if(frame[0] == 0U) return;
    frame[1] = 0x10;
    frame[3] = 0x02;
    frame[5] = 0x04;
    frame[6] = 0x08;
    frame[8] = event->loop;
    frame[10] = event->addr;
    frame[12] = event->detector_type;
    frame[14] = event->alarm_type;
    crc16 = CalcCrc16(frame, 15);
    frame[15] = crc16 & 0xFF;
    frame[16] = crc16 >> 8;
    MBus2SendString(frame, sizeof(frame));
    g_fire_display_wait_response = 1;
    g_fire_display_wait_ticks = 0;
}

/* ������ʾ���¼�����: �����С������¼����ȴ���Ӧ����ʱ���ԡ����� */
static uint8_t MBusCtrl_ServiceFireDisplayEvents(void)
{
    if (g_fire_display_event_count == 0)
        return 0;
    if (MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_FIRE_DISPLAY) == 0U)
        return 0;
    if (g_fire_display_wait_response == 0)
    {
        MBusCtrl_SendFireDisplayEvent();
    }
    else if (++g_fire_display_wait_ticks >= MBUS_FIRE_DISPLAY_RESPONSE_WAIT_TICKS)
    {
        if (g_fire_display_retry_count++ < MBUS_FIRE_DISPLAY_MAX_RETRY)
            MBusCtrl_SendFireDisplayEvent();
        else
        {
            /* XR5000_FIRE_DISPLAY_GENERIC_NAMES_20260729: do not block MBus2 after a display write failure. */
            g_fire_display_event_head = (g_fire_display_event_head + 1U) % MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN;
            g_fire_display_event_count--;
            g_fire_display_wait_response = 0;
            g_fire_display_retry_count = 0;
        }
    }
    return 1;
}

/* ============================================================
 * ���ⱨ��������: 05������д����Ȧ, ��XR2200�ֶ�������״̬����
 * ============================================================ */

/* ����XR2200�ֶ�������״̬�������ⱨ����Ŀ��ֵ: state=2(����)������, state=1(����)���ر� */
static void MBusCtrl_UpdateManualSoundLightTarget(uint8_t manual_state)
{
    uint8_t target_state;

    if (manual_state == 2U)
        target_state = 1U;
    else if (manual_state == 1U)
        target_state = 0U;
    else
        return;

    /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: only valid XR2200 states change the retained target. */
    g_sound_light_target_valid = 1U;
    g_sound_light_target_state = target_state;
    if (g_sound_light_confirmed_valid == 0U || g_sound_light_confirmed_state != target_state)
        g_sound_light_request_pending = 1U;
}

/* �������ⱨ��������֡: 05������, ��Ȧֵ0xFF00(����)��0x0000(�ر�) */
static void MBusCtrl_SendSoundLightControl(void)
{
    uint8_t frame[8] = {0};
    uint16_t crc16;

    frame[0] = MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_SGBJQ);
    if(frame[0] == 0U) return;
    frame[1] = 0x05;
    frame[4] = g_sound_light_target_state ? 0xFF : 0x00;
    crc16 = CalcCrc16(frame, 6);
    frame[6] = crc16 & 0xFF;
    frame[7] = crc16 >> 8;
    MBus2SendString(frame, sizeof(frame));
    g_sound_light_wait_response = 1U;
    g_sound_light_wait_ticks = 0U;
}

/* ���ⱨ�������Ʒ���: �����������Ϳ���֡���ȴ���Ӧ����ʱ���ԡ����� */
static uint8_t MBusCtrl_ServiceSoundLightControl(void)
{
    if (g_sound_light_target_valid == 0U || g_sound_light_request_pending == 0U)
        return 0U;
    if (MBusCtrl_FindAddressByType(MBUS_CONTROL_DEV_SGBJQ) == 0U)
        return 0U;

    if (g_sound_light_wait_response == 0U)
    {
        MBusCtrl_SendSoundLightControl();
    }
    else if (++g_sound_light_wait_ticks >= MBUS_SOUND_LIGHT_RESPONSE_WAIT_TICKS)
    {
        if (g_sound_light_retry_count++ < MBUS_SOUND_LIGHT_MAX_RETRY)
        {
            MBusCtrl_SendSoundLightControl();
        }
        else
        {
            /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: abandon this cycle so polling cannot be blocked forever. */
            g_sound_light_request_pending = 0U;
            g_sound_light_wait_response = 0U;
            g_sound_light_wait_ticks = 0U;
            g_sound_light_retry_count = 0U;
        }
    }
    return 1U;
}

/* ============================================================
 * ��ʼ��: �����豸��, ��Flash�ָ�����״̬
 * ============================================================ */

void MBusCtrl_Init(void)
{
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        g_mbus_ctrl_devices[i].online = 0;
        g_mbus_ctrl_devices[i].disconnect_count = 0;
        g_mbus_ctrl_devices[i].dev_type = MBUS_CONTROL_DEV_UNKNOWN;
        g_mbus_ctrl_devices[i].sensor_state = 0;
        g_mbus_ctrl_devices[i].disconnect_memory = 0;
        g_mbus_ctrl_devices[i].product_code = 0U;
        g_mbus_ctrl_devices[i].national_type_code = 0U;
        g_mbus_ctrl_devices[i].type_confirmed = 0U;
        g_mbus_ctrl_devices[i].identify_stage = MBUS2_STAGE_NATIONAL;
        g_mbus_ctrl_devices[i].identify_fail_count = 0U;
        g_mbus_ctrl_devices[i].identify_request_pending = 0U;
        g_mbus_ctrl_devices[i].last_identify_tick = 0U;
    }
    MBusCtrl_LoadOnlineState();
}

/* ============================================================
 * ����״̬����: ��Ļ�·�����/����豸����
 * ============================================================ */

/* ���õ����豸����״̬: ����ʱͬʱ������߼���/������״̬/���߼��� */
void MBusCtrl_SetOnline(uint8_t addr, uint8_t state)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return;
    g_mbus_ctrl_devices[addr].online = state;
    if (state == 0)
    {
        g_mbus_ctrl_devices[addr].disconnect_count = 0;
        g_mbus_ctrl_devices[addr].sensor_state = 0;
        g_mbus_ctrl_devices[addr].disconnect_memory = 0;
        g_mbus_ctrl_devices[addr].product_code = 0U;
        g_mbus_ctrl_devices[addr].national_type_code = 0U;
        g_mbus_ctrl_devices[addr].dev_type = MBUS_CONTROL_DEV_UNKNOWN;
        g_mbus_ctrl_devices[addr].type_confirmed = 0U;
        g_mbus_ctrl_devices[addr].identify_stage = MBUS2_STAGE_NATIONAL;
        g_mbus_ctrl_devices[addr].identify_fail_count = 0U;
        g_mbus_ctrl_devices[addr].identify_request_pending = 0U;
        DeviceRegistry_SetProductUnknown(DEVICE_REGISTRY_LOOP2, addr, 0U);
    }
}

/* ������������״̬(start~end��ַ��Χ) */
void MBusCtrl_SetOnlineRange(uint8_t start, uint8_t end, uint8_t state)
{
    for (uint8_t addr = start; addr <= end; addr++)
    {
        MBusCtrl_SetOnline(addr, state);
    }
}

/* ============================================================
 * ״̬��ѯ�ӿ�
 * ============================================================ */

/* ��ȡ���߱�־(��Ļ�·�ֵ, ��ʵʱ����״̬) */
uint8_t MBusCtrl_GetOnline(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].online;
}

/* �ж��Ƿ����(��������Ӧ����>=��ֵ) */
uint8_t MBusCtrl_IsDisconnected(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].disconnect_count >= MBUS_CONTROL_DISCONNECT_THRESHOLD ? 1 : 0;
}

/* ��ȡ��·2�����豸���� */
uint8_t MBusCtrl_GetOnlineCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (g_mbus_ctrl_devices[i].online)
            count++;
    }
    return count;
}

uint8_t MBusCtrl_IsIdentified(uint8_t addr)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0U;
    return g_mbus_ctrl_devices[addr].type_confirmed != 0U ? 1U : 0U;
}

uint16_t MBusCtrl_GetNationalTypeCode(uint8_t addr)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return 0U;
    return g_mbus_ctrl_devices[addr].national_type_code;
}

uint16_t MBusCtrl_GetProductCode(uint8_t addr)
{
    if(addr == 0U || addr >= MBUS_CONTROL_MAX_DEVICES) return 0U;
    return g_mbus_ctrl_devices[addr].product_code;
}

uint8_t MBusCtrl_GetActiveCount(void)
{
    uint8_t count = 0U;
    uint8_t addr;
    for(addr = 1U; addr < MBUS_CONTROL_MAX_DEVICES; addr++)
    {
        if(g_mbus_ctrl_devices[addr].online != 0U &&
           g_mbus_ctrl_devices[addr].type_confirmed != 0U &&
           g_mbus_ctrl_devices[addr].disconnect_count < MBUS_CONTROL_DISCONNECT_THRESHOLD)
        {
            count++;
        }
    }
    return count;
}

/* ͳ�ƻ�·2�����豸����(���ߵ����߼���>=��ֵ) */
uint8_t MBusCtrl_GetDisconnectCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (g_mbus_ctrl_devices[i].online &&
            g_mbus_ctrl_devices[i].disconnect_count >= MBUS_CONTROL_DISCONNECT_THRESHOLD)
            count++;
    }
    return count;
}

/* ͳ�ƻ�·2�����豸��(������δ�����Ҵ��ڱ���״̬) */
uint8_t MBusCtrl_GetAlarmCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 1; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (g_mbus_ctrl_devices[i].online &&
            g_mbus_ctrl_devices[i].disconnect_count < MBUS_CONTROL_DISCONNECT_THRESHOLD &&
            MBusCtrl_IsAlarmState(i))
            count++;
    }
    return count;
}

/* ���ݵ�ַ��ȡ�豸��������(XR-SGBJQ/XR2200/FireDisplay) */
const char* MBusCtrl_GetDeviceName(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES || g_mbus_ctrl_devices[addr].type_confirmed == 0U) return NULL;
    return DeviceRegistry_GetName(g_mbus_ctrl_devices[addr].product_code);
}

/* ��ȡ�豸������״ֵ̬(04�������ȡ�ļĴ���ֵ) */
uint8_t MBusCtrl_GetDeviceState(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].sensor_state;
}

/* ��ȡ�豸�����豸������(�������߼���ʾ��), ��ַ��Ч����0 */
uint16_t MBusCtrl_GetNationalCode(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    return g_mbus_ctrl_devices[addr].national_type_code;
}

/* ��ȡ�豸����(MBusCtrlDevType) */
uint8_t MBusCtrl_GetDeviceType(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return MBUS_CONTROL_DEV_UNKNOWN;
    return g_mbus_ctrl_devices[addr].dev_type;
}

/* �ж��豸�Ƿ��ڱ���״̬: SGBJQ(state=1)/XR2200(state=2)/FireDisplay(state=1) */
uint8_t MBusCtrl_IsAlarmState(uint8_t addr)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return 0;
    uint8_t state = g_mbus_ctrl_devices[addr].sensor_state;
    switch (g_mbus_ctrl_devices[addr].dev_type)
    {
        case MBUS_CONTROL_DEV_SGBJQ:  return (state == 1);
        case MBUS_CONTROL_DEV_XR2200: return (state == 2);
        case MBUS_CONTROL_DEV_FIRE_DISPLAY: return (state == 1);
        default: return 0;
    }
}

/* ============================================================
 * Flash�־û�: ����״̬���洢�����
 * ============================================================ */

/* ����ǰ����״̬�����浽Flash(��ַ0x110000) */
void MBusCtrl_SaveOnlineState(void)
{
    uint8_t online_state[MBUS_CONTROL_MAX_DEVICES];
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        online_state[i] = g_mbus_ctrl_devices[i].online;
    }
    W25QXX_Write(online_state, MBUS_CONTROL_FLASH_ADDR, sizeof(online_state));
}

/* ��Flash��������״̬��(0xFF��Ϊδ����, ��Ϊ����) */
void MBusCtrl_LoadOnlineState(void)
{
    uint8_t online_state[MBUS_CONTROL_MAX_DEVICES];
    W25QXX_Read(online_state, MBUS_CONTROL_FLASH_ADDR, sizeof(online_state));
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        if (online_state[i] == 0xFF)
        {
            g_mbus_ctrl_devices[i].online = 0;
        }
        else
        {
            g_mbus_ctrl_devices[i].online = online_state[i];
        }
    }
}

/* ============================================================
 * ��ѯ����: 04�������ȡ�豸״̬�Ĵ���
 * ============================================================ */

/* ��ѯ��һ�������豸: ����04������֡�����͵�MBus2���С�����ʧ����Ƶ��� */
static void MBusControlPollingManage(void)
{
    uint8_t modbus_buff[8]; uint16_t crc16; uint8_t found = 0U; uint32_t now = osKernelGetTickCount();
    for (uint8_t i = 0; i < MBUS_CONTROL_MAX_DEVICES; i++)
    {
        g_mbus_ctrl_polling_addr++;
        if (g_mbus_ctrl_polling_addr >= MBUS_CONTROL_MAX_DEVICES) g_mbus_ctrl_polling_addr = 1U;
        if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].online != 0U &&
           (DeviceRegistry_IsProductUnknown(DEVICE_REGISTRY_LOOP2, g_mbus_ctrl_polling_addr) == 0U ||
            (now - g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].last_identify_tick) >= MBUS2_IDENTIFY_RETRY_MS))
        { found = 1U; break; }
    }
    if(found == 0U) return;
    if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].type_confirmed == 0U)
    {
        if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending != 0U)
        {
            MBusCtrl_MarkIdentifyFailure(g_mbus_ctrl_polling_addr,
                g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_stage == MBUS2_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE);
            if(DeviceRegistry_IsProductUnknown(DEVICE_REGISTRY_LOOP2, g_mbus_ctrl_polling_addr) != 0U)
            {
                g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending = 0U;
                return;
            }
        }
        modbus_buff[2]=0U;
        modbus_buff[3]=g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_stage == MBUS2_STAGE_NATIONAL ? 0x0DU : 0x0EU;
        g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending = 1U;
        g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].last_identify_tick = now;
    }
    else
    {
        modbus_buff[2]=0U; modbus_buff[3]=0U;
        if(g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].disconnect_count < MBUS_CONTROL_DISCONNECT_THRESHOLD)
            g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].disconnect_count++;
    }
    modbus_buff[0]=g_mbus_ctrl_polling_addr; modbus_buff[1]=0x04U; modbus_buff[4]=0U; modbus_buff[5]=1U;
    crc16=CalcCrc16(modbus_buff,6U); modbus_buff[6]=crc16 & 0xFFU; modbus_buff[7]=crc16 >> 8;
    if(SendDataToMBus2Queue(modbus_buff,8U) != 1 && g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].type_confirmed == 0U)
        g_mbus_ctrl_devices[g_mbus_ctrl_polling_addr].identify_request_pending = 0U;
}

/* ============================================================
 * �������ݴ���: У��CRC������04/05/10/06/03��������Ӧ
 * ============================================================ */

/* ����MBus2���豸��Ӧ����: 04��������´�����״̬, 05������ȷ���������, 10������ȷ����ʾ���¼� */
static void MBus2ReceiveSlaveDataDeal(void)
{
    uint16_t crc16 = 0x0000;
    if (uartbuff[MBUS2SITE].recepetion_flag == 1)
    {
        uartbuff[MBUS2SITE].recepetion_flag = 0;
        if (uartbuff[MBUS2SITE].recepetion_len < 2)
        {
            return;
        }

        crc16 = (uartbuff[MBUS2SITE].recepetion_buff[uartbuff[MBUS2SITE].recepetion_len - 1] << 8) |
                uartbuff[MBUS2SITE].recepetion_buff[uartbuff[MBUS2SITE].recepetion_len - 2];
        if (CalcCrc16(uartbuff[MBUS2SITE].recepetion_buff, uartbuff[MBUS2SITE].recepetion_len - 2) == crc16)
        {
            uint8_t dev_addr = uartbuff[MBUS2SITE].recepetion_buff[0];

            if (dev_addr == 0 || dev_addr >= MBUS_CONTROL_MAX_DEVICES)
                return;

            if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x04 && uartbuff[MBUS2SITE].recepetion_len == 7U && uartbuff[MBUS2SITE].recepetion_buff[2] == 2U)
            {
                uint16_t data = (uartbuff[MBUS2SITE].recepetion_buff[3] << 8) | uartbuff[MBUS2SITE].recepetion_buff[4];
                if(g_mbus_ctrl_devices[dev_addr].identify_request_pending != 0U)
                {
                    g_mbus_ctrl_devices[dev_addr].identify_request_pending = 0U;
                    if(g_mbus_ctrl_devices[dev_addr].identify_stage == MBUS2_STAGE_NATIONAL)
                    {
                        g_mbus_ctrl_devices[dev_addr].national_type_code = data;
                        g_mbus_ctrl_devices[dev_addr].identify_stage = MBUS2_STAGE_PRODUCT;
                        g_mbus_ctrl_devices[dev_addr].identify_fail_count = 0U;
                    }
                    else
                    {
                        uint8_t type = MBusCtrl_MapProductType(data);
                        if(type == MBUS_CONTROL_DEV_UNKNOWN)
                            MBusCtrl_MarkIdentifyFailure(dev_addr, DEVICE_IDENTIFY_PRODUCT_UNKNOWN);
                        else if(DeviceRegistry_IsNationalProductMatch(g_mbus_ctrl_devices[dev_addr].national_type_code, data) == 0U)
                            MBusCtrl_MarkIdentifyFailure(dev_addr, DeviceRegistry_IsNationalTypeKnown(g_mbus_ctrl_devices[dev_addr].national_type_code) != 0U ? DEVICE_IDENTIFY_CODE_MISMATCH : DEVICE_IDENTIFY_NATIONAL_UNKNOWN);
                        else
                        {
                            g_mbus_ctrl_devices[dev_addr].product_code = data;
                            g_mbus_ctrl_devices[dev_addr].dev_type = type;
                            g_mbus_ctrl_devices[dev_addr].type_confirmed = 1U;
                            g_mbus_ctrl_devices[dev_addr].identify_stage = MBUS2_STAGE_COMPLETE;
                            g_mbus_ctrl_devices[dev_addr].identify_fail_count = 0U;
                            g_mbus_ctrl_devices[dev_addr].disconnect_count = 0U;
                            DeviceRegistry_SetIdentifyError(DEVICE_REGISTRY_LOOP2, dev_addr, DEVICE_IDENTIFY_OK);
                        }
                    }
                }
                else
                {
                    g_mbus_ctrl_devices[dev_addr].disconnect_count = 0U;
                    g_mbus_ctrl_devices[dev_addr].sensor_state = (uint8_t)data;
                    if(g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_XR2200) MBusCtrl_UpdateManualSoundLightTarget((uint8_t)data);
                    else if(g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_SGBJQ && data <= 1U)
                    { g_sound_light_confirmed_valid = 1U; g_sound_light_confirmed_state = (uint8_t)data; }
                }
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x84U && g_mbus_ctrl_devices[dev_addr].identify_request_pending != 0U)
            {
                g_mbus_ctrl_devices[dev_addr].identify_request_pending = 0U;
                MBusCtrl_MarkIdentifyFailure(dev_addr,
                    g_mbus_ctrl_devices[dev_addr].identify_stage == MBUS2_STAGE_NATIONAL ? DEVICE_IDENTIFY_NATIONAL_NO_RESPONSE : DEVICE_IDENTIFY_PRODUCT_NO_RESPONSE);
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x05 &&
                     g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_SGBJQ &&
                     g_sound_light_wait_response != 0U &&
                     uartbuff[MBUS2SITE].recepetion_len >= 8U &&
                     uartbuff[MBUS2SITE].recepetion_buff[2] == 0x00U &&
                     uartbuff[MBUS2SITE].recepetion_buff[3] == 0x00U &&
                     uartbuff[MBUS2SITE].recepetion_buff[4] == (g_sound_light_target_state ? 0xFFU : 0x00U) &&
                     uartbuff[MBUS2SITE].recepetion_buff[5] == 0x00U)
            {
                /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: accept only the exact 0x05 coil echo. */
                g_mbus_ctrl_devices[dev_addr].disconnect_count = 0;
                g_sound_light_confirmed_valid = 1U;
                g_sound_light_confirmed_state = g_sound_light_target_state;
                g_sound_light_request_pending = 0U;
                g_sound_light_wait_response = 0U;
                g_sound_light_wait_ticks = 0U;
                g_sound_light_retry_count = 0U;
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x06)
            {
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x10 &&
                     g_mbus_ctrl_devices[dev_addr].dev_type == MBUS_CONTROL_DEV_FIRE_DISPLAY &&
                     g_fire_display_wait_response != 0 &&
                     uartbuff[MBUS2SITE].recepetion_len >= 8)
            {
                g_mbus_ctrl_devices[dev_addr].disconnect_count = 0;
                g_fire_display_event_head = (g_fire_display_event_head + 1U) % MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN;
                g_fire_display_event_count--;
                g_fire_display_wait_response = 0;
                g_fire_display_wait_ticks = 0;
                g_fire_display_retry_count = 0;
            }
            else if (uartbuff[MBUS2SITE].recepetion_buff[1] == 0x03)
            {
            }
            crc16 = 0;
        }
    }
}

/* ============================================================
 * RTOS��ѯ����: ��ѭ�������մ��������Ʒ���(����/��ʾ��)����ѯ����, ���20ms
 * ��ѯ���: ÿ10������(200ms)����һ���豸��ѯ
 * ���Ʒ���: ���ⱨ���������ڻ�����ʾ��, �п�������ʱ��ͣ��ѯ
 * ============================================================ */

void MBusControlPollSlaveAndReceiveTask(void* parameter)
{
    uint8_t modbusbuf[8] = {0};
    uint8_t mbus_poll_delay_count = 0;

    for (;;)
    {
        uint8_t mbus2_control_busy = 0U;

        MBus2ReceiveSlaveDataDeal();
        /* XR5000_MBUS2_MANUAL_SOUNDLIGHT_CHANGE_20260730: finish the active transaction before starting another. */
        if (g_sound_light_wait_response != 0U)
            mbus2_control_busy = MBusCtrl_ServiceSoundLightControl();
        else if (g_fire_display_wait_response != 0U)
            mbus2_control_busy = MBusCtrl_ServiceFireDisplayEvents();
        else if (MBusCtrl_ServiceSoundLightControl())
            mbus2_control_busy = 1U;
        else if (MBusCtrl_ServiceFireDisplayEvents())
            mbus2_control_busy = 1U;

        if (mbus2_control_busy != 0U)
        {
            osDelay(20);
            continue;
        }

        mbus_poll_delay_count++;
        if (mbus_poll_delay_count == 10)
        {
            mbus_poll_delay_count = 0;
            MBusControlPollingManage();
        }

        if (ReceiveDataFromMBus2Queue(modbusbuf) == 1)
        {
            MBus2SendString(modbusbuf, sizeof(modbusbuf));
        }

        osDelay(20);
    }
}

/* ============================================================
 * FreeRTOS��Ϣ���нӿ�: MBus2��ѯ���ݶ��в���
 * ============================================================ */

/* ��Modbus֡���͵�MBus2����(������, ����������0) */
int8_t SendDataToMBus2Queue(uint8_t *buf, uint8_t buf_len)
{
    if (xMBus2QueueHandle == NULL)
        return 0;
    if (xQueueSend(xMBus2QueueHandle, buf, 0) == pdFALSE)
        return 0;
    return 1;
}

/* ��MBus2���н�������(������, ���пշ���0) */
int8_t ReceiveDataFromMBus2Queue(uint8_t *buf)
{
    if (xMBus2QueueHandle == NULL)
        return 0;
    if (xQueueReceive(xMBus2QueueHandle, buf, 0) == pdFALSE)
        return 0;
    return 1;
}

/* ============================================================
 * ������ʾ���¼�����: ����¼������ζ���
 * ============================================================ */

/* �������ʾ���¼���������һ���¼�(��·/��ַ/̽��������/��������) */
uint8_t MBusCtrl_PostFireDisplayEvent(uint8_t loop, uint8_t addr, uint8_t detector_type, uint8_t alarm_type)
{
    if (loop == 0 || addr == 0 || detector_type < MBUS_FIRE_DISPLAY_DETECT_TEMP ||
        detector_type > MBUS_FIRE_DISPLAY_DETECT_MANUAL || alarm_type < MBUS_FIRE_DISPLAY_ALARM_FIRE ||
        alarm_type > MBUS_FIRE_DISPLAY_ALARM_FAULT)
        return 0;
    taskENTER_CRITICAL();
    if (g_fire_display_event_count >= MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN)
    {
        taskEXIT_CRITICAL();
        return 0;
    }
    g_fire_display_event_queue[g_fire_display_event_tail].loop = loop;
    g_fire_display_event_queue[g_fire_display_event_tail].addr = addr;
    g_fire_display_event_queue[g_fire_display_event_tail].detector_type = detector_type;
    g_fire_display_event_queue[g_fire_display_event_tail].alarm_type = alarm_type;
    g_fire_display_event_tail = (g_fire_display_event_tail + 1U) % MBUS_FIRE_DISPLAY_EVENT_QUEUE_LEN;
    g_fire_display_event_count++;
    taskEXIT_CRITICAL();
    return 1;
}


/* ����ע��: ����MBus2�����豸������״̬(bsp_test_inject����) */
void MBusCtrl_InjectSensorState(uint8_t addr, uint8_t state)
{
    if (addr == 0 || addr >= MBUS_CONTROL_MAX_DEVICES)
        return;
    g_mbus_ctrl_devices[addr].online = 1;
    g_mbus_ctrl_devices[addr].disconnect_count = 0;
    g_mbus_ctrl_devices[addr].sensor_state = state;
}
