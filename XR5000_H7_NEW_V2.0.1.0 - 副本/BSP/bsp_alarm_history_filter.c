/*
 * 画面78报警历史筛选。
 *
 * 本模块只读取既有报警Flash分区，不修改存储布局，也不参与实时报警处理。
 * 三个筛选条件（回路、日期、报警类型）均可独立使用；未选择时按“全部”处理。
 * 查询及翻页复用一个静态扇区缓存，避免在任务栈中申请大数组。
 */

#include "bsp_alarm_history_filter.h"
#include "bsp_save_ctrl.h"
#include "bsp_mbus_control.h"
#include "bsp_rs485_detect.h"
#include "bsp_screen.h"
#include "hmi_driver.h"
#include <stdio.h>
#include <string.h>

#define ALARM_FILTER_SCREEN_ID            78U
#define ALARM_FILTER_ROWS                 10U
#define ALARM_FILTER_MAX_SECTORS          5U
#define ALARM_FILTER_DATE_TEXT_ID         5U
#define ALARM_FILTER_MESSAGE_ID           61U
#define ALARM_FILTER_PAGE_ID              97U
#define ALARM_FILTER_TOTAL_ID             99U

#define ALARM_FILTER_LOOP_ALL             0U
#define ALARM_FILTER_LOOP_OTHER           4U
#define ALARM_FILTER_STATE_OTHER          4U
#define ALARM_FILTER_STATE_ALL            5U

#define TEXT_ALL_LOOPS       "\xC8\xAB\xB2\xBF\xBB\xD8\xC2\xB7"
#define TEXT_LOOP1           "\xBB\xD8\xC2\xB7" "1"
#define TEXT_LOOP2           "\xBB\xD8\xC2\xB7" "2"
#define TEXT_LOOP3           "\xBB\xD8\xC2\xB7" "3"
#define TEXT_OTHER           "\xC6\xE4\xCB\xFB"
#define TEXT_ALL_STATES      "\xC8\xAB\xB2\xBF\xD7\xB4\xCC\xAC"
#define TEXT_TEMPERATURE     "\xCE\xC2\xB6\xC8"
#define TEXT_SMOKE           "\xD1\xCC\xCE\xED"
#define TEXT_CO              "\xD2\xBB\xD1\xF5\xBB\xAF\xCC\xBC"
#define TEXT_H2              "\xC7\xE2\xC6\xF8"
#define TEXT_QUERYING        "\xD5\xFD\xD4\xDA\xB2\xE9\xD1\xAF\xA3\xAC\xC7\xEB\xC9\xD4\xBA\xF2"
#define TEXT_QUERY_DONE      "\xB2\xE9\xD1\xAF\xCD\xEA\xB3\xC9"
#define TEXT_NO_RECORD       "\xCE\xB4\xB2\xE9\xD1\xAF\xB5\xBD\xB7\xFB\xBA\xCF\xCC\xF5\xBC\xFE\xB5\xC4\xB1\xA8\xBE\xAF\xBC\xC7\xC2\xBC"
#define TEXT_DATE_FORMAT     "\xCA\xB1\xBC\xE4\xB8\xF1\xCA\xBD\xA3\xBA" "YYYYMMDD-YYYYMMDD"
#define TEXT_DATE_INVALID    "\xC7\xEB\xCA\xE4\xC8\xEB\xD3\xD0\xD0\xA7\xC8\xD5\xC6\xDA"
#define TEXT_DATE_ORDER      "\xBD\xE1\xCA\xF8\xC8\xD5\xC6\xDA\xB2\xBB\xC4\xDC\xD4\xE7\xD3\xDA\xBF\xAA\xCA\xBC\xC8\xD5\xC6\xDA"
#define TEXT_NOT_OPEN        "\xB8\xC3\xB7\xD6\xC0\xE0\xD4\xDD\xCE\xB4\xBF\xAA\xB7\xC5"
#define TEXT_READ_FAILED     "Flash\xB1\xA8\xBE\xAF\xBC\xC7\xC2\xBC\xB6\xC1\xC8\xA1\xCA\xA7\xB0\xDC"

typedef struct
{
    uint8_t loop_filter;
    uint8_t state_filter;
    uint8_t date_enabled;
    uint8_t query_valid;
    uint16_t page;
    uint16_t matched_count;
    uint32_t date_begin;
    uint32_t date_end;
    char date_text[18];
} AlarmFilterContext;

static AlarmFilterContext g_alarm_filter;
static FlashReadCache_t g_alarm_sector_cache;

static const uint16_t g_serial_ids[ALARM_FILTER_ROWS] = {6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U};
static const uint16_t g_device_ids[ALARM_FILTER_ROWS] = {16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U, 24U, 25U};
static const uint16_t g_time_ids[ALARM_FILTER_ROWS] = {26U, 27U, 28U, 29U, 30U, 31U, 32U, 33U, 34U, 35U};
static const uint16_t g_state_ids[ALARM_FILTER_ROWS] = {36U, 37U, 38U, 39U, 40U, 41U, 42U, 43U, 44U, 45U};
static const uint16_t g_value_ids[ALARM_FILTER_ROWS] = {86U, 87U, 88U, 89U, 90U, 91U, 92U, 93U, 94U, 95U};

static void AlarmFilter_ClearRows(void)
{
    uint8_t i;
    for(i = 0U; i < ALARM_FILTER_ROWS; i++)
    {
        clearTextValue(ALARM_FILTER_SCREEN_ID, g_serial_ids[i]);
        clearTextValue(ALARM_FILTER_SCREEN_ID, g_device_ids[i]);
        clearTextValue(ALARM_FILTER_SCREEN_ID, g_time_ids[i]);
        clearTextValue(ALARM_FILTER_SCREEN_ID, g_state_ids[i]);
        clearTextValue(ALARM_FILTER_SCREEN_ID, g_value_ids[i]);
    }
}

static void AlarmFilter_ResetResult(void)
{
    g_alarm_filter.query_valid = 0U;
    g_alarm_filter.page = 1U;
    g_alarm_filter.matched_count = 0U;
    AlarmFilter_ClearRows();
    SetTextInt32(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_PAGE_ID, 1U, 0U, 1U);
    SetTextInt32(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_TOTAL_ID, 0U, 0U, 1U);
    clearTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID);
}

static uint8_t AlarmFilter_IsLeapYear(uint16_t year)
{
    return (uint8_t)(((year % 400U) == 0U) || (((year % 4U) == 0U) && ((year % 100U) != 0U)));
}

static uint8_t AlarmFilter_ParseDatePart(const char *text, uint32_t *date_key)
{
    static const uint8_t month_days[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    uint16_t year = 0U;
    uint8_t month = 0U;
    uint8_t day = 0U;
    uint8_t max_day;
    uint8_t i;

    for(i = 0U; i < 8U; i++)
    {
        if(text[i] < '0' || text[i] > '9') return 0U;
    }
    year = (uint16_t)((text[0] - '0') * 1000U + (text[1] - '0') * 100U + (text[2] - '0') * 10U + (text[3] - '0'));
    month = (uint8_t)((text[4] - '0') * 10U + (text[5] - '0'));
    day = (uint8_t)((text[6] - '0') * 10U + (text[7] - '0'));
    if(year < 2000U || year > 2255U || month == 0U || month > 12U) return 0U;
    max_day = month_days[month - 1U];
    if(month == 2U && AlarmFilter_IsLeapYear(year)) max_day = 29U;
    if(day == 0U || day > max_day) return 0U;
    *date_key = (uint32_t)year * 10000UL + (uint32_t)month * 100UL + day;
    return 1U;
}

static uint8_t AlarmFilter_ValidateDate(void)
{
    size_t len = strlen(g_alarm_filter.date_text);
    g_alarm_filter.date_enabled = 0U;
    if(len == 0U || strcmp(g_alarm_filter.date_text, "YYYYMMDD-YYYYMMDD") == 0) return 1U;
    if(len != 17U || g_alarm_filter.date_text[8] != '-')
    {
        SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID, (uint8_t *)TEXT_DATE_FORMAT);
        return 0U;
    }
    if(!AlarmFilter_ParseDatePart(g_alarm_filter.date_text, &g_alarm_filter.date_begin) ||
       !AlarmFilter_ParseDatePart(&g_alarm_filter.date_text[9], &g_alarm_filter.date_end))
    {
        SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID, (uint8_t *)TEXT_DATE_INVALID);
        return 0U;
    }
    if(g_alarm_filter.date_end < g_alarm_filter.date_begin)
    {
        SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID, (uint8_t *)TEXT_DATE_ORDER);
        return 0U;
    }
    g_alarm_filter.date_enabled = 1U;
    return 1U;
}

static uint8_t AlarmFilter_MatchLoop(const FlashSaveFireAlarm_t *record)
{
    uint8_t cluster = record->fs_base.fs_detect_id.cluster_id;
    if(g_alarm_filter.loop_filter == ALARM_FILTER_LOOP_ALL) return 1U;
    if(g_alarm_filter.loop_filter == 1U) return (uint8_t)(cluster == 0U);
    if(g_alarm_filter.loop_filter == 2U) return (uint8_t)(cluster == MBUS_CONTROL_FLASH_ID);
    if(g_alarm_filter.loop_filter == 3U) return (uint8_t)(cluster == RS485_DETECT_FLASH_ID);
    return 0U;
}

static uint8_t AlarmFilter_MatchState(const FlashSaveFireAlarm_t *record)
{
    uint8_t state = record->fs_base.state;
    uint8_t supported = 0U;
    uint8_t group = 0xFFU;

    switch(state)
    {
        case TEMPRT_ALARM:
        case RS485_TEMP_WARNING:
        case LOOP1_TEMP_WARNING:
        case LOOP1_TEMP_WARNING_RECOVERY: group = 0U; supported = 1U; break;
        case SMOKE_ALARM:
        case LOOP1_SMOKE_WARNING:
        case LOOP1_SMOKE_WARNING_RECOVERY: group = 1U; supported = 1U; break;
        case FIRGAS_ALARM:
        case FIRGAS_ALARM_CO:
        case RS485_CO_FIRE:
        case GAS_CO_LOW_ALARM:
        case GAS_CO_HIGH_ALARM: group = 2U; supported = 1U; break;
        case FIRGAS_ALARM_HH:
        case RS485_H2_FIRE:
        case GAS_H2_LOW_ALARM:
        case GAS_H2_HIGH_ALARM: group = 3U; supported = 1U; break;
        case MBUS2_HAND_ALARM: group = ALARM_FILTER_STATE_OTHER; supported = 1U; break;
        case LINKAGE_PRESS:
            if(record->fs_base.fs_detect_id.cabin_or_pack_id == HANDPOT_Package_ID)
            {
                group = ALARM_FILTER_STATE_OTHER;
                supported = 1U;
            }
            break;
        case EAR_RECOVERY: supported = 1U; break;
        default: break;
    }
    if(g_alarm_filter.state_filter == ALARM_FILTER_STATE_ALL) return supported;
    if(state == EAR_RECOVERY) return (uint8_t)(g_alarm_filter.state_filter == 2U || g_alarm_filter.state_filter == 3U);
    return (uint8_t)(supported && group == g_alarm_filter.state_filter);
}

static uint8_t AlarmFilter_MatchDate(const FlashSaveFireAlarm_t *record)
{
    FlashSaveTime_t time_value = {0};
    uint32_t key;
    if(!g_alarm_filter.date_enabled) return 1U;
    getFlashTime_Plus(record->fs_base.fs_time_buff, &time_value);
    key = (uint32_t)(time_value.years + 2000U) * 10000UL + (uint32_t)time_value.months * 100UL + time_value.days;
    return (uint8_t)(key >= g_alarm_filter.date_begin && key <= g_alarm_filter.date_end);
}

static uint8_t AlarmFilter_Match(const FlashSaveFireAlarm_t *record)
{
    return (uint8_t)(AlarmFilter_MatchLoop(record) && AlarmFilter_MatchState(record) && AlarmFilter_MatchDate(record));
}

static void AlarmFilter_FormatDevice(const FlashSaveFireAlarm_t *record, uint8_t *buffer)
{
    uint8_t cluster = record->fs_base.fs_detect_id.cluster_id;
    uint8_t addr = record->fs_base.fs_detect_id.cabin_or_pack_id;
    if(cluster == 0U) sprintf((char *)buffer, "\xB5\xDA" "1" "\xBB\xD8\xC2\xB7 %d\xBA\xC5", addr);
    else if(cluster == MBUS_CONTROL_FLASH_ID) sprintf((char *)buffer, "\xB5\xDA" "2" "\xBB\xD8\xC2\xB7 %d\xBA\xC5", addr);
    else if(cluster == RS485_DETECT_FLASH_ID) sprintf((char *)buffer, "\xB5\xDA" "3" "\xBB\xD8\xC2\xB7 %d\xBA\xC5", addr);
    else if(record->fs_base.state == LINKAGE_PRESS || record->fs_base.state == MBUS2_HAND_ALARM)
        sprintf((char *)buffer, "\xCA\xD6\xB6\xAF\xB1\xA8\xBE\xAF\xC6\xF7");
    else sprintf((char *)buffer, "\xCE\xB4\xD6\xAA\xC9\xE8\xB1\xB8");
}

static void AlarmFilter_FormatState(uint8_t state, uint8_t *buffer)
{
    switch(state)
    {
        case TEMPRT_ALARM: sprintf((char *)buffer, "\xCE\xC2\xB6\xC8\xBB\xF0\xBE\xAF"); break;
        case SMOKE_ALARM: sprintf((char *)buffer, "\xD1\xCC\xCE\xED\xB1\xA8\xBE\xAF"); break;
        case MBUS2_HAND_ALARM:
        case LINKAGE_PRESS: sprintf((char *)buffer, "\xCA\xD6\xB6\xAF\xB1\xA8\xBE\xAF"); break;
        case GAS_CO_LOW_ALARM: sprintf((char *)buffer, "CO\xB5\xCD\xB1\xA8"); break;
        case GAS_CO_HIGH_ALARM: sprintf((char *)buffer, "CO\xB8\xDF\xB1\xA8"); break;
        case GAS_H2_LOW_ALARM: sprintf((char *)buffer, "H2\xB5\xCD\xB1\xA8"); break;
        case GAS_H2_HIGH_ALARM: sprintf((char *)buffer, "H2\xB8\xDF\xB1\xA8"); break;
        case FIRGAS_ALARM:
        case FIRGAS_ALARM_CO: sprintf((char *)buffer, "CO\xB1\xA8\xBE\xAF"); break;
        case FIRGAS_ALARM_HH: sprintf((char *)buffer, "H2\xB1\xA8\xBE\xAF"); break;
        case EAR_RECOVERY: sprintf((char *)buffer, "\xBF\xC9\xC8\xBC\xC6\xF8\xCC\xE5\xBB\xD6\xB8\xB4"); break;
        case RS485_TEMP_WARNING:
        case LOOP1_TEMP_WARNING: sprintf((char *)buffer, "\xCE\xC2\xB6\xC8\xD4\xA4\xBE\xAF"); break;
        case LOOP1_TEMP_WARNING_RECOVERY: sprintf((char *)buffer, "\xCE\xC2\xB6\xC8\xD4\xA4\xBE\xAF\xBB\xD6\xB8\xB4"); break;
        case LOOP1_SMOKE_WARNING: sprintf((char *)buffer, "\xD1\xCC\xCE\xED\xD4\xA4\xBE\xAF"); break;
        case LOOP1_SMOKE_WARNING_RECOVERY: sprintf((char *)buffer, "\xD1\xCC\xCE\xED\xD4\xA4\xBE\xAF\xBB\xD6\xB8\xB4"); break;
        case RS485_CO_FIRE: sprintf((char *)buffer, "CO\xBB\xF0\xBE\xAF"); break;
        case RS485_H2_FIRE: sprintf((char *)buffer, "H2\xBB\xF0\xBE\xAF"); break;
        default: sprintf((char *)buffer, "\xCE\xB4\xD6\xAA\xD7\xB4\xCC\xAC"); break;
    }
}

static void AlarmFilter_ShowRecord(uint8_t row, uint16_t serial, const FlashSaveFireAlarm_t *record)
{
    uint8_t buffer[64] = {0};
    FlashSaveTime_t time_value = {0};
    SetTextInt32(ALARM_FILTER_SCREEN_ID, g_serial_ids[row], serial, 0U, 1U);
    AlarmFilter_FormatDevice(record, buffer);
    SetTextValue(ALARM_FILTER_SCREEN_ID, g_device_ids[row], buffer);
    getFlashTime_Plus(record->fs_base.fs_time_buff, &time_value);
    sprintf((char *)buffer, "%04d\xC4\xEA%02d\xD4\xC2%02d\xC8\xD5 %02d\xCA\xB1%02d\xB7\xD6%02d\xC3\xEB",
            time_value.years + 2000U, time_value.months, time_value.days,
            time_value.hours, time_value.minute, time_value.second);
    SetTextValue(ALARM_FILTER_SCREEN_ID, g_time_ids[row], buffer);
    AlarmFilter_FormatState(record->fs_base.state, buffer);
    SetTextValue(ALARM_FILTER_SCREEN_ID, g_state_ids[row], buffer);
    if(record->data_high == 0xFFFFU || record->fs_base.state == MBUS2_HAND_ALARM || record->fs_base.state == LINKAGE_PRESS)
        SetTextValue(ALARM_FILTER_SCREEN_ID, g_value_ids[row], (uint8_t *)"----");
    else
    {
        sprintf((char *)buffer, "%u", record->data_high);
        SetTextValue(ALARM_FILTER_SCREEN_ID, g_value_ids[row], buffer);
    }
}

static uint8_t AlarmFilter_ScanAndDisplay(void)
{
    int16_t total = getFlashSaveDataNummber(FIRE_FLASH_SAVE);
    uint16_t sector_count;
    uint16_t sector;
    uint16_t matched = 0U;
    uint16_t skip = (uint16_t)((g_alarm_filter.page - 1U) * ALARM_FILTER_ROWS);
    uint8_t displayed = 0U;

    AlarmFilter_ClearRows();
    if(total < 0)
    {
        SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID, (uint8_t *)TEXT_READ_FAILED);
        return 0U;
    }
    sector_count = (uint16_t)(((uint16_t)total + FLASH_ALARM_SUM_PER_SECTOR - 1U) / FLASH_ALARM_SUM_PER_SECTOR);
    if(sector_count > ALARM_FILTER_MAX_SECTORS) sector_count = ALARM_FILTER_MAX_SECTORS;
    for(sector = sector_count; sector > 0U; sector--)
    {
        uint16_t sector_index = (uint16_t)(sector - 1U);
        uint16_t first_index = (uint16_t)(sector_index * FLASH_ALARM_SUM_PER_SECTOR);
        uint16_t end_index = (uint16_t)(first_index + FLASH_ALARM_SUM_PER_SECTOR);
        uint16_t absolute;
        if(end_index > (uint16_t)total) end_index = (uint16_t)total;
        if(BspReadFlashData(FIRE_FLASH_SAVE, g_alarm_sector_cache.byte_buff, (uint8_t)sector_index) < 0)
        {
            SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID, (uint8_t *)TEXT_READ_FAILED);
            return 0U;
        }
        for(absolute = end_index; absolute > first_index; absolute--)
        {
            FlashSaveFireAlarm_t *record = &g_alarm_sector_cache.fs_fire_alarm[absolute - first_index - 1U];
            if(AlarmFilter_Match(record))
            {
                if(matched >= skip && displayed < ALARM_FILTER_ROWS)
                {
                    AlarmFilter_ShowRecord(displayed, (uint16_t)(matched + 1U), record);
                    displayed++;
                }
                matched++;
            }
        }
    }
    g_alarm_filter.matched_count = matched;
    SetTextInt32(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_PAGE_ID, g_alarm_filter.page, 0U, 1U);
    SetTextInt32(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_TOTAL_ID, matched, 0U, 1U);
    SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID,
                 (uint8_t *)(matched == 0U ? TEXT_NO_RECORD : TEXT_QUERY_DONE));
    return 1U;
}

void AlarmHistoryFilter_NotifyScreen(uint16_t screen_id)
{
    if(screen_id != ALARM_FILTER_SCREEN_ID) return;
    memset(&g_alarm_filter, 0, sizeof(g_alarm_filter));
    g_alarm_filter.state_filter = ALARM_FILTER_STATE_ALL;
    g_alarm_filter.page = 1U;
    strcpy(g_alarm_filter.date_text, "YYYYMMDD-YYYYMMDD");
    SetTextValue(ALARM_FILTER_SCREEN_ID, 2U, (uint8_t *)TEXT_ALL_LOOPS);
    SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_DATE_TEXT_ID, (uint8_t *)"YYYYMMDD-YYYYMMDD");
    SetTextValue(ALARM_FILTER_SCREEN_ID, 52U, (uint8_t *)TEXT_ALL_STATES);
    AlarmFilter_ResetResult();
}

void AlarmHistoryFilter_NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state)
{
    static const uint8_t *loop_text[] = {(const uint8_t *)TEXT_ALL_LOOPS, (const uint8_t *)TEXT_LOOP1,
                                         (const uint8_t *)TEXT_LOOP2, (const uint8_t *)TEXT_LOOP3,
                                         (const uint8_t *)TEXT_OTHER};
    static const uint8_t *state_text[] = {(const uint8_t *)TEXT_TEMPERATURE, (const uint8_t *)TEXT_SMOKE,
                                          (const uint8_t *)TEXT_CO, (const uint8_t *)TEXT_H2,
                                          (const uint8_t *)TEXT_OTHER, (const uint8_t *)TEXT_ALL_STATES};
    if(screen_id != ALARM_FILTER_SCREEN_ID || state != 1U) return;
    if(control_id == 3U && item < 5U)
    {
        g_alarm_filter.loop_filter = item;
        SetTextValue(ALARM_FILTER_SCREEN_ID, 2U, (uint8_t *)loop_text[item]);
        AlarmFilter_ResetResult();
    }
    else if(control_id == 53U && item < 6U)
    {
        g_alarm_filter.state_filter = item;
        SetTextValue(ALARM_FILTER_SCREEN_ID, 52U, (uint8_t *)state_text[item]);
        AlarmFilter_ResetResult();
    }
}

void AlarmHistoryFilter_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text)
{
    if(screen_id != ALARM_FILTER_SCREEN_ID || control_id != ALARM_FILTER_DATE_TEXT_ID || text == NULL) return;
    strncpy(g_alarm_filter.date_text, (const char *)text, sizeof(g_alarm_filter.date_text) - 1U);
    g_alarm_filter.date_text[sizeof(g_alarm_filter.date_text) - 1U] = '\0';
    AlarmFilter_ResetResult();
    SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_DATE_TEXT_ID, (uint8_t *)g_alarm_filter.date_text);
}

void AlarmHistoryFilter_NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
    uint16_t total_pages;
    if(screen_id != ALARM_FILTER_SCREEN_ID || state != 1U) return;
    if(control_id == 58U)
    {
        SwitchCurrentScreenId(68U);
        return;
    }
    if(control_id == 57U)
    {
        AlarmFilter_ResetResult();
        if(g_alarm_filter.loop_filter == ALARM_FILTER_LOOP_OTHER)
        {
            SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID, (uint8_t *)TEXT_NOT_OPEN);
            return;
        }
        if(!AlarmFilter_ValidateDate()) return;
        SetTextValue(ALARM_FILTER_SCREEN_ID, ALARM_FILTER_MESSAGE_ID, (uint8_t *)TEXT_QUERYING);
        g_alarm_filter.query_valid = AlarmFilter_ScanAndDisplay();
        return;
    }
    if(!g_alarm_filter.query_valid) return;
    total_pages = (uint16_t)((g_alarm_filter.matched_count + ALARM_FILTER_ROWS - 1U) / ALARM_FILTER_ROWS);
    if(total_pages == 0U) total_pages = 1U;
    if(control_id == 59U && g_alarm_filter.page > 1U)
    {
        g_alarm_filter.page--;
        (void)AlarmFilter_ScanAndDisplay();
    }
    else if(control_id == 60U && g_alarm_filter.page < total_pages)
    {
        g_alarm_filter.page++;
        (void)AlarmFilter_ScanAndDisplay();
    }
}
