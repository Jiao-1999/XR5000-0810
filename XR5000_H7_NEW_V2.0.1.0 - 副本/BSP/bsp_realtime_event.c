#include "bsp_realtime_event.h"
#include "hmi_driver.h"

#include <stdio.h>
#include <string.h>

#define REALTIME_EVENT_SCROLL_TICKS       5000U
#define REALTIME_EVENT_RETURN_TICKS       180000U

#define COLOR_BLACK                       0x0000U
#define COLOR_WHITE                       0xFFFFU
#define COLOR_ALARM_UNCONFIRMED           0xC980U
#define COLOR_FAULT_UNCONFIRMED           0xFCC0U
#define COLOR_CONFIRMED                   0x3666U

typedef struct
{
    uint32_t key;
    RealtimeEventTime time;
    uint8_t category;
    uint8_t confirmed;
    uint8_t active;
    uint8_t seen;
    uint8_t loop;
    uint8_t address;
    const uint8_t *event_name;
}RealtimeEventEntry;

typedef struct
{
    RealtimeEventEntry entry[REALTIME_EVENT_CAPACITY];
    uint16_t count;
    uint16_t revision;
    uint8_t overflow;
}RealtimeEventStore;

static RealtimeEventStore g_store[REALTIME_EVENT_KIND_COUNT];
static uint16_t g_scroll[REALTIME_EVENT_KIND_COUNT][4U];
static uint32_t g_last_scroll_tick;
static uint32_t g_leave_tick;
static uint8_t g_new_alarm;
static uint8_t g_new_fault;
static uint16_t g_render_revision[REALTIME_EVENT_KIND_COUNT];

static const uint16_t g_alarm_text_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {49U, 50U, 51U, 52U, 53U, 54U, 55U},
    {35U, 36U, 37U, 38U, 39U, 40U, 41U},
    {42U, 43U, 44U, 45U, 46U, 47U, 48U},
    { 1U,  2U,  3U,  4U,  5U,  6U,  7U}
};

static const uint16_t g_alarm_state_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {65U, 66U, 67U, 68U, 69U, 70U, 71U},
    {30U, 31U, 32U, 33U, 34U, 56U, 57U},
    { 0U,  0U,  0U,  0U,  0U,  0U,  0U},
    { 0U,  0U,  0U,  0U,  0U,  0U,  0U}
};

static const uint16_t g_fault_text_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {24U, 50U, 51U, 52U, 53U, 54U, 55U},
    {35U, 36U, 37U, 38U, 39U, 40U, 41U},
    {42U, 43U, 44U, 45U, 46U, 47U, 48U},
    { 1U,  2U,  3U,  4U,  5U,  6U,  7U}
};

static const uint16_t g_fault_state_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {65U, 66U, 67U, 68U, 69U, 70U, 71U},
    {27U, 30U, 31U, 32U, 33U, 34U, 49U},
    {56U, 57U, 58U, 59U, 60U, 61U, 62U},
    {63U, 64U, 72U, 73U, 74U, 75U, 76U}
};

static const uint8_t g_alarm_pending_text[] = {
    0xB1U,0xA8U,0xBEU,0xAFU,0xCEU,0xB4U,0xB4U,0xA6U,0xC0U,0xEDU,0U
};
static const uint8_t g_alarm_confirmed_text[] = {
    0xB1U,0xA8U,0xBEU,0xAFU,0xD2U,0xD1U,0xC8U,0xB7U,0xC8U,0xCFU,0U
};
static const uint8_t g_fault_pending_text[] = {
    0xB9U,0xCAU,0xD5U,0xCFU,0xCEU,0xB4U,0xB4U,0xA6U,0xC0U,0xEDU,0U
};
static const uint8_t g_fault_confirmed_text[] = {
    0xB9U,0xCAU,0xD5U,0xCFU,0xD2U,0xD1U,0xC8U,0xB7U,0xC8U,0xCFU,0U
};

static uint8_t RealtimeCategoryCount(RealtimeEventKind kind)
{
    return (kind == REALTIME_EVENT_ALARM) ?
           REALTIME_ALARM_CATEGORY_COUNT : REALTIME_FAULT_CATEGORY_COUNT;
}

static RealtimeEventEntry *RealtimeFindLatest(RealtimeEventStore *store,
                                               uint32_t key)
{
    uint16_t index = store->count;
    while(index != 0U)
    {
        index--;
        if(store->entry[index].key == key) return &store->entry[index];
    }
    return NULL;
}

static uint16_t RealtimeCountCategory(const RealtimeEventStore *store,
                                      uint8_t category)
{
    uint16_t index;
    uint16_t count = 0U;
    for(index = 0U; index < store->count; index++)
    {
        if(store->entry[index].category == category) count++;
    }
    return count;
}

static void RealtimeMoveToNewest(RealtimeEventKind kind, uint8_t category)
{
    uint16_t count = RealtimeCountCategory(&g_store[kind], category);
    g_scroll[kind][category] = (count > REALTIME_EVENT_ROWS_PER_CATEGORY) ?
        (uint16_t)(count - REALTIME_EVENT_ROWS_PER_CATEGORY) : 0U;
}

void BspRealtimeEvent_Init(void)
{
    memset(g_store, 0, sizeof(g_store));
    memset(g_scroll, 0, sizeof(g_scroll));
    memset(g_render_revision, 0xFF, sizeof(g_render_revision));
    g_last_scroll_tick = 0U;
    g_leave_tick = 0U;
    g_new_alarm = 0U;
    g_new_fault = 0U;
}

void BspRealtimeEvent_Reset(void)
{
    BspRealtimeEvent_Init();
}

void BspRealtimeEvent_BeginScan(RealtimeEventKind kind)
{
    uint16_t index;
    if(kind >= REALTIME_EVENT_KIND_COUNT) return;
    for(index = 0U; index < g_store[kind].count; index++)
    {
        if(g_store[kind].entry[index].active != 0U)
            g_store[kind].entry[index].seen = 0U;
    }
}

uint8_t BspRealtimeEvent_Observe(RealtimeEventKind kind, uint8_t category,
                                 uint32_t key, const RealtimeEventTime *time,
                                 uint8_t loop, uint8_t address,
                                 const uint8_t *event_name)
{
    RealtimeEventStore *store;
    RealtimeEventEntry *latest;
    RealtimeEventEntry *entry;
    if(kind >= REALTIME_EVENT_KIND_COUNT || category >= RealtimeCategoryCount(kind) ||
       time == NULL || event_name == NULL) return 0U;

    store = &g_store[kind];
    latest = RealtimeFindLatest(store, key);
    if(latest != NULL)
    {
        latest->seen = 1U;
        if(latest->active != 0U) return 0U;
        if(latest->confirmed == 0U)
        {
            latest->active = 1U;
            store->revision++;
            return 0U;
        }
    }

    if(store->count >= REALTIME_EVENT_CAPACITY)
    {
        store->overflow = 1U;
        if(kind == REALTIME_EVENT_ALARM) g_new_alarm = 1U;
        else g_new_fault = 1U;
        return 0U;
    }

    entry = &store->entry[store->count++];
    memset(entry, 0, sizeof(*entry));
    entry->key = key;
    entry->time = *time;
    entry->category = category;
    entry->active = 1U;
    entry->seen = 1U;
    entry->loop = loop;
    entry->address = address;
    entry->event_name = event_name;
    store->revision++;
    RealtimeMoveToNewest(kind, category);
    if(kind == REALTIME_EVENT_ALARM) g_new_alarm = 1U;
    else g_new_fault = 1U;
    return 1U;
}

void BspRealtimeEvent_EndScan(RealtimeEventKind kind)
{
    uint16_t index;
    RealtimeEventStore *store;
    if(kind >= REALTIME_EVENT_KIND_COUNT) return;
    store = &g_store[kind];
    for(index = 0U; index < store->count; index++)
    {
        if(store->entry[index].active != 0U && store->entry[index].seen == 0U)
        {
            store->entry[index].active = 0U;
            store->revision++;
        }
    }
}

uint16_t BspRealtimeEvent_GetCount(RealtimeEventKind kind, uint8_t category)
{
    if(kind >= REALTIME_EVENT_KIND_COUNT || category >= RealtimeCategoryCount(kind))
        return 0U;
    return RealtimeCountCategory(&g_store[kind], category);
}

uint8_t BspRealtimeEvent_HasUnconfirmed(RealtimeEventKind kind)
{
    uint16_t index;
    if(kind >= REALTIME_EVENT_KIND_COUNT) return 0U;
    if(g_store[kind].overflow != 0U) return 1U;
    for(index = 0U; index < g_store[kind].count; index++)
    {
        if(g_store[kind].entry[index].confirmed == 0U) return 1U;
    }
    return 0U;
}

void BspRealtimeEvent_OnScreenChanged(uint16_t previous_screen,
                                      uint16_t current_screen,
                                      uint32_t now_tick)
{
    if(current_screen == REALTIME_EVENT_SCREEN_ALARM)
    {
        g_new_alarm = 0U;
        g_leave_tick = 0U;
        g_render_revision[REALTIME_EVENT_ALARM] = 0xFFFFU;
    }
    else if(current_screen == REALTIME_EVENT_SCREEN_FAULT)
    {
        g_new_fault = 0U;
        g_leave_tick = 0U;
        g_render_revision[REALTIME_EVENT_FAULT] = 0xFFFFU;
    }
    else if(previous_screen == REALTIME_EVENT_SCREEN_ALARM ||
            previous_screen == REALTIME_EVENT_SCREEN_FAULT)
    {
        g_leave_tick = now_tick;
    }
}

uint16_t BspRealtimeEvent_GetNavigationTarget(uint16_t current_screen,
                                              uint32_t now_tick)
{
    if(g_new_alarm != 0U)
    {
        g_new_alarm = 0U;
        g_new_fault = 0U;
        if(current_screen != REALTIME_EVENT_SCREEN_ALARM)
            return REALTIME_EVENT_SCREEN_ALARM;
    }
    else if(g_new_fault != 0U)
    {
        g_new_fault = 0U;
        if(BspRealtimeEvent_HasUnconfirmed(REALTIME_EVENT_ALARM) == 0U &&
           current_screen != REALTIME_EVENT_SCREEN_FAULT)
            return REALTIME_EVENT_SCREEN_FAULT;
    }

    if(current_screen != REALTIME_EVENT_SCREEN_ALARM &&
       current_screen != REALTIME_EVENT_SCREEN_FAULT &&
       g_leave_tick != 0U && (uint32_t)(now_tick - g_leave_tick) >= REALTIME_EVENT_RETURN_TICKS)
    {
        g_leave_tick = now_tick;
        if(BspRealtimeEvent_HasUnconfirmed(REALTIME_EVENT_ALARM) != 0U)
            return REALTIME_EVENT_SCREEN_ALARM;
        if(BspRealtimeEvent_HasUnconfirmed(REALTIME_EVENT_FAULT) != 0U)
            return REALTIME_EVENT_SCREEN_FAULT;
    }
    return 0U;
}

static const RealtimeEventEntry *RealtimeEntryAt(const RealtimeEventStore *store,
                                                  uint8_t category,
                                                  uint16_t category_index)
{
    uint16_t index;
    uint16_t count = 0U;
    for(index = 0U; index < store->count; index++)
    {
        if(store->entry[index].category != category) continue;
        if(count == category_index) return &store->entry[index];
        count++;
    }
    return NULL;
}

static void RealtimeRenderCategory(RealtimeEventKind kind, uint16_t screen,
                                   uint8_t category)
{
    uint8_t row;
    uint16_t total = RealtimeCountCategory(&g_store[kind], category);
    const uint16_t (*text_ctrl)[REALTIME_EVENT_ROWS_PER_CATEGORY];
    const uint16_t (*state_ctrl)[REALTIME_EVENT_ROWS_PER_CATEGORY];
    text_ctrl = (kind == REALTIME_EVENT_ALARM) ? g_alarm_text_ctrl : g_fault_text_ctrl;
    state_ctrl = (kind == REALTIME_EVENT_ALARM) ? g_alarm_state_ctrl : g_fault_state_ctrl;

    if(total <= REALTIME_EVENT_ROWS_PER_CATEGORY) g_scroll[kind][category] = 0U;
    else if(g_scroll[kind][category] + REALTIME_EVENT_ROWS_PER_CATEGORY > total)
        g_scroll[kind][category] = (uint16_t)(total - REALTIME_EVENT_ROWS_PER_CATEGORY);

    for(row = 0U; row < REALTIME_EVENT_ROWS_PER_CATEGORY; row++)
    {
        uint16_t category_index = (uint16_t)(g_scroll[kind][category] + row);
        uint16_t state_id = state_ctrl[category][row];
        const RealtimeEventEntry *entry = RealtimeEntryAt(&g_store[kind], category,
                                                          category_index);
        if(entry == NULL)
        {
            clearTextValue(screen, text_ctrl[category][row]);
            if(state_id != 0U)
            {
                clearTextValue(screen, state_id);
                SetControlVisiable(screen, state_id, 0U);
            }
        }
        else
        {
            uint8_t line[96U];
            const uint8_t *state_text;
            uint16_t back_color;
            uint16_t fore_color;
            static const uint8_t device_format[] = {
                '%','0','3','u',' ','%','0','4','u','/','%','0','2','u','/','%','0','2','u',' ',
                '%','0','2','u',':','%','0','2','u',':','%','0','2','u',' ',
                0xB5U,0xDAU,'%','u',0xBBU,0xD8U,0xC2U,0xB7U,'%','u',0xBAU,0xC5U,
                ' ','%','s',0U
            };
            static const uint8_t controller_format[] = {
                '%','0','3','u',' ','%','0','4','u','/','%','0','2','u','/','%','0','2','u',' ',
                '%','0','2','u',':','%','0','2','u',':','%','0','2','u',' ',
                0xB1U,0xBEU,0xBBU,0xFAU,'%','u',0xBAU,0xC5U,' ','%','s',0U
            };
            if(entry->loop == 0U)
            {
                snprintf((char *)line, sizeof(line), (const char *)controller_format,
                         (unsigned int)(category_index + 1U),
                         entry->time.year, entry->time.month, entry->time.day,
                         entry->time.hour, entry->time.minute, entry->time.second,
                         entry->address, entry->event_name);
            }
            else
            {
                snprintf((char *)line, sizeof(line), (const char *)device_format,
                         (unsigned int)(category_index + 1U),
                         entry->time.year, entry->time.month, entry->time.day,
                         entry->time.hour, entry->time.minute, entry->time.second,
                         entry->loop, entry->address, entry->event_name);
            }
            SetTextValue(screen, text_ctrl[category][row], line);
            if(kind == REALTIME_EVENT_ALARM)
            {
                state_text = (entry->confirmed != 0U) ?
                    g_alarm_confirmed_text : g_alarm_pending_text;
                back_color = (entry->confirmed != 0U) ?
                    COLOR_CONFIRMED : COLOR_ALARM_UNCONFIRMED;
                fore_color = COLOR_WHITE;
            }
            else
            {
                state_text = (entry->confirmed != 0U) ?
                    g_fault_confirmed_text : g_fault_pending_text;
                back_color = (entry->confirmed != 0U) ?
                    COLOR_CONFIRMED : COLOR_FAULT_UNCONFIRMED;
                fore_color = COLOR_BLACK;
            }
            if(state_id != 0U)
            {
                SetControlVisiable(screen, state_id, 1U);
                SetControlBackColor(screen, state_id, back_color);
                SetControlForeColor(screen, state_id, fore_color);
                SetTextValue(screen, state_id, (uint8_t *)state_text);
            }
        }
    }
}

static void RealtimeUpdateScroll(RealtimeEventKind kind, uint32_t now_tick)
{
    uint8_t category;
    if((uint32_t)(now_tick - g_last_scroll_tick) < REALTIME_EVENT_SCROLL_TICKS) return;
    g_last_scroll_tick = now_tick;
    for(category = 0U; category < RealtimeCategoryCount(kind); category++)
    {
        uint16_t count = RealtimeCountCategory(&g_store[kind], category);
        if(count > REALTIME_EVENT_ROWS_PER_CATEGORY)
        {
            g_scroll[kind][category]++;
            if(g_scroll[kind][category] + REALTIME_EVENT_ROWS_PER_CATEGORY > count)
                g_scroll[kind][category] = 0U;
            g_render_revision[kind] = 0xFFFFU;
        }
    }
}

void BspRealtimeEvent_Refresh(uint16_t current_screen, uint32_t now_tick)
{
    RealtimeEventKind kind;
    uint8_t category;
    if(current_screen == REALTIME_EVENT_SCREEN_ALARM) kind = REALTIME_EVENT_ALARM;
    else if(current_screen == REALTIME_EVENT_SCREEN_FAULT) kind = REALTIME_EVENT_FAULT;
    else return;

    RealtimeUpdateScroll(kind, now_tick);
    if(g_render_revision[kind] == g_store[kind].revision) return;
    SetScreenUpdateEnable(0U);
    for(category = 0U; category < RealtimeCategoryCount(kind); category++)
        RealtimeRenderCategory(kind, current_screen, category);
    if(kind == REALTIME_EVENT_ALARM)
    {
        uint8_t title[32];
        static const uint8_t fire_total_fmt[] = {
            0xBBU,0xF0U,0xBEU,0xAFU,0xD7U,0xDCU,0xCAU,0xFDU,':','%','u',0U
        };
        static const uint8_t gas_total_fmt[] = {
            0xC6U,0xF8U,0xCCU,0xE5U,0xB1U,0xA8U,0xBEU,0xAFU,
            0xD7U,0xDCU,0xCAU,0xFDU,':','%','u',0U
        };
        snprintf((char *)title, sizeof(title), (const char *)fire_total_fmt,
                 (unsigned int)RealtimeCountCategory(&g_store[kind], REALTIME_ALARM_FIRE));
        SetTextValue(current_screen, 24U, title);
        snprintf((char *)title, sizeof(title), (const char *)gas_total_fmt,
                 (unsigned int)RealtimeCountCategory(&g_store[kind], REALTIME_ALARM_GAS));
        SetTextValue(current_screen, 27U, title);
    }
    SetScreenUpdateEnable(1U);
    g_render_revision[kind] = g_store[kind].revision;
}

uint8_t BspRealtimeEvent_ConfirmVisible(RealtimeEventKind kind,
                                       uint8_t category, uint8_t row)
{
    RealtimeEventEntry *entry;
    uint16_t category_index;
    if(kind >= REALTIME_EVENT_KIND_COUNT || category >= RealtimeCategoryCount(kind) ||
       row >= REALTIME_EVENT_ROWS_PER_CATEGORY) return 0U;
    category_index = (uint16_t)(g_scroll[kind][category] + row);
    entry = (RealtimeEventEntry *)RealtimeEntryAt(&g_store[kind], category,
                                                  category_index);
    if(entry == NULL || entry->confirmed != 0U) return 0U;
    entry->confirmed = 1U;
    g_store[kind].revision++;
    return 1U;
}
