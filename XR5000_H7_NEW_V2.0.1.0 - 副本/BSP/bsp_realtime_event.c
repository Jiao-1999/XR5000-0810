/**
 * @file bsp_realtime_event.c
 * @brief 实时报警/故障事件的运行期管理与屏幕显示实现。
 *
 * 本文件维护画面86和画面87所需的RAM事件列表，完成扫描同步、事件去重、
 * 确认状态保存、分类滚动显示以及报警优先的自动跳转。这里保存的数据不会写入
 * Flash，也不会改变设备状态或参与联动控制；历史存储和实际控制由其他模块负责。
 */

#include "bsp_realtime_event.h"
#include "hmi_driver.h"

#include <stdio.h>
#include <string.h>

/* 自动滚动周期为5秒；离开实时页面3分钟后，如仍有未确认事件则自动返回。 */
#define REALTIME_EVENT_SCROLL_TICKS       5000U
#define REALTIME_EVENT_RETURN_TICKS       180000U

/* 屏幕状态栏使用的RGB565颜色值。 */
#define COLOR_BLACK                       0x0000U
#define COLOR_WHITE                       0xFFFFU
#define COLOR_ALARM_UNCONFIRMED           0xC980U
#define COLOR_FAULT_UNCONFIRMED           0xFCC0U
#define COLOR_CONFIRMED                   0x3666U

typedef struct
{
    uint32_t key;                         /* 事件唯一键，用于识别同一事件条件 */
    RealtimeEventTime time;               /* 事件首次产生时间 */
    uint8_t category;                     /* 所属报警/故障分类 */
    uint8_t confirmed;                    /* 0：未确认，1：已人工确认 */
    uint8_t active;                       /* 0：来源已恢复，1：来源当前仍存在 */
    uint8_t seen;                         /* 本轮扫描中是否再次观察到 */
    uint8_t loop;                         /* 回路号；0表示主机本身 */
    uint8_t address;                      /* 设备地址或主机故障编号 */
    const uint8_t *event_name;            /* 事件名称指针，本模块不复制字符串 */
}RealtimeEventEntry;

/* 每种事件大类各自拥有一个固定容量的运行期存储区。 */
typedef struct
{
    RealtimeEventEntry entry[REALTIME_EVENT_CAPACITY]; /* 按产生顺序保存的事件 */
    uint16_t count;                       /* 已占用记录数，系统复位前不回收 */
    uint16_t revision;                    /* 数据版本号，用于避免无变化重复刷新 */
    uint8_t overflow;                     /* 容量已满标志，按未确认事件处理 */
}RealtimeEventStore;

/* 模块运行状态：存储区、各分类滚动位置、跳转标志及屏幕渲染版本。 */
static RealtimeEventStore g_store[REALTIME_EVENT_KIND_COUNT];
static uint16_t g_scroll[REALTIME_EVENT_KIND_COUNT][4U];
static uint32_t g_last_scroll_tick;
static uint32_t g_leave_tick;
static uint8_t g_new_alarm;
static uint8_t g_new_fault;
static uint16_t g_render_revision[REALTIME_EVENT_KIND_COUNT];

/* 画面86中四类报警对应的事件正文控件ID。 */
static const uint16_t g_alarm_text_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {49U, 50U, 51U, 52U, 53U, 54U, 55U},
    {35U, 36U, 37U, 38U, 39U, 40U, 41U},
    {42U, 43U, 44U, 45U, 46U, 47U, 48U},
    { 1U,  2U,  3U,  4U,  5U,  6U,  7U}
};

/* 画面86中各报警行对应的处理状态控件ID；0表示该行没有状态控件。 */
static const uint16_t g_alarm_state_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {65U, 66U, 67U, 68U, 69U, 70U, 71U},
    {30U, 31U, 32U, 33U, 34U, 56U, 57U},
    { 0U,  0U,  0U,  0U,  0U,  0U,  0U},
    { 0U,  0U,  0U,  0U,  0U,  0U,  0U}
};

/* 画面87中四类故障对应的事件正文控件ID。 */
static const uint16_t g_fault_text_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {24U, 50U, 51U, 52U, 53U, 54U, 55U},
    {35U, 36U, 37U, 38U, 39U, 40U, 41U},
    {42U, 43U, 44U, 45U, 46U, 47U, 48U},
    { 1U,  2U,  3U,  4U,  5U,  6U,  7U}
};

/* 画面87中各故障行对应的处理状态控件ID。 */
static const uint16_t g_fault_state_ctrl[4U][REALTIME_EVENT_ROWS_PER_CATEGORY] = {
    {65U, 66U, 67U, 68U, 69U, 70U, 71U},
    {27U, 30U, 31U, 32U, 33U, 34U, 49U},
    {56U, 57U, 58U, 59U, 60U, 61U, 62U},
    {63U, 64U, 72U, 73U, 74U, 75U, 76U}
};

/* 状态栏使用GB2312编码，分别表示报警/故障的未处理和已确认状态。 */
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

/**
 * @brief 获取指定事件大类包含的分类数量。
 * @param kind 事件大类。
 * @return 报警或故障对应的分类数量。
 */
static uint8_t RealtimeCategoryCount(RealtimeEventKind kind)
{
    return (kind == REALTIME_EVENT_ALARM) ?
           REALTIME_ALARM_CATEGORY_COUNT : REALTIME_FAULT_CATEGORY_COUNT;
}

/**
 * @brief 从后向前查找具有相同唯一键的最新记录。
 * @param store 待搜索的事件存储区。
 * @param key 事件唯一键。
 * @return 找到时返回记录指针，否则返回NULL。
 */
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

/**
 * @brief 统计存储区中指定分类的全部保留记录。
 * @param store 事件存储区。
 * @param category 事件分类。
 * @return 该分类的记录总数，包含当前已恢复的历史运行期记录。
 */
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

/**
 * @brief 将指定分类的显示窗口移动到最新一页。
 * @param kind 事件大类。
 * @param category 事件分类。
 */
static void RealtimeMoveToNewest(RealtimeEventKind kind, uint8_t category)
{
    uint16_t count = RealtimeCountCategory(&g_store[kind], category);
    g_scroll[kind][category] = (count > REALTIME_EVENT_ROWS_PER_CATEGORY) ?
        (uint16_t)(count - REALTIME_EVENT_ROWS_PER_CATEGORY) : 0U;
}

/**
 * @brief 初始化实时事件模块的全部RAM状态。
 */
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

/**
 * @brief 响应系统复位，清空全部实时事件及显示状态。
 */
void BspRealtimeEvent_Reset(void)
{
    BspRealtimeEvent_Init();
}

/**
 * @brief 开始一次事件同步扫描，并清除活动记录的本轮可见标志。
 * @param kind 本轮扫描的事件大类。
 */
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

/**
 * @brief 上报扫描中观察到的一条事件，并完成去重或重新激活处理。
 * @param kind 事件大类。
 * @param category 事件分类。
 * @param key 事件唯一键。
 * @param time 事件首次产生时间。
 * @param loop 回路号；0表示主机本身。
 * @param address 设备地址或主机故障编号。
 * @param event_name 长期有效的事件名称字符串，本模块仅保存其指针。
 * @return 真正新增记录时返回1；重复、重新激活、参数无效或容量已满时返回0。
 */
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

/**
 * @brief 结束事件同步扫描，并将本轮未观察到的活动事件标记为已恢复。
 * @param kind 本轮扫描的事件大类。
 * @note 已恢复记录仍保留用于显示和人工确认，直到系统复位才清除。
 */
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

/**
 * @brief 获取指定事件分类的保留记录总数。
 * @param kind 事件大类。
 * @param category 事件分类。
 * @return 记录总数；参数无效时返回0。
 */
uint16_t BspRealtimeEvent_GetCount(RealtimeEventKind kind, uint8_t category)
{
    if(kind >= REALTIME_EVENT_KIND_COUNT || category >= RealtimeCategoryCount(kind))
        return 0U;
    return RealtimeCountCategory(&g_store[kind], category);
}

/**
 * @brief 判断指定事件大类是否存在未确认内容。
 * @param kind 事件大类。
 * @return 存在未确认记录或存储溢出时返回1，否则返回0。
 */
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

/**
 * @brief 判断系统复位后是否曾保存过实时事件。
 * @return 有记录或存储溢出时返回1，否则返回0。
 * @note 用于报警/故障期间保持屏幕背光，不表示事件当前仍然活动。
 */
uint8_t BspRealtimeEvent_HasAny(void)
{
    uint8_t kind;
    for(kind = 0U; kind < REALTIME_EVENT_KIND_COUNT; kind++)
    {
        if(g_store[kind].count != 0U || g_store[kind].overflow != 0U)
            return 1U;
    }
    return 0U;
}

/**
 * @brief 处理进入或离开实时报警/故障画面时的内部状态。
 * @param previous_screen 切换前画面ID。
 * @param current_screen 切换后画面ID。
 * @param now_tick 当前系统节拍。
 */
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

/**
 * @brief 按新事件和三分钟回看规则决定是否需要自动跳转。
 * @param current_screen 当前画面ID。
 * @param now_tick 当前系统节拍。
 * @return 目标画面ID；返回0表示不跳转。报警始终优先于故障。
 */
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

/**
 * @brief 获取某个分类中的第N条记录。
 * @param store 事件存储区。
 * @param category 事件分类。
 * @param category_index 分类内部的顺序索引。
 * @return 找到时返回只读记录指针，否则返回NULL。
 */
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

/**
 * @brief 将一个分类的当前可见窗口渲染到指定画面。
 * @param kind 事件大类。
 * @param screen 目标画面ID。
 * @param category 事件分类。
 * @note 正文与确认状态使用相同的滚动索引更新，避免显示错行。
 */
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

/**
 * @brief 按固定周期推进各分类的自动滚动位置。
 * @param kind 当前显示的事件大类。
 * @param now_tick 当前系统节拍。
 */
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

/**
 * @brief 刷新画面86或87的列表、状态和报警分类总数。
 * @param current_screen 当前画面ID。
 * @param now_tick 当前系统节拍。
 * @note 数据版本未变化且无需滚动时不重复发送屏幕数据。
 */
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

/**
 * @brief 将指定分类当前可见行标记为已确认。
 * @param kind 事件大类。
 * @param category 事件分类。
 * @param row 当前可见窗口中的行号，范围0～6。
 * @return 确认成功返回1；参数无效、无记录或已经确认时返回0。
 */
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
