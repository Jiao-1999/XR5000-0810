#include "bsp_device_threshold.h"

#include "bsp_rs485_detect.h"
#include "bsp_device_registry.h"
#include "system.h"
#include "hmi_driver.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#define THRESHOLD_SCREEN_SET             73U
#define THRESHOLD_SCREEN_QUERY           76U
#define THRESHOLD_MAX_TARGETS            32U
#define THRESHOLD_ITEM_COUNT              7U
#define THRESHOLD_NORMAL_POLLS_BETWEEN    4U
#define THRESHOLD_VERIFY_RETRY_LIMIT       2U
#define THRESHOLD_CONFIRM_MS          10000U

#define THRESHOLD_MODE_SINGLE             0U
#define THRESHOLD_MODE_LOOP               1U
#define THRESHOLD_MODE_NATIONAL           2U

#define THRESHOLD_OP_NONE                 0U
#define THRESHOLD_OP_QUERY                1U
#define THRESHOLD_OP_WRITE                2U
#define THRESHOLD_OP_DEFAULT              3U

#define THRESHOLD_PHASE_READ              0U
#define THRESHOLD_PHASE_WRITE             1U
#define THRESHOLD_PHASE_VERIFY            2U

#define THRESHOLD_RESULT_IDLE             0U
#define THRESHOLD_RESULT_WAIT             1U
#define THRESHOLD_RESULT_OK               2U
#define THRESHOLD_RESULT_FAIL             3U
#define THRESHOLD_RESULT_UNSUPPORTED      4U

typedef struct
{
    uint16_t reg;
    uint16_t maximum;
    uint16_t default_value;
    uint8_t sensor_bit;
    uint16_t value_control;
    uint16_t status_control;
} ThresholdDefinition;

typedef struct
{
    uint8_t page;
    uint8_t mode;
    uint8_t loop;
    uint8_t operation;
    uint8_t running;
    uint8_t waiting;
    uint8_t phase;
    uint8_t target_count;
    uint8_t target_index;
    uint8_t item_index;
    uint8_t skip_normal_polls;
    uint8_t restore_armed;
    uint8_t ui_dirty;
    uint8_t clear_display;
    uint8_t generation;
    uint8_t transaction_generation;
    uint8_t transaction_address;
    uint8_t transaction_item;
    uint8_t transaction_function;
    uint8_t verify_retry_count;
    uint16_t address_or_code;
    uint16_t requested[THRESHOLD_ITEM_COUNT];
    uint16_t values[THRESHOLD_ITEM_COUNT];
    uint8_t requested_valid[THRESHOLD_ITEM_COUNT];
    uint8_t results[THRESHOLD_ITEM_COUNT];
    uint8_t targets[THRESHOLD_MAX_TARGETS];
    uint16_t success_count;
    uint16_t failure_count;
    uint32_t restore_tick;
} ThresholdContext;

static const ThresholdDefinition g_threshold_defs[THRESHOLD_ITEM_COUNT] =
{
    {0x0008U,  200U,   78U, 5U, 22U, 43U}, /* temperature fire */
    {0x000FU, 3000U, 1500U, 3U, 23U, 44U}, /* VOC alarm */
    {0x000BU, 5000U,  500U, 4U, 31U, 45U}, /* CO low */
    {0x000CU, 5000U,  800U, 4U, 38U, 45U}, /* CO high */
    {0x000DU, 5000U,  500U, 2U, 32U, 46U}, /* H2 low */
    {0x000EU, 5000U,  800U, 2U, 39U, 46U}, /* H2 high */
    {0x0012U, 5000U, 1200U, 6U, 26U, 47U}  /* pressure alarm */
};

static ThresholdContext g_threshold;
static const uint8_t g_text_high_must_exceed_low[] =
    {0xB8U,0xDFU,0xB1U,0xA8U,0xD3U,0xA6U,0xB4U,0xF3U,0xD3U,0xDAU,0xB5U,0xCDU,0xB1U,0xA8U,0U};
/* The screen project uses GBK for text controls. Keep runtime Chinese text
 * as explicit bytes so the source-file encoding cannot change the payload. */
static const uint8_t g_text_loop1[] =
    {0xBBU,0xD8U,0xC2U,0xB7U,0x31U,0U};
static const uint8_t g_text_loop3[] =
    {0xBBU,0xD8U,0xC2U,0xB7U,0x33U,0U};
static const uint8_t g_setting_text_single[] =
    {0xB5U,0xA5U,0xB5U,0xE3U,0xC9U,0xE8U,0xD6U,0xC3U,0U};
static const uint8_t g_setting_text_loop_batch[] =
    {0xBBU,0xD8U,0xC2U,0xB7U,0xC5U,0xFAU,0xC1U,0xBFU,0xC9U,0xE8U,0xD6U,0xC3U,0U};
static const uint8_t g_setting_text_type_batch[] =
    {0xC0U,0xE0U,0xD0U,0xCDU,0xC5U,0xFAU,0xC1U,0xBFU,0xC9U,0xE8U,0xD6U,0xC3U,0U};
static const uint8_t g_setting_text_initial_prompt[] =
    {0xC4U,0xFAU,0xD1U,0xA1U,0xD4U,0xF1U,0xB5U,0xC4U,0xCCU,0xBDU,0xB2U,0xE2U,0xC6U,0xF7U,
     0xBFU,0xC9U,0xD2U,0xD4U,0xBDU,0xF8U,0xD0U,0xD0U,0xD2U,0xD4U,0xCFU,0xC2U,0xE3U,0xD0U,
     0xD6U,0xB5U,0xB5U,0xC4U,0xC9U,0xE8U,0xB6U,0xA8U,0xA3U,0xBAU,0U};
static const uint8_t g_setting_text_no_target[] =
    {0xCEU,0xB4U,0xD5U,0xD2U,0xB5U,0xBDU,0xBFU,0xC9U,0xB2U,0xD9U,0xD7U,0xF7U,0xB5U,0xC4U,
     0xD4U,0xDAU,0xCFU,0xDFU,0xC9U,0xE8U,0xB1U,0xB8U,0U};
static const uint8_t g_setting_text_reading[] =
    {0xD5U,0xFDU,0xD4U,0xDAU,0xB6U,0xC1U,0xC8U,0xA1U,0xB5U,0xB1U,0xC7U,0xB0U,0xE3U,0xD0U,
     0xD6U,0xB5U,0U};
static const uint8_t g_setting_text_writing[] =
    {0xD5U,0xFDU,0xD4U,0xDAU,0xC9U,0xE8U,0xD6U,0xC3U,0xE3U,0xD0U,0xD6U,0xB5U,0U};
static const uint8_t g_setting_text_restoring[] =
    {0xD5U,0xFDU,0xD4U,0xDAU,0xBBU,0xD6U,0xB8U,0xB4U,0xC4U,0xACU,0xC8U,0xCFU,0xD6U,0xB5U,0U};
static const uint8_t g_setting_text_need_high_low[] =
    {0xC7U,0xEBU,0xCDU,0xACU,0xCAU,0xB1U,0xCAU,0xE4U,0xC8U,0xEBU,0xB8U,0xDFU,0xB1U,0xA8U,
     0xBAU,0xCDU,0xB5U,0xCDU,0xB1U,0xA8U,0xE3U,0xD0U,0xD6U,0xB5U,0U};
static const uint8_t g_setting_text_invalid_input[] =
    {0xC7U,0xEBU,0xCAU,0xE4U,0xC8U,0xEBU,0xD3U,0xD0U,0xD0U,0xA7U,0xE3U,0xD0U,0xD6U,0xB5U,0U};
static const uint8_t g_setting_text_restore_confirm[] =
    {0x31U,0x30U,0xC3U,0xEBU,0xC4U,0xDAU,0xD4U,0xD9U,0xB4U,0xCEU,0xB5U,0xE3U,0xBBU,0xF7U,
     0xBBU,0xD6U,0xB8U,0xB4U,0xC4U,0xACU,0xC8U,0xCFU,0U};
static const uint8_t g_setting_text_fire_cancelled[] =
    {0xBBU,0xF0U,0xBEU,0xAFU,0xA3U,0xACU,0xB2U,0xD9U,0xD7U,0xF7U,0xD2U,0xD1U,0xC8U,0xA1U,
     0xCFU,0xFBU,0U};
static const uint8_t g_setting_text_confirm_timeout[] =
    {0xBBU,0xD6U,0xB8U,0xB4U,0xC4U,0xACU,0xC8U,0xCFU,0xC8U,0xB7U,0xC8U,0xCFU,0xB3U,0xACU,
     0xCAU,0xB1U,0U};
static const uint8_t g_setting_text_wait[] =
    {0xB4U,0xA6U,0xC0U,0xEDU,0xD6U,0xD0U,0U};
static const uint8_t g_setting_text_success[] =
    {0xB2U,0xD9U,0xD7U,0xF7U,0xB3U,0xC9U,0xB9U,0xA6U,0U};
static const uint8_t g_setting_text_fail[] =
    {0xB2U,0xD9U,0xD7U,0xF7U,0xCAU,0xA7U,0xB0U,0xDCU,0U};
static const uint8_t g_setting_text_done[] =
    {0xB2U,0xD9U,0xD7U,0xF7U,0xCDU,0xEAU,0xB3U,0xC9U,0U};
static const uint8_t g_query_text_input_prompt[] =
    {0xC7U,0xEBU,0xCAU,0xE4U,0xC8U,0xEBU,0xC9U,0xE8U,0xB1U,0xB8U,0xB5U,0xD8U,0xD6U,0xB7U,
     0xBAU,0xF3U,0xB5U,0xE3U,0xBBU,0xF7U,0xB2U,0xE9U,0xD1U,0xAFU,0U};
static const uint8_t g_query_text_target_selected[] =
    {0xD2U,0xD1U,0xD1U,0xA1U,0xD4U,0xF1U,0xC9U,0xE8U,0xB1U,0xB8U,0xA3U,0xACU,0xC7U,0xEBU,
     0xB5U,0xE3U,0xBBU,0xF7U,0xB2U,0xE9U,0xD1U,0xAFU,0U};
static const uint8_t g_query_text_no_target[] =
    {0xCEU,0xB4U,0xD5U,0xD2U,0xB5U,0xBDU,0xBFU,0xC9U,0xB2U,0xE9U,0xD1U,0xAFU,0xB5U,0xC4U,
     0xD4U,0xDAU,0xCFU,0xDFU,0xC9U,0xE8U,0xB1U,0xB8U,0U};
static const uint8_t g_query_text_querying[] =
    {0xD5U,0xFDU,0xD4U,0xDAU,0xB2U,0xE9U,0xD1U,0xAFU,0U};
static const uint8_t g_query_text_online[] =
    {0xD4U,0xDAU,0xCFU,0xDFU,0U};
static const uint8_t g_query_text_offline[] =
    {0xC0U,0xEBU,0xCFU,0xDFU,0U};
static const uint8_t g_query_text_wait[] =
    {0xB2U,0xE9U,0xD1U,0xAFU,0xD6U,0xD0U,0U};
static const uint8_t g_query_text_success[] =
    {0xB2U,0xE9U,0xD1U,0xAFU,0xB3U,0xC9U,0xB9U,0xA6U,0U};
static const uint8_t g_query_text_fail[] =
    {0xB2U,0xE9U,0xD1U,0xAFU,0xCAU,0xA7U,0xB0U,0xDCU,0U};
static const uint8_t g_text_unsupported[] =
    {0xB2U,0xBBU,0xD6U,0xA7U,0xB3U,0xD6U,0xBBU,0xF2U,0xCEU,0xB4U,0xC6U,0xF4U,0xD3U,0xC3U,0U};
static const uint8_t g_query_text_fire_cancelled[] =
    {0xBBU,0xF0U,0xBEU,0xAFU,0xA3U,0xACU,0xB2U,0xE9U,0xD1U,0xAFU,0xD2U,0xD1U,0xC8U,0xA1U,
     0xCFU,0xFBU,0U};
static const uint8_t g_query_text_done[] =
    {0xB2U,0xE9U,0xD1U,0xAFU,0xCDU,0xEAU,0xB3U,0xC9U,0U};
static const uint8_t g_text_success_count[] =
    {0xB3U,0xC9U,0xB9U,0xA6U,0U};
static const uint8_t g_text_fail_count[] =
    {0xCAU,0xA7U,0xB0U,0xDCU,0U};

static void threshold_crc(uint8_t frame[8])
{
    uint16_t crc = CalcCrc16(frame, 6U);
    frame[6] = (uint8_t)(crc & 0xFFU);
    frame[7] = (uint8_t)(crc >> 8);
}

static void threshold_set_overall(const char *text)
{
    uint16_t control = (g_threshold.page == THRESHOLD_SCREEN_SET) ? 5U : 20U;
    if(g_threshold.page == THRESHOLD_SCREEN_SET || g_threshold.page == THRESHOLD_SCREEN_QUERY)
        SetTextValue(g_threshold.page, control, (uint8_t *)text);
}

static void threshold_query_set_loop_text(void)
{
    const uint8_t *text = (g_threshold.loop == 3U) ? g_text_loop3 : g_text_loop1;
    SetTextValue(THRESHOLD_SCREEN_QUERY, 2U, (uint8_t *)text);
}

static void threshold_setting_set_mode_text(void)
{
    const uint8_t *text = g_setting_text_single;
    if(g_threshold.mode == THRESHOLD_MODE_LOOP) text = g_setting_text_loop_batch;
    else if(g_threshold.mode == THRESHOLD_MODE_NATIONAL) text = g_setting_text_type_batch;
    SetTextValue(THRESHOLD_SCREEN_SET, 2U, (uint8_t *)text);
}

static void threshold_setting_set_loop_text(void)
{
    const uint8_t *text = (g_threshold.loop == 3U) ? g_text_loop3 : g_text_loop1;
    SetTextValue(THRESHOLD_SCREEN_SET, 4U, (uint8_t *)text);
}

static void threshold_setting_clear_target_input(void)
{
    SetTextValue(THRESHOLD_SCREEN_SET, 7U, (uint8_t *)"");
}

static void threshold_query_clear_identity(void)
{
    SetTextValue(THRESHOLD_SCREEN_QUERY, 8U, (uint8_t *)"");
    SetTextValue(THRESHOLD_SCREEN_QUERY, 27U, (uint8_t *)"");
    SetTextValue(THRESHOLD_SCREEN_QUERY, 34U, (uint8_t *)"");
}

static void threshold_query_clear_target_input(void)
{
    SetTextValue(THRESHOLD_SCREEN_QUERY, 6U, (uint8_t *)"");
}

static uint8_t threshold_has_result(void)
{
    uint8_t item;
    for(item = 0U; item < THRESHOLD_ITEM_COUNT; item++)
        if(g_threshold.results[item] != THRESHOLD_RESULT_IDLE) return 1U;
    return 0U;
}

static uint8_t threshold_item_supported(const DeviceThresholdIdentity *identity, uint8_t item)
{
    if(identity == NULL || item >= THRESHOLD_ITEM_COUNT) return 0U;
    if((identity->sensor_enable & (uint16_t)(1U << g_threshold_defs[item].sensor_bit)) == 0U) return 0U;
    if(item == 6U && identity->device_type == RS485_DETECT_TYPE_XR805) return 0U;
    return 1U;
}

static void threshold_cancel(void)
{
    g_threshold.running = 0U;
    g_threshold.waiting = 0U;
    g_threshold.operation = THRESHOLD_OP_NONE;
    g_threshold.restore_armed = 0U;
    g_threshold.generation++;
}

static void threshold_clear_results(void)
{
    memset(g_threshold.values, 0, sizeof(g_threshold.values));
    memset(g_threshold.results, 0, sizeof(g_threshold.results));
    g_threshold.success_count = 0U;
    g_threshold.failure_count = 0U;
    g_threshold.ui_dirty = 1U;
}

static uint8_t threshold_collect_targets(void)
{
    uint8_t address;
    DeviceThresholdIdentity identity;
    g_threshold.target_count = 0U;

    if(g_threshold.loop != 3U) return 0U;
    if(g_threshold.mode == THRESHOLD_MODE_SINGLE)
    {
        if(g_threshold.address_or_code == 0U || g_threshold.address_or_code >= RS485_DETECT_MAX_DEVICES) return 0U;
        if(DeviceThreshold_GetLoop3Identity((uint8_t)g_threshold.address_or_code, &identity) == 0U ||
           identity.online == 0U || identity.identified == 0U) return 0U;
        g_threshold.targets[0] = (uint8_t)g_threshold.address_or_code;
        g_threshold.target_count = 1U;
        return 1U;
    }

    for(address = 1U; address < RS485_DETECT_MAX_DEVICES && g_threshold.target_count < THRESHOLD_MAX_TARGETS; address++)
    {
        if(DeviceThreshold_GetLoop3Identity(address, &identity) == 0U || identity.online == 0U || identity.identified == 0U) continue;
        if(g_threshold.mode == THRESHOLD_MODE_NATIONAL && identity.national_code != g_threshold.address_or_code) continue;
        g_threshold.targets[g_threshold.target_count++] = address;
    }
    return (g_threshold.target_count != 0U);
}

static uint8_t threshold_validate_requested(void)
{
    uint8_t index;
    uint8_t any = 0U;
    uint16_t co_low;
    uint16_t co_high;
    uint16_t h2_low;
    uint16_t h2_high;
    for(index = 0U; index < THRESHOLD_ITEM_COUNT; index++)
    {
        if(g_threshold.requested_valid[index] != 0U)
        {
            any = 1U;
            if(g_threshold.requested[index] > g_threshold_defs[index].maximum) return 0U;
        }
    }
    if((g_threshold.requested_valid[2] != 0U) != (g_threshold.requested_valid[3] != 0U) &&
       (g_threshold.mode != THRESHOLD_MODE_SINGLE || g_threshold.results[2] != THRESHOLD_RESULT_OK ||
        g_threshold.results[3] != THRESHOLD_RESULT_OK)) return 3U;
    if((g_threshold.requested_valid[4] != 0U) != (g_threshold.requested_valid[5] != 0U) &&
       (g_threshold.mode != THRESHOLD_MODE_SINGLE || g_threshold.results[4] != THRESHOLD_RESULT_OK ||
        g_threshold.results[5] != THRESHOLD_RESULT_OK)) return 3U;
    co_low = g_threshold.requested_valid[2] != 0U ? g_threshold.requested[2] : g_threshold.values[2];
    co_high = g_threshold.requested_valid[3] != 0U ? g_threshold.requested[3] : g_threshold.values[3];
    h2_low = g_threshold.requested_valid[4] != 0U ? g_threshold.requested[4] : g_threshold.values[4];
    h2_high = g_threshold.requested_valid[5] != 0U ? g_threshold.requested[5] : g_threshold.values[5];
    if((g_threshold.requested_valid[2] != 0U || g_threshold.requested_valid[3] != 0U) && co_high <= co_low) return 2U;
    if((g_threshold.requested_valid[4] != 0U || g_threshold.requested_valid[5] != 0U) && h2_high <= h2_low) return 2U;
    return any;
}

static void threshold_start(uint8_t operation)
{
    threshold_cancel();
    threshold_clear_results();
    if(g_threshold.page == THRESHOLD_SCREEN_QUERY) g_threshold.clear_display = 1U;
    if(threshold_collect_targets() == 0U)
    {
        if(g_threshold.page == THRESHOLD_SCREEN_QUERY)
            threshold_set_overall((const char *)g_query_text_no_target);
        else
            threshold_set_overall((const char *)g_setting_text_no_target);
        return;
    }
    g_threshold.operation = operation;
    g_threshold.running = 1U;
    g_threshold.target_index = 0U;
    g_threshold.item_index = 0U;
    g_threshold.phase = (operation == THRESHOLD_OP_QUERY) ? THRESHOLD_PHASE_READ : THRESHOLD_PHASE_WRITE;
    g_threshold.ui_dirty = 1U;
    if(g_threshold.page == THRESHOLD_SCREEN_QUERY)
        threshold_set_overall((const char *)g_query_text_querying);
    else if(operation == THRESHOLD_OP_QUERY)
        threshold_set_overall((const char *)g_setting_text_reading);
    else if(operation == THRESHOLD_OP_DEFAULT)
        threshold_set_overall((const char *)g_setting_text_restoring);
    else
        threshold_set_overall((const char *)g_setting_text_writing);
}

static uint8_t threshold_advance_to_supported(void)
{
    DeviceThresholdIdentity identity;
    while(g_threshold.target_index < g_threshold.target_count)
    {
        uint8_t address = g_threshold.targets[g_threshold.target_index];
        if(DeviceThreshold_GetLoop3Identity(address, &identity) == 0U || identity.online == 0U || identity.identified == 0U)
        {
            g_threshold.failure_count++;
            g_threshold.target_index++;
            g_threshold.item_index = 0U;
            continue;
        }
        while(g_threshold.item_index < THRESHOLD_ITEM_COUNT)
        {
            uint8_t selected = (g_threshold.operation == THRESHOLD_OP_QUERY || g_threshold.operation == THRESHOLD_OP_DEFAULT) ? 1U : g_threshold.requested_valid[g_threshold.item_index];
            if(selected != 0U && threshold_item_supported(&identity, g_threshold.item_index) != 0U) return 1U;
            if(g_threshold.target_count == 1U && selected != 0U) g_threshold.results[g_threshold.item_index] = THRESHOLD_RESULT_UNSUPPORTED;
            g_threshold.item_index++;
        }
        g_threshold.target_index++;
        g_threshold.item_index = 0U;
    }
    g_threshold.running = 0U;
    g_threshold.operation = THRESHOLD_OP_NONE;
    g_threshold.ui_dirty = 1U;
    return 0U;
}

void DeviceThreshold_Init(void)
{
    memset(&g_threshold, 0, sizeof(g_threshold));
    g_threshold.loop = 3U;
}

void DeviceThreshold_NotifyScreen(uint16_t screen_id)
{
    if(screen_id != THRESHOLD_SCREEN_SET && screen_id != THRESHOLD_SCREEN_QUERY)
    {
        if(g_threshold.page == THRESHOLD_SCREEN_SET || g_threshold.page == THRESHOLD_SCREEN_QUERY) threshold_cancel();
        g_threshold.page = 0U;
        return;
    }
    threshold_cancel();
    threshold_clear_results();
    g_threshold.page = (uint8_t)screen_id;
    g_threshold.mode = THRESHOLD_MODE_SINGLE;
    g_threshold.loop = 1U;
    g_threshold.address_or_code = 0U;
    g_threshold.target_count = 0U;
    g_threshold.clear_display = 1U;
    memset(g_threshold.requested_valid, 0, sizeof(g_threshold.requested_valid));
    if(screen_id == THRESHOLD_SCREEN_QUERY)
    {
        threshold_query_set_loop_text();
        threshold_query_clear_target_input();
        threshold_query_clear_identity();
        threshold_set_overall((const char *)g_query_text_input_prompt);
    }
    else
    {
        threshold_setting_set_mode_text();
        threshold_setting_set_loop_text();
        threshold_setting_clear_target_input();
        threshold_set_overall((const char *)g_setting_text_initial_prompt);
    }
}

void DeviceThreshold_NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item, uint8_t state)
{
    if(state == 0U) return;
    if(screen_id == THRESHOLD_SCREEN_SET && control_id == 350U)
    {
        threshold_cancel(); threshold_clear_results();
        g_threshold.clear_display = 1U;
        memset(g_threshold.requested_valid, 0, sizeof(g_threshold.requested_valid));
        g_threshold.mode = (item <= THRESHOLD_MODE_NATIONAL) ? item : THRESHOLD_MODE_SINGLE;
        g_threshold.address_or_code = 0U;
        threshold_setting_set_mode_text();
    }
    else if(screen_id == THRESHOLD_SCREEN_SET && control_id == 351U)
    {
        threshold_cancel(); threshold_clear_results();
        g_threshold.clear_display = 1U;
        memset(g_threshold.requested_valid, 0, sizeof(g_threshold.requested_valid));
        g_threshold.loop = (item == 0U) ? 1U : 3U;
        g_threshold.address_or_code = 0U;
        threshold_setting_set_loop_text();
    }
    else if(screen_id == THRESHOLD_SCREEN_QUERY && control_id == 350U)
    {
        threshold_cancel(); threshold_clear_results();
        g_threshold.clear_display = 1U;
        g_threshold.loop = (item == 0U) ? 1U : 3U;
        g_threshold.address_or_code = 0U;
        g_threshold.target_count = 0U;
        threshold_query_set_loop_text();
        threshold_query_clear_target_input();
        threshold_query_clear_identity();
        threshold_set_overall((const char *)g_query_text_input_prompt);
    }
}

void DeviceThreshold_NotifyText(uint16_t screen_id, uint16_t control_id, const uint8_t *text)
{
    uint32_t value = 0U;
    uint8_t item;
    if((screen_id != THRESHOLD_SCREEN_SET && screen_id != THRESHOLD_SCREEN_QUERY) || text == NULL) return;
    if(sscanf((const char *)text, "%lu", &value) != 1) return;
    if(value > 0xFFFFUL) value = 0xFFFFUL;
    threshold_cancel();
    if(control_id == 7U || (screen_id == THRESHOLD_SCREEN_QUERY && control_id == 6U))
    {
        if(screen_id == THRESHOLD_SCREEN_SET)
            memset(g_threshold.requested_valid, 0, sizeof(g_threshold.requested_valid));
        g_threshold.address_or_code = (uint16_t)value;
        if(screen_id == THRESHOLD_SCREEN_QUERY) g_threshold.target_count = 0U;
        threshold_clear_results();
        g_threshold.clear_display = 1U;
        if(screen_id == THRESHOLD_SCREEN_SET && g_threshold.mode != THRESHOLD_MODE_LOOP) threshold_start(THRESHOLD_OP_QUERY);
        else if(screen_id == THRESHOLD_SCREEN_QUERY)
            threshold_set_overall((const char *)g_query_text_target_selected);
        return;
    }
    if(screen_id != THRESHOLD_SCREEN_SET) return;
    for(item = 0U; item < THRESHOLD_ITEM_COUNT; item++)
    {
        if(control_id == g_threshold_defs[item].value_control)
        {
            g_threshold.requested[item] = (uint16_t)value;
            g_threshold.requested_valid[item] = 1U;
            g_threshold.restore_armed = 0U;
            return;
        }
    }
}

void DeviceThreshold_NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
    uint8_t valid;
    if(state == 0U) return;
    if(screen_id == THRESHOLD_SCREEN_QUERY && control_id == 301U)
    {
        threshold_start(THRESHOLD_OP_QUERY);
        return;
    }
    if(screen_id != THRESHOLD_SCREEN_SET) return;
    if(control_id == 302U)
    {
        valid = threshold_validate_requested();
        if(valid == 2U) SetTextValue(THRESHOLD_SCREEN_SET, 5U, (uint8_t *)g_text_high_must_exceed_low);
        else if(valid == 3U) threshold_set_overall((const char *)g_setting_text_need_high_low);
        else if(valid == 0U) threshold_set_overall((const char *)g_setting_text_invalid_input);
        else threshold_start(THRESHOLD_OP_WRITE);
    }
    else if(control_id == 305U)
    {
        if(g_threshold.restore_armed == 0U)
        {
            g_threshold.restore_armed = 1U;
            g_threshold.restore_tick = osKernelGetTickCount();
            threshold_set_overall((const char *)g_setting_text_restore_confirm);
        }
        else if((osKernelGetTickCount() - g_threshold.restore_tick) <= THRESHOLD_CONFIRM_MS)
        {
            g_threshold.restore_armed = 0U;
            threshold_start(THRESHOLD_OP_DEFAULT);
        }
    }
}

uint8_t DeviceThreshold_BuildNextFrame(uint8_t frame[8], uint8_t *address)
{
    const ThresholdDefinition *definition;
    uint16_t value;
    if(frame == NULL || address == NULL || g_threshold.running == 0U || g_threshold.waiting != 0U) return 0U;
    if(g_threshold.skip_normal_polls != 0U) return 0U;
    if(threshold_advance_to_supported() == 0U) return 0U;

    definition = &g_threshold_defs[g_threshold.item_index];
    *address = g_threshold.targets[g_threshold.target_index];
    frame[0] = *address;
    if(g_threshold.operation == THRESHOLD_OP_QUERY || g_threshold.phase == THRESHOLD_PHASE_VERIFY)
    {
        frame[1] = 0x03U;
        frame[2] = (uint8_t)(definition->reg >> 8);
        frame[3] = (uint8_t)definition->reg;
        frame[4] = 0U; frame[5] = 1U;
    }
    else
    {
        value = (g_threshold.operation == THRESHOLD_OP_DEFAULT) ? definition->default_value : g_threshold.requested[g_threshold.item_index];
        frame[1] = 0x06U;
        frame[2] = (uint8_t)(definition->reg >> 8);
        frame[3] = (uint8_t)definition->reg;
        frame[4] = (uint8_t)(value >> 8);
        frame[5] = (uint8_t)value;
    }
    threshold_crc(frame);
    g_threshold.waiting = 1U;
    g_threshold.transaction_generation = g_threshold.generation;
    g_threshold.transaction_address = *address;
    g_threshold.transaction_item = g_threshold.item_index;
    g_threshold.transaction_function = frame[1];
    if(g_threshold.target_count == 1U) g_threshold.results[g_threshold.item_index] = THRESHOLD_RESULT_WAIT;
    g_threshold.ui_dirty = 1U;
    return 1U;
}

static void threshold_finish_item(uint8_t success, uint16_t value)
{
    uint8_t item = g_threshold.transaction_item;
    g_threshold.waiting = 0U;
    g_threshold.skip_normal_polls = THRESHOLD_NORMAL_POLLS_BETWEEN;
    g_threshold.verify_retry_count = 0U;
    if(success != 0U)
    {
        g_threshold.values[item] = value;
        g_threshold.results[item] = THRESHOLD_RESULT_OK;
        g_threshold.success_count++;
    }
    else
    {
        g_threshold.results[item] = THRESHOLD_RESULT_FAIL;
        g_threshold.failure_count++;
    }
    g_threshold.item_index++;
    g_threshold.phase = (g_threshold.operation == THRESHOLD_OP_QUERY) ? THRESHOLD_PHASE_READ : THRESHOLD_PHASE_WRITE;
    g_threshold.ui_dirty = 1U;
}

/* A 0x06 reply is only an acknowledgement. Some field devices finish the
 * register write but do not return the echo before the common UART timeout,
 * so the final result is decided by the following 0x03 read-back instead. */
static void threshold_begin_verify(void)
{
    g_threshold.waiting = 0U;
    g_threshold.phase = THRESHOLD_PHASE_VERIFY;
    g_threshold.verify_retry_count = 0U;
    g_threshold.skip_normal_polls = THRESHOLD_NORMAL_POLLS_BETWEEN;
    g_threshold.ui_dirty = 1U;
}

/* Keep retries bounded and retain normal polls between attempts so threshold
 * configuration cannot monopolize UART5 or delay device alarm polling. */
static void threshold_retry_verify_or_finish(void)
{
    if(g_threshold.verify_retry_count < THRESHOLD_VERIFY_RETRY_LIMIT)
    {
        g_threshold.waiting = 0U;
        g_threshold.verify_retry_count++;
        g_threshold.skip_normal_polls = THRESHOLD_NORMAL_POLLS_BETWEEN;
        g_threshold.ui_dirty = 1U;
    }
    else
    {
        threshold_finish_item(0U, 0U);
    }
}

uint8_t DeviceThreshold_HandleResponse(const uint8_t *frame, uint16_t length)
{
    uint16_t value;
    uint16_t expected;
    if(frame == NULL || length < 5U) return 0U;
    /* A page/fire cancellation invalidates the UI job but the UART owner must
     * still consume the already transmitted response and release its lock. */
    if(g_threshold.waiting == 0U)
        return (frame[0] == g_threshold.transaction_address) ? 1U : 0U;
    if(frame[0] != g_threshold.transaction_address) return 0U;
    if(g_threshold.transaction_generation != g_threshold.generation) { g_threshold.waiting = 0U; return 1U; }
    if(frame[1] == (uint8_t)(g_threshold.transaction_function | 0x80U))
    {
        if(g_threshold.transaction_function == 0x03U &&
           g_threshold.operation != THRESHOLD_OP_QUERY &&
           g_threshold.phase == THRESHOLD_PHASE_VERIFY)
            threshold_retry_verify_or_finish();
        else
            threshold_finish_item(0U, 0U);
        return 1U;
    }
    if(frame[1] != g_threshold.transaction_function) return 0U;

    if(frame[1] == 0x06U)
    {
        expected = (g_threshold.operation == THRESHOLD_OP_DEFAULT) ? g_threshold_defs[g_threshold.transaction_item].default_value : g_threshold.requested[g_threshold.transaction_item];
        if(length == 8U &&
           frame[2] == (uint8_t)(g_threshold_defs[g_threshold.transaction_item].reg >> 8) &&
           frame[3] == (uint8_t)g_threshold_defs[g_threshold.transaction_item].reg &&
           frame[4] == (uint8_t)(expected >> 8) && frame[5] == (uint8_t)expected)
        {
            /* The exact echo acknowledges the command; the final success
             * condition is still the following 0x03 read-back. */
            threshold_begin_verify();
            return 1U;
        }
        /* A malformed echo is inconclusive. Keep the UART transaction until
         * its bounded timeout, then confirm the actual register value. */
        return 0U;
    }
    if(frame[1] == 0x03U && length == 7U && frame[2] == 2U)
    {
        value = (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);
        if(g_threshold.operation == THRESHOLD_OP_QUERY) threshold_finish_item(1U, value);
        else
        {
            expected = (g_threshold.operation == THRESHOLD_OP_DEFAULT) ? g_threshold_defs[g_threshold.transaction_item].default_value : g_threshold.requested[g_threshold.transaction_item];
            if(value == expected) threshold_finish_item(1U, value);
            else threshold_retry_verify_or_finish();
        }
        return 1U;
    }
    if(g_threshold.operation != THRESHOLD_OP_QUERY && g_threshold.phase == THRESHOLD_PHASE_VERIFY)
        threshold_retry_verify_or_finish();
    else
        threshold_finish_item(0U, 0U);
    return 1U;
}

void DeviceThreshold_HandleTimeout(void)
{
    if(g_threshold.waiting == 0U) return;
    if(g_threshold.transaction_function == 0x06U &&
       g_threshold.operation != THRESHOLD_OP_QUERY &&
       g_threshold.phase == THRESHOLD_PHASE_WRITE)
        threshold_begin_verify();
    else if(g_threshold.transaction_function == 0x03U &&
            g_threshold.operation != THRESHOLD_OP_QUERY &&
            g_threshold.phase == THRESHOLD_PHASE_VERIFY)
        threshold_retry_verify_or_finish();
    else
        threshold_finish_item(0U, 0U);
}

void DeviceThreshold_NotifyNormalPoll(void)
{
    if(g_threshold.skip_normal_polls != 0U) g_threshold.skip_normal_polls--;
}

void DeviceThreshold_UpdateUI(uint16_t screen_id, uint8_t fire_active)
{
    uint8_t item;
    uint8_t status;
    uint8_t cancelled_by_fire = 0U;
    uint16_t status_control;
    if(fire_active != 0U && g_threshold.running != 0U)
    {
        threshold_cancel();
        cancelled_by_fire = 1U;
        if(g_threshold.page == THRESHOLD_SCREEN_QUERY)
            threshold_set_overall((const char *)g_query_text_fire_cancelled);
        else
            threshold_set_overall((const char *)g_setting_text_fire_cancelled);
    }
    if(g_threshold.restore_armed != 0U && (osKernelGetTickCount() - g_threshold.restore_tick) > THRESHOLD_CONFIRM_MS)
    {
        g_threshold.restore_armed = 0U;
        if(screen_id == THRESHOLD_SCREEN_SET)
            threshold_set_overall((const char *)g_setting_text_confirm_timeout);
    }
    if(screen_id != g_threshold.page || g_threshold.ui_dirty == 0U) return;
    g_threshold.ui_dirty = 0U;

    if(g_threshold.clear_display != 0U)
    {
        g_threshold.clear_display = 0U;
        for(item = 0U; item < THRESHOLD_ITEM_COUNT; item++)
        {
            SetTextValue(screen_id, g_threshold_defs[item].value_control, (uint8_t *)"");
            if(item != 3U && item != 5U)
                SetTextValue(screen_id, g_threshold_defs[item].status_control, (uint8_t *)"");
        }
        if(screen_id == THRESHOLD_SCREEN_QUERY) threshold_query_clear_identity();
    }

    if(screen_id == THRESHOLD_SCREEN_QUERY && g_threshold.target_count == 1U)
    {
        DeviceThresholdIdentity identity;
        if(DeviceThreshold_GetLoop3Identity(g_threshold.targets[0], &identity) != 0U)
        {
            SetTextValue(screen_id, 8U, (uint8_t *)DeviceRegistry_GetName(identity.product_code));
            SetTextInt32(screen_id, 27U, identity.national_code, 0U, 1U);
            SetTextValue(screen_id, 34U, (uint8_t *)(identity.online != 0U ? g_query_text_online : g_query_text_offline));
        }
    }

    for(item = 0U; item < THRESHOLD_ITEM_COUNT; item++)
    {
        if(g_threshold.results[item] == THRESHOLD_RESULT_OK)
            SetTextInt32(screen_id, g_threshold_defs[item].value_control, g_threshold.values[item], 0U, 1U);
        else if(screen_id == THRESHOLD_SCREEN_SET &&
                g_threshold.results[item] == THRESHOLD_RESULT_UNSUPPORTED)
            SetTextValue(screen_id, g_threshold_defs[item].value_control, (uint8_t *)"----");
    }
    /* CO and H2 low/high pairs share one status control. */
    for(item = 0U; item < THRESHOLD_ITEM_COUNT; item++)
    {
        status_control = g_threshold_defs[item].status_control;
        status = g_threshold.results[item];
        if(item == 3U || item == 5U) continue;
        if(item == 2U && g_threshold.results[3] > status) status = g_threshold.results[3];
        if(item == 4U && g_threshold.results[5] > status) status = g_threshold.results[5];
        if(screen_id == THRESHOLD_SCREEN_QUERY)
        {
            if(status == THRESHOLD_RESULT_OK) SetTextValue(screen_id, status_control, (uint8_t *)g_query_text_success);
            else if(status == THRESHOLD_RESULT_WAIT) SetTextValue(screen_id, status_control, (uint8_t *)g_query_text_wait);
            else if(status == THRESHOLD_RESULT_FAIL) SetTextValue(screen_id, status_control, (uint8_t *)g_query_text_fail);
            else if(status == THRESHOLD_RESULT_UNSUPPORTED) SetTextValue(screen_id, status_control, (uint8_t *)g_text_unsupported);
        }
        else
        {
            if(status == THRESHOLD_RESULT_OK) SetTextValue(screen_id, status_control, (uint8_t *)g_setting_text_success);
            else if(status == THRESHOLD_RESULT_WAIT) SetTextValue(screen_id, status_control, (uint8_t *)g_setting_text_wait);
            else if(status == THRESHOLD_RESULT_FAIL) SetTextValue(screen_id, status_control, (uint8_t *)g_setting_text_fail);
            else if(status == THRESHOLD_RESULT_UNSUPPORTED) SetTextValue(screen_id, status_control, (uint8_t *)g_text_unsupported);
        }
    }
    if(cancelled_by_fire == 0U && g_threshold.running == 0U &&
       (g_threshold.success_count != 0U || g_threshold.failure_count != 0U ||
        threshold_has_result() != 0U))
    {
        static char summary[40];
        if(screen_id == THRESHOLD_SCREEN_QUERY)
            (void)snprintf(summary, sizeof(summary), "%s %s:%u %s:%u",
                           (const char *)g_query_text_done,
                           (const char *)g_text_success_count, g_threshold.success_count,
                           (const char *)g_text_fail_count, g_threshold.failure_count);
        else
            (void)snprintf(summary, sizeof(summary), "%s %s:%u %s:%u",
                           (const char *)g_setting_text_done,
                           (const char *)g_text_success_count, g_threshold.success_count,
                           (const char *)g_text_fail_count, g_threshold.failure_count);
        threshold_set_overall(summary);
    }
}
