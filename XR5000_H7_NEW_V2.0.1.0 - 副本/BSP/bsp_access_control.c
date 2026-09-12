#include "bsp_access_control.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "system.h"

static volatile uint8_t g_ordinary_session_valid;
static volatile uint32_t g_ordinary_session_tick;
static volatile uint16_t g_page_grant_screen;
static volatile AccessLevel_t g_page_grant_level;

/* 普通管理员权限只在验证后的5分钟内有效，厂家和超级密码不进入该会话。 */
static uint8_t AccessControl_OrdinarySessionValid(void)
{
    if(g_ordinary_session_valid == 0U)
    {
        return 0U;
    }

    if((uint32_t)(osKernelGetTickCount() - g_ordinary_session_tick) >=
       pdMS_TO_TICKS(ACCESS_ORDINARY_VALID_MS))
    {
        g_ordinary_session_valid = 0U;
        return 0U;
    }

    return 1U;
}

void AccessControl_Init(void)
{
    g_ordinary_session_valid = 0U;
    g_ordinary_session_tick = 0U;
    g_page_grant_screen = 0U;
    g_page_grant_level = ACCESS_LEVEL_I;
}

uint8_t AccessControl_IsGranted(AccessLevel_t level, uint16_t screen_id)
{
    /* 一级操作不需要密码。 */
    if(level == ACCESS_LEVEL_I)
    {
        return 1U;
    }

    /* 超级或厂家密码只授权进入时指定的页面，离开页面后自动清除。 */
    if(g_page_grant_screen == screen_id && g_page_grant_level >= level)
    {
        return 1U;
    }

    if(level == ACCESS_LEVEL_II)
    {
        return AccessControl_OrdinarySessionValid();
    }

    return 0U;
}

uint8_t AccessControl_IsOrdinarySessionActive(void)
{
    return AccessControl_OrdinarySessionValid();
}

uint8_t AccessControl_Verify(AccessLevel_t level, uint32_t password,
                             uint16_t grant_screen)
{
    uint8_t valid = 0U;

    /* 厂家密码是最高权限，可用于任何密码输入位置，但不建立5分钟会话。 */
    if(password == ACCESS_FACTORY_PASSWORD)
    {
        valid = 1U;
    }
    else if(level == ACCESS_LEVEL_II)
    {
        if(password == SystemSaveInfo.user_password)
        {
            g_ordinary_session_tick = osKernelGetTickCount();
            g_ordinary_session_valid = 1U;
            valid = 1U;
        }
        else if(password == SystemSaveInfo.super_admin_password)
        {
            valid = 1U;
        }
    }
    else if(level == ACCESS_LEVEL_III)
    {
        valid = (password == SystemSaveInfo.super_admin_password) ? 1U : 0U;
    }
    /* 普通密码进入二级权限时只建立会话；其他有效密码按目标页面授权。 */
    if(valid != 0U && grant_screen != 0U &&
       (level != ACCESS_LEVEL_II ||
        password != SystemSaveInfo.user_password))
    {
        g_page_grant_screen = grant_screen;
        /* 保留实际授权身份，供密码管理页区分超级管理员与厂家维护。 */
        g_page_grant_level = (password == ACCESS_FACTORY_PASSWORD) ?
                             ACCESS_LEVEL_FACTORY : level;
    }

    return valid;
}

uint8_t AccessControl_TransferPageGrant(uint16_t current_screen,
                                        uint16_t target_screen,
                                        AccessLevel_t level)
{
    /* 仅允许把已经取得的单页授权转交给紧邻的目标设置页面。 */
    if(g_page_grant_screen != current_screen ||
       g_page_grant_level < level || target_screen == 0U)
    {
        return 0U;
    }

    g_page_grant_screen = target_screen;
    return 1U;
}

void AccessControl_OnScreenChanged(uint16_t previous_screen,
                                   uint16_t current_screen)
{
    /* 离开授权页面后立即撤销，确保超级和厂家密码没有跨页记忆。 */
    if(g_page_grant_screen != 0U && previous_screen == g_page_grant_screen &&
       current_screen != g_page_grant_screen)
    {
        AccessControl_ClearPageGrant();
    }
}

void AccessControl_ClearOrdinarySession(void)
{
    g_ordinary_session_valid = 0U;
    g_ordinary_session_tick = 0U;
}

void AccessControl_ClearPageGrant(void)
{
    g_page_grant_screen = 0U;
    g_page_grant_level = ACCESS_LEVEL_I;
}

AccessLevel_t AccessControl_GetPageGrantLevel(uint16_t screen_id)
{
    if(g_page_grant_screen == screen_id)
    {
        return g_page_grant_level;
    }

    return ACCESS_LEVEL_I;
}
