#ifndef __BSP_ACCESS_CONTROL_H
#define __BSP_ACCESS_CONTROL_H

#include "main.h"

typedef enum
{
    ACCESS_LEVEL_I = 1,       /* 一级：查询、消音、信息确认等基础操作 */
    ACCESS_LEVEL_II,          /* 二级：普通管理员操作，可记忆5分钟 */
    ACCESS_LEVEL_III,         /* 三级：超级管理员设置，仅授权当前页面 */
    ACCESS_LEVEL_FACTORY      /* 厂家级：最高权限，不建立记忆会话 */
} AccessLevel_t;

/* 厂家密码可以通过任意级别的密码校验，但不对外建立持久或跨页权限。 */
#define ACCESS_FACTORY_PASSWORD       686868UL
#define ACCESS_ORDINARY_VALID_MS      300000UL

/* 初始化、校验以及页面授权接口统一由本模块管理。 */
void AccessControl_Init(void);
uint8_t AccessControl_IsGranted(AccessLevel_t level, uint16_t screen_id);
uint8_t AccessControl_IsOrdinarySessionActive(void);
uint8_t AccessControl_Verify(AccessLevel_t level, uint32_t password,
                             uint16_t grant_screen);
uint8_t AccessControl_TransferPageGrant(uint16_t current_screen,
                                        uint16_t target_screen,
                                        AccessLevel_t level);
void AccessControl_OnScreenChanged(uint16_t previous_screen,
                                   uint16_t current_screen);
void AccessControl_ClearOrdinarySession(void);
void AccessControl_ClearPageGrant(void);
AccessLevel_t AccessControl_GetPageGrantLevel(uint16_t screen_id);

#endif
