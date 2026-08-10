#ifndef __BSP_PASSWORD_H
#define __BSP_PASSWORD_H

#include <string.h>
#include <stdint.h>

#include "main.h"



/**
 * @brief 验证 10 位序列码 核心验证函数
 * @param input_code: 用户输入的 10 位字符串 (必须以 '\0' 结尾)
 * @param user_name: 当前绑定的用户名 (需预先存储在 STM32 中)
 * @param ship_date: 当前绑定的出货日期字符串 (需预先存储在 STM32 中)
 * @return 1: 验证成功, 0: 失败
 */
int verify_license_code(const char *input_code, const char *user_name, const char *ship_date);

void generate_new_license_code(const char *user_name, const char *ship_date, char *target_code);

// 判断日期是多久
const char* getGenerationDate(uint8_t date);

// 时间映射表
uint16_t getRemainUseDate(uint8_t n);

#endif
