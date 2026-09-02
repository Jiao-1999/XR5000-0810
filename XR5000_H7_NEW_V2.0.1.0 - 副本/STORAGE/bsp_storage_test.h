/**
 * @file    bsp_storage_test.h
 * @brief   黑匣子全流程定时触发自动测试模块(测试方案v2 §1.3)
 * @note    仅测试构建使用. STX_TEST_ENABLE=0 时本模块编译内容为空,
 *          TaskLoop 恢复 portMAX_DELAY 原行为, 量产固件与现版本完全一致.
 *          STX_TEST_ENABLE=1 时:
 *            - StorageTest_Init() 在 StorageTx_Init() 尾部调用
 *            - StorageTest_Tick() 在 StorageTx_TaskLoop() 每循环开头调用
 *          阶段表驱动状态机自动注入 反馈/屏蔽/自检/故障/火警/复位 等事件,
 *          阶段标记经 StorageTx_SendTestLog(0x07) 发送, 存储侧USB CDC转发输出,
 *          与存储侧 [STX_WR] 写入日志同口对账.
 */
#ifndef __BSP_STORAGE_TEST_H
#define __BSP_STORAGE_TEST_H
#include <stdint.h>

#define STX_TEST_ENABLE   0    /* 测试构建=1, 量产=0 */
#define STX_TEST_TICK_MS  100  /* 节拍(ms, 与TaskLoop超时一致) */

void StorageTest_Init(void);  /* StorageTx_Init()尾部调用 */
void StorageTest_Tick(void);  /* StorageTxTask每循环调用 */
#endif
