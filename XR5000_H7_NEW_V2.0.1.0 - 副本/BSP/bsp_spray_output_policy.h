#ifndef __BSP_SPRAY_OUTPUT_POLICY_H
#define __BSP_SPRAY_OUTPUT_POLICY_H

#include <stdint.h>

/*
 * 专用项目喷洒联动策略。
 * 默认关闭，开启时将下面的0U改为1U；关闭时设备码19不接入系统。
 */
#define XR5000_PROJECT_SPRAY_DELAY_ENABLE       0U

#define XR5000_PROJECT_SPRAY_DELAY_MS       30000U
#define XR5000_PROJECT_FCM1011_ADDRESS          1U
#define XR5000_PROJECT_FCM1012_ADDRESS          2U

/* 上电和总复位时清空未完成的倒计时及锁定状态。 */
void SprayOutputPolicy_Init(void);
void SprayOutputPolicy_Reset(void);

/* 回路2解析到设备码19的真实状态边沿后调用。 */
void SprayOutputPolicy_OnButtonState(uint8_t address, uint8_t state);

/* 由回路2任务周期调用；内部仅入队，不阻塞通信轮询。 */
void SprayOutputPolicy_Process(void);

#endif
