/**
 * @file    bsp_fecbus_report.h
 * @brief   FECbus 事件上报接入层 - 为业务代码提供统一的 FECbus 上报接口
 *
 * @details
 *   本模块是对 bsp_fecbus 底层 API 的封装, 供 cmd_process.c 等业务代码调用.
 *   业务代码只需调用 FecbusReport_Fire/Fault/Feedback 等一行,
 *   由本模块自动完成: 事件项构造、功能码与优先级映射、异步入队.
 *
 *   设计目标:
 *     1. 文件独立 - 接入逻辑集中在本文件, 不污染业务代码
 *     2. 接入清晰 - 业务点只需一行调用, 参数语义与 StorageEvent_Log* 对齐
 *     3. 易于维护 - 接入点清单与 bsp_storage_event.c 一致 (21个接入点)
 *
 *   依赖:
 *     bsp_fecbus.h - 底层发送/队列 API、FecbusEventItem_t、功能码常量
 *     bsp_storage_tx.h - DEV_TYPE_xxx / EVT_xxx 设备类型与事件代码常量
 *
 *   功能码与优先级映射:
 *     火警/首警/启动/反馈/手自动切换 -> 功能码5 (紧急事件, PA=1)
 *     故障/故障恢复/屏蔽/解除屏蔽  -> 功能码6 (一般事件, PA=3)
 *     系统复位                     -> 功能码1 (广播, PA=1)
 *     系统消音                     -> 功能码2 (广播, PA=1)
 *     系统自检                     -> 功能码3 (广播, PA=1)
 *
 *   接入点清单 (与 bsp_storage_event.c 21个接入点完全一致):
 *     火警: cmd_process.c L2861/L5916/L11889/L11914 (4点)
 *     故障: cmd_process.c L11880/L11897/L11904/L11905/L11922/L11927 (6点)
 *     反馈: cmd_process.c L7147 (1点)
 *     复位: cmd_process.c BspCmdProcessInit 末尾 (1点)
 *     手自动: bsp_internal_board.c L705/L714/L727/L736/L749/L758 (6点)
 *     启动: bsp_internal_board.c L694 (1点)
 *     屏蔽: bsp_device_disable.c DeviceDisableSet/Clear (2点)
 *     消音/自检: 控制器按键事件 (本接入层提供 API, 接入点由业务决定)
 */
#ifndef __BSP_FECBUS_REPORT_H
#define __BSP_FECBUS_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_storage_tx.h"  /* DEV_TYPE_xxx / EVT_xxx 常量复用 */

/*==============================================================
 * API 声明
 *============================================================*/

/**
 * @brief  上报火警事件 (功能码5, 紧急, PA=1)
 * @param  dev_no:      设备号(手报ID/舱号/探测器地址等)
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx)
 * @param  unit_no:     单元号(通常为1)
 * @param  channel_no:  通道号(通常为0)
 * @note   异步入队, 非阻塞. 首警与火警均用 EVT_FIRE(3), 由设备侧按时间序判定首警.
 *         (与黑匣子 StorageEvent_LogFire 的首警判定解耦, 这里只负责 FECbus 上报)
 */
void FecbusReport_Fire(uint8_t dev_no, uint16_t dev_type,
                       uint8_t unit_no, uint8_t channel_no);

/**
 * @brief  上报故障事件 (功能码6, 一般, PA=3)
 * @param  dev_no:      设备号
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx)
 * @param  unit_no:     单元号(通常为1)
 * @param  channel_no:  通道号(通常为0)
 * @param  is_recover:  0=故障发生(EVT_FAULT) 1=故障恢复(EVT_FAULT_RECOVER)
 * @note   异步入队, 非阻塞.
 */
void FecbusReport_Fault(uint8_t dev_no, uint16_t dev_type,
                        uint8_t unit_no, uint8_t channel_no,
                        uint8_t is_recover);

/**
 * @brief  上报反馈事件 (功能码5, 紧急, PA=1)
 * @param  dev_no:      设备号
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx)
 * @param  state_code:  状态代码(反馈状态值)
 * @note   异步入队, 非阻塞. unit_no=1, channel_no=0 固定.
 */
void FecbusReport_Feedback(uint8_t dev_no, uint16_t dev_type,
                            uint16_t state_code);

/**
 * @brief  上报手动/自动切换事件 (功能码5, 紧急, PA=1)
 * @param  dev_no:     设备号(SYS_HAND_AUTO_Package_ID / PART1_/PART2_HAND_AUTO_Package_ID)
 * @param  is_manual:  1=手动(EVT_MANUAL) 0=自动(EVT_AUTO)
 * @note   异步入队, 非阻塞. dev_type 固定为 DEV_TYPE_CONTROL_DEV.
 */
void FecbusReport_ManualAuto(uint8_t dev_no, uint8_t is_manual);

/**
 * @brief  上报屏蔽/解除屏蔽事件 (功能码6, 一般, PA=3)
 * @param  dev_no:      设备号(探测器地址)
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx)
 * @param  is_release:  0=屏蔽(EVT_SHIELD) 1=解除屏蔽(EVT_SHIELD_RELEASE)
 * @note   异步入队, 非阻塞.
 */
void FecbusReport_Shield(uint8_t dev_no, uint16_t dev_type, uint8_t is_release);

/**
 * @brief  上报启动事件 (功能码5, 紧急, PA=1)
 * @param  dev_no:    设备号(LINKAGE_CLUSTER_ID)
 * @param  dev_type:  设备类型代码(DEV_TYPE_CONTROL_DEV)
 * @note   异步入队, 非阻塞. 事件代码 EVT_START.
 */
void FecbusReport_Start(uint8_t dev_no, uint16_t dev_type);

/**
 * @brief  上报系统复位事件 (功能码1, 广播, PA=1)
 * @note   异步入队, 非阻塞. dev_no=1, dev_type=DEV_TYPE_CONTROLLER,
 *         event=EVT_NORMAL, state=0.
 *         注意: 复位时 Fecbus 可能未就绪, 入队失败可接受.
 */
void FecbusReport_Reset(void);

/**
 * @brief  上报系统消音事件 (功能码2, 广播, PA=1)
 * @note   异步入队, 非阻塞. dev_no=1, dev_type=DEV_TYPE_CONTROLLER,
 *         event=EVT_NORMAL, state=0.
 */
void FecbusReport_Silence(void);

/**
 * @brief  上报系统自检事件 (功能码3, 广播, PA=1)
 * @note   异步入队, 非阻塞. dev_no=1, dev_type=DEV_TYPE_CONTROLLER,
 *         event=EVT_NORMAL, state=0.
 */
void FecbusReport_SelfCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FECBUS_REPORT_H */
