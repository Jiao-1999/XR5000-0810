/**
 * @file    bsp_storage_event.h
 * @brief   黑匣子存储事件接入层 - 为业务代码提供统一的火警/故障/反馈记录接口
 *
 * @details
 *   本模块是对 bsp_storage_tx 底层API的封装, 供 cmd_process.c 等业务代码调用.
 *   业务代码只需调用 StorageEvent_LogFire/LogFault/LogFeedback 一行,
 *   由本模块自动完成: 记录构造、时间戳填充、首警判定、命令码选择、异步入队.
 *
 *   设计目标:
 *     1. 文件独立 - 接入逻辑集中在本文件, 不污染业务代码
 *     2. 接入清晰 - 业务点只需一行调用, 参数语义明确
 *     3. 易于维护 - 接入点清单见 bsp_storage_event.c 顶部注释
 *
 *   依赖:
 *     bsp_storage_tx.h - 底层发送/队列API、EventRecord_t、命令码、事件代码
 *
 *   命令码分配(存储侧独立区段):
 *     0x01 普通事件(可覆盖) - 反馈等
 *     0x02 首警(独立区段)   - 仅首次火警
 *     0x03 火警(独立区段)   - 每次火警
 *     0x04 故障(独立区段)   - 故障发生/恢复
 */
#ifndef __BSP_STORAGE_EVENT_H
#define __BSP_STORAGE_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_storage_tx.h"

/*==============================================================
 * API声明
 *============================================================*/

/**
 * @brief  记录火警事件到黑匣子(自动判定首警)
 * @param  dev_no:      设备号(手报ID/舱号/探测器地址等)
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx, 见 bsp_storage_tx.h)
 * @param  unit_no:     单元号(通常为1)
 * @param  channel_no:  通道号(通常为0)
 * @note   首次调用自动先发0x02(首警)再发0x03(火警); 之后只发0x03.
 *         异步入队, 非阻塞. 首警标志需在系统复位时通过
 *         StorageEvent_ResetFirstFire() 清除.
 */
void StorageEvent_LogFire(uint8_t dev_no, uint16_t dev_type,
                          uint8_t unit_no, uint8_t channel_no);

/**
 * @brief  记录故障事件到黑匣子(含故障恢复)
 * @param  dev_no:      设备号
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx)
 * @param  unit_no:     单元号(通常为1)
 * @param  channel_no:  通道号(通常为0)
 * @param  is_recover:  0=故障发生(EVT_FAULT) 1=故障恢复(EVT_FAULT_RECOVER)
 * @note   使用0x04命令码存入故障独立区段, 不被普通事件覆盖.
 *         异步入队, 非阻塞.
 */
void StorageEvent_LogFault(uint8_t dev_no, uint16_t dev_type,
                           uint8_t unit_no, uint8_t channel_no,
                           uint8_t is_recover);

/**
 * @brief  记录反馈事件到黑匣子(普通区段)
 * @param  dev_no:      设备号
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx)
 * @param  state_code:  状态代码(反馈状态值)
 * @note   使用0x01命令码存入普通区段(先进先出覆盖).
 *         异步入队, 非阻塞.
 */
void StorageEvent_LogFeedback(uint8_t dev_no, uint16_t dev_type,
                              uint16_t state_code);

/**
 * @brief  复位首警标志
 * @note   在 BspCmdProcessInit() 末尾调用, 使下次火警重新判定首警.
 */
void StorageEvent_ResetFirstFire(void);

/**
 * @brief  记录手动/自动切换事件到黑匣子(普通区段)
 * @param  dev_no:     设备号(SYS_HAND_AUTO_Package_ID / PART1_/PART2_HAND_AUTO_Package_ID)
 * @param  is_manual:  1=手动(EVT_MANUAL) 0=自动(EVT_AUTO)
 * @note   使用0x01命令码存入普通区段. dev_type固定为DEV_TYPE_CONTROL_DEV.
 *         异步入队, 非阻塞.
 */
void StorageEvent_LogManualAuto(uint8_t dev_no, uint8_t is_manual);

/**
 * @brief  记录屏蔽/解除屏蔽事件到黑匣子(普通区段)
 * @param  dev_no:      设备号(探测器地址)
 * @param  dev_type:    设备类型代码(DEV_TYPE_xxx, 由调用方根据DeviceRegistryType映射)
 * @param  is_release:  0=屏蔽(EVT_SHIELD) 1=解除屏蔽(EVT_SHIELD_RELEASE)
 * @note   使用0x01命令码存入普通区段. 异步入队, 非阻塞.
 *         设备类型映射(由调用方完成):
 *           DEVICE_TYPE_SMOKE        → DEV_TYPE_SMOKE(21)
 *           DEVICE_TYPE_TEMPERATURE  → DEV_TYPE_TEMPERATURE(31)
 *           DEVICE_TYPE_MULTI_SENSOR → DEV_TYPE_FIRE_ALARM(82, 复合探测器暂用)
 */
void StorageEvent_LogShield(uint8_t dev_no, uint16_t dev_type, uint8_t is_release);

/**
 * @brief  记录启动事件到黑匣子(普通区段, 联动启动按键)
 * @param  dev_no:    设备号(LINKAGE_CLUSTER_ID)
 * @param  dev_type:  设备类型代码(DEV_TYPE_CONTROL_DEV)
 * @note   使用0x01命令码存入普通区段. 异步入队, 非阻塞.
 */
void StorageEvent_LogStart(uint8_t dev_no, uint16_t dev_type);

/**
 * @brief  记录系统复位事件到黑匣子(普通区段)
 * @note   在 BspCmdProcessInit() 末尾调用, 紧跟 StorageEvent_ResetFirstFire().
 *         dev_no=1, dev_type=DEV_TYPE_CONTROLLER, event=EVT_NORMAL, state=0.
 *         使用0x01命令码存入普通区段. 异步入队, 非阻塞.
 *         注意: 复位时StorageTx可能未就绪, 入队失败可接受.
 */
void StorageEvent_LogReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STORAGE_EVENT_H */
