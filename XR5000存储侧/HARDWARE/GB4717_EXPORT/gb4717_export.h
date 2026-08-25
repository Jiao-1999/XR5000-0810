/**
 * @file    gb4717_export.h
 * @brief   GB4717-2024附录B 火灾报警控制器记录导出协议
 * @details 通过USB CDC虚拟串口(PA11-D-/PA12-D+)向PC导出存储侧记录,
 *          底层收发依赖USB_CDC模块.
 *
 *   请求帧(PC->存储侧):
 *     [0x40][设备ID 8字节][版本1][地址1][类型1][命令长度1][命令数据n][CRC16_2][0x40]
 *   响应帧(存储侧->PC):
 *     [0x40][记录总数3][控制器地址1][控制器类型2][产品编号20][事件记录n*17][CRC16_2][0x40]
 *   命令码:
 *     1=顺序读记录, 2=重发, 3=读首警(event_code=2), 4=读火警(event_code=3)
 *     5=读故障(P1-5整改新增, 分区直读)
 *   请求帧校验(P0-3/P1-4整改): 设备识别码须与授权Token一致,
 *     版本号=0x02/地址=0x7E/类型=0x7F/CRC16逐字段校验, 不符静默丢帧
 *   CRC16: MODBUS多项式0xA001, 低字节在前
 */
#ifndef __GB4717_EXPORT_H
#define __GB4717_EXPORT_H

#include "sys.h"
#include "bsp_storage_rx.h"

/**
 * @brief  初始化GB4717导出模块 (复位接收状态机和各读取游标)
 */
void GB4717_ExportInit(void);

/**
 * @brief  GB4717导出主循环处理 (在main的while(1)中调用)
 * @note   从USB CDC读取字节按状态机解析请求帧, 命令完整后执行响应
 */
void GB4717_ExportProcess(void);

#endif /* __GB4717_EXPORT_H */
