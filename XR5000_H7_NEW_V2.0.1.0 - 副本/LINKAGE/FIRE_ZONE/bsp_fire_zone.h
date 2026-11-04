/*==============================================================
 * 文件名称   : bsp_fire_zone.h
 * 模块功能   : 防火分区管理（头文件）
 * 硬件平台   : STM32H723ZGT6 @ 320MHz, Keil MDK-ARM
 * 模块说明   : 将各回路设备划分到不同防火分区，联动逻辑只能
 *              在同一分区的传感器/动作设备之间设定。
 *              - 设备分区映射: (回路号,设备地址) -> 分区号(1-2)
 *              - 分区名称: 每分区16字节GBK名称
 *              - 分区启用状态(V2新增): 每分区启用/停用存储与显示
 *              - Flash持久化: W25Qxx 0x084000扇区, A/B双副本
 *              - 出厂默认: 仅分区1, 全部设备属于分区1
 *
 * 架构定位   : MODEL层，完全自包含，不依赖设备注册表/联动模块。
 *              分区数据变更通过回调函数通知外部(解耦)。
 *==============================================================*/

#ifndef __BSP_FIRE_ZONE_H
#define __BSP_FIRE_ZONE_H

#include "main.h"    /* 引入STM32基本类型: uint8_t, uint16_t等 */

/*--------------------------------------------------------------
 * 1. 常量定义区
 *--------------------------------------------------------------*/

#define FIRE_ZONE_MAX          2   /* 防火分区数量上限（分区号1-2，内部0-1存储） */

/* 设备分区归属取值: 0=未做分区(不参与任何分区功能), 1=分区1, 2=分区2 */
#define FIRE_ZONE_NONE         0U
#define FIRE_ZONE_NAME_LEN     16  /* 分区名称缓冲区字节数(GBK含结束符, 有效15字节) */

/* Flash 存储区 W25Q256 SPI Flash 芯片 0x084000
 * 该扇区位于联动规则区(0x080000起连续3扇区)与元数据区(0x088000)之间, 空闲不冲突
 * 同一扇区内保存A/B双副本: 前半扇区为副本A, 后半扇区为副本B */
#define FIRE_ZONE_FLASH_ADDR   0x084000U                        /* 分区表存储扇区起始地址(4KB) */
#define FIRE_ZONE_FLASH_COPY_A FIRE_ZONE_FLASH_ADDR             /* 副本A地址(扇区前半) */
#define FIRE_ZONE_FLASH_COPY_B (FIRE_ZONE_FLASH_ADDR + 0x800U)  /* 副本B地址(扇区后半) */
#define FIRE_ZONE_MAGIC        0x46525A33U /* "FRZ3": zone_map 采用 0=未分区/1=分区1/2=分区2 */
#define FIRE_ZONE_VERSION      0x05U        /* 存储格式版本号(V4:2分区+系统级设备, 旧版本记录作废) */

/* 设备标识与地址范围约定(与联动规则校验保持一致):
 *   回路1(MBus探测器)  : 设备地址1-100 (MIXTURE_DEVICE_MAX_ADDR)
 *   回路2(MBus控制回路): 设备地址1-63  (MBUS_CONTROL_MAX_DEVICES-1)
 *   回路3(RS485探测器) : 设备地址1-64  (RS485_DETECT_MAX_DEVICES-1)
 *   回路4(IG3302回路)  : 设备地址1-10  (IG3302_MAX_ADDRESS)
 *   映射表内部统一用设备地址作第二维下标(0-100), 回路号-1作第一维下标 */

/* 分区设备槽位(画面84每分区7类设备栏, 顺序与屏工程一致) */
typedef enum {
    FZ_SLOT_TEMP     = 0,   /* 温度        回路1 */
    FZ_SLOT_SMOKE    = 1,   /* 烟雾        回路1 */
    FZ_SLOT_GAS      = 2,   /* 可燃气体    回路3 */
    FZ_SLOT_COMPOUND = 3,   /* 复合探测器  回路3 */
    FZ_SLOT_MANUAL   = 4,   /* 手报        回路2 */
    FZ_SLOT_SIREN    = 5,   /* 声光        回路2 */
    FZ_SLOT_DISPLAY  = 6,   /* 火灾显示盘  回路2 */
    FZ_SLOT_COUNT
} FireZoneSlot;

/* 系统级设备槽位(画面84底部"系统级设备"区) */
typedef enum {
    FZ_SYS_MANUAL  = 0,     /* 系统级手报 */
    FZ_SYS_SIREN   = 1,     /* 系统级声光 */
    FZ_SYS_DISPLAY = 2,     /* 系统级火灾显示盘 */
    FZ_SYS_COUNT
} FireZoneSysDev;

/*--------------------------------------------------------------
 * 2. 回调函数类型定义 - 分区变更通知
 *    由外部模块(如屏幕层)实现并注册，本模块在分区归属或
 *    分区名称发生变化并持久化后调用，用于驱动界面刷新。
 *--------------------------------------------------------------*/

typedef void (*FireZoneChangedCb_t)(void);

/*--------------------------------------------------------------
 * 3. API声明 - 初始化与回调注册
 *--------------------------------------------------------------*/

void FireZone_Init(void);                                /* 模块初始化, 从Flash加载分区表(失败回默认) */
void FireZone_SetChangedCallback(FireZoneChangedCb_t cb); /* 注册分区变更通知回调 */

/*--------------------------------------------------------------
 * 4. API声明 - 设备分区归属查询与设置
 *    返回值约定：0=成功, 2=参数错误
 *--------------------------------------------------------------*/

uint8_t FireZone_GetDeviceZone(uint8_t loop_no, uint16_t dev_no);                    /* 查询设备所属分区号(1-2, 参数非法返回0) */
uint8_t FireZone_SetDeviceZone(uint8_t loop_no, uint16_t dev_no, uint8_t zone_no);   /* 设置设备所属分区(设置后立即持久化) */
uint8_t FireZone_IsSameZone(uint8_t loop1, uint16_t dev1, uint8_t loop2, uint16_t dev2); /* 1=同区(含参数非法)/0=跨区 */

/*--------------------------------------------------------------
 * 5. API声明 - 分区设备枚举(按回路1→2→3、地址升序)
 *--------------------------------------------------------------*/

uint8_t FireZone_GetZoneDeviceCount(uint8_t zone_no);   /* 分区内地址总数(参数非法返回0) */
uint8_t FireZone_GetZoneDeviceAt(uint8_t zone_no, uint8_t index, uint8_t *loop_no, uint16_t *dev_no); /* 取分区内第index个设备(0基), 1=成功 0=越界 */

/*--------------------------------------------------------------
 * 6. API声明 - 分区数量与名称
 *    返回值约定：0=成功, 2=参数错误
 *--------------------------------------------------------------*/

uint8_t     FireZone_GetZoneCount(void);                            /* 当前分区总数(非空最高分区号, 至少1) */
const char* FireZone_GetZoneName(uint8_t zone_no);                  /* 获取分区名称GBK串(参数非法返回空串) */
uint8_t     FireZone_SetZoneName(uint8_t zone_no, const char *name); /* 设置分区名称(GBK, 超15字节截断, 设置后立即持久化) */

/*--------------------------------------------------------------
 * 7. API声明 - 分区启用状态(V2新增)
 *    说明: 启用状态仅存储与显示, 不参与联动求值;
 *    出厂默认: 两个分区全部启用; 旧版本记录加载即作废(回出厂默认)
 *--------------------------------------------------------------*/

uint8_t FireZone_GetZoneEnabled(uint8_t zone_no);             /* 查询分区启用状态(1=启用 0=停用, 参数非法返回1) */
void    FireZone_SetZoneEnabled(uint8_t zone_no, uint8_t on); /* 设置分区启用状态(非0视为启用, 设置后持久化) */
uint8_t FireZone_ToggleZoneEnabled(uint8_t zone_no);          /* 翻转分区启用状态(返回翻转后的状态) */

/*--------------------------------------------------------------
 * 8. API声明 - 批量设备范围写(V2新增)
 *    说明: 一次用户输入=一次Flash擦写; 单次范围写内部已保证
 *    只落盘一次, 常规用法无需配对调用Begin/EndBatch
 *    返回值约定：0=成功, 2=参数错误
 *--------------------------------------------------------------*/

void    FireZone_BeginBatch(void);   /* 挂起自动持久化(可嵌套) */
void    FireZone_EndBatch(void);     /* 恢复自动持久化(挂起期有改动则统一落盘) */

/*--------------------------------------------------------------
 * 9. API声明 - 槽位显示文本持久化(V3新增)
 *    设备范围栏"只以屏输入为准": 输入生成的规范化显示文本
 *    随分区表一并存Flash, 上电自动恢复显示(免重设)。
 *    slot: FZ_SLOT_*(0~6)对应屏幕7类设备栏, text为GBK显示串(<=15字节)。
 *--------------------------------------------------------------*/
uint8_t FireZone_SetSlotDisp(uint8_t zone_no, uint8_t slot, const char *text);
void    FireZone_GetSlotDisp(uint8_t zone_no, uint8_t slot, char *out);

/*--------------------------------------------------------------
 * 9.1 API声明 - 分区显示盘与系统级设备(V4新增)
 *    画面84底部"系统级设备"区设备与各分区关联的火灾显示盘,
 *    均位于回路2(地址1-63), 地址0表示未配置。
 *    返回值约定：0=成功, 2=参数错误
 *--------------------------------------------------------------*/

/* 分区专用：本分区关联的火灾显示盘地址(回路2, 1-63; 0=未配置) */
uint8_t FireZone_SetZoneDisplayAddr(uint8_t zone_no, uint8_t addr);
uint8_t FireZone_GetZoneDisplayAddr(uint8_t zone_no);
/* 系统级设备地址(FireZoneSysDev, 回路2, 1-63; 0=未配置) */
uint8_t FireZone_SetSysDeviceAddr(uint8_t kind, uint8_t addr);
uint8_t FireZone_GetSysDeviceAddr(uint8_t kind);
/* 系统级设备显示文本(原样保存用户输入) */
uint8_t FireZone_SetSysDisp(uint8_t kind, const char *text);
void    FireZone_GetSysDisp(uint8_t kind, char *out);

/*--------------------------------------------------------------
 * 10. API声明 - 出厂复位(V3新增)
 *    清空全部分区数据(映射/名称/启用状态/显示文本),
 *    恢复出厂默认并立即写Flash。用于调试或强制初始化。
 *--------------------------------------------------------------*/
void FireZone_ResetAll(void);
uint8_t FireZone_SetZoneRange(uint8_t zone_no, uint8_t loop_no, uint8_t start, uint8_t end); /* 将回路地址段划入分区(独占语义,一次落盘) */
uint8_t FireZone_ClearZoneRange(uint8_t zone_no, uint8_t loop_no); /* 清空分区在指定回路的设备(归位分区1) */

#endif /* __BSP_FIRE_ZONE_H */
