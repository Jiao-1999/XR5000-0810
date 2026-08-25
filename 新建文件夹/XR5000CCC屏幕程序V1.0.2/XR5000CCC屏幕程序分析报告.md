# XR5000CCC 屏幕程序 V1.0.2 分析报告
## 以及与 XR5000_H7 固件的关联对照

---

## 一、项目概述

### 1.1 基本信息
- **项目名称**: XR5000CCC 屏幕程序 V1.0.2
- **开发工具**: 大彩 VisualTFT（广州大彩光电科技有限公司）
- **屏幕类型**: 大彩串口触摸屏（1024×600分辨率）
- **通信协议**: 大彩自定义二进制协议（0xEE帧头 + 数据 + CRC16 + 0xFFFCFFFF帧尾）
- **通信串口**: STM32端的UART8
- **项目架构**: 纯图形界面工程，无Lua脚本逻辑——所有业务逻辑在STM32端处理

### 1.2 项目结构
```
XR5000CCC屏幕程序V1.0.2/
├── *.tft                    # 屏幕画面定义文件（XML格式）
├── main.lua                 # Lua脚本模板（空壳，无实际逻辑）
├── images/                  # 图片资源（背景图、按钮图标、界面元素）
├── font/                    # 字体资源（GBK/GB2312/ASCII多尺寸点阵字库）
├── videos/                  # 视频资源（开机视频001.mp4）
├── output/                  # 编译输出（controls.xml控件汇总）
├── dciot_build/             # 构建中间文件
└── XR5000PRO屏幕程序20簇PPM版本/  # 20簇PPM变体版本（并行子项目）
```

---

## 二、两版本对比：根版本 vs 20簇PPM版本

### 2.1 根版本（V1.0.2主版本）
- **簇监控数量**: 第1至第20簇（簇级监控界面从1到20）
- **柜级监控**: 第一柜级、第二柜级、第三柜级（合3柜）
- **功能特点**: 
  - 支持CABIN（舱级）和PACK（电池包级）监测
  - 复合火警逻辑设定
  - 设备上下线管理（簇级+舱级+包级）
  - 设备屏蔽管理（簇级+舱级）
  - 灭火启动选择（单装置/双装置）
  - 报警记录查询和清除

### 2.2 20簇PPM版本
- **额外特点**: PPM浓度单位显示（百万分之一），可能用于更高精度的气体浓度显示
- **文件**: 包含与根版本相同的大部分屏幕文件，但在`XR5000PRO屏幕程序20簇PPM版本/`子目录下
- **局限性**: 缺少部分根版本独有的屏幕（如CABIN监测界面、PACK监测界面等）

---

## 三、所有屏幕清单及控件映射

### 3.1 主界面 (screen_id ≈ 1)

| 控件ID | 类型 | 名称 | STM32引用位置 |
|--------|------|------|-------------|
| 1 | Button | 菜单选择 | switch→菜单选择页面 |
| 2 | RTC | 时间显示 | InternalScreenRTCSetting(screen=1, ctrl=32) |
| 4 | TextDisplay | 报警信息1 | cmd_process.c UpdateUI |
| 5 | TextDisplay | 数值显示位1 | - |
| 6 | TextDisplay | 数值显示位2 | - |
| 7 | TextDisplay | 数值显示位3 | - |
| 8 | TextDisplay | 数值显示位4 | - |
| 9 | TextDisplay | 主电状态 | PowerStateUpdataUI(ctrl=9) ← "主电供电"/"主电异常" |
| 10 | Button | - | - |
| 11 | TextDisplay | 主机名称 | NotifyScreen → SystemSaveInfo.zhujiming |
| 12 | TextDisplay | 报警信息2 | UpdateUI |
| 13 | Text | "进风机:" | 静态标签 |
| 14 | Text | "排风机:" | 静态标签 |
| 15 | Text | "灭火装置1:" | 静态标签 |
| 16 | TextDisplay | 电池电压 | PowerStateUpdataUI(ctrl=16) ← BattrayVoltage/100.0 |
| 17 | Text | "灭火装置2:" | 静态标签 |
| 18 | TextDisplay | 进风机状态 | FansStateUpdataUI(ctrl=18) ← "自动/手动/工作/故障/掉线" |
| 19 | TextDisplay | 排风机状态 | FansStateUpdataUI(ctrl=19) |
| 20 | Text | "EMS485ID:" | 静态标签 |
| 21 | TextDisplay | 灭火装置1状态 | OutfirePressureUpdataUI(ctrl=21) ← "正常/故障" |
| 22 | TextDisplay | CAN1 ID显示 | - |
| 23 | TextDisplay | 灭火装置2状态 | OutfirePressureUpdataUI(ctrl=23) |
| 24 | Text | "CAN2ID:" | 静态标签 |
| 25 | TextDisplay | CAN2 ID显示 | - |
| 26 | TextDisplay | - | - |
| 27 | Button | 公司信息 | switch→公司信息 |
| 28 | TextDisplay | 调试信息1 | - |
| 29 | TextDisplay | 调试信息2 | - |
| 30 | TextDisplay | - | - |
| 31 | Button | 簇/仓状态 | - |
| 32 | Button | 密码入口 | switch→二级密码界面 (InternalScreenRTCSetting) |
| 33 | TextDisplay | 备电状态 | PowerStateUpdataUI(ctrl=33) ← "备电充电/放电/断路/短路/欠压" |
| 34 | TextDisplay | 电量百分比 | PowerStateUpdataUI(ctrl=34) ← capacity_percentage |
| 35 | Button | 簇/仓状态2 | - |
| 36 | Text | "场站485ID:" | 静态标签 |
| 37 | Button | 状态监测入口 | switch→状态监测界面 |
| 38 | Button | 状态监测入口2 | switch→状态监测界面 |
| 3 | TextDisplay | 报警信息3 | UpdateUI |

### 3.2 二级密码界面 (screen_id ≈ 2)
- **功能**: 输入6位密码后进入设置页面
- **控件**: 
  - ctrl 2: TextDisplay (密码输入框, max_len=6)
  - ctrl 3: Button "取消" → 返回主界面
  - ctrl 4: Button "确认" → switch→设置界面
- **固件关联**: bsp_super.c SuperAdminPasswordButtonCtrl() 验证密码"68686668"

### 3.3 灭火启动选择 (用于簇级手动灭火)
- **20个按钮**: ctrl 21-40 对应第01-20簇启动按钮
- **ctrl 41**: "返回主界面"按钮
- **固件关联**: cmd_process.c NotifyButton 中处理该画面按钮 → 触发UART_zhongduan状态机

### 3.4 簇监控界面 (第1~第20簇)
- **结构**: 顶层目录有 `第n簇监控界面.tft` (n=1-20)
- **20簇PPM版本子目录**: 同样有1-20簇监控界面
- **通用模板**: `第n簇监控界面.tft` — 模板文件，通过索引切换内容
- **每个簇界面包含**: 传感器类型显示、状态指示灯、温度数值、气体浓度等
- **固件关联**: 
  - UpdateUI() 中根据 `current_screen_id` 刷新 cu_sxzt[]、cang_sxzt[]、cu_tcq_sxzt[]
  - 控件ID映射: PCAC_zxwz_buf[]（在线状态）、PCAC_wdwz_buf[]（温度）、PCAC_ywtb_buf[]（烟雾）等

### 3.5 柜级监控界面 (第一~第四柜级)
- **结构**: 第一至第四柜级监控（4个界面）
- **功能**: 按机柜维度汇总显示各簇状态
- **固件关联**: UpdateUI() 中屏幕15-26的簇/仓在线状态更新

### 3.6 告警查询系统
- `告警查询.tft`: 报警记录列表查看
- `报警显示界面.tft`: 单条报警详细信息
- `查询分类界面.tft`: 按类型分类查询
- `清除报警记录.tft`: 报警记录清除确认
- **固件关联**: 
  - LoadSensor(page) 读取Flash中的报警记录
  - SaveSensor() 存储报警到W25Q128
  - AlarmsSaveInfor 结构体映射到显示控件
  - 报警类型33种（手报/烟感/温感/气溶胶/BMS高温/压力异常/掉线...）

### 3.7 设备管理系列
- `设备上下线.tft` / `设备上下线舱级.tft`: 传感器在线/离线设置
- `设备屏蔽簇级.tft` / `设备屏蔽舱级.tft`: 传感器屏蔽管理
- `仓包上下线管理界面.tft` / `包上下线管理界面.tft`: 电池包管理
- `设备上线管理点型仓.tft`: 点型仓传感器管理
- **固件关联**: 
  - Cang_zx_buf[] / PACK_zx_buf[] / CU_zx_buf[] 在线状态数组
  - SaveCabinOnlineState() / SaveClusterOnlineState() 保存到Flash
  - xMyRs485QueueHandle 发送屏蔽/取消屏蔽Modbus指令

### 3.8 设置系统
- `设置界面.tft`: 主设置页面 → 子功能入口
- `设置用户密码.tft`: 修改登录密码
- `进入设置密码.tft`: 密码输入验证
- `超管密码页.tft`: 超级管理员密码（8次按压5秒内检测）
- `超级管理员界面.tft`: 超级管理员功能面板
- **固件关联**: 
  - SuperAdminButtonCtrl() 8次按压检测（screen 27, ctrl 8）
  - SuperAdminPasswordButtonCtrl() 验证"68686668"
  - FireAlarmThresholdSettingInternalScreenText() 设置报警阈值

### 3.9 逻辑设定系统
- `逻辑设定选择界面.tft`: 逻辑类型选择
- `喷放逻辑设定.tft`: 灭火装置喷放参数设置（延时/次数/间隔/对应关系）
- `喷房逻辑设定帮助界面.tft`: 帮助说明
- `逻辑查看界面.tft`: 已配置逻辑查看
- **固件关联**: 
  - FireAlarmTriggerLogicButtonSet() 处理逻辑按键（数字/类型/与/或/确认/取消）
  - FireAlarmTriggerLogicUpdataUI() 刷新逻辑显示文本
  - FireAlarmJudgeBuffExtract() 解析逻辑字符串→判定数组
  - FireAlarmCompoundLogicJudgement() 执行复合火警判定
  - out_fire_start_ctrl 灭火装置控制结构体

### 3.10 报警阈值设置
- `报警阈值设置界面.tft`: H2/CO/温度阈值
- `报警阈值设置界面2.tft`: 压力/烟雾/甲烷/丙烷阈值
- **固件关联**: 
  - FireAlarmThresholdSettingInternalScreenText() 接收阈值设置
  - 保存到24C04 EEPROM（SystemSaveInfo结构体）
  - 温度预警1=68°C, 预警2=78°C（默认）

### 3.11 巡检系统
- `巡检弹窗.tft`: 巡检进度弹窗
- `按键巡检界面.tft`: 按键/反馈设备巡检
- **固件关联**: InternalScreenBoradRecvDealTask() 处理巡检命令

### 3.12 其他屏幕
- `开机视频.tft`: 启动画面（显示开机视频）
- `公司信息.tft`: 公司/设备信息显示
- `手动自动切换.tft`: 手动/自动模式切换
- `型号设定界面.tft`: 传感器型号配置
- `时间配置界面.tft`: 系统时间设置
- `出场日期规格设置界面.tft`: 出厂日期配置
- `设备许可证界面.tft`: 设备许可证信息
- `故障显示界面.tft`: 故障汇总显示
- `模拟串口助手界面.tft`: 调试用串口助手
- `系统复位弹窗.tft`: 系统软复位确认
- `监控选择界面.tft`: 监控维度选择（簇/柜/舱）
- `CABIN监测界面.tft` / `CABIN显示界面.tft`: 舱级监控
- `PACK监测界面.tft` / `PACK显示界面.tft`: 电池包监控
- `仓复合监测界面.tft`: 多类型传感器复合监测
- `探测器状态显示.tft`: 探测器状态指示灯
- `状态监测界面.tft`: 在"主界面→Button37/38"中被引用，但未见对应.tft文件

---

## 四、STM32固件 ↔ 屏幕程序 通信对照表

### 4.1 画面切换对照

| STM32中引用 | 推测screen_id | 屏幕文件 | 功能说明 |
|------------|--------------|---------|---------|
| 画面1 | 1 | 主界面.tft | 主界面 |
| 画面2 | 2 | (设置相关) | 系统设置入口 |
| 画面15-16 | 15-16 | (CABIN/PACK监测) | 传感器在线/状态显示 |
| 画面17 | 17 | 簇监控(模板) | 簇在线状态(cu_sxzt[]) |
| 画面18 | 18 | 仓监控(模板) | 仓在线状态(cang_sxzt[]) |
| 画面22 | 22 | 报警显示界面.tft | 实时报警信息轮播 |
| 画面23 | 23 | 报警显示界面.tft | 密码设置/报警确认 |
| 画面24 | 24 | 清除报警记录.tft | 记录清除确认 |
| 画面25 | 25 | 簇监控界面 | 簇在线状态(清零操作) |
| 画面26 | 26 | 包屏蔽界面 | 电池包屏蔽状态(cang_pbzt[]) |
| 画面27 | 27 | 超管密码页.tft | 超级管理员入口(8次按压) |
| 画面39 | 39 | (联动监控) | 12路联动设备屏蔽/启用 |
| 画面40 | 40 | 喷放逻辑设定.tft | 灭火装置参数设置 |
| 画面43 | 43 | 逻辑设定选择界面.tft | 火警触发逻辑输入 |
| 画面45 | 45 | 逻辑查看界面.tft | 已配置逻辑查看 |
| 画面46 | 46 | 超级管理员界面.tft | 管理员功能菜单 |
| 画面47-48 | 47-48 | 报警阈值设置界面.tft | 报警阈值设定 |
| 画面51 | 51 | 进入设置密码.tft | 管理员密码验证 |

### 4.2 关键控件ID对照（固件→屏幕）

| 固件代码引用 | screen_id | control_id | 控件类型 | 含义 |
|------------|-----------|-----------|---------|------|
| SetTextValue(1,11, zhujiming) | 1 | 11 | TextDisplay | 主机名称显示 |
| SetTextValue(1,9, 主电状态) | 1 | 9 | TextDisplay | 主电供电/主电异常 |
| SetTextValue(1,33, 备电状态) | 1 | 33 | TextDisplay | 备电充电/放电/断路/短路 |
| SetTextValue(1,18, 风机1) | 1 | 18 | TextDisplay | 进风机状态 |
| SetTextValue(1,19, 风机2) | 1 | 19 | TextDisplay | 排风机状态 |
| SetTextValue(1,21, 灭火装置1) | 1 | 21 | TextDisplay | 灭火装置1状态 |
| SetTextValue(1,23, 灭火装置2) | 1 | 23 | TextDisplay | 灭火装置2状态 |
| SetTextValue(1,34, 电量) | 1 | 34 | TextDisplay | 备电百分比 |
| SetTextValue(1,16, 电压) | 1 | 16 | TextDisplay | 电池电压 |
| 8次按压检测 | 27 | 8 | Button | 超管入口 |
| 灭火装置参数 | 40 | 7/22/39/40/48/49/55/58/61等 | Menu/Text | 喷放配置 |
| 火警逻辑输入 | 43 | 1-9/10/11-21 | Button | 数字键/类型键/逻辑键 |

### 4.3 导航关系图（从.tft文件提取）

```
主界面
├─ Button1 → 菜单选择页面
├─ Button27 → 公司信息
├─ Button32 → 二级密码界面
│   ├─ ctrl3 "取消" → 主界面
│   └─ ctrl4 "确认" → 设置界面
│       ├─ 报警阈值设置
│       ├─ 用户密码设置
│       ├─ 设备管理
│       ├─ 灭火启动选择
│       ├─ 逻辑设定
│       ├─ 时间配置
│       └─ 出厂日期规格设置
├─ Button37/38 → 状态监测界面
└─ 超管入口(8次按压画面27) → 超级管理员界面
    ├─ 设备许可证
    ├─ 型号设定
    ├─ 模拟串口助手
    └─ 系统复位
```

---

## 五、协议层对照

### 5.1 大彩屏幕协议在STM32固件中的实现

```
STM32端:                             屏幕端:
──────────────────────────────────────────────────
HMI_DRIVE/hmi_driver.c               (固件内置)
  ├─ BEGIN_CMD() → 发送0xEE           → 屏幕识别帧头
  ├─ 各种Set/Get指令                   → 控件读写
  ├─ END_CMD() → 发送CRC16+0xFFFCFFFF  → 屏幕识别帧尾
  └─ SEND_DATA() → SendChar()         → 通过UART8发送

HMI_DRIVE/cmd_queue.c
  └─ queue_push(byte)                 → 接收屏幕返回数据
     └─ 32位移位寄存器检测0xFFFCFFFF   → 帧尾匹配
        └─ queue_find_cmd() → 提取帧

HMI_DRIVE/cmd_process.c
  └─ ProcessMessage()                 → 解析屏幕主动上报的通知
     ├─ NOTIFY_TOUCH_PRESS/RELEASE    → 触摸事件
     ├─ NOTIFY_CONTROL                → 按钮点击(kCtrlButton)
     │   └─ MSG_GET_CURRENT_SCREEN    → 画面切换通知
     ├─ NOTIFY_READ_RTC               → RTC读取
     └─ NOTIFY_WRITE/READ_FLASH       → Flash操作结果
```

### 5.2 固件主动调用的屏幕API

| HMI API函数 | 功能 | 使用场景 |
|-----------|------|---------|
| SetTextValue(screen, ctrl, str) | 设置文本控件文字 | 状态更新、报警显示 |
| SetTextInt32(screen, ctrl, val, sign, digits) | 设置整数显示 | 温度、浓度数值 |
| setkey_Value(screen, ctrl, state) | 设置按钮状态(图标切换) | 在线/掉线/屏蔽状态指示 |
| SetScreen(screen_id) | 切换画面 | 自动跳转、菜单导航 |
| GetScreen() | 查询当前画面 | 启动时获取当前画面 |
| clearTextValue(screen, ctrl) | 清除文本控件 | 清除旧数据 |
| SetControlForeColor(screen, ctrl, color) | 设置前景色 | 报警状态颜色变化 |
| set_RTC() + 屏幕RTC控件刷新 | 设置屏幕RTC | 时间同步 |

---

## 六、数据流向完整对照

### 6.1 传感器数据 → 屏幕显示

```
XR805/电池包传感器
  → RS485/M-BUS轮询
  → Cang_*_buf[] / PACK_*_buf[] 数据数组
  → UpdateUI() / InterScreenFreshTask (300ms周期)
  → 根据 current_screen_id 选择性刷新:
    - 画面1: 主电/备电/风机/压力/报警轮播
    - 画面17: 簇在线状态 setkey_Value()
    - 画面18: 仓在线状态 setkey_Value()
    - 画面22: 多报警轮播(8条,3秒切换)
    - 画面26: 包屏蔽状态
  → 通过UART8发送Dacai协议指令到屏幕
```

### 6.2 屏幕操作 → 硬件动作

```
用户触摸屏幕按钮
  → 屏幕通过UART8上报 NOTIFY_CONTROL + kCtrlButton
  → cmd_queue环形缓冲 → queue_find_cmd()帧提取
  → ProcessMessage() → NotifyButton(screen_id, control_id, state)
  → 根据screen_id/control_id分发:
    ├─ 屏蔽操作 → xQueueSend(xMyRs485QueueHandle) → RS485发送
    ├─ 灭火启动 → UART_zhongduan状态机 → 继电器动作
    ├─ 参数设置 → 修改全局变量/结构体 → SaveSystemInfo() → EEPROM/Flash
    ├─ 密码验证 → SuperAdminPasswordButtonCtrl()
    ├─ 逻辑设置 → FireAlarmTriggerLogicButtonSet()
    ├─ 阈值设置 → FireAlarmThresholdSettingInternalScreenText()
    └─ 时间设置 → sscanf解析 → set_RTC() + BM8563_Soft_I2C_SetTime()
```

---

## 七、关键发现与差异

### 7.1 屏幕程序"零逻辑"架构
与STM32固件中拥有庞大logic的`cmd_process.c`（6000+行）形成对比，屏幕端的`main.lua`完全是空壳模板。这意味着：
- 屏幕仅负责图形渲染和触摸事件上报
- **所有业务逻辑、状态管理、判定算法全部在STM32端**
- 屏幕就是一个"哑终端"，降低了屏幕端的开发复杂度
- 画面跳转逻辑也在屏幕工程XML中配置（`switch`属性），而非Lua脚本

### 7.2 版本差异
- 根目录版本更完整，包含CABIN/PACK/仓复合等监测界面
- "20簇PPM版本"子目录专注于20簇+ppm浓度单位的场景，缺少数个特殊界面
- 两个版本的字体和图片资源基本相同

### 7.3 控件ID规律
- 每个画面的控件ID从1开始独立编号
- 固件通过`screen_id + control_id`组合唯一定位控件
- 画面之间导航通过Button的`switch`属性实现，值为目标画面的名称

### 7.4 未匹配的引用
- 固件代码`cmd_process.c`中引用的 `状态监测界面` (Button37/38的switch目标) 在.tft文件列表中未找到对应文件，可能是在VisualTFT工程内的某个未导出屏幕，或者是运行时被替换的画面

---

## 八、资源清单

### 8.1 字体资源（font/normal/）
| 字体文件 | 尺寸 | 用途 |
|---------|------|------|
| ASCII_6_12.bin | 6×12 | 小号英文/数字 |
| ASCII_8_16.bin | 8×16 | 中号英文/数字 |
| ASCII_12_24.bin | 12×24 | 大号英文/数字 |
| ASCII_16_32.bin | 16×32 | 特大号英文/数字 |
| ASCII_32_64.bin | 32×64 | 超大号英文/数字 |
| GBK_12_12.bin | 12×12 | 小号中文 |
| GBK_16_16.bin | 16×16 | 中号中文 |
| GBK_24_24.bin | 24×24 | 大号中文 |
| GB2312_32_32.bin | 32×32 | 特大号中文 |
| GB2312_64_64.bin | 64×64 | 超大号中文 |
| PINYIN.bin | - | 拼音输入法 |

### 8.2 图片资源（关键图标）
| 文件 | 用途 |
|------|------|
| on.png / off.png | 开关状态指示 |
| 按钮红0.png / 按钮红1.png | 红色按钮（报警/停止） |
| 按钮绿0.png / 按钮绿1.png | 绿色按钮（正常/启动） |
| 按钮灰1.png / 按钮灰停止.png / 按钮灰启动.png | 灰色按钮（禁用状态） |
| 启动按钮.ICON / 停止按钮.ICON | 启动/停止开关图标 |
| 上线下线开关.ICON | 在线/离线切换开关 |
| 探测器状态指示灯.ICON | 传感器状态指示图标 |
| 报警阈值*.jpg | 阈值设置背景图 |
| 设备上下线舱级.jpg | 设备管理背景图 |
| 逻辑设定界面.jpg | 逻辑设定背景图 |
| 29*.jpg | 设备监控界面背景图 |
| 复位弹窗.jpg | 复位确认弹窗背景 |
| 进入设置密码.jpg | 密码输入背景 |
| 密码设置.jpg | 密码设置背景 |

### 8.3 视频资源
- `001.mp4`: 开机启动视频（约14MB），在`开机视频.tft`画面中播放

---

## 九、总结

这个屏幕程序是XR5000消防报警控制器的前端显示部分，与STM32固件通过UART8使用大彩协议通信。屏幕本身不包含业务逻辑，所有控制决策由STM32端完成。两个版本（根版本和20簇PPM版本）分别适用于不同的探测器配置场景。

完整的系统交互链路为：

```
传感器(物理层) → XR805/M-BUS/RS485 → STM32固件(采集+判断+逻辑)
  ←→ UART8(大彩协议) ←→ 屏幕程序(显示+触控)
                                  ↑
                            本分析对象
```
