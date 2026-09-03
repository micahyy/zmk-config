# DZ17 NRF52840 ZMK 双模数字小键盘 — 功能规范

## 硬件参数

| 项目 | 规格 |
|:----:|:----:|
| 主控 | nRF52840（贴片模块） |
| 按键 | 17键，直连GPIO |
| RGB | 17颗WS2812，每键一颗，SPIM3驱动（P0.26） |
| 充电管理 | TP4056，STBY状态检测（P0.31） |
| 连接 | USB有线 + BLE 5.0双模 |
| 供电 | 5V直连VBUS，物理开关控制RGB灯电源 |

## 灯光逻辑（核心）

**RGB动画与通道指示灯同时工作，互不干扰：**

- RGB动画正常运行时，通道指示灯以独立颜色覆盖对应按键的LED
- RGB关闭状态下，通道指示灯仍然单独亮起（proxy驱动自主刷新strip）
- 通道切换后，指示灯闪烁 → 连接成功后常亮2秒 → 熄灭，RGB动画恢复完全控制
- USB指示灯：未接入USB时持续闪烁，直到USB连接成功后常亮2秒熄灭
- **断电保留RGB灯效、开关状态、亮度、色相**；重新上电恢复到断电前状态

### 指示灯LED位置映射

> ⚠️ 物理LED编号从1开始（RGB1~RGB17），代码中数组索引 = 物理编号 − 1。

| 按键 | 物理LED | 数组索引 | 通道 | 颜色 | 广播名 |
|:----:|:-------:|:--------:|:----:|:----:|:------:|
| NUM | RGB1 | 0 | NumLock | 白色 0xFFFFFF | — |
| 1（P1） | RGB12 | 11 | BLE 1 | 蓝色 0x0000FF | czm_ble_1 |
| 2（P2） | RGB13 | 12 | BLE 2 | 蓝色 0x0000FF | czm_ble_2 |
| 3（P3） | RGB14 | 13 | BLE 3 | 蓝色 0x0000FF | czm_ble_3 |
| 4（P4） | RGB8 | 7 | USB | 绿色 0x00CC00 | czm_usb |

### 指示灯状态机

```
切换通道 → 对应LED蓝色/绿色闪烁（500ms周期）
         → 已连接/已配对：常亮2秒 → 熄灭，恢复RGB
         → 未连接/配对中：持续闪烁
         → USB无连接：持续绿色闪烁，直到USB接入

NumLock → 宿主上报NumLock=on：NUM键白色常亮
        → 宿主上报NumLock=off：NUM键指示灯熄灭
```

## 快捷键

### 基础层（Layer 0）

| 按键 | 短按 | 长按 |
|:----:|:----:|:----:|
| NUM | 开关数字输入（NumLock） | 进入FN层（Layer 1） |
| / | 除号（KP_DIVIDE） | 进入RGB层（Layer 2） |

### FN层 — Layer 1（长按NUM）

| 快捷键 | 功能 | 说明 |
|:------:|:----:|:----:|
| FN + 1 | 蓝牙通道1 | 选择BLE Profile 0 |
| FN + 2 | 蓝牙通道2 | 选择BLE Profile 1 |
| FN + 3 | 蓝牙通道3 | 选择BLE Profile 2 |
| FN + 4 | 切换USB/BLE输出 | &out OUT_TOG |
| FN + * | ZMK Studio解锁 | 实时改键/RGB |
| FN + - | Bootloader | 进入UF2烧录模式 |
| FN + . | 清除配对 | BT_CLR |

### RGB层 — Layer 2（长按 /）

| 快捷键 | 功能 | ZMK键码 |
|:------:|:----:|:-------:|
| RGB + 7 | RGB开关 | RGB_TOG |
| RGB + 8 | 亮度+ | RGB_BRI |
| RGB + 2 | 亮度- | RGB_BRD |
| RGB + 0 | 灯效切换 | RGB_EFF |

## 物理键位与层映射

```
Layer 0 (默认):
┌──────┬──────┬──────┬──────┐
│ NUM  │  /   │  *   │  -   │
│LT(1) │LT(2) │      │      │
├──────┼──────┼──────┼──────┤
│  7   │  8   │  9   │  +   │
│      │      │      │      │
├──────┼──────┼──────┼──────┤
│  4   │  5   │  6   │ Ent  │
│      │      │      │      │
├──────┼──────┼──────┴──────┘
│  1   │  2   │  3   │
│      │      │      │
├──────┴──────┼──────┤
│     0       │  .   │
│             │      │
└─────────────┴──────┘

Layer 1 (长按NUM):
┌──────┬──────┬──────┬──────┐
│ trans│ trans│Studio│Boot  │
├──────┼──────┼──────┼──────┤
│ trans│ trans│ trans│ trans│
├──────┼──────┼──────┼──────┤
│ OUT  │ trans│ trans│ trans│
├──────┼──────┼──────┴──────┘
│ BT0  │ BT1  │ BT2  │
├──────┴──────┼──────┤
│    trans    │BT_CLR│
└─────────────┴──────┘

Layer 2 (长按 /):
┌──────┬──────┬──────┬──────┐
│ trans│ trans│ trans│ trans│
├──────┼──────┼──────┼──────┤
│ TOG  │ BRI+ │ trans│ trans│
├──────┼──────┼──────┼──────┤
│ trans│ trans│ trans│ trans│
├──────┼──────┼──────┴──────┘
│ trans│ BRI- │ trans│
├──────┴──────┼──────┤
│    EFF      │ trans│
└─────────────┴──────┘
```

## 电源管理

| 状态 | 条件 | 行为 |
|:----:|:----:|:----:|
| 正常工作 | 有按键/活动 | RGB动画正常运行 |
| 浅睡眠 | 无输入5分钟 | RGB熄灭，任意键唤醒 |
| 深度睡眠 | **禁用** | nRF SPIM3驱动bug导致WS2812唤醒后卡死 |

### 睡眠配置

- `CONFIG_ZMK_IDLE_TIMEOUT=300000`（5分钟）
- `CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE=y`
- `CONFIG_ZMK_SLEEP` 不启用

## BLE配置

| 项目 | 值 |
|:----:|:--:|
| 最大配对设备 | 3 |
| 最大连接数 | 3 |
| 默认广播名 | czm_ble_1 |
| 通道切换 | 动态改名为 czm_ble_1/2/3 |
| USB设备名 | czm_usb |

## ZMK Studio

- 已启用，支持实时改键和RGB控制
- Studio锁：断开自动锁定
- 解锁键：FN + *

## 编译信息

| 项目 | 值 |
|:----:|:--:|
| ZMK版本 | v0.3.0 (commit edf5c08) |
| 编译方式 | GitHub Actions |
| 目标板 | nice_nano_v2 (nRF52840) |
| 仓库 | [github.com/micahyy/zmk-config](https://github.com/micahyy/zmk-config) |
| 分支 | main |

---

## 问题跟踪

### 2026-08-28 第二轮实测反馈（commit 80e50c7，Build #95通过）

**问题1：RGB状态断电不持久**
- 现象：通电会自动切换到默认灯效，没有记录断电之前的灯效
- 修复：`CONFIG_ZMK_RGB_UNDERGLOW_ON_START` 从 `y` 改为 `n`。ZMK settings子系统默认启用，会自动保存当前灯效/色相/亮度/开关状态到flash，上电恢复。`*_START` 系列值仅在首次烧录（无settings数据）时作为默认值使用。注意：当前ZMK v0.3.0版本中`CONFIG_ZMK_SETTINGS`符号不存在，不能显式设置。
- 状态：✅ 已修复，待刷入测试

**问题2：BLE指示灯位置偏移**
- 现象：通道1还是在2的位置闪，RGB从RGB1~RGB17，没有RGB0
- 修复：物理LED编号是1-based，代码数组索引需要 `物理编号 − 1`。原索引12/13/14/8分别调整为11/12/13/7。
- 状态：✅ 已修复，待刷入测试（如果位置仍然不对，需要确认PCB上WS2812的实际串接顺序）

**问题3：RGB关闭时通道指示灯不闪**
- 现象：关闭灯的情况下通道指示灯不闪，只常亮关闭前的指示灯
- 根因：proxy驱动依赖ZMK underglow每帧调`update_rgb`来叠加指示灯颜色；RGB关闭后underglow不再发送帧，proxy没有机会刷新strip
- 修复：新增独立refresh工作队列（50ms周期），检测到250ms内无underglow帧时判定为RGB关闭状态，proxy直接构造全黑+指示灯像素写入WS2812。指示灯闪烁/常亮在RGB关闭时也能正常工作。
- 状态：✅ 已修复，待刷入测试

**问题4：NumLock指示灯缺失**
- 现象：numberlock指示灯没有
- 修复：新增第5个指示灯槽位（slot 4），硬编码在NUM键位置（数组索引0 = RGB1），白色0xFFFFFF。订阅`zmk_hid_indicators_changed`事件，读取bit0（HID_LED_NUM_LOCK），宿主上报NumLock开启时白色常亮，关闭时熄灭。与其他通道指示灯互不干扰。
- 状态：✅ 已修复，待刷入测试

**问题5：BLE指示灯蓝色太浅**
- 现象：蓝牙指示灯的颜色有点浅蓝
- 修复：BLE蓝色从 `0x0000CC` 改为 `0x0000FF`（纯蓝，最大亮度）
- 状态：✅ 已修复，待刷入测试

### 修改的文件

| 文件 | 修改内容 |
|:----:|:----:|
| `config/boards/shields/dz17/dz17.conf` | ON_START y→n（settings默认启用，不显式设置CONFIG_ZMK_SETTINGS） |
| `config/boards/shields/dz17/dz17.overlay` | 索引12/13/14/8→11/12/13/7，蓝色0x0000CC→0x0000FF |
| `config/src/dz17_indicator.c` | 新增NumLock槽位/HID事件监听/自主刷新机制，BLE槽位扩展为5个 |

---

## 【2026-08-30 定稿】单色 GPIO 灯方案（替代 WS2812）

WS2812 灯阵硬件拆除，改为 5 颗独立单色 LED，Zephyr gpio-leds 驱动 + 自定义模块 `config/src/dz17_indicator.c`。已合并 main（merge 7418abe1）。

### 硬件接线

每颗灯独立串 1K 限流电阻，电阻在阳极侧（用户习惯）：

```
3.3V → 1K电阻 → 灯长脚(+) → 灯短脚(-) → GPIO
```

| 灯 | 颜色 | GPIO | 含义 |
|---|---|---|---|
| led0 | 白 | P0.22 | NumLock，跟随宿主 HID 报告常亮/灭 |
| led1 | 蓝 | P0.12 | BLE profile 1 |
| led2 | 蓝 | P0.04 | BLE profile 2 |
| led3 | 蓝 | P0.26 | BLE profile 3 |
| led4 | 绿 | P0.08 | USB 输出通道 |

- active-low：GPIO 拉低点亮（gpio-leds GPIO_ACTIVE_LOW）
- **不共电阻**：不同颜色 Vf 差异会致亮度参差、同亮电流翻倍
- 阳极接 nice!nano 板载 3.3V（VDD/REG0，外部负载 ≤25mA），禁接 RAW/VBUS；nRF52840 GPIO 非 5V 容忍

### 充电指示灯（TP4056，纯硬件）

STBY/CHRG 为开漏输出，灯须从 3.3V 取电：

```
3.3V → 1K电阻 → 灯长脚(+) → 灯短脚(-) → TP4056 引脚
```

- 红灯接 CHRG：充电中亮，充满灭
- 绿灯接 STBY：充满常亮，充电中灭
- P0.31 照常接 STBY 供固件读充电状态（zmk,battery-nrf-vddh 的 charge-status-gpios，GPIO_ACTIVE_LOW），与灯并联互不影响

### 灯逻辑

- BLE 通道：当前 profile 已连接 = 对应蓝灯常亮；未连接/广播中 = 500ms 闪烁
- USB 通道：绿灯常亮，三个蓝灯全灭（切通道时互斥）
- 白灯：跟随电脑 NumLock HID 报告
- 蓝牙广播名按 profile 改为 czm_ble_1/2/3

### 一键关灯（新功能）

自定义 behavior `&czm_ledtog`（compatible `czmao,behavior-led-toggle`，binding YAML 在 `config/dts/bindings/behaviors/`）：

- **操作：按住 "/" 键（LT(2)）不放，再按一下 "*" 键** → 5 灯全灭；再操作一次恢复，恢复后自动同步当前 BLE/USB/NumLock 状态
- 关灯期间键盘功能不受影响；关灯状态下 BLE 闪烁/切通道等灯效请求被门控忽略
- layer2 第 3 格（position 2）绑定，layer2 其余位置保持 &trans

### ZMK v0.3.0 自定义 behavior 要点（踩坑记录）

1. v0.3.0 没有通用 `zmk,behavior` binding YAML，自定义 behavior 必须自带 binding YAML（放 config 仓库 `config/dts/bindings/behaviors/xxx.yaml`，`include: zero_param.yaml`），compatible 用厂商标识前缀（如 `czmao,behavior-led-toggle`），否则 edtlib 报 "lacks binding"
2. C 侧 API：`#include <drivers/behavior.h>`，结构体名是 `struct behavior_driver_api`（无 zmk_ 前缀），`BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api)`
3. `#define DT_DRV_COMPAT czmao_behavior_led_toggle` 必须放在所有 include 之前
4. pressed/released 回调返回 `ZMK_BEHAVIOR_OPAQUE`
