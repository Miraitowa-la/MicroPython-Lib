# MicroPython OLED (SSD1306) 扩展库 — 开发者参考文档

> `OLEDDisplay` · `OLED_I2C` · `OLED_SPI`
> 
> 适用平台：ESP32 / ESP32-S3 / RP2040　　MicroPython v1.20+

---

## 目录

1. [概述](#1-概述)
2. [OLED_I2C — I2C 接口驱动](#2-oled_i2c--i2c-接口驱动)
3. [OLED_SPI — SPI 接口驱动](#3-oled_spi--spi-接口驱动)
4. [核心控制与绘图 API](#4-核心控制与绘图-api)
5. [综合使用示例](#5-综合使用示例)
6. [注意事项与常见问题](#6-注意事项与常见问题)

---

## 1. 概述

本库（`oled_display.py`）是一个专为 MicroPython 环境设计的轻量级 OLED 显示驱动模块。底层深度继承了内置的 `framebuf.FrameBuffer`，以极低的内存开销实现了丰富的图形渲染能力。支持 I2C 与 SPI 两种主流物理总线。

### 1.1 模块一览

本库包含一个基类与两个基于不同总线的实现类：

|**类名**|**继承关系**|**传输层**|**适用场景**|
|---|---|---|---|
|`OLEDDisplay`|`framebuf.FrameBuffer`|—|抽象基类，封装通用初始化与控制逻辑|
|`OLED_I2C`|`OLEDDisplay`|I2C|引脚资源紧张、接线简单的场景（4 线）|
|`OLED_SPI`|`OLEDDisplay`|SPI|对屏幕刷新帧率要求较高的场景（7 线）|

### 1.2 支持的芯片与分辨率

- **核心主控**：SSD1306（完美支持），SH1106（兼容支持，需调节横向偏移量）。
- **常见分辨率**：128×64、128×32。

### 1.3 依赖与兼容性

- `machine.I2C`, `machine.SPI`, `machine.Pin` — 硬件接口底层支持。
- `framebuf` — MicroPython 内置的帧缓冲区模块，用于图形绘制计算。

> **提示**　本库无任何外部第三方依赖，直接将 `oled_display.py` 文件上传至单片机的 `/lib/` 目录或根目录即可使用。

---

## 2. OLED_I2C — I2C 接口驱动

### 2.1 快速开始
```python
from machine import I2C, Pin
from oled_display import OLED_I2C

# 1. 初始化 I2C 总线
i2c = I2C(0, scl=Pin(18), sda=Pin(17), freq=400000)

# 2. 创建 OLED 实例 (128x64 分辨率)
oled = OLED_I2C(128, 64, i2c)

# 3. 绘图与刷新
oled.fill(0)
oled.text("System Ready", 16, 28, 1)
oled.show()
```

### 2.2 构造参数

|**参数**|**类型**|**说明**|
|---|---|---|
|`width`|`int`|屏幕像素宽度（通常为 128）（必填）|
|`height`|`int`|屏幕像素高度（通常为 64 或 32）（必填）|
|`i2c`|`I2C`|已配置的 I2C 实例（必填）|
|`addr`|`int`|屏幕 I2C 从机地址，默认 `0x3C`|
|`external_vcc`|`bool`|是否使用外部 VCC 供电，默认 `False`（使用内部电荷泵）|

---

## 3. OLED_SPI — SPI 接口驱动

### 3.1 快速开始
```python
from machine import SPI, Pin
from oled_display import OLED_SPI

# 1. 初始化 SPI 总线
spi = SPI(1, baudrate=8000000, polarity=0, phase=0)

# 2. 配置控制引脚
dc  = Pin(2, Pin.OUT)   # 数据/命令选择
res = Pin(3, Pin.OUT)   # 硬件复位
cs  = Pin(4, Pin.OUT)   # 片选（可选）

# 3. 创建 OLED 实例
oled = OLED_SPI(128, 64, spi, dc, res, cs)

# 4. 绘图与刷新
oled.fill(0)
oled.text("SPI Fast Mode", 0, 0, 1)
oled.show()
```

### 3.2 构造参数

|**参数**|**类型**|**说明**|
|---|---|---|
|`width`|`int`|屏幕像素宽度（必填）|
|`height`|`int`|屏幕像素高度（必填）|
|`spi`|`SPI`|已配置的 SPI 实例（必填）|
|`dc`|`Pin`|Data/Command 控制引脚（必填）|
|`res`|`Pin`|Reset 硬件复位引脚（必填）|
|`cs`|`Pin \| None`|Chip Select 片选引脚，若屏幕直连且无其他 SPI 设备可设为 `None`|
|`external_vcc`|`bool`|是否使用外部 VCC 供电，默认 `False`|

---

## 4. 核心控制与绘图 API

### 4.1 硬件控制方法

以下方法由 `OLEDDisplay` 基类提供，主要用于控制屏幕本身的硬件状态。

|**方法**|**参数说明**|**描述**|
|---|---|---|
|`show()`|—|**核心方法**：将 RAM 中的缓冲区数据推送到 OLED 屏幕显示|
|`poweroff()`|—|关闭显示，屏幕进入低功耗休眠模式|
|`poweron()`|—|唤醒屏幕，恢复显示|
|`contrast(contrast)`|`contrast`: 0~255|设置屏幕对比度（亮度）|
|`invert(invert)`|`invert`: 0 或 1|设置是否反相显示（1为白底黑字，0为黑底白字）|

### 4.2 绘图 API (继承自 framebuf)

由于驱动直接继承了 `framebuf`，所有绘图操作都是在内存（Buffer）中进行的。调用这些方法后，**必须调用 `show()` 方法才能在屏幕上看到效果**。

|**方法**|**参数示例**|**说明**|
|---|---|---|
|`fill(c)`|`c`: 0 或 1|用指定颜色填充整个屏幕（0=清除，1=全亮）|
|`pixel(x, y, c)`|`x, y`: 坐标, `c`: 颜色|在指定坐标画一个像素点|
|`line(x1, y1, x2, y2, c)`|—|画一条直线|
|`rect(x, y, w, h, c)`|`w, h`: 宽高|画一个空心矩形|
|`fill_rect(x, y, w, h, c)`|`w, h`: 宽高|画一个实心矩形|
|`text(s, x, y, c)`|`s`: 字符串|在指定位置绘制默认 8x8 像素的 ASCII 文本|

---

## 5. 综合使用示例

以下示例展示了如何在物联网网关设备中，结合 Modbus 数据采集，实时在 OLED 屏幕上绘制系统状态和传感器数据面板。

```python
from machine import I2C, Pin
from oled_display import OLED_I2C
import time

# 假设环境：ESP32-S3，作为 Modbus 主站网关采集温湿度和气体数据
i2c = I2C(0, scl=Pin(18), sda=Pin(17), freq=400000)
oled = OLED_I2C(128, 64, i2c)

def update_ui(temp, humi, gas_ch4, modbus_status):
    oled.fill(0)  # 清理上一帧
    
    # 顶部状态栏
    oled.text("IoT Gateway v1.0", 0, 0, 1)
    oled.line(0, 10, 127, 10, 1)
    
    # 数据展示区
    oled.text(f"Temp: {temp:.1f} C", 0, 16, 1)
    oled.text(f"Humi: {humi:.1f} %", 0, 26, 1)
    
    # 模拟进度条展示 CH4 浓度
    oled.text("CH4:", 0, 40, 1)
    bar_width = int((gas_ch4 / 100.0) * 80) # 假设满量程100
    oled.rect(35, 40, 82, 10, 1)            # 外框
    oled.fill_rect(36, 41, bar_width, 8, 1) # 填充
    
    # 底部通讯状态
    status_str = "MB: OK" if modbus_status else "MB: ERR"
    oled.text(status_str, 80, 56, 1)
    
    # 刷入屏幕
    oled.show()

# 模拟主循环
while True:
    # 模拟从 Modbus 从站读取到的数据
    sim_temp = 26.5
    sim_humi = 55.2
    sim_ch4  = 45.0 
    
    update_ui(sim_temp, sim_humi, sim_ch4, True)
    time.sleep(1)
```

---

## 6. 注意事项与常见问题

### 6.1 `show()` 机制释疑

新手最常见的问题是“调用了 `oled.text()` 为什么屏幕没反应”。由于本库采用 `framebuf` 机制，所有的图形绘制操作实际上是在修改单片机内部的 RAM（`bytearray`）。只有在执行 `oled.show()` 时，这块 RAM 才会通过 I2C 或 SPI 总线批量发送给 SSD1306 芯片。这种机制避免了每次画一个像素就通信一次带来的极高延迟。

### 6.2 显存与格式 (MONO_VLSB)

该驱动使用 `MONO_VLSB`（单色，垂直低位在前）格式。128x64 的屏幕需要 $128 \times (64 \div 8) = 1024$ 字节的连续内存。对于内存极小的单片机，如果在实例化时报 `MemoryError`，请在 `boot.py` 中尽早进行内存垃圾回收（`gc.collect()`）。

### 6.3 I2C 地址问题 (0x3C vs 0x3D)

多数 0.96 寸屏幕的默认 I2C 地址为 `0x3C`，但部分屏幕背面有电阻跳线，可能将地址设为了 `0x3D`。如果遇到 `OSError: [Errno 19] ENODEV` 或 `OLED_ERR_BUS`，可使用 `i2c.scan()` 扫描总线确认实际地址，并在初始化时传入：

Python

```
oled = OLED_I2C(128, 64, i2c, addr=0x3D)
```

### 6.4 SPI 速率配置

SSD1306 芯片的 SPI 时钟频率最高可支持约 10MHz。在初始化硬件 SPI 时，推荐设置 `baudrate=8000000`（8MHz）即可获得极其丝滑的刷新体验，过高的频率（如 40MHz）可能导致飞线通信时出现画面撕裂或雪花点。