# MicroPython-Lib

这是一个面向 MicroPython 的功能库合集，目前包含 3 个独立模块：

- `MG996RLib`：MG996R 舵机控制库
- `ModbusLib`：Modbus RTU / TCP 通信库
- `OLEDLib`：SSD1306 OLED 显示驱动库

这些库主要适用于 ESP32、RP2040、STM32 等运行 MicroPython 的开发板，可按需单独使用，也可以组合成完整的嵌入式项目。

## 仓库结构

```text
MicroPython-Lib/
├─ MG996RLib/
├─ ModbusLib/
└─ OLEDLib/
```

## 子库说明

### MG996RLib

用于控制 MG996R 舵机，提供单舵机控制、多舵机分组控制、角度设置、脉宽设置、缓慢移动等功能。

说明文档：
[MG996RLib/README.md](e:\Project\ESP32\MicroPython-Lib\MG996RLib\README.md)

### ModbusLib

用于实现 Modbus 通信，包含 RTU 从站、RTU 主站、TCP 从站、TCP 主站四个模块，适合串口 RS-485 与以太网 / Wi-Fi 通信场景。

说明文档：
[ModbusLib/README.md](e:\Project\ESP32\MicroPython-Lib\ModbusLib\README.md)

### OLEDLib

用于驱动 SSD1306 OLED 屏幕，支持 I2C 和 SPI 两种接口，适合显示文本、图形和运行状态信息。

说明文档：
[OLEDLib/README.md](e:\Project\ESP32\MicroPython-Lib\OLEDLib\README.md)

## 使用建议

- 如果只需要某一个功能，可以直接使用对应目录中的 `.py` 文件。
- 如果想了解详细接口、参数和示例，直接进入各子目录的 `README.md` 查看即可。
- 建议将需要的库文件上传到 MicroPython 设备的 `/lib/` 目录后再在业务代码中导入。
