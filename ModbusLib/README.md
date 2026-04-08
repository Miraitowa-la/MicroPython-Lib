# MicroPython Modbus 扩展库 — 开发者参考文档

> `modbus_rtu_slave` · `modbus_rtu_master` · `modbus_tcp_slave` · `modbus_tcp_master`
>
> 适用平台：ESP32 / RP2040 / STM32　　MicroPython v1.20+

---

## 目录

1. [概述](#1-概述)
2. [modbus_rtu_slave — RTU 从站](#2-modbus_rtu_slave--rtu-从站)
3. [modbus_rtu_master — RTU 主站](#3-modbus_rtu_master--rtu-主站)
4. [modbus_tcp_slave — TCP 从站](#4-modbus_tcp_slave--tcp-从站)
5. [modbus_tcp_master — TCP 主站](#5-modbus_tcp_master--tcp-主站)
6. [Modbus 标准异常码参考](#6-modbus-标准异常码参考)
7. [综合使用示例](#7-综合使用示例)
8. [注意事项与常见问题](#8-注意事项与常见问题)

---

## 1. 概述

本库提供四个独立的 MicroPython Modbus 协议实现模块，支持 RTU（串口）与 TCP（以太网/Wi-Fi）两种物理层，Slave（从站/服务端）与 Master（主站/客户端）两种角色，共四种组合。各模块之间不存在强依赖关系，可单独引用。

### 1.1 模块一览

| 文件名 | 主类 | 数据类 | 常量前缀 | 传输层 |
|---|---|---|---|---|
| `modbus_rtu_slave.py` | `ModbusRTUSlave` | `MRTSDataMap` | `MRTS_` | UART |
| `modbus_rtu_master.py` | `ModbusRTUMaster` | `MRTMResponse` | `MRTM_` | UART |
| `modbus_tcp_slave.py` | `ModbusTCPSlave` | `MTCSDataMap` | `MTCS_` | TCP/IP |
| `modbus_tcp_master.py` | `ModbusTCPMaster` | `MTCMResponse` | `MTCM_` | TCP/IP |

### 1.2 命名规范

为避免多模块同时使用时的命名冲突，所有公共符号均携带各自前缀：

- `MRTS_` — RTU Slave 常量（Modbus RTU Slave）
- `MRTM_` — RTU Master 常量（Modbus RTU Master）
- `MTCS_` — TCP Slave 常量（Modbus TCP Slave）
- `MTCM_` — TCP Master 常量（Modbus TCP Master）

### 1.3 支持的功能码

| 功能码 | 标准名 | API 方法 | 返回数据 | 说明 |
|---|---|---|---|---|
| `0x01` | FC01 | `read_coils` | `bool list` | 读线圈（位） |
| `0x02` | FC02 | `read_discrete_inputs` | `bool list` | 读离散输入（只读位） |
| `0x03` | FC03 | `read_holding_regs` | `int list` | 读保持寄存器（uint16，可读写） |
| `0x04` | FC04 | `read_input_regs` | `int list` | 读输入寄存器（uint16，只读） |
| `0x05` | FC05 | `write_single_coil` | — | 写单个线圈，值 True/False |
| `0x06` | FC06 | `write_single_reg` | — | 写单个保持寄存器 |
| `0x0F` | FC15 | `write_multiple_coils` | — | 写多个线圈，传 bool 列表 |
| `0x10` | FC16 | `write_multiple_regs` | — | 写多个保持寄存器，传 int 列表 |
| `0x64` | — | `custom_cb` 回调 | — | 自定义配置（仅 Slave 端实现） |

> **提示**　四个模块均支持 FC01～FC16 全部 8 个标准功能码。0x64 自定义配置仅在 Slave 端以回调形式实现，Master 端可通过原始帧发送实现（不在标准 API 内）。

### 1.4 依赖与兼容性

- `machine.UART`，`machine.Pin` — RTU 系列（MicroPython 内置）
- `usocket` — TCP 系列（MicroPython 内置，需网络已就绪）
- `array` — 寄存器存储优化（可选，不可用时自动退回 `list`）

所有模块均无第三方依赖，直接将 `.py` 文件上传至设备 `/lib/` 目录即可使用。

---

## 2. modbus_rtu_slave — RTU 从站

### 2.1 快速开始

```python
from modbus_rtu_slave import ModbusRTUSlave, MRTSDataMap
from machine import UART

# 1. 创建数据区
dm = MRTSDataMap(coil_count=16, holding_reg_count=32, input_reg_count=8)
dm.holding_regs[0] = 0x1234

# 2. 初始化串口
uart = UART(1, baudrate=9600, tx=17, rx=16, timeout=10)

# 3. 创建从站
slave = ModbusRTUSlave(uart, slave_addr=1, data_map=dm, rs485_de_pin=4)

# 4. 主循环
while True:
    slave.process()                     # 非阻塞，有帧则处理
    dm.input_regs[0] = read_sensor()    # 更新只读数据
```

### 2.2 MRTSDataMap — 数据映射

数据映射对象是应用层与协议层共享的内存区域，直接读写其字段即可更新 Modbus 寄存器值。

#### 构造参数

| 参数 | 类型 | 说明 |
|---|---|---|
| `coil_count` | `int` | 线圈数量（位寻址，每 8 位占 1 字节） |
| `discrete_count` | `int` | 离散输入数量（只读位） |
| `holding_reg_count` | `int` | 保持寄存器数量（uint16，可读写） |
| `input_reg_count` | `int` | 输入寄存器数量（uint16，只读） |

#### 数据字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `.coils` | `bytearray` | 线圈位压缩存储，8 位/字节 |
| `.discrete_inputs` | `bytearray` | 离散输入位压缩存储 |
| `.holding_regs` | `array('H') / list` | 保持寄存器，uint16 列表 |
| `.input_regs` | `array('H') / list` | 输入寄存器，uint16 列表 |

#### 静态方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `get_bit(buf, idx)` | `bool` | 读取 bytearray 中第 idx 位的值 |
| `set_bit(buf, idx, val)` | `None` | 设置 bytearray 中第 idx 位 |

### 2.3 ModbusRTUSlave — 构造参数

| 参数 | 类型 | 说明 |
|---|---|---|
| `uart` | `UART` | 已配置的串口实例（必填） |
| `slave_addr` | `int` | 从站地址，范围 1\~247（必填） |
| `data_map` | `MRTSDataMap` | 数据区映射对象（必填） |
| `rs485_de_pin` | `int \| Pin \| None` | RS-485 DE 方向控制引脚，`None` = 不控制 |
| `de_active_high` | `bool` | DE 极性，`True` = 高电平发送（默认 `True`） |
| `use_crc_table` | `bool` | `True` = 查表法 CRC（快），`False` = 移位法（省内存） |
| `rx_timeout_ms` | `int` | 帧间超时毫秒数，默认 10ms（≈ 3.5 字符 @ 9600） |
| `buf_size` | `int` | 收发缓冲区字节数，默认 256 |
| `write_cb` | `callable \| None` | 写操作前回调，签名见下文 |
| `custom_cb` | `callable \| None` | 0x64 自定义配置回调，签名见下文 |
| `user_data` | `any` | 用户自定义数据，协议栈不使用 |

### 2.4 核心方法

| 方法 / 属性 | 返回 | 说明 |
|---|---|---|
| `process()` | `bool` | 非阻塞轮询一帧；处理了帧返回 `True`，无数据返回 `False` |
| `get_holding_reg(addr)` | `int` | 读取保持寄存器 |
| `set_holding_reg(addr, value)` | `None` | 写入保持寄存器 |
| `get_input_reg(addr)` | `int` | 读取输入寄存器 |
| `set_input_reg(addr, value)` | `None` | 写入输入寄存器 |
| `get_coil(addr)` | `bool` | 读取线圈状态 |
| `set_coil(addr, value)` | `None` | 设置线圈状态 |
| `get_discrete(addr)` | `bool` | 读取离散输入 |
| `set_discrete(addr, value)` | `None` | 设置离散输入 |
| `.slave_addr`（属性） | `int` | 当前从站地址，可赋值动态更改 |

### 2.5 回调函数

#### write_cb — 写操作前拦截

```python
def my_write_cb(slave, func_code, start_addr, quantity) -> bool:
    # 返回 False 时协议栈自动回复 SLAVE_DEVICE_FAILURE 异常
    if func_code in (0x05, 0x06, 0x0F, 0x10):
        if start_addr < 4:   # 地址 0~3 只读
            return False
    return True
```

| 参数 | 类型 | 说明 |
|---|---|---|
| `slave` | `ModbusRTUSlave` | 当前从站实例 |
| `func_code` | `int` | 触发写操作的功能码 |
| `start_addr` | `int` | 写入起始地址 |
| `quantity` | `int` | 写入数量 |
| 返回值 | `bool` | `True` = 允许写入，`False` = 拒绝并回复异常 |

#### custom_cb — 0x64 自定义配置

```python
def my_custom_cb(slave, param_addr, param_val) -> bool:
    print(f'自定义配置: addr={param_addr:#06x} val={param_val:#06x}')
    return True   # 返回 True 回复成功，False 回复异常
```

| 参数 | 类型 | 说明 |
|---|---|---|
| `slave` | `ModbusRTUSlave` | 当前从站实例 |
| `param_addr` | `int` | 自定义参数地址（uint16） |
| `param_val` | `int` | 自定义参数值（uint16） |
| 返回值 | `bool` | `True` = 执行成功，`False` = 执行失败 |

---

## 3. modbus_rtu_master — RTU 主站

### 3.1 快速开始

```python
from modbus_rtu_master import ModbusRTUMaster, MRTM_OK
from machine import UART

uart   = UART(1, baudrate=9600, tx=17, rx=16, timeout=50)
master = ModbusRTUMaster(uart, rs485_de_pin=4, retries=2)

# 读从站 1 的保持寄存器 0~4
resp = master.read_holding_regs(1, 0, 5)
if resp.ok:
    print('寄存器值:', resp.data)   # [1234, 0, 0, 0, 0]
else:
    print('错误:', resp)            # <MRTMResponse ERR=TIMEOUT>

# 写单个寄存器
master.write_single_reg(1, 10, 9999)
```

### 3.2 ModbusRTUMaster — 构造参数

| 参数 | 类型 | 说明 |
|---|---|---|
| `uart` | `UART` | 已配置的串口实例（必填） |
| `rs485_de_pin` | `int \| Pin \| None` | RS-485 DE 方向控制引脚 |
| `de_active_high` | `bool` | DE 极性，默认 `True` |
| `response_timeout_ms` | `int` | 等待从站响应超时，默认 200ms |
| `inter_frame_ms` | `int` | 帧间静默时间，默认 5ms |
| `use_crc_table` | `bool` | `True` = 查表法 CRC，默认 `True` |
| `retries` | `int` | 超时后重试次数，默认 0（不重试） |
| `buf_size` | `int` | 收发缓冲区字节数，默认 256 |

### 3.3 请求 API

所有方法第一个参数均为目标从站地址 `slave_addr`（1\~247）。

| 方法 | 返回 | 说明 |
|---|---|---|
| `read_coils(addr, start, qty)` | `MRTMResponse` | FC01 读线圈，`data` = `bool` 列表 |
| `read_discrete_inputs(addr, start, qty)` | `MRTMResponse` | FC02 读离散输入，`data` = `bool` 列表 |
| `read_holding_regs(addr, start, qty)` | `MRTMResponse` | FC03 读保持寄存器，`data` = `int` 列表 |
| `read_input_regs(addr, start, qty)` | `MRTMResponse` | FC04 读输入寄存器，`data` = `int` 列表 |
| `write_single_coil(addr, coil, value)` | `MRTMResponse` | FC05 写单个线圈，`value` = `True/False` |
| `write_single_reg(addr, reg, value)` | `MRTMResponse` | FC06 写单个寄存器，`value` = uint16 |
| `write_multiple_coils(addr, start, values)` | `MRTMResponse` | FC15 写多个线圈，`values` = `bool` 列表 |
| `write_multiple_regs(addr, start, values)` | `MRTMResponse` | FC16 写多个寄存器，`values` = `int` 列表 |

### 3.4 MRTMResponse — 响应对象

| 属性 | 类型 | 说明 |
|---|---|---|
| `.ok` | `bool` | `True` 表示请求成功（`error_code == 0`） |
| `.error_code` | `int` | `0` = 成功，负数 = 错误，见错误码表 |
| `.exception` | `int` | 从站 Modbus 异常码（仅 `ERR_EXCEPTION` 时有效） |
| `.data` | `list \| None` | 读操作返回的数据列表（写操作为 `None`） |
| `.raw` | `bytes` | 原始响应帧（含 CRC） |

### 3.5 错误码

| 常量 | 值 | 说明 |
|---|---|---|
| `MRTM_OK` | `0` | 请求成功 |
| `MRTM_ERR_TIMEOUT` | `-1` | 等待响应超时 |
| `MRTM_ERR_CRC` | `-2` | CRC 校验失败 |
| `MRTM_ERR_EXCEPTION` | `-3` | 从站返回 Modbus 异常码 |
| `MRTM_ERR_INVALID_RESPONSE` | `-4` | 响应帧格式错误 |
| `MRTM_ERR_INVALID_PARAM` | `-5` | 请求参数非法（数量越界等） |

---

## 4. modbus_tcp_slave — TCP 从站

### 4.1 快速开始

```python
import network, time
from modbus_tcp_slave import ModbusTCPSlave, MTCSDataMap

# 连接 Wi-Fi（ESP32 示例）
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect('MySSID', 'MyPassword')
while not wlan.isconnected():
    time.sleep_ms(100)

# 数据区
dm = MTCSDataMap(coil_count=16, holding_reg_count=32, input_reg_count=8)
dm.holding_regs[0] = 0xABCD

# 创建并启动从站
slave = ModbusTCPSlave(dm, unit_id=1, port=502, max_clients=4)
slave.start()

while True:
    slave.process()              # 非阻塞轮询
    slave.set_input_reg(0, 42)   # 更新传感器值
    time.sleep_ms(1)
```

### 4.2 MTCSDataMap — 数据映射

结构与 `MRTSDataMap` 完全相同，构造参数、字段和静态方法一致，详见[第 2.2 节](#22-mrtsdatamap--数据映射)。

### 4.3 ModbusTCPSlave — 构造参数

| 参数 | 类型 | 说明 |
|---|---|---|
| `data_map` | `MTCSDataMap` | 数据区映射对象（必填） |
| `unit_id` | `int` | Unit ID，`0xFF` = 接受全部（默认 `1`） |
| `host` | `str` | 监听地址，默认 `'0.0.0.0'`（所有网口） |
| `port` | `int` | 监听端口，默认 `502` |
| `max_clients` | `int` | 最大并发连接数，默认 `4` |
| `buf_size` | `int` | 每连接接收缓冲字节数，默认 `300` |
| `write_cb` | `callable \| None` | 写操作前回调，签名同 `write_cb`（[第 2.5 节](#25-回调函数)） |
| `custom_cb` | `callable \| None` | 0x64 自定义配置回调 |
| `user_data` | `any` | 用户自定义数据 |

### 4.4 生命周期与核心方法

| 方法 / 属性 | 返回 | 说明 |
|---|---|---|
| `start()` | `None` | 开始监听，创建非阻塞 server socket |
| `stop()` | `None` | 关闭所有连接并释放 socket |
| `process()` | `None` | 非阻塞轮询：接受新连接、处理已有连接数据 |
| `.client_count` | `int` | 当前已连接客户端数量（只读属性） |
| `get/set_holding_reg(addr[, val])` | `int/None` | 读写保持寄存器 |
| `get/set_input_reg(addr[, val])` | `int/None` | 读写输入寄存器 |
| `get/set_coil(addr[, val])` | `bool/None` | 读写线圈 |
| `get/set_discrete(addr[, val])` | `bool/None` | 读写离散输入 |

### 4.5 MBAP 帧结构

Modbus TCP 使用 MBAP（Modbus Application Protocol）头替代 RTU 的地址字节和 CRC，传输可靠性由 TCP 层保证。

```
┌──────────────┬──────────────┬──────────────┬─────────┬──────────────────────┐
│ Transaction  │  Protocol ID │    Length    │ Unit ID │        PDU           │
│    ID (2B)   │  (2B, 0x00)  │    (2B)      │  (1B)   │  FC(1) + Data(N)     │
└──────────────┴──────────────┴──────────────┴─────────┴──────────────────────┘
  由 Master 生成   固定 0x0000   Unit ID + PDU   从站地址   功能码 + 数据
```

---

## 5. modbus_tcp_master — TCP 主站

### 5.1 快速开始

```python
from modbus_tcp_master import ModbusTCPMaster

# ── 方式一：手动管理连接 ──────────────────────────────────────────────────────
master = ModbusTCPMaster('192.168.1.100', port=502, unit_id=1, retries=2)

resp = master.read_holding_regs(0, 5)   # 读寄存器 0~4
if resp.ok:
    print(resp.data)                    # [0xABCD, 0, 0, 0, 0]

master.write_single_reg(10, 9999)
master.write_multiple_regs(20, [1, 2, 3, 4])
master.disconnect()

# ── 方式二：with 语句（自动断开）─────────────────────────────────────────────
with ModbusTCPMaster('192.168.1.100') as m:
    resp = m.read_input_regs(0, 4)
    if resp.ok:
        print('输入寄存器:', resp.data)

# ── 方式三：TCP-RTU 网关（单连接访问多从站）──────────────────────────────────
gw = ModbusTCPMaster('192.168.1.1')
for uid in range(1, 5):
    resp = gw.read_holding_regs(0, 1, unit_id=uid)   # 覆盖默认 unit_id
    print(f'从站 {uid}:', resp)
gw.disconnect()
```

### 5.2 ModbusTCPMaster — 构造参数

| 参数 | 类型 | 说明 |
|---|---|---|
| `host` | `str` | 从站 IP 地址（必填） |
| `port` | `int` | 从站端口，默认 `502` |
| `unit_id` | `int` | 默认 Unit ID，默认 `1` |
| `connect_timeout_ms` | `int` | TCP 连接超时，默认 3000ms |
| `response_timeout_ms` | `int` | 等待响应超时，默认 1000ms |
| `keep_alive` | `bool` | `True` = 保持连接复用（默认），`False` = 每次请求后断开 |
| `retries` | `int` | 超时或连接失败后重试次数，默认 `1` |
| `buf_size` | `int` | 收发缓冲区字节数，默认 `300` |

### 5.3 请求 API

所有方法均包含可选的 `unit_id` 参数，用于覆盖构造时指定的默认值（适用于 TCP-RTU 网关场景）。

| 方法 | 返回 | 说明 |
|---|---|---|
| `read_coils(start, qty[, uid])` | `MTCMResponse` | FC01 读线圈 |
| `read_discrete_inputs(start, qty[, uid])` | `MTCMResponse` | FC02 读离散输入 |
| `read_holding_regs(start, qty[, uid])` | `MTCMResponse` | FC03 读保持寄存器 |
| `read_input_regs(start, qty[, uid])` | `MTCMResponse` | FC04 读输入寄存器 |
| `write_single_coil(addr, value[, uid])` | `MTCMResponse` | FC05 写单个线圈 |
| `write_single_reg(addr, value[, uid])` | `MTCMResponse` | FC06 写单个寄存器 |
| `write_multiple_coils(start, values[, uid])` | `MTCMResponse` | FC15 写多个线圈 |
| `write_multiple_regs(start, values[, uid])` | `MTCMResponse` | FC16 写多个寄存器 |

### 5.4 连接管理

| 方法 / 属性 | 返回 | 说明 |
|---|---|---|
| `connect()` | `bool` | 主动建立 TCP 连接，返回是否成功 |
| `disconnect()` | `None` | 断开连接 |
| `.connected` | `bool` | 是否已连接（只读属性） |
| `__enter__ / __exit__` | — | 支持 `with` 语句自动管理连接生命周期 |

### 5.5 MTCMResponse — 响应对象

| 属性 | 类型 | 说明 |
|---|---|---|
| `.ok` | `bool` | `True` 表示请求成功 |
| `.error_code` | `int` | `0` = 成功，负数 = 错误，见错误码表 |
| `.exception` | `int` | 从站 Modbus 异常码 |
| `.data` | `list \| None` | 读操作返回的数据列表 |
| `.raw` | `bytes` | 原始响应 PDU（不含 MBAP 头） |

### 5.6 错误码

| 常量 | 值 | 说明 |
|---|---|---|
| `MTCM_OK` | `0` | 请求成功 |
| `MTCM_ERR_TIMEOUT` | `-1` | 等待响应超时 |
| `MTCM_ERR_CONNECT` | `-2` | TCP 连接失败 |
| `MTCM_ERR_EXCEPTION` | `-3` | 从站返回 Modbus 异常码 |
| `MTCM_ERR_INVALID_RESPONSE` | `-4` | 响应帧格式错误 |
| `MTCM_ERR_INVALID_PARAM` | `-5` | 请求参数非法（数量越界等） |

---

## 6. Modbus 标准异常码参考

当从站无法正常处理请求时，返回功能码最高位置 1（如 FC03 → `0x83`）加异常码的响应帧：

| 异常码 | 常量名 | 说明 |
|---|---|---|
| `0x01` | `ILLEGAL_FUNCTION` | 功能码不被从站支持 |
| `0x02` | `ILLEGAL_DATA_ADDRESS` | 请求地址超出从站数据范围 |
| `0x03` | `ILLEGAL_DATA_VALUE` | 请求数据值非法（如线圈值不是 `0x0000` / `0xFF00`） |
| `0x04` | `SLAVE_DEVICE_FAILURE` | 从站内部处理失败（`write_cb` 返回 `False`） |

---

## 7. 综合使用示例

### 7.1 RTU 从站 + RS-485（STM32 / ESP32）

```python
from modbus_rtu_slave import ModbusRTUSlave, MRTSDataMap
from machine import UART
import time

# 数据区：16 线圈、8 离散、32 保持、8 输入
dm = MRTSDataMap(16, 8, 32, 8)

def write_guard(slave, fc, start, qty):
    """拒绝写入保持寄存器 0~3（配置区只读）"""
    if fc in (0x06, 0x10) and start < 4:
        return False
    return True

uart  = UART(1, baudrate=9600, tx=17, rx=16, timeout=10)
slave = ModbusRTUSlave(uart, 1, dm, rs485_de_pin=4, write_cb=write_guard)

while True:
    slave.process()
    dm.input_regs[0] = 300                              # 模拟温度 × 10
    dm.input_regs[1] = 650                              # 模拟湿度 × 10
    MRTSDataMap.set_bit(dm.discrete_inputs, 0, True)    # 运行状态
    time.sleep_ms(10)
```

### 7.2 RTU 主站轮询多从站

```python
from modbus_rtu_master import ModbusRTUMaster, MRTM_ERR_TIMEOUT
from machine import UART
import time

uart   = UART(1, baudrate=9600, tx=17, rx=16, timeout=50)
master = ModbusRTUMaster(uart, rs485_de_pin=4, retries=1)

SLAVES = [1, 2, 3]   # 从站地址列表

while True:
    for addr in SLAVES:
        resp = master.read_input_regs(addr, 0, 2)
        if resp.ok:
            temp = resp.data[0] / 10.0
            humi = resp.data[1] / 10.0
            print(f'从站 {addr}: 温度={temp}°C  湿度={humi}%')
        elif resp.error_code == MRTM_ERR_TIMEOUT:
            print(f'从站 {addr}: 离线')
        time.sleep_ms(50)
    time.sleep(1)
```

### 7.3 TCP 从站（ESP32 Wi-Fi）

```python
import network, time
from modbus_tcp_slave import ModbusTCPSlave, MTCSDataMap

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect('MySSID', 'MyPassword')
while not wlan.isconnected():
    time.sleep_ms(200)
print('IP:', wlan.ifconfig()[0])

dm = MTCSDataMap(holding_reg_count=64, input_reg_count=16)

slave = ModbusTCPSlave(dm, unit_id=1, port=502)
slave.start()

tick = 0
while True:
    slave.process()
    tick += 1
    if tick % 100 == 0:                   # 每 100ms 更新一次
        slave.set_input_reg(0, tick)      # 计数器示例
    time.sleep_ms(1)
```

### 7.4 TCP 主站访问 PLC

```python
from modbus_tcp_master import ModbusTCPMaster

# 访问西门子 S7-1200 Modbus TCP 模块（举例）
with ModbusTCPMaster('192.168.0.10', port=502, unit_id=0) as plc:
    # 读 40001~40010（Modbus 地址从 0 开始，即 0~9）
    resp = plc.read_holding_regs(0, 10)
    if resp.ok:
        for i, v in enumerate(resp.data):
            print(f'  MW{i*2:03d} = {v}')

    # 写输出线圈
    plc.write_single_coil(0, True)                          # Q0.0 = ON
    plc.write_multiple_coils(0, [True, False, True, True])  # Q0.0~Q0.3
```

---

## 8. 注意事项与常见问题

### 8.1 RTU 帧间超时配置

Modbus RTU 使用帧间静默时间（3.5 个字符时间）作为帧边界。`UART` 的 `timeout` 参数与 `rx_timeout_ms` 需配合设置：

```python
# 9600bps，1 字符时间 ≈ 1.04ms，3.5 字符 ≈ 3.6ms
# 建议 UART timeout 与 rx_timeout_ms 均设为 10ms（留一定余量）
uart  = UART(1, baudrate=9600, tx=17, rx=16, timeout=10)
slave = ModbusRTUSlave(uart, 1, dm, rx_timeout_ms=10)
```

> **注意**　`timeout` 设置过大会导致响应迟缓；设置过小会造成一帧被截断为多帧。高波特率（115200+）应适当减小此值。

### 8.2 RS-485 方向切换时序

RS-485 收发切换时序对通信可靠性至关重要。本库在发送结束后会等待足够的位时间再切换回接收模式。若出现首字节丢失问题，可适当增大切换后的延时裕量（修改 `_send_response` 中的 `+ 200` μs 偏移值）。

### 8.3 TCP 多连接内存占用

每个 TCP 客户端连接消耗约 `buf_size` 字节的接收缓冲区。在内存受限的设备（如 ESP8266）上建议将 `max_clients` 设为 1\~2，`buf_size` 设为 128。

### 8.4 CRC 模式选择

```python
# 查表法：速度快，占用 512 字节常量内存
ModbusRTUSlave(uart, 1, dm, use_crc_table=True)    # 默认，推荐

# 移位法：速度慢约 8 倍，但节省 512 字节 RAM
ModbusRTUSlave(uart, 1, dm, use_crc_table=False)   # 内存极紧张时使用
```

### 8.5 线圈 / 离散输入位序

Modbus 规范规定：字节内最低位（bit 0）对应最小地址的线圈，与字节高位优先顺序相反。本库严格遵循此规范：

```python
# 线圈 0~7 存储在 coils[0]
# 线圈 0 → coils[0] bit0，线圈 7 → coils[0] bit7
MRTSDataMap.set_bit(dm.coils, 0, True)   # 线圈 0 = ON
MRTSDataMap.set_bit(dm.coils, 7, True)   # 线圈 7 = ON
# 此时 coils[0] = 0b10000001 = 0x81
```

### 8.6 地址规范

本库使用协议层地址（从 0 开始），与某些 HMI / SCADA 工具显示的编号（从 1 开始，如 40001）存在偏移：

```python
# Modbus 地址 0  ←→  线圈编号 00001，寄存器编号 40001
# Modbus 地址 99 ←→  线圈编号 00100，寄存器编号 40100
master.read_holding_regs(slave=1, start=0, qty=1)   # 读 40001
```

### 8.7 TCP keep_alive 与长时间空闲

当 `keep_alive=True` 时，长时间无请求可能导致路由器或 NAT 设备关闭连接而客户端无感知。此时下一次请求会触发 `MTCM_ERR_CONNECT`，主站会自动重连（由 `retries` 控制次数）。对于轮询间隔超过 30 秒的场景，建议设置 `keep_alive=False` 或手动在空闲时 `disconnect()` / `connect()`。