# MG996R 舵机驱动库 — 开发者参考文档

> `mg996r_servo.py`
>
> 适用平台：ESP32 / RP2040 / STM32　　MicroPython v1.20+

---

## 目录

1. [控制原理](#1-控制原理)
2. [快速开始](#2-快速开始)
3. [全局常量](#3-全局常量)
4. [MG996RServo — 单舵机驱动](#4-mg996rservo--单舵机驱动)
5. [ServoGroup — 多舵机编组](#5-servogroup--多舵机编组)
6. [内部辅助函数](#6-内部辅助函数)
7. [综合使用示例](#7-综合使用示例)
8. [注意事项](#8-注意事项)

---

## 1. 控制原理

MG996R 通过标准 PWM 信号控制角度，周期固定为 **20ms（50Hz）**，高电平脉宽决定转轴位置：

```
脉宽（µs）    角度
   500    →    0°
  1500    →   90°（回中）
  2500    →  180°
```

线性换算公式：

```
pulse_us = 500 + angle × (2500 - 500) / 180
         = 500 + angle × 11.11...
```

反向换算（脉宽 → 角度）：

```
angle = (pulse_us - 500) × 180 / (2500 - 500)
```

MicroPython 通过 `machine.PWM` 直接驱动 GPIO，使用 `duty_ns()`（纳秒精度）或 `duty_u16()`（16 位占空比）写入脉宽，库内部自动选择可用接口。

---

## 2. 快速开始

```python
from mg996r_servo import MG996RServo

# 创建实例，连接 GPIO 15，初始回中
servo = MG996RServo(pin=15)

# 设置角度
servo.set_angle(0)      # 转到 0°
servo.set_angle(180)    # 转到 180°
servo.center()          # 回中（90°）

# 缓慢扫描
servo.slow_move(180, step_deg=2, step_delay_ms=15)

# 停止 PWM 输出（省电）
servo.stop()
```

---

## 3. 全局常量

| 常量 | 默认值 | 说明 |
|---|---|---|
| `SERVO_FREQ_HZ` | `50` | PWM 频率（Hz），对应 20ms 周期 |
| `SERVO_PULSE_MIN_US` | `500` | 0° 对应脉宽（µs） |
| `SERVO_PULSE_MAX_US` | `2500` | 180° 对应脉宽（µs） |
| `SERVO_ANGLE_MIN` | `0` | 最小角度（度） |
| `SERVO_ANGLE_MAX` | `180` | 最大角度（度） |

> **提示**　非标准舵机的脉宽范围可能与默认值不同，通过构造参数 `pulse_min_us` / `pulse_max_us` 覆盖，无需修改全局常量。

---

## 4. MG996RServo — 单舵机驱动

### 4.1 构造参数

```python
MG996RServo(pin, freq=50, init_angle=90, pulse_min_us=500, pulse_max_us=2500)
```

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `pin` | `int \| Pin` | 必填 | PWM 输出引脚号或 `Pin` 对象 |
| `freq` | `int` | `50` | PWM 频率（Hz），通常无需更改 |
| `init_angle` | `int` | `90` | 初始化角度，超出范围自动钳位 |
| `pulse_min_us` | `int` | `500` | 0° 对应脉宽（µs），适配非标准舵机 |
| `pulse_max_us` | `int` | `2500` | 180° 对应脉宽（µs），适配非标准舵机 |

构造时自动完成以下操作：创建 `machine.PWM` 实例 → 设置频率 → 驱动到 `init_angle`。

### 4.2 核心方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `set_angle(angle)` | `None` | 设置角度（0 ~ 180°），超出范围自动钳位 |
| `set_pulse(pulse_us)` | `None` | 直接设置脉宽（µs），用于精细控制或非标准舵机 |
| `get_angle()` | `int` | 返回当前角度，未初始化时返回 `-1` |
| `center()` | `None` | 舵机回中（90°） |
| `stop()` | `None` | 停止 PWM 输出，释放扭矩（调用后 `running = False`） |
| `start(freq=None)` | `None` | 重新启动 PWM，恢复上次角度（`stop()` 后使用） |
| `slow_move(target, step_deg, step_delay_ms)` | `None` | 缓慢移动到目标角度（阻塞式） |

#### `slow_move` 参数详情

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `target_angle` | `int` | 必填 | 目标角度（0 ~ 180°），自动钳位 |
| `step_deg` | `int` | `1` | 每步步进角度（建议 1 ~ 5） |
| `step_delay_ms` | `int` | `15` | 每步等待时间（ms，建议 10 ~ 20） |

步进越小、延时越大，运动越平滑；步进越大，运动越快但可能丢步。

### 4.3 属性

| 属性 | 类型 | 说明 |
|---|---|---|
| `.angle` | `int` | 当前角度（只读，等同于 `get_angle()`） |
| `.running` | `bool` | PWM 是否正在输出 |
| `.pulse_min_us` | `int` | 当前实例 0° 脉宽设置 |
| `.pulse_max_us` | `int` | 当前实例 180° 脉宽设置 |

### 4.4 上下文管理器

`MG996RServo` 支持 `with` 语句，离开作用域时自动调用 `stop()`：

```python
with MG996RServo(pin=16, init_angle=0) as s:
    s.slow_move(180, step_deg=1, step_delay_ms=12)
# 此处自动 stop()，PWM 停止，扭矩释放
```

### 4.5 异常行为

在 `stop()` 之后调用 `set_angle()`、`set_pulse()`、`slow_move()` 均会抛出 `RuntimeError`，提示先调用 `start()` 恢复。

```python
servo.stop()
servo.set_angle(90)    # RuntimeError: 舵机已停止，请先调用 start() 重新启动
servo.start()
servo.set_angle(90)    # OK
```

---

## 5. ServoGroup — 多舵机编组

`ServoGroup` 将多个 `MG996RServo` 实例组合为一组，支持批量操作。

### 5.1 构造

```python
ServoGroup(servos: list)
```

| 参数 | 类型 | 说明 |
|---|---|---|
| `servos` | `list[MG996RServo]` | 舵机实例列表，顺序即为索引顺序 |

### 5.2 方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `set_all(angle)` | `None` | 所有舵机同时转到同一角度 |
| `set_angles(angles)` | `None` | 按索引分别设置各舵机角度，`angles` 列表长度不足时剩余舵机保持不动 |
| `center_all()` | `None` | 所有舵机回中（90°） |
| `stop_all()` | `None` | 停止所有舵机 PWM 输出 |
| `get_angles()` | `list[int]` | 返回所有舵机当前角度列表 |

### 5.3 索引访问

`ServoGroup` 支持下标访问和 `len()`：

```python
group = ServoGroup([MG996RServo(12), MG996RServo(13)])
group[0].set_angle(45)     # 单独控制第一个舵机
print(len(group))          # 2
```

### 5.4 示例

```python
from mg996r_servo import MG996RServo, ServoGroup

group = ServoGroup([
    MG996RServo(pin=12),   # 舵机 0
    MG996RServo(pin=13),   # 舵机 1
    MG996RServo(pin=14),   # 舵机 2
])

group.center_all()                      # 全部回中
group.set_angles([0, 90, 180])          # 分别设置
print(group.get_angles())               # [0, 90, 180]
group.stop_all()                        # 全部停止
```

---

## 6. 内部辅助函数

以下函数为模块内部实现，前缀 `_servo_`，不属于公共 API，但可在需要时直接调用：

| 函数 | 返回 | 说明 |
|---|---|---|
| `_servo_clamp_angle(angle)` | `int` | 角度钳位到 0 ~ 180 |
| `_servo_clamp_pulse(pulse_us)` | `int` | 脉宽钳位到 500 ~ 2500 µs |
| `_servo_angle_to_pulse_us(angle)` | `int` | 角度 → 脉宽（µs） |
| `_servo_pulse_us_to_angle(pulse_us)` | `int` | 脉宽（µs）→ 角度（近似整数） |

---

## 7. 综合使用示例

### 7.1 单舵机基本控制

```python
from mg996r_servo import MG996RServo
import time

servo = MG996RServo(pin=15, init_angle=90)
print(f"初始角度: {servo.angle}°")     # 90

servo.set_angle(0)
time.sleep_ms(600)
servo.set_angle(180)
time.sleep_ms(600)
servo.center()
time.sleep_ms(400)
servo.stop()
```

### 7.2 精细脉宽控制（非标准舵机适配）

```python
# 假设某舵机 0° 对应 600µs，180° 对应 2400µs
servo = MG996RServo(pin=15, pulse_min_us=600, pulse_max_us=2400)

servo.set_pulse(600)    # 0°
servo.set_pulse(1500)   # 90°（近似）
servo.set_pulse(2400)   # 180°
```

### 7.3 缓慢扫描（平滑运动）

```python
from mg996r_servo import MG996RServo

servo = MG996RServo(pin=15)

# 从 90° 缓慢移动到 0°（2°/步，每步 15ms）
servo.slow_move(0, step_deg=2, step_delay_ms=15)

# 再缓慢移动到 180°（1°/步，每步 10ms，最平滑）
servo.slow_move(180, step_deg=1, step_delay_ms=10)

servo.stop()
```

### 7.4 with 语句自动管理资源

```python
from mg996r_servo import MG996RServo
import time

with MG996RServo(pin=16, init_angle=0) as s:
    s.slow_move(180, step_deg=2, step_delay_ms=12)
    time.sleep(1)
    s.center()
# 自动 stop()，无需手动调用
```

### 7.5 stop / start 循环控制

```python
from mg996r_servo import MG996RServo
import time

servo = MG996RServo(pin=15)

servo.set_angle(45)
time.sleep(1)

servo.stop()            # 释放扭矩（此时舵机可被外力转动）
print(servo.running)    # False
time.sleep(2)

servo.start()           # 恢复 PWM，回到上次角度 45°
print(servo.running)    # True
servo.stop()
```

### 7.6 机械臂三轴联动

```python
from mg996r_servo import MG996RServo, ServoGroup
import time

# 底座 / 大臂 / 小臂
arm = ServoGroup([
    MG996RServo(pin=12, init_angle=90),   # 底座
    MG996RServo(pin=13, init_angle=90),   # 大臂
    MG996RServo(pin=14, init_angle=90),   # 小臂
])

# 动作序列
poses = [
    [90,  90,  90],    # 初始姿态
    [45,  60, 120],    # 左转俯冲
    [135, 120,  60],   # 右转抬臂
    [90,  90,  90],    # 回到初始
]

for angles in poses:
    arm.set_angles(angles)
    time.sleep_ms(800)

arm.stop_all()
```

---

## 8. 注意事项

### 8.1 引脚选择

并非所有 GPIO 均支持硬件 PWM。以 ESP32 为例，任意 GPIO 均可使用软件 PWM，但推荐使用硬件 PWM 通道（GPIO 0/2/4/5/12~19/21~23/25~27/32~33）以保证频率精度。RP2040 的硬件 PWM 切片分配规则请参考官方文档。

### 8.2 duty_ns 兼容性

`MG996RServo` 优先使用 `duty_ns()`（纳秒精度），不支持时自动退回 `duty_u16()` 计算。若两种接口均不可用，将在运行时报 `AttributeError`，此时需检查 MicroPython 版本是否过旧（v1.20 以下可能不含 `duty_ns`）。

### 8.3 电源注意事项

MG996R 在堵转或满载时电流可达 **2.5A**，建议使用独立 5V/3A 以上电源供电，**不可**直接由开发板 5V 引脚驱动，否则会导致主控复位或损坏。PWM 信号线可直接连接开发板 GPIO，但信号地必须与电源地共地。

### 8.4 slow_move 阻塞说明

`slow_move()` 是阻塞函数，执行期间不处理其他任务。若需要非阻塞控制，可在 `uasyncio` 中将其改写为协程：

```python
import uasyncio as asyncio
from mg996r_servo import MG996RServo, _servo_clamp_angle
import time

async def async_slow_move(servo, target, step_deg=1, step_delay_ms=15):
    target = _servo_clamp_angle(target)
    current = servo.angle if servo.angle >= 0 else 90
    while current != target:
        current = min(current + step_deg, target) if current < target \
                  else max(current - step_deg, target)
        servo.set_angle(current)
        await asyncio.sleep_ms(step_delay_ms)
```

### 8.5 角度精度

由于 MicroPython 整数除法截断，角度与脉宽之间存在 ±1µs 的舍入误差，对应约 ±0.09°，在实际使用中可忽略不计。`set_pulse()` 后通过 `get_angle()` 读回的角度为近似值，如需精确知道当前脉宽，应在应用层自行记录。

### 8.6 多舵机信号干扰

多路 PWM 同时切换时，若共地走线过细，可能引发信号干扰。建议各路 PWM 信号线加 100Ω 串联电阻，地线使用较粗导线并集中接地。
