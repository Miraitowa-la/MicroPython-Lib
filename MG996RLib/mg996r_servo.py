"""
mg996r_servo.py  —  MicroPython MG996R 舵机驱动库
==================================================
控制原理：
    PWM 周期  = 20ms（50Hz）
    高电平宽度  0.5ms ~ 2.5ms  对应  0° ~ 180°

脉宽与角度换算：
    pulse_us = PULSE_MIN_US + angle * (PULSE_MAX_US - PULSE_MIN_US) / ANGLE_MAX
             = 500 + angle * 11.11...

硬件要求：
    - 支持 PWM 输出的 GPIO 引脚
    - 使用 machine.PWM，频率设置为 50Hz

命名规范（为后续多舵机/多驱动扩展预留）：
    - 类前缀  Servo
    - 常量前缀 SERVO_
    - 内部方法 _servo_*

依赖：machine.PWM、machine.Pin、time
兼容平台：ESP32 / RP2040 / STM32（MicroPython v1.20+）
"""

from machine import PWM, Pin
import time

# ---------------------------------------------------------------------------
# 全局参数常量
# ---------------------------------------------------------------------------
SERVO_FREQ_HZ      = 50          # PWM 频率，50Hz = 20ms 周期
SERVO_PULSE_MIN_US = 500         # 0°  对应脉宽（µs）
SERVO_PULSE_MAX_US = 2500        # 180° 对应脉宽（µs）
SERVO_ANGLE_MIN    = 0           # 最小角度（度）
SERVO_ANGLE_MAX    = 180         # 最大角度（度）

# MicroPython PWM duty_ns 精度为纳秒，转换系数
_US_TO_NS = 1000


# ---------------------------------------------------------------------------
# 内部辅助函数
# ---------------------------------------------------------------------------
def _servo_clamp_angle(angle: int) -> int:
    """角度钳位：限制在 SERVO_ANGLE_MIN ~ SERVO_ANGLE_MAX。"""
    if angle < SERVO_ANGLE_MIN:
        return SERVO_ANGLE_MIN
    if angle > SERVO_ANGLE_MAX:
        return SERVO_ANGLE_MAX
    return angle


def _servo_clamp_pulse(pulse_us: int) -> int:
    """脉宽钳位：限制在 SERVO_PULSE_MIN_US ~ SERVO_PULSE_MAX_US。"""
    if pulse_us < SERVO_PULSE_MIN_US:
        return SERVO_PULSE_MIN_US
    if pulse_us > SERVO_PULSE_MAX_US:
        return SERVO_PULSE_MAX_US
    return pulse_us


def _servo_angle_to_pulse_us(angle: int) -> int:
    """角度 → 脉宽（µs），线性插值。"""
    return SERVO_PULSE_MIN_US + (
        angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) // SERVO_ANGLE_MAX
    )


def _servo_pulse_us_to_angle(pulse_us: int) -> int:
    """脉宽（µs）→ 角度（近似整数）。"""
    pulse_us = _servo_clamp_pulse(pulse_us)
    return (pulse_us - SERVO_PULSE_MIN_US) * SERVO_ANGLE_MAX // (
        SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US
    )


# ---------------------------------------------------------------------------
# MG996RServo 主类
# ---------------------------------------------------------------------------
class MG996RServo:
    """
    MG996R 舵机驱动（单实例）。

    参数
    ----
    pin          : int | Pin   PWM 输出引脚号或 Pin 对象
    freq         : int         PWM 频率，默认 50Hz（勿随意更改）
    init_angle   : int         初始角度，默认 90°（回中）
    pulse_min_us : int         0°  对应脉宽（µs），默认 500
    pulse_max_us : int         180° 对应脉宽（µs），默认 2500

    示例
    ----
    servo = MG996RServo(pin=15)
    servo.set_angle(90)
    servo.slow_move(0, step_deg=2, step_delay_ms=15)
    servo.stop()
    """

    def __init__(
        self,
        pin,
        freq: int = SERVO_FREQ_HZ,
        init_angle: int = 90,
        pulse_min_us: int = SERVO_PULSE_MIN_US,
        pulse_max_us: int = SERVO_PULSE_MAX_US,
    ):
        # 允许直接传入 Pin 对象或引脚号
        if isinstance(pin, int):
            pin = Pin(pin)
        self._pwm          = PWM(pin, freq=freq)
        self._freq         = freq
        self._pulse_min_us = pulse_min_us
        self._pulse_max_us = pulse_max_us
        self._current_angle: int = -1          # 尚未设置
        self._running: bool = True

        # 初始化到指定角度
        self.set_angle(_servo_clamp_angle(init_angle))

    # -----------------------------------------------------------------------
    # 内部：写入 PWM 占空比（纳秒精度，适配各平台）
    # -----------------------------------------------------------------------
    def _write_pulse_us(self, pulse_us: int) -> None:
        """将脉宽（µs）写入 PWM 硬件。优先使用 duty_ns，退回 duty_u16。"""
        if hasattr(self._pwm, "duty_ns"):
            self._pwm.duty_ns(pulse_us * _US_TO_NS)
        else:
            # duty_u16：占空比 = pulse_us / period_us * 65535
            period_us = 1_000_000 // self._freq
            self._pwm.duty_u16(pulse_us * 65535 // period_us)

    # -----------------------------------------------------------------------
    # 核心 API
    # -----------------------------------------------------------------------
    def set_angle(self, angle: int) -> None:
        """
        设置舵机角度（0 ~ 180°）。
        超出范围自动钳位，不抛出异常。
        """
        if not self._running:
            raise RuntimeError("舵机已停止，请先调用 start() 重新启动")
        angle = _servo_clamp_angle(angle)
        pulse_us = _servo_angle_to_pulse_us(angle)
        self._write_pulse_us(pulse_us)
        self._current_angle = angle

    def set_pulse(self, pulse_us: int) -> None:
        """
        直接设置脉宽（µs），用于精细控制或非标准舵机。
        超出范围自动钳位至 SERVO_PULSE_MIN_US ~ SERVO_PULSE_MAX_US。
        """
        if not self._running:
            raise RuntimeError("舵机已停止，请先调用 start() 重新启动")
        pulse_us = _servo_clamp_pulse(pulse_us)
        self._write_pulse_us(pulse_us)
        self._current_angle = _servo_pulse_us_to_angle(pulse_us)

    def get_angle(self) -> int:
        """返回当前角度（度）。未初始化时返回 -1。"""
        return self._current_angle

    def center(self) -> None:
        """舵机回中（90°）。"""
        self.set_angle(90)

    def stop(self) -> None:
        """
        停止 PWM 输出，释放扭矩（省电模式）。
        停止后舵机失去保持力；如需恢复，调用 start()。
        """
        self._pwm.deinit()
        self._running = False

    def start(self, freq: int = None) -> None:
        """
        重新启动 PWM 输出（stop() 后使用）。
        恢复到上次记录的角度位置。
        """
        if freq is not None:
            self._freq = freq
        # 重新初始化 PWM（deinit 后需重建）
        self._pwm = PWM(self._pwm, freq=self._freq)
        self._running = True
        if self._current_angle >= 0:
            self.set_angle(self._current_angle)

    def slow_move(
        self,
        target_angle: int,
        step_deg: int = 1,
        step_delay_ms: int = 15,
    ) -> None:
        """
        缓慢移动到目标角度（阻塞式）。

        参数
        ----
        target_angle  : int  目标角度（0 ~ 180°），自动钳位
        step_deg      : int  每步步进角度（建议 1 ~ 5，默认 1）
        step_delay_ms : int  每步延时毫秒数（建议 10 ~ 20，默认 15）
        """
        if not self._running:
            raise RuntimeError("舵机已停止，请先调用 start() 重新启动")
        if step_deg < 1:
            step_deg = 1

        target_angle = _servo_clamp_angle(target_angle)
        current = self._current_angle if self._current_angle >= 0 else 90

        while current != target_angle:
            if current < target_angle:
                current = min(current + step_deg, target_angle)
            else:
                current = max(current - step_deg, target_angle)
            self.set_angle(current)
            time.sleep_ms(step_delay_ms)

    # -----------------------------------------------------------------------
    # 属性
    # -----------------------------------------------------------------------
    @property
    def angle(self) -> int:
        """当前角度（只读属性，同 get_angle()）。"""
        return self._current_angle

    @property
    def running(self) -> bool:
        """PWM 输出是否正在运行。"""
        return self._running

    @property
    def pulse_min_us(self) -> int:
        """0° 对应脉宽（µs）。"""
        return self._pulse_min_us

    @property
    def pulse_max_us(self) -> int:
        """180° 对应脉宽（µs）。"""
        return self._pulse_max_us

    # -----------------------------------------------------------------------
    # 上下文管理器（with 语句自动 stop）
    # -----------------------------------------------------------------------
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.stop()

    def __repr__(self) -> str:
        state = f"angle={self._current_angle}°" if self._running else "stopped"
        return f"<MG996RServo {state}>"


# ---------------------------------------------------------------------------
# 多舵机编组辅助类
# ---------------------------------------------------------------------------
class ServoGroup:
    """
    多舵机编组，统一控制。

    示例
    ----
    group = ServoGroup([
        MG996RServo(pin=12),
        MG996RServo(pin=13),
        MG996RServo(pin=14),
    ])
    group.set_all(90)
    group.set_angles([0, 90, 180])
    group.stop_all()
    """

    def __init__(self, servos: list):
        self._servos = list(servos)

    def set_all(self, angle: int) -> None:
        """所有舵机同时转到同一角度。"""
        for s in self._servos:
            s.set_angle(angle)

    def set_angles(self, angles: list) -> None:
        """
        按索引分别设置各舵机角度。
        angles 列表长度不足时剩余舵机保持不变。
        """
        for s, a in zip(self._servos, angles):
            s.set_angle(a)

    def center_all(self) -> None:
        """所有舵机回中（90°）。"""
        for s in self._servos:
            s.center()

    def stop_all(self) -> None:
        """停止所有舵机 PWM 输出。"""
        for s in self._servos:
            s.stop()

    def get_angles(self) -> list:
        """返回所有舵机当前角度列表。"""
        return [s.get_angle() for s in self._servos]

    def __len__(self) -> int:
        return len(self._servos)

    def __getitem__(self, idx):
        return self._servos[idx]

    def __repr__(self) -> str:
        return f"<ServoGroup count={len(self._servos)} angles={self.get_angles()}>"


# ---------------------------------------------------------------------------
# 使用示例（直接运行时执行）
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import time

    # ── 单舵机基本控制 ────────────────────────────────────────────────────
    servo = MG996RServo(pin=15, init_angle=90)
    print(servo)                   # <MG996RServo angle=90°>

    servo.set_angle(0)             # 转到最小角
    time.sleep_ms(500)

    servo.set_angle(180)           # 转到最大角
    time.sleep_ms(500)

    servo.center()                 # 回中

    # ── 精细脉宽控制 ─────────────────────────────────────────────────────
    servo.set_pulse(1500)          # 约 90°
    time.sleep_ms(300)

    # ── 缓慢扫描 ─────────────────────────────────────────────────────────
    servo.slow_move(180, step_deg=2, step_delay_ms=15)
    servo.slow_move(0,   step_deg=1, step_delay_ms=10)

    # ── with 语句自动停止 ─────────────────────────────────────────────────
    with MG996RServo(pin=16) as s:
        s.set_angle(45)
        time.sleep(1)
    # 离开 with 块后自动调用 stop()

    # ── 多舵机编组 ────────────────────────────────────────────────────────
    group = ServoGroup([
        MG996RServo(pin=12),
        MG996RServo(pin=13),
        MG996RServo(pin=14),
    ])
    group.set_all(90)
    time.sleep(1)
    group.set_angles([0, 90, 180])
    time.sleep(1)
    print(group.get_angles())      # [0, 90, 180]
    group.stop_all()

    servo.stop()