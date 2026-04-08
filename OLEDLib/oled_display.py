"""
oled_display.py  —  MicroPython OLED (SSD1306) 扩展库
=============================================================
命名规则：
  - 基础类   → OLEDDisplay
  - I2C 接口 → OLED_I2C
  - SPI 接口 → OLED_SPI

支持芯片：
  SSD1306 (128x64, 128x32)
  兼容 SH1106 (通过 offset 参数)

依赖：machine.I2C / machine.SPI, framebuf
兼容平台：ESP32 / ESP32-S3 / RP2040 (MicroPython v1.20+)
"""

import framebuf
from machine import Pin, I2C, SPI
import time

# ---------------------------------------------------------------------------
# OLED 常量定义
# ---------------------------------------------------------------------------
OLED_SET_CONTRAST        = 0x81
OLED_SET_ENTIRE_ON       = 0xA4
OLED_SET_NORM_INV        = 0xA6
OLED_SET_DISP            = 0xAE
OLED_SET_MEM_ADDR        = 0x20
OLED_SET_COL_ADDR        = 0x21
OLED_SET_PAGE_ADDR       = 0x22
OLED_SET_DISP_START_LINE = 0x40
OLED_SET_SEG_REMAP       = 0xA0
OLED_SET_MUX_RATIO       = 0xA8
OLED_SET_COM_OUT_DIR     = 0xC0
OLED_SET_DISP_OFFSET     = 0xD3
OLED_SET_COM_PIN_CFG     = 0xDA
OLED_SET_DISP_CLK_DIV    = 0xD5
OLED_SET_PRECHARGE       = 0xD9
OLED_SET_VCOM_DESEL      = 0xDB
OLED_SET_CHARGE_PUMP     = 0x8D

# ---------------------------------------------------------------------------
# 错误码
# ---------------------------------------------------------------------------
OLED_OK                  = 0
OLED_ERR_BUS             = -1

# ---------------------------------------------------------------------------
# OLEDDisplay 基类
# ---------------------------------------------------------------------------
class OLEDDisplay(framebuf.FrameBuffer):
    """
    OLED 驱动抽象基类，继承自 framebuf。
    提供标准绘图 API：text(), line(), rect(), fill_rect(), pixel(), blit() 等。
    """
    __slots__ = ("width", "height", "external_vcc", "buffer", "pages")

    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        # 初始化 framebuf: 格式为 MONO_VLSB (1位色，纵向低位起始)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        """执行 SSD1306 标准初始化序列"""
        for cmd in (
            OLED_SET_DISP,               # 关闭显示
            OLED_SET_MEM_ADDR, 0x00,     # 水平寻址模式
            OLED_SET_DISP_START_LINE,    # 起始行 0
            OLED_SET_SEG_REMAP | 0x01,   # 重映射 SEG (左右反转)
            OLED_SET_MUX_RATIO, self.height - 1,
            OLED_SET_COM_OUT_DIR | 0x08, # 重映射 COM (上下反转)
            OLED_SET_DISP_OFFSET, 0x00,
            OLED_SET_COM_PIN_CFG, 0x02 if self.width > 2 * self.height else 0x12,
            OLED_SET_DISP_CLK_DIV, 0x80,
            OLED_SET_PRECHARGE, 0x22 if self.external_vcc else 0xF1,
            OLED_SET_VCOM_DESEL, 0x30,   # 0.83*Vcc
            OLED_SET_CONTRAST, 0x7F,     # 中等亮度
            OLED_SET_ENTIRE_ON,          # 输出遵循 RAM 内容
            OLED_SET_NORM_INV,           # 正常显示（不反相）
            OLED_SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            OLED_SET_DISP | 0x01,        # 开启显示
        ):
            self.write_cmd(cmd)
        self.show()

    def poweroff(self):
        self.write_cmd(OLED_SET_DISP)

    def poweron(self):
        self.write_cmd(OLED_SET_DISP | 0x01)

    def contrast(self, contrast):
        """设置对比度 (0-255)"""
        self.write_cmd(OLED_SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        """显示反相"""
        self.write_cmd(OLED_SET_NORM_INV | (invert & 1))

    def show(self):
        """刷新缓冲区数据到屏幕"""
        self.write_cmd(OLED_SET_COL_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.width - 1)
        self.write_cmd(OLED_SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)

    def write_cmd(self, cmd):
        raise NotImplementedError

    def write_data(self, buf):
        raise NotImplementedError


# ---------------------------------------------------------------------------
# OLED_I2C 实现
# ---------------------------------------------------------------------------
class OLED_I2C(OLEDDisplay):
    """SSD1306 通过 I2C 传输"""
    __slots__ = ("_i2c", "_addr", "_temp")

    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self._i2c = i2c
        self._addr = addr
        self._temp = bytearray(2)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self._temp[0] = 0x80  # Co=1, D/C#=0 (指令控制字节)
        self._temp[1] = cmd
        try:
            self._i2c.writeto(self._addr, self._temp)
        except OSError:
            return OLED_ERR_BUS

    def write_data(self, buf):
        # I2C 传输数据时，首字节 0x40 表示后续为数据
        self._i2c.writeto(self._addr, b'\x40' + buf)


# ---------------------------------------------------------------------------
# OLED_SPI 实现
# ---------------------------------------------------------------------------
class OLED_SPI(OLEDDisplay):
    """SSD1306 通过 SPI 传输"""
    __slots__ = ("_spi", "_dc", "_res", "_cs")

    def __init__(self, width, height, spi, dc, res, cs=None, external_vcc=False):
        self._spi = spi
        self._dc = dc   # Data/Command pin
        self._res = res # Reset pin
        self._cs = cs   # Chip Select pin
        
        self._dc.init(Pin.OUT, value=0)
        self._res.init(Pin.OUT, value=0)
        if self._cs:
            self._cs.init(Pin.OUT, value=1)
            
        # 硬件复位过程
        self._res.value(1)
        time.sleep_ms(1)
        self._res.value(0)
        time.sleep_ms(10)
        self._res.value(1)
        
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self._dc.value(0) # 指令模式
        if self._cs: self._cs.value(0)
        self._spi.write(bytearray([cmd]))
        if self._cs: self._cs.value(1)

    def write_data(self, buf):
        self._dc.value(1) # 数据模式
        if self._cs: self._cs.value(0)
        self._spi.write(buf)
        if self._cs: self._cs.value(1)


# ---------------------------------------------------------------------------
# 使用示例
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # --- 场景 1: 使用 ESP32-S3 的 I2C 接口 ---
    from machine import I2C, Pin
    
    # 初始化 I2C
    i2c = I2C(0, scl=Pin(18), sda=Pin(17), freq=400000)
    
    # 创建 OLED 对象
    oled = OLED_I2C(128, 64, i2c)
    
    # 绘图测试
    oled.fill(0)                   # 清屏
    oled.text("ESP32-S3 Modbus", 0, 0, 1)
    oled.line(0, 12, 127, 12, 1)   # 画线
    oled.rect(10, 20, 50, 30, 1)   # 画矩形
    oled.fill_rect(70, 20, 40, 30, 1) # 实心矩形
    
    # 特色展示：显示 Modbus 状态（模拟）
    status = "Online"
    oled.text(f"Status: {status}", 0, 56, 1)
    
    oled.show()  # 必须调用此方法，数据才会刷入屏幕

    # --- 场景 2: 使用 SPI 接口 (如果硬件支持) ---
    """
    from machine import SPI, Pin
    spi = SPI(1, baudrate=8000000, polarity=0, phase=0)
    oled_spi = OLED_SPI(128, 64, spi, dc=Pin(2), res=Pin(3), cs=Pin(4))
    oled_spi.text("SPI Mode", 0, 0, 1)
    oled_spi.show()
    """