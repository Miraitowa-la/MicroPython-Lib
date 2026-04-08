"""
modbus_rtu_slave.py  —  MicroPython Modbus RTU Slave 扩展库
============================================================
命名规则：
  - RTU Slave  → 本文件，前缀 ModbusRTUSlave / mrts_*
  - RTU Master → modbus_rtu_master.py，前缀 ModbusRTUMaster / mrtm_*
  - TCP Slave  → modbus_tcp_slave.py，  前缀 ModbusTCPSlave / mtcs_*
  - TCP Master → modbus_tcp_master.py， 前缀 ModbusTCPMaster / mtcm_*

支持功能码：
  0x01  读线圈
  0x02  读离散输入
  0x03  读保持寄存器
  0x04  读输入寄存器
  0x05  写单个线圈
  0x06  写单个寄存器
  0x0F  写多个线圈
  0x10  写多个寄存器
  0x64  自定义配置（可选回调）

依赖：machine.UART、machine.Pin（RS-485 DE 控制可选）
兼容平台：ESP32 / RP2040 / STM32（MicroPython v1.20+）

用法示例见文件末尾 __main__ 块。
"""

import struct
from machine import UART, Pin

# ---------------------------------------------------------------------------
# 功能码常量
# ---------------------------------------------------------------------------
MRTS_FUNC_READ_COILS        = 0x01
MRTS_FUNC_READ_DISCRETE     = 0x02
MRTS_FUNC_READ_HOLDING      = 0x03
MRTS_FUNC_READ_INPUT        = 0x04
MRTS_FUNC_WRITE_SINGLE_COIL = 0x05
MRTS_FUNC_WRITE_SINGLE_REG  = 0x06
MRTS_FUNC_WRITE_MULTI_COILS = 0x0F
MRTS_FUNC_WRITE_MULTI_REGS  = 0x10
MRTS_FUNC_CUSTOM_CONFIG     = 0x64

# ---------------------------------------------------------------------------
# 异常码常量
# ---------------------------------------------------------------------------
MRTS_EX_ILLEGAL_FUNCTION    = 0x01
MRTS_EX_ILLEGAL_DATA_ADDR   = 0x02
MRTS_EX_ILLEGAL_DATA_VALUE  = 0x03
MRTS_EX_DEVICE_FAILURE      = 0x04

# ---------------------------------------------------------------------------
# CRC16 查表（Modbus 标准多项式 0xA001）
# ---------------------------------------------------------------------------
_MRTS_CRC16_TABLE = (
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
    0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
    0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
    0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
    0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
    0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
    0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
    0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
    0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
    0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
    0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
    0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
    0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
    0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
    0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
    0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040,
)


def _mrts_crc16(data: bytes | bytearray, use_table: bool = True) -> int:
    """计算 Modbus CRC16。use_table=True 用查表法（快），False 用移位法（省内存）。"""
    crc = 0xFFFF
    if use_table:
        for b in data:
            crc = (crc >> 8) ^ _MRTS_CRC16_TABLE[(crc ^ b) & 0xFF]
    else:
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
    return crc


# ---------------------------------------------------------------------------
# 数据区：线圈 / 离散输入用 bytearray 位压缩，寄存器用 array('H')
# ---------------------------------------------------------------------------
class MRTSDataMap:
    """
    Modbus RTU Slave 数据区映射。

    所有数组均可在初始化后直接读写，作为设备应用层与协议层的共享数据。

    参数
    ----
    coil_count         : int  线圈数量（位，内部按字节存储）
    discrete_count     : int  离散输入数量
    holding_reg_count  : int  保持寄存器数量（uint16）
    input_reg_count    : int  输入寄存器数量（uint16）
    """
    __slots__ = (
        "coils", "coil_count",
        "discrete_inputs", "discrete_count",
        "holding_regs", "holding_reg_count",
        "input_regs", "input_reg_count",
    )

    def __init__(
        self,
        coil_count: int = 0,
        discrete_count: int = 0,
        holding_reg_count: int = 0,
        input_reg_count: int = 0,
    ):
        self.coil_count         = coil_count
        self.discrete_count     = discrete_count
        self.holding_reg_count  = holding_reg_count
        self.input_reg_count    = input_reg_count

        # 位存储：每字节存 8 位
        self.coils          = bytearray((coil_count    + 7) // 8) if coil_count    else None
        self.discrete_inputs= bytearray((discrete_count+ 7) // 8) if discrete_count else None

        # 寄存器：使用 array 节省内存（MicroPython 支持）
        try:
            from array import array
            self.holding_regs = array("H", [0] * holding_reg_count) if holding_reg_count else None
            self.input_regs   = array("H", [0] * input_reg_count)   if input_reg_count   else None
        except ImportError:
            # 退回 list
            self.holding_regs = [0] * holding_reg_count if holding_reg_count else None
            self.input_regs   = [0] * input_reg_count   if input_reg_count   else None

    # ---- 位操作辅助 --------------------------------------------------------
    @staticmethod
    def get_bit(buf: bytearray, idx: int) -> bool:
        return bool(buf[idx >> 3] & (1 << (idx & 7)))

    @staticmethod
    def set_bit(buf: bytearray, idx: int, val: bool) -> None:
        if val:
            buf[idx >> 3] |= (1 << (idx & 7))
        else:
            buf[idx >> 3] &= ~(1 << (idx & 7))


# ---------------------------------------------------------------------------
# RS-485 方向控制辅助类
# ---------------------------------------------------------------------------
class _MRTS_RS485:
    """RS-485 DE（发送使能）引脚管理。de_active_high=True 表示高电平发送。"""
    __slots__ = ("_pin", "_active_high")

    def __init__(self, de_pin: int | Pin, de_active_high: bool = True):
        if isinstance(de_pin, int):
            self._pin = Pin(de_pin, Pin.OUT)
        else:
            self._pin = de_pin
        self._active_high = de_active_high
        self.rx_mode()

    def tx_mode(self):
        self._pin.value(1 if self._active_high else 0)

    def rx_mode(self):
        self._pin.value(0 if self._active_high else 1)


# ---------------------------------------------------------------------------
# ModbusRTUSlave 主类
# ---------------------------------------------------------------------------
class ModbusRTUSlave:
    """
    Modbus RTU 从站（Slave）。

    参数
    ----
    uart          : machine.UART  已配置好的串口实例
    slave_addr    : int           从站地址 1~247
    data_map      : MRTSDataMap   数据区映射对象
    rs485_de_pin  : int|Pin|None  RS-485 DE 引脚，None 则不控制方向
    de_active_high: bool          DE 极性，True=高电平发送
    use_crc_table : bool          True=查表法 CRC（快），False=移位法（省内存）
    rx_timeout_ms : int           帧间超时，单位毫秒（默认 10ms，约 3.5 字符时间@9600）
    buf_size      : int           收发缓冲区大小（字节）
    write_cb      : callable|None 写操作前回调 fn(slave, func_code, start, qty)->bool
    custom_cb     : callable|None 0x64 自定义配置回调 fn(slave, param_addr, param_val)->bool
    user_data     : any           用户自定义数据，协议栈不使用
    """

    def __init__(
        self,
        uart: UART,
        slave_addr: int,
        data_map: MRTSDataMap,
        *,
        rs485_de_pin=None,
        de_active_high: bool = True,
        use_crc_table: bool = True,
        rx_timeout_ms: int = 10,
        buf_size: int = 256,
        write_cb=None,
        custom_cb=None,
        user_data=None,
    ):
        if not (1 <= slave_addr <= 247):
            raise ValueError("slave_addr 必须在 1~247 范围内")

        self._uart          = uart
        self._slave_addr    = slave_addr
        self._data_map      = data_map
        self._use_crc_table = use_crc_table
        self._rx_timeout_ms = rx_timeout_ms
        self._write_cb      = write_cb
        self._custom_cb     = custom_cb
        self.user_data      = user_data

        # RS-485 方向控制
        self._rs485 = (_MRTS_RS485(rs485_de_pin, de_active_high)
                       if rs485_de_pin is not None else None)

        # 收发缓冲区
        self._rx_buf = bytearray(buf_size)
        self._tx_buf = bytearray(buf_size)

    # -----------------------------------------------------------------------
    # 属性
    # -----------------------------------------------------------------------
    @property
    def slave_addr(self) -> int:
        return self._slave_addr

    @slave_addr.setter
    def slave_addr(self, addr: int):
        if not (1 <= addr <= 247):
            raise ValueError("slave_addr 必须在 1~247 范围内")
        self._slave_addr = addr

    @property
    def data_map(self) -> MRTSDataMap:
        return self._data_map

    # -----------------------------------------------------------------------
    # 核心：主循环处理（非阻塞，应在 while True 中持续调用）
    # -----------------------------------------------------------------------
    def process(self) -> bool:
        """
        非阻塞轮询，读取并处理一帧请求。
        返回 True 表示本次处理了一帧，False 表示无数据。
        """
        rx_len = self._recv_frame()
        if rx_len < 4:
            return False

        buf = self._rx_buf

        # 地址过滤（0xFF = 广播）
        req_addr = buf[0]
        if req_addr != self._slave_addr and req_addr != 0xFF:
            return False

        # CRC 校验
        rcv_crc = (buf[rx_len - 1] << 8) | buf[rx_len - 2]
        calc_crc = _mrts_crc16(memoryview(buf)[:rx_len - 2], self._use_crc_table)
        if rcv_crc != calc_crc:
            return False

        func_code = buf[1]
        self._tx_buf[0] = self._slave_addr
        self._tx_buf[1] = func_code

        self._dispatch(func_code, buf, rx_len)
        return True

    # -----------------------------------------------------------------------
    # 帧接收（阻塞式读取，依赖 UART timeout 实现帧间隔检测）
    # -----------------------------------------------------------------------
    def _recv_frame(self) -> int:
        """读取一帧，返回实际字节数。"""
        # 先读第 1 字节以触发超时计时
        n = self._uart.readinto(memoryview(self._rx_buf)[:1])
        if not n:
            return 0
        total = 1
        while True:
            chunk = self._uart.readinto(
                memoryview(self._rx_buf)[total:], self._rx_timeout_ms
            )
            if not chunk:
                break
            total += chunk
            if total >= len(self._rx_buf):
                break
        return total

    # -----------------------------------------------------------------------
    # 功能码分发
    # -----------------------------------------------------------------------
    def _dispatch(self, func_code: int, buf: bytearray, rx_len: int):
        dm = self._data_map

        if func_code == MRTS_FUNC_READ_COILS:
            self._handle_read_bits(func_code, buf, dm.coils, dm.coil_count, 2000)

        elif func_code == MRTS_FUNC_READ_DISCRETE:
            self._handle_read_bits(func_code, buf, dm.discrete_inputs, dm.discrete_count, 2000)

        elif func_code == MRTS_FUNC_READ_HOLDING:
            self._handle_read_regs(func_code, buf, dm.holding_regs, dm.holding_reg_count)

        elif func_code == MRTS_FUNC_READ_INPUT:
            self._handle_read_regs(func_code, buf, dm.input_regs, dm.input_reg_count)

        elif func_code == MRTS_FUNC_WRITE_SINGLE_COIL:
            self._handle_write_single_coil(buf)

        elif func_code == MRTS_FUNC_WRITE_SINGLE_REG:
            self._handle_write_single_reg(buf)

        elif func_code == MRTS_FUNC_WRITE_MULTI_COILS:
            self._handle_write_multi_coils(buf, rx_len)

        elif func_code == MRTS_FUNC_WRITE_MULTI_REGS:
            self._handle_write_multi_regs(buf, rx_len)

        elif func_code == MRTS_FUNC_CUSTOM_CONFIG:
            self._handle_custom_config(buf, rx_len)

        else:
            self._send_exception(func_code, MRTS_EX_ILLEGAL_FUNCTION)

    # -----------------------------------------------------------------------
    # 0x01 / 0x02  读位（线圈 / 离散输入）
    # -----------------------------------------------------------------------
    def _handle_read_bits(self, fc, buf, bit_buf, bit_count, max_qty):
        if bit_buf is None or bit_count == 0:
            self._send_exception(fc, MRTS_EX_ILLEGAL_FUNCTION)
            return
        start = (buf[2] << 8) | buf[3]
        qty   = (buf[4] << 8) | buf[5]
        if qty < 1 or qty > max_qty:
            self._send_exception(fc, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if start + qty > bit_count:
            self._send_exception(fc, MRTS_EX_ILLEGAL_DATA_ADDR)
            return
        byte_cnt = (qty + 7) >> 3
        self._tx_buf[2] = byte_cnt
        # 清零响应位区域
        for i in range(byte_cnt):
            self._tx_buf[3 + i] = 0
        for i in range(qty):
            idx = start + i
            if bit_buf[idx >> 3] & (1 << (idx & 7)):
                self._tx_buf[3 + (i >> 3)] |= (1 << (i & 7))
        self._send_response(3 + byte_cnt)

    # -----------------------------------------------------------------------
    # 0x03 / 0x04  读寄存器（保持 / 输入）
    # -----------------------------------------------------------------------
    def _handle_read_regs(self, fc, buf, reg_buf, reg_count):
        if reg_buf is None or reg_count == 0:
            self._send_exception(fc, MRTS_EX_ILLEGAL_FUNCTION)
            return
        start = (buf[2] << 8) | buf[3]
        qty   = (buf[4] << 8) | buf[5]
        if qty < 1 or qty > 125:
            self._send_exception(fc, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if start + qty > reg_count:
            self._send_exception(fc, MRTS_EX_ILLEGAL_DATA_ADDR)
            return
        self._tx_buf[2] = qty * 2
        for i in range(qty):
            v = reg_buf[start + i]
            self._tx_buf[3 + i * 2] = (v >> 8) & 0xFF
            self._tx_buf[4 + i * 2] = v & 0xFF
        self._send_response(3 + qty * 2)

    # -----------------------------------------------------------------------
    # 0x05  写单个线圈
    # -----------------------------------------------------------------------
    def _handle_write_single_coil(self, buf):
        dm = self._data_map
        if dm.coils is None or dm.coil_count == 0:
            self._send_exception(MRTS_FUNC_WRITE_SINGLE_COIL, MRTS_EX_ILLEGAL_FUNCTION)
            return
        addr = (buf[2] << 8) | buf[3]
        val  = (buf[4] << 8) | buf[5]
        if val not in (0x0000, 0xFF00):
            self._send_exception(MRTS_FUNC_WRITE_SINGLE_COIL, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if addr >= dm.coil_count:
            self._send_exception(MRTS_FUNC_WRITE_SINGLE_COIL, MRTS_EX_ILLEGAL_DATA_ADDR)
            return
        if not self._invoke_write_cb(MRTS_FUNC_WRITE_SINGLE_COIL, addr, 1):
            return
        MRTSDataMap.set_bit(dm.coils, addr, val == 0xFF00)
        # 回显请求帧前 6 字节
        self._tx_buf[:6] = buf[:6]
        self._tx_buf[0] = self._slave_addr
        self._send_response(6)

    # -----------------------------------------------------------------------
    # 0x06  写单个保持寄存器
    # -----------------------------------------------------------------------
    def _handle_write_single_reg(self, buf):
        dm = self._data_map
        if dm.holding_regs is None or dm.holding_reg_count == 0:
            self._send_exception(MRTS_FUNC_WRITE_SINGLE_REG, MRTS_EX_ILLEGAL_FUNCTION)
            return
        addr = (buf[2] << 8) | buf[3]
        val  = (buf[4] << 8) | buf[5]
        if addr >= dm.holding_reg_count:
            self._send_exception(MRTS_FUNC_WRITE_SINGLE_REG, MRTS_EX_ILLEGAL_DATA_ADDR)
            return
        if not self._invoke_write_cb(MRTS_FUNC_WRITE_SINGLE_REG, addr, 1):
            return
        dm.holding_regs[addr] = val
        self._tx_buf[:6] = buf[:6]
        self._tx_buf[0] = self._slave_addr
        self._send_response(6)

    # -----------------------------------------------------------------------
    # 0x0F  写多个线圈
    # -----------------------------------------------------------------------
    def _handle_write_multi_coils(self, buf, rx_len):
        dm = self._data_map
        if dm.coils is None or dm.coil_count == 0:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_COILS, MRTS_EX_ILLEGAL_FUNCTION)
            return
        start    = (buf[2] << 8) | buf[3]
        qty      = (buf[4] << 8) | buf[5]
        byte_cnt = buf[6]
        if qty < 1 or qty > 1968:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_COILS, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if start + qty > dm.coil_count:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_COILS, MRTS_EX_ILLEGAL_DATA_ADDR)
            return
        if rx_len < 7 + byte_cnt + 2:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_COILS, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if not self._invoke_write_cb(MRTS_FUNC_WRITE_MULTI_COILS, start, qty):
            return
        for i in range(qty):
            bit_val = bool(buf[7 + (i >> 3)] & (1 << (i & 7)))
            MRTSDataMap.set_bit(dm.coils, start + i, bit_val)
        self._tx_buf[2] = buf[2]; self._tx_buf[3] = buf[3]
        self._tx_buf[4] = buf[4]; self._tx_buf[5] = buf[5]
        self._send_response(6)

    # -----------------------------------------------------------------------
    # 0x10  写多个保持寄存器
    # -----------------------------------------------------------------------
    def _handle_write_multi_regs(self, buf, rx_len):
        dm = self._data_map
        if dm.holding_regs is None or dm.holding_reg_count == 0:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_REGS, MRTS_EX_ILLEGAL_FUNCTION)
            return
        start    = (buf[2] << 8) | buf[3]
        qty      = (buf[4] << 8) | buf[5]
        if qty < 1 or qty > 123:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_REGS, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if start + qty > dm.holding_reg_count:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_REGS, MRTS_EX_ILLEGAL_DATA_ADDR)
            return
        if rx_len < 7 + qty * 2 + 2:
            self._send_exception(MRTS_FUNC_WRITE_MULTI_REGS, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if not self._invoke_write_cb(MRTS_FUNC_WRITE_MULTI_REGS, start, qty):
            return
        for i in range(qty):
            dm.holding_regs[start + i] = (buf[7 + i * 2] << 8) | buf[8 + i * 2]
        self._tx_buf[2] = buf[2]; self._tx_buf[3] = buf[3]
        self._tx_buf[4] = buf[4]; self._tx_buf[5] = buf[5]
        self._send_response(6)

    # -----------------------------------------------------------------------
    # 0x64  自定义配置
    # -----------------------------------------------------------------------
    def _handle_custom_config(self, buf, rx_len):
        if rx_len != 8:   # addr(1)+fc(1)+param_addr(2)+param_val(2)+crc(2)
            self._send_exception(MRTS_FUNC_CUSTOM_CONFIG, MRTS_EX_ILLEGAL_DATA_VALUE)
            return
        if self._custom_cb is None:
            self._send_exception(MRTS_FUNC_CUSTOM_CONFIG, MRTS_EX_ILLEGAL_FUNCTION)
            return
        param_addr = (buf[2] << 8) | buf[3]
        param_val  = (buf[4] << 8) | buf[5]
        if self._custom_cb(self, param_addr, param_val):
            self._tx_buf[:6] = buf[:6]
            self._tx_buf[0] = self._slave_addr
            self._send_response(6)
        else:
            self._send_exception(MRTS_FUNC_CUSTOM_CONFIG, MRTS_EX_ILLEGAL_DATA_VALUE)

    # -----------------------------------------------------------------------
    # 写入回调辅助
    # -----------------------------------------------------------------------
    def _invoke_write_cb(self, fc: int, start: int, qty: int) -> bool:
        if self._write_cb is None:
            return True
        if not self._write_cb(self, fc, start, qty):
            self._send_exception(fc, MRTS_EX_DEVICE_FAILURE)
            return False
        return True

    # -----------------------------------------------------------------------
    # 发送辅助
    # -----------------------------------------------------------------------
    def _send_response(self, length: int):
        """追加 CRC 并发送响应帧。"""
        crc = _mrts_crc16(memoryview(self._tx_buf)[:length], self._use_crc_table)
        self._tx_buf[length]     = crc & 0xFF
        self._tx_buf[length + 1] = (crc >> 8) & 0xFF
        total = length + 2
        if self._rs485:
            self._rs485.tx_mode()
        self._uart.write(memoryview(self._tx_buf)[:total])
        if self._rs485:
            # 等待发送完成再切回接收（轮询 txdone，兼容性好）
            # machine.UART 无统一"发送完成"接口，通过足够长的延时保证
            import time
            bit_us = 1_000_000 // self._uart.baudrate if hasattr(self._uart, "baudrate") else 104
            time.sleep_us(total * 10 * bit_us + 200)
            self._rs485.rx_mode()

    def _send_exception(self, fc: int, ex_code: int):
        self._tx_buf[0] = self._slave_addr
        self._tx_buf[1] = fc | 0x80
        self._tx_buf[2] = ex_code
        self._send_response(3)

    # -----------------------------------------------------------------------
    # 便捷属性：直接访问寄存器
    # -----------------------------------------------------------------------
    def get_holding_reg(self, addr: int) -> int:
        dm = self._data_map
        if dm.holding_regs is None or addr >= dm.holding_reg_count:
            raise IndexError(f"保持寄存器地址 {addr} 越界")
        return dm.holding_regs[addr]

    def set_holding_reg(self, addr: int, value: int):
        dm = self._data_map
        if dm.holding_regs is None or addr >= dm.holding_reg_count:
            raise IndexError(f"保持寄存器地址 {addr} 越界")
        dm.holding_regs[addr] = value & 0xFFFF

    def get_input_reg(self, addr: int) -> int:
        dm = self._data_map
        if dm.input_regs is None or addr >= dm.input_reg_count:
            raise IndexError(f"输入寄存器地址 {addr} 越界")
        return dm.input_regs[addr]

    def set_input_reg(self, addr: int, value: int):
        dm = self._data_map
        if dm.input_regs is None or addr >= dm.input_reg_count:
            raise IndexError(f"输入寄存器地址 {addr} 越界")
        dm.input_regs[addr] = value & 0xFFFF

    def get_coil(self, addr: int) -> bool:
        dm = self._data_map
        if dm.coils is None or addr >= dm.coil_count:
            raise IndexError(f"线圈地址 {addr} 越界")
        return MRTSDataMap.get_bit(dm.coils, addr)

    def set_coil(self, addr: int, value: bool):
        dm = self._data_map
        if dm.coils is None or addr >= dm.coil_count:
            raise IndexError(f"线圈地址 {addr} 越界")
        MRTSDataMap.set_bit(dm.coils, addr, value)

    def get_discrete(self, addr: int) -> bool:
        dm = self._data_map
        if dm.discrete_inputs is None or addr >= dm.discrete_count:
            raise IndexError(f"离散输入地址 {addr} 越界")
        return MRTSDataMap.get_bit(dm.discrete_inputs, addr)

    def set_discrete(self, addr: int, value: bool):
        dm = self._data_map
        if dm.discrete_inputs is None or addr >= dm.discrete_count:
            raise IndexError(f"离散输入地址 {addr} 越界")
        MRTSDataMap.set_bit(dm.discrete_inputs, addr, value)

    def __repr__(self) -> str:
        return (
            f"<ModbusRTUSlave addr={self._slave_addr} "
            f"uart={self._uart} rs485={'yes' if self._rs485 else 'no'}>"
        )


# ---------------------------------------------------------------------------
# 使用示例（在开发板上运行 python -c "import modbus_rtu_slave" 不会执行）
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # ---- 1. 创建数据区 -------------------------------------------------------
    dm = MRTSDataMap(
        coil_count        = 16,   # 16 个线圈
        discrete_count    = 8,    # 8 个离散输入（只读）
        holding_reg_count = 32,   # 32 个保持寄存器
        input_reg_count   = 8,    # 8 个输入寄存器（只读）
    )
    # 预置一些初始值
    dm.holding_regs[0] = 0x1234
    dm.input_regs[0]   = 100
    MRTSDataMap.set_bit(dm.coils, 0, True)

    # ---- 2. 写入前回调（可选，做权限控制） ------------------------------------
    def my_write_cb(slave, func_code, start_addr, quantity):
        # 保持寄存器 0~3 只读，拒绝写入
        if func_code in (MRTS_FUNC_WRITE_SINGLE_REG, MRTS_FUNC_WRITE_MULTI_REGS):
            if start_addr < 4:
                return False
        return True

    # ---- 3. 自定义配置回调（可选，处理 0x64） --------------------------------
    def my_custom_cb(slave, param_addr, param_val):
        print(f"自定义配置: addr=0x{param_addr:04X} val=0x{param_val:04X}")
        return True  # 返回 True 表示执行成功

    # ---- 4. 初始化 UART（以 ESP32 为例） -------------------------------------
    uart = UART(1, baudrate=9600, tx=17, rx=16, timeout=10)

    # ---- 5. 创建从站实例 -----------------------------------------------------
    slave = ModbusRTUSlave(
        uart          = uart,
        slave_addr    = 1,
        data_map      = dm,
        rs485_de_pin  = 4,        # RS-485 DE 接 GPIO4，不需要则删除此行
        de_active_high= True,
        use_crc_table = True,
        write_cb      = my_write_cb,
        custom_cb     = my_custom_cb,
    )

    # ---- 6. 主循环 -----------------------------------------------------------
    import time
    print("Modbus RTU Slave 启动，地址:", slave.slave_addr)
    while True:
        slave.process()        # 非阻塞：有帧则处理，无帧立即返回
        # 在这里更新传感器值到 input_regs / discrete_inputs
        # slave.set_input_reg(0, read_sensor())
        time.sleep_ms(1)
