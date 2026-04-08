"""
modbus_rtu_master.py  —  MicroPython Modbus RTU Master 扩展库
=============================================================
命名规则：
  - RTU Slave  → modbus_rtu_slave.py，  前缀 ModbusRTUSlave / MRTS_
  - RTU Master → 本文件，               前缀 ModbusRTUMaster / MRTM_
  - TCP Slave  → modbus_tcp_slave.py，  前缀 ModbusTCPSlave  / MTCS_
  - TCP Master → modbus_tcp_master.py， 前缀 ModbusTCPMaster / MTCM_

支持功能码：
  0x01  读线圈
  0x02  读离散输入
  0x03  读保持寄存器
  0x04  读输入寄存器
  0x05  写单个线圈
  0x06  写单个寄存器
  0x0F  写多个线圈
  0x10  写多个寄存器

依赖：machine.UART、machine.Pin（RS-485 DE 可选）
兼容平台：ESP32 / RP2040 / STM32（MicroPython v1.20+）
"""

from machine import UART, Pin
import time

# ---------------------------------------------------------------------------
# 功能码常量
# ---------------------------------------------------------------------------
MRTM_FUNC_READ_COILS        = 0x01
MRTM_FUNC_READ_DISCRETE     = 0x02
MRTM_FUNC_READ_HOLDING      = 0x03
MRTM_FUNC_READ_INPUT        = 0x04
MRTM_FUNC_WRITE_SINGLE_COIL = 0x05
MRTM_FUNC_WRITE_SINGLE_REG  = 0x06
MRTM_FUNC_WRITE_MULTI_COILS = 0x0F
MRTM_FUNC_WRITE_MULTI_REGS  = 0x10

# ---------------------------------------------------------------------------
# 异常码常量
# ---------------------------------------------------------------------------
MRTM_EX_ILLEGAL_FUNCTION    = 0x01
MRTM_EX_ILLEGAL_DATA_ADDR   = 0x02
MRTM_EX_ILLEGAL_DATA_VALUE  = 0x03
MRTM_EX_DEVICE_FAILURE      = 0x04

# ---------------------------------------------------------------------------
# 错误码（本地）
# ---------------------------------------------------------------------------
MRTM_OK                     = 0     # 成功
MRTM_ERR_TIMEOUT            = -1    # 响应超时
MRTM_ERR_CRC                = -2    # CRC 错误
MRTM_ERR_EXCEPTION          = -3    # 从站返回异常码
MRTM_ERR_INVALID_RESPONSE   = -4    # 响应帧格式错误
MRTM_ERR_INVALID_PARAM      = -5    # 参数非法

# ---------------------------------------------------------------------------
# CRC16 查表
# ---------------------------------------------------------------------------
_MRTM_CRC16_TABLE = (
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


def _mrtm_crc16(data, use_table=True):
    crc = 0xFFFF
    if use_table:
        for b in data:
            crc = (crc >> 8) ^ _MRTM_CRC16_TABLE[(crc ^ b) & 0xFF]
    else:
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


# ---------------------------------------------------------------------------
# RS-485 方向控制（与 slave 库同结构，独立命名避免依赖）
# ---------------------------------------------------------------------------
class _MRTM_RS485:
    __slots__ = ("_pin", "_active_high")

    def __init__(self, de_pin, de_active_high=True):
        self._pin = Pin(de_pin, Pin.OUT) if isinstance(de_pin, int) else de_pin
        self._active_high = de_active_high
        self.rx_mode()

    def tx_mode(self):
        self._pin.value(1 if self._active_high else 0)

    def rx_mode(self):
        self._pin.value(0 if self._active_high else 1)


# ---------------------------------------------------------------------------
# 响应结果对象
# ---------------------------------------------------------------------------
class MRTMResponse:
    """
    RTU Master 请求响应结果。

    属性
    ----
    error_code : int        MRTM_OK(0) 或 负数错误码
    exception  : int        从站异常码（error_code==MRTM_ERR_EXCEPTION 时有效）
    data       : list|None  读操作返回数据（寄存器值列表 或 bool 列表）
    raw        : bytes      原始响应帧（含 CRC）
    """
    __slots__ = ("error_code", "exception", "data", "raw")

    def __init__(self, error_code=MRTM_OK, exception=0, data=None, raw=b""):
        self.error_code = error_code
        self.exception  = exception
        self.data       = data
        self.raw        = raw

    @property
    def ok(self) -> bool:
        return self.error_code == MRTM_OK

    def __repr__(self):
        if self.ok:
            return f"<MRTMResponse OK data={self.data}>"
        elif self.error_code == MRTM_ERR_EXCEPTION:
            return f"<MRTMResponse EXCEPTION=0x{self.exception:02X}>"
        else:
            codes = {
                MRTM_ERR_TIMEOUT: "TIMEOUT",
                MRTM_ERR_CRC: "CRC_ERR",
                MRTM_ERR_INVALID_RESPONSE: "INVALID",
                MRTM_ERR_INVALID_PARAM: "BAD_PARAM",
            }
            return f"<MRTMResponse ERR={codes.get(self.error_code, self.error_code)}>"


# ---------------------------------------------------------------------------
# ModbusRTUMaster 主类
# ---------------------------------------------------------------------------
class ModbusRTUMaster:
    """
    Modbus RTU 主站（Master）。

    参数
    ----
    uart           : machine.UART  已配置好的串口实例
    rs485_de_pin   : int|Pin|None  RS-485 DE 引脚，None 则不控制方向
    de_active_high : bool          DE 极性，True=高电平发送
    response_timeout_ms : int      等待从站响应超时（默认 200ms）
    inter_frame_ms : int           帧间静默时间（默认 5ms，约 3.5 字符@9600）
    use_crc_table  : bool          True=查表法 CRC
    retries        : int           超时后重试次数（默认 0=不重试）
    buf_size       : int           收发缓冲区大小
    """

    def __init__(
        self,
        uart: UART,
        *,
        rs485_de_pin=None,
        de_active_high: bool = True,
        response_timeout_ms: int = 200,
        inter_frame_ms: int = 5,
        use_crc_table: bool = True,
        retries: int = 0,
        buf_size: int = 256,
    ):
        self._uart            = uart
        self._timeout_ms      = response_timeout_ms
        self._inter_frame_ms  = inter_frame_ms
        self._use_crc_table   = use_crc_table
        self._retries         = retries
        self._tx_buf          = bytearray(buf_size)
        self._rx_buf          = bytearray(buf_size)
        self._rs485 = (_MRTM_RS485(rs485_de_pin, de_active_high)
                       if rs485_de_pin is not None else None)

    # -----------------------------------------------------------------------
    # 公共 API — 读操作
    # -----------------------------------------------------------------------
    def read_coils(self, slave_addr: int, start: int, qty: int) -> MRTMResponse:
        """FC01 读线圈，返回 bool 列表。"""
        return self._read_bits(MRTM_FUNC_READ_COILS, slave_addr, start, qty)

    def read_discrete_inputs(self, slave_addr: int, start: int, qty: int) -> MRTMResponse:
        """FC02 读离散输入，返回 bool 列表。"""
        return self._read_bits(MRTM_FUNC_READ_DISCRETE, slave_addr, start, qty)

    def read_holding_regs(self, slave_addr: int, start: int, qty: int) -> MRTMResponse:
        """FC03 读保持寄存器，返回 int 列表（uint16）。"""
        return self._read_regs(MRTM_FUNC_READ_HOLDING, slave_addr, start, qty)

    def read_input_regs(self, slave_addr: int, start: int, qty: int) -> MRTMResponse:
        """FC04 读输入寄存器，返回 int 列表（uint16）。"""
        return self._read_regs(MRTM_FUNC_READ_INPUT, slave_addr, start, qty)

    # -----------------------------------------------------------------------
    # 公共 API — 写操作
    # -----------------------------------------------------------------------
    def write_single_coil(self, slave_addr: int, addr: int, value: bool) -> MRTMResponse:
        """FC05 写单个线圈。"""
        val_word = 0xFF00 if value else 0x0000
        return self._request_6byte(MRTM_FUNC_WRITE_SINGLE_COIL, slave_addr, addr, val_word)

    def write_single_reg(self, slave_addr: int, addr: int, value: int) -> MRTMResponse:
        """FC06 写单个保持寄存器。"""
        return self._request_6byte(MRTM_FUNC_WRITE_SINGLE_REG, slave_addr, addr, value & 0xFFFF)

    def write_multiple_coils(self, slave_addr: int, start: int, values: list) -> MRTMResponse:
        """FC0F 写多个线圈。values: bool 列表。"""
        qty = len(values)
        if qty < 1 or qty > 1968:
            return MRTMResponse(MRTM_ERR_INVALID_PARAM)
        byte_cnt = (qty + 7) >> 3
        # 构造请求帧
        idx = 0
        self._tx_buf[idx] = slave_addr;         idx += 1
        self._tx_buf[idx] = MRTM_FUNC_WRITE_MULTI_COILS; idx += 1
        self._tx_buf[idx] = (start >> 8) & 0xFF; idx += 1
        self._tx_buf[idx] = start & 0xFF;        idx += 1
        self._tx_buf[idx] = (qty >> 8) & 0xFF;   idx += 1
        self._tx_buf[idx] = qty & 0xFF;          idx += 1
        self._tx_buf[idx] = byte_cnt;            idx += 1
        for i in range(byte_cnt):
            self._tx_buf[idx + i] = 0
        for i, v in enumerate(values):
            if v:
                self._tx_buf[idx + (i >> 3)] |= (1 << (i & 7))
        idx += byte_cnt
        return self._send_recv(idx, expected_len=6)

    def write_multiple_regs(self, slave_addr: int, start: int, values: list) -> MRTMResponse:
        """FC10 写多个保持寄存器。values: int 列表（uint16）。"""
        qty = len(values)
        if qty < 1 or qty > 123:
            return MRTMResponse(MRTM_ERR_INVALID_PARAM)
        idx = 0
        self._tx_buf[idx] = slave_addr;          idx += 1
        self._tx_buf[idx] = MRTM_FUNC_WRITE_MULTI_REGS; idx += 1
        self._tx_buf[idx] = (start >> 8) & 0xFF; idx += 1
        self._tx_buf[idx] = start & 0xFF;        idx += 1
        self._tx_buf[idx] = (qty >> 8) & 0xFF;   idx += 1
        self._tx_buf[idx] = qty & 0xFF;          idx += 1
        self._tx_buf[idx] = qty * 2;             idx += 1
        for v in values:
            v &= 0xFFFF
            self._tx_buf[idx]     = (v >> 8) & 0xFF
            self._tx_buf[idx + 1] = v & 0xFF
            idx += 2
        return self._send_recv(idx, expected_len=6)

    # -----------------------------------------------------------------------
    # 内部：读位帧
    # -----------------------------------------------------------------------
    def _read_bits(self, fc, slave_addr, start, qty):
        if qty < 1 or qty > 2000:
            return MRTMResponse(MRTM_ERR_INVALID_PARAM)
        self._build_read_request(slave_addr, fc, start, qty)
        byte_cnt = (qty + 7) >> 3
        resp = self._send_recv(6, expected_len=3 + byte_cnt)
        if not resp.ok:
            return resp
        raw = resp.raw
        if len(raw) < 3 + byte_cnt + 2:
            return MRTMResponse(MRTM_ERR_INVALID_RESPONSE, raw=raw)
        result = []
        for i in range(qty):
            result.append(bool(raw[3 + (i >> 3)] & (1 << (i & 7))))
        resp.data = result
        return resp

    # -----------------------------------------------------------------------
    # 内部：读寄存器帧
    # -----------------------------------------------------------------------
    def _read_regs(self, fc, slave_addr, start, qty):
        if qty < 1 or qty > 125:
            return MRTMResponse(MRTM_ERR_INVALID_PARAM)
        self._build_read_request(slave_addr, fc, start, qty)
        resp = self._send_recv(6, expected_len=3 + qty * 2)
        if not resp.ok:
            return resp
        raw = resp.raw
        if len(raw) < 3 + qty * 2 + 2:
            return MRTMResponse(MRTM_ERR_INVALID_RESPONSE, raw=raw)
        result = []
        for i in range(qty):
            result.append((raw[3 + i * 2] << 8) | raw[4 + i * 2])
        resp.data = result
        return resp

    # -----------------------------------------------------------------------
    # 内部：6 字节标准写请求（FC05/FC06）
    # -----------------------------------------------------------------------
    def _request_6byte(self, fc, slave_addr, addr, value):
        self._tx_buf[0] = slave_addr
        self._tx_buf[1] = fc
        self._tx_buf[2] = (addr >> 8) & 0xFF
        self._tx_buf[3] = addr & 0xFF
        self._tx_buf[4] = (value >> 8) & 0xFF
        self._tx_buf[5] = value & 0xFF
        return self._send_recv(6, expected_len=6)

    # -----------------------------------------------------------------------
    # 内部：构造读请求帧头（6 字节 PDU，不含 CRC）
    # -----------------------------------------------------------------------
    def _build_read_request(self, slave_addr, fc, start, qty):
        self._tx_buf[0] = slave_addr
        self._tx_buf[1] = fc
        self._tx_buf[2] = (start >> 8) & 0xFF
        self._tx_buf[3] = start & 0xFF
        self._tx_buf[4] = (qty >> 8) & 0xFF
        self._tx_buf[5] = qty & 0xFF

    # -----------------------------------------------------------------------
    # 内部：发送请求并接收响应（含重试）
    # -----------------------------------------------------------------------
    def _send_recv(self, tx_pdu_len: int, expected_len: int) -> MRTMResponse:
        # 追加 CRC
        crc = _mrtm_crc16(memoryview(self._tx_buf)[:tx_pdu_len], self._use_crc_table)
        self._tx_buf[tx_pdu_len]     = crc & 0xFF
        self._tx_buf[tx_pdu_len + 1] = (crc >> 8) & 0xFF
        total_tx = tx_pdu_len + 2

        for attempt in range(self._retries + 1):
            resp = self._do_transaction(total_tx, expected_len)
            if resp.error_code != MRTM_ERR_TIMEOUT:
                return resp
            if attempt < self._retries:
                time.sleep_ms(self._inter_frame_ms * 2)
        return resp

    def _do_transaction(self, total_tx: int, expected_len: int) -> MRTMResponse:
        # 清空接收缓冲
        self._uart.read()

        # 发送
        if self._rs485:
            self._rs485.tx_mode()
        self._uart.write(memoryview(self._tx_buf)[:total_tx])
        if self._rs485:
            # 等待发送完毕
            baud = getattr(self._uart, "baudrate", 9600)
            time.sleep_us(total_tx * 10 * 1_000_000 // baud + 200)
            self._rs485.rx_mode()

        # 接收（预期长度 + CRC 2 字节）
        total_rx = expected_len + 2
        deadline = time.ticks_ms() + self._timeout_ms
        received = 0
        while received < total_rx:
            chunk = self._uart.readinto(
                memoryview(self._rx_buf)[received:total_rx],
                max(1, time.ticks_diff(deadline, time.ticks_ms()))
            )
            if not chunk:
                break
            received += chunk
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                break

        if received < 4:
            return MRTMResponse(MRTM_ERR_TIMEOUT)

        raw = bytes(self._rx_buf[:received])

        # CRC 校验
        rcv_crc  = (raw[-1] << 8) | raw[-2]
        calc_crc = _mrtm_crc16(raw[:-2], self._use_crc_table)
        if rcv_crc != calc_crc:
            return MRTMResponse(MRTM_ERR_CRC, raw=raw)

        # 异常响应检测（功能码最高位置 1）
        if raw[1] & 0x80:
            ex = raw[2] if received >= 3 else 0
            return MRTMResponse(MRTM_ERR_EXCEPTION, exception=ex, raw=raw)

        return MRTMResponse(MRTM_OK, raw=raw)

    def __repr__(self):
        return (
            f"<ModbusRTUMaster uart={self._uart} "
            f"timeout={self._timeout_ms}ms rs485={'yes' if self._rs485 else 'no'}>"
        )


# ---------------------------------------------------------------------------
# 使用示例
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    from machine import UART

    uart = UART(1, baudrate=9600, tx=17, rx=16, timeout=50)
    master = ModbusRTUMaster(
        uart,
        rs485_de_pin=4,
        response_timeout_ms=300,
        retries=2,
    )

    # 读从站 1 的保持寄存器 0~4
    resp = master.read_holding_regs(1, 0, 5)
    if resp.ok:
        print("寄存器值:", resp.data)
    else:
        print("读取失败:", resp)

    # 写从站 1 的寄存器 10
    resp = master.write_single_reg(1, 10, 1234)
    print("写入结果:", resp)

    # 读线圈
    resp = master.read_coils(1, 0, 8)
    if resp.ok:
        print("线圈状态:", resp.data)
