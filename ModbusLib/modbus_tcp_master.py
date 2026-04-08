"""
modbus_tcp_master.py  —  MicroPython Modbus TCP Master 扩展库
=============================================================
命名规则：
  - RTU Slave  → modbus_rtu_slave.py，  前缀 ModbusRTUSlave / MRTS_
  - RTU Master → modbus_rtu_master.py， 前缀 ModbusRTUMaster / MRTM_
  - TCP Slave  → modbus_tcp_slave.py，  前缀 ModbusTCPSlave  / MTCS_
  - TCP Master → 本文件，               前缀 ModbusTCPMaster / MTCM_

支持功能码：
  0x01  读线圈
  0x02  读离散输入
  0x03  读保持寄存器
  0x04  读输入寄存器
  0x05  写单个线圈
  0x06  写单个寄存器
  0x0F  写多个线圈
  0x10  写多个寄存器

Modbus TCP MBAP 头：
  Transaction ID (2) | Protocol ID (2, 固定 0) |
  Length (2) | Unit ID (1) | FC (1) | Data (N)

连接策略：
  - 懒连接：首次请求时建立连接
  - keep_alive=True（默认）：复用连接；False：每次请求后断开
  - 连接失败或断开后自动重连

依赖：usocket（network 已连接）
兼容平台：ESP32 / RP2040 / W5500（MicroPython v1.20+）
"""

import usocket as socket
import time

# ---------------------------------------------------------------------------
# 功能码常量
# ---------------------------------------------------------------------------
MTCM_FUNC_READ_COILS        = 0x01
MTCM_FUNC_READ_DISCRETE     = 0x02
MTCM_FUNC_READ_HOLDING      = 0x03
MTCM_FUNC_READ_INPUT        = 0x04
MTCM_FUNC_WRITE_SINGLE_COIL = 0x05
MTCM_FUNC_WRITE_SINGLE_REG  = 0x06
MTCM_FUNC_WRITE_MULTI_COILS = 0x0F
MTCM_FUNC_WRITE_MULTI_REGS  = 0x10

# ---------------------------------------------------------------------------
# 异常码常量
# ---------------------------------------------------------------------------
MTCM_EX_ILLEGAL_FUNCTION    = 0x01
MTCM_EX_ILLEGAL_DATA_ADDR   = 0x02
MTCM_EX_ILLEGAL_DATA_VALUE  = 0x03
MTCM_EX_DEVICE_FAILURE      = 0x04

# ---------------------------------------------------------------------------
# 本地错误码
# ---------------------------------------------------------------------------
MTCM_OK                     = 0
MTCM_ERR_TIMEOUT            = -1
MTCM_ERR_CONNECT            = -2
MTCM_ERR_EXCEPTION          = -3
MTCM_ERR_INVALID_RESPONSE   = -4
MTCM_ERR_INVALID_PARAM      = -5

_MTCM_PROTOCOL_ID = 0x0000


# ---------------------------------------------------------------------------
# 响应结果对象
# ---------------------------------------------------------------------------
class MTCMResponse:
    """
    TCP Master 请求响应结果。

    属性
    ----
    error_code : int        MTCM_OK(0) 或 负数错误码
    exception  : int        从站异常码（error_code==MTCM_ERR_EXCEPTION 时有效）
    data       : list|None  读操作返回的数据列表
    raw        : bytes      原始响应 PDU（不含 MBAP）
    """
    __slots__ = ("error_code", "exception", "data", "raw")

    def __init__(self, error_code=MTCM_OK, exception=0, data=None, raw=b""):
        self.error_code = error_code
        self.exception  = exception
        self.data       = data
        self.raw        = raw

    @property
    def ok(self) -> bool:
        return self.error_code == MTCM_OK

    def __repr__(self):
        if self.ok:
            return f"<MTCMResponse OK data={self.data}>"
        elif self.error_code == MTCM_ERR_EXCEPTION:
            return f"<MTCMResponse EXCEPTION=0x{self.exception:02X}>"
        else:
            codes = {
                MTCM_ERR_TIMEOUT:          "TIMEOUT",
                MTCM_ERR_CONNECT:          "CONNECT_ERR",
                MTCM_ERR_INVALID_RESPONSE: "INVALID",
                MTCM_ERR_INVALID_PARAM:    "BAD_PARAM",
            }
            return f"<MTCMResponse ERR={codes.get(self.error_code, self.error_code)}>"


# ---------------------------------------------------------------------------
# ModbusTCPMaster 主类
# ---------------------------------------------------------------------------
class ModbusTCPMaster:
    """
    Modbus TCP 主站（Master / Client）。

    参数
    ----
    host               : str   从站 IP 地址
    port               : int   从站端口（默认 502）
    unit_id            : int   Unit ID（默认 1）
    connect_timeout_ms : int   TCP 连接超时（默认 3000ms）
    response_timeout_ms: int   等待响应超时（默认 1000ms）
    keep_alive         : bool  是否保持连接（默认 True）
    retries            : int   超时后重试次数（默认 1）
    buf_size           : int   收发缓冲区（字节）
    """

    def __init__(
        self,
        host: str,
        port: int = 502,
        unit_id: int = 1,
        *,
        connect_timeout_ms: int = 3000,
        response_timeout_ms: int = 1000,
        keep_alive: bool = True,
        retries: int = 1,
        buf_size: int = 300,
    ):
        self._host            = host
        self._port            = port
        self._unit_id         = unit_id
        self._conn_timeout_ms = connect_timeout_ms
        self._resp_timeout_ms = response_timeout_ms
        self._keep_alive      = keep_alive
        self._retries         = retries
        self._buf_size        = buf_size

        self._sock      = None
        self._trans_id  = 0          # 事务 ID，自增
        self._tx_buf    = bytearray(buf_size)
        self._rx_buf    = bytearray(buf_size)

    # -----------------------------------------------------------------------
    # 公共 API — 读操作
    # -----------------------------------------------------------------------
    def read_coils(self, start: int, qty: int, unit_id: int = None) -> MTCMResponse:
        """FC01 读线圈，返回 bool 列表。"""
        return self._read_bits(MTCM_FUNC_READ_COILS, start, qty, unit_id)

    def read_discrete_inputs(self, start: int, qty: int, unit_id: int = None) -> MTCMResponse:
        """FC02 读离散输入，返回 bool 列表。"""
        return self._read_bits(MTCM_FUNC_READ_DISCRETE, start, qty, unit_id)

    def read_holding_regs(self, start: int, qty: int, unit_id: int = None) -> MTCMResponse:
        """FC03 读保持寄存器，返回 int 列表（uint16）。"""
        return self._read_regs(MTCM_FUNC_READ_HOLDING, start, qty, unit_id)

    def read_input_regs(self, start: int, qty: int, unit_id: int = None) -> MTCMResponse:
        """FC04 读输入寄存器，返回 int 列表（uint16）。"""
        return self._read_regs(MTCM_FUNC_READ_INPUT, start, qty, unit_id)

    # -----------------------------------------------------------------------
    # 公共 API — 写操作
    # -----------------------------------------------------------------------
    def write_single_coil(self, addr: int, value: bool, unit_id: int = None) -> MTCMResponse:
        """FC05 写单个线圈。"""
        val_word = 0xFF00 if value else 0x0000
        pdu = bytes([
            MTCM_FUNC_WRITE_SINGLE_COIL,
            (addr >> 8) & 0xFF, addr & 0xFF,
            (val_word >> 8) & 0xFF, val_word & 0xFF,
        ])
        return self._transaction(pdu, expected_pdu_len=5, unit_id=unit_id)

    def write_single_reg(self, addr: int, value: int, unit_id: int = None) -> MTCMResponse:
        """FC06 写单个保持寄存器。"""
        value &= 0xFFFF
        pdu = bytes([
            MTCM_FUNC_WRITE_SINGLE_REG,
            (addr >> 8) & 0xFF, addr & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF,
        ])
        return self._transaction(pdu, expected_pdu_len=5, unit_id=unit_id)

    def write_multiple_coils(self, start: int, values: list, unit_id: int = None) -> MTCMResponse:
        """FC0F 写多个线圈。values: bool 列表。"""
        qty = len(values)
        if qty < 1 or qty > 1968:
            return MTCMResponse(MTCM_ERR_INVALID_PARAM)
        byte_cnt = (qty + 7) >> 3
        data = bytearray(byte_cnt)
        for i, v in enumerate(values):
            if v:
                data[i >> 3] |= (1 << (i & 7))
        pdu = bytes([
            MTCM_FUNC_WRITE_MULTI_COILS,
            (start >> 8) & 0xFF, start & 0xFF,
            (qty >> 8) & 0xFF, qty & 0xFF,
            byte_cnt,
        ]) + bytes(data)
        return self._transaction(pdu, expected_pdu_len=5, unit_id=unit_id)

    def write_multiple_regs(self, start: int, values: list, unit_id: int = None) -> MTCMResponse:
        """FC10 写多个保持寄存器。values: int 列表（uint16）。"""
        qty = len(values)
        if qty < 1 or qty > 123:
            return MTCMResponse(MTCM_ERR_INVALID_PARAM)
        data = bytearray(qty * 2)
        for i, v in enumerate(values):
            v &= 0xFFFF
            data[i * 2]     = (v >> 8) & 0xFF
            data[i * 2 + 1] = v & 0xFF
        pdu = bytes([
            MTCM_FUNC_WRITE_MULTI_REGS,
            (start >> 8) & 0xFF, start & 0xFF,
            (qty >> 8) & 0xFF, qty & 0xFF,
            qty * 2,
        ]) + bytes(data)
        return self._transaction(pdu, expected_pdu_len=5, unit_id=unit_id)

    # -----------------------------------------------------------------------
    # 连接管理
    # -----------------------------------------------------------------------
    def connect(self) -> bool:
        """主动建立 TCP 连接，返回是否成功。"""
        if self._sock is not None:
            return True
        try:
            addr = socket.getaddrinfo(self._host, self._port)[0][-1]
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self._conn_timeout_ms / 1000)
            s.connect(addr)
            s.settimeout(self._resp_timeout_ms / 1000)
            self._sock = s
            return True
        except OSError as e:
            self._sock = None
            return False

    def disconnect(self):
        """断开连接。"""
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    # -----------------------------------------------------------------------
    # 内部：读位
    # -----------------------------------------------------------------------
    def _read_bits(self, fc, start, qty, unit_id):
        if qty < 1 or qty > 2000:
            return MTCMResponse(MTCM_ERR_INVALID_PARAM)
        pdu = bytes([
            fc,
            (start >> 8) & 0xFF, start & 0xFF,
            (qty >> 8) & 0xFF, qty & 0xFF,
        ])
        byte_cnt = (qty + 7) >> 3
        resp = self._transaction(pdu, expected_pdu_len=2 + byte_cnt, unit_id=unit_id)
        if not resp.ok:
            return resp
        raw = resp.raw
        if len(raw) < 2 + byte_cnt:
            return MTCMResponse(MTCM_ERR_INVALID_RESPONSE, raw=raw)
        result = [bool(raw[2 + (i >> 3)] & (1 << (i & 7))) for i in range(qty)]
        resp.data = result
        return resp

    # -----------------------------------------------------------------------
    # 内部：读寄存器
    # -----------------------------------------------------------------------
    def _read_regs(self, fc, start, qty, unit_id):
        if qty < 1 or qty > 125:
            return MTCMResponse(MTCM_ERR_INVALID_PARAM)
        pdu = bytes([
            fc,
            (start >> 8) & 0xFF, start & 0xFF,
            (qty >> 8) & 0xFF, qty & 0xFF,
        ])
        resp = self._transaction(pdu, expected_pdu_len=2 + qty * 2, unit_id=unit_id)
        if not resp.ok:
            return resp
        raw = resp.raw
        if len(raw) < 2 + qty * 2:
            return MTCMResponse(MTCM_ERR_INVALID_RESPONSE, raw=raw)
        result = [(raw[2 + i * 2] << 8) | raw[3 + i * 2] for i in range(qty)]
        resp.data = result
        return resp

    # -----------------------------------------------------------------------
    # 内部：事务（含重试）
    # -----------------------------------------------------------------------
    def _transaction(self, pdu: bytes, expected_pdu_len: int, unit_id=None) -> MTCMResponse:
        uid = unit_id if unit_id is not None else self._unit_id
        self._trans_id = (self._trans_id + 1) & 0xFFFF

        # 构造 MBAP + PDU
        pdu_len_field = 1 + len(pdu)   # Unit ID(1) + PDU
        mbap = bytes([
            (self._trans_id >> 8) & 0xFF, self._trans_id & 0xFF,
            0x00, 0x00,
            (pdu_len_field >> 8) & 0xFF, pdu_len_field & 0xFF,
            uid,
        ])
        frame = mbap + pdu

        for attempt in range(self._retries + 1):
            resp = self._do_transaction(frame, expected_pdu_len, self._trans_id)
            if resp.error_code not in (MTCM_ERR_TIMEOUT, MTCM_ERR_CONNECT):
                break
            # 重试前重连
            self.disconnect()
            if attempt < self._retries:
                time.sleep_ms(50)

        if not self._keep_alive:
            self.disconnect()

        return resp

    def _do_transaction(self, frame: bytes, expected_pdu_len: int, trans_id: int) -> MTCMResponse:
        # 确保连接
        if self._sock is None:
            if not self.connect():
                return MTCMResponse(MTCM_ERR_CONNECT)

        # 发送
        try:
            self._sock.send(frame)
        except OSError:
            self.disconnect()
            return MTCMResponse(MTCM_ERR_CONNECT)

        # 接收 MBAP（7 字节）
        mbap = self._recv_exact(7)
        if mbap is None:
            self.disconnect()
            return MTCMResponse(MTCM_ERR_TIMEOUT)
        if len(mbap) < 7:
            self.disconnect()
            return MTCMResponse(MTCM_ERR_INVALID_RESPONSE)

        # 解析 MBAP
        resp_trans = (mbap[0] << 8) | mbap[1]
        resp_proto = (mbap[2] << 8) | mbap[3]
        resp_len   = (mbap[4] << 8) | mbap[5]   # Unit ID + PDU 长度

        if resp_proto != _MTCM_PROTOCOL_ID:
            self.disconnect()
            return MTCMResponse(MTCM_ERR_INVALID_RESPONSE)

        # 接收剩余 PDU（resp_len - 1，去掉 Unit ID）
        pdu_len = resp_len - 1
        if pdu_len < 1 or pdu_len > self._buf_size:
            self.disconnect()
            return MTCMResponse(MTCM_ERR_INVALID_RESPONSE)

        pdu_data = self._recv_exact(pdu_len)
        if pdu_data is None or len(pdu_data) < pdu_len:
            self.disconnect()
            return MTCMResponse(MTCM_ERR_TIMEOUT)

        # 检测从站异常响应
        if pdu_data[0] & 0x80:
            ex = pdu_data[1] if len(pdu_data) >= 2 else 0
            return MTCMResponse(MTCM_ERR_EXCEPTION, exception=ex, raw=bytes(pdu_data))

        # 事务 ID 匹配（宽松检查，不中断连接）
        if resp_trans != trans_id:
            return MTCMResponse(MTCM_ERR_INVALID_RESPONSE, raw=bytes(pdu_data))

        return MTCMResponse(MTCM_OK, raw=bytes(pdu_data))

    # -----------------------------------------------------------------------
    # 精确接收 N 字节（超时返回 None）
    # -----------------------------------------------------------------------
    def _recv_exact(self, n: int):
        buf = bytearray(n)
        received = 0
        deadline = time.ticks_ms() + self._resp_timeout_ms
        while received < n:
            remaining_ms = time.ticks_diff(deadline, time.ticks_ms())
            if remaining_ms <= 0:
                return None
            try:
                self._sock.settimeout(remaining_ms / 1000)
                chunk = self._sock.recv(n - received)
                if not chunk:
                    return None
                buf[received: received + len(chunk)] = chunk
                received += len(chunk)
            except OSError:
                return None
        return buf

    # -----------------------------------------------------------------------
    # 上下文管理器支持（with 语句）
    # -----------------------------------------------------------------------
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    def __repr__(self):
        return (
            f"<ModbusTCPMaster {self._host}:{self._port} "
            f"unit_id={self._unit_id} connected={self.connected}>"
        )


# ---------------------------------------------------------------------------
# 使用示例
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import network, time

    # 连接 Wi-Fi
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect("SSID", "PASSWORD")
    while not wlan.isconnected():
        time.sleep_ms(200)

    # ---- 方式一：手动管理连接 ------------------------------------------------
    master = ModbusTCPMaster("192.168.1.100", port=502, unit_id=1, retries=2)

    resp = master.read_holding_regs(0, 5)
    if resp.ok:
        print("寄存器:", resp.data)
    else:
        print("错误:", resp)

    master.write_single_reg(10, 9999)
    master.write_multiple_regs(20, [1, 2, 3, 4])

    resp = master.read_coils(0, 8)
    if resp.ok:
        print("线圈:", resp.data)

    master.disconnect()

    # ---- 方式二：with 语句自动管理连接 ----------------------------------------
    with ModbusTCPMaster("192.168.1.100") as m:
        resp = m.read_input_regs(0, 4)
        if resp.ok:
            print("输入寄存器:", resp.data)

    # ---- 方式三：访问多个从站（通过 unit_id 参数覆盖）------------------------
    gateway = ModbusTCPMaster("192.168.1.1", port=502)
    for uid in range(1, 5):
        resp = gateway.read_holding_regs(0, 1, unit_id=uid)
        print(f"  从站 {uid}:", resp)
    gateway.disconnect()
