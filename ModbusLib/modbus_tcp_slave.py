"""
modbus_tcp_slave.py  —  MicroPython Modbus TCP Slave 扩展库
============================================================
命名规则：
  - RTU Slave  → modbus_rtu_slave.py，  前缀 ModbusRTUSlave / MRTS_
  - RTU Master → modbus_rtu_master.py， 前缀 ModbusRTUMaster / MRTM_
  - TCP Slave  → 本文件，               前缀 ModbusTCPSlave  / MTCS_
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
  0x64  自定义配置（可选回调）

Modbus TCP MBAP 头：
  Transaction ID (2) | Protocol ID (2, 固定 0x0000) |
  Length (2) | Unit ID (1) | Function Code (1) | Data (N)

依赖：usocket（network 已连接）
兼容平台：ESP32 / RP2040 / W5500（MicroPython v1.20+）

多连接模式：最多同时接受 MAX_CLIENTS 个 TCP 连接（非阻塞轮询）。
"""

import usocket as socket
import select

# ---------------------------------------------------------------------------
# 功能码常量
# ---------------------------------------------------------------------------
MTCS_FUNC_READ_COILS        = 0x01
MTCS_FUNC_READ_DISCRETE     = 0x02
MTCS_FUNC_READ_HOLDING      = 0x03
MTCS_FUNC_READ_INPUT        = 0x04
MTCS_FUNC_WRITE_SINGLE_COIL = 0x05
MTCS_FUNC_WRITE_SINGLE_REG  = 0x06
MTCS_FUNC_WRITE_MULTI_COILS = 0x0F
MTCS_FUNC_WRITE_MULTI_REGS  = 0x10
MTCS_FUNC_CUSTOM_CONFIG     = 0x64

# ---------------------------------------------------------------------------
# 异常码常量
# ---------------------------------------------------------------------------
MTCS_EX_ILLEGAL_FUNCTION    = 0x01
MTCS_EX_ILLEGAL_DATA_ADDR   = 0x02
MTCS_EX_ILLEGAL_DATA_VALUE  = 0x03
MTCS_EX_DEVICE_FAILURE      = 0x04

# MBAP 固定字段
_MTCS_PROTOCOL_ID = 0x0000
_MTCS_MBAP_LEN    = 6   # MBAP 头长度（不含数据）


# ---------------------------------------------------------------------------
# 数据区（与 RTU Slave 相同结构，独立定义避免强依赖）
# ---------------------------------------------------------------------------
class MTCSDataMap:
    """
    Modbus TCP Slave 数据区映射。

    参数
    ----
    coil_count        : int  线圈数量
    discrete_count    : int  离散输入数量
    holding_reg_count : int  保持寄存器数量
    input_reg_count   : int  输入寄存器数量
    """
    __slots__ = (
        "coils", "coil_count",
        "discrete_inputs", "discrete_count",
        "holding_regs", "holding_reg_count",
        "input_regs", "input_reg_count",
    )

    def __init__(self, coil_count=0, discrete_count=0,
                 holding_reg_count=0, input_reg_count=0):
        self.coil_count        = coil_count
        self.discrete_count    = discrete_count
        self.holding_reg_count = holding_reg_count
        self.input_reg_count   = input_reg_count

        self.coils          = bytearray((coil_count    + 7) // 8) if coil_count    else None
        self.discrete_inputs= bytearray((discrete_count+ 7) // 8) if discrete_count else None

        try:
            from array import array
            self.holding_regs = array("H", [0] * holding_reg_count) if holding_reg_count else None
            self.input_regs   = array("H", [0] * input_reg_count)   if input_reg_count   else None
        except ImportError:
            self.holding_regs = [0] * holding_reg_count if holding_reg_count else None
            self.input_regs   = [0] * input_reg_count   if input_reg_count   else None

    @staticmethod
    def get_bit(buf, idx):
        return bool(buf[idx >> 3] & (1 << (idx & 7)))

    @staticmethod
    def set_bit(buf, idx, val):
        if val:
            buf[idx >> 3] |= (1 << (idx & 7))
        else:
            buf[idx >> 3] &= ~(1 << (idx & 7))


# ---------------------------------------------------------------------------
# ModbusTCPSlave 主类
# ---------------------------------------------------------------------------
class ModbusTCPSlave:
    """
    Modbus TCP 从站（Slave / Server）。

    参数
    ----
    data_map     : MTCSDataMap  数据区映射对象
    unit_id      : int          Unit ID（0~255，默认 1；0xFF=接受全部）
    host         : str          监听地址（默认 "0.0.0.0"）
    port         : int          监听端口（默认 502）
    max_clients  : int          最大并发连接数（默认 4）
    buf_size     : int          每连接收发缓冲区（字节）
    write_cb     : callable|None  写前回调 fn(slave, fc, start, qty)->bool
    custom_cb    : callable|None  0x64 回调 fn(slave, param_addr, param_val)->bool
    user_data    : any          用户自定义数据
    """

    def __init__(
        self,
        data_map: MTCSDataMap,
        *,
        unit_id: int = 1,
        host: str = "0.0.0.0",
        port: int = 502,
        max_clients: int = 4,
        buf_size: int = 300,
        write_cb=None,
        custom_cb=None,
        user_data=None,
    ):
        self._data_map   = data_map
        self._unit_id    = unit_id
        self._host       = host
        self._port       = port
        self._max_clients = max_clients
        self._buf_size   = buf_size
        self._write_cb   = write_cb
        self._custom_cb  = custom_cb
        self.user_data   = user_data

        self._server_sock = None
        # 已连接客户端列表：[(sock, addr, rx_buf), ...]
        self._clients: list = []

    # -----------------------------------------------------------------------
    # 生命周期
    # -----------------------------------------------------------------------
    def start(self):
        """开始监听，创建非阻塞 server socket。"""
        if self._server_sock:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self._host, self._port))
        s.listen(self._max_clients)
        s.setblocking(False)
        self._server_sock = s
        print(f"[ModbusTCPSlave] 监听 {self._host}:{self._port}  unit_id={self._unit_id}")

    def stop(self):
        """关闭所有连接。"""
        for sock, _, _ in self._clients:
            try:
                sock.close()
            except Exception:
                pass
        self._clients.clear()
        if self._server_sock:
            self._server_sock.close()
            self._server_sock = None

    # -----------------------------------------------------------------------
    # 核心：主循环轮询（非阻塞，应在 while True 中持续调用）
    # -----------------------------------------------------------------------
    def process(self):
        """非阻塞轮询：接受新连接，处理已有连接数据。"""
        if self._server_sock is None:
            return

        # --- 接受新连接 ---
        if len(self._clients) < self._max_clients:
            try:
                conn, addr = self._server_sock.accept()
                conn.setblocking(False)
                self._clients.append((conn, addr, bytearray(self._buf_size)))
            except OSError:
                pass

        # --- 处理已有连接 ---
        dead = []
        for entry in self._clients:
            sock, addr, rx_buf = entry
            try:
                data = sock.recv(self._buf_size)
                if not data:
                    dead.append(entry)
                    continue
                # 一次可能收到多帧，循环解析
                self._parse_and_respond(sock, data)
            except OSError as e:
                # EAGAIN(11) = 暂无数据，正常
                if e.args[0] != 11:
                    dead.append(entry)

        for entry in dead:
            try:
                entry[0].close()
            except Exception:
                pass
            self._clients.remove(entry)

    # -----------------------------------------------------------------------
    # 解析 MBAP + PDU，组装响应
    # -----------------------------------------------------------------------
    def _parse_and_respond(self, sock, data: bytes):
        offset = 0
        while offset + _MTCS_MBAP_LEN <= len(data):
            # MBAP 头
            trans_id   = (data[offset]     << 8) | data[offset + 1]
            proto_id   = (data[offset + 2] << 8) | data[offset + 3]
            pdu_len    = (data[offset + 4] << 8) | data[offset + 5]  # Unit ID + FC + Data
            unit_id    = data[offset + 6]

            if proto_id != _MTCS_PROTOCOL_ID:
                break   # 非 Modbus TCP，丢弃
            if offset + _MTCS_MBAP_LEN + pdu_len - 1 > len(data):
                break   # 帧不完整

            # Unit ID 过滤（0xFF 广播全部接受）
            if self._unit_id != 0xFF and unit_id != self._unit_id:
                offset += _MTCS_MBAP_LEN + pdu_len - 1
                continue

            pdu_start = offset + _MTCS_MBAP_LEN   # 指向 FC 字节
            pdu_data  = data[pdu_start: pdu_start + pdu_len - 1]
            func_code = pdu_data[0]

            resp_pdu = self._dispatch(func_code, pdu_data)

            # 组装响应 MBAP
            resp_len = 1 + len(resp_pdu)   # Unit ID(1) + PDU
            header = bytes([
                (trans_id >> 8) & 0xFF, trans_id & 0xFF,
                0x00, 0x00,
                (resp_len >> 8) & 0xFF, resp_len & 0xFF,
                unit_id,
            ])
            try:
                sock.send(header + resp_pdu)
            except OSError:
                pass

            offset += _MTCS_MBAP_LEN + pdu_len - 1

    # -----------------------------------------------------------------------
    # 功能码分发，返回 bytes（PDU，不含 MBAP）
    # -----------------------------------------------------------------------
    def _dispatch(self, func_code: int, pdu: bytes) -> bytes:
        dm = self._data_map

        if func_code == MTCS_FUNC_READ_COILS:
            return self._read_bits(func_code, pdu, dm.coils, dm.coil_count, 2000)
        elif func_code == MTCS_FUNC_READ_DISCRETE:
            return self._read_bits(func_code, pdu, dm.discrete_inputs, dm.discrete_count, 2000)
        elif func_code == MTCS_FUNC_READ_HOLDING:
            return self._read_regs(func_code, pdu, dm.holding_regs, dm.holding_reg_count)
        elif func_code == MTCS_FUNC_READ_INPUT:
            return self._read_regs(func_code, pdu, dm.input_regs, dm.input_reg_count)
        elif func_code == MTCS_FUNC_WRITE_SINGLE_COIL:
            return self._write_single_coil(pdu)
        elif func_code == MTCS_FUNC_WRITE_SINGLE_REG:
            return self._write_single_reg(pdu)
        elif func_code == MTCS_FUNC_WRITE_MULTI_COILS:
            return self._write_multi_coils(pdu)
        elif func_code == MTCS_FUNC_WRITE_MULTI_REGS:
            return self._write_multi_regs(pdu)
        elif func_code == MTCS_FUNC_CUSTOM_CONFIG:
            return self._custom_config(pdu)
        else:
            return self._exception_pdu(func_code, MTCS_EX_ILLEGAL_FUNCTION)

    # -----------------------------------------------------------------------
    # 0x01 / 0x02  读位
    # -----------------------------------------------------------------------
    def _read_bits(self, fc, pdu, bit_buf, bit_count, max_qty):
        if bit_buf is None or bit_count == 0:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_FUNCTION)
        if len(pdu) < 5:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_DATA_VALUE)
        start = (pdu[1] << 8) | pdu[2]
        qty   = (pdu[3] << 8) | pdu[4]
        if qty < 1 or qty > max_qty:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_DATA_VALUE)
        if start + qty > bit_count:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_DATA_ADDR)
        byte_cnt = (qty + 7) >> 3
        resp = bytearray(2 + byte_cnt)
        resp[0] = fc
        resp[1] = byte_cnt
        for i in range(qty):
            idx = start + i
            if bit_buf[idx >> 3] & (1 << (idx & 7)):
                resp[2 + (i >> 3)] |= (1 << (i & 7))
        return bytes(resp)

    # -----------------------------------------------------------------------
    # 0x03 / 0x04  读寄存器
    # -----------------------------------------------------------------------
    def _read_regs(self, fc, pdu, reg_buf, reg_count):
        if reg_buf is None or reg_count == 0:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_FUNCTION)
        if len(pdu) < 5:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_DATA_VALUE)
        start = (pdu[1] << 8) | pdu[2]
        qty   = (pdu[3] << 8) | pdu[4]
        if qty < 1 or qty > 125:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_DATA_VALUE)
        if start + qty > reg_count:
            return self._exception_pdu(fc, MTCS_EX_ILLEGAL_DATA_ADDR)
        resp = bytearray(2 + qty * 2)
        resp[0] = fc
        resp[1] = qty * 2
        for i in range(qty):
            v = reg_buf[start + i]
            resp[2 + i * 2] = (v >> 8) & 0xFF
            resp[3 + i * 2] = v & 0xFF
        return bytes(resp)

    # -----------------------------------------------------------------------
    # 0x05  写单个线圈
    # -----------------------------------------------------------------------
    def _write_single_coil(self, pdu):
        dm = self._data_map
        if dm.coils is None or dm.coil_count == 0:
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_COIL, MTCS_EX_ILLEGAL_FUNCTION)
        if len(pdu) < 5:
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_COIL, MTCS_EX_ILLEGAL_DATA_VALUE)
        addr = (pdu[1] << 8) | pdu[2]
        val  = (pdu[3] << 8) | pdu[4]
        if val not in (0x0000, 0xFF00):
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_COIL, MTCS_EX_ILLEGAL_DATA_VALUE)
        if addr >= dm.coil_count:
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_COIL, MTCS_EX_ILLEGAL_DATA_ADDR)
        if not self._invoke_write_cb(MTCS_FUNC_WRITE_SINGLE_COIL, addr, 1):
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_COIL, MTCS_EX_DEVICE_FAILURE)
        MTCSDataMap.set_bit(dm.coils, addr, val == 0xFF00)
        return bytes(pdu[:5])  # 回显

    # -----------------------------------------------------------------------
    # 0x06  写单个寄存器
    # -----------------------------------------------------------------------
    def _write_single_reg(self, pdu):
        dm = self._data_map
        if dm.holding_regs is None or dm.holding_reg_count == 0:
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_REG, MTCS_EX_ILLEGAL_FUNCTION)
        if len(pdu) < 5:
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_REG, MTCS_EX_ILLEGAL_DATA_VALUE)
        addr = (pdu[1] << 8) | pdu[2]
        val  = (pdu[3] << 8) | pdu[4]
        if addr >= dm.holding_reg_count:
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_REG, MTCS_EX_ILLEGAL_DATA_ADDR)
        if not self._invoke_write_cb(MTCS_FUNC_WRITE_SINGLE_REG, addr, 1):
            return self._exception_pdu(MTCS_FUNC_WRITE_SINGLE_REG, MTCS_EX_DEVICE_FAILURE)
        dm.holding_regs[addr] = val
        return bytes(pdu[:5])

    # -----------------------------------------------------------------------
    # 0x0F  写多个线圈
    # -----------------------------------------------------------------------
    def _write_multi_coils(self, pdu):
        dm = self._data_map
        if dm.coils is None or dm.coil_count == 0:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_COILS, MTCS_EX_ILLEGAL_FUNCTION)
        if len(pdu) < 6:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_COILS, MTCS_EX_ILLEGAL_DATA_VALUE)
        start    = (pdu[1] << 8) | pdu[2]
        qty      = (pdu[3] << 8) | pdu[4]
        byte_cnt = pdu[5]
        if qty < 1 or qty > 1968:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_COILS, MTCS_EX_ILLEGAL_DATA_VALUE)
        if start + qty > dm.coil_count:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_COILS, MTCS_EX_ILLEGAL_DATA_ADDR)
        if len(pdu) < 6 + byte_cnt:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_COILS, MTCS_EX_ILLEGAL_DATA_VALUE)
        if not self._invoke_write_cb(MTCS_FUNC_WRITE_MULTI_COILS, start, qty):
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_COILS, MTCS_EX_DEVICE_FAILURE)
        for i in range(qty):
            MTCSDataMap.set_bit(dm.coils, start + i,
                                bool(pdu[6 + (i >> 3)] & (1 << (i & 7))))
        return bytes([MTCS_FUNC_WRITE_MULTI_COILS,
                      pdu[1], pdu[2], pdu[3], pdu[4]])

    # -----------------------------------------------------------------------
    # 0x10  写多个寄存器
    # -----------------------------------------------------------------------
    def _write_multi_regs(self, pdu):
        dm = self._data_map
        if dm.holding_regs is None or dm.holding_reg_count == 0:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_REGS, MTCS_EX_ILLEGAL_FUNCTION)
        if len(pdu) < 6:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_REGS, MTCS_EX_ILLEGAL_DATA_VALUE)
        start = (pdu[1] << 8) | pdu[2]
        qty   = (pdu[3] << 8) | pdu[4]
        if qty < 1 or qty > 123:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_REGS, MTCS_EX_ILLEGAL_DATA_VALUE)
        if start + qty > dm.holding_reg_count:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_REGS, MTCS_EX_ILLEGAL_DATA_ADDR)
        if len(pdu) < 6 + qty * 2:
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_REGS, MTCS_EX_ILLEGAL_DATA_VALUE)
        if not self._invoke_write_cb(MTCS_FUNC_WRITE_MULTI_REGS, start, qty):
            return self._exception_pdu(MTCS_FUNC_WRITE_MULTI_REGS, MTCS_EX_DEVICE_FAILURE)
        for i in range(qty):
            dm.holding_regs[start + i] = (pdu[6 + i * 2] << 8) | pdu[7 + i * 2]
        return bytes([MTCS_FUNC_WRITE_MULTI_REGS,
                      pdu[1], pdu[2], pdu[3], pdu[4]])

    # -----------------------------------------------------------------------
    # 0x64  自定义配置
    # -----------------------------------------------------------------------
    def _custom_config(self, pdu):
        # FC(1) + param_addr(2) + param_val(2) = 5 字节
        if len(pdu) != 5:
            return self._exception_pdu(MTCS_FUNC_CUSTOM_CONFIG, MTCS_EX_ILLEGAL_DATA_VALUE)
        if self._custom_cb is None:
            return self._exception_pdu(MTCS_FUNC_CUSTOM_CONFIG, MTCS_EX_ILLEGAL_FUNCTION)
        param_addr = (pdu[1] << 8) | pdu[2]
        param_val  = (pdu[3] << 8) | pdu[4]
        if self._custom_cb(self, param_addr, param_val):
            return bytes(pdu[:5])
        return self._exception_pdu(MTCS_FUNC_CUSTOM_CONFIG, MTCS_EX_ILLEGAL_DATA_VALUE)

    # -----------------------------------------------------------------------
    # 辅助
    # -----------------------------------------------------------------------
    def _invoke_write_cb(self, fc, start, qty) -> bool:
        if self._write_cb is None:
            return True
        return bool(self._write_cb(self, fc, start, qty))

    @staticmethod
    def _exception_pdu(fc: int, ex_code: int) -> bytes:
        return bytes([fc | 0x80, ex_code])

    # -----------------------------------------------------------------------
    # 便捷属性：直接读写数据区
    # -----------------------------------------------------------------------
    def get_holding_reg(self, addr):
        return self._data_map.holding_regs[addr]

    def set_holding_reg(self, addr, value):
        self._data_map.holding_regs[addr] = value & 0xFFFF

    def get_input_reg(self, addr):
        return self._data_map.input_regs[addr]

    def set_input_reg(self, addr, value):
        self._data_map.input_regs[addr] = value & 0xFFFF

    def get_coil(self, addr):
        return MTCSDataMap.get_bit(self._data_map.coils, addr)

    def set_coil(self, addr, value):
        MTCSDataMap.set_bit(self._data_map.coils, addr, value)

    def get_discrete(self, addr):
        return MTCSDataMap.get_bit(self._data_map.discrete_inputs, addr)

    def set_discrete(self, addr, value):
        MTCSDataMap.set_bit(self._data_map.discrete_inputs, addr, value)

    @property
    def client_count(self):
        return len(self._clients)

    def __repr__(self):
        return (
            f"<ModbusTCPSlave {self._host}:{self._port} "
            f"unit_id={self._unit_id} clients={len(self._clients)}>"
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
    print("IP:", wlan.ifconfig()[0])

    # 数据区
    dm = MTCSDataMap(
        coil_count        = 16,
        discrete_count    = 8,
        holding_reg_count = 32,
        input_reg_count   = 8,
    )
    dm.holding_regs[0] = 0xABCD

    # 创建并启动从站
    slave = ModbusTCPSlave(dm, unit_id=1, port=502, max_clients=4)
    slave.start()

    while True:
        slave.process()
        # 在此更新传感器数据到 input_regs
        # slave.set_input_reg(0, read_adc())
        time.sleep_ms(1)
