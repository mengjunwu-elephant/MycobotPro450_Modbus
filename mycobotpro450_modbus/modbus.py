import logging
import serial
import threading
import time

logger = logging.getLogger(__name__)

# Modbus 到位反馈寄存器地址（对应 pymycobot 协议 0x5B）
ARRIVAL_FEEDBACK_ADDR = 0x005B
# 运动到位默认超时（秒），对齐 pymycobot 运动等待量级
DEFAULT_MOTION_TIMEOUT = 300


class ModbusRTU:
    def __init__(
        self,
        port,
        baudrate=115200,
        slave=0x2D,
        timeout=5,
        debug=False,
        motion_timeout=DEFAULT_MOTION_TIMEOUT,
    ):
        self.port = port
        self.baud = baudrate
        self.slave = slave
        self.timeout = timeout
        self.debug = debug
        self.motion_timeout = motion_timeout
        # 创建线程锁
        self._lock = threading.RLock()

        if self.debug:
            self._setup_debug_logging()

        # 串口连接也放在锁保护下
        with self._lock:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )

    def _setup_debug_logging(self):
        """为模块 logger 挂载 StreamHandler（本地时间），避免重复添加。"""
        logger.setLevel(logging.DEBUG)
        marker = "_mycobotpro450_modbus_debug"
        if any(getattr(h, marker, False) for h in logger.handlers):
            return
        handler = logging.StreamHandler()
        setattr(handler, marker, True)
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(logging.Formatter(
            fmt="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        logger.addHandler(handler)
        logger.propagate = False

    # ----------------------
    # CRC16(MODBUS)
    # ----------------------
    def crc16(self, data: bytes):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x01:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    def _expected_resp_len(self, resp: bytes):
        """根据已收到字节推断完整帧长度；信息不足时返回 None。"""
        if len(resp) < 2:
            return None
        func = resp[1]
        if func & 0x80:
            # Modbus 异常响应：从站+功能码+异常码+CRC
            return 5
        if func == 0x03:
            if len(resp) < 3:
                return None
            return 3 + resp[2] + 2
        if func == 0x10:
            if len(resp) < 4:
                return None
            reg = (resp[2] << 8) | resp[3]
            # 0x005B 到位反馈：从站+功能码+地址2+数量2+状态2+CRC2
            if reg == ARRIVAL_FEEDBACK_ADDR:
                return 10
            # 标准写多寄存器应答：8 字节
            return 8
        return None

    def _validate_crc(self, frame: bytes):
        if len(frame) < 4:
            if self.debug:
                logger.error("Response too short: %d bytes, data=%s", len(frame), frame.hex())
            raise ValueError(f"Response too short: {len(frame)} bytes")
        data = frame[:-2]
        crc_recv = frame[-2:]
        crc_calc = self.crc16(data)
        if crc_recv != crc_calc:
            if self.debug:
                logger.error(
                    "CRC ERROR recv=%s calc=%s frame=%s",
                    crc_recv.hex(), crc_calc.hex(), frame.hex(),
                )
            raise ValueError(f"CRC ERROR recv={crc_recv.hex()} calc={crc_calc.hex()}")
        return frame

    def _recv_frame(self, timeout=None):
        """按功能码拼出一整帧并做 CRC 校验。"""
        if timeout is None:
            timeout = self.timeout
        start_time = time.time()
        resp = b""

        while time.time() - start_time < timeout:
            chunk = self.ser.read(1)
            if chunk:
                resp += chunk
                expected = self._expected_resp_len(resp)
                if expected is not None and len(resp) >= expected:
                    resp = resp[:expected]
                    break
            else:
                time.sleep(0.01)

        if self.debug:
            if resp:
                logger.debug("RECV (%d bytes): %s", len(resp), resp.hex())
            else:
                logger.debug("RECV (0 bytes): <empty>")

        if not resp:
            if self.debug:
                logger.error("No response from device (timeout)")
            raise TimeoutError("No response from device")

        return self._validate_crc(resp)

    def _send_frame(self, payload: bytes, clear_buffer=True):
        frame = payload + self.crc16(payload)
        if self.debug:
            logger.debug("SEND (%d bytes): %s", len(frame), frame.hex())
        if clear_buffer:
            self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        return frame

    # ----------------------
    # 通用发送函数（已包含线程锁）
    # ----------------------
    def send(self, payload: bytes, resp_len=None):
        with self._lock:
            self._send_frame(payload)
            return self._recv_frame(timeout=self.timeout)

    def write_multiple_regs(
        self,
        reg_addr,
        values,
        wait_arrival=False,
        _async=False,
        motion_timeout=None,
    ):
        """
        写入多个寄存器（线程安全）

        Args:
            reg_addr: 起始寄存器地址
            values: 寄存器值列表
            wait_arrival: 是否等待 0x5B 到位反馈（运动指令）
            _async: True 时只发送不等待到位，返回 1（对齐 pymycobot）
            motion_timeout: 到位等待超时（秒），默认 self.motion_timeout
        """
        with self._lock:
            addr_h = (reg_addr >> 8) & 0xFF
            addr_l = reg_addr & 0xFF
            reg_count = len(values)
            num_h = (reg_count >> 8) & 0xFF
            num_l = reg_count & 0xFF
            byte_count = reg_count * 2

            data_bytes = bytearray()
            for value in values:
                data_bytes.extend([(value >> 8) & 0xFF, value & 0xFF])

            payload = bytes([
                self.slave,
                0x10,
                addr_h, addr_l,
                num_h, num_l,
                byte_count
            ]) + bytes(data_bytes)

            # 异步：只发指令，对齐 pymycobot 返回 1
            if wait_arrival and _async:
                self._send_frame(payload)
                return 1

            if wait_arrival:
                if motion_timeout is None:
                    motion_timeout = self.motion_timeout
                self._send_frame(payload)
                # 第一层：标准 0x10 写应答
                ack = self._recv_frame(timeout=self.timeout)
                if len(ack) < 4 or ack[1] != 0x10:
                    if self.debug:
                        logger.error("Unexpected write ACK: %s", ack.hex())
                    raise ValueError(f"Unexpected write ACK: {ack.hex()}")
                # 第二层：0x5B 到位反馈
                arrival = self._recv_frame(timeout=motion_timeout)
                if (
                    len(arrival) < 10
                    or arrival[1] != 0x10
                    or ((arrival[2] << 8) | arrival[3]) != ARRIVAL_FEEDBACK_ADDR
                ):
                    if self.debug:
                        logger.error("Unexpected arrival feedback: %s", arrival.hex())
                    raise ValueError(f"Unexpected arrival feedback: {arrival.hex()}")
                status = (arrival[6] << 8) | arrival[7]
                if self.debug:
                    logger.debug("Arrival status: %s", status)
                return status

            resp = self.send(payload)
            return resp

    def read_reg(self, reg_addr, reg_len=1):
        """
        读取寄存器数据（线程安全）
        """
        with self._lock:
            addr_h = (reg_addr >> 8) & 0xFF
            addr_l = reg_addr & 0xFF
            num_h = (reg_len >> 8) & 0xFF
            num_l = reg_len & 0xFF

            payload = bytes([
                self.slave,
                0x03,
                addr_h, addr_l,
                num_h, num_l
            ])
            resp = self.send(payload)
            # resp 格式： addr func bytecount data… crc
            byte_count = resp[2]
            data = resp[3:3 + byte_count]
            # 如果是单寄存器返回（2 字节）
            if byte_count == 2:
                return (data[0] << 8) | data[1]

            # 多寄存器返回（列表）
            values = []
            for i in range(0, byte_count, 2):
                values.append((data[i] << 8) | data[i + 1])
            return values

    def close(self):
        """安全关闭串口连接"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        """支持上下文管理器"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出上下文时自动关闭连接"""
        self.close()
