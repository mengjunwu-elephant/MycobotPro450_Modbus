import logging
import serial
import threading
import time

logger = logging.getLogger(__name__)


class ModbusRTU:
    def __init__(self, port, baudrate=115200, slave=0x2D, timeout=5, debug=False):
        self.port = port
        self.baud = baudrate
        self.slave = slave
        self.timeout = timeout
        self.debug = debug
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

    # ----------------------
    # 通用发送函数（已包含线程锁）
    # ----------------------
    def send(self, payload: bytes, resp_len=None):
        with self._lock:
            # 拼包
            frame = payload + self.crc16(payload)
            if self.debug:
                logger.debug("SEND (%d bytes): %s", len(frame), frame.hex())

            # 清空输入缓冲区
            self.ser.reset_input_buffer()

            # 发包
            self.ser.write(frame)
            self.ser.flush()

            # 使用更精确的等待方式
            start_time = time.time()
            resp = b""

            # 分段读取，避免长时间阻塞
            while time.time() - start_time < self.timeout:
                chunk = self.ser.read(1)  # 每次读1个字节
                if chunk:
                    resp += chunk
                    # 如果已经收到完整帧，提前退出
                    if len(resp) >= 7:  # 最小完整响应长度
                        # 检查是否收到完整帧（根据功能码判断预期长度）
                        if resp[1] == 0x03:  # 读寄存器功能码
                            byte_count = resp[2]
                            expected_len = 3 + byte_count + 2  # 地址1+功能码1+字节数1+数据+CRC2
                            if len(resp) >= expected_len:
                                break
                else:
                    time.sleep(0.01)  # 短暂休眠，避免CPU占用过高

            if self.debug:
                if resp:
                    logger.debug("RECV (%d bytes): %s", len(resp), resp.hex())
                else:
                    logger.debug("RECV (0 bytes): <empty>")

            if not resp:
                if self.debug:
                    logger.error("No response from device (timeout)")
                raise TimeoutError("No response from device")

            # 检查响应长度
            if len(resp) < 4:
                if self.debug:
                    logger.error("Response too short: %d bytes, data=%s", len(resp), resp.hex())
                raise ValueError(f"Response too short: {len(resp)} bytes")

            # CRC 校验
            data = resp[:-2]
            crc_recv = resp[-2:]
            crc_calc = self.crc16(data)

            if crc_recv != crc_calc:
                if self.debug:
                    logger.error(
                        "CRC ERROR recv=%s calc=%s frame=%s",
                        crc_recv.hex(), crc_calc.hex(), resp.hex(),
                    )
                raise ValueError(f"CRC ERROR recv={crc_recv.hex()} calc={crc_calc.hex()}")

            return resp

    def write_multiple_regs(self, reg_addr, values):
        """
        写入多个寄存器（线程安全）
        """
        with self._lock:
            addr_h = (reg_addr >> 8) & 0xFF
            addr_l = reg_addr & 0xFF
            reg_count = len(values)
            num_h = (reg_count >> 8) & 0xFF
            num_l = reg_count & 0xFF
            byte_count = reg_count * 2

            # 构建数据部分
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
