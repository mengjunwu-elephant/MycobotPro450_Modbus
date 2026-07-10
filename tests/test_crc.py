from mycobotpro450_modbus import ModbusRTU


def test_crc16_known_modbus_frame() -> None:
    client = ModbusRTU.__new__(ModbusRTU)
    assert client.crc16(bytes.fromhex("01030000000A")) == bytes.fromhex("c5cd")

