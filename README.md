# MycobotPro450 Modbus

MyCobot Pro 450 的 Modbus RTU 客户端库，基于串口通信，提供底层协议封装与高层机器人控制接口。

本项目从 `elephant-pytest` 自动化仓库中拆分而来，便于独立版本管理、测试与复用。

## 功能概览

- 基于 Modbus RTU（功能码 `0x03` 读寄存器、`0x10` 写多寄存器）
- 线程安全的串口收发与 CRC16 校验
- `Pro450` 高层 API：状态查询、关节/坐标运动、IO 与 RGB 控制
- 可选 `debug` 帧日志（发送/接收原始 hex）

## 包结构

```text
mycobotpro450_modbus/
  __init__.py   # 对外导出：Pro450、ModbusRTU、CommandAddress
  modbus.py     # Modbus RTU 传输层（串口、CRC、读写）
  address.py    # 寄存器 / 指令地址常量
  pro450.py     # Pro450 高层机器人接口
```

推荐从包根导入：

```python
from mycobotpro450_modbus import Pro450, ModbusRTU, CommandAddress
```

## 环境要求

- Python >= 3.10
- 依赖：`pyserial>=3.5`

## 安装

开发模式安装（含测试依赖）：

```powershell
pip install -e .[dev]
```

仅安装运行依赖：

```powershell
pip install -e .
```

## 快速开始

```python
from mycobotpro450_modbus import Pro450

robot = Pro450("COM10", baudrate=115200)
try:
    print(robot.get_system_version())
    print(robot.get_angles())
    robot.power_on()
    robot.send_angles([0, 0, 0, 0, 0, 0], speed=20)
finally:
    robot.close()
```

也可用上下文管理器自动关闭串口：

```python
with Pro450("COM10", baudrate=115200) as robot:
    print(robot.is_power_on())
```

完整示例见 `examples/basic_usage.py`。

## 构造参数

`Pro450` / `ModbusRTU` 共用以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | 必填 | 串口名，如 `COM10`、`/dev/ttyUSB0` |
| `baudrate` | `115200` | 波特率 |
| `slave` | `0x2D` | Modbus 从站地址 |
| `timeout` | `5` | 普通读写超时（秒） |
| `motion_timeout` | `300` | 运动到位（`0x5B`）等待超时（秒） |
| `debug` | `False` | 是否输出 SEND/RECV 帧日志 |

## 调试日志

将 `debug=True` 后，库会通过标准 `logging` 输出带本地时间戳与字节长度的收发帧：

```python
robot = Pro450("COM10", baudrate=115200, debug=True)
```

示例输出：

```text
2026-11-12 17:40:01.123 [DEBUG] SEND (8 bytes): 2d0300200006xxxx
2026-11-12 17:40:01.145 [DEBUG] RECV (17 bytes): 2d03...
```

超时、响应过短、CRC 错误会在 `debug=True` 时记录错误日志，并继续抛出异常。

## 运动到位反馈（0x5B）

运动类指令会先收到标准 Modbus `0x10` 写应答，再收到 `0x005B` 到位反馈帧。库会解析第二层反馈并返回状态码（对齐 pymycobot：`0` 表示成功）。

```text
ACK:     2d 10 <reg> <qty> <crc>          # 8 字节
到位:    2d 10 00 5b <qty> <status> <crc> # 10 字节，status=0 成功
```

- `_async=False`（默认，jog 除外）：阻塞等到位，返回状态码
- `_async=True`：只发送指令，立即返回 `1`（对齐 pymycobot）
- `jog_angle` / `jog_coord` / `jog_rpy` 默认 `_async=True`

```python
status = robot.send_angles([0, 0, 0, 0, 0, 0], 20)  # 等到位，返回 0
robot.send_angles([0, 0, 0, 0, 0, 0], 20, _async=True)  # 不等待，返回 1
```

## 主要接口说明

### 状态读取

| 方法 | 说明 |
|------|------|
| `get_system_version()` | 系统版本号 |
| `get_fresh_mode()` | 运行模式（队列/插补/刷新） |
| `is_power_on()` | 是否上电 |
| `get_error_information()` | 运动报警信息 |
| `get_robot_status()` | 末端运动相关状态 |
| `get_angles()` | 六关节角度（度） |
| `get_coords()` | 末端位姿 `[x,y,z,rx,ry,rz]`（mm / 度） |
| `is_motor_pause()` | 是否暂停 |
| `is_moving()` | 是否运动中 |
| `get_digital_input()` | 末端输入状态 |
| `get_base_io_input(pin_no)` | 底座输入状态 |
| `get_servos_speed()` / `get_servos_current()` / `get_servos_status()` | 伺服速度 / 电流 / 状态 |
| `get_motor_temps()` | 各关节驱动板温度（6 个无符号整数） |

### 系统与安全

| 方法 | 说明 |
|------|------|
| `power_on()` / `power_off()` | 上电 / 掉电 |
| `clear_error_information()` | 清除运动报错 |
| `over_limit_return_zero()` | 超限回零（仅插补模式） |
| `servo_restore(joint=254)` | 清除电机报错；`254` 表示全部关节 |
| `pause(gentle_stop=True)` / `resume()` / `stop(mode=True)` | 暂停 / 继续 / 停止（`True` 缓停，`False` 急停） |

### 运动控制

| 方法 | 说明 |
|------|------|
| `send_angle(joint, angle, speed, _async=False)` | 单关节运动，返回到位状态码 |
| `send_angles(angles, speed, _async=False)` | 多关节运动，返回到位状态码 |
| `send_coord(axis, value, speed, _async=False)` | 单坐标运动，返回到位状态码 |
| `send_coords(coords, speed, _async=False)` | 多坐标运动，返回到位状态码 |
| `jog_angle(..., _async=True)` / `jog_coord(...)` / `jog_rpy(...)` | 持续运动 / RPY 旋转 |
| `jog_increment_angle(..., _async=False)` / `jog_increment_coord(...)` | 角度 / 坐标步进 |

单位约定：

- 关节角、rx/ry/rz：度
- x/y/z：毫米
- `speed`：通常为 `1~100`
- 关节号：`1~6`（部分接口 `254` 表示全部）

### IO 与灯光

| 方法 | 说明 |
|------|------|
| `set_motor_enable(joint, state)` | 关节使能；`0` 放松，`1` 使能 |
| `set_brake(joint, state)` | 关节刹车；`0` 释放，`1` 刹车 |
| `set_base_io_output(pin, state)` | 底座 IO 输出（引脚 `1~12`） |
| `set_digital_output(pin, state)` | 末端 IO 输出（引脚 `1~2`） |
| `set_color(r, g, b)` | 末端 RGB（`0~255`） |

### 底层与常量

如需自行组包，可直接使用：

- `ModbusRTU`：`read_reg` / `write_multiple_regs` / `send` / `crc16`
- `CommandAddress`：寄存器与指令地址常量（见 `address.py`）

## 测试

```powershell
pytest
```

## 版本管理

发布版本建议使用 Git tag：

```text
v1.0.1
v1.0.2
```
