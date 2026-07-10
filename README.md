# MycobotPro450 Modbus

MyCobot Pro 450 Modbus RTU client library.

This project was split from the `elephant-pytest` automation repository so the Modbus client can be versioned, tested, and reused independently.

## Install

```powershell
pip install -e .[dev]
```

## Basic Usage

```python
from mycobotpro450_modbus import Pro450

robot = Pro450("COM10", baudrate=115200)
print(robot.get_angles())
```

## Test

```powershell
pytest
```

## Versioning

Use tags for published versions:

```text
v0.1.0
v0.1.1
```

