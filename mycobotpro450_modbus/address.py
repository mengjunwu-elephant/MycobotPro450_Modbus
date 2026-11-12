class CommandAddress:
    """指令地址常量类"""
    # 系统控制指令
    POWER_ON = 0x0010  # 机器人上电
    POWER_OFF = 0x0011  # 机器人掉电
    CLEAR_MOTION_ERROR = 0x0008  # 清除运动报错
    RETURN_TO_ZERO = 0x0004  # 超限回零
    CLEAR_MOTOR_ERROR = 0x00E7  # 清除电机报错

    # 读取状态寄存器
    SYSTEM_VERSION = 0x0002  # 系统版本
    FRESH_MODE = 0x0017  # 运行模式
    POWER_STATUS = 0x0012  # 上电状态
    ERROR_INFO = 0x0007  # 运动报警
    ROBOT_STATUS = 0x00A2  # 运动状态
    ANGLES = 0x0020  # 全关节角度
    COORDS = 0x0023  # 坐标系坐标
    MOTOR_PAUSE = 0x0027  # 运动暂停状态
    IS_MOVING = 0x002B  # 运动状态
    DIGITAL_IO_INPUT = 0x00A1  # 末端输入状态
    BASE_IO_INPUT = 0x007B  # 底座输入状态
    SERVOS_SPEED = 0x00E1  # 伺服速度
    SERVOS_CURRENT = 0x00E2  # 伺服电流
    SERVOS_STATUS = 0x00E4  # 伺服状态
    MOTOR_TEMPS = 0x00E5  # 关节驱动板温度

    # 运动控制指令
    SINGLE_ANGLE = 0x0021  # 单关节控制
    MULTIPLE_ANGLES = 0x0022  # 多关节控制
    SINGLE_COORD = 0x0024  # 单坐标控制
    MULTIPLE_COORDS = 0x0025  # 多坐标控制
    PAUSE_MOTION = 0x0026  # 运动暂停
    RESUME_MOTION = 0x0028  # 运动继续
    STOP_MOTION = 0x0029  # 运动停止
    ARRIVAL_FEEDBACK = 0x005B  # 到位反馈（运动第二层返回）

    # 持续运动指令
    CONTINUOUS_JOINT = 0x0030  # 关节持续运动
    CONTINUOUS_COORD = 0x0032  # 坐标持续运动
    RPY_ROTATION = 0x00F5  # RPY旋转

    # 步进运动指令
    STEP_ANGLE = 0x0033  # 角度步进
    STEP_COORD = 0x0034  # 坐标步进

    # 关节控制指令
    MOTOR_ENABLE = 0x0013  # 关节使能状态
    BRAKE_CONTROL = 0x0019  # 关节刹车状态

    # IO控制指令
    BASE_IO_OUTPUT = 0x00A0  # 底部IO控制
    DIGITAL_IO_OUTPUT = 0x0061  # 末端IO控制
    RGB_COLOR = 0x000C  # RGB颜色控制
