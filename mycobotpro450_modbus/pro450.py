from .address import CommandAddress
from .modbus import ModbusRTU


class Pro450(ModbusRTU):
    def get_system_version(self):
        """获取系统版本号"""
        return self.read_reg(CommandAddress.SYSTEM_VERSION, 1) / 10

    def get_fresh_mode(self):
        """获取运行模式（队列/插补/刷新模式）"""
        return self.read_reg(CommandAddress.FRESH_MODE, 1)

    def is_power_on(self):
        """检查上电状态"""
        return self.read_reg(CommandAddress.POWER_STATUS, 1)

    def get_error_information(self):
        """获取运动报警信息"""
        return self.read_reg(CommandAddress.ERROR_INFO, 1)

    def get_robot_status(self):
        """获取末端运动相关状态"""
        return self.read_reg(CommandAddress.ROBOT_STATUS, 14)

    def get_angles(self):
        """获取所有关节角度（单位：度）"""
        raw = self.read_reg(CommandAddress.ANGLES, 6)

        def to_signed(val):
            """将无符号整数转换为有符号整数"""
            return val - 65536 if val > 32767 else val

        angles = [to_signed(d) / 100 for d in raw]
        return angles

    def get_coords(self):
        """获取末端坐标系坐标（位置单位：mm，旋转单位：度）"""
        raw = self.read_reg(CommandAddress.COORDS, 6)

        def to_signed(val):
            """将无符号整数转换为有符号整数"""
            return val - 65536 if val > 32767 else val

        coords = [
            to_signed(raw[0]) / 10,  # x (mm)
            to_signed(raw[1]) / 10,  # y (mm)
            to_signed(raw[2]) / 10,  # z (mm)
            to_signed(raw[3]) / 100,  # rx (度)
            to_signed(raw[4]) / 100,  # ry (度)
            to_signed(raw[5]) / 100  # rz (度)
        ]
        return coords

    def is_motor_pause(self):
        """检查运动暂停状态"""
        return self.read_reg(CommandAddress.MOTOR_PAUSE, 1)

    def is_moving(self):
        """检查机器人是否正在运动"""
        return self.read_reg(CommandAddress.IS_MOVING, 1)

    def get_digital_input(self):
        """获取末端输入状态"""
        return self.read_reg(CommandAddress.DIGITAL_IO_INPUT, 12)

    def get_base_io_input(self, pin_no):
        """获取底座输入状态"""
        return self.write_multiple_regs(CommandAddress.BASE_IO_INPUT, [pin_no])

    def get_servos_speed(self):
        """获取各关节伺服速度"""
        raw = self.read_reg(CommandAddress.SERVOS_SPEED, 6)

        def to_signed(val):
            """将无符号整数转换为有符号整数"""
            return val - 65536 if val > 32767 else val

        servos_speed = [to_signed(i) for i in raw]
        return servos_speed

    def get_servos_current(self):
        """获取各关节伺服电流"""
        raw = self.read_reg(CommandAddress.SERVOS_CURRENT, 6)

        def to_signed(val):
            """将无符号整数转换为有符号整数"""
            return val - 65536 if val > 32767 else val

        servos_current = [to_signed(i) for i in raw]
        return servos_current

    def get_servos_status(self):
        """获取各关节伺服状态"""
        raw = self.read_reg(CommandAddress.SERVOS_STATUS, 6)

        def to_signed(val):
            """将无符号整数转换为有符号整数"""
            return val - 65536 if val > 32767 else val

        servos_status = [to_signed(i) for i in raw]
        return servos_status

    def get_motor_temps(self):
        """获取各关节驱动板温度（无符号原值，6 个整数）"""
        return self.read_reg(CommandAddress.MOTOR_TEMPS, 6)

    # ==================== 设置接口 ====================

    def power_on(self):
        """机器人上电"""
        return self.write_multiple_regs(CommandAddress.POWER_ON, [0x0000])

    def power_off(self):
        """机器人掉电"""
        return self.write_multiple_regs(CommandAddress.POWER_OFF, [0x0000])

    def clear_error_information(self):
        """清除运动报错"""
        return self.write_multiple_regs(CommandAddress.CLEAR_MOTION_ERROR, [0x0000])

    def over_limit_return_zero(self):
        """超限回零（仅插补模式可使用），等待到位反馈"""
        return self.write_multiple_regs(
            CommandAddress.RETURN_TO_ZERO, [0x0000], wait_arrival=True
        )

    def servo_restore(self, joint=254):
        """
        清除电机报错

        Args:
            joint: 关节号 1-6（具体关节），254（所有关节）
        """
        return self.write_multiple_regs(CommandAddress.CLEAR_MOTOR_ERROR, [joint])

    def send_angle(self, joint, angle, speed, _async=False):
        """
        单关节控制

        Args:
            joint: 关节号 1-6
            angle: 目标角度（度）
            speed: 运动速度 1-100
            _async: True 只发送不等待到位，返回 1；False 等待 0x5B 并返回状态码
        """
        angle_int = int(angle * 100)
        if angle_int < 0:
            angle_int = 65536 + angle_int

        values = [joint, angle_int, speed]
        return self.write_multiple_regs(
            CommandAddress.SINGLE_ANGLE, values, wait_arrival=True, _async=_async
        )

    def send_angles(self, angles, speed, _async=False):
        """
        多关节控制（到位后有第二层 0x5B 反馈）

        Args:
            angles: 6个关节角度列表 [j1, j2, j3, j4, j5, j6]（度）
            speed: 运动速度 1-100
            _async: True 只发送不等待到位，返回 1；False 等待 0x5B 并返回状态码
        """
        angle_ints = []
        for angle in angles:
            angle_int = int(angle * 100)
            if angle_int < 0:
                angle_int = 65536 + angle_int
            angle_ints.append(angle_int)

        values = angle_ints + [speed]
        return self.write_multiple_regs(
            CommandAddress.MULTIPLE_ANGLES, values, wait_arrival=True, _async=_async
        )

    def send_coord(self, joint, coord_value, speed, _async=False):
        """
        单坐标控制

        Args:
            joint: 坐标轴 1-6 (1:x, 2:y, 3:z, 4:rx, 5:ry, 6:rz)
            coord_value: 坐标值 (x,y,z单位mm; rx,ry,rz单位度)
            speed: 运动速度 1-100
            _async: True 只发送不等待到位，返回 1；False 等待 0x5B 并返回状态码
        """
        if joint <= 3:  # x, y, z 坐标
            coord_int = int(coord_value * 10)
        else:  # rx, ry, rz 旋转
            coord_int = int(coord_value * 100)

        if coord_int < 0:
            coord_int = 65536 + coord_int

        values = [joint, coord_int, speed]
        return self.write_multiple_regs(
            CommandAddress.SINGLE_COORD, values, wait_arrival=True, _async=_async
        )

    def send_coords(self, coords, speed, _async=False):
        """
        多坐标控制

        Args:
            coords: 坐标列表 [x, y, z, rx, ry, rz] (x,y,z单位mm; rx,ry,rz单位度)
            speed: 运动速度 1-100
            _async: True 只发送不等待到位，返回 1；False 等待 0x5B 并返回状态码
        """
        coord_ints = []
        for i, coord in enumerate(coords):
            if i < 3:  # x, y, z
                coord_int = int(coord * 10)
            else:  # rx, ry, rz
                coord_int = int(coord * 100)

            if coord_int < 0:
                coord_int = 65536 + coord_int
            coord_ints.append(coord_int)

        values = coord_ints + [speed]
        return self.write_multiple_regs(
            CommandAddress.MULTIPLE_COORDS, values, wait_arrival=True, _async=_async
        )

    def pause(self, gentle_stop=True):
        """
        运动暂停

        Args:
            gentle_stop: True-缓停, False-急停
        """
        stop_type = 1 if gentle_stop else 0
        return self.write_multiple_regs(CommandAddress.PAUSE_MOTION, [stop_type])

    def resume(self):
        """运动继续"""
        return self.write_multiple_regs(CommandAddress.RESUME_MOTION, [0x0000])

    def stop(self, mode=True):
        """
        运动停止

        Args:
            mode: True-缓停, False-急停
        """
        stop_type = 1 if mode else 0
        return self.write_multiple_regs(CommandAddress.STOP_MOTION, [stop_type])

    def jog_angle(self, joint, direction, speed, _async=True):
        """
        关节持续运动

        Args:
            joint: 关节号 1-6
            direction: 运动方向 0/1
            speed: 运动速度 1-100
            _async: 默认 True（对齐 pymycobot）；False 时等待到位反馈
        """
        values = [joint, direction, speed]
        return self.write_multiple_regs(
            CommandAddress.CONTINUOUS_JOINT, values, wait_arrival=True, _async=_async
        )

    def jog_coord(self, axis, direction, speed, _async=True):
        """
        坐标持续运动

        Args:
            axis: 坐标轴 1-6 (1:x, 2:y, 3:z, 4:rx, 5:ry, 6:rz)
            direction: 运动方向 0/1
            speed: 运动速度 1-100
            _async: 默认 True（对齐 pymycobot）；False 时等待到位反馈
        """
        values = [axis, direction, speed]
        return self.write_multiple_regs(
            CommandAddress.CONTINUOUS_COORD, values, wait_arrival=True, _async=_async
        )

    def jog_rpy(self, axis, direction, speed, _async=True):
        """
        RPY旋转

        Args:
            axis: 旋转轴 1-Roll, 2-Pitch, 3-Yaw
            direction: 旋转方向 0/1
            speed: 旋转速度 1-100
            _async: 默认 True（对齐 pymycobot）；False 时等待到位反馈
        """
        values = [axis, direction, speed]
        return self.write_multiple_regs(
            CommandAddress.RPY_ROTATION, values, wait_arrival=True, _async=_async
        )

    def jog_increment_angle(self, joint, step_angle, speed, _async=False):
        """
        角度步进

        Args:
            joint: 关节号 1-6
            step_angle: 步进角度（度）
            _async: True 只发送不等待到位，返回 1
        """
        step_int = int(step_angle * 100)
        if step_int < 0:
            step_int = 65536 + step_int

        values = [joint, step_int, speed]
        return self.write_multiple_regs(
            CommandAddress.STEP_ANGLE, values, wait_arrival=True, _async=_async
        )

    def jog_increment_coord(self, axis, step_value, speed, _async=False):
        """
        坐标步进

        Args:
            axis: 坐标轴 1-6
            step_value: 步进值 (x,y,z单位mm; rx,ry,rz单位度)
            _async: True 只发送不等待到位，返回 1
        """
        if axis <= 3:  # x, y, z
            step_int = int(step_value * 10)
        else:  # rx, ry, rz
            step_int = int(step_value * 100)

        if step_int < 0:
            step_int = 65536 + step_int

        values = [axis, step_int, speed]
        return self.write_multiple_regs(
            CommandAddress.STEP_COORD, values, wait_arrival=True, _async=_async
        )

    def set_motor_enable(self, joint, state):
        """
        设置关节使能状态

        Args:
            joint: 关节号 1-6, 254（所有关节）
            state: 0-放松, 1-使能
        """
        values = [joint, state]
        return self.write_multiple_regs(CommandAddress.MOTOR_ENABLE, values)

    def set_brake(self, joint, state):
        """
        设置关节刹车状态

        Args:
            joint: 关节号 1-6
            state: 0-释放, 1-刹车
        """
        values = [joint, state]
        return self.write_multiple_regs(CommandAddress.BRAKE_CONTROL, values)

    def set_base_io_output(self, pin_number, state):
        """
        设置底部IO状态

        Args:
            pin_number: 引脚号 1-12
            state: 0/1
        """
        values = [pin_number, state]
        return self.write_multiple_regs(CommandAddress.BASE_IO_OUTPUT, values)

    def set_digital_output(self, pin_number, state):
        """
        设置末端IO状态

        Args:
            pin_number: 引脚号 1-2
            state: 0/1
        """
        values = [pin_number, state]
        return self.write_multiple_regs(CommandAddress.DIGITAL_IO_OUTPUT, values)

    def set_color(self, r, g, b):
        """
        设置末端RGB颜色

        Args:
            r: 红色值 0-255
            g: 绿色值 0-255
            b: 蓝色值 0-255
        """
        values = [r, g, b]
        return self.write_multiple_regs(CommandAddress.RGB_COLOR, values)
