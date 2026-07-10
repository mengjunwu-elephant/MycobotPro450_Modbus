from mycobotpro450_modbus import Pro450


def main() -> None:
    robot = Pro450("COM10", baudrate=115200)
    print(robot.get_angles())


if __name__ == "__main__":
    main()

