"""Basic usage example for MycobotPro450 Modbus library."""

from mycobotpro450_modbus import Pro450


def main() -> None:
    # Public API: import from package root (not from submodule paths).
    # Set debug=True to log SEND/RECV frames via standard logging.
    robot = Pro450("COM15", baudrate=115200, debug=False)
    try:
        print(robot.get_angles())
        print(robot.get_motor_temps())
    finally:
        robot.close()


if __name__ == "__main__":
    main()
