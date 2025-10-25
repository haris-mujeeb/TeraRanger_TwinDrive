import sys
import time
import signal
import logging

from PyQt5 import QtWidgets

from robotInterface import RobotInterface
from mazeSolver import MazeSolver

# Initialize logger
main_logger = logging.getLogger(__name__)
main_logger.setLevel(logging.INFO)


def setup_robot_interface():
    """Initialize and configure the robot interface."""
    RECEIVE_HOST = '0.0.0.0'
    RECEIVE_PORT = 12346
    SEND_HOST = '192.168.4.1'
    SEND_PORT = 12345

    robot = RobotInterface(RECEIVE_HOST, RECEIVE_PORT, SEND_HOST, SEND_PORT)
    robot.set_logging_level(logging.INFO)
    robot.start_receiving()
    robot.send_command_to_robot("STOP,0,0")
    time.sleep(1)
    robot.save_sensor_data = True
    return robot


def main():
    app = QtWidgets.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    main_logger.info("🚀 Initializing Robot Interface...")
    robot = setup_robot_interface()

    main_logger.info("📈 Showing robot plot...")
    robot.plot_widget.show()
    robot.plot_widget.setFocus()

    main_logger.info("🧭 Starting MazeSolver...")
    maze_solver = MazeSolver(robot)
    maze_solver.set_logging_level(logging.DEBUG)
    maze_solver.show_plot()

    main_logger.info("✅ Running application...")
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
