import sys
import time
import threading
import signal
import logging

from robot_interface import RobotInterface
from PyQt5 import QtWidgets

# Initialize a logger for the main script
main_logger = logging.getLogger(__name__)

# It's good practice to set a default level for this logger too,
# especially if logger_config.py doesn't cover all loggers.
# For debugging the main script itself, INFO is often a good start.
main_logger.setLevel(logging.DEBUG)


if __name__ == "__main__":
    # Initialize the PyQt5 application. This MUST be the first PyQt operation.
    app = QtWidgets.QApplication(sys.argv)

    # Configure graceful exit on Ctrl+C.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Define network configuration for robot communication.
    # RECEIVE_HOST = '192.168.4.2'  # IP for receiving data from the robot
    RECEIVE_HOST = '0.0.0.0'      # IP for receiving data from the robot using UDP
    RECEIVE_PORT = 12346
    SEND_HOST = '192.168.4.1'     # Robot's IP for sending commands
    SEND_PORT = 12345

    # Instantiate the RobotInterface. It manages its own plotting and data reception.
    robot = RobotInterface(RECEIVE_HOST, RECEIVE_PORT, SEND_HOST, SEND_PORT)
    robot.set_logging_level(logging.INFO) # Set logging level for detailed feedback from RobotInterface
    robot.start_receiving()
    robot.send_command_to_esp("STOP,0,0") # Send an initial command to set the ESP32's pythonClientIP
    time.sleep(1)  # Wait for action completion

    # Display the real-time plot window.
    robot.plot_widget.show()
    robot.plot_widget.setFocus()
    main_logger.info("✨ PyQtGraph plot window is now open. ✨")
    

    # Define and start the robot's autonomous control loop in a separate thread.
    def run_robot_control_logic():
        """
        Manages the robot's autonomous behavior, sending commands
        based on sensor data or pre-programmed sequences.
        """
        main_logger.info("🤖 Starting robot control logic in a dedicated thread...")
        time.sleep(2) # Allow network connections and initial data reception to stabilize

        try:
          while True:
            # Start recieving telemetry
        
            distance = robot.get_robot_sensor_value(1)
            # if distance is None:
            #   print("⚠️ Sensor data temporarily unavailable. Retrying...")
            #   time.sleep(1)
            #   continue

            # commands = [
            #   f"MOVE,{distance + 50},10",
            #   "TURN,90,10",
            #   f"MOVE,{distance + 2*50},10",
            #   "TURN,180,10",
            #   f"MOVE,{distance + 3*50},10",
            #   "TURN,270,10",
            #   f"MOVE,{distance + 4*50},10",
            #   "TURN,360,10"
            # ]

            # for command in commands:
            #     robot.path_planning(command)
            #     time.sleep(0.1)  # Wait for action completion


            # print("✅ Square movement completed.")
            # break
            time.sleep(0.5)

        except KeyboardInterrupt:
            main_logger.critical("🛑 Robot control thread interrupted by user.")
        except Exception as e:
            main_logger.exception("❌ An unexpected error occurred in the robot control thread.") # Logs error with stack trace

    # Create and start the control thread. 'daemon=True' ensures it exits with the main program.
    robot_control_thread = threading.Thread(target=run_robot_control_logic, daemon=True)
    robot_control_thread.start()

    # Start the PyQt5 application's event loop. This blocks the main thread
    # until the GUI window is closed, keeping the application responsive.
    main_logger.info("📈 Application running. Close the plot window to exit. 📉")
    sys.exit(app.exec_())

    # This code executes only after the PyQtGraph window is closed.
    main_logger.info("Application shutdown complete.")