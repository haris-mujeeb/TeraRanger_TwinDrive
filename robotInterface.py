import socket
import threading
import logging
import logger_config
import csv
import os
from datetime import datetime
import time
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui
import collections

# Configure logging
logger_config.setup_logging()


class CustomPlotWidget(pg.PlotWidget):
    """
    A pg.PlotWidget subclass that handles keyboard events for robot control
    and mouse click events for target navigation.
    It passes key and mouse events to its associated RobotInterface instance.
    """

    def __init__(self, robot_interface, parent=None):
        super().__init__(parent)
        self.robot_interface = robot_interface  # Store reference to RobotInterface
        self.setFocusPolicy(QtCore.Qt.StrongFocus)  # Essential for receiving key events
        self.move_distance = 15  # Example value (cm), adjust as needed
        self.turn_angle = 15  # Example value (degrees), adjust as needed
        self.speed = 20  # Example value (unit/s), adjust as needed


    def keyPressEvent(self, event: QtGui.QKeyEvent):
        # Delegate the actual command logic to the RobotInterface
        # Pass the event and key-specific parameters (move_distance, turn_angle, speed)
        command_sent = self.robot_interface._handle_key_press(
            event, self.move_distance, self.turn_angle, self.speed
        )
        if not command_sent:
            super().keyPressEvent(event)


    def mousePressEvent(self, event: QtGui.QMouseEvent):
        """
        Handles mouse press events on the plot.
        If a left-click occurs, it attempts to move the robot to that location.
        """
        if event.button() == QtCore.Qt.LeftButton:
            # Get the position of the mouse click in the plot's view coordinates
            pos = self.plotItem.vb.mapSceneToView(event.pos())
            target_x = pos.x()
            target_y = pos.y()

            self.robot_interface.logger.info(f"Mouse clicked at plot coordinates: X={target_x:.2f}, Y={target_y:.2f}")
            
            self.robot_interface.move_to_xy(target_x, target_y, self.speed)
            event.accept() # Indicate that the event has been handled
        else:
            super().mousePressEvent(event) # Pass other mouse events up the chain


class RobotInterface:
    """
    Manages communication with a robot via Wi-Fi and provides 2D plotting
    of robot path and ToF sensor data.
    """
    def __init__(self, host_receive: str, port_receive: int, host_send: str, port_send: int):
        self.host_receive = host_receive
        self.port_receive = port_receive
        self.host_send = host_send
        self.port_send = port_send
        self.udp_socket = None
        self.receiving_thread = None
        self.running = False # Control flag for the receiving loop         

        
        
        # Sensor data storage, initialized for consistency and numerical operations
        self.plot_history_length = 1000 # Example: Keep last 3000 points visible on plot
        self._robot_x_history = collections.deque([0.0], maxlen=self.plot_history_length)
        self._robot_y_history = collections.deque([0.0], maxlen=self.plot_history_length)
        self._tof_data_history = collections.deque(maxlen=self.plot_history_length)
        self._current_x = 0.0
        self._current_y = 0.0
        self._tof_sensor_values = np.zeros(8) # 8 ToF sensors, values in millimeters
        self._robot_sensor_values = np.zeros(8) # 8 general robot sensor values
        
        self.prev_distance = 0.0
        self.prev_angle = 0.0

        # Relative angles for 8 ToF sensors, in radians
        # These angles are relative to the robot's forward direction.
        self.relative_angles_rad = np.deg2rad(np.arange(22.15, 342.15, 45))

        self._data_lock = threading.Lock() # Protects shared sensor data from race conditions

        self._current_move_thread: threading.Thread = None
        self._cancel_move_flag = threading.Event() # Event to signal cancellation

        # Logging setup
        self.logger = logging.getLogger(__name__)
        self.set_logging_level(logging.WARNING) # Default to warnings to minimize console output
        self.logging_enabled = False # Tracks current logging state

        # CSV 
        self.save_sensor_data_flag = False
        
        # PyQtGraph plotting setup
        self.logger.info("📈 Initializing live plot window...")
        self.plot_widget = CustomPlotWidget(self) # Pass self to CustomPlotWidget
        self.plot_widget.setWindowTitle("🤖 Robot Navigation Map")
        self.plot_widget.setLabel('bottom', "X Position", units='cm')
        self.plot_widget.setLabel('left', "Y Position", units='cm')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setAspectLocked(True) # Ensures proper scaling

        # Initialize plot items
        self.path_curve = self.plot_widget.plot(
            self._robot_x_history, self._robot_y_history,
            pen=pg.mkPen(color='b', width=2), symbol='o', symbolSize=5, name='Robot Path'
        )
        self.current_pos_scatter = pg.ScatterPlotItem(
            size=12, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0), symbol='o', name='Current Position'
        )
        self.plot_widget.addItem(self.current_pos_scatter)
        self.end_points_scatter = pg.ScatterPlotItem(
            x=[], y=[], pen=pg.mkPen('g', width=1), brush=pg.mkBrush('g'),
            size=3, symbol='s', name='ToF Endpoints'
        )
        self.plot_widget.addItem(self.end_points_scatter)
        
        self.arrow = pg.ArrowItem(angle=0, headLen=5, headWidth=20, tailLen=40, pxMode =True)
        self.plot_widget.addItem(self.arrow)
        
        self.robot_info_text = pg.TextItem(text="", anchor=(0, 0), color=(255, 255, 255))
        self.plot_widget.addItem(self.robot_info_text)
        self.robot_info_text.setPos(10, -30)

        # QTimer for periodic plot updates
        self.timer = QtCore.QTimer()
        self.timer.setInterval(10) # 10 updates per second
        self.timer.timeout.connect(self._update_plot)
        self.timer.start()

        # Tolerances for movement and turning
        self.ANGLE_TOLERANCE_DEG = 2.0  # Degrees
        self.DISTANCE_TOLERANCE_CM = 5.0 # Centimeters


    def set_logging_level(self, level: int):
        """Sets the logging level."""
        self.logger.setLevel(level)
        self.logging_enabled = (level <= logging.INFO)
        self.logger.debug(f"Logging set to: {logging.getLevelName(level)}")


    def start_receiving(self):
        """Starts the data receiving loop in a separate thread."""
        if not self.running:
            try:
                # Create and bind the socket ONLY ONCE here
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_socket.bind((self.host_receive, self.port_receive))
                self.udp_socket.settimeout(1) # Set a timeout for recvfrom

                self.running = True
                self.receiving_thread = threading.Thread(target=self._get_data_from_wifi_loop, daemon=True)
                self.receiving_thread.start()
                self.logger.info(f"👂Started UDP receiving thread.")
            except Exception as e:
                self.logger.error(f"❌ Failed to start UDP receiving: {e}", exc_info=True)
                self.running = False # Ensure flag is false if startup fails
        else:
            self.logger.warning("UDP receiving thread is already running.")


    def stop_receiving(self):
        """Stops the data receiving loop and closes the socket."""
        if self.running:
            self.running = False
            self.logger.info("Signaling UDP receiving thread to stop...")
            if self.receiving_thread and self.receiving_thread.is_alive():
                self.receiving_thread.join(timeout=2) # Give thread time to exit
                if self.receiving_thread.is_alive():
                    self.logger.warning("UDP receiving thread did not terminate gracefully.")
                else:
                    self.logger.info("UDP receiving thread stopped.")
            if self.udp_socket:
                self.udp_socket.close() # Close socket cleanly
            self.udp_socket = None # Clear reference
        else:
            self.logger.warning("UDP receiving thread is not running.")


    def send_command_to_robot(self, command_string):
        """
        Sends a command string to the ESP32 via UDP.
        
        :param command_string: The command string to send (e.g., "MOVE,100,50").
        """
        try:
            # Create a socket just for sending, or reuse the receiving socket if it's open.
            # Creating a new one for each command is simpler and safe for UDP.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as send_sock:
                send_sock.sendto(command_string.encode('utf-8'), (self.host_send, self.port_send))
                self.logger.debug(f"⬆️ Sent command to {self.host_send}:{self.port_send}: '{command_string}'")
        except Exception as e:
            self.logger.error(f"❌ Failed to send command: {e}", exc_info=True)
        
            
    def _get_data_from_wifi_loop(self):
        """Continuously receives data from the robot."""
        # Socket is now created and bound in start_receiving(),
        # so this loop just needs to receive.
        if not self.udp_socket:
            self.logger.error("❌ UDP socket not initialized. Receiving loop cannot start.")
            self.running = False
            return       
        
        # Create a UDP socket
        self.logger.debug(f"👂 Listening for UDP data on {self.host_receive}:{self.port_receive}")
        
        while self.running: # Use a class attribute to control the loop
            try:
                # Receive data (up to 1024 bytes) and the sender's address
                # The ESP32 will send from 192.168.4.1 (its AP IP) on some ephemeral port
                received_data, sender_address = self.udp_socket.recvfrom(255)
                
                # Decode and strip the received data
                decoded_data = received_data.decode('utf-8').strip()
                
                if decoded_data:
                    self.logger.debug(f"⬇️ Received from {sender_address}: '{decoded_data}'")
                    # You can optionally store the sender_address if you need to reply
                    # For this setup, we assume the ESP32 is sending unsolicited telemetry
                    
                    # Process the received data (e.g., update robot state)
                    self._parse_received_data(decoded_data)
                # No 'else' for connection closed by peer, as UDP is connectionless.
                # An empty packet might indicate a specific protocol message, but not a connection close.

            except socket.timeout:
                # No data received within the timeout period. This is normal.
                self.logger.debug(f"No data recieved during UDP data reception.")
                pass 
            except Exception as e:
                self.logger.error(f"❌ Error during UDP data reception: {e}", exc_info=True)
                # Consider a small pause to prevent a tight loop on continuous errors
                time.sleep(0.1) 


    def _parse_received_data(self, data_string: str):
        """Parses incoming data strings and dispatches to appropriate handlers."""
        # Split on the first occurrence of '\r\n' to handle multi-line messages
        parts = data_string.split('\r\n', 1)
        
        # Check for "MF" data (ToF sensors)
        if parts[0].startswith("MF\t"):
            self.parse_tof_data(parts[0])
        else:
            self.logger.warning(f"❓ Unknown data format in part 1: '{parts[0]}'")           
        
        # Check for "RB" data (Robot sensors)
        if len(parts) > 1 and parts[1].startswith("RB\t"):
            self.parse_robot_data(parts[1])
        elif len(parts) > 1: # If there's a second part but it's not "RB"
            self.logger.warning(f"❓ Unknown data format in part 2: '{parts[1]}'")
        elif len(parts) == 1 and not parts[0].startswith("MF\t"): # If only one part and it wasn't MF
            self.logger.warning(f"❓ Unknown single data format: '{data_string}'")
        

    def parse_robot_data(self, data_string: str):
        """Parses robot sensor data (gyro, distance, etc.)."""
        if '[ERROR]' in data_string:
          self.logger.error(f"❌ Error from Mobile Base: {data_string.split('[ERROR]')[1].strip()}")
          return
        
        if not data_string.startswith("RB\t"):
            self.logger.warning("⚠️ Invalid robot data format: Missing 'RB\\t' prefix.")
            return
        values_str = data_string[3:].split(',')
        try:
            if not values_str or all(not s.strip() for s in values_str):
                self.logger.warning("⚠️ Robot data values are empty or whitespace-only.")
                return

            float_values = [float(v.strip()) for v in values_str if v.strip()]
            with self._data_lock:
                self._robot_sensor_values = np.array(float_values)
            self.logger.debug("✅ Robot sensor values updated: %s", self._robot_sensor_values)
        except ValueError as e:
            self.logger.error(f"❌ Error parsing robot sensor values '{data_string}': {e}")
        except Exception as e:
            self.logger.error(f"❌ Unexpected error parsing robot data: {e}", exc_info=True)


    def parse_tof_data(self, data_string: str):
        """Parses ToF sensor data."""
        if not data_string.startswith("MF\t"):
            self.logger.warning("⚠️ Invalid ToF data format: Missing 'MF\\t' prefix.")
            return

        values_str = data_string[3:].split('\t')
        try:
            if not values_str or all(not s.strip() for s in values_str):
                self.logger.warning("⚠️ ToF data values are empty or whitespace-only.")
                return

            expected_sensors = len(self.relative_angles_rad)
            if len(values_str) != expected_sensors:
                self.logger.warning(
                    f"⚠️ ToF value count mismatch. Expected {expected_sensors}, got {len(values_str)} from '{data_string}'."
                )
                return

            float_values = [float(v.strip()) for v in values_str if v.strip()]
            with self._data_lock:
                self._tof_sensor_values = np.array(float_values)
                self._tof_sensor_values[1] = self._tof_sensor_values[1] # Distance in CM 
            self.logger.debug("✅ ToF sensor values updated: %s", self._tof_sensor_values)
        except ValueError as e:
            self.logger.error(f"❌ Error parsing ToF values '{data_string}': {e}")
        except Exception as e:
            self.logger.error(f"❌ Unexpected error parsing ToF data: {e}", exc_info=True)


    def save_sensor_data_to_csv(self, flag = True):
        self.save_sensor_data_flag = flag


    def _save_sensor_data_to_csv(self, filename: str = 'sensor_data.csv'):
        """Appends current sensor readings to a CSV file."""
        try:
            file_exists = os.path.isfile(filename)
            with open(filename, mode='a', newline='') as file:
                writer = csv.writer(file)

                # Write header if file doesn't exist AND there's data to represent
                if not file_exists and (len(self._tof_sensor_values) > 0 or len(self._robot_sensor_values) > 0):
                    header = ['Timestamp'] + \
                             [f'ToF_{i}' for i in range(len(self._tof_sensor_values))] + \
                             [f'Robot_{i}' for i in range(len(self._robot_sensor_values))]
                    writer.writerow(header)

                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                with self._data_lock:
                    # Convert NumPy arrays to lists for CSV writer compatibility
                    tof_data_list = self._tof_sensor_values.tolist()
                    robot_data_list = self._robot_sensor_values.tolist()

                    # Ensure consistent column count even if data is missing
                    if not tof_data_list and len(self.relative_angles_rad) > 0:
                        tof_data_list = [np.nan] * len(self.relative_angles_rad)
                    if not robot_data_list and 8 > 0: # Assuming 8 robot sensor values
                        robot_data_list = [np.nan] * 8

                    row = [f'"{timestamp}"'] + tof_data_list + robot_data_list
                writer.writerow(row)
                self.logger.debug("💾 Sensor data saved to CSV.")
        except Exception as e:
            self.logger.error(f"❌ Failed to write to CSV: {e}", exc_info=True)


    def _calculate_points_for_plot(self) -> bool:
        """
        Calculates and updates robot path and ToF endpoint coordinates.
        Always returns True if sufficient sensor data is available for processing,
        ensuring the plot is consistently updated.
        """
        with self._data_lock:
            # Check for sufficient robot sensor data
            if len(self._robot_sensor_values) < 2:
                self.logger.debug("Robot sensor data insufficient for plot calculation (need gyro and distance).")
                return False

            current_distance = self._robot_sensor_values[1]
            gyro_angle_degrees = self._robot_sensor_values[0]
            pitch_angle_degrees = self._robot_sensor_values[7]

            # Initialize prev_distance on the very first valid data point.
            # This ensures incremental_distance is correctly calculated from the second point onwards.
            if self._robot_x_history == [0.0] and self._robot_y_history == [0.0] and self.prev_distance == 0.0:
                self.prev_distance = current_distance
                self.logger.debug("First sensor data received. Initializing previous distance for path calculation.")
                # We proceed to calculate ToF points for this initial static position.
                # The robot's own path won't extend until actual movement.

            incremental_distance = current_distance - self.prev_distance
            self.prev_distance = current_distance # Update prev_distance for the next iteration

            gyro_angle_rad = np.deg2rad(gyro_angle_degrees) # Robot's absolute heading in radians
            pitch_angle_rad = np.deg2rad(pitch_angle_degrees)
            
            # Calculate incremental change in X and Y.
            # If incremental_distance is 0, dx and dy will be 0, and the same point
            # will be appended, correctly representing a stationary robot on the path.
            dx = incremental_distance * np.cos(gyro_angle_rad)
            dy = incremental_distance * np.sin(gyro_angle_rad)

            # Update robot's current position lists.
            self._current_x = self._robot_x_history[-1] + dx
            self._current_y = self._robot_y_history[-1] + dy
            self._robot_x_history.append(self._current_x)
            self._robot_y_history.append(self._current_y)
            self.logger.debug(f"Robot position calculated: X={self._robot_x_history[-1]:.2f}, Y={self._robot_y_history[-1]:.2f}")


            # Calculate and accumulate ToF endpoints (building a map).
            # This logic now always runs if ToF sensor data is available,
            # regardless of whether the robot's distance changed.
            if len(self._tof_sensor_values) > 0:
                # IMPORTANT: Adjust unit conversion (/10) if your ToF values are NOT in cm
                # and you intend for the plot to be in a different unit (e.g., cm).
                # 'slam_values' will be in the same unit as your plot axes.
                slam_values_in_cm = self._tof_sensor_values / 10 # Example: Converting mm to cm for plot

                angles_absolute = gyro_angle_rad + self.relative_angles_rad

                # Filter out invalid ToF readings (e.g., negative values)
                valid_indices = slam_values_in_cm >= 0
                valid_distances = slam_values_in_cm[valid_indices]
                valid_angles = angles_absolute[valid_indices]

                if len(valid_distances) > 0:
                    # Calculate absolute coordinates of ToF endpoint readings
                    end_x_coords = self._robot_x_history[-1] + valid_distances * np.cos(valid_angles) * np.cos(pitch_angle_rad)
                    end_y_coords = self._robot_y_history[-1] + valid_distances * np.sin(valid_angles) * np.cos(pitch_angle_rad)

                    # Stack new ToF points onto the accumulated 'end_points' array
                    for x_val, y_val in zip(end_x_coords, end_y_coords):
                      self._tof_data_history.append((x_val, y_val))
                    self.logger.debug(f"Added {len(self._tof_data_history)} ToF points. Total: {len(self._tof_data_history)}")
                else:
                    self.logger.debug("No valid ToF endpoints to plot for this scan.")
            else:
                self.logger.debug("No ToF sensor values available for point calculation.")

            # Always return True to ensure update_plot is triggered by the timer,
            # allowing the plot to refresh its display, even if only ToF data updated
            # or robot position remained static.
            self.logger.debug(f"📊 Plot data prepared. Robot: ({self._robot_x_history[-1]:.2f}, {self._robot_y_history[-1]:.2f}), ToF Map Points: {len(self._tof_data_history)}")
            return True

    def _update_plot(self):
        """Updates the 2D plot with the latest robot position and ToF data."""
        self._calculate_points_for_plot() # Recalculate points based on new sensor data

        # Update robot path line
        self.path_curve.setData(self._robot_x_history, self._robot_y_history)

        # Update current robot position marker
        if self._robot_x_history: # Check if points exist before accessing last element
          x, y = self._robot_x_history[-1], self._robot_y_history[-1]
          angle_deg = self._robot_sensor_values[0] if len(self._robot_sensor_values) > 0 else 0
          
          # Check if robot position or angle has changed
          if (x != self._robot_x_history[-2] or y != self._robot_y_history[-2] or angle_deg != self.prev_angle):
            self.current_pos_scatter.setData([x], [y])
            self.arrow.setPos(x, y)
            self.arrow.setStyle(angle=-angle_deg) # Adjust for PyQTGraph's arrow orientation
            
        else:
            self.current_pos_scatter.clear()
            self.arrow.hide()  # Hide the arrow if no position data
            self.robot_info_text.setText("")  # Clear the text if no position data

        # Update ToF endpoints (showing all accumulated points)
        if self._tof_data_history:
            all_tof_points = list(self._tof_data_history) 
            if all_tof_points: # Ensure the list is not empty before converting to array
                # Convert the list of (x,y) tuples into a 2D NumPy array (N, 2)
                # np.array can handle a list of tuples correctly to form a 2D array
                tof_points_array = np.array(all_tof_points)
                
                # Now, slice the 2D array for x and y coordinates
                self.end_points_scatter.setData(tof_points_array[:, 0], tof_points_array[:, 1])
            else:
                self.end_points_scatter.clear() # Clear if all_tof_points is empty
        else:
            self.end_points_scatter.clear()

        self.plot_widget.autoRange()
        # self.plot_widget.setXRange(-1000, 1000) # Example: x from -1000mm to +1000mm
        # self.plot_widget.setYRange(-1000, 1000) # Example: y from -1000mm to +1000mm

        # Invert the Y-axis to make down positive
        y_range = self.plot_widget.viewRange()[1]  # Get current Y range
        self.plot_widget.setYRange(y_range[1], y_range[0])  # Set Y range in reverse order

        if (self.save_sensor_data_flag):
            """Set the flag for saving sensor data to CSV."""
            self._save_sensor_data_to_csv()

    def _handle_key_press(self, event: QtGui.QKeyEvent, move_distance: float, turn_angle: float, speed: float) -> bool:
        """
        Processes key press events to generate and send robot commands.
        Called by the CustomPlotWidget's keyPressEvent.

        Args:
            event: The QtGui.QKeyEvent object.
            move_distance: The distance to move forward/backward.
            turn_angle: The angle to turn left/right.
            speed: The speed for movement/turning.

        Returns:
            True if a command was sent, False otherwise.
        """
        current_distance_in_cm = self._robot_sensor_values[1]
        current_angle = self._robot_sensor_values[0]
        command = ""
        self.logger.debug(f"Key pressed. Current robot angle: {current_angle}")

        # Define a dictionary to map keys to command generation logic
        key_actions = {
            QtCore.Qt.Key_W: lambda dist: f"MOVE,{dist + move_distance},{speed}",
            QtCore.Qt.Key_S: lambda dist: f"MOVE,{dist - move_distance},{speed}",
            QtCore.Qt.Key_A: lambda _: f"TURN,{current_angle + turn_angle},{speed}",
            QtCore.Qt.Key_D: lambda _: f"TURN,{current_angle - turn_angle},{speed}",
            QtCore.Qt.Key_R: lambda _: "STOP,0,0",
            QtCore.Qt.Key_G: lambda _: self._move_to_target_prompt(speed), # 'G' key for Go to Target (manual input)
        }

        # Check for 'MOVE' commands requiring distance sensor data first
        if event.key() in (QtCore.Qt.Key_W, QtCore.Qt.Key_S) and current_distance_in_cm is None:
            self.logger.warning("Sensor data (distance) unavailable for 'MOVE' command. Please wait for robot data.")
            return False  # Indicate that no command was sent

        # Check for 'TURN' commands requiring angle sensor data first
        if event.key() in (QtCore.Qt.Key_A, QtCore.Qt.Key_D) and current_angle is None:
            self.logger.warning("Sensor data (angle) unavailable for 'TURN' command. Please wait for robot data.")
            return False  # Indicate that no command was sent

        # Process the key if it's in our defined actions
        if event.key() in key_actions:
            if event.key() == QtCore.Qt.Key_G:
                key_actions[event.key()](speed)
                return True
            else:
                command = key_actions[event.key()](current_distance_in_cm) # Pass current_distance for MOVE, ignored by TURN
            self.logger.info(f"Sending command: {command}")
        else:
            return False  # Key not handled by robot commands

        if command:
            self.send_command_to_robot(command)
            return True
        return False


    def _move_to_target_prompt(self, speed: float):
        """
        Prompts the user for target x, y coordinates,
        then initiates the movement to that target.
        """
        try:
            target_x_str, ok_x = QtWidgets.QInputDialog.getText(self.plot_widget, 'Target X', 'Enter target X position (cm):')
            if not ok_x: return
            target_x = float(target_x_str)

            target_y_str, ok_y = QtWidgets.QInputDialog.getText(self.plot_widget, 'Target Y', 'Enter target Y position (cm):')
            if not ok_y: return
            target_y = float(target_y_str)

            # Start the movement in a new thread to avoid blocking the GUI
            self._start_new_move_thread(target_x, target_y, speed)

        except ValueError:
            self.logger.error("Invalid input for target coordinates or angle. Please enter numbers.")
        except Exception as e:
            self.logger.error(f"Error getting target input: {e}")

    def _cancel_current_move(self):
        """
        Signals the currently running _move_to_target thread to stop and waits for it.
        """
        if self._current_move_thread and self._current_move_thread.is_alive():
            self.logger.warning("🛑 New move request received. Cancelling current robot movement...")
            self.send_command_to_robot("STOP,0,0") # Send a STOP command to the robot
            self._cancel_move_flag.set() # Set the event to signal cancellation
            self._current_move_thread.join(timeout=2) # Wait a bit for the old thread to terminate
            if self._current_move_thread.is_alive():
                self.logger.error("❌ Previous move thread did not terminate gracefully.")
            else:
                self.logger.info("✅ Previous move thread terminated.")
        self._cancel_move_flag.clear() # Clear the flag for the new thread


    def _start_new_move_thread(self, target_x: float, target_y: float, speed: float):
        """
        Manages starting a new _move_to_target thread, ensuring only one runs at a time.
        """
        self._cancel_current_move() # Attempt to cancel any existing move

        self.logger.info(f"Starting new move thread to Target: X={target_x:.2f}cm, Y={target_y:.2f}cm at speed {speed}")
        self._current_move_thread = threading.Thread(
            target=self._move_to_target,
            args=(target_x, target_y, speed),
            daemon=True # Make daemon so it exits with main program
        )
        self._current_move_thread.start()


    def move_to_xy(self, target_x: float, target_y: float, speed: float):
        """
        Initiates robot movement to a target specified by a mouse click.
        This function is called by the CustomPlotWidget's mousePressEvent.
        """
        self.logger.info(f"Move to target: X={target_x:.2f}cm, Y={target_y:.2f}cm at speed {speed}")
        # Start the movement in a new thread to avoid blocking the GUI
        self._start_new_move_thread(target_x, target_y, speed)


    def _move_to_target(self, target_x: float, target_y: float, speed: float):
        """
        Moves the robot from its current position and angle to the specified
        target coordinates and angle. This is a blocking function and should be
        run in a separate thread.

        Args:
            target_x: The target X coordinate in cm.
            target_y: The target Y coordinate in cm.
            speed: The speed for movement and turning.
        """
        self.logger.info(f"Executing movement to Target: X={target_x:.2f}cm, Y={target_y:.2f}cm at speed {speed}")

        # Check for cancellation before starting any major step
        if self._cancel_move_flag.is_set():
            self.logger.info("Movement cancelled before starting (flag already set).")
            self._cancel_move_flag.clear() # Clear it for consistency
            return

        current_x = self._robot_x_history[-1] if self._robot_x_history else 0.0
        current_y = self._robot_y_history[-1] if self._robot_y_history else 0.0
        current_angle = self._robot_sensor_values[0] # Assuming index 0 is gyro angle

        if current_angle is None:
            self.logger.error("Cannot move to target: current robot angle not available.")
            return

        # # Normalize current_angle to 0-360 just in case it's not already
        # current_angle = (current_angle % 360 + 360) % 360

        # 1. Calculate initial turn to face the target position
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        
        angle_to_target_rad = np.arctan2(delta_y, delta_x)
        angle_to_target_deg = np.degrees(angle_to_target_rad)

        # # Normalize angle_to_target_deg to 0-360
        # angle_to_target_deg = (angle_to_target_deg % 360 + 360) % 360

        # Calculate the absolute target angle for the first turn command.
        # This is the direction the robot needs to *face* to point towards the target point.
        target_heading_for_move = angle_to_target_deg

        # Calculate the angular difference for the shortest turn
        # This part is for *calculating* the turn needed, not for sending the command directly.
        angle_diff_to_face_target = target_heading_for_move - current_angle
        if angle_diff_to_face_target > 180:
            angle_diff_to_face_target -= 360
        elif angle_diff_to_face_target < -180:
            angle_diff_to_face_target += 360

        # Check if a turn is needed based on tolerance
        if abs(angle_diff_to_face_target) > self.ANGLE_TOLERANCE_DEG:
            self.logger.info(f"Turning from {current_angle:.2f}° to face target point ({target_x:.2f}, {target_y:.2f}). Required turn: {angle_diff_to_face_target:.2f}°")
            
            # Send the absolute target heading to the robot's TURN command
            command_turn = f"TURN,{target_heading_for_move},{speed}" 
            self.send_command_to_robot(command_turn)
            
            # --- Active Wait for Turn Completion ---
            start_time = time.time()
            timeout = 10.0 # Max time to wait for turn (e.g., 10 seconds)
            
            while time.time() - start_time < timeout:
                if self._cancel_move_flag.is_set():
                    self.logger.info("Movement cancelled during initial turn.")
                    self.send_command_to_robot("STOP,0,0") # Stop the robot
                    return # Exit the function
              
                current_angle = self._robot_sensor_values[0]
                if current_angle is None:
                    self.logger.warning("No angle data while waiting for turn. Continuing to wait...")
                    time.sleep(0.1) # Small sleep to avoid busy-waiting without data
                    continue

                # Calculate the difference
                angle_diff = abs(target_heading_for_move - current_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff

                if angle_diff <= self.ANGLE_TOLERANCE_DEG:
                    self.logger.info(f"✅ Robot reached target angle within tolerance ({self.ANGLE_TOLERANCE_DEG}°). Current: {current_angle:.2f}° (Target: {target_heading_for_move:.2f}°)")
                    break
                time.sleep(0.05) # Check every 50ms
            else:
                self.logger.warning(f"❌ Robot did not reach initial turn target angle {target_heading_for_move:.2f}° within {timeout}s. Current: {current_angle:.2f}°")
                self.send_command_to_robot("STOP,0,0") # Attempt to stop the robot
                time.sleep(0.5)
            
            # Re-read current angle after turn (if it completed or timed out)
            # This is important for the next step's calculations.
            current_angle = self._robot_sensor_values[0]
            if current_angle is None:
                self.logger.error("Current angle not available after initial turn attempt.")
                return


        # 2. Move to the target position
        distance_to_target_in_cm = np.sqrt(delta_x**2 + delta_y**2)
        if distance_to_target_in_cm > self.DISTANCE_TOLERANCE_CM: # Only move if significant distance
            initial_robot_distance_in_cm = self._robot_sensor_values[1] # Assuming index 1 is distance
            if initial_robot_distance_in_cm is None:
                self.logger.error("Current robot distance not available for move command.")
                return

            # The MOVE command uses the *current* distance sensor reading as a baseline
            # and adds the desired incremental movement. We need to define a target
            # distance *value* that the robot's odometer should reach.
            target_distance_value_in_cm = initial_robot_distance_in_cm + distance_to_target_in_cm
            self.logger.info(f"Moving to target position. Distance needed: {distance_to_target_in_cm:.2f}cm. Target distance value: {target_distance_value_in_cm:.2f}")
            command_move = f"MOVE,{target_distance_value_in_cm},{speed}"
            self.send_command_to_robot(command_move)

            # --- Active Wait for Move Completion ---
            start_time = time.time()
            timeout = distance_to_target_in_cm / speed * 2.0 if speed > 0 else 10.0 # Double estimated time as timeout
            timeout = max(timeout, 10.0) # Ensure a minimum timeout

            while time.time() - start_time < timeout:
                current_distance_in_cm = self._robot_sensor_values[1]
                
                if current_distance_in_cm is None:
                    self.logger.warning("No distance data while waiting for move. Continuing to wait...")
                    time.sleep(0.1)
                    continue

                # Check if current distance is close to or past the target distance value
                # We check for abs(diff) AND if current is >= target to ensure it doesn't stop short if it's very close
                if abs(target_distance_value_in_cm - current_distance_in_cm) <= self.DISTANCE_TOLERANCE_CM or \
                   current_distance_in_cm >= target_distance_value_in_cm: # Added check for reaching/passing target
                    self.logger.info(f"✅ Robot reached target distance within tolerance ({self.DISTANCE_TOLERANCE_CM}cm) or passed. Current: {current_distance_in_cm:.2f}cm")
                    break
                
                # Also check if the robot has overshot significantly (could indicate an issue)
                if current_distance_in_cm > target_distance_value_in_cm + self.DISTANCE_TOLERANCE_CM * 5: # Overshot by more than 5x tolerance
                    self.logger.warning(f"⚠️ Robot significantly overshot target distance. Current: {current_distance_in_cm:.2f}cm, Target: {target_distance_value_in_cm:.2f}cm")
                    break # Stop waiting, proceed to next step
                
                time.sleep(0.05) # Check every 50ms
            else:
                self.logger.warning(f"❌ Robot did not reach target distance {target_distance_value_in_cm:.2f}cm within {timeout:.1f}s. Current: {current_distance_in_cm:.2f}cm")
                self.send_command_to_robot("STOP,0,0")
                time.sleep(0.5)

        self.logger.info("✅ Robot reached target position.")
    
    
    def rotate_to_degrees(self, angle: float, speed : float=25):
        self.send_command_to_robot(f"TURN,{angle},{speed}")
        self.logger.info(f"🔁 Turning to {angle:.2f}° with {speed}% speed")


    def stop_robot(self):
        self.send_command_to_robot("STOP,0,0")
        self.logger.info(f"⛔ STOP called !")
    
    @property
    def robot_x_position_history(self):
        """Returns a list of robot's X positions (history)."""
        return list(self._robot_x_history)

    @property
    def robot_y_position_history(self):
        """Returns a list of robot's Y positions (history)."""
        return list(self._robot_y_history)

    @property
    def tof_data_history(self):
        """Returns a list of ToF (x, y) tuples representing obstacle points."""
        return list(self._tof_data_history)

    @property
    def recent_robot_sensor_values(self):
        """Returns a list of ToF (x, y) tuples representing obstacle points."""
        return list(self._robot_sensor_values)
