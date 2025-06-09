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
    A pg.PlotWidget subclass that handles keyboard events for robot control.
    It passes key events to its associated RobotInterface instance.
    """

    def __init__(self, robot_interface, parent=None):
        super().__init__(parent)
        self.robot_interface = robot_interface  # Store reference to RobotInterface
        self.setFocusPolicy(QtCore.Qt.StrongFocus)  # Essential for receiving key events

        # Define these properties as they are used in keyPressEvent.
        # You might want to make these configurable from RobotInterface or a config file.
        self.move_distance = 20  # Example value (cm), adjust as needed
        self.turn_angle = 20  # Example value (degrees), adjust as needed
        self.speed = 50  # Example value (unit/s), adjust as needed

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        # Delegate the actual command logic to the RobotInterface
        # Pass the event and key-specific parameters (move_distance, turn_angle, speed)
        command_sent = self.robot_interface._handle_key_press(
            event, self.move_distance, self.turn_angle, self.speed
        )
        if not command_sent:
            # If the RobotInterface didn't handle the key (e.g., it was an unrecognized key
            # or sensor data was missing for a 'MOVE' command), pass it up the chain.
            super().keyPressEvent(event)


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
        self.tof_sensor_values_mm = np.zeros(8) # 8 ToF sensors, values in millimeters
        self.robot_sensor_values = np.zeros(7) # 7 general robot sensor values

        # # Robot path points, stored as Python lists for efficient appending
        # self.x_points = [0.0]
        # self.y_points = [0.0]
        
        self.plot_history_length = 1000 # Example: Keep last 2000 points visible on plot
        self.x_points = collections.deque([0.0], maxlen=self.plot_history_length)
        self.y_points = collections.deque([0.0], maxlen=self.plot_history_length)

        # Accumulated ToF endpoint data for the map, starts empty
        # self.end_points = np.empty((0, 2))
        self.end_points = collections.deque(maxlen=self.plot_history_length)
        
        self.prev_distance = 0.0
        self.prev_angle = 0.0

        # Relative angles for 8 ToF sensors, in radians
        # These angles are relative to the robot's forward direction.
        self.relative_angles_rad = np.deg2rad(np.arange(22.15, 342.15, 45))

        self._data_lock = threading.Lock() # Protects shared sensor data from race conditions

        # Logging setup
        self.logger = logging.getLogger(__name__)
        self.set_logging_level(logging.WARNING) # Default to warnings to minimize console output
        self.logging_enabled = False # Tracks current logging state

        # PyQtGraph plotting setup
        self.logger.info("📈 Initializing live plot window...")
        self.plot_widget = CustomPlotWidget(self)
        self.plot_widget.setWindowTitle("Robot Navigation Map")
        self.plot_widget.setLabel('bottom', "X Position", units='mm')
        self.plot_widget.setLabel('left', "Y Position", units='mm')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setAspectLocked(True) # Ensures proper scaling

        # Initialize plot items
        self.path_curve = self.plot_widget.plot(
            self.x_points, self.y_points,
            pen=pg.mkPen(color='b', width=2), symbol='o', symbolSize=5, name='Robot Path'
        )
        self.current_pos_scatter = pg.ScatterPlotItem(
            size=12, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0), symbol='o', name='Current Position'
        )
        self.plot_widget.addItem(self.current_pos_scatter)
        self.end_points_scatter = pg.ScatterPlotItem(
            x=[], y=[], pen=pg.mkPen('g', width=1), brush=pg.mkBrush('g'),
            size=5, symbol='s', name='ToF Endpoints'
        )
        self.plot_widget.addItem(self.end_points_scatter)
        
        self.arrow = pg.ArrowItem(angle=0, headLen=40, headWidth=10, tailLen=10, brush='r', pxMode =True)
        self.plot_widget.addItem(self.arrow)
        
        self.robot_info_text = pg.TextItem(text="", anchor=(0, 0), color=(255, 255, 255))
        self.plot_widget.addItem(self.robot_info_text)
        self.robot_info_text.setPos(10, -30)

        # QTimer for periodic plot updates
        self.timer = QtCore.QTimer()
        self.timer.setInterval(200) # 10 updates per second
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()


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
                self.logger.info("Closed UDP socket.")
            self.udp_socket = None # Clear reference
        else:
            self.logger.warning("UDP receiving thread is not running.")


    def send_command_to_esp(self, command_string):
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
        part1, part2 = data_string.split('\r\n', 1)
        
        if part1.startswith("MF\t"):
            self.parse_tof_data(part1)
        else:
            self.logger.warning(f"❓ Unknown data format: '{data_string}'")          
        if part2.startswith("RB\t"):
            self.parse_robot_data(part2)
        else:
            self.logger.warning(f"❓ Unknown data format: '{data_string}'")          


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
                self.robot_sensor_values = np.array(float_values)
            self.logger.debug("✅ Robot sensor values updated: %s", self.robot_sensor_values)
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
                self.tof_sensor_values_mm = np.array(float_values)
            self.logger.debug("✅ ToF sensor values updated: %s", self.tof_sensor_values_mm)
        except ValueError as e:
            self.logger.error(f"❌ Error parsing ToF values '{data_string}': {e}")
        except Exception as e:
            self.logger.error(f"❌ Unexpected error parsing ToF data: {e}", exc_info=True)


    def get_robot_sensor_value(self, index: int, default=None):
        """Safely retrieves a robot sensor value by index."""
        with self._data_lock:
            try:
                return self.robot_sensor_values[index]
            except IndexError:
                self.logger.error(
                    f"❌ IndexError: Robot sensor value at index {index} out of range (len: {len(self.robot_sensor_values)})."
                )
                return default
            except Exception as e:
                self.logger.error(f"❌ Error accessing robot sensor value at {index}: {e}", exc_info=True)
                return default


    def get_tof_sensor_value(self, index: int, default=None):
        """Safely retrieves a ToF sensor value by index."""
        with self._data_lock:
            try:
                return self.tof_sensor_values_mm[index]
            except IndexError:
                self.logger.error(
                    f"❌ IndexError: ToF sensor value at index {index} out of range (len: {len(self.tof_sensor_values_mm)})."
                )
                return default
            except Exception as e:
                self.logger.error(f"❌ Error accessing ToF sensor value at {index}: {e}", exc_info=True)
                return default


    def save_sensor_data_to_csv(self, filename: str = 'sensor_data.csv'):
        """Appends current sensor readings to a CSV file."""
        try:
            file_exists = os.path.isfile(filename)
            with open(filename, mode='a', newline='') as file:
                writer = csv.writer(file)

                # Write header if file doesn't exist AND there's data to represent
                if not file_exists and (len(self.tof_sensor_values_mm) > 0 or len(self.robot_sensor_values) > 0):
                    header = ['Timestamp'] + \
                             [f'ToF_{i}' for i in range(len(self.tof_sensor_values_mm))] + \
                             [f'Robot_{i}' for i in range(len(self.robot_sensor_values))]
                    writer.writerow(header)

                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                with self._data_lock:
                    # Convert NumPy arrays to lists for CSV writer compatibility
                    tof_data_list = self.tof_sensor_values_mm.tolist()
                    robot_data_list = self.robot_sensor_values.tolist()

                    # Ensure consistent column count even if data is missing
                    if not tof_data_list and len(self.relative_angles_rad) > 0:
                        tof_data_list = [np.nan] * len(self.relative_angles_rad)
                    if not robot_data_list and 7 > 0: # Assuming 7 robot sensor values
                        robot_data_list = [np.nan] * 7

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
            if len(self.robot_sensor_values) < 2:
                self.logger.debug("Robot sensor data insufficient for plot calculation (need gyro and distance).")
                return False

            current_distance = self.robot_sensor_values[1]
            gyro_angle = -self.robot_sensor_values[0]

            # Initialize prev_distance on the very first valid data point.
            # This ensures incremental_distance is correctly calculated from the second point onwards.
            if self.x_points == [0.0] and self.y_points == [0.0] and self.prev_distance == 0.0:
                self.prev_distance = current_distance
                self.logger.debug("First sensor data received. Initializing previous distance for path calculation.")
                # We proceed to calculate ToF points for this initial static position.
                # The robot's own path won't extend until actual movement.

            incremental_distance = current_distance - self.prev_distance
            self.prev_distance = current_distance # Update prev_distance for the next iteration

            theta = np.deg2rad(gyro_angle) # Robot's absolute heading in radians

            # Calculate incremental change in X and Y.
            # If incremental_distance is 0, dx and dy will be 0, and the same point
            # will be appended, correctly representing a stationary robot on the path.
            dx = incremental_distance * np.cos(theta)
            dy = incremental_distance * np.sin(theta)

            # Update robot's current position lists.
            self.x_points.append(self.x_points[-1] + dx)
            self.y_points.append(self.y_points[-1] + dy)
            self.logger.debug(f"Robot position calculated: X={self.x_points[-1]:.2f}, Y={self.y_points[-1]:.2f}")


            # Calculate and accumulate ToF endpoints (building a map).
            # This logic now always runs if ToF sensor data is available,
            # regardless of whether the robot's distance changed.
            if len(self.tof_sensor_values_mm) > 0:
                # IMPORTANT: Adjust unit conversion (/10) if your ToF values are NOT in mm
                # and you intend for the plot to be in a different unit (e.g., cm).
                # 'slam_values' will be in the same unit as your plot axes.
                slam_values = self.tof_sensor_values_mm / 10 # Example: Converting mm to cm for plot

                angles_absolute = theta + self.relative_angles_rad

                # Filter out invalid ToF readings (e.g., negative values)
                valid_indices = slam_values >= 0
                valid_distances = slam_values[valid_indices]
                valid_angles = angles_absolute[valid_indices]

                if len(valid_distances) > 0:
                    # Calculate absolute coordinates of ToF endpoint readings
                    end_x_coords = self.x_points[-1] + valid_distances * np.cos(valid_angles)
                    end_y_coords = self.y_points[-1] + valid_distances * np.sin(valid_angles)

                    # Stack new ToF points onto the accumulated 'end_points' array
                    new_tof_points = np.c_[end_x_coords, end_y_coords]
                    for x_val, y_val in zip(end_x_coords, end_y_coords):
                      self.end_points.append((x_val, y_val))
                    self.logger.debug(f"Added {len(new_tof_points)} ToF points. Total: {len(self.end_points)}")
                else:
                    self.logger.debug("No valid ToF endpoints to plot for this scan.")
            else:
                self.logger.debug("No ToF sensor values available for point calculation.")

            # Always return True to ensure update_plot is triggered by the timer,
            # allowing the plot to refresh its display, even if only ToF data updated
            # or robot position remained static.
            self.logger.debug(f"📊 Plot data prepared. Robot: ({self.x_points[-1]:.2f}, {self.y_points[-1]:.2f}), ToF Map Points: {len(self.end_points)}")
            return True


    def update_plot(self):
        """Updates the 2D plot with the latest robot position and ToF data."""
        self._calculate_points_for_plot() # Recalculate points based on new sensor data

        # Update robot path line
        self.path_curve.setData(self.x_points, self.y_points)

        # Update current robot position marker
        if self.x_points: # Check if points exist before accessing last element
          x, y = self.x_points[-1], self.y_points[-1]
          angle_deg = self.robot_sensor_values[0] if len(self.robot_sensor_values) > 0 else 0
          
          # Check if robot position or angle has changed
          if (x != self.x_points[-2] or y != self.y_points[-2] or angle_deg != self.prev_angle):
            self.current_pos_scatter.setData([x], [y])
            self.arrow.setPos(x, y)
            self.arrow.setStyle(angle=angle_deg + 180)
            
        else:
            self.current_pos_scatter.clear()
            self.arrow.hide()  # Hide the arrow if no position data
            self.robot_info_text.setText("")  # Clear the text if no position data

        # Update ToF endpoints (showing all accumulated points)
        if self.end_points:
            all_tof_points = list(self.end_points) 
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
        current_distance = self.get_robot_sensor_value(1)
        current_angle = self.get_robot_sensor_value(0)
        command = ""
        print("Dis", current_distance)
        print("Angle", current_angle)
        # Define a dictionary to map keys to command generation logic
        # This makes the code cleaner and easier to extend.
        key_actions = {
            QtCore.Qt.Key_W: lambda dist: f"MOVE,{dist + move_distance},{speed}",
            QtCore.Qt.Key_S: lambda dist: f"MOVE,{dist - move_distance},{speed}",
            QtCore.Qt.Key_A: lambda _: f"TURN,{current_angle - turn_angle},{speed}",
            QtCore.Qt.Key_D: lambda _: f"TURN,{current_angle + turn_angle},{speed}",
            QtCore.Qt.Key_R: lambda _: "STOP,0,0",
        }

        # Check for 'MOVE' commands requiring sensor data first
        if event.key() in (QtCore.Qt.Key_W, QtCore.Qt.Key_S) and current_distance is None:
            self.logger.warning("Sensor data unavailable for 'MOVE' command. Please wait for robot data.")
            return False  # Indicate that no command was sent

        # Process the key if it's in our defined actions
        if event.key() in key_actions:
            # For 'MOVE' commands, current_distance is passed; for others, it's ignored
            command = key_actions[event.key()](current_distance)
            print(f"Sending command: {command}")
        else:
            return False  # Key not handled by robot commands

        if command:
            self.send_command_to_esp(command)
            return True
        return False

    def path_planning(self, command: str):
        """Sends a command for path planning."""
        self.send_command_to_esp(command)