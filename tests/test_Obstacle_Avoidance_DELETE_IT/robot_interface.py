import serial
import threading
import time
import logging
import logger_config

# Set up logging configuration
logger_config.setup_logging()

class RobotInterface:
  def __init__(self, port: str, baudrate: int):
    """Initialize the RobotInterface with serial communication settings."""
    self.port = port
    self.baudrate = baudrate
    self.ser = None
    
    # Sensor values
    self.tof_sensor_values = []
    self.robot_sensor_values = []
    self.R_MAX = 700  # Maximum limit for ToF sensor values in centimeters

    # Command queue
    self.command_queue = []  
    
    # Logger
    self.logging_enabled = False
    self.logger = logging.getLogger(__name__)

    # Thread for serial communication
    self.running = True
    self._thread = threading.Thread(target=self._run_serial_loop, daemon=True)
    self._thread.start()

  def _run_serial_loop(self):
    """Continuously read from the serial port and send queued commands."""
    self._connect_serial()
    while self.running:
        self._read_serial_data()
        self._send_queued_commands()
        time.sleep(0.001)  # Prevent excessive CPU usage

  def set_logging(self, enabled: bool):
    """Enable or disable logging."""
    self.logger.setLevel(logging.INFO if enabled else logging.WARNING)
    self.logger.info("Logging is %s.", "enabled" if enabled else "disabled")

  def _connect_serial(self):
    """Establish a serial connection."""
    try:
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.logger.info(f"Closed the serial port: {self.port}.")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.logger.info(f"Connected to {self.port} at {self.baudrate} baud.")
    except serial.SerialException as e:
        self.logger.error(f"Failed to connect to {self.port}: {e}")
        self.ser = None

  def _ensure_serial_connection(self):
    """Ensure that the serial connection is established."""
    if not self.ser or not self.ser.is_open:
        self._connect_serial()

  def _read_serial_data(self):
    """Read data from the serial port and parse it."""
    self._ensure_serial_connection()
    try:
      if self.ser.in_waiting > 0:
        raw_data = self.ser.readline()
        self.logger.debug(f"Raw data: {raw_data}")
        self.parse_data(raw_data.decode('utf-8').strip())
    except UnicodeDecodeError as e:
      self.logger.warning(f"UnicodeDecodeError: {e}. Skipping this line.")
    except (serial.SerialException, PermissionError) as e:
      self.logger.error(f"Serial error: {e}. Reconnecting...")
      self.ser = None
    except Exception as e:
      self.logger.error(f"Unexpected read error: {e}")

  def parse_data(self, data: str):
    """Parse incoming data based on its prefix."""
    if data.startswith("RT"):
      self._parse_robot_data(data)
    elif data.startswith("MF"):
      self._parse_tof_data(data)
          
  def _parse_robot_data(self, data: str):
    """Parse robot sensor data."""
    try:
      values = data[3:].strip().split(",")
      float_values = [float(value) for value in values if self._is_valid_float(value)]
      self.robot_sensor_values = float_values
      self.logger.info("Extracted Robot Sensor values: %s", self.robot_sensor_values)
    except ValueError as e:
      self.logger.error("Error parsing Robot Sensor values: %s", e)
    except Exception as e:
      self.logger.error("Unexpected error while parsing Robot Sensor data: %s", e)

  def _parse_tof_data(self, data: str):
    """Parse Time-of-Flight sensor data."""
    try:
      values = data[3:].split('\t')
      if len(values) != 8:
          self.logger.error(f"Expected 8 values after 'MF', but got {len(values)}. Skipping this line.")
          return
      self.tof_sensor_values = [min(int(value), self.R_MAX) if value.isdigit() else self.R_MAX for value in values]
      self.logger.info("Extracted ToF values: %s", self.tof_sensor_values)
    except ValueError as e:
      self.logger.error("Error parsing ToF values: %s", e)
    except Exception as e:
      self.logger.error("Unexpected error while parsing ToF data: %s", e)

  def _is_valid_float(self, value: str) -> bool:
    """Check if a value can be converted to a float."""
    try:
      float(value)
      return True
    except ValueError:
      return False

  def send_command(self, command: str):
    """Queue a command to be sent to the robot."""
    self.command_queue.append(command)

  def _send_queued_commands(self):
    """Send commands from the queue to the robot."""
    self._ensure_serial_connection()
    while self.command_queue:
      command = self.command_queue.pop(0)
      try:
        delimited_command = f"{command}\n"
        self.ser.write(delimited_command.encode())
        self.logger.info(f"Sent command: {command}")
      except (serial.SerialException, PermissionError) as e:
        self.logger.error(f"Send error: {e}. Retrying...")
        self.ser = None
        break  # Stop sending commands if there's an error
      except Exception as e:
        self.logger.error(f"Send error: {e}")
        break

  def close(self):
    """Close the serial port and stop the communication thread."""
    self.running = False
    if self.ser and self.ser.is_open:
      self.ser.close()
      self.logger.info(f"Serial port {self.port} closed.")
    self._thread.join()
    self.logger.info("Serial thread stopped.")
