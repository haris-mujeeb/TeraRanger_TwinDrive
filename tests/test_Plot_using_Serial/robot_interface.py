import serial
import logging
import asyncio
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# Configure logging
logging.basicConfig(level=logging.WARNING, format='%(asctime)s - %(levelname)s - %(message)s')

class RobotInterface:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.tof_sensor_values = []
        self.robot_sensor_values = []
        self.ser = None
        self.logging_enabled = False
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop)
        self._thread.daemon = True
        self._thread.start()
        
        # Setup Plot
        self.R_MAX = 400  # Max radial distance
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.scat = self.ax.scatter([], [])
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self.ax.set_title("Live ToF Sensor Data (Polar)")

        # Start Animation
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, blit=False, interval=500)


    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def set_logging(self, enabled: bool):
        """Enable or disable logging."""
        self.logging_enabled = enabled
        if enabled:
            logging.getLogger().setLevel(logging.INFO)
            logging.info("Logging is enabled.")
        else:
            logging.getLogger().setLevel(logging.WARNING)
            logging.info("Logging is disabled.")

    def _connect_serial_sync(self):
        """Synchronous serial connection for internal use."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                logging.info(f"❌ Closed the serial port: {self.port}.")

            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            logging.info(f"✅ Connected to {self.port} at {self.baudrate} baud.")
            return True
        except serial.SerialException as e:
            logging.error(f"❌ Failed to connect to {self.port}: {e}")
            self.ser = None
            return False

    def _ensure_serial_connection_sync(self):
        """Synchronous connection check."""
        if not self.ser or not self.ser.is_open:
            while not self._connect_serial_sync():
                logging.warning("Retrying serial connection...")
                time.sleep(2)

    def _read_serial_data_sync(self):
        """Synchronous serial reading."""
        self._ensure_serial_connection_sync()
        try:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline()
                logging.info(f"Raw data: {raw_data}")
                self.parse_data(raw_data.decode('utf-8').strip())
        except (serial.SerialException, PermissionError) as e:
            logging.error(f"❌ Serial error: {e}. Reconnecting...")
            self.ser = None
        except Exception as e:
            logging.error(f"❌ Unexpected read error: {e}")

    def parse_data(self, data):
        """Parse the incoming serial data."""
        if data.startswith("RT"):
            self.parse_robot_data(data)
        if data.startswith("MF"):
            self.parse_tof_data(data)

    def parse_robot_data(self, data):
        try:
            values = data[3:].strip().split(",")
            float_values = [float(value) for value in values]
            self.robot_sensor_values = float_values
            logging.info("✅ Extracted Robot Sensor values: %s", self.robot_sensor_values)
        except ValueError as e:
            logging.error("❌ Error parsing Robot Sensor values: %s", e)
        except Exception as e:
            logging.error("❌ Unexpected error while parsing Robot Sensor data: %s", e)

    def parse_tof_data(self, data):
        try:
            values = data[3:].split('\t')
            if len(values) != 8:
                logging.error(f"⚠️ Error: Expected 8 values after 'MF', but got {len(values)}. Skipping this line.")
                return
            self.tof_sensor_values = [int(value) for value in values]
            logging.info("✅ Extracted ToF values: %s", self.tof_sensor_values)
        except ValueError as e:
            logging.error("❌ Error parsing ToF values: %s", e)
        except Exception as e:
            logging.error("❌ Unexpected error while parsing ToF data: %s", e)

    def send_command(self, command):
      """Send a command to the robot."""
      self._ensure_serial_connection_sync()
      try:
        delimited_command = f"{command}\n"  # Add newline delimiter
        self.ser.write(delimited_command.encode())
        logging.info(f"📤 Sent command: {command}")
        return True
      except (serial.SerialException, PermissionError) as e:
        logging.error(f"⚠️ Send error: {e}. Retrying...")
        self.ser = None
        return False
      except Exception as e:
        logging.error(f"⚠️ Send error: {e}")
        return False

    def start_reading(self):
      """Start reading serial data in a background thread."""
      def read_loop():
        while True:
          self._read_serial_data_sync()
          plt.show()
      threading.Thread(target=read_loop, daemon=True).start()
      plt.show()

    def close(self):
        """Close the serial port and stop the background thread."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logging.info(f"Serial port {self.port} closed.")
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join()
            logging.info("Async loop stopped")
            
    def update_plot(self, frame):
        """Update the polar plot with new ToF sensor data."""
        if self.tof_sensor_values:
            num_sensors = len(self.tof_sensor_values)
            angles = np.linspace(np.pi/8, (2 * np.pi) + (np.pi / 8), num_sensors, endpoint=False)
            adjusted_data = [self.R_MAX if val == -1 else val for val in self.tof_sensor_values]

            self.ax.clear()
            self.ax.set_theta_zero_location("N")
            self.ax.set_theta_direction(1)
            self.ax.set_rlim(0, self.R_MAX)
            self.ax.set_title("Live ToF Sensor Data (Polar)")
            self.ax.scatter(angles, adjusted_data)

        return self.scat,