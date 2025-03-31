import socket
import threading
import logging
import logger_config

# Set up logging configuration
logger_config.setup_logging()

class RobotInterface:
  def __init__(self, host_receive, port_receive, host_send, port_send):
    self.host_receive = host_receive
    self.port_receive = port_receive
    self.host_send = host_send
    self.port_send = port_send
    threading.Thread(target=self._get_data_from_wifi_loop, daemon=True).start()

    self.tof_sensor_values = []
    self.robot_sensor_values = []

    self.logger = logging.getLogger(__name__)
    self.set_logging(False)
    
    
  def set_logging(self, enabled: bool):
    """Enable or disable logging."""
    self.logger.setLevel(logging.INFO if enabled else logging.WARNING)
    self.logger.info("Logging is %s.", "enabled" if enabled else "disabled")
  
  
  def send_command(self, data):
    """Sends data to the ESP32."""
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((self.host_send, self.port_send))
        self.logger.info(f"🚀 Connected to {self.host_send}:{self.port_send}")
        client_socket.sendall(data.encode())
        self.logger.info(f"📤 Sent: {data}")
    except Exception as e:
      self.logger.error(f"❌ Send error: {e}")
      self.logger.exception("Exception occurred") #for full stack trace.
  
  
  def _get_data_from_wifi_loop(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
          server_socket.bind((self.host_receive, self.port_receive))
          server_socket.listen(1)
          self.logger.info(f"👂 Listening on {self.host_receive}:{self.port_receive}...")

          client_socket, client_address = server_socket.accept()
          self.logger.info(f"🤝 Connection from {client_address}")

          while True:
            received_data = client_socket.recv(1024).decode().strip()
            if received_data:
              self.logger.info(f"📥 Received: {received_data}")
              # client_socket.sendall(b"OK\n")
              self._parse_recieved_data(received_data)
            else:
              self.logger.info("🚪 Connection closed.")
              break
      except ConnectionResetError:
        self.logger.warning("⚠️ Connection reset.")
      except KeyboardInterrupt:
        self.logger.error("🛑 Program interrupted by user.")
      except Exception as e:
        self.logger.error(f"❌ Error: {e}")
        self.logger.exception("Exception occurred") #for full stack trace.


  def _parse_recieved_data(self, data_string):
    parts = data_string.strip().split('\t')
    if len(parts) < 4:
      self.logger.error(f"⚠️ Error: Expected at least 4 parts in data string, but got {len(parts)}. Skipping.")
      return
    
    robot_data_parts = parts[:2]  # Convert the first three parts to integers
    tof_data_parts = parts[2:]  # Keep the rest as strings
    
    self.parse_robot_data(robot_data_parts)
    self.parse_tof_data(tof_data_parts)


  def parse_robot_data(self, data_parts):
    try:
      if not data_parts:
          self.logger.error("⚠️ Error: Robot data parts are empty.")
          return     
           
      robot_data_string = data_parts[0] # Changed to first part of parts.
      values_str = robot_data_string.split(",")
      float_values = [float(value) for value in values_str]
      self.robot_sensor_values = float_values
      self.logger.info("✅ Extracted Robot Sensor values: %s", self.robot_sensor_values)
    except ValueError as e:
        self.logger.error("❌ Error parsing Robot Sensor values: %s", e)
    except Exception as e:
        self.logger.error("❌ Unexpected error while parsing Robot Sensor data: %s", e)
  
  
  def parse_tof_data(self, data_parts):
    try:
      if not data_parts:
        self.logger.error("⚠️ Error: ToF data parts are empty.")
        return
      
      if len(data_parts) != 8:
          print(data_parts)
          self.logger.error(f"⚠️ Error: Expected 8 values after 'MF', but got {len(data_parts)}. Skipping this line.")
          return
        
      int_values = [int(value) for value in data_parts]
      self.tof_sensor_values = [int(value) for value in int_values]
      self.logger.info("✅ Extracted ToF values: %s", self.tof_sensor_values)
    except ValueError as e:
        self.logger.error("❌ Error parsing ToF values: %s", e)
    except Exception as e:
        self.logger.error("❌ Unexpected error while parsing ToF data: %s", e)