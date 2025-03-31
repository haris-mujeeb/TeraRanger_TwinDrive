import socket
import threading
import logging
import logger_config
import math

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
    self.robot_angle = 0    
    self.robot_abs_dist = 0    
    self.robot_ultrasonic = 0    
    self.R_MAX = 700
    self.entry_angle = 0
    self.target_angle = 0
    self.target_distance = 0
    self.DISTANCE_CENTERLINE = 260
    self.DISTANCE_THREASHOLD = 25
    self.DELTA_THETA = 1

    self.logger = logging.getLogger(__name__)
    self.set_logging(False)


  def set_logging(self, enabled: bool):
    """Enable or disable logging."""
    self.logger.setLevel(logging.INFO if enabled else logging.WARNING)
    self.logger.info("Logging is %s.", "enabled" if enabled else "disabled")


  def _send_command(self, data):
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
          self.logger.debug(f"👂 Listening on {self.host_receive}:{self.port_receive}...")

          client_socket, client_address = server_socket.accept()
          self.logger.debug(f"🤝 Connection from {client_address}")

          while True:
            received_data = client_socket.recv(1024).decode().strip()
            if received_data:
              self.logger.debug(f"📥 Received: {received_data}")
              # client_socket.sendall(b"OK\n")
              self._parse_recieved_data(received_data)
            else:
              self.logger.debug("🚪 Connection closed.")
              break
      except ConnectionResetError:
        self.logger.warning("⚠️ Connection reset.")
      except KeyboardInterrupt:
        self.logger.error("🛑 Program interrupted by user.")
      except Exception as e:
        self.logger.error(f"❌ Error: {e}")
        self.logger.exception("Exception occurred") #for full stack trace.
  
  
  def _parse_recieved_data(self, data: str):
    """Parse incoming data based on its prefix."""
    if data.startswith("MF"):
      self._parse_tof_data(data)
    else:
      self._parse_robot_data(data)


  def _parse_robot_data(self, data: str):
    """Parse robot sensor data."""
    try:
      if not data:
        return
      values = data.strip().split(",")
      float_values = [float(value) for value in values]
      float_values = float_values
      self.robot_angle = float_values[0]
      self.robot_abs_dist = float_values[1]
      self.robot_ultrasonic = float_values[2]
      self.logger.info("Extracted Robot Sensor values: %s", float_values)
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

  # def _parse_recieved_data(self, data_string):
  #   parts = data_string.strip().split('\t')
  #   if len(parts) < 4:
  #     self.logger.error(f"⚠️ Error: Expected at least 4 parts in data string, but got {len(parts)}. Skipping.")
  #     return
    
  #   robot_data_parts = parts[:2]  # Convert the first three parts to integers
  #   tof_data_parts = parts[2:]  # Keep the rest as strings
    
  #   self._parse_robot_data(robot_data_parts)
  #   self._parse_tof_data(tof_data_parts)


  # def _parse_robot_data(self, data_parts):
  #   try:
  #     if not data_parts:
  #         self.logger.error("⚠️ Error: Robot data parts are empty.")
  #         return     
           
  #     robot_data_string = data_parts[0] # Changed to first part of parts.
  #     values_str = robot_data_string.split(",")
  #     float_values = [float(value) for value in values_str]
  #     robot_sensor_values = float_values
  #     self.robot_angle = robot_sensor_values[0]
  #     self.robot_abs_dist = robot_sensor_values[1]
  #     self.robot_ultrasonic = robot_sensor_values[2]
  #     self.logger.info("✅ Extracted Robot Sensor values: %s", robot_sensor_values)
  #   except ValueError as e:
  #       self.logger.error("❌ Error parsing Robot Sensor values: %s", e)
  #   except Exception as e:
  #       self.logger.error("❌ Unexpected error while parsing Robot Sensor data: %s", e)


  # def _parse_tof_data(self, data_parts):
  #   try:
  #     if not data_parts:
  #       self.logger.error("⚠️ Error: ToF data parts are empty.")
  #       return
      
  #     if len(data_parts) != 8:
  #         print(data_parts)
  #         self.logger.error(f"⚠️ Error: Expected 8 values after 'MF', but got {len(data_parts)}. Skipping this line.")
  #         return
        
  #     int_values = [int(value) for value in data_parts]
  #     self.tof_sensor_values = [int(value) for value in int_values]
  #     self.logger.info("✅ Extracted ToF values: %s", self.tof_sensor_values)
  #   except ValueError as e:
  #       self.logger.error("❌ Error parsing ToF values: %s", e)
  #   except Exception as e:
  #       self.logger.error("❌ Unexpected error while parsing ToF data: %s", e)


  def move_absolute(self, distance):
    self.target_distance = distance
    cmd = f"MOVE,{distance},10\n"
    self._send_command(cmd)


  def rotate_absolute(self, angle):
    cmd = f"TURN,{self.entry_angle + angle},10\n"
    self._send_command(cmd)


  def stop_robot(self):
    cmd = f"STOP,0,0\n"
    self._send_command(cmd)


  def obstacle_found(self):
    try:
      # Calculate average distances from ToF sensors
      dist_obs_front = (self.robot.tof_sensor_values[0] + self.robot.tof_sensor_values[7]) / 2
      dist_obs_left = (self.robot.tof_sensor_values[1] + self.robot.tof_sensor_values[2]) / 2
      dist_obs_back = (self.robot.tof_sensor_values[3] + self.robot.tof_sensor_values[4]) / 2
      dist_obs_right = (self.robot.tof_sensor_values[5] + self.robot.tof_sensor_values[6]) / 2
      robot_ultrasonic_value = self.robot.robot_sensor_values[2]

      # Check if any of the distances are smaller than 200 and return the corresponding side
      if dist_obs_front < 200:
          return "front"  # Obstacle detected in front
      if dist_obs_left < 200:
          return "left"  # Obstacle detected on the left
      if dist_obs_back < 200:
          return "back"  # Obstacle detected behind
      if dist_obs_right < 200:
          return "right"  # Obstacle detected on the right
      if robot_ultrasonic_value < 20:
          return "ultrasonic"  # Obstacle detected by ultrasonic sensor

      return None  # No obstacle found

    except IndexError as e:
      print(f"IndexError: {e}. Check if sensor values are properly initialized.")
      return None  # Return None or handle the error as needed

    except Exception as e:
      print(f"An unexpected error occurred: {e}")
      return None  # Return None or handle the error as needed


  def calculate_path(self, target_x, target_y):
    """Calculates the necessary angle and distance to move to a target point."""
    current_x = self.robot_abs_dist * math.cos(math.radians(self.robot_angle))
    current_y = self.robot_abs_dist * math.sin(math.radians(self.robot_angle))

    delta_x = target_x - current_x
    delta_y = target_y - current_y

    self.target_distance = math.sqrt(delta_x**2 + delta_y**2)
    self.target_angle = math.degrees(math.atan2(delta_y, delta_x))

    # Adjust angle based on the robot's current orientation
    self.target_angle = self.target_angle - self.robot_angle

    return self.target_distance, self.target_angle


  def navigate_to_point(self, target_x, target_y):
    """Navigates the robot to a specified target point using calculated path."""
    distance, angle = self.calculate_path(target_x, target_y)
    self.logger.info(f"Navigating to (x={target_x}, y={target_y}). Distance: {distance}, Angle: {angle}")

    self.target_distance = self.robot_abs_dist + distance # Make target_distance absolute.
    self.rotate_absolute(angle) #rotate to the target angle.

    obstacle = self.move_target()
    if obstacle:
        self.logger.warning(f"Obstacle detected: {obstacle}")
        return obstacle # return the obstacle that was detected.

    return None # No obstacle encountered.
  
  
  def dynamic_path_planning(self, target_x, target_y):
    """Dynamically plans and adjusts the path based on live sensor data."""
    while True:
      obstacle = self.navigate_to_point(target_x, target_y)
      if obstacle:
        # Re-plan the path based on the obstacle
        self.logger.info("Re-planning path...")
        # Example: Simple obstacle avoidance (adjust as needed)
        if obstacle == "front":
          self.rotate_absolute(90) #rotate 90 degrees
          self.move_absolute(200) #move 200 mm
          self.rotate_absolute(-90) #rotate back
        elif obstacle == "left":
          self.rotate_absolute(90)
          self.move_absolute(100)
          self.rotate_absolute(-90)
        elif obstacle == "right":
          self.rotate_absolute(-90)
          self.move_absolute(100)
          self.rotate_absolute(90)
        elif obstacle == "ultrasonic":
          self.stop_robot()
          self.logger.warning("Ultrasonic obstacle detected. Stopping.")
          return # stop, or re-plan differently.
        else:
          return #stop, or re-plan differently.

      else:
          # Target reached
          self.logger.info("Target reached.")
          return # stop, or move to next target.


  def move_target(self):
    distance_reached = abs(self.target_distance - self.robot_abs_dist) < 5
    angle_reached = abs(self.target_angle - self.robot_angle) < 3
    while not distance_reached and not angle_reached:
      self.move_absolute(self.target_distance)
      self.rotate_absolute(self.target_angle)
      if self.obstacle_found():
        self.stop_robot()
        return self.obstacle_found()
    return None


  def enter_the_maze(self):
    value = self.DISTANCE_CENTERLINE - self.tof_sensor_values[5]
    value = value / 30
    angle_radians = math.atan(value)
    self.target_angle = math.degrees(angle_radians)
    self._send_command()
    self.target_distance = 30
    
    # if self.tof_sensor_values[5] - self.tof_sensor_values[6] > 5:
    #   self.entry_angle = self.entry_angle - self.DELTA_THETA  # Rotate right
    #   self.target_angle = self.entry_angle
    #   cmd = f"TURN,{self.target_angle},10\n"
    #   self._send_command(cmd)
    #   return False

    # elif self.tof_sensor_values[6] - self.tof_sensor_values[5] > 5:
    #   self.entry_angle = self.entry_angle + self.DELTA_THETA  # Rotate left
    #   self.target_angle = self.entry_angle
    #   cmd = f"TURN,{self.target_angle},10\n"
    #   self._send_command(cmd)
    #   return False

    # else:
    #   return True


  def right_wall_follower(self):
    try:
      if ((self.tof_sensor_values[5] + self.tof_sensor_values[6]) * 0.5) < (self.DISTANCE_CENTERLINE - self.DISTANCE_THREASHOLD):
        if (self.target_distance > self.robot_abs_dist):
          self.target_angle = self.target_angle - self.DELTA_THETA  # Rotate right
        else:
          self.target_angle = self.target_angle + self.DELTA_THETA  # Rotate left

      elif ((self.tof_sensor_values[5] + self.tof_sensor_values[6]) * 0.5) > (self.DISTANCE_CENTERLINE + self.DISTANCE_THREASHOLD):
        if (self.target_distance > self.robot_abs_dist):
          self.target_angle = self.target_angle + self.DELTA_THETA  # Rotate left
        else:
          self.target_angle = self.target_angle - self.DELTA_THETA  # Rotate right

      cmd = f"TURN,{self.target_angle},10\n"
      self._send_command(cmd)
    except:
      pass


  def move_relative(self, rel_dist):
    try:
      if self.robot_ultrasonic > 22:
        distance = self.robot_abs_dist + rel_dist
        self.move_absolute(distance)
      else:
        self.stop_robot()
    except:
      pass