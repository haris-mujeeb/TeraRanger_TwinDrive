import time
from robot_interface import RobotInterface

# Example usage of the RobotInterface class
if __name__ == "__main__":
  # Configuration details for the ESP32
  host_receive = '192.168.4.2'  # Python server IP
  port_receive = 12345           # Server port for receiving data
  host_send = '192.168.4.1'      # ESP32 Access Point IP
  port_send = 12345               # Server port for sending data

  try:
    robot = RobotInterface(host_receive, port_receive, host_send, port_send)
  except KeyboardInterrupt:
    print("🛑 Program interrupted by user.")