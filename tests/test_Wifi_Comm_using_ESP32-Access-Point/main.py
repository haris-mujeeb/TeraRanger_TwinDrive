import time
from robot_interface import RobotInterface

# Example usage of the RobotInterface class
if __name__ == "__main__":
  # Configuration details for the ESP32
  host_receive = '192.168.4.2'  # Python server IP
  port_receive = 12345           # Server port for receiving data
  host_send = '192.168.4.1'      # ESP32 Access Point IP
  port_send = 12345               # Server port for sending data

  robot = RobotInterface(host_receive, port_receive, host_send, port_send)
  robot.set_logging(True)
  received_data = robot.get_data_from_wifi()

  try:
    while True:
      # Example of sending data
      data_to_send = "Test data from PC to ESP32"
      robot.send_data_to_wifi(data_to_send)

      # Wait for a few seconds before sending again
      time.sleep(0.01)  # Adjust the delay as needed
          
  except KeyboardInterrupt:
    print("🛑 Program interrupted by user.")