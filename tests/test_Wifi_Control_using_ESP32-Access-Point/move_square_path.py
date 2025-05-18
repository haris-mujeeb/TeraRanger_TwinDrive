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

  # Define the square movement pattern
  time.sleep(5)  # Wait for first data to be recieved
  commands = [
    # f"MOVE,{robot.robot_sensor_values[1] + 50},10",  # Move forward
    # f"TURN,90,10",  # Turn 90 degrees right
    # f"MOVE,{robot.robot_sensor_values[1] + 50},10",  # Move forward
    # f"TURN,180,10",  # Turn 90 degrees right
    # f"MOVE,{robot.robot_sensor_values[1] + 50},10",  # Move forward
    # f"TURN,270,10",  # Turn 90 degrees right
    # f"MOVE,{robot.robot_sensor_values[1] + 50},10",  # Move forward
    # f"TURN,360,10"   # Turn 90 degrees right (back to start orientation)
  ]

  try:
    for command in commands:
        robot.send_command(command)
        time.sleep(5)  # Wait for action completion
    # time.sleep(500)  # Wait for action completion

    print("✅ Square movement completed.")

  except KeyboardInterrupt:
    print("🛑 Program interrupted by user.")