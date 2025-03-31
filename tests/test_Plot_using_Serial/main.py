from robot_interface import RobotInterface
import time

robot = RobotInterface("COM11", 9600)
# robot.set_logging(True) # enable logging
robot.start_reading()

try:
  last_toggle = time.time()
  interval = 1
  command = "MOVE"
  speed = 25
  move_direction = 1


  while True:
    value = 10 * move_direction

    formatted_command = f"{command},{value},{speed}\n"

    if time.time() - last_toggle >= interval:
      move_direction *= -1
      last_toggle = time.time() 

    robot.send_command(formatted_command)
    time.sleep(0.5)

except KeyboardInterrupt:
  print("Program interrupted.")
finally:
  robot.close()