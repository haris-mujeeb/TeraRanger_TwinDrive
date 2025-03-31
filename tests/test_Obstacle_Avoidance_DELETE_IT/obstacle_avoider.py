from robot_interface import RobotInterface
from time import sleep

class ObstacleAvoider:
  def __init__(self, robot: RobotInterface):
    self.robot = robot
    
    self.robot_angle = 0
    self.robot_distance = 0
    self.robot_ultrasonic_value = 0

    self.dist_obs_front = 0
    self.dist_obs_left = 0
    self.dist_obs_right = 0
    self.dist_obs_back = 0
    
    self.abs_angle = 0
    self.abs_x_dis = 0
    self.abs_y_dis = 0
    
    self.abs_target_distance = 0
    self.abs_target_angle = 0
    self.robot.logger.info("ObstacleAvoider initialized.")

  def _calculate(self):
    self.dist_obs_front = (self.robot.tof_sensor_values[0] + self.robot.tof_sensor_values[7])/2
    self.dist_obs_left = (self.robot.tof_sensor_values[1] + self.robot.tof_sensor_values[2])/2
    self.dist_obs_back = (self.robot.tof_sensor_values[3] + self.robot.tof_sensor_values[4])/2
    self.dist_obs_right = (self.robot.tof_sensor_values[5] + self.robot.tof_sensor_values[6])/2
    self.robot_angle = self.robot.robot_sensor_values[0]
    self.robot_distance = self.robot.robot_sensor_values[1]
    self.robot_ultrasonic_value = self.robot.robot_sensor_values[2]

  def _centering(self):
    if not self.robot.tof_sensor_values or len(self.robot.tof_sensor_values) < 8:
      return

    self._calculate()
    self.robot.logger.info(f"left side: {self.dist_obs_left}")
    self.robot.logger.info(f"right side: {self.dist_obs_right}")
    
    last_angle = self.robot_angle
    last_distance = self.robot_distance
    flag = True
    
    if (self.dist_obs_right < 200 and flag):
      # if command not send it would stuck
      while abs(self.robot_angle - (last_angle - 90)) > 3:
        command = "TURN"
        speed = 10
        value = last_angle - 90.0
        formatted_command = f"{command},{value},{speed}\n"
        self.robot.send_command(formatted_command)
        sleep(2)
      
      # while abs(self.robot_distance - (last_distance + 50)) > 5:
      #   command = "MOVE"
      #   speed = 10
      #   value = last_distance + 50
      #   formatted_command = f"{command},{value},{speed}\n"
      #   self.robot.send_command(formatted_command)
      #   sleep(1)
      
        
      # while abs(self.robot_angle - (last_angle)) > 3:
      #   command = "TURN"
      #   speed = 10
      #   value = last_angle
      #   formatted_command = f"{command},{value},{speed}\n"
      #   self.robot.send_command(formatted_command)
      #   sleep(0.1)

    # if (right_side > 250):
    #   command = "TURN"
    #   speed = 25
    #   value = self.abs_angle - 10
    #   formatted_command = f"{command},{value},{speed}\n"      
    #   self.robot.send_command(formatted_command)

  def _next_point(self):
    if not self.robot.tof_sensor_values or len(self.robot.tof_sensor_values) < 8:
      return
    
    if not self.robot.robot_sensor_values or len(self.robot.robot_sensor_values) < 3:
      return
        
    self.robot_angle = self.robot.robot_sensor_values[2]
    # self.
    self.dist_obs_front = (self.robot.tof_sensor_values[0] + self.robot.tof_sensor_values[7])/2
    
    if abs(self.dist_obs_front - self.abs_target_distance) < 200:
      command = "MOVE"
      value = self.abs_target_distance
      speed = 25
      self.robot.send_command(f"{command},{value},{speed}\n")