from PyQt5.QtCore import QObject
import pyqtgraph as pg
import threading
import numpy as np
import time
import logging
import logger_config


# Configure logging
logger_config.setup_logging()

class MazeSolver(QObject):
    def __init__(self, robot_interface):
        super().__init__()
        self._robot = robot_interface
        self._plot_history_length = 300
        self._tof_history_array = np.zeros(self._plot_history_length)
        self._robot_x = np.zeros(self._plot_history_length)
        self._robot_y = np.zeros(self._plot_history_length)
        
        self._movement_speed = 20
        self._forward_step_in_cm = 20
        self._clearance_from_the_front_wall = 20
        self._clearance_from_the_right_wall = 20
        self._rotation_speed = 20
        
        self._front_wall_points_rel = None
        self._left_wall_points_rel = None
        self._right_wall_points_rel = None

        self._search_angle_degrees = 45
        self._data_points_distance_threshold = 12
        self._data_point_depth_tolerance_threshold = 40   
                     
        self._max_side_is_obstacle_dist_threshold = 40
        self._max_front_is_obstacle_dist_threshold = 40
        self.average_front_wall_distance = None
        self.average_left_wall_distance = None
        self.average_right_wall_distance = None
        
        self.isFrontObstacle = False
        self.isLeftObstacle = False
        self.isRightObstacle = False
        
          # Tolerances for movement and turning
        self.ANGLE_TOLERANCE_DEG = 2.0  # Degrees
        self.DISTANCE_TOLERANCE_CM = 10.0 # Centimeters
        self._target_dist = None
        self._last_command_time = 0.0 # Initialize to 0.0 or time.time()
        self._movement_timeout_sec = 5.0 # Define the timeout duration
        
        self._running = True

        self.logger = logging.getLogger(__name__)
        self.set_logging_level(logging.INFO) # Default to info to minimize console output
        self.logging_enabled = False # Tracks current logging state
        
        self._init_plot()
        self._control_thread = threading.Thread(target=self._run_solver_loop, daemon=True)
        # time.sleep(1)
        self._control_thread.start()


    def set_logging_level(self, level: int):
        """Sets the logging level."""
        self.logger.setLevel(level)
        self.logging_enabled = (level <= logging.INFO)
        self.logger.debug(f"Logging set to: {logging.getLevelName(level)}")


    def _init_plot(self):
        """Initialize the plot window and its components."""
        self.logger.info("📈 Initializing plot for Maze Solving debug...")
        self.plot_window = pg.plot(title="🧭 Maze Solver Visualization")
        self.robot_curve = self.plot_window.plot(pen='g', name="Robot Path")
        self.box_curve = self.plot_window.plot(pen='r', name="Obstacle Box")
        self.tof_scatter = pg.ScatterPlotItem(
            x=[], y=[], pen=pg.mkPen('g', width=1), brush=pg.mkBrush('g'),
            size=2, symbol='s', name='ToF Endpoints'
        )
        self.front_wall_scatter = pg.ScatterPlotItem(
            x=[], y=[], pen=pg.mkPen('w', width=1), brush=pg.mkBrush('orange'),
            size=5, symbol='s', name='ToF Endpoints'
        )
        self.left_wall_scatter = pg.ScatterPlotItem(
            x=[], y=[], pen=pg.mkPen('w', width=1), brush=pg.mkBrush('b'),
            size=5, symbol='s', name='Left Wall Endpoints'
        )
        self.right_wall_scatter = pg.ScatterPlotItem(
            x=[], y=[], pen=pg.mkPen('w', width=1), brush=pg.mkBrush('r'),
            size=5, symbol='s', name='Right Wall Endpoints'
        )
        self.fitted_right_wall_line = self.plot_window.plot(pen=pg.mkPen('y', width=2), name="Fitted Right Wall")  # NEW LINE
        self.plot_window.addItem(self.tof_scatter)
        self.plot_window.addItem(self.front_wall_scatter)
        self.plot_window.addItem(self.left_wall_scatter)
        self.plot_window.addItem(self.right_wall_scatter)
        self.plot_window.addItem(self.fitted_right_wall_line)
        self.current_pos_scatter = pg.ScatterPlotItem(
            size=12, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0), symbol='o', name='Current Position'
        )
        self.plot_window.addItem(self.current_pos_scatter)

    def show_plot(self):
        """Display the plot window."""
        self.plot_window.show()


    def _run_solver_loop(self):
      while self._running:
        """Main loop for the maze solver."""
        current_angle = self._robot.recent_robot_sensor_values[0]
        self._robot_x = np.array(self._robot._robot_y_history)
        self._robot_y = np.array(self._robot._robot_x_history)
        self._tof_history_array = np.array(self._robot.tof_data_history)
        
        self._check_where_am_i(current_angle)
        
        self._decide_what_to_do()
                
        time.sleep(0.1)


    def _rotate_and_get_more_data(self, angle):
        """Check for obstacles in the robot's path."""
        self._robot.rotate_to_degrees(angle + self._search_angle_degrees, self._rotation_speed)

        time.sleep(2)

        self._robot.rotate_to_degrees(angle, self._rotation_speed)

        # time.sleep(2)


    def _check_where_am_i(self, angle):
        """Determine the robot's position and update the plot."""
        # Ensure robot position history is updated frequently by the robot interface
        # before using it here.
        # This assumes _robot_x_history and _robot_y_history in self._robot are always current.
        self._robot_x = self._robot._current_x # Corrected access
        self._robot_y = self._robot._current_y # Corrected access

        if not self._robot_x or not self._robot_y:
            self.logger.warning("Robot position history is empty. Cannot update localization.")
            return
        
        # Always get the latest ToF data
        self._tof_history_array  = np.array(self._robot._tof_data_history)

        if self._tof_history_array.size == 0 or self._tof_history_array.ndim < 2:
            self.logger.info("No valid ToF data available for obstacle detection.")
            self.average_front_wall_distance = None
            self.isFrontObstacle = False
            self.front_wall_scatter.clear()
            self.average_left_wall_distance = None
            self.isLeftObstacle = False
            self.left_wall_scatter.clear()
            self.average_right_wall_distance = None
            self.isRightObstacle = False
            self.right_wall_scatter.clear()
            return

        x_points_abs = self._tof_history_array[:, 0]
        y_points_abs = self._tof_history_array[:, 1]

        # Get the current robot center for relative calculations
        x_c = self._robot_x
        y_c = self._robot_y

        # Use the first _convert_points_to_inertial_frame if you want to convert
        # global ToF points to points relative to the robot's current frame.
        # The second one converts relative points to global, which isn't what
        # you want for `__find_front_wall_points` etc.
        # So, we pass the absolute ToF points and the robot's current absolute angle
        # to get points relative to the robot's current (x_c, y_c) and orientation.
        rotated_points_rel_to_robot = self._convert_points_to_robot_frame(x_points_abs, y_points_abs, angle, x_c, y_c) # New helper
        
        x_points_rel = rotated_points_rel_to_robot[:, 0]
        y_points_rel = rotated_points_rel_to_robot[:, 1]

        if x_points_rel.size and y_points_rel.size:
            self.tof_scatter.setData(x_points_rel, y_points_rel)
        else:
            self.tof_scatter.clear() # Clear if no points

        # The find_wall_points methods should work with points relative to the robot's center (0,0)
        # when we've already transformed them.
        
        self._front_wall_points_rel = self.__find_front_wall_points(x_points_rel, y_points_rel, 0, 0) # Use 0,0 as center
        self._left_wall_points_rel = self.__find_left_wall_points(x_points_rel, y_points_rel, 0, 0)    # Use 0,0 as center
        self._right_wall_points_rel = self.__find_right_wall_points(x_points_rel, y_points_rel, 0, 0)  # Use 0,0 as center


        def _filtered_mean(data):
            return abs(float(np.mean(data)))

        # --- Front Wall ---
        if self._front_wall_points_rel.shape[0] >= 2:
            dx = self._front_wall_points_rel[:, 0]
            # Average should just be over dx as it's already relative to robot's front
            self.average_front_wall_distance = _filtered_mean(dx) 
            self.front_wall_scatter.setData(self._front_wall_points_rel[:, 0], self._front_wall_points_rel[:, 1])
            self.isFrontObstacle = (
                self.average_front_wall_distance is not None and
                self.average_front_wall_distance < self._max_front_is_obstacle_dist_threshold # Removed abs as it's already positive distance
            )
        else:
            self.average_front_wall_distance = None
            self.isFrontObstacle = False
            self.front_wall_scatter.clear()


        # --- Left Wall ---
        if self._left_wall_points_rel.shape[0] >= 2:
            dy = self._left_wall_points_rel[:, 1]
            # For left wall, we're interested in the positive y-axis
            self.average_left_wall_distance = _filtered_mean(dy) 
            self.left_wall_scatter.setData(self._left_wall_points_rel[:, 0], self._left_wall_points_rel[:, 1])
            self.isLeftObstacle = (
                self.average_left_wall_distance is not None and
                self.average_left_wall_distance < self._max_side_is_obstacle_dist_threshold
            )
        else:
            self.average_left_wall_distance = None
            self.isLeftObstacle = False
            self.left_wall_scatter.clear()


        # --- Right Wall ---
        if self._right_wall_points_rel.shape[0] >= 2:
            dy = self._right_wall_points_rel[:, 1]
            # For right wall, we're interested in the negative y-axis values, but the distance is positive
            self.average_right_wall_distance = _filtered_mean(abs(dy)) # Take absolute to get positive distance
            self.right_wall_scatter.setData(self._right_wall_points_rel[:, 0], self._right_wall_points_rel[:, 1])
            self.isRightObstacle = (
                self.average_right_wall_distance is not None and
                self.average_right_wall_distance < self._max_side_is_obstacle_dist_threshold
            )
        else:
            self.average_right_wall_distance = None
            self.isRightObstacle = False
            self.right_wall_scatter.clear()

        # Logging the obstacle status
        self.logger.info(f'Average Front Wall Distance: {self.average_front_wall_distance:.2f}' if self.average_front_wall_distance is not None else 'Average Front Wall Distance: N/A')
        self.logger.info(f'⬆️ Front is blocked: {self.isFrontObstacle} {"✅" if not self.isFrontObstacle else "⛔"}')
        self.logger.info(f'Average Left Wall Distance: {self.average_left_wall_distance:.2f}' if self.average_left_wall_distance is not None else 'Average Left Wall Distance: N/A')
        self.logger.info(f'⬅️ Left is blocked: {self.isLeftObstacle} {"✅" if not self.isLeftObstacle else "⛔"}')
        self.logger.info(f'Average Right Wall Distance: {self.average_right_wall_distance:.2f}' if self.average_right_wall_distance is not None else 'Average Right Wall Distance: N/A')
        self.logger.info(f'➡️ Right is blocked: {self.isRightObstacle} {"✅" if not self.isRightObstacle else "⛔"}')

        # The time.sleep(0.1) and clearing scatters here might make your plot flicker
        # and prevent continuous visualization. You might want to update the plot
        # at a slower rate or in a dedicated plotting function, not within
        # _check_where_am_i if it's called very frequently.
        # For now, let's keep it to clear for the next frame.
        
        self.current_pos_scatter.setData([0], [0])
        
        time.sleep(0.1) 
        self.tof_scatter.clear()
        self.front_wall_scatter.clear()
        self.left_wall_scatter.clear()
        self.right_wall_scatter.clear()
        self.current_pos_scatter.clear()



    def _convert_points_to_inertial_frame(self, x_vals, y_vals, angle):
        """Rotate points to the robot's relative frame."""
        # Convert input lists to NumPy arrays if they are not already
        x_vals = np.array(x_vals)
        y_vals = np.array(y_vals)

        theta = np.deg2rad(angle)
        x_c = self._robot_x
        y_c = self._robot_y

        # Translate points to origin
        x_translated = x_vals - x_c
        y_translated = y_vals - y_c

        # Apply rotation
        x_rotated = x_translated * np.cos(theta) - y_translated * np.sin(theta)
        y_rotated = x_translated * np.sin(theta) + y_translated * np.cos(theta)

        # Translate points back to original position
        return np.column_stack((x_rotated, y_rotated))

    def _convert_points_to_inertial_frame(self, x_vals_rel, y_vals_rel, angle_abs_degrees):
        """Rotate points to the robot's relative frame."""
        # Convert input lists to NumPy arrays if they are not already
        x_vals_rel = np.array(x_vals_rel)
        y_vals_rel = np.array(y_vals_rel)

        theta = np.deg2rad(angle_abs_degrees)

        # Apply rotation
        x_rotated = x_vals_rel * np.cos(theta) - y_vals_rel * np.sin(theta)
        y_rotated = x_vals_rel * np.sin(theta) + y_vals_rel * np.cos(theta)
        
                
        x_c = self._robot._robot_x_history
        y_c = self._robot._robot_y_history

        # Translate points back to original position
        return np.column_stack((x_rotated + x_c, y_rotated + y_c))
       
      
    def __find_front_wall_points(self, x_points, y_points, x_centre, y_centre):
        # Convert input lists to NumPy arrays if they are not already
        x_points = np.array(x_points)
        y_points = np.array(y_points)

        # Create a boolean mask for the conditions
        mask = (x_points > (x_centre + 10)) &\
          (y_points > (y_centre - self._data_points_distance_threshold)) &\
          (y_points < (y_centre + self._data_points_distance_threshold))

        # Use the mask to filter the points
        wall_points = np.column_stack((x_points[mask], y_points[mask]))

        return wall_points
      

    def __find_left_wall_points(self, x_points, y_points, x_centre, y_centre):
        # Convert input lists to NumPy arrays if they are not already
        x_points = np.array(x_points)
        y_points = np.array(y_points)

        # Create a boolean mask for the conditions
        # mask = (y_points > (y_centre + 8)) & (y_points < (y_centre + self._data_point_depth_tolerance_threshold)) &\
        #       (x_points > (x_centre - self._data_points_distance_threshold)) &\
        #       (x_points < (x_centre + self._data_points_distance_threshold))
        mask = (y_points > (y_centre + 8)) & (y_points < (y_centre + self._data_point_depth_tolerance_threshold)) &\
              (x_points > (x_centre)) &\
              (x_points < (x_centre + self._data_points_distance_threshold))

        # Use the mask to filter the points
        wall_points = np.column_stack((x_points[mask], y_points[mask]))

        return wall_points


    def __find_right_wall_points(self, x_points, y_points, x_centre, y_centre):
        # Convert input lists to NumPy arrays if they are not already
        x_points = np.array(x_points)
        y_points = np.array(y_points)

        # Create a boolean mask for the conditions
        valid_y_min = y_centre - self._data_point_depth_tolerance_threshold
        valid_y_max = y_centre - 8
        mask = (y_points > valid_y_min) & (y_points < valid_y_max) & \
              (x_points > (x_centre - self._data_points_distance_threshold)) & \
              (x_points < (x_centre + self._data_points_distance_threshold))

        mask = (y_points > valid_y_min) & (y_points < valid_y_max) & \
              (x_points > (x_centre)) & \
              (x_points < (x_centre + self._data_points_distance_threshold))

        # Use the mask to filter the points
        wall_points = np.column_stack((x_points[mask], y_points[mask]))

        return wall_points
    
    
    def _decide_what_to_do(self):
        """Decide the robot's movement based on obstacle detection."""
        
        # Check for maze solved condition first
        if (
            self.average_front_wall_distance is not None and self.average_front_wall_distance > 70 and
            self.average_left_wall_distance is not None and self.average_left_wall_distance > 70 and
            self.average_right_wall_distance is not None and self.average_right_wall_distance > 70
        ):
            self._running = False
            self.logger.info('🥳🎉 Maze is Solved !!!')
            return # Exit if maze is solved

        # If a movement command was just sent and it's not yet completed or timed out,
        # _moveForward will return False. In that case, we don't want to try
        # another action (like turning) yet. We keep waiting for the current command.
        # This is crucial for the non-blocking timeout to work.
        if self._target_dist is not None and \
           abs(self._robot.recent_robot_sensor_values[1] - self._target_dist) >= self.DISTANCE_TOLERANCE_CM and \
           (time.time() - self._last_command_time) <= self._movement_timeout_sec:
            
            self.logger.debug("Still waiting for current movement command to complete or timeout.")
            # We explicitly call _moveForward here to let it re-evaluate its state
            # (e.g., if it needs to resend due to timeout)
            self._moveForward() 
            return # Don't decide a new action, just continue waiting for the current one


        # Now, check for wall alignment before moving if there's a right wall
        if self.isRightObstacle and self.average_right_wall_distance is not None:
             # Add a tolerance to avoid constant micro-adjustments
            if abs(self.average_right_wall_distance - self._clearance_from_the_right_wall) > self.DISTANCE_TOLERANCE_CM:
                self.logger.info('📐 Aligning with Right Wall...')
                # self._align_with_right_wall()
                # After aligning, we might want to immediately check movement conditions again
                # without waiting for the next main loop cycle. Or, let the next loop handle it.
                # For simplicity, we'll let the next loop iteration re-evaluate.
                return 

        # If we reach here, either no command was set, the previous one completed/timed out,
        # AND we're not currently aligning with the right wall.
        # Now, proceed with deciding the next action.
        if not self.isFrontObstacle:
            self.logger.info('🔼 Moving Forward')
            self._moveForward()
        elif not self.isLeftObstacle:
            self.logger.info('◀️ Turning Left and Moving Forward')
            self._turnLeft()
            self._moveForward() 
        elif not self.isRightObstacle:
            # Note: This branch might be less frequently taken if you prioritize aligning first
            self.logger.info('▶️ Turning Right and Moving Forward')
            self._turnRight()
            self._moveForward() 
        else:
            self.logger.info('🔄 Turning Around and Moving Forward')
            self._turnAround()
            self._moveForward() 

    
    def _moveForward(self):
        # Calculate the relative distance to move
        if self.average_front_wall_distance is None:
            delta_x_rel = self._forward_step_in_cm
        else:
            delta_x_rel = self.average_front_wall_distance - self._clearance_from_the_front_wall

        current_robot_position = self._robot.recent_robot_sensor_values[1]

        # Check if a new command should be sent:
        # 1. No target is currently set (initial move)
        # 2. Robot has reached the previous target within tolerance
        # 3. A timeout has occurred (robot hasn't reached target within _movement_timeout_sec)
        time_since_last_command = time.time() - self._last_command_time

        time.sleep(0.1)

        if self._target_dist is None or \
           abs(current_robot_position - self._target_dist) < self.DISTANCE_TOLERANCE_CM or \
           time_since_last_command > self._movement_timeout_sec:

            if time_since_last_command > self._movement_timeout_sec and self._target_dist is not None:
                self.logger.warning(f"⚠️ Movement to {self._target_dist:.2f} cm timed out after {self._movement_timeout_sec}s. Resending command.")
            elif self._target_dist is not None:
                self.logger.info(f"✅ Reached target {self._target_dist:.2f} cm (current: {current_robot_position:.2f} cm). Sending new command.")

            # Calculate the new absolute target position
            new_target_position = current_robot_position + delta_x_rel
            
            # Formulate the command to send to the robot
            command = f"MOVE,{new_target_position},{self._movement_speed}"
            
            # Update the stored target distance and the timestamp of the last command
            self._target_dist = new_target_position
            self._last_command_time = time.time()
            
            # Send the command to the robot interface
            self._robot.send_command_to_robot(command)
            self.logger.info(f"🚀 Sent MOVE command: {command}. New target: {self._target_dist:.2f} cm")
            return True # Command sent
        else:
            self.logger.info(f"⏳ Waiting for robot to reach target {self._target_dist:.2f} cm (current: {current_robot_position:.2f} cm)")
            return False # Waiting for previous command to be reached
      
      
    def _turnLeft(self):
        current_angle = self._robot.recent_robot_sensor_values[0]
        target_angle = current_angle + 90
        self._robot.rotate_to_degrees(target_angle, self._rotation_speed)
        time.sleep(5)

    def _turnRight(self):
        current_angle = self._robot.recent_robot_sensor_values[0]
        target_angle = current_angle - 90
        self._robot.rotate_to_degrees(target_angle, self._rotation_speed)
        time.sleep(5)

    def _turnAround(self):
        current_angle = self._robot.recent_robot_sensor_values[0]
        target_angle = current_angle + 180
        self._robot.rotate_to_degrees(target_angle, self._rotation_speed)
        time.sleep(10)
        
        
    def _convert_points_to_robot_frame(self, x_vals_global, y_vals_global, robot_angle_degrees, robot_x_global, robot_y_global):
        """
        Converts global ToF points (x_vals_global, y_vals_global) into the robot's
        local coordinate frame, given the robot's current global angle and position.

        The robot's local frame has its origin at the robot's center,
        its positive x-axis pointing forward, and its positive y-axis pointing left.
        """
        x_vals_global = np.array(x_vals_global)
        y_vals_global = np.array(y_vals_global)

        # Translate points so robot's current global position is the origin
        x_translated = x_vals_global - robot_x_global
        y_translated = y_vals_global - robot_y_global

        # Rotate points by the negative of the robot's angle
        # to bring them into the robot's own reference frame.
        theta = np.deg2rad(robot_angle_degrees)
        
        # Standard rotation matrix for rotating points by -theta
        # [ cos(-theta) -sin(-theta) ] = [ cos(theta)  sin(theta) ]
        # [ sin(-theta)  cos(-theta) ] = [-sin(theta)  cos(theta) ]
        x_rotated = x_translated * np.cos(theta) + y_translated * np.sin(theta)
        y_rotated = -x_translated * np.sin(theta) + y_translated * np.cos(theta)

        return np.column_stack((x_rotated, y_rotated))
