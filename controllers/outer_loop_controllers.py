""" Outer Loop Controllers """

import math

from controllers import PIDController

""" Station Keeping Controller """
class StationKeepingController(object):
    def __init__(self):

        self._desired_x_body_angle = 0.0
        self._desired_y_body_angle = 0.0

        self._position_controller_XZ = PIDController()
        self._position_controller_YZ = PIDController()

    def set_gains(self, kP, kI, kD):
        self._position_controller_XZ.set_pid_gains(kP, kI, kD)
        self._position_controller_YZ.set_pid_gains(kP, kI, kD)
    
    def set_error_value(self, 
        x_linear_pos_error, 
        y_linear_pos_error,
        x_linear_vel_error, 
        y_linear_vel_error):
        self._position_controller_XZ.set_error_values(x_linear_pos_error,x_linear_vel_error)
        self._position_controller_YZ.set_error_values(y_linear_pos_error,y_linear_vel_error)

    def set_max_angle(self, max_angle):
        """ set maxium angle in DEGREES """
        self._position_controller_XZ.set_max_output(max_angle)
        self._position_controller_YZ.set_max_output(max_angle)
    
    def get_angle_output(self):
        self._desired_x_body_angle = math.radians(self._position_controller_XZ.get_pid_output())
        self._desired_y_body_angle = math.radians(self._position_controller_YZ.get_pid_output())
    
    def clear_all_error_values(self):
        self._position_controller_XZ.clear_error_values()
        self._position_controller_YZ.clear_error_values()


""" Velocity Controller """
# TODO Finish implementation
class VelocityController(object):
    def __init__(self):

        self._desired_x_body_angle = 0.0
        self._desired_y_body_angle = 0.0
        
        self._sum_x_vel = 0.0
        self._sum_y_vel = 0.0

        self._average_x_vel = 0.0
        self._average_y_vel = 0.0
        
        self._control_swithch_XZ = False
        self._control_swithch_YZ = False

        self._velocity_controller_XZ = PIDController()
        self._velocity_controller_YZ = PIDController()

