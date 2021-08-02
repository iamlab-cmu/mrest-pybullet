""" PID Balancing Controller """

from controllers import PIDController

class BalancingController(object):
    """ Initialize balancing controller class """
    def __init__(self):
        self._x_current_amps = 0.0
        self._y_current_amps = 0.0

        self._balance_controller_XZ = PIDController()
        self._balance_controller_YZ = PIDController()

    def set_gains(self, kP, kI, kD):
        self._balance_controller_XZ.set_pid_gains(kP,kI,kD)
        self._balance_controller_YZ.set_pid_gains(kP,kI,kD)
    
    def set_error_value(self, 
        x_linear_pos_error, 
        y_linear_pos_error,
        x_linear_vel_error, 
        y_linear_vel_error):
        self._balance_controller_XZ.set_error_values(x_linear_pos_error,x_linear_vel_error)
        self._balance_controller_YZ.set_error_values(y_linear_pos_error,y_linear_vel_error)

    def set_max_current(self, max_current):
        self._balance_controller_XZ.set_max_output(max_current)
        self._balance_controller_YZ.set_max_output(max_current)
    
    def get_current_output(self):
        self._x_current_amps = self._balance_controller_XZ.get_pid_output()
        self._y_current_amps = self._balance_controller_YZ.get_pid_output()
    
    def clear_all_error_values(self):
        self._balance_controller_XZ.clear_error_values()
        self._balance_controller_YZ.clear_error_values()