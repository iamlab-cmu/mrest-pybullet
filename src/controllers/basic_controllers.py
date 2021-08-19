""" A PID control class """

import math
import numpy as np

class PIDController(object):
    """" Generates a generic PID controller """
    
    def __init__(self):
        """ Initialize the controller class """
        self._kP = 0.0
        self._kI = 0.0
        self._kD = 0.0

        self._error_val = 0.0
        self._integral_error = 0.0
        self._derivative_error = 0.0

        self._max_output = 0.0

        self._antiwindup = False
        self._max_integral_error = 10
    
    def set_pid_gains(self, kP, kI, kD):
        dI = kI - self._kI
        if dI > 0.0001 or dI < -0.0001:
            self._integral_error = 0

        self._kP = kP
        self._kI = kI
        self._kD = kD

    def set_antiwindup(self, enable, max_integral_error=10):
        self._antiwindup = enable
        self._max_integral_error = max_integral_error

    def calculate_error_values(self, value, reference, time_period):
        error = reference - value
        
        self._derivative_error = (error - self._error_val)/time_period
        self._error_val = error
        self._integral_error += self._error_val *time_period

        if self._antiwindup:
            self._integral_error = np.clip(self._integral_error, -self._max_integral_error,self._max_integral_error)
    
    def set_error_values(self, error, derivate_error):
        self._error_val = error
        self._integral_error += self._error_val/100
        self._derivative_error = derivate_error

        if self._antiwindup:
            self._integral_error = np.clip(self._integral_error, -self._max_integral_error,self._max_integral_error)
    
    def clear_error_values(self):
        self._error_val = 0.0
        self._integral_error = 0.0
        self._derivative_error = 0.0
    
    def set_max_output(self, max_output):
        self._max_output = max_output
    
    def get_pid_output(self):
        output = self._kP * self._error_val + self._kI * self._integral_error + self._kD* self._derivative_error
        if math.fabs(output) > self._max_output:
            if output > 0:
                output = self._max_output
            else:
                output = -self._max_output

        return output
    