""" Ballbot body controller """

from definitions import *
from balancing_controller import BalancingController
from outer_loop_controllers import StationKeepingController

class BodyController(object):
    def __init__(self):
        
        self._actual_period = BALLBOT_TIME_PERIOD_MS/1000
        self.xBodyAngle = 0.0
        self.yBodyAngle = 0.0

        """ Controllers """
        self._balancing_control = BalancingController()
        self._station_keeping_control = StationKeepingController()

        # Start status for each of the controllers
        self._balancing_started = False
        self._station_keeping_started = False

    def get_data(self, time_period):
        self._actual_period = time_period

        
