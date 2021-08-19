""" Ballbot body controller """
import math
from .definitions import *
from .balancing_controller import BalancingController, COMBalancingController
from .outer_loop_controllers import StationKeepingController

class BodyController(object):
    def __init__(self):
        
        self._actual_period = BALLBOT_TIME_PERIOD_MS/1000
        self.xBodyAngle = 0.0
        self.yBodyAngle = 0.0

        """ State Variables """
        self.xBodyAngle = 0.0
        self.yBodyAngle = 0.0

        self.xCOM = 0.0
        self.yCOM = 0.0

        self.xBallPosition = 0.0
        self.yBallPosition = 0.0

        self.xBallVelocity = 0.0
        self.yBallVelocity = 0.0

        self.yawAng = 0.0
        self.yaAngVel = 0.0

        """ Control Variables """
        self.xDesiredCOM = 0.0
        self.yDesiredCOM = 0.0

        self.xDesiredBallPosition = 0.0
        self.yDesiredBallPosition = 0.0

        self.xDesiredBallVelocity = 0.0
        self.yDesiredBallVelocity = 0.0

        # World position and velocity
        self.xDesiredWorldPosition = 0.0
        self.yDesiredWorldPosition = 0.0

        self.xDesiredWorldVelocity = 0.0
        self.yDesiredWorldVelocity = 0.0

        self.xZeroCOM = 0.0
        self.xZeroShapeCOM = 0.0
        self.xZeroEstCOM = 0.0
        self.xZeroArmCOM = 0.0
        self.xStationKeepCOM = 0.0
        
        self.yZeroCOM = 0.0
        self.yZeroShapeCOM = 0.0
        self.yZeroEstCOM = 0.0
        self.yZeroArmCOM = 0.0
        self.yStationKeepCOM = 0.0

        self.xBallCurrent = 0.0
        self.yBallCurrent = 0.0

        self.xPosErr_prev = 0.0
        self.yPosErr_prev = 0.0

        """ Control Setpoints """
        self.xPlannedBodyAngle = 0.0
        self.yPlannedBodyAngle = 0.0

        """ Controllers """
        self._balancing_control = BalancingController()
        self._com_balancing_control = COMBalancingController()
        self._station_keeping_control = StationKeepingController()

        self._balancing_control.set_gains(75,1,3)
        self._balancing_control.set_max_current(100)
        self._com_balancing_control.set_max_torque(100)

        self._station_keeping_control.set_gains(0.1,0,-0.01, 0.01,0, -0.001)
        self._station_keeping_control.set_max_com_displacement(0.1)

        # Start status for each of the controllers
        self._balancing_started = False
        self._station_keeping_started = False

    def set_data(self, time_period, body_orient_euler,ball_velocity):
        self._actual_period = time_period

        # get IMU data
        self.xBodyAngle = body_orient_euler[0]
        self.yBodyAngle = body_orient_euler[1]
        self.yawAng = body_orient_euler[2]
        print("xBodyAngle: ", self.xBodyAngle)
        print("yBodyAngle: ", self.yBodyAngle)
        
        #self.xBallVelocity = ball_velocity[0]
        #self.yBallVelocity = ball_velocity[1]
    
        #  COM position
        #self.xCOM = BALLBOT_COM_M * math.sin(self.xBodyAngle)
        #self.yCOM = BALLBOT_COM_M * math.sin(self.yBodyAngle)

        # Ball position and velocity
    def update_body_orient(self,body_orient_euler):
        # get IMU data
        self.xBodyAngle = body_orient_euler[0]
        self.yBodyAngle = body_orient_euler[1]
        self.yawAng = body_orient_euler[2]

    def update_com_position(self,xCOM,yCOM):
        self.xCOM = xCOM
        self.yCOM = yCOM

    def update_ball_position(self,xBallPos, yBallPos):
        self.xBallPosition = xBallPos
        self.yBallPosition = yBallPos

    def update_ball_velocity(self,xBallVel,yBallVel):
        self.xBallVelocity = xBallVel
        self.yBallVelocity = yBallVel

    def set_planned_body_angles(self,plannedXAngle, plannedYAngle):
        self.xPlannedBodyAngle = plannedXAngle
        self.yPlannedBodyAngle = plannedYAngle

    def set_desired_body_angles(self, desiredXAngle, desiredYAngle):
        """
            set desired body angles [rad]

            @param[in] desiredXAngle body angle in body frame [rad]
            @param[in] desiredYAngle body angle in body frame [rad]
        """
        self.xDesiredBodyAngle = desiredXAngle
        self.yDesiredBodyAngle = desiredYAngle
        
        # convert to desired com position
        yDesiredBodyCOM = BALLBOT_COM_M * math.sin(self.xDesiredBodyAngle)
        xDesiredBodyCOM = BALLBOT_COM_M * math.sin(self.yDesiredBodyAngle)
        # TODO: eventually want to set COM pos in body frame
        xDesiredCOM = self.xBallPosition + xDesiredBodyCOM
        yDesiredCOM = self.yBallPosition + yDesiredBodyCOM
        self.set_desired_com_position(xDesiredCOM,yDesiredCOM)

    def set_desired_angles_odom_frame(self, desiredXAngle, desiredYAngle):
        # TODO: Implement rotation onces yaw DOF is added
        #heading = self.yawAng
        #bodyXAngleDesired = math.cos(heading) * desiredXAngle + math.sin(heading) * desiredYangle
        #bodyYAngleDesired = -math.sin(heading) * desiredXAngle + math.cos(heading) * desiredYangle
        bodyXAngleDesired = desiredXAngle
        bodyYAngleDesired = desiredYAngle
        self.set_desired_body_angles(bodyXAngleDesired, bodyYAngleDesired)

    def set_desired_velocity_odom_frame(self, desiredXVel, desiredYVel):
        xDesiredWorldVelocity = desiredXVel
        yDesiredWorldVelocity = desiredYVel
        
        #heading = self.yawAng
        #xDesiredBallVelocity = math.cos(heading) * desiredXVel + math.sin(heading) * desiredYVel
        #yDesiredBallVelocity = -math.sin(heading) * desiredXVel + math.cos(heading) * desiredYVel
        self.xDesiredBallVelocity = xDesiredWorldVelocity
        self.yDesiredBallVelocity = yDesiredWorldVelocity
        
    def set_desired_com_position(self, desiredXPos, desiredYPos):
        """
            set the desired com position [m]

            @param[in] desiredXPos ball position in world frame but body oriented
            @param[in] desiredYPos ball position in world frame but body oriented 

            TODO: change so imput is in body frame, not int world frame
        """
        self.xDesiredCOM = desiredXPos
        self.yDesiredCOM = desiredYPos
 
    def set_desired_ball_position(self, desiredXPos, desiredYPos):
        """
            set the desired ball position [m]

            @param[in] desiredXPos ball position in world frame but body oriented
            @param[in] desiredYPos ball position in world frame but body oriented
        """
        self.xDesiredBallPosition = desiredXPos
        self.yDesiredBallPosition = desiredYPos
    
    def set_desired_world_position(self, desiredXPos, desiredYPos):
        self.xDesiredWorldPosition = desiredXPos
        self.yDesiredWorldPosition = desiredYPos

    def set_desired_ball_velocity(self, desiredXVel, desiredYVel):
        self.xDesiredBallVelocity = desiredXVel
        self.yDesiredBallVelocity = desiredYVel
    
    def set_desired_world_velocity(self, desiredXVel, desiredYVel):
        self.xDesiredWorldVelocity  = desiredXVel
        self.yDesiredWorldVelocity = desiredYVel

    def set_zero_shape_com(self, zeroXPos, zeroYPos):
        self.xZeroShapeCOM = zeroXPos
        self.yZeroShapeCOM = zeroYPos

    def balance(self, time_period_s):
        if not self._balancing_started:
            self._balancing_started = True
        
        xTotalZeroCOM = self.xZeroCOM + self.xZeroShapeCOM + self.xZeroEstCOM + self.xZeroArmCOM + self.xStationKeepCOM
        yTotalZeroCOM = self.yZeroCOM + self.yZeroShapeCOM + self.yZeroEstCOM + self.yZeroArmCOM + self.yStationKeepCOM

        # x = real - desired - offset
        xPosErr = self.xCOM - self.xDesiredCOM - xTotalZeroCOM
        yPosErr = self.yCOM - self.yDesiredCOM - yTotalZeroCOM

        xVelErr = xPosErr - self.xPosErr_prev/time_period_s
        yVelErr = yPosErr - self.yPosErr_prev/time_period_s

        self.xPosErr_prev = xPosErr
        self.yPosErr_prev = yPosErr

        #xVelErr = self.xDesiredBallVelocity - self.xBallVelocity
        #yVelErr = self.yDesiredBallVelocity - self.yBallVelocity

        self._balancing_control.set_error_value(xPosErr, yPosErr, xVelErr, yVelErr)
        self._balancing_control.get_current_output()
        
        #print("xCOM: ", self.xCOM, "| xDesiredCOM: ", self.xDesiredCOM)
        #print("yCOM: ", self.yCOM, "| yDesiredCOM: ", self.yDesiredCOM)
        self._com_balancing_control.calculate_error_value(self.xCOM,self.xDesiredCOM, self.yCOM, self.yDesiredCOM,time_period_s)
        self._com_balancing_control.get_torque_output()

        self.xBallCurrent = self._balancing_control._x_current_amps
        self.yBallCurrent = self._balancing_control._y_current_amps
        self.xBallTorque = self._com_balancing_control.torque_x_nm
        self.yBallTorque = self._com_balancing_control.torque_y_nm

    def station_keep(self):
        if not self._station_keeping_started:
            self._station_keeping_started = True

        # Position error
        #TODO: add if statement to specificy in which frame, currently only in Body Frame
        xPosErr = self.xDesiredBallPosition - self.xBallPosition
        yPosErr = self.yDesiredBallPosition - self.yBallPosition
        
        # Velocity error
        xVelErr = 0.0 - self.xBallVelocity
        yVelErr = 0.0 - self.yBallVelocity

        self._station_keeping_control.set_error_value(xPosErr,yPosErr,xVelErr,yVelErr)
        self._station_keeping_control.get_com_output()

        #print("BallPosition: ", self.xBallPosition, ",", self.yBallPosition)
        #print("PosErr: ", xPosErr, "," , yPosErr)
        #print("StationKeep X: ", self._station_keeping_control._desired_x_body_angle)
        #print("StationKeep Y: ", self._station_keeping_control._desired_y_body_angle)

        # TODO: set desired COM position
        xComPosStationKeep = self.xBallPosition + self._station_keeping_control._desired_x_com
        yComPosStationKeep = self.yBallPosition + self._station_keeping_control._desired_y_com
        self.set_desired_com_position(xComPosStationKeep,yComPosStationKeep)
        #self.set_desired_body_angles(self._station_keeping_control._desired_x_body_angle,
        #    self._station_keeping_control._desired_y_body_angle)
        

    def clear_balancing_error_values(self):
        self._balancing_control.clear_all_error_values()
        self._balancing_started = False

    def clear_station_keeping_error_values(self):
        self._station_keeping_control.clear_all_error_values()
        self._station_keeping_started = False
