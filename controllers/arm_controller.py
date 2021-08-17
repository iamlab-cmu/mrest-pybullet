from controllers import PIDController
import numpy as np

class ArmController(object):
    def __init__(self):
        self.nJoints = 7
        self._joint_controllers = [PIDController() for j in range(self.nJoints)]
        for j in range(self.nJoints):
            self._joint_controllers[j].set_antiwindup(True,10)
            self._joint_controllers[j].set_max_output(1000)
        self.desiredAngles = np.array([0.0 for j in range(self.nJoints)])
        self.desiredAngleVel = np.array([0.0 for j in range(self.nJoints)])
        self.gravity_torque = np.array([0.0 for j in range(self.nJoints)])
        self.set_gains([[50,0,1],[50,0,1],[10,0,0.1],[10,0,0.0],[1,0,0.0],[5,0,0.0],[1,0,0.0]])

    def set_gains(self, gains):
        for j in range(self.nJoints):
            self._joint_controllers[j].set_pid_gains(gains[j][0],gains[j][1],gains[j][2])

    def set_desired_angles(self, q):
        self.desiredAngles = q

    def set_desired_angle_vel(self, qdot):
        self.desiredAngleVel = qdot

    def set_gravity_torque(self,tau):
        self.gravity_torque = tau

    def update_current_state(self, qCurrent, qdotCorrent):
        self.currentAngles = qCurrent
        self.currentAngleVel = qdotCorrent

    def update(self, time_period_s):
        # PosErr = self.desiredAngles-self.currentAngles
        # VelErr = self.desiredAngleVel-self.currentAngleVel
        for j in range(self.nJoints):
            self._joint_controllers[j].calculate_error_values(self.currentAngles[j], self.desiredAngles[j],time_period_s) 
        self.armTorques = [self._joint_controllers[j].get_pid_output() + self.gravity_torque[j] for j in range(self.nJoints)]
