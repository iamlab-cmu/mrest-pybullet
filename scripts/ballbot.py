""" Class to encapsulate a pybullet model of the CMU ballbot """

import pybullet as p
import numpy as np

from utils import drawInertiaBox, computeCOMposVel

MAX_IMBD_TORQUE_NM = 100

class Ballbot:
    def __init__(self, urdf_path=''):
        self._urdf_path = urdf_path
        self.reset()
        self.update_robot_state()

        self._arm_mode = p.POSITION_CONTROL
    
    def reset(self):
        startPos = [0, 0, 0.12]
        startOrientation = p.getQuaternionFromEuler([0,0,0.0])
        self.robot = p.loadURDF(self._urdf_path, startPos, startOrientation, useFixedBase=False)
        self.nJoints = p.getNumJoints(self.robot)
        
        self.arm_joint_names = []
        self.jointIds = []

        # TODO: Ask Cornelia why changing the damping is necessary
        p.changeDynamics(self.robot, -1, linearDamping=0, angularDamping=0)
        for j in range(p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.robot, j)
            # print(info)
            jointName = info[1]
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                self.jointIds.append(j)
                self.arm_joint_names.append(jointName)
        
        self.nArmJoints = len(self.arm_joint_names)
        p.changeDynamics(self.robot, 0, linearDamping=0.5, angularDamping=0.5)

    def draw_inertia_box(self):
        # body
        drawInertiaBox(p,self.robot,-1,[1,0,0])
        
        # link arms
        for i in range(self.nJoints):
            drawInertiaBox(p,self.robot,i,[0,1,0])
    
    def get_body_orientation(self):
        """ 
            Return the body orientation 
                body_euler = [xAngle,yAngle,yaw]
        """
        imu_position, imu_orientation = p.getBasePositionAndOrientation(self.robot)
        imu_euler = p.getEulerFromQuaternion(imu_orientation)

        body_euler = [-imu_euler[1],imu_euler[0],imu_euler[2]]
        return body_euler

    def get_base_velocity(self):
        linear, angular = p.getBaseVelocity(self.robot)

    def get_ball_state(self):
        self.ball_state = p.getLinkState(self.robot,0)

    def get_com_state(self):
        self.com_pos, self.com_vel = computeCOMposVel(p,self.robot)

    def update_robot_state(self):
        self.get_body_orientation()
        self.get_base_velocity()
        self.get_ball_state()
        self.get_com_state()

    def get_model_id(self):
        return self.robot

    def set_arm_torque_mode(self):
        self._arm_mode = p.TORQUE_CONTROL
    
    def set_arm_position_mode(self):
        self._arm_mode = p.POSITION_CONTROL

    def drive_imbd(self, torque_x, torque_y):
        
        # Saturate IMBD torques
        torque_x = np.clip(torque_x, -MAX_IMBD_TORQUE_NM, MAX_IMBD_TORQUE_NM)
        torque_y = np.clip(torque_y, -MAX_IMBD_TORQUE_NM, MAX_IMBD_TORQUE_NM)
        
        torque_commands = [torque_x, torque_y, 0.0]

        # Rotate torque to be applied relative to a fixed body orientation 
        # frame instead of the ball frame
        ball_orientation = self.ball_state[1]
        ball_orientation_inv = p.invertTransform([0,0,0], ball_orientation)
        torque_commands = p.rotateVector(ball_orientation_inv[1],np.array(torque_commands).reshape(3,1))
        # unlock motors
        p.setJointMotorControlMultiDof(self.robot,
                                   0,
                                   controlMode=p.POSITION_CONTROL,
                                   targetPosition=[0, 0, 0, 1],
                                   force=[0, 0, 0])
        
        # change to torque mode and set desired torques
        p.setJointMotorControlMultiDof(self.robot,
                                   0,
                                   controlMode=p.TORQUE_CONTROL,
                                   force=torque_commands)

    
    def drive_arms(self, target_pos):
        if self._arm_mode == p.POSITION_CONTROL:
            for i in range(self.nArmJoints):
                p.setJointMotorControl2(self.robot, self.jointIds[i], 
                    p.POSITION_CONTROL,target_pos[i], force = 5 * 240.)


