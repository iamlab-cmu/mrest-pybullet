""" Class to encapsulate a pybullet model of the CMU ballbot """

import pybullet as p
import numpy as np

from utils import drawInertiaBox, computeCOMposVel
from transformation import *

MAX_IMBD_TORQUE_NM = 100

class Ballbot:
    def __init__(self, urdf_path='',startPos=[0,0,0.12],startOrientationEuler=[0,0,0]):
        self._urdf_path = urdf_path
        self.reset(startPos,startOrientationEuler)

        self.update_robot_state()

        self._arm_mode = p.TORQUE_CONTROL

        # State of the robot in BODY Frame 
        self.xAngleBody = 0.0
        self.yAngleBody = 0.0
        self.yawBody = 0.0

    
    def reset(self,startPos,startOrientationEuler):
        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
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
        # TODO make sure this is correct: friction between ball and ground plane
        p.changeDynamics(self.robot, 0, linearDamping=0.5, angularDamping=0.5)

    def print_model_info(self):
        print("num bodies: ", p.getNumBodies())
        for i in range(p.getNumBodies()):
            print("info: ", p.getBodyInfo(i))
        #for i in range(p.getNumBodies()):
        #    info = p.getBodyInfo(self.robot, i)
        #    print(info)

    def draw_inertia_box(self):
        # body
        drawInertiaBox(p,self.robot,-1,[1,0,0])
        
        # link arms
        for i in range(self.nJoints):
            drawInertiaBox(p,self.robot,i,[0,1,0])
    
    def rotateWorldToBodyFrame(self, pWorld):
        """
            Rotates the X and Y coordinate of a vector in the World frame to the Body frame
        """
        worldToBodyRotation = p.getQuaternionFromEuler([0,0,-self.bodyOrientEuler[2]])
        pBody = p.rotateVector(worldToBodyRotation, np.array(pWorld).reshape(3,1))
        return pBody

    def rotateBodyToWorldFrame(self,pBody):
        """
            Rotates the X and Y coordinate of a vector in the Body frame to the World frame
        """
        bodyToWorldRotation = p.getQuaternionFromEuler([0,0,self.bodyOrientEuler[2]])
        pWorld = p.rotateVector(bodyToWorldRotation, np.array(pBody).reshape(3,1))
        return pWorld
    
    def transformWorldToBodyFrame(self, pWorld):
        # translate world to body
        pBody = pWorld[0:1] - self.bodyPositionInWorldFrame[0:1]

        # rotate world to body
        worldToBodyRotation = p.getQuaternionFromEuler([0,0,-self.bodyOrientEuler[2]])
        pBody = p.rotateVector(worldToBodyRotation, np.array(pBody).reshape(3,1))
        return pBody

    def transformWorldToDriveFrame(self,pWorld):
        # translate world to body
        worldToDriveTrans = [self.ballPosInWorldFrame[0],self.ballPosInWorldFrame[1], 0]
        #pDrive = pWorld - worldToDriveTrans
        pDrive = pWorld
        # rotate world to body
        worldToDriveRotation = p.getQuaternionFromEuler([0,0,-self.bodyOrientEuler[2]])
        pDrive = p.rotateVector(worldToDriveRotation, np.array(pDrive).reshape(3,1))
        return pDrive

    def transformWorldToBallFrame(self,pWorld):
        worldToBallRotation = self.ball_state[1]
        ballToWorldTransform = p.invertTransform([0,0,0], worldToBallRotation)
        return p.rotateVector(ballToWorldTransform[1], np.array(pWorld).reshape(3,1))
        
    def get_body_orientation(self):
        """ 
            Return the body orientation 
                bodyOrientEuler = [xAngle,yAngle,yaw]
        """
        imu_position, imu_orientation = p.getBasePositionAndOrientation(self.robot)
        imu_euler = p.getEulerFromQuaternion(imu_orientation)

        # TODO: ballbot has a weird convention to define xBodyAngle and yBodyAngle need to make them the same.
        self.bodyOrientEuler = [imu_euler[0],imu_euler[1],imu_euler[2]]
        self.bodyPositionInWorldFrame = imu_position
        return self.bodyOrientEuler

    def get_base_velocity(self):
        linear, angular = p.getBaseVelocity(self.robot)

    def get_ball_state(self):
        # TODO: make the ball link id a variable
        self.ball_state = p.getLinkState(self.robot,0,1)
        joint_state = p.getJointStateMultiDof(self.robot,0)
        ball_orientation = self.ball_state[1]

        # Extract ball position
        self.ballPosInWorldFrame = self.ball_state[0]
        self.ballPosInBodyOrient = self.rotateWorldToBodyFrame(self.ballPosInWorldFrame)

        # Radial velocity in world frame
        # self.ballRadialVelInWorldFrame = p.rotateVector(ball_orientation,np.array(joint_state[1]).reshape(3,1))
        # The above is equivalent     
        self.ballRadialVelInWorldFrame = self.ball_state[7]
        # Roate from World frame to Body frame
        self.ballRadialVelInBodyOrient = self.rotateWorldToBodyFrame(self.ballRadialVelInWorldFrame)
        #print("xBallRadialVelInWorld: ", self.ballRadialVelInWorldFrame[0])
        #print("yBallRadialVelInWorld: ", self.ballRadialVelInWorldFrame[1])
        #print("xBallRadialVelInBody: ", self.ballRadialVelInBodyFrame[0])
        #print("yBallRadialVelInBody: ", self.ballRadialVelInBodyFrame[1])

        # Linear velocity in world frame
        self.ballLinVelInWorldFrame = self.ball_state[6]
        self.ballLinVelInBodyOrient = self.rotateWorldToBodyFrame(self.ballLinVelInWorldFrame)
        #print("xBallLinVelInWorld: ", self.ballLinVelInWorldFrame[0])
        #print("yBallLinVelInWorld: ", self.ballLinVelInWorldFrame[1])
        #print("xBallLinVelInBody: ", self.ballLinVelInBodyOrient[0])
        #print("yBallLinVelInBody: ", self.ballLinVelInBodyOrient[1])

    def get_com_state(self):
        self.com_pos, self.com_vel = computeCOMposVel(p,self.robot)
        self.comPosInBodyOrient = self.rotateWorldToBodyFrame(self.com_pos)
        self.comPosInDriveFrame = self.transformWorldToDriveFrame(self.com_pos)
        #print("COMWorld: ", self.com_pos)
        #print("COMDrive: ", self.comPosInDriveFrame)

    def get_arms_state(self):
        self.arm_pos = [p.getJointState(self.robot, self.jointIds[i])[0] for i in range(self.nArmJoints)] 
        self.arm_vel = [p.getJointState(self.robot, self.jointIds[i])[1] for i in range(self.nArmJoints)] 

    def update_robot_state(self):
        self.get_body_orientation()
        self.get_base_velocity()
        self.get_ball_state()
        self.get_com_state()
        self.get_arms_state()

    def get_model_id(self):
        return self.robot

    def set_arm_torque_mode(self):
        self._arm_mode = p.TORQUE_CONTROL
    
    def set_arm_position_mode(self):
        self._arm_mode = p.POSITION_CONTROL

    def drive_imbd(self, torque_x, torque_y):
        """
            Applies the torques given in body frame to drive the ball

            @parameter[in] torque_x     torque (Nm) to be applied around the x-axis in the body frame
            @parameter[in] torque_y     torque (Nm) to be applied around the y-axis in the body frame
        """
        # Saturate IMBD torques
        torque_x = np.clip(torque_x, -MAX_IMBD_TORQUE_NM, MAX_IMBD_TORQUE_NM)
        torque_y = np.clip(torque_y, -MAX_IMBD_TORQUE_NM, MAX_IMBD_TORQUE_NM)
        
        torque_commands = [torque_x, torque_y, 0.0]
        
        #print("TorqueInput: ", torque_commands)
        # Rotate torque from BODY frame to WORLD frame
        torque_commands = self.rotateBodyToWorldFrame(torque_commands)
        #print("TorqueWorld: ", torque_commands)
        # Rotate torque from WORLD frame to BALL frame  
        torque_commands = self.transformWorldToBallFrame(torque_commands)
        #print("TorqueBall: ", torque_commands)

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

    
    def drive_arms(self, position_cmd, torque_cmd):
        if self._arm_mode == p.POSITION_CONTROL:
            for i in range(self.nArmJoints):
                p.setJointMotorControl2(self.robot, self.jointIds[i], 
                    p.POSITION_CONTROL,position_cmd[i], force = 5 * 240.)

        if self._arm_mode == p.TORQUE_CONTROL:
            for i in range(self.nArmJoints):
                # first "unlock joint" for torque control, force = friction
                p.setJointMotorControl2(self.robot, self.jointIds[i], p.VELOCITY_CONTROL, force=1)
                p.setJointMotorControl2(self.robot, self.jointIds[i], p.TORQUE_CONTROL, force = torque_cmd[i])


