""" Class to encapsulate a pybullet model of the CMU ballbot """
import pybullet as p
import numpy as np

from robot.definitions import *
from robot.state import State
from utils import *
from transformation import *
from sensors.lidar import Lidar

MAX_IMBD_TORQUE_NM = 100


class Ballbot:
    def __init__(self, urdf_path='', startPos=[0, 0, 0.12], startOrientationEuler=[0, 0, 0]):
        self._urdf_path = urdf_path
        self.reset(startPos, startOrientationEuler)

        self._arm_mode = p.POSITION_CONTROL
        #self._arm_mode = p.TORQUE_CONTROL

        self.state = State()
        self.update_robot_state()

        # Add Sensors
        self.lidar = Lidar(self.robot, self.linkIds[BODY_LASER_LINK_NAME])
        
        # Add force torque sensors 
        self.enable_force_torque_sensors()

        # State of the robot in BODY Frame
        self.xAngleBody = 0.0
        self.yAngleBody = 0.0
        self.yawBody = 0.0

    def reset(self, startPos, startOrientationEuler):
        # Convert from Ballbot Body Orient notation to Pybullet notation
        startOrientationEulerStandardFrame = convertEulerBBToStandardFrame(
            startOrientationEuler)
        startOrientation = p.getQuaternionFromEuler(
            startOrientationEulerStandardFrame)
        self.robot = p.loadURDF(self._urdf_path, startPos,
                                startOrientation, useFixedBase=False)
        self.nJoints = p.getNumJoints(self.robot)

        self.arm_joint_names = []
        self.jointIds = []
        self.ftJointIds  = []

        # TODO: Ask Cornelia why changing the damping is necessary
        p.changeDynamics(self.robot, -1, linearDamping=0, angularDamping=0)

        # Extract arm joint information
        self.linkIds = {}
        for j in range(p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, j, linearDamping=0.5,
                             angularDamping=0.5)
            info = p.getJointInfo(self.robot, j)
            jointName = info[1].decode('UTF-8')
            jointType = info[2]

            # Store link Ids
            linkName = info[12].decode('UTF-8')
            self.linkIds[linkName] = j

            # Store arm joint ids
            if jointName in ARMS_JOINT_NAMES:
                self.jointIds.append(j)
                self.arm_joint_names.append(jointName)
            
            if jointName in FT_SENSOR_JOINT_NAMES:
                self.ftJointIds.append(j)

        self.nArmJoints = len(self.arm_joint_names)

        # TODO make sure this is correct: friction between ball and ground plane
        p.changeDynamics(self.robot, 0, linearDamping=0.5, angularDamping=0.5)

    def set_initial_config(self, joint_positions):
        for jointIndex in range(self.nJoints):
            p.resetJointState(self.robot, jointIndex,
                              joint_positions[jointIndex])

    def set_arms_intial_config(self, joint_positions):
        for i in range(self.nArmJoints):
            p.resetJointState(self.robot, self.jointIds[i], joint_positions[i])

    def enable_force_torque_sensors(self):
        [p.enableJointForceTorqueSensor(self.robot,self.ftJointIds[i])for i in range(len(self.ftJointIds))]

    def print_model_info(self):
        print("num bodies: ", p.getNumBodies())
        print("info1: ", p.getBodyInfo(self.robot))
        for i in range(p.getNumBodies()):
            print("info: ", p.getBodyInfo(i))
        # for i in range(p.getNumBodies()):
        #    info = p.getBodyInfo(self.robot, i)
        #    print(info)

    def print_joint_info(self):
        for i in range(self.nJoints):
            info = p.getJointInfo(self.robot, i)
            print("Joint ID: ", info[0], ", Name: ", info[1])
            print(" Damping: ", info[6])
            print(" Friction: ", info[7])
            print(" Link: ", info[12])

    def draw_inertia_box(self):
        # body
        drawInertiaBox(p, self.robot, -1, [1, 0, 0])

        # link arms
        for i in range(self.nJoints):
            drawInertiaBox(p, self.robot, i, [0, 1, 0])

    def rotateWorldToBodyFrame(self, pWorld):
        """
            Rotates the X and Y coordinate of a vector in the World frame to the Body frame
        """
        worldToBodyRotation = p.getQuaternionFromEuler(
            [0, 0, -self.bodyOrientEuler[2]])
        pBody = p.rotateVector(worldToBodyRotation,
                               np.array(pWorld).reshape(3, 1))
        return pBody

    def rotateBodyToWorldFrame(self, pBody):
        """
            Rotates the X and Y coordinate of a vector in the Body frame to the World frame
        """
        bodyToWorldRotation = p.getQuaternionFromEuler(
            [0, 0, self.bodyOrientEuler[2]])
        pWorld = p.rotateVector(bodyToWorldRotation,
                                np.array(pBody).reshape(3, 1))
        return pWorld

    def transformWorldToBodyFrame(self, pWorld):
        # translate world to body
        pBody = pWorld[0:1] - self.bodyPositionInWorldFrame[0:1]

        # rotate world to body
        worldToBodyRotation = p.getQuaternionFromEuler(
            [0, 0, -self.bodyOrientEuler[2]])
        pBody = p.rotateVector(worldToBodyRotation,
                               np.array(pBody).reshape(3, 1))
        return pBody

    def transformWorldToDriveFrame(self, pWorld):
        # translate world to body
        worldToDriveTrans = [self.ballPosInWorldFrame[0],
                             self.ballPosInWorldFrame[1], 0]
        # pDrive = pWorld - worldToDriveTrans
        pDrive = pWorld
        # rotate world to body
        worldToDriveRotation = p.getQuaternionFromEuler(
            [0, 0, -self.bodyOrientEuler[2]])
        pDrive = p.rotateVector(worldToDriveRotation,
                                np.array(pDrive).reshape(3, 1))
        return pDrive

    def transformWorldToBallFrame(self, pWorld):
        worldToBallRotation = self.ball_state[1]
        ballToWorldTransform = p.invertTransform(
            [0, 0, 0], worldToBallRotation)
        return p.rotateVector(ballToWorldTransform[1], np.array(pWorld).reshape(3, 1))

    def get_body_orientation(self):
        """
            Return the body orientation
                bodyOrientEuler = [xAngle,yAngle,yaw]
        """
        imu_position, imu_orientation = p.getBasePositionAndOrientation(
            self.robot)
        imu_euler = p.getEulerFromQuaternion(imu_orientation)

        # TODO: ballbot has a weird convention to define xBodyAngle and yBodyAngle need to make them the same.
        #self.bodyOrientEuler = [imu_euler[0], imu_euler[1], imu_euler[2]]
        self.bodyOrientEuler = [-imu_euler[1], imu_euler[0], imu_euler[2]]
        self.bodyPositionInWorldFrame = imu_position

        # Update state variable
        self.state.update_body_state(imu_euler[1], -imu_euler[0], imu_euler[2])
        return self.bodyOrientEuler

    def get_base_velocity(self):
        """
            Returns the angular velocity of the body 
                bodyAngVel = [xAngVel,yAngVel,yawVel]
        """
        self.bodyLinVel, self.bodyAngVel = p.getBaseVelocity(self.robot)
        return self.bodyAngVel

    def get_ball_state(self):
        # TODO: make the ball link id a variable
        self.ball_state = p.getLinkState(self.robot, 0, 1)
        joint_state = p.getJointStateMultiDof(self.robot, 0)
        ball_orientation = self.ball_state[1]

        # Extract ball position
        self.ballPosInWorldFrame = self.ball_state[0]
        self.ballPosInBodyOrient = self.rotateWorldToBodyFrame(
            self.ballPosInWorldFrame)

        # Radial velocity in world frame
        # self.ballRadialVelInWorldFrame = p.rotateVector(ball_orientation,np.array(joint_state[1]).reshape(3,1))
        # The above is equivalent
        self.ballRadialVelInWorldFrame = self.ball_state[7]
        # Roate from World frame to Body frame
        self.ballRadialVelInBodyOrient = self.rotateWorldToBodyFrame(
            self.ballRadialVelInWorldFrame)
        # print("xBallRadialVelInWorld: ", self.ballRadialVelInWorldFrame[0])
        # print("yBallRadialVelInWorld: ", self.ballRadialVelInWorldFrame[1])
        # print("xBallRadialVelInBody: ", self.ballRadialVelInBodyFrame[0])
        # print("yBallRadialVelInBody: ", self.ballRadialVelInBodyFrame[1])

        # Linear velocity in world frame
        self.ballLinVelInWorldFrame = self.ball_state[6]
        self.ballLinVelInBodyOrient = self.rotateWorldToBodyFrame(
            self.ballLinVelInWorldFrame)
        # print("xBallLinVelInWorld: ", self.ballLinVelInWorldFrame[0])
        # print("yBallLinVelInWorld: ", self.ballLinVelInWorldFrame[1])
        # print("xBallLinVelInBody: ", self.ballLinVelInBodyOrient[0])
        # print("yBallLinVelInBody: ", self.ballLinVelInBodyOrient[1])

    def get_com_state(self):
        self.com_pos, self.com_vel = computeCOMposVel(p, self.robot)
        self.comPosInBodyOrient = self.rotateWorldToBodyFrame(self.com_pos)
        self.comPosInDriveFrame = self.transformWorldToDriveFrame(self.com_pos)
        # print("COMWorld: ", self.com_pos)
        # print("COMDrive: ", self.comPosInDriveFrame)

    def get_arms_state(self):
        self.arm_pos = [p.getJointState(self.robot, self.jointIds[i])[
            0] for i in range(self.nArmJoints)]
        self.arm_vel = [p.getJointState(self.robot, self.jointIds[i])[
            1] for i in range(self.nArmJoints)]
        # TODO: need to find a way to publish joint torques to the ros topics

        # if arm in position control, get torque. Otherwise torque is the commanded torque set in drive_arms
        if self._arm_mode is p.POSITION_CONTROL:
            self.arm_torque = [p.getJointState(self.robot, self.jointIds[i])[
                3] for i in range(self.nArmJoints)]

    def get_wrist_wrench_measurement(self):
        wrench_right = p.getJointState(self.robot, self.ftJointIds[0])[2]
        wrench_left = p.getJointState(self.robot, self.ftJointIds[1])[2]
        return wrench_right, wrench_left

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
        print("Arm control mode set to TORQUE_CONTROL")

    def set_arm_position_mode(self):
        self._arm_mode = p.POSITION_CONTROL
        print("Arm control mode set to POSITION_CONTROL")

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

        # print("TorqueInput: ", torque_commands)
        # Rotate torque from BODY frame to WORLD frame
        torque_commands = self.rotateBodyToWorldFrame(torque_commands)
        # print("TorqueWorld: ", torque_commands)
        # Rotate torque from WORLD frame to BALL frame
        torque_commands = self.transformWorldToBallFrame(torque_commands)
        # print("TorqueBall: ", torque_commands)

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
        for i in range(self.nArmJoints):
            # first "unlock joint" for torque control and simulate friction force, (i.e. force = friction)
            # TODO: Where did this model for friction came from? Since a force is being direction has to be set.
            dyn_friction = abs(JOINT_DYNAMIC_FRICTION*self.arm_vel[i])
            static_friction = ARMS_JOINT_FRICTION[i] if abs(
                self.arm_vel[i]) < STICTION_VEL_THRESHOLD else 0.0
            friction_force = static_friction + dyn_friction
            p.setJointMotorControl2(
                self.robot, self.jointIds[i], p.VELOCITY_CONTROL, targetVelocity=0, force=friction_force)

            p.setJointMotorControl2(
                self.robot, self.jointIds[i], p.TORQUE_CONTROL, force=torque_cmd[i])
        if self._arm_mode is p.TORQUE_CONTROL:
            self.arm_torque = torque_cmd
