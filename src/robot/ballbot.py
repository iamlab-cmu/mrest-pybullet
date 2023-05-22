""" Class to encapsulate a pybullet model of the CMU ballbot """
import pybullet as p
import numpy as np

from robot.definitions import *
from robot.state import State
from utils import *
from transformation import *
from sensors.lidar import Lidar

MAX_IMBD_TORQUE_NM = 100
MAX_YAW_TORQUE_NM = 100

class Ballbot:
    def __init__(self, urdf_path='', startPos=[0, 0, 0.12], startOrientationEuler=[0, 0, 0]):
        self._urdf_path = urdf_path
        self.reset(startPos, startOrientationEuler)

        # self._arm_mode = p.POSITION_CONTROL
        self._arm_mode = p.TORQUE_CONTROL

        self._turret_mode = p.POSITION_CONTROL

        self._barrett_hands_mode = p.TORQUE_CONTROL

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

        
        if ENABLE_TURRET_CAMERA:
            viewMatrix = p.computeViewMatrix(
                        cameraEyePosition=[0, 3, 2],
                        cameraTargetPosition=[0, 0, 1],
                        cameraUpVector=[0, 0, 1])

            self.turretCamera_projectionMatrix = p.computeProjectionMatrixFOV(
                            fov=45.0,
                            aspect=1.0,
                            nearVal=0.1,
                            farVal=5.1)
            self.update_turretCamera()
        # Example projection matrix from https://github.com/bulletphysics/bullet3/issues/1616
        # fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
        # projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

        # width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        #                                             width=224,
        #                                             height=224,
        #                                             viewMatrix=viewMatrix,
        #                                             projectionMatrix=self.turretCamera_projectionMatrix)

    def reset(self, startPos, startOrientationEuler):
        # Convert from Ballbot Body Orient notation to Pybullet notation
        startOrientationEulerStandardFrame = convertEulerBBToStandardFrame(
            startOrientationEuler)
        startOrientation = p.getQuaternionFromEuler(
            startOrientationEulerStandardFrame)
        # self.robot = p.loadURDF(self._urdf_path, startPos,
        #                         startOrientation, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.robot = p.loadURDF(self._urdf_path, startPos,
                                startOrientation, useFixedBase=False)
        
        self.nJoints = p.getNumJoints(self.robot)

        self.arm_joint_names = []
        self.turret_joint_names = []
        self.jointIds = []
        self.ftJointIds  = []
        self.turretJointIds  = []
        self.ballJointId = []

        self.hand_type = 'knob' # 'barrett', 'parallel_gripper'
        self.barrett_left_joint_names = []
        self.barrett_left_jointIds = []
        self.barrett_right_joint_names = []
        self.barrett_right_jointIds = []

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

            if jointName in TURRET_JOINT_NAMES:
                self.turretJointIds.append(j)
                self.turret_joint_names.append(jointName)
            
            if jointName in FT_SENSOR_JOINT_NAMES:
                self.ftJointIds.append(j)
            
            if jointName in KNOB_JOINT_NAMES:
                self.hand_type = 'knob'
            
            if jointName in BARRETT_LEFT_JOINT_NAMES:
                self.hand_type = 'barrett'
                self.barrett_left_jointIds.append(j)
                self.barrett_left_joint_names.append(jointName)
            
            if jointName in BARRETT_RIGHT_JOINT_NAMES:
                self.hand_type = 'barrett'
                self.barrett_right_jointIds.append(j)
                self.barrett_right_joint_names.append(jointName)
            
            if linkName == REND_EFFECTOR_NAME:
                self.rightEndEffectorId = self.linkIds[REND_EFFECTOR_NAME]
            if linkName == LEND_EFFECTOR_NAME:
                self.leftEndEffectorId = self.linkIds[LEND_EFFECTOR_NAME]

            if jointName in BALL_JOINT_NAME:
                self.ballJointId.append(j)
                print("ball joint id: ", self.ballJointId)

        self.nArmJoints = len(self.arm_joint_names)
        self.nTurretJoints = len(self.turret_joint_names)

        self.nBarrettLeftJoints = len(self.barrett_left_joint_names)
        self.nBarrettRightJoints = len(self.barrett_right_joint_names)
        
        # TODO make sure this is correct: friction between ball and ground plane
        p.changeDynamics(self.robot, 0, linearDamping=0.5, angularDamping=0.5)

    def set_initial_config(self, joint_positions):
        for jointIndex in range(self.nJoints):
            p.resetJointState(self.robot, jointIndex,
                              joint_positions[jointIndex])

    def set_arms_intial_config(self, joint_positions):
        for i in range(self.nArmJoints):
            p.resetJointState(self.robot, self.jointIds[i], joint_positions[i])
    
    def set_barrett_left_hand_intial_config(self, joint_positions):
        for i in range(self.nBarrettLeftJoints):
            p.resetJointState(self.robot, self.barrett_left_jointIds[i], joint_positions[i])
    
    def set_barrett_right_hand_intial_config(self, joint_positions):
        for i in range(self.nBarrettRightJoints):
            p.resetJointState(self.robot, self.barrett_right_jointIds[i], joint_positions[i])
    
    def set_turret_intial_config(self, joint_positions):
        for i in range(self.nTurretJoints):
            p.resetJointState(self.robot, self.turretJointIds[i], joint_positions[i])

    def enable_force_torque_sensors(self):
        [p.enableJointForceTorqueSensor(self.robot,self.ftJointIds[i])for i in range(len(self.ftJointIds))]

    def print_model_info(self):
        print("num bodies: ", p.getNumBodies())
        print("info1: ", p.getBodyInfo(self.robot))
        for i in range(p.getNumBodies()):
            print("info: ", p.getBodyInfo(i))

    def print_joint_info(self):
        for i in range(self.nJoints):
            info = p.getJointInfo(self.robot, i)
            print("Joint ID: ", info[0], ", Name: ", info[1])
            print(" Damping: ", info[6])
            print(" Friction: ", info[7])
            print(" Link: ", info[12])
    
    def print_link_info(self):
        print(self.linkIds)

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
        # pBody = pWorld[0:1] - self.bodyPositionInWorldFrame[0:1]

        pBody = np.array(pWorld[0:2]) - np.array(self.bodyPositionInWorldFrame[0:2])
        pBody = np.append(pBody, pWorld[2])
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
        # imu_position, imu_orientation = p.getBasePositionAndOrientation(
        #     self.robot)
        imu_state = p.getLinkState(
            self.robot, self.ballJointId[0]+1, 0)
        imu_position = imu_state[0]
        imu_orientation = imu_state[1]
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
        self.ball_state = p.getLinkState(self.robot, self.ballJointId[0], 1)
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
    
    def update_turretCamera(self):
        turret_state = p.getLinkState(self.robot, self.linkIds[TURRET_CAMERA_LINK_NAME], computeForwardKinematics=True)
        com_p = turret_state[0]
        rot_matrix = p.getMatrixFromQuaternion(turret_state[1])
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (1, 0, 0) # z-axis
        init_up_vector = (0, 0, 1) # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)

        viewMatrix = p.computeViewMatrix(
                        cameraEyePosition=com_p,
                        cameraTargetPosition=com_p + 0.1 * camera_vector,
                        cameraUpVector=up_vector)
        
        # camx, camy, camz = com_p
        # viewMatrix = p.computeViewMatrix(
        #                 cameraEyePosition=[camx, camy, camz],
        #                 cameraTargetPosition=[camx-0, camy+5, camz-4],
        #                 cameraUpVector=[0, 0, 1])

        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                                    width=224,
                                                    height=224,
                                                    viewMatrix=viewMatrix,
                                                    projectionMatrix=self.turretCamera_projectionMatrix)
        return rgbImg

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

    def get_turret_state(self):
        self.turret_pos = [p.getJointState(self.robot, self.turretJointIds[i])[
            0] for i in range(self.nTurretJoints)]
        self.turret_vel = [p.getJointState(self.robot, self.turretJointIds[i])[
            1] for i in range(self.nTurretJoints)]
        # TODO: need to find a way to publish joint torques to the ros topics

        # if arm in position control, get torque. Otherwise torque is the commanded torque set in drive_arms
        if self._turret_mode is p.POSITION_CONTROL:
            self.turret_torque = [p.getJointState(self.robot, self.turretJointIds[i])[
                3] for i in range(self.nTurretJoints)]
    
    def get_barrett_hands_state(self):
        self.barrett_left_hand_pos = [p.getJointState(self.robot, self.barrett_left_jointIds[i])[
            0] for i in range(self.nBarrettLeftJoints)]
        self.barrett_left_hand_vel = [p.getJointState(self.robot, self.barrett_left_jointIds[i])[
            1] for i in range(self.nBarrettLeftJoints)]
        
        self.barrett_right_hand_pos = [p.getJointState(self.robot, self.barrett_right_jointIds[i])[
            0] for i in range(self.nBarrettRightJoints)]
        self.barrett_right_hand_vel = [p.getJointState(self.robot, self.barrett_right_jointIds[i])[
            1] for i in range(self.nBarrettRightJoints)]
        
        # if arm in position control, get torque. Otherwise torque is the commanded torque set in drive_arms
        if self._barrett_hands_mode is p.POSITION_CONTROL:
            self.barrett_left_hand_torque = [p.getJointState(self.robot, self.barrett_left_jointIds[i])[
                3] for i in range(self.nBarrettLeftJoints)]
            self.barrett_right_hand_torque = [p.getJointState(self.robot, self.barrett_right_jointIds[i])[
                3] for i in range(self.nBarrettRightJoints)]

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
        self.get_turret_state()
        if self.hand_type == 'barrett':
            self.get_barrett_hands_state()

    def get_model_id(self):
        return self.robot

    def set_arm_torque_mode(self):
        self._arm_mode = p.TORQUE_CONTROL
        print("Arm control mode set to TORQUE_CONTROL")

    def set_arm_position_mode(self):
        self._arm_mode = p.POSITION_CONTROL
        print("Arm control mode set to POSITION_CONTROL")
    
    def set_turret_torque_mode(self):
        self._turret_mode = p.TORQUE_CONTROL
        print("Turret control mode set to TORQUE_CONTROL")

    def set_turret_position_mode(self):
        self._turret_mode = p.POSITION_CONTROL
        print("Turret control mode set to POSITION_CONTROL")
    
    def set_barrett_hands_torque_mode(self):
        self._barrett_hands_mode = p.TORQUE_CONTROL
        print("Barrett hands control mode set to TORQUE_CONTROL")

    def set_barrett_hands_position_mode(self):
        self._barrett_hands_mode = p.POSITION_CONTROL
        print("Barrett hands control mode set to POSITION_CONTROL")
    
    def apply_external_wrench(self, wrench, linkId, position = [0,0,0], frame = p.WORLD_FRAME ):
        """
            Applies an instaneous wrench (force and torque) to a given frame in a link. 
            After each simulation step, the external forces are cleared to zero.
            
            @parameter[in]  wrench      6DOF vector containing force (N) and torque (Nm) 
                                        to apply [Fx,Fy,Fz,Tx,Ty,Tz] aligned with the WORLD_FRAME
            @parameter[in]  frame       the frame to which the wrench is applied to
            @parameter[in]  position    3D position [m] on the link where the force is applied 
            @parameter[in]  frame       Specify the coordinate system of force/position: 
                                        either WORLD_FRAME for Cartesian world coordinates or 
                                        LINK_FRAME for local link coordinates
        """

        forces = wrench[0:3]
        torques = wrench[3:]
        p.applyExternalForce(objectUniqueId=self.robot, linkIndex=linkId, forceObj=forces, posObj = position, flags=frame)
        p.applyExternalTorque(objectUniqueId=self.robot, linkIndex=linkId, torqueObj=torques, flags=frame)


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
                                       self.ballJointId[0],
                                       controlMode=p.POSITION_CONTROL,
                                       targetPosition=[0, 0, 0, 1],
                                       force=[0, 0, 0])

        # change to torque mode and set desired torques
        p.setJointMotorControlMultiDof(self.robot,
                                       self.ballJointId[0],
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
            # Way to add damping to the system
            p.setJointMotorControl2(
                self.robot, self.jointIds[i], p.VELOCITY_CONTROL, targetVelocity=0, force=friction_force)

            p.setJointMotorControl2(
                self.robot, self.jointIds[i], p.TORQUE_CONTROL, force=torque_cmd[i])
        if self._arm_mode is p.TORQUE_CONTROL:
            self.arm_torque = torque_cmd

    def drive_turret(self, position_cmd, torque_cmd):
        for i in range(self.nTurretJoints):
            # first "unlock joint" for torque control and simulate friction force, (i.e. force = friction)
            # TODO: Where did this model for friction came from? Since a force is being direction has to be set.
            dyn_friction = abs(JOINT_DYNAMIC_FRICTION*self.turret_vel[i])
            static_friction = TURRET_JOINT_FRICTION[i] if abs(
                self.turret_vel[i]) < STICTION_VEL_THRESHOLD else 0.0
            friction_force = static_friction + dyn_friction
            p.setJointMotorControl2(
                self.robot, self.turretJointIds[i], p.VELOCITY_CONTROL, targetVelocity=0, force=friction_force)

            p.setJointMotorControl2(
                self.robot, self.turretJointIds[i], p.TORQUE_CONTROL, force=torque_cmd[i])
        if self._turret_mode is p.TORQUE_CONTROL:
            self.turret_torque = torque_cmd

    def drive_barrett_hands(self, position_cmd_left, position_cmd_right, torque_cmd_left, torque_cmd_right):
        for i in range(self.nBarrettLeftJoints):
            # first "unlock joint" for torque control and simulate friction force, (i.e. force = friction)
            # TODO: Where did this model for friction came from? Since a force is being direction has to be set.
            dyn_friction = abs(JOINT_DYNAMIC_FRICTION*self.barrett_left_hand_vel[i])
            static_friction = BARRETT_HAND_JOINT_FRICTION if abs(
                self.barrett_left_hand_vel[i]) < STICTION_VEL_THRESHOLD else 0.0
            friction_force = static_friction + dyn_friction
            # p.setJointMotorControl2(
            #     self.robot, self.barrett_left_jointIds[i], p.VELOCITY_CONTROL, targetVelocity=0, force=friction_force)

            p.setJointMotorControl2(
                self.robot, self.barrett_left_jointIds[i], p.VELOCITY_CONTROL, targetVelocity=np.clip(torque_cmd_left[i],-10.,10.), force=10.)
            
            # p.setJointMotorControl2(
            #     self.robot, self.barrett_left_jointIds[i], p.POSITION_CONTROL, targetPosition=position_cmd_left[i], force=10.)
    
        for i in range(self.nBarrettRightJoints):
            # first "unlock joint" for torque control and simulate friction force, (i.e. force = friction)
            # TODO: Where did this model for friction came from? Since a force is being direction has to be set.
            dyn_friction = abs(JOINT_DYNAMIC_FRICTION*self.barrett_right_hand_vel[i])
            static_friction = BARRETT_HAND_JOINT_FRICTION if abs(
                self.barrett_right_hand_vel[i]) < STICTION_VEL_THRESHOLD else 0.0
            friction_force = static_friction + dyn_friction
            # p.setJointMotorControl2(
            #     self.robot, self.barrett_right_jointIds[i], p.VELOCITY_CONTROL, targetVelocity=0, force=friction_force)

            p.setJointMotorControl2(
                self.robot, self.barrett_right_jointIds[i], p.VELOCITY_CONTROL, targetVelocity=np.clip(torque_cmd_right[i],-10.,10.), force=10.)

            # p.setJointMotorControl2(
            #     self.robot, self.barrett_right_jointIds[i], p.POSITION_CONTROL, targetPosition=position_cmd_right[i], force=10.)
        # print("left: ", np.clip(torque_cmd_left,-1,1))
        # print("right: ", np.clip(torque_cmd_right,-1,1))
        
        if self._barrett_hands_mode is p.TORQUE_CONTROL:
            self.barrett_left_hand_torque = torque_cmd_left.copy()
            self.barrett_right_hand_torque = torque_cmd_right.copy()
            
    
    def drive_yaw(self, torque_z):
        """
            Applies the torques given in body frame to drive the yaw

            @parameter[in] torque_z     torque (Nm) to be applied around the z-axis in the global frame
        """
        # Saturate yaw torques
        torque_z = np.clip(torque_z, -MAX_YAW_TORQUE_NM, MAX_YAW_TORQUE_NM)

        torque_commands = [0.0, 0.0, torque_z]

        # Rotate torque from WORLD frame to BALL frame
        torque_commands = self.transformWorldToBallFrame(torque_commands)
        # print("TorqueBall: ", torque_commands)

        # # unlock motors
        p.setJointMotorControlMultiDof(self.robot,
                                       self.ballJointId[0],
                                       controlMode=p.POSITION_CONTROL,
                                       targetPosition=[0, 0, 0, 1],
                                       force=[0, 0, 0])

        # change to torque mode and set desired torques
        p.setJointMotorControlMultiDof(self.robot,
                                       self.ballJointId[0],
                                       controlMode=p.TORQUE_CONTROL,
                                       force=torque_commands)