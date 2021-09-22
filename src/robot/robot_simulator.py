#!/usr/bin/env python
#  Copyright Microdynamic Systems Laboratory 2021
#
# @author Cornelia Bauer <cornelib@andrew.cmu.edu>
# @author Roberto Shu <rshum@cmu.edu>
#
# @brief Python script to launch a ballbot simulation in pybuller
#

import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import time
from datetime import datetime
import numpy as np
from enum import Enum

from robot.definitions import *
from environments.corner_env import *
from environments.environments import TableEnv
from robot.ballbot import Ballbot as ballbot_sim
from controllers.body_controller import BodyController
from controllers.arm_controller import ArmController

# Simulation parameters
# TODO: simulation time step has to be set to 1/240 as it is the default pybullet time step
#  can be changed using p.setTimeStep()
SIMULATION_TIME_STEP_S = 1/240.
MAX_SIMULATION_TIME_S = 10
USE_ROS = True

if USE_ROS:
    # ROS imports
    import rosgraph
    import rospy
    import rospkg
    from rosgraph_msgs.msg import Clock
    from ballbot_arm_msgs.msg import ArmCommand, ArmsJointState
    from rt_msgs.msg import OlcCmd, VelCmd, State, Odom
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import LaserScan

    # Find package work space to retrieve urdf
    rospack = rospkg.RosPack()
    PACKAGE_WS_PATH = rospack.get_path(
        'ballbot_arm_description').split("/ballbot_arm_description")[0]
else:
    # PACKAGE_WS_PATH =  '/home/rshu/Workspace/pybullet_ws/src/'
    #PACKAGE_WS_PATH =  '/usr0/home/cornelib/sandbox/bullet_ws/src/'
    PACKAGE_WS_PATH = '/home/ballbot/Workspace/pybullet_ws/src/'


class BallState(Enum):
    STATIC = 1
    SAFETY_CHECK = 2
    BALANCE = 3
    OLC = 4
    STATION_KEEP = 5
    VEL_CONTROL = 6
    BALANCE_LEGS_UP = 7
    DFC = 8


class RobotSimulator(object):
    def __init__(self, startPos=[0.0, 0.0, 0.12], startOrientationEuler=[0, 0, 0]):
        # set pybullet environment
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

        # set environment physics
        p.setGravity(0, 0, -10)

        # set ground plane
        planeId = p.loadURDF("plane.urdf")

        # Load ballbot model
        self.ballbot = ballbot_sim(urdf_path=PACKAGE_WS_PATH + URDF_NAME,
                                   startPos=startPos, startOrientationEuler=startOrientationEuler)

        # TODO make a cleaner version for setting initial state pose.
        joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0.0, 0.0, 0.0]
        self.ballbot.set_initial_config(joint_positions)
        arm_joint_position = [0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0, -1.5, 0, 0, 0]
        self.ballbot.set_arms_intial_config(arm_joint_position)

        self.ballbot_state = BallState.BALANCE
        # self.ballbot.print_model_info()
        # self.ballbot.print_joint_info()

        # By default have an empty environment
        self.environemnt = None

        self.setup_gui()
        if USE_ROS:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

            p.resetDebugVisualizerCamera(
                cameraDistance=4.0, cameraYaw=110, cameraPitch=-30, cameraTargetPosition=[0.24, -0.02, -0.09])
            self.setup_ROS()

        # Setup controller
        self.setup_controller()
        self.read_user_params()

        # Control Variables
        self.olcCmdXAng = 0.0
        self.olcCmdYAng = 0.0
        self.olcCmdXVel = 0.0
        self.olcCmdYVel = 0.0

        # setup static ballbot for arm gravity compensation
        self.physicsClientStatic = bc.BulletClient(connection_mode=p.DIRECT)
        self.physicsClientStatic.setGravity(0, 0, -10)
        self.robot_static = self.physicsClientStatic.loadURDF(
            PACKAGE_WS_PATH + URDF_NAME, startPos, p.getQuaternionFromEuler(startOrientationEuler), useFixedBase=True)
        for j in range(self.physicsClientStatic.getNumJoints(self.robot_static)):
            self.physicsClientStatic.changeDynamics(
                self.robot_static, j, linearDamping=0, angularDamping=0)

    def setup_gui(self):
        # set user debug parameters
        self.gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
        self.controller_gains = []
        self.controller_gains.append(p.addUserDebugParameter(
            "Kp", 0, 5000, 250))
        self.controller_gains.append(p.addUserDebugParameter(
            "Kd", 0, 100, 10))
        self.controller_gains.append(p.addUserDebugParameter(
            "Ki", 0, 100, 10))
        self.armPosCmdId = []
        for i in range(len(self.ballbot.arm_joint_names)):
            self.armPosCmdId.append(p.addUserDebugParameter(
                self.ballbot.arm_joint_names[i].decode("utf-8"), -4, 4, 0))

        # OLC CMDS
        self.olc_cmd = []
        self.olc_cmd.append(p.addUserDebugParameter(
            "olc_cmd_XAng", -0.2, 0.2, 0.))
        self.olc_cmd.append(p.addUserDebugParameter(
            "olc_cmd_YAng", -0.2, 0.2, 0.))

    def setup_ROS(self):
        if not rosgraph.is_master_online():
            print("------------------------------------------------------------------")
            print("Error: ROS master is not running. Please run 'roscore' in terminal.")
            p.disconnect()
            raise SystemExit
        rospy.init_node('pybullet_ballbot')
        # Subscriber
        # Ball cmds and state
        self.olc_cmd_sub = rospy.Subscriber(
            "/rt/olc_cmd", OlcCmd, self.update_olc_cmd)
        self.olc_command_ROS = {'xAng': 0.0,
                                'yAng': 0.0, 'xVel': 0.0, 'yVel': 0.0}

        self.vel_cmd_sub = rospy.Subscriber(
            "/rt/vel_cmd", VelCmd, self.update_vel_cmd)

        self.state_cmd_sub = rospy.Subscriber(
            "/rt/state_cmd", State, self.update_state_cmd)

        # Arms
        self.rarm_joint_sub = rospy.Subscriber(
            "/ballbotArms/controller/joint_impedance/right/command", ArmCommand, self.update_rarm_cmd)
        self.rarm_joint_command = [0, 0, 0, 0, 0, 0, 0]
        self.larm_joint_sub = rospy.Subscriber(
            "/ballbotArms/controller/joint_impedance/left/command", ArmCommand, self.update_larm_cmd)
        self.larm_joint_command = [0, 0, 0, 0, 0, 0, 0]

        self.rarm_effort_sub = rospy.Subscriber(
            "/ballbotArms/controller/effort/right/command", Float64MultiArray, self.update_rarm_effort_cmd)
        self.rarm_torque_command = [0, 0, 0, 0, 0, 0, 0]
        self.larm_effort_sub = rospy.Subscriber(
            "/ballbotArms/controller/effort/left/command", Float64MultiArray, self.update_larm_effort_cmd)
        self.larm_torque_command = [0, 0, 0, 0, 0, 0, 0]

        # Sensors
        self.body_laser_pub = rospy.Publisher(
            "/scan", LaserScan, queue_size=10)
        self.body_laser_msg = LaserScan()
        self.body_laser_msg.header.frame_id = BODY_LASER_LINK_NAME

        # Publisher
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1000)
        self.sim_clock_msg = Clock()
        self.sim_wall_time = 0.0

        self.odom_pub = rospy.Publisher("/rt/odom", Odom, queue_size=1000)
        self.odom_msg = Odom()
        self.arms_pub = rospy.Publisher(
            "/ballbotArms/hardware_interface/joint_states", ArmsJointState, queue_size=1000)
        self.arms_msg = ArmsJointState()
        print("ROS communication initialized")

    def setup_environment(self, env):
        self.environemnt = env

    def start_video_log(self, video_file_name):
        # augment filename with date and time
        dt_string = datetime.now().strftime("-%Y-%m-%d-%H-%M-%S")
        file_name = video_file_name + dt_string + ".mp4"
        self.log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, file_name)

    def stop_video_log(self):
        if self.log_id is not None:
            p.stopStateLogging(self.log_id)
        else:
            print("[ERROR] no video log initialized, call start_video_log()")

    def update_olc_cmd(self, msg):
        self.olc_command_ROS['xAng'] = msg.xAng
        self.olc_command_ROS['yAng'] = msg.yAng
        self.olc_command_ROS['xVel'] = msg.xVel
        self.olc_command_ROS['yVel'] = msg.yVel

    def update_vel_cmd(self, msg):
        self.olc_command_ROS['xVel'] = msg.velX
        self.olc_command_ROS['yVel'] = msg.velY

    def update_state_cmd(self, msg):
        # self.state_command['ballState'] = msg.ballState
        self.update_robot_state(BallState(msg.ballState))

    def update_rarm_cmd(self, msg):
        self.rarm_joint_command = msg.position

    def update_larm_cmd(self, msg):
        self.larm_joint_command = msg.position

    def update_rarm_effort_cmd(self, msg):
        self.rarm_torque_command = msg.data

    def update_larm_effort_cmd(self, msg):
        self.larm_torque_command = msg.data

    def setup_controller(self):
        self.body_controller = BodyController()
        self.rarm_controller = ArmController()
        self.larm_controller = ArmController()
        self.arm_joint_command = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def update_robot_state(self, state):
        if state == BallState.STATIC:
            self.ballbot_state = BallState.STATIC
            print("Ballbot state changed to STATIC")
        elif state == BallState.BALANCE:
            if self.ballbot_state != BallState.BALANCE:
                self.ballbot_state = BallState.BALANCE
                print("Ballbot state changed to BALANCE")
        elif state == BallState.STATION_KEEP:
            if self.ballbot_state != BallState.STATION_KEEP:
                self.ballbot_state = BallState.STATION_KEEP
                print("Ballbot state changed to STATION KEEP")
        elif state == BallState.OLC:
            if self.ballbot_state != BallState.OLC:
                self.ballbot_state = BallState.OLC
                print("Ballbot state changed to OLC")
        else:
            self.ballbot_state = BallState.BALANCE
            print("[ERROR] Invalid robot state, set by default to BALANCE")

    def step(self):

        # Update robot sensors
        self.ballbot.lidar.update()

        # Update robot state
        self.ballbot.update_robot_state()
        body_orient_euler = self.ballbot.get_body_orientation()
        body_orient_euler_vel = self.ballbot.get_base_velocity()
        self.ballbot.get_ball_state()
        # self.body_controller.set_data(SIMULATION_TIME_STEP_S,body_orient_euler,ball_velocity)
        self.body_controller.update_body_orient(body_orient_euler)
        self.body_controller.update_com_position(
            self.ballbot.comPosInBodyOrient[0], self.ballbot.comPosInBodyOrient[1])
        self.body_controller.update_ball_position(
            self.ballbot.ballPosInBodyOrient[0], self.ballbot.ballPosInBodyOrient[1])
        self.body_controller.update_ball_velocity(
            self.ballbot.ballLinVelInBodyOrient[0], self.ballbot.ballLinVelInBodyOrient[1])

        """ Ball Commands """
        # Station Keeping controller
        if self.ballbot_state == BallState.STATION_KEEP:
            if not self.body_controller._station_keeping_started:
                self.body_controller.set_desired_ball_position(
                    self.ballbot.ballPosInBodyOrient[0], self.ballbot.ballPosInBodyOrient[1])
            self.body_controller.station_keep()
        else:
            self.body_controller.clear_station_keeping_error_values()

        # Outer Loop controller
        if self.ballbot_state == BallState.OLC:
            self.body_controller.set_desired_angles_odom_frame(
                self.olcCmdXAng, self.olcCmdYAng)
            self.body_controller.set_desired_velocity_odom_frame(
                self.olcCmdXVel, self.olcCmdYVel)

        # Balancing controller
        if (self.ballbot_state == BallState.BALANCE or self.ballbot_state == BallState.OLC
                or self.ballbot_state == BallState.STATION_KEEP or self.ballbot_state == BallState.VEL_CONTROL):
            # If none of the outer loop controllers are running,
            # set desired angles and linear position to zero
            if self.ballbot_state == BallState.BALANCE:
                self.body_controller.set_planned_body_angles(0, 0)
                self.body_controller.set_desired_body_angles(0, 0)
                self.body_controller.set_desired_ball_position(0, 0)
                self.body_controller.set_desired_ball_velocity(0, 0)
                self.body_controller.set_desired_world_velocity(0, 0)
                self.body_controller.set_desired_com_position(
                    self.ballbot.ballPosInBodyOrient[0], self.ballbot.ballPosInBodyOrient[1])
            # Filter feedback
            self.body_controller.balance(SIMULATION_TIME_STEP_S)
        else:
            self.body_controller.clear_balancing_error_values()

        # Set torque commands
        current_yy = -self.body_controller.xBallCurrent
        current_xx = self.body_controller.yBallCurrent
        torque_yy = self.body_controller.yBallTorque
        torque_xx = self.body_controller.xBallTorque

        '''
        print("torque_xx: ", torque_xx)
        print("torque_yy: ", torque_yy)
        print("current_xx: ", current_xx)
        print("current_yy: ", current_yy)
        '''

        """ Arm Commands """
        if self.ballbot._arm_mode == p.POSITION_CONTROL:
            #  calculate gravity torques
            self.calculate_gravity_torques()

            self.rarm_controller.update_current_state(
                self.ballbot.arm_pos[0:7], self.ballbot.arm_vel[0:7])
            self.rarm_controller.set_desired_angles(
                self.arm_joint_command[0:7])
            self.rarm_controller.set_gravity_torque(self.gravity_torques[0:7])
            self.rarm_controller.update(SIMULATION_TIME_STEP_S)
            rarm_torques = self.rarm_controller.armTorques

            self.larm_controller.update_current_state(
                self.ballbot.arm_pos[7:], self.ballbot.arm_vel[7:])
            self.larm_controller.set_desired_angles(self.arm_joint_command[7:])
            self.larm_controller.set_gravity_torque(self.gravity_torques[7:])
            self.larm_controller.update(SIMULATION_TIME_STEP_S)
            larm_torques = self.larm_controller.armTorques
        else:
            rarm_torques = self.rarm_torque_command
            larm_torques = self.larm_torque_command

        # Apply torque to robot
        self.ballbot.drive_arms(self.arm_joint_command, np.concatenate(
            (np.array(rarm_torques), np.array(larm_torques))))
        self.ballbot.drive_imbd(torque_xx, torque_yy)
        # self.ballbot.drive_imbd(current_xx,current_yy)

    def read_user_params(self):
        Kp = p.readUserDebugParameter(self.controller_gains[0])
        Kd = p.readUserDebugParameter(self.controller_gains[1])
        Ki = p.readUserDebugParameter(self.controller_gains[2])
        self.body_controller._com_balancing_control.set_gains(Kp, Ki, Kd)

        # desired arm joint angles
        for i in range(len(self.ballbot.arm_joint_names)):
            self.arm_joint_command[i] = p.readUserDebugParameter(
                self.armPosCmdId[i])

        # olc cmds
        self.olcCmdXAng = p.readUserDebugParameter(self.olc_cmd[0])
        self.olcCmdYAng = p.readUserDebugParameter(self.olc_cmd[1])

    def read_ROS_params(self):
        self.arm_joint_command = np.concatenate(
            (np.array(self.rarm_joint_command), np.array(self.larm_joint_command)))
        # TODO ask Roberto if this should only be called if robot in OLC mode
        self.olcCmdXAng = self.olc_command_ROS['xAng']
        self.olcCmdYAng = self.olc_command_ROS['yAng']
        self.olcCmdXVel = self.olc_command_ROS['xVel']
        self.olcCmdYVel = self.olc_command_ROS['yVel']

    def publish_state(self):
        self.odom_msg.xPos = self.ballbot.ballPosInWorldFrame[0]
        self.odom_msg.yPos = self.ballbot.ballPosInWorldFrame[1]
        # TODO make sure this is the yaw we want here and all these properties are correct
        self.odom_msg.yaw = self.ballbot.bodyOrientEuler[2]
        self.odom_msg.xVel = self.ballbot.ballLinVelInWorldFrame[0]
        self.odom_msg.yVel = self.ballbot.ballLinVelInWorldFrame[1]
        self.odom_msg.xAng = self.ballbot.bodyOrientEuler[0]
        self.odom_msg.yAng = self.ballbot.bodyOrientEuler[1]
        self.odom_msg.xAngVel = self.ballbot.ballRadialVelInBodyOrient[0]
        self.odom_msg.yAngVel = self.ballbot.ballRadialVelInBodyOrient[1]
        self.odom_pub.publish(self.odom_msg)

        self.arms_msg.header.stamp = rospy.Time.now()
        self.arms_msg.right_arm_state.position = self.ballbot.arm_pos[0:7]
        self.arms_msg.right_arm_state.velocity = self.ballbot.arm_vel[0:7]
        self.arms_msg.left_arm_state.position = self.ballbot.arm_pos[7:]
        self.arms_msg.left_arm_state.velocity = self.ballbot.arm_vel[7:]
        self.arms_pub.publish(self.arms_msg)

    def publish_sensor_data(self):

        # Body Lidar
        self.body_laser_msg.header.stamp = rospy.Time.now()
        self.body_laser_msg.range_min = self.ballbot.lidar.angle_min
        self.body_laser_msg.range_max = self.ballbot.lidar.angle_max
        self.body_laser_msg.angle_increment = self.ballbot.lidar.angle_delta
        self.body_laser_msg.range_max = self.ballbot.lidar.range_max
        self.body_laser_msg.range_min = self.ballbot.lidar.range_min

        self.body_laser_msg.ranges = self.ballbot.lidar.get_hits()

        self.body_laser_pub.publish(self.body_laser_msg)

    def publish_sim_time(self):
        self.sim_wall_time += SIMULATION_TIME_STEP_S
        self.sim_clock_msg.clock = rospy.Time.from_sec(self.sim_wall_time)
        self.clock_pub.publish(self.sim_clock_msg)

    def calculate_gravity_torques(self):
        # get gravity torque for current arm state from parallel simulation
        for i in range(self.ballbot.nArmJoints):
            # self.physicsClientStatic.setJointMotorControl2(self.robot_static, self.ballbot.jointIds[i],
            #   p.POSITION_CONTROL,self.arm_joint_command[i], force = 5 * 240.)
            self.physicsClientStatic.setJointMotorControl2(self.robot_static, self.ballbot.jointIds[i],
                                                           p.POSITION_CONTROL, self.ballbot.arm_pos[i], force=5 * 240.)
        for i in range(10):
            self.physicsClientStatic.stepSimulation()
        self.gravity_torques = [self.physicsClientStatic.getJointState(
            self.robot_static, self.ballbot.jointIds[i])[-1] for i in range(self.ballbot.nArmJoints)]
