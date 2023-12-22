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
from environments.environments import TableEnv
import quaternion
from omegaconf import OmegaConf
from tqdm import trange

from robot.definitions import *
from robot.ballbot import Ballbot as ballbot_sim
from controllers.body_controller import BodyController
from controllers.outer_loop_controllers import StationKeepingController
from controllers.arm_controller import ArmController
from controllers.arm_controller import TaskSpaceArmController
from controllers.turret_controller import TurretController
from controllers.barrett_left_hand_controller import BarrettLeftHandController
from controllers.barrett_right_hand_controller import BarrettRightHandController


# Simulation parameters
# TODO: simulation time step has to be set to 1/240 as it is the default pybullet time step
#  can be changed using p.setTimeStep()
SIMULATION_TIME_STEP_S = 1/240.
SENSOR_TIME_STEP_S = 1/60.
MAX_SIMULATION_TIME_S = 10

PRINT_MODEL_INFO = False
DRAW_CONTACT_FORCES = False

import os
os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '330'

PACKAGE_WS_PATH = NON_ROS_PATH

# from robot.robot_simulator import BallState

class BallState(Enum):
    STATIC = 1
    SAFETY_CHECK = 2
    BALANCE = 3
    OLC = 4
    STATION_KEEP = 5
    VEL_CONTROL = 6
    BALANCE_LEGS_UP = 7
    DFC = 8

class RobotSimulatorPickup(object):
    def __init__(self, env_cfg=None, task_cfg=None):
        self.task_cfg = task_cfg
        if env_cfg is None:
            env_cfg = OmegaConf.load('/home/saumyas/ballbot_sim_py3_ws/src/ballbot_pybullet_sim/src/config/envs/ballbot_pickup_env.yaml')
        
        self.env_cfg = env_cfg
        use_gui = env_cfg['use_gui']
        self.enable_turret_camera = env_cfg['cameras']['enable_turret_camera']
        self.enable_static_camera = env_cfg['cameras']['enable_static_camera']

        self.use_arm_olc = False
        self.use_arm_olc_com = False

        # set pybullet environment
        if use_gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        
        if not use_gui and (self.enable_turret_camera or self.enable_static_camera): # if using camera without GUI on
            import pkgutil
            egl = pkgutil.get_loader('eglRenderer')
            self.plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin") #without this (https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=13395) issue arises

        # set environment physics
        p.setGravity(0, 0, -10)

        # set ground plane
        planeId = p.loadURDF("plane.urdf")

        # Load ballbot model
        ball_frame_start_pos = [0.0,0.0,0.11]
        ball_frame_start_orient_euler = env_cfg['ballbot_initial_state']['ball']['init_ball_orient']
        self.ballbot = ballbot_sim(urdf_path=PACKAGE_WS_PATH + URDF_NAME,
                                   startPos=ball_frame_start_pos, startOrientationEuler=ball_frame_start_orient_euler)

        if PRINT_MODEL_INFO:
            self.ballbot.print_model_info()
            self.ballbot.print_joint_info()
            self.ballbot.print_link_info()

        # By default have an empty environment
        self.environment = None
        env = TableEnv(startPos=env_cfg['env_initial_state']['table']['table_pos'], startOrientationEuler=[
                   0., 0., np.radians(0.)])
        self.setup_environment(env)

        if USE_ROS:
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0, cameraYaw=180., cameraPitch=-20, cameraTargetPosition=[0.0, 0.0, 1.0])
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            self.setup_ROS()
        else:
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0, cameraYaw=180., cameraPitch=-20, cameraTargetPosition=[0.0, 0.0, 1.0])
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            # self.setup_gui()
            # self.read_user_params()
            # pass

        # setup static ballbot for arm gravity compensation
        self.physicsClientStatic = bc.BulletClient(connection_mode=p.DIRECT)
        self.physicsClientStatic.setGravity(0, 0, -10)
        self.robot_static = self.physicsClientStatic.loadURDF(
            PACKAGE_WS_PATH + URDF_NAME, ball_frame_start_pos, p.getQuaternionFromEuler(ball_frame_start_orient_euler), useFixedBase=True)
        for j in range(self.physicsClientStatic.getNumJoints(self.robot_static)):
            self.physicsClientStatic.changeDynamics(
                self.robot_static, j, linearDamping=0, angularDamping=0)
        
        # self.save_freq = self.task_cfg.get('FT_save_freq',4)
        self.reset()
        self.ballbot.update_robot_state()

    def reset_control_variables(self):
        # Body
        self.olcCmdXAng = 0.0
        self.olcCmdYAng = 0.0
        self.olcCmdXVel = 0.0
        self.olcCmdYVel = 0.0
        
        # Arms
        self.rarm_joint_command = self.init_arm_joint_position[:7]
        self.larm_joint_command = self.init_arm_joint_position[7:]
        self.arm_joint_command = self.init_arm_joint_position.copy()
        
        self.rarm_torque_command = np.zeros(int(self.ballbot.nArmJoints/2))
        self.larm_torque_command = np.zeros(int(self.ballbot.nArmJoints/2))

        self.rarm_ts_command_pos = np.zeros((3,))
        self.rarm_ts_command_quat = quaternion.from_float_array([1., 0., 0., 0.]) # TODO(saumya)
        self.larm_ts_command_pos = np.zeros((3,))
        self.larm_ts_command_quat = quaternion.from_float_array([1., 0., 0., 0.]) # TODO(saumya)
        self.tsc_enable_base = False

        self.wrench_right = np.zeros(6)
        self.wrench_left = np.zeros(6)

        self.turret_joint_command = self.init_turret_joint_position # pan, tilt
        self.turret_torque_command = [0, 0] # pan, tilt

        if self.ballbot.hand_type == 'barrett':
            self.barrett_left_hand_joint_command = self.init_barrett_left_joint_positions.copy()
            self.barrett_right_hand_joint_command = self.init_barrett_right_joint_positions.copy()

        self.rarm_ts_command = [0, 0, 0]
        self.larm_ts_command = [0, 0, 0]

        self.barrett_gripper_close_state = np.array([1.47, 0., 0., 1.47, 0., 0., 1.47, 0.])
        self.barrett_gripper_open_state = np.array([0., 0., 0., 0., 0., 0., 0., 0.])

    def reset_state(self):
        # Reset ballbot base
        if self.env_cfg['ballbot_initial_state']['ball']['randomize_ball_pos']:
            init_ball_xy = np.random.uniform(
                low=self.env_cfg['ballbot_initial_state']['ball']['min_xy'], 
                high=self.env_cfg['ballbot_initial_state']['ball']['max_xy'])
            init_ball_pos = [init_ball_xy[0], init_ball_xy[1], self.env_cfg['ballbot_initial_state']['ball']['init_ball_pos'][2]]
        else:
            init_ball_pos= self.env_cfg['ballbot_initial_state']['ball']['init_ball_pos']
        p.resetBasePositionAndOrientation(self.ballbot.robot,
                                        init_ball_pos,
                                        p.getQuaternionFromEuler(self.env_cfg['ballbot_initial_state']['ball']['init_ball_orient']))

        # Reset ballbot arms
        if self.env_cfg['ballbot_initial_state']['left_arm']['randomize_arm_q'] or self.env_cfg['ballbot_initial_state']['right_arm']['randomize_arm_q']:
            raise NotImplementedError('Randomization of arm initial joint states not implemented')
        else:
            self.init_arm_joint_position = np.append(
                np.array(self.env_cfg['ballbot_initial_state']['right_arm']['init_arm_q']),
                np.array(self.env_cfg['ballbot_initial_state']['left_arm']['init_arm_q'])
            )
        self.ballbot.set_arms_initial_config(self.init_arm_joint_position)

        # Reset turret
        if self.env_cfg['ballbot_initial_state']['turret']['randomize_q']:
            raise NotImplementedError('Randomization of turret initial state not implemented')
        else:
            self.init_turret_joint_position = self.env_cfg['ballbot_initial_state']['turret']['init_q'] # ['turret_pan', 'turret_tilt']
        self.ballbot.set_turret_intial_config(self.init_turret_joint_position)
        
        if self.ballbot.hand_type == 'barrett':
            if self.env_cfg['ballbot_initial_state']['gripper_left']['randomize_q']:
                raise NotImplementedError('Randomization of barrett left hand initial state not implemented')
            else:
                if self.env_cfg['ballbot_initial_state']['gripper_left']['init_state'] == 'open':
                    self.init_barrett_left_joint_positions = np.zeros(self.ballbot.nBarrettLeftJoints)
                else:
                    self.init_barrett_left_joint_positions = np.array([1.47, 0., 0., 1.47, 0., 0., 1.47, 0.])
            self.ballbot.set_barrett_left_hand_intial_config(self.init_barrett_left_joint_positions)

            if self.env_cfg['ballbot_initial_state']['gripper_left']['randomize_q']:
                raise NotImplementedError('Randomization of barrett right hand initial state not implemented')
            else:
                if self.env_cfg['ballbot_initial_state']['gripper_right']['init_state'] == 'open':
                    self.init_barrett_right_joint_positions = np.zeros(self.ballbot.nBarrettRightJoints)
                else:
                    self.init_barrett_right_joint_positions = np.array([1.47, 0., 0., 1.47, 0., 0., 1.47, 0.])
            self.ballbot.set_barrett_right_hand_intial_config(self.init_barrett_right_joint_positions)

        # reset environment
        successful_sample = False
        if self.env_cfg['env_initial_state']['objects']['randomize_obj_locations']:
            for i in range(self.env_cfg['env_initial_state']['max_samples']):
                cube1_xy = np.random.uniform(
                    low=self.env_cfg['env_initial_state']['objects']['cube1']['min_pos'], 
                    high=self.env_cfg['env_initial_state']['objects']['cube1']['max_pos'])
                cube2_xy = np.random.uniform(
                    low=self.env_cfg['env_initial_state']['objects']['cube2']['min_pos'], 
                    high=self.env_cfg['env_initial_state']['objects']['cube2']['max_pos'])
                if np.linalg.norm(cube1_xy[:2]-cube2_xy[:2]) > 0.2:
                    successful_sample = True
                    break
            if not successful_sample:
                cube1_xy = self.env_cfg['env_initial_state']['objects']['cube1']['init_pos']
                cube2_xy = self.env_cfg['env_initial_state']['objects']['cube2']['init_pos']
        else:
            cube1_xy = self.env_cfg['env_initial_state']['objects']['cube1']['init_pos']
            cube2_xy = self.env_cfg['env_initial_state']['objects']['cube2']['init_pos']
        
        cube1_pos = np.array(self.environment.table_start_pos) + \
            np.array([cube1_xy[0], cube1_xy[1], self.environment.table_height+self.environment.cube_height/2])
        p.resetBasePositionAndOrientation(self.environment.cubeId1,
                        cube1_pos,
                        p.getQuaternionFromEuler([0.,0.,0.]))
    
        cube2_pos = np.array(self.environment.table_start_pos) + \
            np.array([cube2_xy[0], cube2_xy[1], self.environment.table_height+self.environment.cube_height/2])
        p.resetBasePositionAndOrientation(self.environment.cubeId2,
                        cube2_pos,
                        p.getQuaternionFromEuler([0.,0.,0.]))
        
    def reset(self, env_cfg=None):
        # TODO: use env_cfg to reset block colors
        self.current_step = 0
        self.contacted_block = False

        self.setup_controller()
        self.reset_state()
        self.reset_control_variables()

        self.ballbot._arm_mode = p.POSITION_CONTROL # p.TORQUQ_CONTROL, TASK_SPACE_CONTROL
        self.ballbot._turret_mode = p.POSITION_CONTROL
        self.ballbot._barrett_hands_mode = p.POSITION_CONTROL
        self.ballbot_state = BallState.BALANCE

        self.ballbot.update_robot_state()

        for t in range(50):
            self.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)
            self._set_goal()
        
        return self._get_obs()

    def _get_obs(self):
        right_ee_frame_info = p.getLinkState(self.ballbot.robot, self.ballbot.linkIds['toolR'])
        contacts = p.getContactPoints(self.ballbot.robot, self.environment.cubeId2)
        obs_contact = np.array(0.)
        if len(contacts) > 0 or self.contacted_block:
            self.contacted_block = True
            obs_contact = np.array(1.)
        
        obs = np.append(np.array(right_ee_frame_info[0]), obs_contact)
        return obs

    def _set_goal(self):
        cube_pos, cube_quat = p.getBasePositionAndOrientation(self.environment.cubeId2)
        self.target_right_pos_reach = np.array(cube_pos) + np.array([0., 0., 0.03])

        self.target_right_pos_lift = self.target_right_pos_reach + np.array([0., 0.2, 0.1])
        
    def evaluate_state(self):
        
        cube_pos, cube_quat = p.getBasePositionAndOrientation(self.environment.cubeId2)

        # Success criteria for lifting the cube
        lifted_cube = np.linalg.norm(self.target_right_pos_lift - cube_pos) < 0.07

        # Success criteria for grasping the cube
        right_ee_frame_info = p.getLinkState(self.ballbot.robot, self.ballbot.linkIds['toolR'])
        reached_cube = np.linalg.norm(right_ee_frame_info[0] - (np.array(cube_pos) + np.array([0., 0., 0.03]))) < 0.07
        
        gripper_closed = np.linalg.norm(self.ballbot.barrett_right_hand_pos - self.barrett_gripper_close_state) < 1.3
        
        success = reached_cube and gripper_closed

        reward = 0.
        info = {'success': success,}
        return reward, info
        
    def set_arm_task_space_control_mode(self, arms='right', enable_base=True):
        self.control_left_arm = True if (arms=='both' or arms == 'left') else False
        self.control_right_arm = True if (arms=='both' or arms == 'right') else False
        
        self.ballbot._arm_mode = 'TASK_SPACE_CONTROL'
        self.tsc_enable_base = enable_base
        if enable_base:
            self.update_robot_state(BallState.OLC)
        else:
            self.update_robot_state(BallState.BALANCE)
        print(f"Arm control mode set to TASK SPACE CONTROL for {arms} arms with enable_base={enable_base}")

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
                self.ballbot.arm_joint_names[i], -4, 4, 0))

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
        ### Subscriber
        # Ball cmds and state 
        if self.use_arm_olc:
            self.olc_right_cmd_sub = rospy.Subscriber(
                "/rt/right/olc_cmd", OlcCmd, self.update_right_olc_cmd)
            self.olc_left_cmd_sub = rospy.Subscriber(
                "/rt/left/olc_cmd", OlcCmd, self.update_left_olc_cmd)

        self.olc_cmd_sub = rospy.Subscriber(
            "/rt/olc_cmd", OlcCmd, self.update_olc_cmd)
        

        self.olc_command_ROS = {'xAng': 0.0, 'yAng': 0.0, 
                                'xVel': 0.0, 'yVel': 0.0,
                                'xAng_left': 0.0, 'yAng_left': 0.0, 
                                'xVel_left': 0.0, 'yVel_left': 0.0,
                                'xAng_right': 0.0, 'yAng_right': 0.0, 
                                'xVel_right': 0.0, 'yVel_right': 0.0,}

        self.vel_cmd_sub = rospy.Subscriber(
            "/rt/vel_cmd", VelCmd, self.update_vel_cmd)

        self.state_cmd_sub = rospy.Subscriber(
            "/rt/state_cmd", State, self.update_state_cmd)

        # Arms
        self.torque_mode_srv = rospy.Service('/ballbotArms/switch_torque_mode', SetBool, self.update_torque_mode)
        self.rarm_joint_sub = rospy.Subscriber(
            "/ballbotArms/controller/joint_impedance/right/command", ArmCommand, self.update_rarm_cmd)
        self.larm_joint_sub = rospy.Subscriber(
            "/ballbotArms/controller/joint_impedance/left/command", ArmCommand, self.update_larm_cmd)

        self.rarm_effort_sub = rospy.Subscriber(
            "/ballbotArms/controller/effort/right/command", Float64MultiArray, self.update_rarm_effort_cmd)
        self.larm_effort_sub = rospy.Subscriber(
            "/ballbotArms/controller/effort/left/command", Float64MultiArray, self.update_larm_effort_cmd)

        # task space controller commands
        self.rarm_ts_sub = rospy.Subscriber(
            "/task_space_control/right/command", TaskSpaceTrajectory, self.update_rarm_ts_cmd)
        self.larm_ts_sub = rospy.Subscriber(
            "/task_space_control/left/command", TaskSpaceTrajectory, self.update_larm_ts_cmd)

        # External force commands
        self.external_wrench_right_cmd_sub = rospy.Subscriber("/pybullet/wrench_cmd/right/",Wrench,self.update_external_wrench_right_cmd)
        self.external_wrench_left_cmd_sub = rospy.Subscriber("/pybullet/wrench_cmd/left/",Wrench,self.update_external_wrench_left_cmd)

        ### Publisher
        # Sensors
        self.body_laser_pub = rospy.Publisher(
            "/scan", LaserScan, queue_size=10)
        self.body_laser_msg = LaserScan()
        self.body_laser_msg.header.frame_id = BODY_LASER_LINK_NAME
        
        # Time
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1000)
        self.sim_clock_msg = Clock()
        self.sim_wall_time = 0.0

        # Joint State Publisher
        self.joint_state_pub = rospy.Publisher(
            "/joint_states", JointState, queue_size=1000)
        self.odom_pub = rospy.Publisher("/rt/odom", Odom, queue_size=1000)
        self.odom_msg = Odom()
        self.arms_pub = rospy.Publisher(
            "/ballbotArms/hardware_interface/joint_states", ArmsJointState, queue_size=1000)
        self.arms_msg = ArmsJointState()
        self.body_state_pub = rospy.Publisher(
            "/rt/body_state", Odom, queue_size=10)
        self.body_state_msg = Odom()

        # Force Torque sensor publish
        self.wrench_right_pub = rospy.Publisher("/pybullet/ballbot/state/wrench/right", WrenchStamped, queue_size=1000)
        self.wrench_right_msg = WrenchStamped()
        self.wrench_left_pub = rospy.Publisher("/pybullet/ballbot/state/wrench/left", WrenchStamped, queue_size=1000)
        self.wrench_left_msg = WrenchStamped()

        # TF publisher
        self.tf_pub = tf2_ros.TransformBroadcaster()
        #self.tf_pub = rospy.Publisher("/tf",TFMessage, queue_size=1000)

        print("ROS communication initialized")

    def setup_environment(self, env):
        self.environment = env

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

    def update_right_olc_cmd(self, msg):
        self.olc_command_ROS['xAng_right'] = msg.yAng
        self.olc_command_ROS['yAng_right'] = msg.xAng
        self.olc_command_ROS['xVel_right'] = msg.yVel
        self.olc_command_ROS['yVel_right'] = msg.xVel

    def update_left_olc_cmd(self, msg):
        self.olc_command_ROS['xAng_left'] = msg.yAng
        self.olc_command_ROS['yAng_left'] = msg.xAng
        self.olc_command_ROS['xVel_left'] = msg.yVel
        self.olc_command_ROS['yVel_left'] = msg.xVel

    def update_olc_cmd(self, msg):
        if self.use_arm_olc:
            self.olc_command_ROS['xAng'] = (self.olc_command_ROS['xAng_right']+self.olc_command_ROS['xAng_left'])/1.
            self.olc_command_ROS['yAng'] = (self.olc_command_ROS['yAng_right']+self.olc_command_ROS['yAng_left'])/1.
            self.olc_command_ROS['xVel'] = (self.olc_command_ROS['xVel_right']+self.olc_command_ROS['xVel_left'])/1.
            self.olc_command_ROS['yVel'] = (self.olc_command_ROS['yVel_right']+self.olc_command_ROS['yVel_left'])/1.
        else:
            self.olc_command_ROS['xAng'] = msg.yAng
            self.olc_command_ROS['yAng'] = msg.xAng
            self.olc_command_ROS['xVel'] = msg.yVel
            self.olc_command_ROS['yVel'] = msg.xVel

    def update_vel_cmd(self, msg):
        self.olc_command_ROS['xVel'] = msg.velX
        self.olc_command_ROS['yVel'] = msg.velY

    def update_state_cmd(self, msg):
        # self.state_command['ballState'] = msg.ballState
        self.update_robot_state(BallState(msg.ballState))

    def update_torque_mode(self, req):
        if req.data==True:
            self.ballbot.set_arm_torque_mode()
            print("Ballbot arm mode changed to TORQUE")
        else:
            self.ballbot.set_arm_position_mode()
            self.ballbot.set_turret_position_mode()
            print("Ballbot arm mode changed to POSITION")

    def update_rarm_cmd(self, msg):
        self.rarm_joint_command = msg.position

    def update_larm_cmd(self, msg):
        self.larm_joint_command = msg.position

    def update_rarm_ts_cmd(self, msg):
        self.rarm_ts_command = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z ])

    def update_larm_ts_cmd(self, msg):
        self.larm_ts_command = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z ])

    def update_rarm_effort_cmd(self, msg):
        self.rarm_torque_command = msg.data
        print('Arm message received:', self.rarm_torque_command)

    def update_larm_effort_cmd(self, msg):
        self.larm_torque_command = msg.data
        print('Arm message received:', self.larm_torque_command)

    def update_turret_cmd(self, msg):
        self.turret_joint_command = msg.position

    def update_turret_effort_cmd(self, msg):
        self.turret_torque_command = msg.data

    def update_barrett_left_hand_cmd(self, msg):
        self.barrett_left_hand_joint_command = msg.position

    def update_barrett_left_hand_effort_cmd(self, msg):
        self.barrett_left_hand_torque_command = msg.data
    
    def update_barrett_right_hand_cmd(self, msg):
        self.barrett_right_hand_joint_command = msg.position

    def update_barrett_right_hand_effort_cmd(self, msg):
        self.barrett_right_hand_torque_command = msg.data
    
    def update_external_wrench_right_cmd(self, msg):
        wrench_cmd = [msg.force.x, msg.force.y, msg.force.z, 
                                          msg.torque.x, msg.torque.y, msg.torque.z]

        self.ballbot.apply_external_wrench(wrench_cmd, self.ballbot.rightEndEffectorId)

    def update_external_wrench_left_cmd(self, msg):
        wrench_cmd = [msg.force.x, msg.force.y, msg.force.z, 
                                          msg.torque.x, msg.torque.y, msg.torque.z]

        self.ballbot.apply_external_wrench(wrench_cmd, self.ballbot.leftEndEffectorId)

    def setup_controller(self):
        self.body_controller = BodyController()
        self.body_controller._com_balancing_control.set_gains(250,10,10)
        self.rarm_controller = ArmController()
        self.larm_controller = ArmController()
        self.turret_controller = TurretController()

        self.task_space_left_arm_controller = TaskSpaceArmController(arm='left')
        self.task_space_right_arm_controller = TaskSpaceArmController(arm='right')

        self.tsc_ball_controller = StationKeepingController()
        self.tsc_ball_controller.set_max_angle(6.)
        self.tsc_ball_controller.set_gains(8., 0.0, 1., 8., 0.0, 1.) # kPx, kIx, kDx, kPy, kIy, kDy

        if self.ballbot.hand_type == 'barrett':
            self.barrett_left_hand_controller = BarrettLeftHandController(self.ballbot.nBarrettLeftJoints)
            self.barrett_right_hand_controller = BarrettRightHandController(self.ballbot.nBarrettRightJoints)

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

    def update_sensors(self):
        # Use to update robot sensors at a lower frequency
        # if self.current_step % int(SENSOR_TIME_STEP_S/SIMULATION_TIME_STEP_S) == 0:
        if ENABLE_LASER:
            self.lidarFeedback = self.ballbot.lidar.update()
        if self.enable_turret_camera:
            self.turretCameraFeedback = self.ballbot.update_turretCamera(self.env_cfg['cameras']['image_width'], self.env_cfg['cameras']['image_height'])
        if self.enable_static_camera:
            self.staticCameraFeedback = self.ballbot.update_staticCamera(self.env_cfg['cameras']['image_width'], self.env_cfg['cameras']['image_height'])

    def render(self, camera_name='left_cap2', width=256, height=256):
        if camera_name == 'left_cap2':
            return self.ballbot.update_staticCamera(width, height)
        elif camera_name == 'eye_in_hand_90':
            return self.ballbot.update_turretCamera(width, height)
        else:
            NotImplementedError('Invalid camera name') 

    def step(self):

        # self.update_sensors() # Very slow, use when needed
        self.wrench_right, self.wrench_left = self.ballbot.get_wrist_wrench_measurement()

        # Update robot state
        self.ballbot.update_robot_state()
        body_orient_euler = self.ballbot.get_body_orientation()
        #print("body Angles: ", body_orient_euler)
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

        # Task space controller
        if self.tsc_enable_base and self.ballbot._arm_mode == 'TASK_SPACE_CONTROL':
            xVelErr = 0.0 - self.ballbot.ballLinVelInBodyOrient[0]
            yVelErr = 0.0 - self.ballbot.ballLinVelInBodyOrient[1]
            
            if self.control_left_arm:
                left_ee_frame_info = p.getLinkState(self.ballbot.robot, self.ballbot.linkIds['toolL'])
                left_PosErr_shoulder_frame = self.larm_ts_command_pos - left_ee_frame_info[0]
            
            if self.control_right_arm:
                right_ee_frame_info = p.getLinkState(self.ballbot.robot, self.ballbot.linkIds['toolR'])
                right_PosErr_shoulder_frame = self.rarm_ts_command_pos - right_ee_frame_info[0]
            
            if self.control_left_arm and self.control_right_arm:
                PosErr_avg = (left_PosErr_shoulder_frame + right_PosErr_shoulder_frame)/2
            elif self.control_left_arm:
                PosErr_avg = left_PosErr_shoulder_frame.copy()
            else:
                PosErr_avg = right_PosErr_shoulder_frame.copy()

            self.tsc_ball_controller.set_error_value(
                    PosErr_avg[0], PosErr_avg[1], xVelErr, yVelErr)
            self.tsc_ball_controller.get_angle_output()
            self.olcCmdXAng = self.tsc_ball_controller._desired_y_body_angle
            self.olcCmdYAng = self.tsc_ball_controller._desired_x_body_angle

        # Station Keeping controller
        if self.ballbot_state == BallState.STATION_KEEP:
            if not self.body_controller._station_keeping_started:
                self.body_controller.set_desired_ball_position(
                    self.ballbot.ballPosInBodyOrient[0], self.ballbot.ballPosInBodyOrient[1])
            self.body_controller.station_keep()
        else:
            self.body_controller.clear_station_keeping_error_values()

        # Outer Loop controller
        if self.ballbot_state == BallState.OLC and not self.use_arm_olc_com:
            self.body_controller.set_desired_angles_odom_frame(
                self.olcCmdXAng, self.olcCmdYAng)
            # print("olcCmdXAng",self.olcCmdXAng)
            # print("olcCmdYAng",self.olcCmdYAng)
            self.body_controller.set_desired_velocity_odom_frame(
                self.olcCmdXVel, self.olcCmdYVel)
        elif self.ballbot_state == BallState.OLC and self.use_arm_olc_com:
            self.body_controller.set_desired_body_angles_arms(
                self.olc_command_ROS['xAng_left'], self.olc_command_ROS['yAng_left'],
                self.olc_command_ROS['xAng_right'], self.olc_command_ROS['yAng_right'])
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

        # Run yaw controller
        self.body_controller.run_yaw_control(SIMULATION_TIME_STEP_S)

        # Set torque commands
        current_yy = -self.body_controller.xBallCurrent
        current_xx = self.body_controller.yBallCurrent
        torque_yy = self.body_controller.yBallTorque
        torque_xx = self.body_controller.xBallTorque
        torque_zz = self.body_controller.zBallTorque

        # p.addUserDebugLine([self.olcCmdXAng, self.olcCmdYAng,0.0],[self.olcCmdXAng, self.olcCmdYAng,0.1], [1,0,0], lineWidth=20,lifeTime=0.1)
        # p.addUserDebugLine([self.arm_ts_command[0],self.arm_ts_command[1],self.arm_ts_command[2]],
        #                    [self.arm_ts_command[0],self.arm_ts_command[1],self.arm_ts_command[2]+0.1], [0,1,0], lineWidth=20,lifeTime=0.01)
        # p.addUserDebugLine([self.arm_ts_command[3],self.arm_ts_command[4],self.arm_ts_command[5]],
        #                    [self.arm_ts_command[3],self.arm_ts_command[4],self.arm_ts_command[5]+0.1], [0,0,1], lineWidth=20,lifeTime=0.01)
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
        elif self.ballbot._arm_mode == 'TASK_SPACE_CONTROL':
            if self.control_left_arm:
                current_left_q = np.array(self.ballbot.arm_pos[7:])
                current_left_qdot = np.array(self.ballbot.arm_vel[7:])
                
                des_left_ee_pos, des_left_ee_quat = self.ballbot.transformWorldToShoulderFrame(self.larm_ts_command_pos, 
                                                                                        self.larm_ts_command_quat, arm='left')
                self.task_space_left_arm_controller.set_desired_pose(des_left_ee_pos, des_left_ee_quat)
                self.task_space_left_arm_controller.update_current_state(
                    current_left_q, current_left_qdot)
                self.task_space_left_arm_controller.update(SIMULATION_TIME_STEP_S)
                larm_torques = np.array(self.task_space_left_arm_controller.armTorques).flatten()
            else:
                larm_torques = np.zeros(int(self.ballbot.nArmJoints/2))
            
            if self.control_right_arm:
                current_right_q = np.array(self.ballbot.arm_pos[:7])
                current_right_qdot = np.array(self.ballbot.arm_vel[:7])
                
                des_right_ee_pos, des_right_ee_quat = self.ballbot.transformWorldToShoulderFrame(self.rarm_ts_command_pos, 
                                                                                        self.rarm_ts_command_quat, arm='right')
                self.task_space_right_arm_controller.set_desired_pose(des_right_ee_pos, des_right_ee_quat)
                self.task_space_right_arm_controller.update_current_state(
                    current_right_q, current_right_qdot)
                self.task_space_right_arm_controller.update(SIMULATION_TIME_STEP_S)
                rarm_torques = np.array(self.task_space_right_arm_controller.armTorques).flatten()
            else:
                rarm_torques = np.zeros(int(self.ballbot.nArmJoints/2))

        else:
            rarm_torques = self.rarm_torque_command
            larm_torques = self.larm_torque_command
            # print('larm_torques: ', larm_torques)
            # print('rarm_torques: ', rarm_torques)
        """ Turret Commands """
        if self.ballbot._turret_mode == p.POSITION_CONTROL:

            self.turret_controller.update_current_state(
                self.ballbot.turret_pos, self.ballbot.turret_vel)
            self.turret_controller.set_desired_angles(
                self.turret_joint_command)
            self.turret_controller.update(SIMULATION_TIME_STEP_S)
            turret_torques = self.turret_controller.turretTorques
        else:
            turret_torques = self.turret_torque_command
        
        """ Barrett Hand Commands """
        if self.ballbot.hand_type == 'barrett':
            if self.ballbot._barrett_hands_mode == p.POSITION_CONTROL:

                self.barrett_left_hand_controller.update_current_state(
                    self.ballbot.barrett_left_hand_pos, self.ballbot.barrett_left_hand_vel)
                self.barrett_right_hand_controller.update_current_state(
                    self.ballbot.barrett_right_hand_pos, self.ballbot.barrett_right_hand_vel)
                
                self.barrett_left_hand_controller.set_desired_angles(
                    self.barrett_left_hand_joint_command)
                self.barrett_right_hand_controller.set_desired_angles(
                    self.barrett_right_hand_joint_command)

                self.barrett_left_hand_controller.update(SIMULATION_TIME_STEP_S)
                self.barrett_right_hand_controller.update(SIMULATION_TIME_STEP_S)

                barrett_left_hand_torques = self.barrett_left_hand_controller.barrettLeftHandTorques
                barrett_right_hand_torques = self.barrett_right_hand_controller.barrettRightHandTorques
            else:
                barrett_left_hand_torques = self.barrett_left_hand_torque_command
                barrett_right_hand_torques = self.barrett_right_hand_torque_command

        # Apply torque to robot
        self.ballbot.drive_arms(self.arm_joint_command, np.concatenate(
            (np.array(rarm_torques), np.array(larm_torques))))
        self.ballbot.drive_turret(self.turret_joint_command, 
            np.array(turret_torques))
        if self.ballbot.hand_type == 'barrett':
            self.ballbot.drive_barrett_hands(self.barrett_left_hand_joint_command, self.barrett_right_hand_joint_command, 
                                np.array(barrett_left_hand_torques), np.array(barrett_right_hand_torques))
            
        self.ballbot.drive_imbd(torque_xx, torque_yy)
        # self.ballbot.drive_imbd(current_xx,current_yy)
        self.ballbot.drive_yaw(torque_zz)
        
        contacts = p.getContactPoints(self.ballbot.robot)
        if DRAW_CONTACT_FORCES:
            p.removeAllUserDebugItems()
            for c in contacts:
                p.addUserDebugLine(c[6],c[6] + np.array(c[7])*c[9], [1,0,0])
        self.current_step += 1

    def step_tsc(self, action, action_type='delta_obs_pos', all_low_res = False, all_high_res = False):
        # print('action:', action)
        if all_low_res:
            save_freq = self.task_cfg.get('I3_save_freq',24)
        elif all_high_res:
            save_freq = self.task_cfg.get('Ih_save_freq',12)
        else:
            save_freq = self.task_cfg.get('FT_save_freq',4)

        right_ee_frame_info = p.getLinkState(self.ballbot.robot, self.ballbot.linkIds['toolR'])
        # print('Current ee pos:', right_ee_frame_info[0])
        if action_type == 'delta_obs_pos' or action_type == 'delta_des_tsc':
            self.rarm_ts_command_pos = np.array(right_ee_frame_info[0]) + action[:3]
        elif action_type == 'des_tsc':
            self.rarm_ts_command_pos = action[:3]
        else:
            raise NotImplementedError("Action type not available")
        
        self.rarm_ts_command_quat = quaternion.as_quat_array([0.50829406408023, -0.491436150946075, 0.492614818619556, 0.50740348288175]) # format = 'wxyz'
        # print('Desired ee pos:', self.rarm_ts_command_pos)
        if action[3] > 0.5:
            self.barrett_right_hand_joint_command = self.barrett_gripper_close_state.copy()
        else:
            self.barrett_right_hand_joint_command = self.barrett_gripper_open_state.copy()

        # TODO: maybe do minjerk traj from current to desired?
        for t in range(save_freq):
            self.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)

        reward, info = self.evaluate_state()
        return self._get_obs(), reward, False, info

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
        self.arm_ts_command = np.concatenate(
            (np.array(self.rarm_ts_command), np.array(self.larm_ts_command)))

    def publish_state(self):
        self.odom_msg.xPos = self.ballbot.ballPosInWorldFrame[0]
        self.odom_msg.yPos = self.ballbot.ballPosInWorldFrame[1]
        # TODO make sure this is the yaw we want here and all these properties are correct
        self.odom_msg.yaw = self.ballbot.bodyOrientEuler[2]
        self.odom_msg.xVel = self.ballbot.ballLinVelInWorldFrame[0]
        self.odom_msg.yVel = self.ballbot.ballLinVelInWorldFrame[1]
        self.odom_msg.xAng = self.ballbot.state.xAng
        self.odom_msg.yAng = self.ballbot.state.yAng
        self.odom_msg.xAngVel = self.ballbot.ballRadialVelInBodyOrient[0]
        self.odom_msg.yAngVel = self.ballbot.ballRadialVelInBodyOrient[1]
        self.odom_pub.publish(self.odom_msg)

        self.arms_msg.header.stamp = rospy.Time.now()
        self.arms_msg.right_arm_state.position = self.ballbot.arm_pos[0:7]
        self.arms_msg.right_arm_state.velocity = self.ballbot.arm_vel[0:7]
        self.arms_msg.right_arm_state.effort = self.ballbot.arm_torque[0:7]
        self.arms_msg.left_arm_state.position = self.ballbot.arm_pos[7:]
        self.arms_msg.left_arm_state.velocity = self.ballbot.arm_vel[7:]
        self.arms_msg.left_arm_state.effort = self.ballbot.arm_torque[7:]
        self.arms_pub.publish(self.arms_msg)

        self.body_state_msg.xPos = self.ballbot.ballPosInWorldFrame[0]
        self.body_state_msg.yPos = self.ballbot.ballPosInWorldFrame[1]
        # TODO make sure this is the yaw we want here and all these properties are correct
        self.body_state_msg.yaw = self.ballbot.bodyOrientEuler[2]
        self.body_state_msg.xVel = self.ballbot.ballLinVelInWorldFrame[0]
        self.body_state_msg.yVel = self.ballbot.ballLinVelInWorldFrame[1]
        #self.body_state_msg.xAng = self.ballbot.bodyOrientEuler[0]
        #self.body_state_msg.yAng = self.ballbot.bodyOrientEuler[1]
        self.body_state_msg.xAng = self.ballbot.state.xAng
        self.body_state_msg.yAng = self.ballbot.state.yAng
        self.body_state_msg.xAngVel = self.ballbot.ballRadialVelInBodyOrient[0]
        self.body_state_msg.yAngVel = self.ballbot.ballRadialVelInBodyOrient[1]
        self.body_state_pub.publish(self.body_state_msg)

        self.wrench_right_msg.header.stamp = rospy.Time.now()
        self.wrench_right_msg.header.frame_id = FT_SENSOR_JOINT_NAMES[0]
        self.wrench_right_msg.wrench.force.x = self.wrench_right[0]
        self.wrench_right_msg.wrench.force.y = self.wrench_right[1]
        self.wrench_right_msg.wrench.force.z = self.wrench_right[2]
        self.wrench_right_msg.wrench.torque.x = self.wrench_right[3]
        self.wrench_right_msg.wrench.torque.y = self.wrench_right[4]
        self.wrench_right_msg.wrench.torque.z = self.wrench_right[5]
        self.wrench_right_pub.publish(self.wrench_right_msg)
        
        self.wrench_left_msg.header.stamp = rospy.Time.now()
        self.wrench_left_msg.header.frame_id = FT_SENSOR_JOINT_NAMES[1]
        self.wrench_left_msg.wrench.force.x = self.wrench_left[0]
        self.wrench_left_msg.wrench.force.y = self.wrench_left[1]
        self.wrench_left_msg.wrench.force.z = self.wrench_left[2]
        self.wrench_left_msg.wrench.torque.x = self.wrench_left[3]
        self.wrench_left_msg.wrench.torque.y = self.wrench_left[4]
        self.wrench_left_msg.wrench.torque.z = self.wrench_left[5]
        self.wrench_left_pub.publish(self.wrench_left_msg)

    def publish_sensor_data(self):
        # Body Lidar
        self.body_laser_msg.header.stamp = rospy.Time.now()
        self.body_laser_msg.angle_min = self.ballbot.lidar.angle_min
        self.body_laser_msg.angle_max = self.ballbot.lidar.angle_max
        self.body_laser_msg.angle_increment = self.ballbot.lidar.angle_delta
        self.body_laser_msg.range_max = self.ballbot.lidar.range_max
        self.body_laser_msg.range_min = self.ballbot.lidar.range_min

        self.body_laser_msg.ranges = self.ballbot.lidar.get_hits()

        self.body_laser_pub.publish(self.body_laser_msg)

    def publish_joint_state(self):
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.header.frame_id = "joints"
        self.joint_state_msg.name = JOINT_NAMES

        # TODO add joints for turret
        turret_pos = [0.0, 0.0]
        self.joint_state_msg.position = self.ballbot.state.body_state()+ \
            self.ballbot.arm_pos + turret_pos
        #self.joint_state_msg.velocity = self.ballbot.bodyAngVel + self.ballbot.arm_vel
        #self.joint_state_msg.effort = 0
        self.joint_state_pub.publish(self.joint_state_msg)

    def publish_tf_data(self):
        self.tf_data = []

        # TF: odom -> base_link
        self.tf_data.append(TransformStamped())
        self.tf_data[0].header.stamp = rospy.Time.now()
        self.tf_data[0].header.frame_id = "odom"
        self.tf_data[0].child_frame_id = "base_link"
        self.tf_data[0].transform.translation.x = self.ballbot.ballPosInWorldFrame[0]
        self.tf_data[0].transform.translation.y = self.ballbot.ballPosInWorldFrame[1]
        self.tf_data[0].transform.translation.z = 0.0
        self.tf_data[0].transform.rotation.x = 0.0
        self.tf_data[0].transform.rotation.y = 0.0
        self.tf_data[0].transform.rotation.z = 0.0
        self.tf_data[0].transform.rotation.w = 1.0

        # TF base_link->base_footprint
        self.tf_data.append(TransformStamped())
        self.tf_data[1].header.stamp = rospy.Time.now()
        self.tf_data[1].header.frame_id = "base_link"
        self.tf_data[1].child_frame_id = "base_footprint"
        self.tf_data[1].transform.translation.x = 0.0
        self.tf_data[1].transform.translation.y = 0.0
        self.tf_data[1].transform.translation.z = 0.0
        self.tf_data[1].transform.rotation.x = 0.0
        self.tf_data[1].transform.rotation.y = 0.0
        # TODO: figure out why this is the case in ball code.
        self.tf_data[1].transform.rotation.z = np.sin(
            self.ballbot.bodyOrientEuler[2]/2)
        self.tf_data[1].transform.rotation.w = np.cos(
            self.ballbot.bodyOrientEuler[2]/2)

        # Broadcast TF
        for i in range(len(self.tf_data)):
            self.tf_pub.sendTransform(self.tf_data[i])

    def publish_sim_time(self):
        self.sim_wall_time += SIMULATION_TIME_STEP_S
        self.sim_clock_msg.clock = rospy.Time.from_sec(self.sim_wall_time)
        self.clock_pub.publish(self.sim_clock_msg)

    def publish_ros_data(self):
        self.publish_sim_time()
        self.publish_state()
        self.publish_sensor_data()
        self.publish_joint_state()
        self.publish_tf_data()

    def calculate_gravity_torques(self):
        # get gravity torque for current arm state from parallel simulation
        for i in range(self.ballbot.nArmJoints):
            pass
            # self.physicsClientStatic.setJointMotorControl2(self.robot_static, self.ballbot.jointIds[i],
            #   p.POSITION_CONTROL,self.arm_joint_command[i], force = 5 * 240.)
            self.physicsClientStatic.setJointMotorControl2(self.robot_static, self.ballbot.jointIds[i],
                                                           p.POSITION_CONTROL, self.ballbot.arm_pos[i], force=5 * 240.)
        for i in range(10):
            self.physicsClientStatic.stepSimulation()
        self.gravity_torques = [self.physicsClientStatic.getJointState(
            self.robot_static, self.ballbot.jointIds[i])[-1] for i in range(self.ballbot.nArmJoints)]
