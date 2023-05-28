import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import time
from datetime import datetime
import numpy as np
from enum import Enum
from tqdm import trange
import quaternion

from robot.robot_simulator_pickup_env import *
from robot.definitions import *
from environments.environments import TableEnv
from skills.arm_skills import GotoJointPosition, GotoTaskSpacePose
from skills.composite_skills import *
from skills.gripper_skills import OpenGripper, CloseGripper
from skills.body_skills import GotoBallPosition
from omegaconf import OmegaConf

# Simulation parameters
LOG_VIDEO = False
VIDEO_FILE_NAME = "ballbot_pickup_task"

if __name__ == "__main__":
    # set pybullet environment
    env_cfg = OmegaConf.load('/home/saumyas/ballbot_sim_py3_ws/src/ballbot_pybullet_sim/src/config/envs/ballbot_pickup_env.yaml')
    robot_simulator = RobotSimulatorPickup(env_cfg=env_cfg)

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)
    
    ## ========== Test initial state stability ==============
    # for i in trange(800):
    #     robot_simulator.step()
    #     p.stepSimulation()
    #     time.sleep(SIMULATION_TIME_STEP_S)

    ## ========== Test task space controller ==============
    # T = 5 #seconds
    # left_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolL'])
    # right_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolR'])

    # target_left_pos = np.array(left_ee_frame_info[0]) + np.array([0.0, 1.5, 0.2])
    # target_left_quat = quaternion.as_quat_array([0.50829406408023, -0.491436150946075, 0.492614818619556, 0.50740348288175]) # format = 'wxyz'

    # target_right_pos = np.array(right_ee_frame_info[0]) + np.array([0.0, 1.5, 0.2])
    # target_right_quat = quaternion.as_quat_array([0.50829406408023, -0.491436150946075, 0.492614818619556, 0.50740348288175]) # format = 'wxyz'
    
    # skill = GotoTaskSpacePose(robot_simulator, 
    #                           target_left_pos, 
    #                           target_left_quat,
    #                           target_right_pos, 
    #                           target_right_quat,
    #                           T, arms='both', enable_base=True)
    # skill.execute()

    # T = 3 #seconds
    # skill = GotoJointPosition(robot_simulator, np.array([0.,0.,0.,-1.4,0.,0.,0.]), np.array([0.,0.,0.,1.4,0.,0.,0.]), T)
    # skill.execute()

    # T = 7 #seconds
    # skill = GotoBallPosition(robot_simulator, np.array([0.,0.15]), T)
    # skill.execute()

    # T = 5 #seconds
    # skill = OpenGripper(robot_simulator, T, hands='right')
    # skill.execute()

    # T = 3 #seconds
    # skill = GotoJointPosition(robot_simulator, np.array([0.,0.,0.,-1.4,0.,0.,0.]), np.array([0.2,0.,0.,1.4,0.,0.,0.]), T)
    # skill.execute()


    ## ============= Pickup object using task space control ===================
    for i in range(1):
        robot_simulator.reset()
        skill = PickUpBlockSkill(robot_simulator, img_save_freq=24)
        skill.execute()

    ## ================== Move near table using station keeping control =====================
    # T = 5 #seconds
    # steps = int(T/SIMULATION_TIME_STEP_S)
    # ball_des = [0.0,0.0]
    # robot_simulator.update_robot_state(BallState.STATION_KEEP)
    # ball_pos = robot_simulator.ballbot.ballPosInWorldFrame
    # ballxpos = np.linspace(ball_pos[0],ball_des[0],steps)
    # ballypos = np.linspace(ball_pos[1],ball_des[1],steps)
    # for i in trange(steps):
    # # while(1):
    #     robot_simulator.body_controller.set_desired_ball_position(ballxpos[i],ballypos[i])
        
    #     # Read user params
    #     if USE_ROS:
    #         robot_simulator.read_ROS_params()
    #     else:
    #         robot_simulator.read_user_params()
    #     robot_simulator.step()
    #     p.stepSimulation()

    #     if USE_ROS:
    #         robot_simulator.publish_ros_data()

    #     time.sleep(SIMULATION_TIME_STEP_S)

    ## ======================== Use joint space position control to move arms ==========================
    # T = 5 #seconds
    # steps = int(T/SIMULATION_TIME_STEP_S)
    # robot_simulator.ballbot.set_arm_position_mode()
    # robot_simulator.update_robot_state(BallState.STATION_KEEP)
    # for i in trange(steps):
    # # while(1):
    #     robot_simulator.arm_joint_command = np.array([0.9,0.,0.,0., 0., 0.,0.,-0.9,0.,0.,-0.,0.,0.,0.]).copy()
    #     # Read user params
    #     if USE_ROS:
    #         robot_simulator.read_ROS_params()
    #     else:
    #         robot_simulator.read_user_params()
    #     robot_simulator.step()
    #     p.stepSimulation()

    #     if USE_ROS:
    #         robot_simulator.publish_ros_data()

    #     time.sleep(SIMULATION_TIME_STEP_S)

    ## ======================== Use joint space position control to move gripper ========================
    # T = 5 #seconds
    # steps = int(T/SIMULATION_TIME_STEP_S)
    # # robot_simulator.ballbot.set_barrett_hands_torque_mode()
    # joint_id = 21
    # info = p.getJointInfo(robot_simulator.ballbot.robot, joint_id)
    # jointName = info[1].decode('UTF-8')
    # jointType = info[2]
    # print('jointName', jointName)
    # for i in trange(steps):
    #     # robot_simulator.barrett_left_hand_torque_command = np.array([0.,-10.,0.,0.,0.,0.,0.,0.])
    #     # robot_simulator.barrett_right_hand_torque_command = np.array([0.,-10.,0.,0.,0.,0.,0.,0.])
    #     # Read user params
    #     if USE_ROS:
    #         robot_simulator.read_ROS_params()
    #     else:
    #         robot_simulator.read_user_params()
    #     # robot_simulator.step()
        
    #     p.setJointMotorControl2(
    #         robot_simulator.ballbot.robot, joint_id, p.VELOCITY_CONTROL, targetVelocity=1., force=1.)
    #     # p.setJointMotorControl2(
    #     #     robot_simulator.ballbot.robot, joint_id, p.POSITION_CONTROL, targetPosition=0.9, force=10.)
    #     p.stepSimulation()

    #     if USE_ROS:
    #         robot_simulator.publish_ros_data()

    #     time.sleep(SIMULATION_TIME_STEP_S)


    if LOG_VIDEO:
        robot_simulator.stop_video_log()
        print("SAVING VIDEO")

