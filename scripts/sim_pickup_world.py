import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import time
from datetime import datetime
import numpy as np
from enum import Enum
from tqdm import trange

from robot.robot_simulator import *
from robot.definitions import *
from environments.environments import TableEnv
from skills.arm_skills import GotoJointPosition, GotoTaskSpacePose
from skills.gripper_skills import OpenGripper, CloseGripper
from skills.body_skills import GotoBallPosition

# Simulation parameters
LOG_VIDEO = False
VIDEO_FILE_NAME = "ballbot_pickup_task"

if __name__ == "__main__":
    # set pybullet environment
    # init_arm_joint_position = np.array([0.,0.,0.,0.9, 0., 0.,0.,0.,0.,0.,-0.9,0.,0.,0.])
    # init_arm_joint_position = np.array([0.9,0.,0.,0., 0., 0.,0.,-0.9,0.,0.,-0.,0.,0.,0.])
    init_arm_joint_position = np.array([0.,0.,0.,0.0,0.,0.,0.,0.,0.,0.,-0.0,0.,0.,0.])
    robot_simulator = RobotSimulator(
        startPos=[0, 0., 0.106], startOrientationEuler=[0, 0, 0], 
        init_arm_joint_position=init_arm_joint_position,
        init_left_gripper_state= 'close',
        init_right_gripper_state='open',
        )
        
    robot_simulator.ballbot.set_arm_torque_mode()

    env = TableEnv(startPos=[0.0, 1.3, 0.], startOrientationEuler=[
                   0., 0., np.radians(0.)])
    robot_simulator.setup_environment(env)

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)
    
    robot_simulator.ballbot.update_robot_state()
    
    # T = 5 #seconds
    # left_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolL'])
    # right_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolR'])
    # target_left_pose = np.array(left_ee_frame_info[0]) + np.array([0.0, 1.5, 0.2])
    # target_right_pose = np.array(right_ee_frame_info[0]) + np.array([1.0, 1.5, 0.2])
    # skill = GotoTaskSpacePose(robot_simulator, target_left_pose, target_right_pose, T, arms='left', enable_base=True)
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



    # Pickup object using task space control

    # Reach object
    for link_name in BARRETT_RIGHT_LINK_NAMES:
        p.changeDynamics(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds[link_name], lateralFriction=1.7)

    T = 15 # seconds
    cube_pos, cube_quat = p.getBasePositionAndOrientation(robot_simulator.environment.cubeId2)
    cube_pos = np.array(cube_pos)
    cube_pos[2] = cube_pos[2] + 0.03
    skill = GotoTaskSpacePose(robot_simulator, [], np.array(cube_pos), T, arms='right', enable_base=True)
    skill.execute()
    
    # Close gripper
    T = 8 #seconds  
    skill = CloseGripper(robot_simulator, T, hands='right')
    skill.execute()

    # # Create fixed link between gripper and object
    # cube_pos, cube_quat = p.getBasePositionAndOrientation(robot_simulator.environment.cubeId2)
    # constraint_id = p.createConstraint(robot_simulator.ballbot.robot,
    #                                 robot_simulator.ballbot.linkIds['R_bh_base_link'], 
    #                                 robot_simulator.environment.cubeId2,
    #                                 -1,
    #                                 p.JOINT_FIXED,
    #                                 jointAxis=[0, 0, 1],
    #                                 parentFramePosition=[0, 0, 0],
    #                                 childFramePosition=[0, 0, 0])
    # p.changeConstraint(constraint_id, maxForce=100,erp=0.2)

    T = 5 # seconds
    cube_pos, cube_quat = p.getBasePositionAndOrientation(robot_simulator.environment.cubeId2)
    cube_pos = np.array(cube_pos)
    cube_pos[2] = cube_pos[2] + 0.2
    skill = GotoTaskSpacePose(robot_simulator, [], np.array(cube_pos), T, arms='right', enable_base=True)
    skill.execute()

    # # Move near table using station keeping control
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

    # # Use joint space position control to move arms
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

    # # Use joint space position control to move gripper
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

