#!/usr/bin/env python
#  Copyright Microdynamic Systems Laboratory 2021
#
# @author Cornelia Bauer <cornelib@andrew.cmu.edu>
# @author Roberto Shu <rshum@cmu.edu>
#
# @brief Python script to launch a ballbot simulation in pybullet
#

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

# Simulation parameters
LOG_VIDEO = True
VIDEO_FILE_NAME = "ballbot_tsc_oppositereach"

if __name__ == "__main__":
    # set pybullet environment
    robot_simulator = RobotSimulator(
        startPos=[0, 0.0, 0.12], startOrientationEuler=[0, np.deg2rad(0), 0])

    """ Main Loop """
    robot_simulator.update_robot_state(BallState.OLC)
    
    robot_simulator.ballbot.set_arm_torque_mode()

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)

    T = 4000
    ball_des0 = [0.0,0.0]
    # ballxpos0 = np.arange(0,ball_des0[0],ball_des0[0]/T)
    ballypos0 = np.arange(1.0,ball_des0[1],-1/T)

    ball_des1 = [-0.5,0.5]
    ballxpos1 = np.arange(ball_des0[0],ball_des1[0],(ball_des1[0]-ball_des0[0])/T)
    # ballypos1 = np.arange(ball_des0[1],ball_des1[1],(ball_des1[1]-ball_des0[1])/T)
    ballypos1 = np.array([0.5]*len(ballxpos1))
    # ballxpos1 = np.array([0.5]*len(ballypos1))

    ball_des2 = [0.,0.]
    ballxpos2 = np.arange(ball_des1[0],ball_des2[0],(ball_des2[0]-ball_des1[0])/T)
    ballypos2 = np.arange(ball_des1[1],ball_des2[1],(ball_des2[1]-ball_des1[1])/T)
    
    
    for i in trange(4000):
    # while(True):
        # robot_simulator.body_controller.set_desired_ball_position(0.,ballypos0[i])
        # # Read user params
        # if i < T:
        #     robot_simulator.body_controller.set_desired_ball_position(ballxpos0[i],ballypos0[i])
        # elif i >= T and i < 2*T:
        #     robot_simulator.body_controller.set_desired_ball_position(ballxpos1[i-T],ballypos1[i-T])
        # else:
        #     robot_simulator.body_controller.set_desired_ball_position(ballxpos2[i-2*T],ballypos2[i-2*T])

        if USE_ROS:
            robot_simulator.read_ROS_params()
        else:
            robot_simulator.read_user_params()
        robot_simulator.step()
        p.stepSimulation()
        # print(robot_simulator.ballbot.ball_state[0])

        if USE_ROS:
            robot_simulator.publish_ros_data()

        time.sleep(SIMULATION_TIME_STEP_S)
    
    if LOG_VIDEO:
        robot_simulator.stop_video_log()
        print("SAVING VIDEO")
    import ipdb; ipdb.set_trace()
    robot_simulator.update_robot_state(BallState.STATION_KEEP)
    T = 3000
    robot_simulator.ballbot.get_ball_state()
    ballx = robot_simulator.ballbot.ballPosInWorldFrame[0]
    bally = robot_simulator.ballbot.ballPosInWorldFrame[1]
    print(f'current ball state=({ballx}, {bally})')
    ball_des0 = [ballx,bally+0.2]
    print(f"ball_des0", ball_des0)
    ballxpos0 = np.linspace(ballx,ball_des0[0],T)
    ballypos0 = np.linspace(bally,ball_des0[1],T)

    for i in trange(T):
        robot_simulator.body_controller.set_desired_ball_position(ballxpos0[i],ballypos0[i])
        # Read user params
        if USE_ROS:
            robot_simulator.read_ROS_params()
        else:
            robot_simulator.read_user_params()
        robot_simulator.step()
        p.stepSimulation()

        if USE_ROS:
            robot_simulator.publish_ros_data()

        time.sleep(SIMULATION_TIME_STEP_S)

    for i in trange(30*240):
    # while(True):
        # Read user params
        # if i < T:
        #     robot_simulator.body_controller.set_desired_ball_position(ballxpos0[i],ballypos0[i])
        # elif i >= T and i < 2*T:
        #     robot_simulator.body_controller.set_desired_ball_position(ballxpos1[i-T],ballypos1[i-T])
        # else:
        #     robot_simulator.body_controller.set_desired_ball_position(ballxpos2[i-2*T],ballypos2[i-2*T])

        if USE_ROS:
            robot_simulator.read_ROS_params()
        else:
            robot_simulator.read_user_params()
        robot_simulator.step()
        p.stepSimulation()
        # print(robot_simulator.ballbot.ball_state[0])

        if USE_ROS:
            robot_simulator.publish_ros_data()

        time.sleep(SIMULATION_TIME_STEP_S)

    if LOG_VIDEO:
        robot_simulator.stop_video_log()
        print("SAVING VIDEO")

    p.disconnect()