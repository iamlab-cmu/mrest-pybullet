#!/usr/bin/env python
#  Copyright Microdynamic Systems Laboratory 2021
#
# @author Roberto Shu <rshum@cmu.edu>
# @author Cornelia Bauer <cornelib@cmu.edu>
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

from robot.robot_simulator import *
from robot.definitions import *
from environments.environments import CornerEnv
from tqdm import trange

# Simulation parameters
LOG_VIDEO = False
VIDEO_FILE_NAME = "ballbot_push"

if __name__ == "__main__":
    # set pybullet environment
    robot_simulator = RobotSimulator(
        startPos=[0, 1.0, 0.12], startOrientationEuler=[0, np.deg2rad(0), np.deg2rad(0)])

    """ Main Loop """
    robot_simulator.update_robot_state(BallState.OLC)
    robot_simulator.ballbot.set_arm_torque_mode()

    env = CornerEnv()
    robot_simulator.setup_environment(env)

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)

    T = 8000
    ball_des0 = [2.0,2.0]
    ballxpos0 = np.arange(0,ball_des0[0],ball_des0[0]/T)
    ballypos0 = np.arange(0,ball_des0[1],ball_des0[1]/T)
    
    while(1):
    # for i in trange(30*240):
    # from tqdm import trange
    # for _ in trange(1000):
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

    robot_simulator.update_robot_state(BallState.STATION_KEEP)
    robot_simulator.ballbot.get_ball_state()
    ballx = robot_simulator.ballbot.ballPosInWorldFrame[0]
    bally = robot_simulator.ballbot.ballPosInWorldFrame[1]
    print(f'current ball state=({ballx}, {bally})')
    ball_des0 = [ballx,bally+0.4]
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


    if LOG_VIDEO:
        robot_simulator.stop_video_log()
        print("SAVING VIDEO")
