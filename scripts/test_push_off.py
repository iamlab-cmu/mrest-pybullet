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
from environments.environments import CornerEnv

# Simulation parameters
LOG_VIDEO = False
VIDEO_FILE_NAME = "ballbot_test_pushoff"

if __name__ == "__main__":
    # set pybullet environment
    robot_simulator = RobotSimulator(
        startPos=[0, 1.0, 0.12], startOrientationEuler=[0, np.deg2rad(0), 0])

    """ Main Loop """
    robot_simulator.update_robot_state(BallState.OLC)
    
    robot_simulator.ballbot.set_arm_torque_mode()

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)
    
    env = CornerEnv()
    robot_simulator.setup_environment(env)
    
    # MOVE ARMS
    for i in trange(15*240):
        if USE_ROS:
            robot_simulator.read_ROS_params()
        else:
            robot_simulator.read_user_params()
        robot_simulator.step()
        p.stepSimulation()

        if USE_ROS:
            robot_simulator.publish_ros_data()

        time.sleep(SIMULATION_TIME_STEP_S)
    

    # MOVE BALL
    robot_simulator.update_robot_state(BallState.STATION_KEEP)
    T = 1000
    robot_simulator.ballbot.get_ball_state()
    ballx = robot_simulator.ballbot.ballPosInWorldFrame[0]
    bally = robot_simulator.ballbot.ballPosInWorldFrame[1]
    print(f'current ball state=({ballx}, {bally})')
    ball_des0 = [ballx,bally+0.5]
    print(f"ball_des0", ball_des0)
    ballxpos0 = np.linspace(ballx,ball_des0[0],T)
    ballypos0 = np.linspace(bally,ball_des0[1],T)
    robot_simulator.body_controller._station_keeping_control.set_gains(0.2, 0, -0.01, 0.2, 0, -0.001)

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

    # DO NOTHING
    for i in trange(20*240):
    # while(True):
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

    # PUSH OFF WALL BY EXTENDING ARMS
    robot_simulator.ballbot.set_arm_position_mode()

    T = 2000
    robot_simulator.ballbot.get_ball_state()
    ballx = robot_simulator.ballbot.ballPosInWorldFrame[0]
    bally = robot_simulator.ballbot.ballPosInWorldFrame[1]
    print(f'current ball state=({ballx}, {bally})')
    ball_des0 = [ballx,bally-0.5]
    print(f"ball_des0", ball_des0)
    ballxpos0 = np.linspace(ballx,ball_des0[0],T)
    ballypos0 = np.linspace(bally,ball_des0[1],T)
    robot_simulator.body_controller._station_keeping_control.set_gains(0.2, 0, -0.01, 0.2, 0, -0.001)

    for i in trange(T):
    # while(True):
        robot_simulator.body_controller.set_desired_ball_position(ballxpos0[i],ballypos0[i])
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