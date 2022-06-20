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

# Simulation parameters
LOG_VIDEO = True
VIDEO_FILE_NAME = "ballbot_grasp"

if __name__ == "__main__":
    # set pybullet environment
    robot_simulator = RobotSimulator(
        startPos=[0, 1.5, 0.12], startOrientationEuler=[0, np.deg2rad(0), np.deg2rad(0)])

    """ Main Loop """
    robot_simulator.update_robot_state(BallState.OLC)
    robot_simulator.ballbot.set_arm_torque_mode()

    env = CornerEnv()
    robot_simulator.setup_environment(env)

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)

    # while(1):
    from tqdm import trange
    for _ in trange(1000):
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
