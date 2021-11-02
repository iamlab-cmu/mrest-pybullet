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

from robot.robot_simulator import *
from robot.definitions import *
from environments.corner_env import *
from environments.environments import TableEnv

# Simulation parameters
LOG_VIDEO = False
VIDEO_FILE_NAME = "ballbot_grasp"

if __name__ == "__main__":
    # set pybullet environment
    robot_simulator = RobotSimulator(
        startPos=[0, 0, 0.12], startOrientationEuler=[0, 0, 0])

    """ Main Loop """
    robot_simulator.update_robot_state(BallState.OLC)
    robot_simulator.ballbot.set_arm_torque_mode()

    env = TableEnv(startPos=[1.0, 0., 0.], startOrientationEuler=[
                   0., 0., np.radians(90.)])
    robot_simulator.setup_environment(env)

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)

    while(1):
        # Read user params
        robot_simulator.read_user_params()
        if USE_ROS:
            robot_simulator.read_ROS_params()
        robot_simulator.step()
        p.stepSimulation()

        if USE_ROS:
            robot_simulator.publish_ros_data()


        time.sleep(SIMULATION_TIME_STEP_S)

    if LOG_VIDEO:
        robot_simulator.stop_video_log()
