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
from environments.environments import TableEnv

# Simulation parameters
LOG_VIDEO = True
VIDEO_FILE_NAME = "ballbot_test_station_keep"

if __name__ == "__main__":
    # set pybullet environment
    robot_simulator = RobotSimulator(
        startPos=[0, 0, 0.12], startOrientationEuler=[0, 0, 0])

    test_type = "TEST_STATION_KEEP"

    """ Main Loop """
    if test_type == "TEST_STATION_KEEP":
        robot_simulator.update_robot_state(BallState.STATION_KEEP)
    else:
        robot_simulator.update_robot_state(BallState.OLC)
        
    robot_simulator.ballbot.set_arm_torque_mode()

    env = TableEnv(startPos=[0.0, 1., 0.], startOrientationEuler=[
                   0., 0., np.radians(0.)])
    robot_simulator.setup_environment(env)

    if LOG_VIDEO:
        robot_simulator.start_video_log(VIDEO_FILE_NAME)

    
    # while(1):
    from tqdm import trange
    for i in trange(500):
        if test_type == "TEST_STATION_KEEP":
            robot_simulator.body_controller.set_desired_ball_position(1,1)
        if test_type == "TEST_TURRET":
            if i < 200:
                robot_simulator.turret_controller.set_desired_angles([0.0,1.57])
            elif i >= 200 and i < 400:
                robot_simulator.turret_controller.set_desired_angles([0.0,1.])
            elif i >= 400 and i < 600:
                robot_simulator.turret_controller.set_desired_angles([2,1.])
            else:
                robot_simulator.turret_controller.set_desired_angles([-2,1.])

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
        print("STOP LOGGING VIDEO")
    
    p.unloadPlugin(robot_simulator.plugin)
