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
from robot.utils import *
from environments.environments import TableEnv

# Simulation parameters
LOG_VIDEO = False
VIDEO_FILE_NAME = "ballbot_test_station_keep"

if __name__ == "__main__":
    # set pybullet environment
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    # self.plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin") #without this (https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=13395) issue arises

    # set environment physics
    p.setGravity(0, 0, -10)

    # set ground plane
    planeId = p.loadURDF("plane.urdf")
    startPos=[0.0, 0.0, 0.12]
    startOrientationEuler=[0, 0, 0]
    startOrientationEulerStandardFrame = convertEulerBBToStandardFrame(
            startOrientationEuler)
    startOrientation = p.getQuaternionFromEuler(
        startOrientationEulerStandardFrame)
    robot = p.loadURDF('/home/saumyas/ballbot_sim_py3_ws/src/ballbot_arm_description/robots/urdf/ballbot_pybullet_wBarrettHands.urdf', startPos,
                                startOrientation, useFixedBase=True)
    # robot = p.loadURDF('/home/saumyas/ballbot_sim_py3_ws/src/barrett_hand_common/barrett_hand_description/robots/barrett_alone.urdf', startPos,
    #                             startOrientation, useFixedBase=True)

    # p.resetBasePositionAndOrientation(robot, startPos, startOrientation)
    
    joint_id = 0
    info = p.getJointInfo(robot, joint_id)
    jointName = info[1].decode('UTF-8')
    jointType = info[2]
    print('jointName', jointName)
    # drive_links_ids = [20, 21, 22, 23, 24, 25, 26, 27, 38, 39, 40, 41, 42, 43, 44, 45]
    drive_links_ids = [21]
    # drive_links_ids = [3, 4, 5, 6, 7, 8, 9, 10]
    while(1):
        for link_id in drive_links_ids:
            p.setJointMotorControl2(
                robot, link_id, p.VELOCITY_CONTROL, targetVelocity=0.5, force=10.)
            # p.setJointMotorControl2(
            #     robot, joint_id, p.POSITION_CONTROL, targetPosition=1., force=100.)
        p.stepSimulation()

        time.sleep(SIMULATION_TIME_STEP_S)

    # p.unloadPlugin(robot_simulator.plugin)
