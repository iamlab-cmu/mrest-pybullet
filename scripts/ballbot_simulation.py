# Copyright Roberto Shu 2021
#
# @brief Python script to launch a ballbot simulation in pybuller
#

import os
import pybullet as p
import pybullet_data
import time 
import math
import numpy as np
import sys

PACKAGE_WS_PATH =  '/home/rshu/Workspace/pybullet_ws/src/'
sys.path.insert(1, PACKAGE_WS_PATH + '/ballbot_pybullet_sim/controllers')

from definitions import  * 
from ballbot import Ballbot as ballbot_sim
from balancing_controller import COMBalancingController



# Simulation parameters
SIMULATION_TIME_STEP_S = 0.01
MAX_SIMULATION_TIME_S = 10
USE_ROS = False

if __name__ == "__main__":

    # set pybullet environment
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
   
    # set environment physics
    p.setGravity(0, 0, -10)

    # set ground plane
    planeId = p.loadURDF("plane.urdf")

    # load ballbot model
    ballbot = ballbot_sim(urdf_path=PACKAGE_WS_PATH + URDF_NAME)

    # set user debug parameters
    gravId = p.addUserDebugParameter("gravity", -10,10,-10)
    controller_gains = []
    controller_gains.append(p.addUserDebugParameter(
        "Kp", 0, 5000, 250))
    controller_gains.append(p.addUserDebugParameter(
        "Kd", 0, 100, 10))
    controller_gains.append(p.addUserDebugParameter(
        "Ki", 0, 100, 10))
    lean_angle = []
    lean_angle.append(p.addUserDebugParameter(
        "lean_angle_x", -0.2, 0.2, 0))
    lean_angle.append(p.addUserDebugParameter(
        "lean_angle_y", -0.2, 0.2, 0))
    
    armPosCmdId = []
    for i in range(len(ballbot.arm_joint_names)):
        armPosCmdId.append(p.addUserDebugParameter(
            ballbot.arm_joint_names[i].decode("utf-8"), -4, 4, 0))
    
    arm_joint_command = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    # setup controllers
    balancing_controller = COMBalancingController()
    balancing_controller.set_max_torque(100)
    
    """ Main Loop """
    while(1):
        ##sim_env.step()
        
        # balancing controller
        Kp = p.readUserDebugParameter(controller_gains[0])
        Kd = p.readUserDebugParameter(controller_gains[1])
        Ki = p.readUserDebugParameter(controller_gains[2])

        # desired lean angle
        euler_des_x = p.readUserDebugParameter(lean_angle[0])
        euler_des_y = p.readUserDebugParameter(lean_angle[1])

        # desired arm joint angles
        for i in range(len(ballbot.arm_joint_names)):
            arm_joint_command[i] = p.readUserDebugParameter(armPosCmdId[i])
        
        # Update robot state
        ballbot.update_robot_state()

        # Calculate controller output
        balancing_controller.set_gains(Kp,Ki,Kd)
        balancing_controller.calculate_error_value(
            ballbot.com_pos[0],ballbot.ball_state[0][0],
            ballbot.com_pos[1],ballbot.ball_state[0][1],
            SIMULATION_TIME_STEP_S)
        balancing_controller.get_torque_output()


        # Set torque commands
        torque_yy = -balancing_controller.torque_x_nm
        torque_xx = balancing_controller.torque_y_nm

        # Apply torque to robot 
        ballbot.drive_arms(arm_joint_command)
        ballbot.drive_imbd(torque_xx,torque_yy)

        p.stepSimulation()
        time.sleep(SIMULATION_TIME_STEP_S)

