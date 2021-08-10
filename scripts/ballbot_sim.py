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
from enum import Enum

# PACKAGE_WS_PATH =  '/home/rshu/Workspace/pybullet_ws/src/'
PACKAGE_WS_PATH =  '/usr0/home/cornelib/sandbox/bullet_ws/src/'
sys.path.insert(1, PACKAGE_WS_PATH + '/ballbot_pybullet_sim/controllers')

from definitions import  * 
from ballbot import Ballbot as ballbot_sim
from body_controller import BodyController

# Simulation parameters
SIMULATION_TIME_STEP_S = 0.01
MAX_SIMULATION_TIME_S = 10
USE_ROS = True

if USE_ROS:
  # ROS imports
  import rospy
  from ballbot_arm_msgs.msg import ArmCommand
  from ballbot_arm_msgs.msg import ArmsJointState
  from std_msgs.msg import Float64MultiArray

class BallState(Enum):
  STATIC = 1
  SAFETY_CHECK = 2
  BALANCE = 3
  OLC = 4
  STATION_KEEP = 5
  VEL_CONTROL = 6
  BALANCE_LEGS_UP = 7
  DFC=8


class RobotSimulator(object):
    def __init__(self, startPos = [0.0,0.0,0.12], startOrientationEuler=[0,0,0]):
         # set pybullet environment
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
   
        # set environment physics
        p.setGravity(0, 0, -10)

        # set ground plane
        planeId = p.loadURDF("plane.urdf")
             
        # Load ballbot model
        self.ballbot = ballbot_sim(urdf_path=PACKAGE_WS_PATH + URDF_NAME,
          startPos=startPos,startOrientationEuler=startOrientationEuler)
        self.ballbot_state = BallState.BALANCE
        #self.ballbot.print_model_info()

        self.setup_gui()
        if USE_ROS:
          self.setup_ROS()

        # Setup controller 
        self.setup_controller()
        self.read_user_params()

    def setup_gui(self):
        # set user debug parameters
        self.gravId = p.addUserDebugParameter("gravity", -10,10,-10)
        self.controller_gains = []
        self.controller_gains.append(p.addUserDebugParameter(
            "Kp", 0, 5000, 250))
        self.controller_gains.append(p.addUserDebugParameter(
            "Kd", 0, 100, 10))
        self.controller_gains.append(p.addUserDebugParameter(
            "Ki", 0, 100, 10))
        self.lean_angle = []
        self.lean_angle.append(p.addUserDebugParameter(
            "lean_angle_x", -0.2, 0.2, 0))
        self.lean_angle.append(p.addUserDebugParameter(
            "lean_angle_y", -0.2, 0.2, 0))

        self.armPosCmdId = []
        for i in range(len(self.ballbot.arm_joint_names)):
            self.armPosCmdId.append(p.addUserDebugParameter(
                self.ballbot.arm_joint_names[i].decode("utf-8"), -4, 4, 0))

    def setup_ROS(self):
        rospy.init_node('pybullet_ballbot')
        self.rarm_joint_sub = rospy.Subscriber("/ballbotArms/controller/joint_impedance/right/command", ArmCommand, self.update_rarm_cmd)
        self.rarm_joint_command = [0,0,0,0,0,0,0]
        self.larm_joint_sub = rospy.Subscriber("/ballbotArms/controller/joint_impedance/left/command", ArmCommand, self.update_larm_cmd)
        self.larm_joint_command = [0,0,0,0,0,0,0]

        self.rarm_effort_sub = rospy.Subscriber("/ballbotArms/controller/effort/right/command", Float64MultiArray, self.update_rarm_effort_cmd)
        self.rarm_effort_command = [0,0,0,0,0,0,0]
        self.larm_effort_sub = rospy.Subscriber("/ballbotArms/controller/effort/left/command", Float64MultiArray, self.update_larm_effort_cmd)
        self.larm_effort_command = [0,0,0,0,0,0,0]

        self.arms_pub =rospy.Publisher("/ballbotArms/hardware_interface/joint_states", ArmsJointState, queue_size=10)
        self.arms_msg = ArmsJointState()
        print("ROS communication initialized")

    def update_rarm_cmd(self, msg):
        self.rarm_joint_command = msg.position
    
    def update_larm_cmd(self, msg):
        self.larm_joint_command = msg.position

    def update_rarm_effort_cmd(self,msg):
        self.rarm_joint_command = msg.data

    def update_larm_effort_cmd(self,msg):
        self.larm_joint_command = msg.data

    def setup_controller(self):
        self.body_controller = BodyController()

        self.arm_joint_command = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    def update_robot_state(self, state):
      if state == BallState.STATIC:
        self.ballbot_state = BallState.STATIC
        print("Ballbot state changed to STATIC")
      elif state == BallState.BALANCE:
        if self.ballbot_state != BallState.BALANCE:
          self.ballbot_state = BallState.BALANCE
          print("Ballbot state changed to BALANCE")
      elif state == BallState.STATION_KEEP:
        if self.ballbot_state != BallState.STATION_KEEP:
          self.ballbot_state = BallState.STATION_KEEP
          print("Ballbot state changed to STATION KEEP")
      else:
        self.ballbot_state = BallState.BALANCE
        print("[ERROR] Invalid robot state, set by default to BALANCE")

    def step(self):
        # Update robot state
        self.ballbot.update_robot_state()
        body_orient_euler = self.ballbot.get_body_orientation()
        body_orient_euler_vel = self.ballbot.get_base_velocity()
        self.ballbot.get_ball_state()
        #self.body_controller.set_data(SIMULATION_TIME_STEP_S,body_orient_euler,ball_velocity)
        self.body_controller.update_com_position(self.ballbot.comPosInBodyOrient[0],self.ballbot.comPosInBodyOrient[1])
        self.body_controller.update_ball_position(self.ballbot.ballPosInBodyOrient[0], self.ballbot.ballPosInBodyOrient[1])
        self.body_controller.update_ball_velocity(self.ballbot.ballLinVelInBodyOrient[0], self.ballbot.ballLinVelInBodyOrient[1])

        """ Ball Commands """
        # Station Keeping controller 
        
        if self.ballbot_state == BallState.STATION_KEEP:
          if not self.body_controller._station_keeping_started:
            self.body_controller.set_desired_ball_position(self.ballbot.ballPosInBodyOrient[0],self.ballbot.ballPosInBodyOrient[1])
          self.body_controller.station_keep()
        else:
          self.body_controller.clear_station_keeping_error_values()

        # Balancing controller 
        if (self.ballbot_state == BallState.BALANCE or self.ballbot_state == BallState.OLC
          or self.ballbot_state == BallState.STATION_KEEP or self.ballbot_state == BallState.VEL_CONTROL):
          # If none of the outer loop controllers are running, 
          # set desired angles and linear position to zero
          if self.ballbot_state == BallState.BALANCE:
            self.body_controller.set_planned_body_angles(0,0)
            self.body_controller.set_desired_body_angles(0,0)
            self.body_controller.set_desired_ball_position(0,0)
            self.body_controller.set_desired_ball_velocity(0,0)
            self.body_controller.set_desired_world_velocity(0,0)
            self.body_controller.set_desired_com_position(self.ballbot.ballPosInBodyOrient[0],self.ballbot.ballPosInBodyOrient[1])
          # Filter feedback
          self.body_controller.balance(SIMULATION_TIME_STEP_S)
        else:
          self.body_controller.clear_balancing_error_values()
        
        # Set torque commands
        current_yy = -self.body_controller.xBallCurrent
        current_xx = self.body_controller.yBallCurrent
        torque_yy = self.body_controller.yBallTorque
        torque_xx= self.body_controller.xBallTorque

        '''
        print("torque_xx: ", torque_xx)
        print("torque_yy: ", torque_yy)
        print("current_xx: ", current_xx)
        print("current_yy: ", current_yy)
        '''

        # Apply torque to robot 
        self.ballbot.drive_arms(self.arm_joint_command)
        self.ballbot.drive_imbd(torque_xx,torque_yy)
        #self.ballbot.drive_imbd(current_xx,current_yy)
      
    def read_user_params(self):
        Kp = p.readUserDebugParameter(self.controller_gains[0])
        Kd = p.readUserDebugParameter(self.controller_gains[1])
        Ki = p.readUserDebugParameter(self.controller_gains[2])
        self.body_controller._com_balancing_control.set_gains(Kp,Ki,Kd)

        # desired lean angle
        euler_des_x = p.readUserDebugParameter(self.lean_angle[0])
        euler_des_y = p.readUserDebugParameter(self.lean_angle[1])

        # desired arm joint angles
        for i in range(len(self.ballbot.arm_joint_names)):
            self.arm_joint_command[i] = p.readUserDebugParameter(self.armPosCmdId[i])

    def read_ROS_params(self):
        self.arm_joint_command = np.concatenate((np.array(self.rarm_joint_command), np.array(self.larm_joint_command)))
    
    def publish_state(self):
        self.arms_msg.header.stamp = rospy.Time.now()
        self.arms_msg.right_arm_state.position = self.ballbot.arm_pos[0:6]
        self.arms_msg.right_arm_state.velocity = self.ballbot.arm_vel[0:6]
        self.arms_msg.left_arm_state.position = self.ballbot.arm_pos[7:]
        self.arms_msg.left_arm_state.velocity = self.ballbot.arm_vel[7:]
        self.arms_pub.publish(self.arms_msg)


if __name__ == "__main__":

  # set pybullet environment
  robot_simulator = RobotSimulator()
  SIMTYPE = 2

  """ Main Loop """
  if(SIMTYPE == 1):
    robot_simulator.step()
    while(1):
      x = 1
  elif(SIMTYPE ==2):

    while(1):
      # Read user params
      robot_simulator.read_user_params()
      if USE_ROS:
        robot_simulator.read_ROS_params()

      robot_simulator.update_robot_state(BallState.STATION_KEEP)
      robot_simulator.step()
      p.stepSimulation()

      if USE_ROS:
        robot_simulator.publish_state()

      time.sleep(SIMULATION_TIME_STEP_S)
  