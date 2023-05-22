import numpy as np
import pybullet as p
from robot.robot_simulator import *
import time
from tqdm import trange

class OpenGripper():
    def __init__(self, robot_simulator, T=2, hands='both'):
        self.T = T
        self.steps = int(T/SIMULATION_TIME_STEP_S)
        self.robot_simulator = robot_simulator
        self.robot_simulator.ballbot.set_barrett_hands_position_mode()
        # self.robot_simulator.ballbot.set_barrett_hands_torque_mode()
        self.control_left_hand = True if (hands=='both' or hands == 'left') else False
        self.control_right_hand = True if (hands=='both' or hands == 'right') else False

        current_left_q = np.array(robot_simulator.ballbot.barrett_left_hand_pos)
        current_right_q = np.array(robot_simulator.ballbot.barrett_right_hand_pos)

        if self.control_left_hand:
            target_left_q = np.array([0., 0., 0., 0., 0., 0., 0., 0.])
            self.left_hand_traj = np.linspace(current_left_q, target_left_q, self.steps)
            self.barrett_left_hand_controller.set_gains([[3,0,0.1]]*self.barrett_left_hand_controller.nJoints)
        
        if self.control_right_hand:
            target_right_q = np.array([0., 0., 0., 0., 0., 0., 0., 0.])
            self.right_hand_traj = np.linspace(current_right_q, target_right_q, self.steps)
            self.barrett_right_hand_controller.set_gains([[3,0,0.1]]*self.barrett_right_hand_controller.nJoints)

    def execute(self):
        for i in trange(self.steps):
            if self.control_left_hand:
                self.robot_simulator.barrett_left_hand_joint_command = self.left_hand_traj[i].copy()
            
            if self.control_right_hand:
                self.robot_simulator.barrett_right_hand_joint_command = self.right_hand_traj[i].copy()

            # self.robot_simulator.barrett_left_hand_torque_command = np.array([0.,-10.,0.,0.,0.,0.,0.,0.])
            # self.robot_simulator.barrett_right_hand_torque_command = np.array([0.,-10.,0.,0.,0.,0.,0.,0.])

            self.robot_simulator.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)


class CloseGripper():
    def __init__(self, robot_simulator, T=2, hands='both'):
        self.T = T
        self.steps = int(T/SIMULATION_TIME_STEP_S)
        self.robot_simulator = robot_simulator
        self.robot_simulator.ballbot.set_barrett_hands_position_mode()

        self.control_left_hand = True if (hands=='both' or hands == 'left') else False
        self.control_right_hand = True if (hands=='both' or hands == 'right') else False

        current_left_q = np.array(robot_simulator.ballbot.barrett_left_hand_pos)
        current_right_q = np.array(robot_simulator.ballbot.barrett_right_hand_pos)

        if self.control_left_hand:
            target_left_q = np.array([1.47, 0., 0., 1.47, 0., 0., 1.47, 0.])
            self.left_hand_traj = np.linspace(current_left_q, target_left_q, self.steps)
        
        if self.control_right_hand:
            target_right_q = np.array([1.47, 0., 0., 1.47, 0., 0., 1.47, 0.])
            self.right_hand_traj = np.linspace(current_right_q, target_right_q, self.steps)
    
    
    def execute(self):
        for i in trange(self.steps):
            if self.control_left_hand:
                self.robot_simulator.barrett_left_hand_joint_command = self.left_hand_traj[i].copy()
            
            if self.control_right_hand:
                self.robot_simulator.barrett_right_hand_joint_command = self.right_hand_traj[i].copy()

            self.robot_simulator.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)