import numpy as np
import pybullet as p
from robot.robot_simulator import *
import time

class GotoBallPosition():
    def __init__(self, robot_simulator, target_ball_pos, T):
        self.T = T
        self.steps = int(T/SIMULATION_TIME_STEP_S)
        current_ball_pos = np.array(robot_simulator.ballbot.ballPosInWorldFrame)
        self.ball_traj = np.linspace(current_ball_pos[:2], target_ball_pos, self.steps)
        self.robot_simulator = robot_simulator
        robot_simulator.update_robot_state(BallState.STATION_KEEP)
        robot_simulator.body_controller._station_keeping_control.set_gains(0.11, 0.00, -0.003, 0.11, 0.00, -0.003)
    
    def execute(self):
        for i in range(self.steps):
            self.robot_simulator.body_controller.set_desired_ball_position(self.ball_traj[i,0],self.ball_traj[i,1])
            self.robot_simulator.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)
