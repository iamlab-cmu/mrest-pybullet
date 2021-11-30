"""
Class to store the state of the ballbot state to one single variable
"""
import numpy as np

class State:
  def __init__(self):
    # Body State variables
    self.xPos = 0.0
    self.yPos = 0.0
    self.xAng = 0.0
    self.yAng = 0.0
    self.yaw = 0.0
    

    # Arm state variables
    self.q_arm = {}
    self.dq_arm = {}
    self.q_arm["right"] = np.zeros(7)
    self.q_arm["left"] = np.zeros(7)
    self.dq_arm["right"] = np.zeros(7)
    self.dq_arm["left"] = np.zeros(7)

  def update_body_state(self, xAng, yAng, yaw):
    self.xAng = xAng
    self.yAng = yAng
    self.yaw = yaw

  def body_state(self):
    return [self.xAng,self.yAng, self.yaw]