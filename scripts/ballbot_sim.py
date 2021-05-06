# Copyright Roberto Shu 2021
#
# @brief Python script to launch a ballbot simulation in pybuller
#

import pybullet as p
import json
import time
import pybullet_data
import numpy as np

from dataclasses import dataclass

@dataclass
class Joint:
  index: int
  name: str
  type: int
  gIndex: int
  uIndex: int
  flags: int
  damping: float
  friction: float
  lowerLimit: float
  upperLimit: float
  maxForce: float
  maxVelocity: float
  linkName: str
  axis: tuple
  parentFramePosition: tuple
  parentFrameOrientation: tuple
  parentIndex: int

  def __post_init__(self):
    self.name = str(self.name, 'utf-8')
    self.linkName = str(self.linkName, 'utf-8')


# Connect GUI interface
useGUI = True
if useGUI:
  p.connect(p.GUI)
else:
  p.connect(p.DIRECT)


# Setup PyBullet environment
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.81)
p.setTimeOut(10000)
#p.setAdditionalSearchPath()
p.setPhysicsEngineParameter(numSolverIterations=30)
rate_sec = 1./240. # Control/Sim rate

# Load Ballbot URDF
package_path = "/home/rshu/Workspace/pybullet_ws/src/ballbot_arm_description"
urdf_path = package_path + "/robots/urdf/ballbot_plus_pybullet.urdf"
flags=p.URDF_MAINTAIN_LINK_ORDER+p.URDF_USE_SELF_COLLISION

startOrientation = p.getQuaternionFromEuler([0,np.deg2rad(0),0])
startLocation = [0,0,0]
ballbot = p.loadURDF(urdf_path,startLocation, startOrientation,
                      globalScaling=0.25,
                      useFixedBase=False,
                      flags=flags)

# Print Ballbot joint info
print(f"ballbot unique ID: {ballbot}")
for i in range(p.getNumJoints(ballbot)):
  joint = Joint(*p.getJointInfo(ballbot, i))
  print(joint)

# Add control to joints
maxForce = 0
targetVel = 1000
# joint = [-Y, -X,0]
p.setJointMotorControlMultiDof(ballbot,
  1, 
  controlMode = p.TORQUE_CONTROL, 
  targetVelocity=[targetVel,0,0], 
  force=[0, 1000, 0])

# Main Loop
while(p.isConnected()):
  imu_position, imu_orientation = p.getBasePositionAndOrientation(ballbot)
  print(f"imu_orientation: {imu_orientation}")
  p.stepSimulation()
  time.sleep(rate_sec)
