import os
import pybullet as p
import time
import pybullet_data
import math
import numpy as np


physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.12]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load Ballbot URDF
# path/to/ballbot_arm_description
package_path = "/usr0/home/cornelib/sandbox/ballbot_ws/src/ballbot_arm_description"
urdf_path = package_path + "/robots/urdf/ballbot_plus_pybullet.urdf"
ballbot = p.loadURDF(urdf_path, startPos,
                     startOrientation, useFixedBase=False)

# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

controller_gains = []
controller_gains.append(p.addUserDebugParameter(
    "Kp", 0, 5000, 1000))
controller_gains.append(p.addUserDebugParameter(
    "Kd", 0, 1000, 0))
# controller_gains.append(p.addUserDebugParameter(
#     "Ki", 0, 1000, 0))
lean_angle = []
lean_angle.append(p.addUserDebugParameter(
    "lean_angle_x", -0.2, 0.2, 0))
lean_angle.append(p.addUserDebugParameter(
    "lean_angle_y", -0.2, 0.2, 0))

p.changeDynamics(ballbot, -1, linearDamping=0, angularDamping=0)
jointIds = []
paramIds = []
paramIds_ball = []
gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
for j in range(p.getNumJoints(ballbot)):
    p.changeDynamics(ballbot, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(ballbot, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        jointIds.append(j)
        paramIds.append(p.addUserDebugParameter(
            jointName.decode("utf-8"), -4, 4, 0))
    # if (jointType == p.JOINT_SPHERICAL):
    #     paramIds_ball.append(p.addUserDebugParameter(
    #         jointName.decode("utf-8"), -10, 10, 0))
    #     paramIds_ball.append(p.addUserDebugParameter(
    #         jointName.decode("utf-8"), -10, 10, 0))

p.changeDynamics(ballbot, 0, linearDamping=0.5, angularDamping=0.5)

# i gain (optional)
# err_xi = 0
# err_yi = 0
# p.setRealTimeSimulation(1)
while (1):
    p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(
            ballbot, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)

    # Balancing Controller
    Kp = p.readUserDebugParameter(controller_gains[0])
    Kd = p.readUserDebugParameter(controller_gains[1])
    # Ki = p.readUserDebugParameter(controller_gains[2])

    euler_des_x = p.readUserDebugParameter(lean_angle[0])
    euler_des_y = p.readUserDebugParameter(lean_angle[1])

    imu_position, imu_orientation = p.getBasePositionAndOrientation(ballbot)
    imu_euler = p.getEulerFromQuaternion(imu_orientation)

    linear, angular = p.getBaseVelocity(ballbot)
    print(imu_euler)

    # err_xi += imu_euler[0]/100
    # err_xi = np.clip(err_xi, -10, 10)
    # err_yi += imu_euler[0]/100
    # err_yi = np.clip(err_yi, -10, 10)
    torqueXX = Kp * (imu_euler[0]-euler_des_x) + \
        Kd*(-angular[0])  # + Ki*err_xi
    torqueYY = Kp * (imu_euler[1]-euler_des_y) + \
        Kd*(-angular[1])  # + Ki*err_yi

    ball_state = p.getLinkState(ballbot, 0)
    orient = ball_state[1]
    rot_mat = np.reshape(
        p.getMatrixFromQuaternion(orient),
        (3, 3))
    control_torque = np.matmul(rot_mat.transpose(), np.array(
        [torqueXX, torqueYY, 0.0]).reshape(3, 1))

    p.setJointMotorControlMultiDof(ballbot,
                                   0,
                                   controlMode=p.POSITION_CONTROL,
                                   targetPosition=[0, 0, 0, 1],
                                   force=[0, 0, 0])
    p.setJointMotorControlMultiDof(ballbot,
                                   0,
                                   controlMode=p.TORQUE_CONTROL,
                                   force=control_torque)
    p.stepSimulation()
    time.sleep(0.01)


p.disconnect()
