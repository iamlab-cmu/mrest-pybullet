import os
import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray


def drawInertiaBox(parentUid, parentLinkIndex, color):
    dyn = p.getDynamicsInfo(parentUid, parentLinkIndex)
    mass = dyn[0]
    frictionCoeff = dyn[1]
    inertia = dyn[2]
    if (mass > 0):
        Ixx = inertia[0]
        Iyy = inertia[1]
        Izz = inertia[2]
        boxScaleX = 0.5 * math.sqrt(6 * (Izz + Iyy - Ixx) / mass)
        boxScaleY = 0.5 * math.sqrt(6 * (Izz + Ixx - Iyy) / mass)
        boxScaleZ = 0.5 * math.sqrt(6 * (Ixx + Iyy - Izz) / mass)

        halfExtents = [boxScaleX, boxScaleY, boxScaleZ]
        pts = [[halfExtents[0], halfExtents[1], halfExtents[2]],
               [-halfExtents[0], halfExtents[1], halfExtents[2]],
               [halfExtents[0], -halfExtents[1], halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], halfExtents[2]],
               [halfExtents[0], halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], halfExtents[1], -halfExtents[2]],
               [halfExtents[0], -halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], -halfExtents[2]]]

        p.addUserDebugLine(pts[0],
                           pts[1],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[1],
                           pts[3],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[3],
                           pts[2],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[2],
                           pts[0],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)

        p.addUserDebugLine(pts[0],
                           pts[4],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[1],
                           pts[5],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[2],
                           pts[6],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[3],
                           pts[7],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)

        p.addUserDebugLine(pts[4 + 0],
                           pts[4 + 1],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 1],
                           pts[4 + 3],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 3],
                           pts[4 + 2],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 2],
                           pts[4 + 0],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)


def computeCOMposVel(uid: int):
    """Compute center-of-mass position and velocity."""
    jointIndices = range(nJoints)
    link_states = p.getLinkStates(uid, jointIndices, computeLinkVelocity=1)
    link_pos = np.array([s[0] for s in link_states])
    link_vel = np.array([s[-2] for s in link_states])
    tot_mass = 0.
    masses = []
    for j in jointIndices:
        mass_, *_ = p.getDynamicsInfo(uid, j)
        masses.append(mass_)
        tot_mass += mass_

    # add base position and velocity (Link_Body, id -1)
    body_mass, *_ = p.getDynamicsInfo(uid, -1)
    tot_mass += body_mass

    body_position, body_orientation = p.getBasePositionAndOrientation(uid)
    body_vel_linear, body_vel_angular = p.getBaseVelocity(uid)

    masses = np.asarray(masses)[:, None]
    com_pos = np.sum(masses * link_pos, axis=0) / tot_mass
    com_pos += body_mass * np.asarray(body_position) / tot_mass
    com_vel = np.sum(masses * link_vel, axis=0) / tot_mass
    com_vel += body_mass * np.asarray(body_vel_linear) / tot_mass
    return com_pos, com_vel


rospy.init_node("lean_angle_pub")

COM_pub = pub = rospy.Publisher(
    '/COM', Float64MultiArray, queue_size=1, latch=True)
COM_msg = Float64MultiArray()
pub = rospy.Publisher(
    '/lean_angle', Float64MultiArray, queue_size=1, latch=True)
lean_angle_msg = Float64MultiArray()

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


controller_gains = []
controller_gains.append(p.addUserDebugParameter(
    "Kp", 0, 5000, 250))
controller_gains.append(p.addUserDebugParameter(
    "Kd", 0, 1000, 1))
controller_gains.append(p.addUserDebugParameter(
    "Ki", 0, 1000, 1))
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
nJoints = p.getNumJoints(ballbot)
for j in range(p.getNumJoints(ballbot)):
    p.changeDynamics(ballbot, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(ballbot, j)
    # print(info)
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

# COM
# linkIndicesSim = range(p.getNumJoints(ballbot))
# ls = p.getLinkStates(
#     ballbot, linkIndicesSim, computeForwardKinematics=True)
com_pos_init, com_vel_init = computeCOMposVel(ballbot)
drawInertiaBox(ballbot, -1, [1, 0, 0])
# drawInertiaBox(quadruped,motor_front_rightR_joint, [1,0,0])

for i in range(nJoints):
    drawInertiaBox(ballbot, i, [0, 1, 0])

# i gain (optional)
err_xi = 0
err_yi = 0
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
    Ki = p.readUserDebugParameter(controller_gains[2])

    # desired lean angle
    euler_des_x = p.readUserDebugParameter(lean_angle[0])
    euler_des_y = p.readUserDebugParameter(lean_angle[1])
    # current lean angle
    imu_position, imu_orientation = p.getBasePositionAndOrientation(ballbot)
    imu_euler = p.getEulerFromQuaternion(imu_orientation)

    linear, angular = p.getBaseVelocity(ballbot)
    # print(imu_euler)
    # get COM error
    ball_state = p.getLinkState(ballbot, 0)
    com_pos, com_vel = computeCOMposVel(ballbot)
    err_x = com_pos[0]-ball_state[0][0]
    err_y = com_pos[1]-ball_state[0][1]
    # print(ball_state[0][0])
    err_xi += err_x/100
    err_xi = np.clip(err_xi, -10, 10)
    err_yi += err_y/100
    err_yi = np.clip(err_yi, -10, 10)

    torqueXX = - Kp * err_y - \
        Kd*(com_vel[1]) - Ki*err_yi
    torqueXX = np.clip(torqueXX, -20, 20)
    torqueYY = Kp * err_x + \
        Kd*(com_vel[0]) + Ki*err_xi
    torqueYY = np.clip(torqueYY, -20, 20)

    control_torque = [torqueXX, torqueYY, 0.0]
    ball_orient = ball_state[1]
    ball_orient_inv = p.invertTransform([0, 0, 0], ball_orient)
    control_torque = p.rotateVector(
        ball_orient_inv[1], np.array(control_torque).reshape(3, 1))

    p.setJointMotorControlMultiDof(ballbot,
                                   0,
                                   controlMode=p.POSITION_CONTROL,
                                   targetPosition=[0, 0, 0, 1],
                                   force=[0, 0, 0])
    p.setJointMotorControlMultiDof(ballbot,
                                   0,
                                   controlMode=p.TORQUE_CONTROL,
                                   force=control_torque)

    COM_msg.data = [com_pos[0], com_pos[1], com_pos[2],
                    control_torque[0], control_torque[1], control_torque[2]]
    lean_angle_msg.data = [euler_des_x,
                           imu_euler[0], euler_des_y, imu_euler[1]]
    COM_pub.publish(COM_msg)
    pub.publish(lean_angle_msg)

    p.stepSimulation()
    time.sleep(0.01)


p.disconnect()
