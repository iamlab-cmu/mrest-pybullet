
import os
import pybullet as p
import time
import pybullet_data
import math
import numpy as np

from utils import drawInertiaBox, computeCOMposVel

sim_dt = 0.01

use_ROS = True


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
    "Kd", 0, 100, 10))
controller_gains.append(p.addUserDebugParameter(
    "Ki", 0, 100, 10))
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


p.changeDynamics(ballbot, 0, linearDamping=0.5, angularDamping=0.5)

# COM
com_pos_init, com_vel_init = computeCOMposVel(p, ballbot)
drawInertiaBox(p, ballbot, -1, [1, 0, 0])

for i in range(nJoints):
    drawInertiaBox(p, ballbot, i, [0, 1, 0])

# i gain (optional)
err_xi = 0
err_yi = 0
# d gain
err_x_prev = 0
err_y_prev = 0
# p.setRealTimeSimulation(1)

if use_ROS:
    import rospy
    from std_msgs.msg import Float64MultiArray, Header
    from sensor_msgs.msg import JointState

    def cb_right_arm(data):
        targetPos = data.data
        for i in range(7):
            c = paramIds[i]
            # targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(
                ballbot, jointIds[i], p.POSITION_CONTROL, targetPos[i], force=5 * 240.)

    rospy.init_node("ballbot_pybullet")
    #
    COM_pub = rospy.Publisher(
        '/COM', Float64MultiArray, queue_size=1, latch=True)
    COM_msg = Float64MultiArray()

    lean_angle_pub = rospy.Publisher(
        'joint_states/lean_angle', JointState, queue_size=1)
    lean_angle_msg = JointState()
    lean_angle_msg.header = Header()
    lean_angle_msg.name = ['xAng',	'yAng']

    right_arm_sub = rospy.Subscriber(
        "arm_right_cmd", Float64MultiArray, cb_right_arm)

while (1):
    p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    if not use_ROS:
        # move arm with sliders
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
    com_pos, com_vel = computeCOMposVel(p, ballbot)

    # P
    err_x = com_pos[0]-ball_state[0][0]
    err_y = com_pos[1]-ball_state[0][1]

    # D
    err_xd = (err_x - err_x_prev)/sim_dt
    err_yd = (err_y - err_y_prev)/sim_dt
    err_x_prev = err_x
    err_y_prev = err_y

    # I
    err_xi += err_x/100
    err_xi = np.clip(err_xi, -10, 10)
    err_yi += err_y/100
    err_yi = np.clip(err_yi, -10, 10)

    torqueXX = - (Kp * err_y +
                  Kd*err_yd + Ki*err_yi)
    torqueXX = np.clip(torqueXX, -100, 100)
    torqueYY = Kp * err_x + \
        Kd*err_xd + Ki*err_xi
    torqueYY = np.clip(torqueYY, -100, 100)

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

    if use_ROS:
        # update msgs
        COM_msg.data = [err_x, err_y, com_pos[2],
                        control_torque[0], control_torque[1], control_torque[2]]

        lean_angle_msg.header = Header()
        lean_angle_msg.position = [imu_euler[0], imu_euler[1]]
        lean_angle_msg.velocity = [angular[0], angular[1]]

        # publish
        lean_angle_pub.publish(lean_angle_msg)
        COM_pub.publish(COM_msg)

    p.stepSimulation()
    time.sleep(sim_dt)


p.disconnect()
