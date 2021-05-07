import os
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load Ballbot URDF
# path/to/ballbot_arm_description
package_path = "/usr0/home/cornelib/sandbox/ballbot_ws/src/ballbot_arm_description"
urdf_path = package_path + "/robots/urdf/ballbot_plus_pybullet.urdf"
ballbot = p.loadURDF(urdf_path, startPos,
                     startOrientation, useFixedBase=False)

# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

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
    if (jointType == p.JOINT_SPHERICAL):
        paramIds_ball.append(p.addUserDebugParameter(
            jointName.decode("utf-8"), -10, 10, 0))
        paramIds_ball.append(p.addUserDebugParameter(
            jointName.decode("utf-8"), -10, 10, 0))


# p.setRealTimeSimulation(1)
while (1):
    p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(
            ballbot, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
    c = paramIds_ball[0]
    fx = p.readUserDebugParameter(c)
    c = paramIds_ball[1]
    fy = p.readUserDebugParameter(c)
    p.setJointMotorControlMultiDof(ballbot,
                                   0,
                                   controlMode=p.POSITION_CONTROL,
                                   targetPosition=[0, 0, 0, 1],
                                   force=[0, 0, 0])
    p.setJointMotorControlMultiDof(ballbot,
                                   0,
                                   controlMode=p.TORQUE_CONTROL,
                                   force=[fx, fy, 0])
    imu_position, imu_orientation = p.getBasePositionAndOrientation(ballbot)
    print(f"imu_orientation: {imu_orientation}")
    p.stepSimulation()
    time.sleep(0.01)


p.disconnect()
