#!/usr/bin/env python
#  Copyright Microdynamic Systems Laboratory 2021
#
# @author Cornelia Bauer <cornelib@cmu.edu>
#
# @brief Python script to move ballbot to waypoints in pybullet
#


from os import times

from numpy.core.function_base import linspace
import scipy
import rospy
from ballbot_arm_msgs.msg import ArmCommand
from sensor_msgs.msg import JointState
import numpy as np
from rt_msgs.msg import OlcCmd, VelCmd, State, Odom
from std_msgs.msg import Header
import scipy.interpolate
import rospkg
rospack = rospkg.RosPack()
PACKAGE_WS_PATH = rospack.get_path('ballbot_pybullet_sim').split("/ballbot_pybullet_sim")[0]
traj_freq = 50


def get_base_waypoints(base_init):
     # import base waypoints 
    n_waypoints = rospy.get_param('/base/motion/number_waypoints')
    param = "/base/motion/waypoint"

    base_waypoints = np.array([rospy.get_param(param+str(i)+'/position') for i in range(n_waypoints)])
    base_waypoints = np.append(base_init, base_waypoints, axis=0)
    base_durations = [rospy.get_param(param+str(i)+'/duration') for i in range(n_waypoints)]

    timesteps = np.array([0.0])
    timesteps_init = np.array([0.0])
    for duration in base_durations:
        timesteps_init = np.append(timesteps_init, timesteps_init[-1]+duration)
        timesteps = np.append(timesteps,np.linspace(timesteps[-1], timesteps[-1]+duration, int(duration*traj_freq)+1, endpoint=True)[1:])
    base_des_pos = np.array([np.interp(timesteps, timesteps_init, base_waypoints[:,i]) for i in range(2)])
    return base_des_pos

def get_arm_waypoints(side, init_pos, init_vel):
    # import arm waypoints 
    n_waypoints = rospy.get_param('/'+ side + '/motion/number_waypoints')
    param = '/' + side + '/motion/waypoint'

    arm_pos_waypoints = np.array([rospy.get_param(param+str(i)+'/position') for i in range(n_waypoints)])
    arm_pos_waypoints = np.append(init_pos, arm_pos_waypoints, axis=0)
    arm_vel_waypoints = np.array([rospy.get_param(param+str(i)+'/velocity') for i in range(n_waypoints)])
    arm_vel_waypoints = np.append(init_vel, arm_vel_waypoints, axis=0)
    arm_durations = [rospy.get_param(param+str(i)+'/duration') for i in range(n_waypoints)]

    timesteps = np.array([0.0])
    timesteps_init = np.array([0.0])
    for duration in arm_durations:
        timesteps_init = np.append(timesteps_init, timesteps_init[-1]+duration)
        timesteps = np.append(timesteps,np.linspace(timesteps[-1], timesteps[-1]+duration, int(duration*traj_freq)+1, endpoint=True)[1:])
    # arm_des_pos = np.array([np.interp(timesteps, timesteps_init, arm_pos_waypoints[:,i]) for i in range(7)])
    arm_des_pos = np.array([scipy.interpolate.CubicSpline(timesteps_init, arm_pos_waypoints[:,i], bc_type='natural')(timesteps) for i in range(7)])

    return arm_des_pos


def trajPublisher():
    pub_olc = rospy.Publisher('/rt/olc_cmd', OlcCmd, queue_size=10)
    pub_arm_right = rospy.Publisher('/ballbotArms/controller/joint_impedance/right/command', ArmCommand, queue_size=10)
    pub_arm_left = rospy.Publisher('/ballbotArms/controller/joint_impedance/left/command', ArmCommand, queue_size=10)

    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(traj_freq) # 100hz

    # get current state robot state
    base_init = np.array([[0.0, 0.0]])

    current_joint_states_msg = rospy.wait_for_message('/joint_states',JointState)
    q_right_init = np.array([current_joint_states_msg.position[3:10]])
    q_left_init = np.array([current_joint_states_msg.position[10:17]])
    qdot_right_init = np.zeros((1,7))
    qdot_left_init = np.zeros((1,7))


    # import base waypoints     
    base_des_pos = get_base_waypoints(base_init)
    q_right_des = get_arm_waypoints('right', q_right_init, qdot_right_init)
    q_left_des = get_arm_waypoints('left', q_left_init, qdot_left_init)

    olc_msg = OlcCmd()
    right_arm_msg = ArmCommand()
    left_arm_msg = ArmCommand()
    for i in range(max(len(base_des_pos[0]), len(q_right_des[0]),len(q_left_des[0]))):
        if i<len(base_des_pos[0])-1:
            olc_msg.xAng, olc_msg.yAng = [base_des_pos[0,i], base_des_pos[1,i]]
        if i<len(q_right_des[0])-1:
            right_arm_msg.position = q_right_des[:,i]
        if i<len(q_left_des[0])-1:
            left_arm_msg.position = q_left_des[:,i]


        pub_olc.publish(olc_msg)
        pub_arm_right.publish(right_arm_msg)
        pub_arm_left.publish(left_arm_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        trajPublisher()
    except rospy.ROSInterruptException:
        pass

