#!/usr/bin/env python
#  Copyright Microdynamic Systems Laboratory 2021
#
# @author Cornelia Bauer <cornelib@cmu.edu>
#
# @brief Python script to move ballbot to waypoints in pybullet
#

import rospy
from ballbot_arm_msgs.msg import TaskSpaceTrajectory
from ballbot_arm_msgs.msg import TaskSpaceControl
import numpy as np
from rt_msgs.msg import OlcCmd
from std_msgs.msg import Header
import rospkg
rospack = rospkg.RosPack()
PACKAGE_WS_PATH = rospack.get_path('ballbot_pybullet_sim').split("/ballbot_pybullet_sim")[0]
traj_freq = 50


def get_arm_waypoints(side, init_pos, init_orient):

    n_waypoints = rospy.get_param('/'+ side + '/motion/number_waypoints')
    param = '/' + side + '/motion/waypoint'

    arm_pos_waypoints = np.array([rospy.get_param(param+str(i)+'/position') for i in range(n_waypoints)])
    arm_pos_waypoints = np.append(init_pos, arm_pos_waypoints, axis=0)
    arm_orient_waypoints = np.array([rospy.get_param(param+str(i)+'/orientation') for i in range(n_waypoints)])
    arm_orient_waypoints = np.append(init_orient, arm_orient_waypoints, axis=0)
    arm_durations = [rospy.get_param(param+str(i)+'/duration') for i in range(n_waypoints)]

    timesteps = np.array([0.0])
    timesteps_init = np.array([0.0])
    for duration in arm_durations:
        timesteps_init = np.append(timesteps_init, timesteps_init[-1]+duration)
        timesteps = np.append(timesteps,np.linspace(timesteps[-1], timesteps[-1]+duration, int(duration*traj_freq)+1, endpoint=True)[1:])
    arm_des_pos = np.array([np.interp(timesteps, timesteps_init, arm_pos_waypoints[:,i]) for i in range(3)])
    from scipy.spatial.transform import Slerp
    from scipy.spatial.transform import Rotation as R
    rots = R.from_quat(arm_orient_waypoints)
    slerp = Slerp(timesteps_init, rots)
    arm_des_rots = slerp(timesteps).as_quat()
    return arm_des_pos, arm_des_rots


def trajPublisher():
    pub_arm_right = rospy.Publisher('/task_space_control/right/command', TaskSpaceTrajectory, queue_size=10)
    pub_arm_left = rospy.Publisher('/task_space_control/left/command', TaskSpaceTrajectory, queue_size=10)

    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(traj_freq) # 100hz

    # get current state robot state
    base_init = np.array([[0.0, 0.0]])
    current_right_pose_msg = rospy.wait_for_message('/task_space_control/right/debug/reference',TaskSpaceControl)
    right_pos_init = np.array([[current_right_pose_msg.pose_actual.pose.position.x, current_right_pose_msg.pose_actual.pose.position.y, current_right_pose_msg.pose_actual.pose.position.z]])
    right_orient_init = np.array([[current_right_pose_msg.pose_actual.pose.orientation.x, current_right_pose_msg.pose_actual.pose.orientation.y, current_right_pose_msg.pose_actual.pose.orientation.z, current_right_pose_msg.pose_actual.pose.orientation.w]])

    current_left_pose_msg = rospy.wait_for_message('/task_space_control/left/debug/reference',TaskSpaceControl)
    left_pos_init = np.array([[current_left_pose_msg.pose_actual.pose.position.x, current_left_pose_msg.pose_actual.pose.position.y, current_left_pose_msg.pose_actual.pose.position.z]])
    left_orient_init = np.array([[current_left_pose_msg.pose_actual.pose.orientation.x, current_left_pose_msg.pose_actual.pose.orientation.y, current_left_pose_msg.pose_actual.pose.orientation.z, current_left_pose_msg.pose_actual.pose.orientation.w]])

    # import base waypoints
    arm_right_des_pos, arm_right_des_orient = get_arm_waypoints('right', right_pos_init, right_orient_init)
    arm_left_des_pos, arm_left_des_orient = get_arm_waypoints('left', left_pos_init, left_orient_init)

    right_arm_msg = TaskSpaceTrajectory()
    left_arm_msg = TaskSpaceTrajectory()

    for i in range(max(len(arm_right_des_pos[0]),len(arm_left_des_pos[0]))):
        if i<len(arm_right_des_pos[0])-1:
            right_arm_msg.pose.position.x, right_arm_msg.pose.position.y, right_arm_msg.pose.position.z = arm_right_des_pos[0,i],arm_right_des_pos[1,i],arm_right_des_pos[2,i]
            right_arm_msg.pose.orientation.x = arm_right_des_orient[i,0]
            right_arm_msg.pose.orientation.y = arm_right_des_orient[i,1]
            right_arm_msg.pose.orientation.z = arm_right_des_orient[i,2]
            right_arm_msg.pose.orientation.w = arm_right_des_orient[i,3]

        if i<len(arm_left_des_pos[0])-1:
            left_arm_msg.pose.position.x, left_arm_msg.pose.position.y, left_arm_msg.pose.position.z = arm_left_des_pos[0,i],arm_left_des_pos[1,i],arm_left_des_pos[2,i]
            left_arm_msg.pose.orientation.x = arm_left_des_orient[i,0]
            left_arm_msg.pose.orientation.y = arm_left_des_orient[i,1]
            left_arm_msg.pose.orientation.z = arm_left_des_orient[i,2]
            left_arm_msg.pose.orientation.w = arm_left_des_orient[i,3]

        # left_arm_msg.header.frame_id = 'world'
        # right_arm_msg.header.frame_id = 'world'
        
        pub_arm_right.publish(right_arm_msg)
        pub_arm_left.publish(left_arm_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        trajPublisher()
    except rospy.ROSInterruptException:
        pass

