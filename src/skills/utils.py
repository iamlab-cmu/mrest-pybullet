import quaternion
import numpy as np

def min_jerk_trajectory_generator_pose(initial_position, initial_quat, final_position, final_quat, n_steps):
    r = np.linspace(0,1,n_steps)
    t_min_jerk = (10 * r ** 3 - 15 * r ** 4 + 6 * r ** 5)
    
    pos_traj = initial_position.reshape(3,1) + (final_position - initial_position).reshape(3,1) * t_min_jerk.reshape(1,n_steps)
    quat_traj = quaternion.slerp(
        initial_quat, 
        final_quat,
        0, 1, t_min_jerk
    )
    return pos_traj.T, quat_traj

def linear_trajectory_generator_pose(initial_position, initial_quat, final_position, final_quat, n_steps):
    t_linear= np.linspace(0,1,n_steps)

    pos_traj = initial_position.reshape(3,1) + (final_position - initial_position).reshape(3,1) * t_linear.reshape(1,n_steps)
    quat_traj = quaternion.slerp(
        initial_quat, 
        final_quat,
        0, 1, t_linear
    )
    return pos_traj.T, quat_traj

def convert_bullet_quat_to_quaternion(quat):
    """
    Converts pybullet quaternion vector which is in 'xyzw' format to np.quaternion which accepts arrays in 'wxyz' order
    """
    return quaternion.from_float_array([quat[3], quat[0], quat[1], quat[2]])

def convert_quaternion_to_bullet_quat(quat):
    """
    Converts np.quaternion which accepts arrays in 'wxyz' order to  pybullet quaternion vector which is in 'xyzw' format
    """
    return [quat.x, quat.y, quat.z, quat.w]