import numpy as np
import pybullet as p
from robot.robot_simulator_pickup_env import *
from controllers.arm_controller import TaskSpaceArmController
from controllers.outer_loop_controllers import StationKeepingController

import time
from pandas import DataFrame
from .utils import *

class GotoJointPosition():
    def __init__(self, robot_simulator, target_left_q, target_right_q, T):
        self.T = T
        self.steps = int(T/SIMULATION_TIME_STEP_S)
        current_left_q = np.array(robot_simulator.ballbot.arm_pos[7:])
        current_right_q = np.array(robot_simulator.ballbot.arm_pos[:7])
        
        self.left_arm_traj = np.linspace(current_left_q, target_left_q, self.steps)
        self.right_arm_traj = np.linspace(current_right_q, target_right_q, self.steps)
        self.robot_simulator = robot_simulator
        self.robot_simulator.ballbot.set_arm_position_mode()

    def execute(self):
        for i in range(self.steps):
            arm_joint_command = np.concatenate(
                (self.right_arm_traj[i], self.left_arm_traj[i]))
            self.robot_simulator.arm_joint_command = arm_joint_command.copy()
            self.robot_simulator.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)

class GotoTaskSpacePose():
    def __init__(self, robot_simulator, 
                target_left_pos, 
                target_left_quat, 
                target_right_pos, 
                target_right_quat, 
                T, arms='both',
                enable_base=False,
                interp_type='linear',
                termination_condition='time'):
        self.T = T
        self.steps = int(T/SIMULATION_TIME_STEP_S)
        self.robot_simulator = robot_simulator
        self.enable_base = enable_base
        self.termination_condition = termination_condition
        self.debug = True

        self.control_left_arm = True if (arms=='both' or arms == 'left') else False
        self.control_right_arm = True if (arms=='both' or arms == 'right') else False
        
        if self.control_left_arm:
            self.task_space_left_arm_controller = TaskSpaceArmController(arm='left')
            self.task_space_left_arm_controller.set_gains([[2000.0, 2000.0, 2000, 20., 20., 20.],
                                                            [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]])
            left_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolL'])
            left_ee_init_pos = np.array(left_ee_frame_info[0])
            left_ee_init_quat = convert_bullet_quat_to_quaternion(left_ee_frame_info[1])

            if interp_type == 'min_jerk':
                self.left_arm_pos_traj, self.left_arm_quat_traj = min_jerk_trajectory_generator_pose(
                    left_ee_init_pos, left_ee_init_quat, target_left_pos, target_left_quat, self.steps
                )
            if interp_type == 'linear':
                self.left_arm_pos_traj, self.left_arm_quat_traj = linear_trajectory_generator_pose(
                    left_ee_init_pos, left_ee_init_quat, target_left_pos, target_left_quat, self.steps
                )

        if self.control_right_arm:
            self.task_space_right_arm_controller = TaskSpaceArmController(arm='right')
            self.task_space_right_arm_controller.set_gains([[2000.0, 2000.0, 2000.0, 25., 25., 25.], 
                                                            [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]])
            right_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolR'])
            right_ee_init_pos = np.array(right_ee_frame_info[0])
            right_ee_init_quat = convert_bullet_quat_to_quaternion(right_ee_frame_info[1])

            if interp_type == 'min_jerk':
                self.right_arm_pos_traj, self.right_arm_quat_traj = min_jerk_trajectory_generator_pose(
                    right_ee_init_pos, right_ee_init_quat, target_right_pos, target_right_quat, self.steps
                )
            if interp_type == 'linear':
                self.right_arm_pos_traj, self.right_arm_quat_traj = linear_trajectory_generator_pose(
                    right_ee_init_pos, right_ee_init_quat, target_right_pos, target_right_quat, self.steps
                )
            
            if self.debug:
                import matplotlib.pyplot as plt
                fig = plt.figure(figsize=(6, 6))
                fig, ax = plt.subplots(2, 2)
                interp_traj = np.array([[quat.x, quat.y, quat.z, quat.w] for quat in self.right_arm_quat_traj])
                ax[0,0].plot(interp_traj[:,0])
                ax[0,1].plot(interp_traj[:,1])
                ax[1,0].plot(interp_traj[:,2])
                ax[1,1].plot(interp_traj[:,3])
                plt.savefig(f'./media/interpolated_quats_right_ee.jpg')

        if enable_base:
            self.ball_controller = StationKeepingController()
            self.ball_controller.set_max_angle(6.)
            self.ball_controller.set_gains(8., 0.0, 1., 8., 0.0, 1.) # kPx, kIx, kDx, kPy, kIy, kDy
            robot_simulator.update_robot_state(BallState.OLC)

        self.robot_simulator.ballbot.set_arm_torque_mode()

    def transformWorldToShoulderFrame(self, pWorld,  quatWorld, arm='left'):
        shoulder_link_name = 'LArm0' if arm == 'left' else 'RArm0'
        shoulder_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds[shoulder_link_name])

        # shoulderFrame = convert_bullet_quat_to_quaternion(shoulder_frame_info[1])
        # quatShoulder = shoulderFrame.inverse() * quatWorld

        quatWorld = convert_quaternion_to_bullet_quat(quatWorld)
        shoulderToWorldTransform = p.invertTransform(shoulder_frame_info[4], shoulder_frame_info[5])
        pShoulder, quatShoulder = p.multiplyTransforms(shoulderToWorldTransform[0], shoulderToWorldTransform[1],
                                    pWorld, quatWorld)
        quatShoulder = convert_bullet_quat_to_quaternion(quatShoulder)

        return pShoulder, quatShoulder

    def terminate_skill(self):
        if self.termination_condition == 'time':
            return False # For loop in the skill takes care of this exit condition

        if self.termination_condition == 'contact':
            # check for contact between the cube and the object
            contacts = p.getContactPoints(self.robot_simulator.ballbot.robot, self.robot_simulator.environment.cubeId2)
            pass
        
        if self.termination_condition == 'goal':
            # check if current posiiton, velocity and quat is near enough to the goal
            # TODO: Implement this
            return False

    def execute(self):
        
        if self.debug:
            if self.control_left_arm:
                obs_pose_left, des_pose_left, torque_left = [], [], []
                left_arm_goal_marker = p.addUserDebugLine(
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1, 0, 0], 20, 0)
                toolL_marker = p.addUserDebugLine(
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1, 0, 0], 20, 0)
                
            if self.control_right_arm:
                obs_pose_right, des_pose_right, torque_right = [], [], []
                right_arm_goal_marker = p.addUserDebugLine(
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 1, 0], 20, 0)
                toolR_marker = p.addUserDebugLine(
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 1, 0], 20, 0)

        for i in range(self.steps):
            if self.control_left_arm:
                current_left_q = np.array(self.robot_simulator.ballbot.arm_pos[7:])
                current_left_qdot = np.array(self.robot_simulator.ballbot.arm_vel[7:])
                
                des_left_ee_pos, des_left_ee_quat = self.transformWorldToShoulderFrame(self.left_arm_pos_traj[i], 
                                                                                       self.left_arm_quat_traj[i], arm='left')
                self.task_space_left_arm_controller.set_desired_pose(des_left_ee_pos, des_left_ee_quat)
                self.task_space_left_arm_controller.update_current_state(
                    current_left_q, current_left_qdot)
                self.task_space_left_arm_controller.update(SIMULATION_TIME_STEP_S)
                larm_torques = self.task_space_left_arm_controller.armTorques
                self.robot_simulator.larm_torque_command = np.array(larm_torques).flatten()

            if self.control_right_arm:
                current_right_q = np.array(self.robot_simulator.ballbot.arm_pos[:7])
                current_right_qdot = np.array(self.robot_simulator.ballbot.arm_vel[:7])
                
                des_right_ee_pos, des_right_ee_quat = self.transformWorldToShoulderFrame(self.right_arm_pos_traj[i], 
                                                                                         self.right_arm_quat_traj[i], arm='right')
                self.task_space_right_arm_controller.set_desired_pose(des_right_ee_pos, des_right_ee_quat)

                #debug
                # right_ee_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds['toolR'])
                # curr_ee_pos_shoulder_frame, curr_ee_pos_quat_shoulder_frame = self.transformWorldToShoulderFrame(right_ee_frame_info[0], 
                #                                                 convert_bullet_quat_to_quaternion(right_ee_frame_info[1]), arm='right')
                
                self.task_space_right_arm_controller.update_current_state(
                    current_right_q, current_right_qdot)
                self.task_space_right_arm_controller.update(SIMULATION_TIME_STEP_S)
                rarm_torques = self.task_space_right_arm_controller.armTorques
                self.robot_simulator.rarm_torque_command = np.array(rarm_torques).flatten()

            if self.enable_base:
                xVelErr = 0.0 - self.robot_simulator.ballbot.ballLinVelInBodyOrient[0]
                yVelErr = 0.0 - self.robot_simulator.ballbot.ballLinVelInBodyOrient[1]
                
                if self.control_left_arm:
                    left_ee_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds['toolL'])
                    left_PosErr_shoulder_frame = self.left_arm_pos_traj[i] - left_ee_frame_info[0]
                
                if self.control_right_arm:
                    right_ee_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds['toolR'])
                    right_PosErr_shoulder_frame = self.right_arm_pos_traj[i] - right_ee_frame_info[0]
                
                if self.control_left_arm and self.control_right_arm:
                    PosErr_avg = (left_PosErr_shoulder_frame + right_PosErr_shoulder_frame)/2
                elif self.control_left_arm:
                    PosErr_avg = left_PosErr_shoulder_frame.copy()
                else:
                    PosErr_avg = right_PosErr_shoulder_frame.copy()

                self.ball_controller.set_error_value(
                        PosErr_avg[0], PosErr_avg[1], xVelErr, yVelErr)
                self.ball_controller.get_angle_output()
                self.robot_simulator.olcCmdXAng = self.ball_controller._desired_y_body_angle
                self.robot_simulator.olcCmdYAng = self.ball_controller._desired_x_body_angle
            
            self.robot_simulator.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)

            exit_skill = self.terminate_skill()
            if exit_skill:
                break

            if self.debug:
                if self.control_left_arm:
                    obs_pose_left.append(self.task_space_left_arm_controller.currPos)
                    des_pose_left.append(des_left_ee_pos)
                    torque_left.append(larm_torques)
                    p.addUserDebugLine(self.left_arm_pos_traj[i], [self.left_arm_pos_traj[i,0], self.left_arm_pos_traj[i,1], self.left_arm_pos_traj[i,2]+0.03],
                                        [1, 0, 0], 20, 0, replaceItemUniqueId=left_arm_goal_marker)
                    p.addUserDebugLine(left_ee_frame_info[0], [left_ee_frame_info[0][0], left_ee_frame_info[0][1], left_ee_frame_info[0][2]+0.03],
                                        [0, 0, 1], 20, 0, replaceItemUniqueId=toolL_marker)
                if self.control_right_arm:
                    des_pose_right.append(des_right_ee_pos)
                    obs_pose_right.append(self.task_space_right_arm_controller.currPos)
                    torque_right.append(rarm_torques)
                    p.addUserDebugLine(self.right_arm_pos_traj[i], [self.right_arm_pos_traj[i,0], self.right_arm_pos_traj[i,1], self.right_arm_pos_traj[i,2]+0.03],
                                        [0, 1, 0], 20, 0, replaceItemUniqueId=right_arm_goal_marker)
                    p.addUserDebugLine(right_ee_frame_info[0], [right_ee_frame_info[0][0], right_ee_frame_info[0][1], right_ee_frame_info[0][2]+0.03],
                                        [0, 0, 1], 20, 0, replaceItemUniqueId=toolR_marker)
                
        if self.debug:
            if self.control_left_arm:
                obs_pose_left = np.stack(obs_pose_left, axis=0)
                des_pose_left = np.stack(des_pose_left, axis=0)
                torque_left = np.stack(torque_left, axis=0)
                df = DataFrame({
                    'torque_left0': torque_left[:,0],
                    'torque_left1': torque_left[:,1],
                    'torque_left2': torque_left[:,2],
                    'torque_left3': torque_left[:,3],
                    'torque_left4': torque_left[:,4],
                    'torque_left5': torque_left[:,5],
                    'torque_left6': torque_left[:,6],
                    })
                fig = df.plot().get_figure()
                fig.savefig(f'./media/torque_left.jpg')

                df = DataFrame({
                    'obs_pose_left0': obs_pose_left[:,0],
                    'obs_pose_left1': obs_pose_left[:,1],
                    'obs_pose_left2': obs_pose_left[:,2],
                    'des_pose_left0': des_pose_left[:,0],
                    'des_pose_left1': des_pose_left[:,1],
                    'des_pose_left2': des_pose_left[:,2],
                    })
                fig = df.plot().get_figure()
                fig.savefig(f'./media/obs_pose_left.jpg')
            
            if self.control_right_arm:
                obs_pose_right = np.stack(obs_pose_right, axis=0)
                des_pose_right = np.stack(des_pose_right, axis=0)
                torque_right = np.stack(torque_right, axis=0)
                df = DataFrame({
                    'torque_right0': torque_right[:,0],
                    'torque_right1': torque_right[:,1],
                    'torque_right2': torque_right[:,2],
                    'torque_right3': torque_right[:,3],
                    'torque_right4': torque_right[:,4],
                    'torque_right5': torque_right[:,5],
                    'torque_right6': torque_right[:,6],
                    })
                fig = df.plot().get_figure()
                fig.savefig(f'./media/torque_right.jpg')

                df = DataFrame({
                    'obs_pose_right0': obs_pose_right[:,0],
                    'obs_pose_right1': obs_pose_right[:,1],
                    'obs_pose_right2': obs_pose_right[:,2],
                    'des_pose_right0': des_pose_right[:,0],
                    'des_pose_right1': des_pose_right[:,1],
                    'des_pose_right2': des_pose_right[:,2],
                    })
                fig = df.plot().get_figure()
                fig.savefig(f'./media/obs_pose_right.jpg')