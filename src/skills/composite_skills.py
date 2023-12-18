import numpy as np
import pybullet as p
from robot.robot_simulator_pickup_env import BallState

from controllers.outer_loop_controllers import StationKeepingController
from .arm_skills import *
from .gripper_skills import *
from moviepy.editor import ImageSequenceClip
from tqdm import trange
import quaternion

class GotoBlockSkill():
    def __init__(self, robot_simulator):
        self.robot_simulator = robot_simulator
    
    def execute(self):
        ## Reach object using task space skill
        T = 5 # seconds
        cube_pos, cube_quat = p.getBasePositionAndOrientation(self.robot_simulator.environment.cubeId2)
        cube_pos = np.array(cube_pos)
        cube_pos[2] = cube_pos[2] + 0.03
        target_right_quat = quaternion.as_quat_array([0.50829406408023, -0.491436150946075, 0.492614818619556, 0.50740348288175]) # format = 'wxyz'
        skill = GotoTaskSpacePose(self.robot_simulator, 
                                [], 
                                [],
                                np.array(cube_pos), 
                                target_right_quat,
                                T, arms='right',
                                enable_base=True,
                                interp_type='linear',
                                termination_condition='time')
        skill.execute()

        ## Close gripper
        T = 3 #seconds  
        skill = CloseGripper(self.robot_simulator, T, hands='right')
        skill.execute()

        ## Create fixed link between gripper and object
        # cube_pos, cube_quat = p.getBasePositionAndOrientation(self.robot_simulator.environment.cubeId2)
        # constraint_id = p.createConstraint(self.robot_simulator.ballbot.robot,
        #                                 self.robot_simulator.ballbot.linkIds['R_bh_base_link'], 
        #                                 self.robot_simulator.environment.cubeId2,
        #                                 -1,
        #                                 p.JOINT_FIXED,
        #                                 jointAxis=[0, 0, 1],
        #                                 parentFramePosition=[0, 0, 0],
        #                                 childFramePosition=[0, 0, 0])
        # p.changeConstraint(constraint_id, maxForce=100,erp=0.2)

        ## Raise up grasped object
        # T = 5 # seconds
        # cube_pos, cube_quat = p.getBasePositionAndOrientation(self.robot_simulator.environment.cubeId2)
        # cube_pos = np.array(cube_pos)
        # cube_pos[2] = cube_pos[2] + 0.2
        # skill = GotoTaskSpacePose(self.robot_simulator, 
        #                         [], 
        #                         [],
        #                         np.array(cube_pos), 
        #                         target_right_quat,
        #                         T, arms='right',
        #                         enable_base=True,
        #                         interp_type='linear',
        #                         termination_condition='time')
        # skill.execute()

class PickUpBlockSkill():
    def __init__(self, robot_simulator, img_save_freqs={}):
        
        self.T_reach = 4
        self.steps_reach = int(self.T_reach/SIMULATION_TIME_STEP_S)

        self.T_lift = 4
        self.steps_lift = int(self.T_lift/SIMULATION_TIME_STEP_S)

        self.total_steps = self.steps_reach + self.steps_lift
        self.img_save_freqs = img_save_freqs
        
        self.robot_simulator = robot_simulator
        self.enable_base = True
        self.debug = True
        self.contacted_block = False
        self.interp_type = 'min_jerk'
        self.termination_condition = 'goal' # time

        cube_pos, cube_quat = p.getBasePositionAndOrientation(self.robot_simulator.environment.cubeId2)
        self.target_right_pos_reach = np.array(cube_pos) + np.array([0., 0., 0.03])

        self.target_right_pos_lift = self.target_right_pos_reach + np.array([0., 0.2, 0.1])

        target_right_quat = quaternion.as_quat_array([0.50829406408023, -0.491436150946075, 0.492614818619556, 0.50740348288175]) # format = 'wxyz'

        self.task_space_right_arm_controller = TaskSpaceArmController(arm='right')
        # self.task_space_right_arm_controller.set_gains([[3000.0, 3000.0, 3000.0, 25., 25., 25.], 
        #                                                     [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]])
        self.task_space_right_arm_controller.set_gains([[10000.0, 10000.0, 10000.0, 25., 25., 25.], 
                                                            [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]])
        right_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolR'])
        right_ee_init_pos = np.array(right_ee_frame_info[0])
        right_ee_init_quat = convert_bullet_quat_to_quaternion(right_ee_frame_info[1])

        if self.interp_type == 'min_jerk':
            self.right_arm_pos_traj_reach, self.right_arm_quat_traj_reach = min_jerk_trajectory_generator_pose(
                right_ee_init_pos, right_ee_init_quat, self.target_right_pos_reach, target_right_quat, self.steps_reach
            )
            self.right_arm_pos_traj_lift, self.right_arm_quat_traj_lift = min_jerk_trajectory_generator_pose(
                self.target_right_pos_reach, right_ee_init_quat, self.target_right_pos_lift, target_right_quat, self.steps_lift
            )
        if self.interp_type == 'linear':
            self.right_arm_pos_traj_reach, self.right_arm_quat_traj_reach = linear_trajectory_generator_pose(
                right_ee_init_pos, right_ee_init_quat, self.target_right_pos_reach, target_right_quat, self.steps_reach
            )
            self.right_arm_pos_traj_lift, self.right_arm_quat_traj_lift = linear_trajectory_generator_pose(
                self.target_right_pos_reach, right_ee_init_quat, self.target_right_pos_lift, target_right_quat, self.steps_lift
            )
        print(f"Total steps in this skill are {self.total_steps}")

        if self.debug:
            import matplotlib.pyplot as plt
            fig = plt.figure(figsize=(6, 6))
            fig, ax = plt.subplots(2, 2)
            interp_traj = np.array([[quat.x, quat.y, quat.z, quat.w] for quat in self.right_arm_quat_traj_reach])
            ax[0,0].plot(interp_traj[:,0])
            ax[0,1].plot(interp_traj[:,1])
            ax[1,0].plot(interp_traj[:,2])
            ax[1,1].plot(interp_traj[:,3])
            plt.savefig(f'./media/interpolated_quats_right_ee.jpg')

        if self.enable_base:
            self.ball_controller = StationKeepingController()
            self.ball_controller.set_max_angle(6.)
            self.ball_controller.set_gains(9., 0.0, 1., 9., 0.0, 1.) # kPx, kIx, kDx, kPy, kIy, kDy
            robot_simulator.update_robot_state(BallState.OLC)

        self.robot_simulator.ballbot.set_arm_torque_mode()
        self.gripper_close_state = np.array([1.47, 0., 0., 1.47, 0., 0., 1.47, 0.])

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
            
            cube_pos, cube_quat = p.getBasePositionAndOrientation(self.robot_simulator.environment.cubeId2)

            # right_ee_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds['toolR'])
            # gripper_closed = np.linalg.norm(self.robot_simulator.ballbot.barrett_right_hand_pos - self.gripper_close_state)
            # reach = np.linalg.norm(right_ee_frame_info[0] - (np.array(cube_pos) + np.array([0., 0., 0.03])))
            # print('gripper_closed',gripper_closed)
            # print('reach',reach)
            # success = (reach<0.07 and gripper_closed<1.3)
            # print('success',success)

            if np.linalg.norm(self.target_right_pos_lift - cube_pos) < 0.07:
                return True
            else:
                return False

    def execute(self):
        vid_turret, vid_static = [], []
        vid_static_sideview = []
        obs_pose_right, des_pose_right, torque_right = [], [], []
        obs_contact = []
        wrench_right = []
        if self.debug:
            right_arm_goal_marker = p.addUserDebugLine(
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 1, 0], 20, 0)
            toolR_marker = p.addUserDebugLine(
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 1, 0], 20, 0)

        skill_t = 0
        right_arm_pos_traj = self.right_arm_pos_traj_reach.copy()
        right_arm_quat_traj = self.right_arm_quat_traj_reach.copy()
        success = False
        for i in trange(self.total_steps):
            
            current_right_q = np.array(self.robot_simulator.ballbot.arm_pos[:7])
            current_right_qdot = np.array(self.robot_simulator.ballbot.arm_vel[:7])
            
            des_right_ee_pos, des_right_ee_quat = self.transformWorldToShoulderFrame(right_arm_pos_traj[skill_t], 
                                                                                    right_arm_quat_traj[skill_t], arm='right')
            self.task_space_right_arm_controller.set_desired_pose(des_right_ee_pos, des_right_ee_quat)

            self.task_space_right_arm_controller.update_current_state(
                current_right_q, current_right_qdot)
            self.task_space_right_arm_controller.update(SIMULATION_TIME_STEP_S)
            rarm_torques = self.task_space_right_arm_controller.armTorques
            self.robot_simulator.rarm_torque_command = np.array(rarm_torques).flatten()

            if self.enable_base:
                xVelErr = 0.0 - self.robot_simulator.ballbot.ballLinVelInBodyOrient[0]
                yVelErr = 0.0 - self.robot_simulator.ballbot.ballLinVelInBodyOrient[1]

                right_ee_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds['toolR'])
                right_PosErr = right_arm_pos_traj[skill_t] - right_ee_frame_info[0]
                
                PosErr_avg = right_PosErr.copy()

                self.ball_controller.set_error_value(
                        PosErr_avg[0], PosErr_avg[1], xVelErr, yVelErr)
                self.ball_controller.get_angle_output()
                self.robot_simulator.olcCmdXAng = self.ball_controller._desired_y_body_angle
                self.robot_simulator.olcCmdYAng = self.ball_controller._desired_x_body_angle

            des_pose_right.append(right_arm_pos_traj[skill_t])
            obs_pose_right.append(right_ee_frame_info[0])
            torque_right.append(rarm_torques)
            if self.contacted_block:
                obs_contact.append(1.)
            else:
                obs_contact.append(0.)
            wrench_right.append(self.robot_simulator.wrench_right)

            # Detect contact
            skill_t += 1
            contacts = p.getContactPoints(self.robot_simulator.ballbot.robot, self.robot_simulator.environment.cubeId2)
            if len(contacts) > 0 or self.contacted_block:
                if not self.contacted_block: # First time contact is made the reach skill converts to lift skills and gripper closes
                    skill_t = 0
                self.contacted_block = True
                self.robot_simulator.barrett_right_hand_joint_command = self.gripper_close_state
                right_arm_pos_traj = self.right_arm_pos_traj_lift.copy()
                right_arm_quat_traj = self.right_arm_quat_traj_lift.copy()
            if skill_t > self.steps_reach-1:
                break
            
            self.robot_simulator.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)

            # if i % int(SENSOR_TIME_STEP_S/SIMULATION_TIME_STEP_S) == 0:
            if i % self.img_save_freqs['Ih_save_freq'] == 0:
                if self.robot_simulator.enable_turret_camera:
                    # turretCameraFeedback = self.robot_simulator.ballbot.update_turretCamera(
                    #     self.robot_simulator.env_cfg['cameras']['image_width'], self.robot_simulator.env_cfg['cameras']['image_height'])
                    turretCameraFeedback = self.robot_simulator.ballbot.update_turretCamera(320, 240)
                    vid_turret.append(turretCameraFeedback)
            if i % self.img_save_freqs['I3_save_freq'] == 0:
                if self.robot_simulator.enable_static_camera:
                    # staticCameraFeedback = self.robot_simulator.ballbot.update_staticCamera(
                    #     self.robot_simulator.env_cfg['cameras']['image_width'], self.robot_simulator.env_cfg['cameras']['image_height'])
                    staticCameraFeedback = self.robot_simulator.ballbot.update_staticCamera(640, 480)
                    vid_static.append(staticCameraFeedback)

                    staticCameraFeedbackSideView = self.robot_simulator.ballbot.update_staticCamera_sideview(
                        self.robot_simulator.env_cfg['cameras']['image_width'], self.robot_simulator.env_cfg['cameras']['image_height'])
                    # staticCameraFeedbackBothViews = np.append(staticCameraFeedback, staticCameraFeedbackSideView, axis=1)
                    vid_static_sideview.append(staticCameraFeedbackSideView)

            if self.terminate_skill():
                success = True
                break

            if self.debug:
                p.addUserDebugLine(right_arm_pos_traj[skill_t-1], [right_arm_pos_traj[skill_t-1,0], right_arm_pos_traj[skill_t-1,1], right_arm_pos_traj[skill_t-1,2]+0.03],
                                    [0, 1, 0], 20, 0, replaceItemUniqueId=right_arm_goal_marker)
                p.addUserDebugLine(right_ee_frame_info[0], [right_ee_frame_info[0][0], right_ee_frame_info[0][1], right_ee_frame_info[0][2]+0.03],
                                    [0, 0, 1], 20, 0, replaceItemUniqueId=toolR_marker)
        
        obs_pose_right = np.stack(obs_pose_right, axis=0)
        des_pose_right = np.stack(des_pose_right, axis=0)
        torque_right = np.stack(torque_right, axis=0)
        if self.debug:
            # save gif
            if self.robot_simulator.enable_turret_camera:
                cl = ImageSequenceClip(vid_turret, fps=10)
                cl.write_gif('./media/turret_Camera.gif', fps=10, logger=None)
                del cl
            if self.robot_simulator.enable_static_camera:
                cl = ImageSequenceClip(vid_static, fps=10)
                cl.write_gif('./media/static_Camera.gif', fps=10, logger=None)
                del cl
            
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
            p.removeAllUserDebugItems()

        traj_info = {
            'observed_pos': obs_pose_right,
            'desired_pos': des_pose_right,
            'observed_contact': np.array(obs_contact),
            'observed_wrench': np.array(wrench_right),
            'eye_in_hand_90': vid_turret,
            'left_cap2': vid_static,
            'vid_static_sideview': vid_static_sideview,
            'success': success
        }
        return traj_info
    

class PickUpBlockSkillTSCMode():
    def __init__(self, robot_simulator, img_save_freqs=24):
        
        self.T_reach = 10
        self.steps_reach = int(self.T_reach/SIMULATION_TIME_STEP_S)

        self.T_lift = 10
        self.steps_lift = int(self.T_lift/SIMULATION_TIME_STEP_S)

        self.total_steps = self.steps_reach + self.steps_lift
        self.img_save_freqs = img_save_freqs
        
        self.robot_simulator = robot_simulator
        self.enable_base = True
        self.debug = True
        self.contacted_block = False
        self.interp_type = 'min_jerk'
        self.termination_condition = 'goal' # time

        cube_pos, cube_quat = p.getBasePositionAndOrientation(self.robot_simulator.environment.cubeId2)
        target_right_pos_reach = np.array(cube_pos) + np.array([0., 0., 0.03])

        self.target_right_pos_lift = target_right_pos_reach + np.array([0., 0.2, 0.1])

        target_right_quat = quaternion.as_quat_array([0.50829406408023, -0.491436150946075, 0.492614818619556, 0.50740348288175]) # format = 'wxyz'

        # self.task_space_right_arm_controller = TaskSpaceArmController(arm='right')
        # self.task_space_right_arm_controller.set_gains([[3000.0, 3000.0, 3000.0, 25., 25., 25.], 
        #                                                     [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]])
        right_ee_frame_info = p.getLinkState(robot_simulator.ballbot.robot, robot_simulator.ballbot.linkIds['toolR'])
        right_ee_init_pos = np.array(right_ee_frame_info[0])
        right_ee_init_quat = convert_bullet_quat_to_quaternion(right_ee_frame_info[1])

        if self.interp_type == 'min_jerk':
            self.right_arm_pos_traj_reach, self.right_arm_quat_traj_reach = min_jerk_trajectory_generator_pose(
                right_ee_init_pos, right_ee_init_quat, target_right_pos_reach, target_right_quat, self.steps_reach
            )
            self.right_arm_pos_traj_lift, self.right_arm_quat_traj_lift = min_jerk_trajectory_generator_pose(
                target_right_pos_reach, right_ee_init_quat, self.target_right_pos_lift, target_right_quat, self.steps_lift
            )
        if self.interp_type == 'linear':
            self.right_arm_pos_traj_reach, self.right_arm_quat_traj_reach = linear_trajectory_generator_pose(
                right_ee_init_pos, right_ee_init_quat, target_right_pos_reach, target_right_quat, self.steps_reach
            )
            self.right_arm_pos_traj_lift, self.right_arm_quat_traj_lift = linear_trajectory_generator_pose(
                target_right_pos_reach, right_ee_init_quat, self.target_right_pos_lift, target_right_quat, self.steps_lift
            )
        print(f"Total steps in this skill are {self.total_steps}")

        if self.debug:
            import matplotlib.pyplot as plt
            fig = plt.figure(figsize=(6, 6))
            fig, ax = plt.subplots(2, 2)
            interp_traj = np.array([[quat.x, quat.y, quat.z, quat.w] for quat in self.right_arm_quat_traj_reach])
            ax[0,0].plot(interp_traj[:,0])
            ax[0,1].plot(interp_traj[:,1])
            ax[1,0].plot(interp_traj[:,2])
            ax[1,1].plot(interp_traj[:,3])
            plt.savefig(f'./media/interpolated_quats_right_ee.jpg')

        # if self.enable_base:
        #     self.ball_controller = StationKeepingController()
        #     self.ball_controller.set_max_angle(6.)
        #     self.ball_controller.set_gains(8., 0.0, 1., 8., 0.0, 1.) # kPx, kIx, kDx, kPy, kIy, kDy
        #     robot_simulator.update_robot_state(BallState.OLC)

        self.robot_simulator.set_arm_task_space_control_mode(arms='right', enable_base=True)
        self.gripper_close_state = np.array([1.47, 0., 0., 1.47, 0., 0., 1.47, 0.])

    # def transformWorldToShoulderFrame(self, pWorld,  quatWorld, arm='left'):
    #     shoulder_link_name = 'LArm0' if arm == 'left' else 'RArm0'
    #     shoulder_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds[shoulder_link_name])

    #     # shoulderFrame = convert_bullet_quat_to_quaternion(shoulder_frame_info[1])
    #     # quatShoulder = shoulderFrame.inverse() * quatWorld

    #     quatWorld = convert_quaternion_to_bullet_quat(quatWorld)
    #     shoulderToWorldTransform = p.invertTransform(shoulder_frame_info[4], shoulder_frame_info[5])
    #     pShoulder, quatShoulder = p.multiplyTransforms(shoulderToWorldTransform[0], shoulderToWorldTransform[1],
    #                                 pWorld, quatWorld)
    #     quatShoulder = convert_bullet_quat_to_quaternion(quatShoulder)

    #     return pShoulder, quatShoulder

    def terminate_skill(self):
        if self.termination_condition == 'time':
            return False # For loop in the skill takes care of this exit condition

        if self.termination_condition == 'contact':
            # check for contact between the cube and the object
            contacts = p.getContactPoints(self.robot_simulator.ballbot.robot, self.robot_simulator.environment.cubeId2)
            pass
        
        if self.termination_condition == 'goal':
            # check if current posiiton, velocity and quat is near enough to the goal
            cube_pos, cube_quat = p.getBasePositionAndOrientation(self.robot_simulator.environment.cubeId2)
            success = np.linalg.norm(self.target_right_pos_lift - cube_pos) < 0.055
            return success

    def execute(self):
        vid_turret, vid_static = [], []
        obs_pose_right, des_pose_right, torque_right = [], [], []
        obs_contact = []
        if self.debug:
            right_arm_goal_marker = p.addUserDebugLine(
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 1, 0], 20, 0)
            toolR_marker = p.addUserDebugLine(
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 1, 0], 20, 0)

        skill_t = 0
        right_arm_pos_traj = self.right_arm_pos_traj_reach.copy()
        right_arm_quat_traj = self.right_arm_quat_traj_reach.copy()
        success = False
        for i in trange(self.total_steps):
            self.robot_simulator.rarm_ts_command_pos = right_arm_pos_traj[skill_t].copy()
            self.robot_simulator.rarm_ts_command_quat = right_arm_quat_traj[skill_t].copy()
            # current_right_q = np.array(self.robot_simulator.ballbot.arm_pos[:7])
            # current_right_qdot = np.array(self.robot_simulator.ballbot.arm_vel[:7])
            
            # des_right_ee_pos, des_right_ee_quat = self.transformWorldToShoulderFrame(right_arm_pos_traj[skill_t], 
            #                                                                         right_arm_quat_traj[skill_t], arm='right')
            # self.task_space_right_arm_controller.set_desired_pose(des_right_ee_pos, des_right_ee_quat)

            # self.task_space_right_arm_controller.update_current_state(
            #     current_right_q, current_right_qdot)
            # self.task_space_right_arm_controller.update(SIMULATION_TIME_STEP_S)
            # rarm_torques = self.task_space_right_arm_controller.armTorques
            # self.robot_simulator.rarm_torque_command = np.array(rarm_torques).flatten()

            # if self.enable_base:
            #     xVelErr = 0.0 - self.robot_simulator.ballbot.ballLinVelInBodyOrient[0]
            #     yVelErr = 0.0 - self.robot_simulator.ballbot.ballLinVelInBodyOrient[1]

            #     right_ee_frame_info = p.getLinkState(self.robot_simulator.ballbot.robot, self.robot_simulator.ballbot.linkIds['toolR'])
            #     right_PosErr = right_arm_pos_traj[skill_t] - right_ee_frame_info[0]
                
            #     PosErr_avg = right_PosErr.copy()

            #     self.ball_controller.set_error_value(
            #             PosErr_avg[0], PosErr_avg[1], xVelErr, yVelErr)
            #     self.ball_controller.get_angle_output()
            #     self.robot_simulator.olcCmdXAng = self.ball_controller._desired_y_body_angle
            #     self.robot_simulator.olcCmdYAng = self.ball_controller._desired_x_body_angle

            des_pose_right.append(right_arm_pos_traj[skill_t])
            # obs_pose_right.append(right_ee_frame_info[0])
            # torque_right.append(rarm_torques)
            if self.contacted_block:
                obs_contact.append(1.)
            else:
                obs_contact.append(0.)

            # Detect contact
            skill_t += 1
            contacts = p.getContactPoints(self.robot_simulator.ballbot.robot, self.robot_simulator.environment.cubeId2)
            if len(contacts) > 0 or self.contacted_block:
                if not self.contacted_block: # First time contact is made the reach skill converts to lift skills and gripper closes
                    skill_t = 0
                self.contacted_block = True
                self.robot_simulator.barrett_right_hand_joint_command = self.gripper_close_state
                right_arm_pos_traj = self.right_arm_pos_traj_lift.copy()
                right_arm_quat_traj = self.right_arm_quat_traj_lift.copy()
            if skill_t > self.steps_reach-1:
                break
            
            self.robot_simulator.step()
            p.stepSimulation()
            time.sleep(SIMULATION_TIME_STEP_S)

            # if i % int(SENSOR_TIME_STEP_S/SIMULATION_TIME_STEP_S) == 0:
            if i % self.img_save_freqs['I3_save_freq'] == 0:
                self.robot_simulator.update_sensors()
                if self.robot_simulator.enable_turret_camera:
                    vid_turret.append(self.robot_simulator.turretCameraFeedback)
                if self.robot_simulator.enable_static_camera:
                    vid_static.append(self.robot_simulator.staticCameraFeedback)

            if self.terminate_skill():
                success = True
                break

            if self.debug:
                p.addUserDebugLine(right_arm_pos_traj[skill_t-1], [right_arm_pos_traj[skill_t-1,0], right_arm_pos_traj[skill_t-1,1], right_arm_pos_traj[skill_t-1,2]+0.03],
                                    [0, 1, 0], 20, 0, replaceItemUniqueId=right_arm_goal_marker)
                # p.addUserDebugLine(right_ee_frame_info[0], [right_ee_frame_info[0][0], right_ee_frame_info[0][1], right_ee_frame_info[0][2]+0.03],
                #                     [0, 0, 1], 20, 0, replaceItemUniqueId=toolR_marker)
        
        obs_pose_right = np.stack(obs_pose_right, axis=0)
        des_pose_right = np.stack(des_pose_right, axis=0)
        torque_right = np.stack(torque_right, axis=0)
        if self.debug:
            # save gif
            if self.robot_simulator.enable_turret_camera:
                cl = ImageSequenceClip(vid_turret, fps=10)
                cl.write_gif('./media/turret_Camera.gif', fps=10, logger=None)
                del cl
            if self.robot_simulator.enable_static_camera:
                cl = ImageSequenceClip(vid_static, fps=10)
                cl.write_gif('./media/static_Camera.gif', fps=10, logger=None)
                del cl
            
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
            p.removeAllUserDebugItems()

        traj_info = {
            'observed_pos': obs_pose_right,
            'desired_pos': des_pose_right,
            'observed_contact': np.array(obs_contact),
            'eye_in_hand_90': vid_turret,
            'left_cap2': vid_static,
            'success': success
        }
        return traj_info