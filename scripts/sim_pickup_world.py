from robot.robot_simulator_pickup_env import RobotSimulatorPickup
from robot.definitions import *
from skills.composite_skills import PickUpBlockSkill
from omegaconf import OmegaConf 
from moviepy.editor import ImageSequenceClip

if __name__ == "__main__":
    # set pybullet environment
    env_cfg = OmegaConf.load('/home/saumyas/sims/ballbot_ws/src/ballbot_pybullet_sim/src/config/envs/ballbot_pickup_env.yaml')
    task_cfg = {'save_freqs': env_cfg['data_collection']['save_freqs']}
    robot_simulator = RobotSimulatorPickup(env_cfg=env_cfg, task_cfg=task_cfg)

    ## ============= Pickup object using task space control ===================
    vid_static = []
    robot_simulator.reset()
    skill = PickUpBlockSkill(robot_simulator, img_save_freqs=env_cfg['data_collection']['save_freqs'])
    traj_info = skill.execute()
    vid_static.extend(traj_info['vid_static_sideview'])

    print("Finished executing. Saving video now")

    # Video saving
    cl = ImageSequenceClip(vid_static, fps=10)
    cl.write_gif('./media/vid_static_sideview_test2.gif', fps=10, logger=None)
    del cl