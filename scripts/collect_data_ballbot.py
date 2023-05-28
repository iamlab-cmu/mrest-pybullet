import numpy as np
import os
from omegaconf import OmegaConf
import yaml
from pathlib import Path
from PIL import Image
import pickle
from tqdm import trange
from collections import OrderedDict

from robot.robot_simulator_pickup_env import *
from skills.composite_skills import *

def recursively_filter_noyaml_subdicts(data):
    '''Certain key-values cannot be stored in yaml files. Hence recursively filter them out.
    
    These are marked with the _no_yaml flag.
    '''
    if not isinstance(data, dict) and not isinstance(data, OrderedDict):
        return data

    no_yaml = data.get('_no_yaml', False)
    if no_yaml:
        return None

    no_yaml_data = {}
    for k, v in data.items():
        no_yaml_data[k] = recursively_filter_noyaml_subdicts(v)
        if no_yaml_data[k] is None:
            del no_yaml_data[k]
    return no_yaml_data

def save_env_config(env_config, data_dir: Path, config_name: str):
    '''Save env config as pickle and yaml file for quick read.'''
    pkl_path = data_dir / (config_name + '.pkl')
    with open(pkl_path, 'wb') as pkl_f:
        pickle.dump(env_config, pkl_f, protocol=4)
    yaml_path = data_dir / (config_name + '.yaml')
    with open(yaml_path, 'w') as yaml_f:
        # yaml.dump(env_config, yaml_f, default_flow_style=False)
        env_config = recursively_filter_noyaml_subdicts(env_config)
        yaml.safe_dump(env_config, yaml_f, default_flow_style=False)

def save_traj(traj_info, save_dir, env_name, save_freq, demo_num):
    
    save_dir_demo = save_dir / f'demo{demo_num}'
    save_dir_demo.mkdir()

    observed_pos = traj_info['observed_pos'][::save_freq, :]
    observed_contact = traj_info['observed_contact'][::save_freq].reshape(-1,1)

    actions_obs = observed_pos[1:] - observed_pos[:-1]
    actions_obs = np.append(actions_obs, np.zeros((1,observed_pos.shape[1])), axis=0)
    actions_des = traj_info['desired_pos'][::save_freq, :]
    actions_des_delta = (traj_info['desired_pos'] - traj_info['observed_pos'])[::save_freq, :]
    
    non_image_data = {
            'observations': np.append(observed_pos, observed_contact, axis=1),
            'actions_obs': np.append(actions_obs, observed_contact, axis=1),
            'actions_des': np.append(actions_obs, observed_contact, axis=1),
            'actions_des_delta': np.append(actions_obs, observed_contact, axis=1),
        }
    
    with open(save_dir_demo / 'info.pickle', 'wb') as f:
        pickle.dump(non_image_data, f)
    
    T = observed_pos.shape[0] # Images are already returned at the lowered frequency to make data collection faster
    for camera_name in ['left_cap2', 'eye_in_hand_90']:
        save_dir_imgs = save_dir_demo / camera_name
        save_dir_imgs.mkdir()
        for t in range(T):
            tmp_img = Image.fromarray(traj_info[camera_name][t].astype(np.uint8))
            tmp_img.save(save_dir_imgs / f'img_t{t}.png')

def generate_env_configs(cfg):
    # TODO: picking up only cube2 for now which is red in color
    # TODO: only one skill so far 'pick'
    env_configs = []
    env_config = OrderedDict(
        target_object='blockA',
        blockA_config='red',
        blockB_config='blue',
        
        task_command_color='Pick red block',
        task_command_size='Pick red block',
        
        skill='pick',
        # TODO: For now we set the skill as the task can change this later?
        task='pick',
        task_command_type='Pick red block',
        task_command_lang='Pick red block',

        only_use_block_objects=True,
        num_demos_per_env=cfg['data_collection']['pick']['num_demos_per_env'],
        update_block_colors=True,
        camera_names=['left_cap2', 'eye_in_hand_90'],
        )
    env_configs.append(env_config.copy())
    return env_configs

def get_env_name(env_cfg):
    # TODO: fix for push off task
    skill = env_cfg['skill']
    target_block_color = env_cfg['blockA_config']
    env_name = f'env_ballbot_{skill}_{target_block_color}_block'
    return env_name

def collect_and_save_demo_data_for_env_config(robot_simulator, data_dir, env_name, env_cfg, save_freq):
    save_dir = data_dir / env_name
    save_dir.parent.mkdir(exist_ok=True)
    save_dir.mkdir()
    save_traj_idx = 0
    for i in trange(env_cfg['num_demos_per_env']):
        robot_simulator.reset(env_cfg)
        skill = PickUpBlockSkill(robot_simulator, img_save_freq=save_freq)
        skill.debug = False
        traj_info = skill.execute()
        if traj_info['success']:
            save_traj_idx +=1
            save_traj(traj_info, save_dir, env_name, save_freq, save_traj_idx)

def collect_data(num_trajs, main_data_dir, data_type, save_freq):
    cfg = OmegaConf.load('/home/saumyas/ballbot_sim_py3_ws/src/ballbot_pybullet_sim/src/config/envs/ballbot_pickup_env.yaml')
    robot_simulator = RobotSimulatorPickup(env_cfg=cfg)
    
    data_dir = main_data_dir / data_type
    if not data_dir.exists():
        os.makedirs(data_dir)
    
    env_configs = generate_env_configs(cfg)
    to_save_env_configs = OrderedDict()
    for env_cfg in env_configs:
        env_name = get_env_name(env_cfg)
        to_save_env_configs[env_name] = env_cfg
        # collect_and_save_demo_data_for_env_config(robot_simulator, data_dir, env_name, env_cfg, save_freq)

    save_env_config(to_save_env_configs, main_data_dir, f'{data_type}_env_configs')
    print(f"Did save demos at: {main_data_dir}")

if __name__ == "__main__":
    main_data_dir = Path('/home/saumyas/experiment_results/object_centric/r3m/data/ballbot_pickup_slow/train')
    if not main_data_dir.exists():
        os.makedirs(main_data_dir)
    
    data_types = ['train']
    save_freq = 24
    num_demos = 500
    
    for data_type in data_types:
        collect_data(num_demos, main_data_dir, data_type, save_freq)