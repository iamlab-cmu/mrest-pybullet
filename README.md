# Ballbot PyBullet Sim

This package uses PyBullte to simulate the CMU ballbot.

## Installation instructions
 
### Clone and install mrest-pybullet 

```
git clone git@github.com:iamlab-cmu/mrest-pybullet.git
cd mrest-pybullet
pip install -e .
```

### Install orocos_kinematics_dynamics
Follow installation instructions here https://github.com/orocos/orocos_kinematics_dynamics.
```
git clone git@github.com:orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics
git submodule update --init
```
Some useful debugging steps:
```
cp <INSTALL_DIR>/orocos_kinematics_dynamics/python_orocos_kdl/build/PyKDL.so ~/anaconda3/envs/<YOUR_CONDA_ENV>/lib/python3.8/lib-dynload
cp <INSTALL_DIR>/orocos_kinematics_dynamics/python_orocos_kdl/build/PyKDL.so /usr/local/lib
```

### Install quaternion:
```
conda install -c conda-forge quaternion
or
python -m pip install --upgrade --force-reinstall numpy-quaternion
```