# Ballbot PyBullet Sim

This package uses PyBullte to simulate the CMU ballbot.

## Installation instructions
 
### Clone and install mrest-pybullet 

```
git clone git@github.com:iamlab-cmu/mrest-pybullet.git
cd mrest-pybullet
pip install -e .
```

### Install dependencies
```
git clone git@github.com:orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics
git submodule update --init
```
Follow installation instructions here https://github.com/orocos/orocos_kinematics_dynamics.

```
conda install -c conda-forge quaternion
or
python -m pip install --upgrade --force-reinstall numpy-quaternion
```

```
pip install urdf-parser-py
```