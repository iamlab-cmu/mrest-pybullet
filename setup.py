# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['controllers', 'environments', 'robot', 'sensors'],
    scripts=[''],
    package_dir={'': 'src'}
)

setup(**setup_args)
