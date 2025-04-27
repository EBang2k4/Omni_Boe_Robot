## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
<<<<<<< HEAD
    packages=['omni_diff_teleop'],
=======
    packages=['turtlebot3_teleop'],
>>>>>>> 4ec3c541ead7fec1064bdf766b353f26eccd63dd
    package_dir={'': 'src'}
)

setup(**setup_args)
