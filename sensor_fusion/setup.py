#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['sensor_fusion'],
    package_dir={'': 'src'},
    install_requires=['filterpy'],
)

setup(**setup_args)
