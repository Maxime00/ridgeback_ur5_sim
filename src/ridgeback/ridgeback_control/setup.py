#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ridgeback_control'],
    package_dir={'': 'scripts'},
)

setup(**setup_args)

