#!/usr/bin/env python

from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# Get the release/version string
with open(path.join(here, 'RELEASE'), encoding='utf-8') as f:
    release = f.read()

setup(
    name='rosbag2csv',
    version=release,
    author='Roland Jung',
    author_email='roland.jung@aau.at',    
    description='ROS1 rosbag to CSV file converter and vice versa.',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://gitlab.aau.at/aau-cns/py3_pkgs/rosbag2csv/',
    project_urls={
        "Bug Tracker": "https://gitlab.aau.at/aau-cns/py3_pkgs/rosbag2csv/issues",
    },    
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU GPLv3 License",
        "Operating System :: OS Independent",
    ],
    
    packages=find_packages(exclude=["test_*", "TODO*"]),
    python_requires='>=3.6',
    install_requires=['numpy', 'tqdm', 'pandas', 'argparse', 'PyYAML', 'spatialmath-python', 'rospy', 'rospkg', 'pycryptodomex', 'pycryptodome', 'gnupg', 'lz4', 'script_utils', 'spatial_csv_formats'],
)
