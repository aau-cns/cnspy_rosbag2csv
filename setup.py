#!/usr/bin/env python

from distutils.core import setup

setup(name='rosbag2csv',
      version='1.0',
      description='Python Distribution Utilities',
      author='Roland Jung',
      author_email='roland.jung@aau.at',
      url='https://gitlab.aau.at/aau-cns/py3_pkgs/rosbag2csv/',
      packages=['distutils', 'distutils.command', 'numpy', 'tqdm', 'pandas', 'argparse', 'PyYAML', 'spatialmath-python', 'rospy', 'rospkg', 'pycryptodomex', 'pycryptodome', 'gnupg', 'lz4', 'script_utils', 'spatial_csv_formats'],
     )
