#!/usr/bin/env python

from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()



setup(
    name='cnspy_rosbag2csv',
    version="0.2.5",
    author='Roland Jung',
    author_email='roland.jung@aau.at',    
    description='ROS1 rosbag to CSV file converter and vice versa.',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://github.com/aau-cns/cnspy_rosbag2csv/',
    project_urls={
        "Bug Tracker": "https://github.com/aau-cns/cnspy_rosbag2csv/issues",
    },    
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
    ],
    
    packages=find_packages(exclude=["test_*", "TODO*"]),
    python_requires='>=3.6',
    install_requires=['numpy', 'tqdm', 'pandas', 'argparse', 'PyYAML', 'rospkg', 'spatialmath-python', 'pycryptodomex', 'pycryptodome', 'gnupg', 'lz4', 'cnspy_spatial_csv_formats>=0.2.2'],
    entry_points={
        'console_scripts': [
            'CSV2ROSbag = cnspy_rosbag2csv.CSV2ROSbag:main',
            'ROSbag2CSV = cnspy_rosbag2csv.ROSbag2CSV:main',
            'ROSbag_ReTimestamp = cnspy_rosbag2csv.ROSbag_ReTimestamp:main',
            'ROSbagMerge = cnspy_rosbag2csv.ROSbagMerge:main',

        ],
    },
)
