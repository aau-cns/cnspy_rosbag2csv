#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2018, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Requirements:
# sudo pip install PyYAML  rospkg catkin_pkg enum tqdm

import sys
import rosbag
import time
import string
import os
import argparse
import yaml
import csv
from enum import Enum
from tqdm import tqdm

from script_utils.utils import *


def TUM_header():
    return ['# timestamp', 'x', 'y', 'z']


def message_string(msg_, t_):
    t = float(t_.secs) + float(t_.nsecs) * 1e-9
    return ["%f" % t, str(msg_.x), str(msg_.y), str(msg_.z)]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Extracting position information from a given bag file and store the topics in a folder named after the bag file, in csv files named after the topic')
    parser.add_argument('--bagfile', help='input bag file', default="not specified")
    parser.add_argument('--topics', nargs='*', help='desired topics', default=[])
    parser.add_argument('--verbose', action='store_true', default=False)

    tp_start = time.time()
    args = parser.parse_args()

    bagfile_name = args.bagfile
    if not os.path.isfile(bagfile_name):
        print("could not find file %s" % bagfile_name)
        exit_failure()

    print(bagfile_name)

    folder = string.rstrip(bagfile_name, ".bag")
    try:  # else already exists
        os.makedirs(folder)
    except:
        pass

    topic_filewriter = dict()
    topic_headerwritten = dict()
    for topicName in args.topics:

        if topicName[0] != '/':
            print ("Not a propper topic name: %s (should start with /)" % topicName)
            continue

        filename = str(folder + '/') + string.replace(topicName[1:], '/', '_') + '.csv'

        csvfile = open(filename, 'w+')
        filewriter = csv.writer(csvfile, delimiter=',', lineterminator='\n')
        topic_filewriter[topicName] = filewriter
        topic_headerwritten[topicName] = False
        print ("creating csv file: %s " % filename)

    bag = rosbag.Bag(bagfile_name)
    info_dict = yaml.load(bag._get_yaml_info())

    num_messages = info_dict['messages']

    if args.verbose:
        print("num messages " + str(num_messages))
        print (info_dict)

    for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
        if topic in args.topics:

            file_writer = topic_filewriter[topic]

            if not topic_headerwritten[topic]:
                file_writer.writerow(TUM_header())
                topic_headerwritten[topic] = True

            content = message_string(msg, t)
            file_writer.writerow(content)

    print (" ")
    print("finished after [%s sec]\n" % str(time.time() - tp_start))
    exit_success()
