#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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

import rosbag
import time
import string
import os
import argparse
import yaml
import csv
from tqdm import tqdm

from CSVLine2ROSMsg import CSVLine2ROSMsg
from ros_csv_formats.CSVFormat import CSVFormat
from ROSMessageTypes import ROSMessageTypes

from script_utils.utils import *


class CSVParser:
    curr_msg = None
    done = False
    t = None
    fmt = None
    file = None
    line_number = 0
    msg_type = None

    def __init__(self, fn, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED):
        self.fmt = CSVFormat.identify_format(fn)
        assert (self.fmt is not CSVFormat.none)
        self.msg_type = msg_type

        self.file = open(fn, "r")
        line = self.file.readline()
        header = str(line).rstrip("\n\r")
        self.line_number = 1

    def __del__(self):
        # Closing files
        self.file.close()

    def next_line(self):
        line = self.file.readline()
        if not line:
            self.done = True
            return False
        line = str(line).rstrip("\n\r")
        self.curr_msg, self.t = CSVLine2ROSMsg.to(self.fmt, line, self.line_number, self.msg_type)
        self.line_number += 1
        return True


class CSV2ROSbag:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_name, topic_list, ros_msg_type, result_dir=".", fn_list=[], verbose=False):
        if len(topic_list) < 1:
            print("CSVROSbag: no topics specified!")
            return False

        if any(not os.path.isfile(x) for x in fn_list):
            print("CSVROSbag: could not find file: %s" % str(fn_list))
            return False

        if not os.path.exists(result_dir):
            os.makedirs(os.path.abspath(result_dir))

        fn = os.path.join(os.path.abspath(result_dir), bagfile_name)
        if verbose:
            print("CSVROSbag:")
            print("* bagfile name: " + str(fn))
            print("* topic_list: \t " + str(topic_list))
            print("* filename_list: " + str(fn_list))
            print("* ROS msg type: \t " + str(ros_msg_type))
        bag = rosbag.Bag(fn, 'w')
