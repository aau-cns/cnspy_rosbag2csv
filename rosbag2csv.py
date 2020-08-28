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

import sys
import rosbag
import time
import string
import os
import argparse
import yaml
import csv
from tqdm import tqdm

from CSVFormat import CSVFormat
from ROSMessageType import ROSMessageType

from script_utils.utils import *
from tum_eval.TUMCSVheader import TUMCSVheader


class Rosbag2Csv:
    def __init__(self, ):
        pass

    @staticmethod
    def extract(bagfile_name, topic_list, result_dir="", fn_list=[], verbose=False, format='TUM'):
        if not os.path.isfile(bagfile_name):
            print("ROSMsg2CSV: could not find file: %s" % bagfile_name)
            return False

        if len(topic_list) < 1:
            print("ROSMsg2CSV: no topics specified!")
            return False

        if fn_list:
            if len(topic_list) != len(fn_list):
                print("ROSMsg2CSV: topic_list and fn_list must have the same length!")
                return False

        if verbose:
            print("ROSMsg2CSV:")
            print("* bagfile name: " + str(bagfile_name))
            print("* topic_list: \t " + str(topic_list))
            print("* filename_list: " + str(fn_list))
            print("* CSV format: \t " + str(format))

        if result_dir == "":
            folder = string.rstrip(bagfile_name, ".bag")
        else:
            folder = result_dir

        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(folder))

        topic_filewriter = dict()
        topic_headerwritten = dict()

        idx = 0
        for topicName in topic_list:

            if topicName[0] != '/':
                print ("ROSMsg2CSV: Not a propper topic name: %s (should start with /)" % topicName)
                continue

            if not fn_list:
                filename = str(folder + '/') + string.replace(topicName[1:], '/', '_') + '.csv'
            else:
                fn = fn_list[idx]
                [root, ext] = os.path.splitext(fn)
                [head, tail] = os.path.split(root)
                if ext:
                    filename = str(folder + '/') + tail + ext
                else:
                    filename = str(folder + '/') + tail + '.csv'

            csvfile = open(filename, 'w+')
            filewriter = csv.writer(csvfile, delimiter=',', lineterminator='\n')
            topic_filewriter[topicName] = filewriter
            topic_headerwritten[topicName] = False

            if verbose:
                print ("ROSMsg2CSV: creating csv file: %s " % filename)

            idx = idx + 1

        bag = rosbag.Bag(bagfile_name)
        info_dict = yaml.load(bag._get_yaml_info())

        num_messages = info_dict['messages']

        if verbose:
            print("\nROSMsg2CSV: num messages " + str(num_messages))

        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in args.topics:
                message_type = ROSMessageType.get_message_type(msg)
                if message_type != ROSMessageType.NOT_SUPPORTED:
                    file_writer = topic_filewriter[topic]

                    if not topic_headerwritten[topic]:
                        file_writer.writerow(TUMCSVheader.pose_stamped())
                        topic_headerwritten[topic] = True

                    # TODO: add more message_to_xxx options
                    content = None
                    if format == CSVFormat.TUM:
                        content = CSVFormat.message_to_tum(msg, t, message_type)
                    elif format == CSVFormat.TUM_short:
                        content = CSVFormat.message_to_tum_short(msg, t, message_type)
                    else:
                        print ("ROSMsg2CSV: unsupported format: %s " % str(format))
                        return False

                    file_writer.writerow(content)

        if verbose:
            print("\nROSMsg2CSV: extracting done! ")
        return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Rosbag2Csv: extract and store given topics of a rosbag into a CSV file')
    parser.add_argument('--bagfile', help='input bag file', default="not specified")
    parser.add_argument('--topics', nargs='*', help='desired topics', default=[])
    parser.add_argument('--filenames', nargs='*', help='csv filename of corresponding topic', default=[])
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--format', type=CSVFormat, help='CSV format', choices=list(CSVFormat), default=CSVFormat.TUM)

    tp_start = time.time()
    args = parser.parse_args()

    if Rosbag2Csv.extract(bagfile_name=args.bagfile, topic_list=args.topics,
                          fn_list=args.filenames, result_dir=args.result_dir,
                          verbose=args.verbose, format=args.format):
        print (" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
        exit_success()
    else:
        exit_failure()
