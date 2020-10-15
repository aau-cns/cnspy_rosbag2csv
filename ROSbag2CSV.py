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

from ROSMsg2CVSLine import ROSMsg2CSVLine
from ros_csv_formats.CSVFormat import CSVFormat
from ROSMessageTypes import ROSMessageTypes

from script_utils.utils import *


class ROSbag2CSV:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_name, topic_list, result_dir="", fn_list=[], verbose=False, format='TUM'):
        if not os.path.isfile(bagfile_name):
            print("ROSbag2CSV: could not find file: %s" % bagfile_name)
            return False

        if len(topic_list) < 1:
            print("ROSbag2CSV: no topics specified!")
            return False

        if fn_list:
            if len(topic_list) != len(fn_list):
                print("ROSbag2CSV: topic_list and fn_list must have the same length!")
                return False

        if verbose:
            print("ROSbag2CSV:")
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
                print("ROSbag2CSV: Not a propper topic name: %s (should start with /)" % topicName)
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
                print("ROSbag2CSV: creating csv file: %s " % filename)

            idx = idx + 1
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("ROSbag2CSV: Unexpected error!")
            return False

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']
        for topicName in topic_list:
            found = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found = True

            if not found:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        if verbose:
            print("\nROSbag2CSV: num messages " + str(num_messages))

        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in topic_list:
                message_type = ROSMessageTypes.get_message_type(msg)
                if message_type != ROSMessageTypes.NOT_SUPPORTED:
                    file_writer = topic_filewriter[topic]

                    if not topic_headerwritten[topic]:
                        file_writer.writerow(CSVFormat.get_header(format))
                        topic_headerwritten[topic] = True

                    # TODO: all conversions are done in ROSMsg2CSVLine
                    content = ROSMsg2CSVLine.to(format, msg, t, message_type)

                    if content is not None:
                        file_writer.writerow(content)

        # check if a topic was found by checking if the topic header was written
        for topicName in topic_list:
            if not topic_headerwritten[topicName]:
                print("\nROSbag2CSV: \n\tWARNING topic [" + str(topicName) + "] was not in bag-file")
                print("\tbag file [" + str(bagfile_name) + "] constains: ")
                # print(info_dict['topics'])
                for t in info_dict['topics']:
                    print(t['topic'])

            return False

        if verbose:
            print("\nROSbag2CSV: extracting done! ")
        return True


if __name__ == "__main__":
    # test1: --bagfile ../test/example.bag --topics /uwb_trilateration/tagDistance_raw /pose_sensor/pose /fcu/current_pose --verbose  --filenames uwb /rasdf/body_pose imu_pose.csv
    # test2: --bagfile ../test/example.bag --topics /CS_200_MAV1/estimated_poseWithCov  /pose_sensor/pose --verbose --filename mav_PoseCov.csv sensor_PoseCov.csv --format PoseCov
    # test3: --bagfile ../test/example.bag --topics /CS_200_MAV1/estimated_poseWithCov  /pose_sensor/pose --verbose --filename mav_PoseWithCov.csv sensor_PoseWithCov.csv --format PoseWithCov

    parser = argparse.ArgumentParser(
        description='ROSbag2CSV: extract and store given topics of a rosbag into a CSV file')
    parser.add_argument('--bagfile', help='input bag file', default="not specified")
    parser.add_argument('--topics', nargs='*', help='desired topics', default=[])
    parser.add_argument('--filenames', nargs='*', help='csv filename of corresponding topic', default=[])
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--format', help='CSV format', choices=CSVFormat.list(),
                        default=str(CSVFormat.TUM))

    tp_start = time.time()
    args = parser.parse_args()

    if ROSbag2CSV.extract(bagfile_name=args.bagfile, topic_list=args.topics,
                          fn_list=args.filenames, result_dir=args.result_dir,
                          verbose=args.verbose, format=CSVFormat(args.format)):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
        exit_success()
    else:
        exit_failure()
