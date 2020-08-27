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
from enum import Enum
from tqdm import tqdm

from script_utils.utils import *
from tum_eval.tum_csv_header import tum_csv_header


class ROSMessageType(Enum):
    NOT_SUPPORTED = 0
    # GEOMETRY_MSGS_POINT = 3 == VECTOR3
    GEOMETRY_MSGS_POINTSTAMPED = 1  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PointStamped.html
    GEOMETRY_MSGS_VECTOR3 = 2  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html
    GEOMETRY_MSGS_VECTOR3STAMPED = 3  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html
    GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED = 4  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
    GEOMETRY_MSGS_POSEWITHCOVARIANCE = 5  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html
    GEOMETRY_MSGS_POSESTAMPED = 6  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
    GEOMETRY_MSGS_POSE = 7  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
    GEOMETRY_MSGS_QUATERNION = 8  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html
    GEOMETRY_MSGS_QUATERNIONSTAMPED = 9  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/QuaternionStamped.html
    GEOMETRY_MSGS_TRANSFORM = 10  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Transform.html
    GEOMETRY_MSGS_TRANSFORMSTAMPED = 11  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html


class CSVformat(Enum):
    TUM = 'TUM'
    none = 'none'

    def __str__(self):
        return self.value


def get_message_type(msg_):
    """

    :rtype: ROSMessageType
    """

    if hasattr(msg_, 'header'):  # STAMPED
        if hasattr(msg_, 'pose') and hasattr(msg_.pose, 'covariance') and hasattr(msg_.pose, 'pose'):
            return ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED

        if hasattr(msg_, 'pose') and hasattr(msg_.pose, 'position'):
            return ROSMessageType.GEOMETRY_MSGS_POSESTAMPED

        if hasattr(msg_, 'point'):
            return ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED

        if hasattr(msg_, 'vector'):
            return ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED

        if hasattr(msg_, 'transform'):
            return ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED

        if hasattr(msg_, 'quaternion'):
            return ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED

    else:  # NOT STAMPED
        if hasattr(msg_, 'pose') and hasattr(msg_, 'covariance'):
            return ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE

        if hasattr(msg_, 'position') and hasattr(msg_, 'orientation'):
            return ROSMessageType.GEOMETRY_MSGS_POSE

        if hasattr(msg_, 'translation') and hasattr(msg_, 'rotation'):
            return ROSMessageType.GEOMETRY_MSGS_TRANSFORM

        if hasattr(msg_, 'x') and hasattr(msg_.pose, 'y') and hasattr(msg_, 'z') and hasattr(msg_, 'w'):
            return ROSMessageType.GEOMETRY_MSGS_QUATERNION

        if hasattr(msg_, 'x') and hasattr(msg_.pose, 'y') and hasattr(msg_, 'z'):
            return ROSMessageType.GEOMETRY_MSGS_VECTOR3

    return ROSMessageType.NOT_SUPPORTED


def message_to_tum(msg_, t_, msg_type=ROSMessageType.NOT_SUPPORTED):
    """

    :rtype: list of floats
    """

    if msg_type == ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED:
        return ["%f" % msg_.header.stamp.to_sec(), str(msg_.point.x), str(msg_.point.y), str(msg_.point.z), "0", "0",
                "0", "1"]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_VECTOR3:
        t = float(t_.secs) + float(t_.nsecs) * 1e-9
        return ["%f" % str(t), str(msg_.x), str(msg_.y), str(msg_.z), "0", "0", "0", "1"]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED:
        return ["%f" % msg_.header.stamp.to_sec(), str(msg_.vector.x), str(msg_.vector.y),
                str(msg_.vector.z), "0", "0", "0", "1"]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
        return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.pose.position.x), str(msg_.pose.pose.position.y),
                str(msg_.pose.pose.position.z), str(msg_.pose.pose.orientation.x),
                str(msg_.pose.pose.orientation.y), str(msg_.pose.pose.orientation.z),
                str(msg_.pose.pose.orientation.w)]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
        t = float(t_.secs) + float(t_.nsecs) * 1e-9
        return ["%f" % str(t), str(msg_.pose.position.x), str(msg_.pose.position.y),
                str(msg_.pose.position.z), str(msg_.pose.orientation.x),
                str(msg_.pose.orientation.y), str(msg_.pose.orientation.z),
                str(msg_.pose.orientation.w)]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
        return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.position.x), str(msg_.pose.position.y),
                str(msg_.pose.position.z), str(msg_.pose.orientation.x), str(msg_.pose.orientation.y),
                str(msg_.pose.orientation.z), str(msg_.pose.orientation.w)]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_POSE:
        t = float(t_.secs) + float(t_.nsecs) * 1e-9
        return ["%f" % str(t), str(msg_.position.x), str(msg_.position.y),
                str(msg_.position.z), str(msg_.orientation.x), str(msg_.orientation.y),
                str(msg_.orientation.z), str(msg_.orientation.w)]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNION:
        t = float(t_.secs) + float(t_.nsecs) * 1e-9
        return ["%f" % str(t), "0", "0", "0", str(msg_.x), str(msg_.y), str(msg_.z), str(msg_.w)]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED:
        return ["%f" % msg_.header.stamp.to_sec(), "0", "0", "0", str(msg_.pose.quaternion.x),
                str(msg_.pose.quaternion.y), str(msg_.pose.quaternion.z), str(msg_.pose.quaternion.w)]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORM:
        t = float(t_.secs) + float(t_.nsecs) * 1e-9
        return ["%f" % str(t), str(msg_.translation.x), str(msg_.translation.y),
                str(msg_.translation.z), str(msg_.rotation.x), str(msg_.rotation.y),
                str(msg_.rotation.z), str(msg_.rotation.w)]

    if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED:
        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.transform.translation.x),
                    str(msg_.transform.translation.y),
                    str(msg_.transform.translation.z), str(msg_.transform.rotation.x), str(msg_.transform.rotation.y),
                    str(msg_.transform.rotation.z), str(msg_.transform.rotation.w)]

    # else:
    return []


class Rosbag2Csv:
    def __init__(self, ):
        pass

    @staticmethod
    def extract(bagfile_name, topic_list, result_dir="", fn_list=[], verbose=False, format='TUM'):
        if not os.path.isfile(bagfile_name):
            print("ROSMsg2CSV: could not find file %s" % bagfile_name)
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
            print("* bagfile name: \t" + str(bagfile_name))
            print("* topic_list: \t" + str(topic_list))

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
            print("* result_dir: \t" + str(folder))

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
                filename = fn_list[idx]

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
            print("ROSMsg2CSV: num messages " + str(num_messages))

        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in args.topics:
                message_type = get_message_type(msg)
                if message_type != ROSMessageType.NOT_SUPPORTED:
                    file_writer = topic_filewriter[topic]

                    if not topic_headerwritten[topic]:
                        file_writer.writerow(tum_csv_header.pose_stamped())
                        topic_headerwritten[topic] = True

                    # TODO: add more message_to_xxx options
                    content = None
                    if format == CSVformat.TUM:
                        content = message_to_tum(msg, t, message_type)
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
    parser.add_argument('--format', type=CSVformat, help='CSV format', choices=list(CSVformat), default=CSVformat.TUM)

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
