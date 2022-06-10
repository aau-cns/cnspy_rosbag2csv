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
########################################################################################################################
import rosbag
import time
from std_msgs.msg import Header, Time
import os
import argparse
import yaml
import csv
from tqdm import tqdm

from cnspy_rosbag2csv.CSVLine2ROSMsg import CSVLine2ROSMsg
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
import cnspy_script_utils.utils as script_utils

from cnspy_rosbag2csv.ROSMessageTypes import ROSMessageTypes
from cnspy_rosbag2csv.CSVParser import CSVParser


class CSV2ROSbag:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_name, topic_list, fn_list, fmt_list, result_dir=".", verbose=False):
        if len(topic_list) < 1:
            print("CSVROSbag: no topics specified!")
            return False
        if len(topic_list) != len(fn_list) or len(topic_list) != len(fmt_list):
            print("CSVROSbag: topic_list[{0}], fn_list[{1}], and fmt_list[{2}] must match!".format(len(topic_list),
                                                                                                   len(fn_list),
                                                                                                   len(fmt_list)))
            return False
        elif verbose:
            for i in range(0, len(topic_list)):
                print("match: {0} <-> {1} <->  {2}".format(topic_list[i], fn_list[i], fmt_list[i]))

        if any(not os.path.isfile(x) for x in fn_list):
            print("CSVROSbag: could not find a file: %s" % str(fn_list))
            return False

        if any(not isinstance(x, ROSMessageTypes) for x in fmt_list):
            print("CSVROSbag: not a ROSMessageTypes: %s" % str(fmt_list))
            return False

        if any(topicName[0] != '/' for topicName in topic_list):
            print("CSVROSbag:  Not a propper topic name: %s (should start with /)" % str(topic_list))
            return False

        if result_dir == "" or result_dir is None:
            fn = fn_list[0]
            result_dir, tail = os.path.split(os.path.abspath(fn))

        else:
            if not os.path.exists(result_dir):
                os.makedirs(os.path.abspath(result_dir))

        fn = os.path.join(os.path.abspath(result_dir), bagfile_name)
        if verbose:
            print("CSVROSbag:")
            print("* result_dir: " + str(result_dir))
            print("* bagfile name: " + str(fn))
            print("* topic_list: \t " + str(topic_list))
            print("* filename_list: " + str(fn_list))
            print("* ROS msg type: \t " + str(fmt_list))

        parser_dict = {}
        idx = 0
        for topicName in topic_list:
            parser_dict[topicName] = CSVParser(fn=fn_list[idx], msg_type=fmt_list[idx])
            if verbose:
                print("CSVROSbag: creating CSVParser for: %s reading from %s " % (topicName, fn_list[idx]))
            idx += 1

        h = Header()
        bag = rosbag.Bag(fn, 'w')
        while not all(value.done for key, value in parser_dict.items()):

            t_min = float("inf")
            tpc = None
            for key, value in parser_dict.items():
                # check if the CSV file is not done:
                # -- currently we will miss the last line of the csv file!
                # -- because once the last line is with next_line read, this flag is set!
                # and if the current time stamp is the smallest
                if not value.done and value.t < t_min:
                    tpc = key
                    t_min = value.t

            if tpc is not None:
                h.stamp = h.stamp.from_sec(t_min)
                bag.write(tpc, parser_dict[tpc].curr_msg, t=h.stamp)
                good = parser_dict[tpc].next_line()
                if not good and verbose:
                    fn = parser_dict[tpc].fn
                    n = parser_dict[tpc].line_number
                    print("CSVROSbag: DONE with topic {0} from file {1} (num. lines={2})!".format(tpc, fn, n))

        if verbose:
            info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)
            print("CSVROSbag: bag info: \n")
            print(info_dict)
        bag.close()
        return True



if __name__ == "__main__":
    # --bagfile_name dummy.bag --topics /pose_est /pose_gt --filenames ./sample_data/ID1-pose-est-posorient-cov.csv ./sample_data/ID1-pose-gt.csv --fmt_list GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED GEOMETRY_MSGS_POSESTAMPED --verbose

    parser = argparse.ArgumentParser(
        description='CSV2ROSbag: read CSV files and convert lines to specified ROS msg and store them into a rosbag')
    parser.add_argument('--bagfile_name', help='name of bag file (no path!)', default="not specified")
    parser.add_argument('--topics', nargs='*', help='topics to create', default=[])
    parser.add_argument('--filenames', nargs='*', help='csv filename of corresponding topic', default=[])
    parser.add_argument('--fmt_list', nargs='*', type=ROSMessageTypes, help='CSV format', choices=list(ROSMessageTypes),
                        default=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)

    tp_start = time.time()
    args = parser.parse_args()

    if CSV2ROSbag.extract(bagfile_name=args.bagfile_name, topic_list=args.topics,
                          fn_list=args.filenames, fmt_list=args.fmt_list, result_dir=args.result_dir,
                          verbose=args.verbose):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
        script_utils.exit_success()
    else:
        script_utils.exit_failure()
