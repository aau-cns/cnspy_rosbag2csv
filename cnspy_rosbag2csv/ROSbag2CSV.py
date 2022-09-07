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
import os
import argparse
import yaml
import csv
from tqdm import tqdm

from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType
from cnspy_rosbag2csv.ROSMsg2CSVLine import ROSMsg2CSVLine
from cnspy_rosbag2csv.ROSMessageTypes import ROSMessageTypes


class ROSbag2CSV:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_name, topic_list, result_dir="", fn_list=[], verbose=False, fmt=CSVSpatialFormatType.TUM,
                est_err_type=EstimationErrorType.none,
                err_rep=ErrorRepresentationType.none,
                ):
        """"
        Extracts a list of topic from a rosbag file and stores each topic in a file specified in "fn_list"


        Example:
        >> args.bagfile  = "example.bag"
        >> args.topics =  ["/CS_200_MAV1/estimated_poseWithCov",  "/pose_sensor/pose"]
        >> args.verbose = True
        >> args.result_dir = "./results"
        >> args.filenames = ["mav_PoseWithCov.csv", "sensor_PoseWithCov"]
        >> args.format = CSVSpatialFormatType.PoseWithCov
        >> ROSbag2CSV.extract(bagfile_name=args.bagfile, topic_list=args.topics,
                      fn_list=args.filenames, result_dir=args.result_dir,
                      verbose=args.verbose, fmt=CSVSpatialFormatType(args.format)):


        Input:
        bagfile_name -- file name of the rosbag
        topic_list -- list of topic names, these must start with a "/" (absolute topic name)
        result_dir  -- root directory the files to be created (defined in fn_list)
        fn_list -- list of file names;  (not the entire path!) just name with or without extension


        Output:
        res -- boolean about success
        """

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
            print("* CSV format: \t " + str(fmt))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("ROSbag2CSV: Unexpected error!")

            return False

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSbag2CSV: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## create result dir:
        if result_dir == "":
            folder = str.rstrip(bagfile_name, ".bag")
        else:
            folder = result_dir

        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(folder))

        ## create csv file according to the topic names:
        dict_file_writers = dict()
        dict_header_written = dict()
        dict_csvfile_hdls = dict()
        idx = 0
        for topicName in topic_list:

            if topicName[0] != '/':
                print("ROSbag2CSV: Not a proper topic name: %s (should start with /)" % topicName)
                continue

            if not fn_list:
                filename = str(folder + '/') + str.replace(topicName[1:], '/', '_') + '.csv'
            else:
                fn = fn_list[idx]
                [root, ext] = os.path.splitext(fn)
                [head, tail] = os.path.split(root)
                if ext:
                    filename = str(folder + '/') + tail + ext
                else:
                    filename = str(folder + '/') + tail + '.csv'

            csvfile = open(filename, 'w+')
            file_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
            dict_file_writers[topicName] = file_writer
            dict_header_written[topicName] = False
            dict_csvfile_hdls[topicName] = csvfile

            if verbose:
                print("ROSbag2CSV: creating csv file: %s " % filename)

            idx = idx + 1

        ## check if desired topics are in the bag file:
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

        ## extract the desired topics from the BAG file
        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in topic_list:
                message_type = ROSMessageTypes.get_message_type(msg)
                if message_type != ROSMessageTypes.NOT_SUPPORTED:
                    file_writer = dict_file_writers[topic]

                    if not dict_header_written[topic]:
                        file_writer.writerow(CSVSpatialFormatType.get_header(fmt))
                        dict_header_written[topic] = True

                    # HINT: all conversions are done in ROSMsg2CSVLine
                    content = ROSMsg2CSVLine.to(fmt, msg, t, message_type,
                                                est_err_type=est_err_type, err_rep=err_rep)

                    if content is not None:
                        file_writer.writerow(content)

        ## CLEANUP:
        # close all csv files
        for topicName in topic_list:
            dict_csvfile_hdls[topicName].close()

        # check if a topic was found by checking if the topic header was written
        for topicName in topic_list:
            if not dict_header_written[topicName]:
                print("\nROSbag2CSV: \n\tWARNING topic [" + str(topicName) + "] was not in bag-file")
                print("\tbag file [" + str(bagfile_name) + "] contains: ")
                # print(info_dict['topics'])
                for t in info_dict['topics']:
                    print(t['topic'])
                return False

        if verbose:
            print("\nROSbag2CSV: extracting done! ")

        bag.close()
        return True


if __name__ == "__main__":
    # test3: python3 ROSbag2CSV.py --bagfile ../test/example.bag --topics /CS_200_MAV1/estimated_poseWithCov  /pose_sensor/pose --verbose --filename mav_PoseWithCov.csv sensor_PoseWithCov.csv --format PoseWithCov
    # test4: python3 ROSbag2CSV.py --bagfile ./sample_data/empty_bag.bag --topics /uwb_trilateration/tagDistance_raw /pose_sensor/pose /fcu/current_pose --verbose  --filenames uwb /rasdf/body_pose imu_pose.csv
    # test5: python3 ROSbag2CSV.py --bagfile ./sample_data/dummy.bag --topics /pose_est /pose_gt --verbose  --filenames est gt --format TUM
    parser = argparse.ArgumentParser(
        description='ROSbag2CSV: extract and store given topics of a rosbag into a CSV file')
    parser.add_argument('--bagfile', help='input bag file', default="not specified")
    parser.add_argument('--topics', nargs='*', help='desired topics', default=[])
    parser.add_argument('--filenames', nargs='*', help='csv filename of corresponding topic', default=[])
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--format', help='CSV format', choices=CSVSpatialFormatType.list(),
                        default=str(CSVSpatialFormatType.TUM))
    parser.add_argument('--est_err_type', help='Estimation error type (e.g. global/local pose)', choices=EstimationErrorType.list(),
                        default=str(EstimationErrorType.none))
    parser.add_argument('--err_rep', help='Error representation type', choices=ErrorRepresentationType.list(),
                        default=str(ErrorRepresentationType.none))
    tp_start = time.time()
    args = parser.parse_args()

    if ROSbag2CSV.extract(bagfile_name=args.bagfile, topic_list=args.topics,
                          fn_list=args.filenames, result_dir=args.result_dir,
                          verbose=args.verbose, fmt=CSVSpatialFormatType(args.format),
                          est_err_type=EstimationErrorType(args.est_err_type),
                          err_rep=ErrorRepresentationType(args.err_rep)
                          ):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))