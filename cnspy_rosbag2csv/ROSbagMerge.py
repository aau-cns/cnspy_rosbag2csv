#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2023, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
from tqdm import tqdm


class ROSbagMerge:
    def __init__(self):
        pass

    @staticmethod
    def on_white_list(topic, white_list):
        if white_list is None:
            return True
        else:
            for x in white_list:
                if topic.find(x) != -1:
                    return True
        return False


    @staticmethod
    def extract(outbag_name, input_dir=None, input_files=None, verbose=False,
                use_header_timestamp=True, white_list=None):
        if input_dir is not None and input_files is not None:
            print("ROSbagMerge: specify either input_dir or input_files!")
            return False

        if input_files is None:
            if not os.path.isdir(input_dir):
                print("ROSbagMerge: %s is no directory" % input_dir)
                return False
            else:
                input_dir = os.path.abspath(input_dir)
                files = os.listdir(input_dir)
                file_list = []
                for fn in files:
                    input_bag_name = os.path.abspath(os.path.join(input_dir, fn))
                    file_list.append(input_bag_name)

        else:
            for file in input_files:
                if not os.path.isfile(file) or not file.endswith(".bag"):
                    print("ROSbagMerge: %s is not a bag file" % file)
                    return False
            file_list = input_files

        outbag_name = os.path.abspath(outbag_name)
        if verbose:
            print("ROSbagMerge:")
            print("* output bagfile name: " + str(outbag_name))
            if input_dir is not None:
                print("* input_dir: \t " + str(input_dir))

            print("* input files:")
            for x in file_list:
                print("*     %s" % str(x))

            if white_list:
                print("* white_list:")
                for x in white_list:
                    print("*   %s" % str(x))
        pass

        with rosbag.Bag(outbag_name, 'w') as outbag:
            for input_bag_name in file_list:
                if os.path.isfile(input_bag_name) and input_bag_name.endswith(".bag"):
                    if verbose:
                        print("* merging bag %s into %s" % (input_bag_name, outbag_name))
                    try:
                        bag = rosbag.Bag(input_bag_name)
                    except:
                        print("ROSbagMerge: Unexpected error while opening bag file %s!" % input_bag_name)
                        continue

                    # get meta info about loaded bag file
                    info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)
                    if info_dict is None or 'messages' not in info_dict:
                        if verbose:
                            print("ROSbagMerge: Unexpected error, bag file might be empty!")
                        bag.close()
                        continue
                    num_messages = info_dict['messages']
                    if verbose:
                        print("\nROSbagMerge: %s contains num messages %s" % (input_bag_name, str(num_messages)))

                    # merge all topics...
                    for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                        if ROSbagMerge.on_white_list(topic=topic, white_list=white_list):
                            if use_header_timestamp and hasattr(msg, "header"):
                                outbag.write(topic, msg, msg.header.stamp)
                            else:
                                outbag.write(topic, msg, t)

            pass
        return True

#--outbag_name /home/jungr/workspace/datasets/MultiAgentUWB/EuRoC_D140_A0_Mesh0/run1/sim_tp.bag
#--input_dir /home/jungr/workspace/datasets/MultiAgentUWB/EuRoC_D140_A0_Mesh0/run1/bags
#--verbose
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='ROSbagMerge: merge all bag file in a specified directory into one bag file')
    parser.add_argument('--outbag_name', help='name of created bag file', default="output.bag")
    parser.add_argument('--input_dir', help='directory containing bag files to be merged',
                        default=None)
    parser.add_argument('--input_files', type=str, nargs='+',
                        help='a list of files to be merged',
                        default=None)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=False)
    parser.add_argument('-l', '--white_list', type=str, nargs='+',
                        help='white list of topic names or fractions of it that are expected', default=None)
    tp_start = time.time()
    args = parser.parse_args()

    if ROSbagMerge.extract(outbag_name=args.outbag_name,
                           input_dir=args.input_dir,
                           input_files=args.input_files,
                           verbose=args.verbose,
                           use_header_timestamp=args.use_header_timestamp,
                           white_list=args.white_list):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))