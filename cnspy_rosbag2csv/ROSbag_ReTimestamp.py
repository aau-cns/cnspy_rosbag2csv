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


class ROSbag_ReTimestamp:
    def __init__(self):
        pass

    @staticmethod
    def extract(outbag_name,
                inbag_name,
                verbose=False,
                offset_sec=0.0,
                use_header_timestamp=True):
        if not os.path.isfile(inbag_name):
            print("ROSbagMerge: %s is no file" % inbag_name)
            return False

        outbag_name = os.path.abspath(outbag_name)
        if verbose:
            print("ROSbagMerge:")
            print("* output bagfile name: " + str(outbag_name))
            print("* input bagfile name: : \t " + str(inbag_name))
            print("* use_header_timestamp: : \t " + str(use_header_timestamp))
        pass

        with rosbag.Bag(outbag_name, 'w') as outbag:
            input_bag_name = os.path.abspath(inbag_name)
            if os.path.isfile(input_bag_name) and input_bag_name.endswith(".bag"):
                if verbose:
                    print("* merging bag %s into %s" % (input_bag_name, outbag_name))
                try:
                    bag = rosbag.Bag(input_bag_name)
                except:
                    print("ROSbagMerge: Unexpected error while opening bag file %s!" % input_bag_name)
                    return False

                # get meta info about loaded bag file
                info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)
                if info_dict is None or 'messages' not in info_dict:
                    if verbose:
                        print("ROSbagMerge: Unexpected error, bag file might be empty!")
                    bag.close()
                    return False

                num_messages = info_dict['messages']
                if verbose:
                    print("\nROSbagMerge: %s contains num messages %s" % (input_bag_name, str(num_messages)))

                # merge all topics...
                for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                    if use_header_timestamp and hasattr(msg, "header"):
                        outbag.write(topic, msg, msg.header.stamp)
                    else:
                        outbag.write(topic, msg, t)

            pass
        return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='ROSbag_ReTimestamp: sets the rosbag time to the timestamp of the msgs header')
    parser.add_argument('--outbag_name', help='name of created bag file', default="output.bag")
    parser.add_argument('--inbag_name', help='name of original bag file',
                        required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    #parser.add_argument('--offset_sec', help='offest added to the timestamp', default=0)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=True)

    tp_start = time.time()
    args = parser.parse_args()

    if ROSbag_ReTimestamp.extract(outbag_name=args.outbag_name,
                           inbag_name=args.inbag_name,
                           offset_sec=0.0, #float(args.offset_sec),
                           verbose=args.verbose,
                           use_header_timestamp=args.use_header_timestamp):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))