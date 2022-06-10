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

import os
import unittest
from cnspy_rosbag2csv.ROSbag2CSV import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class ROSbag2CSV_Test(unittest.TestCase):
    def test_convert2PosOrientCov(self):
        fn_list = ['/mav_PosOrientWithCov.csv', '/sensor_PosOrientWithCov.csv']
        topic_list = ['/pose_est', '/pose_gt']

        bagfile = str(SAMPLE_DATA_DIR + '/example.bag')    
        self.assertTrue(ROSbag2CSV.extract(bagfile_name=bagfile, topic_list=topic_list,
                                           fn_list=fn_list, result_dir=str(SAMPLE_DATA_DIR + '/results'),
                                           verbose=True, fmt=CSVSpatialFormatType('PosOrientWithCov')))

    def test_convert2PosCov(self):
        fn_list = ['/mav_PoseWithCov.csv', '/sensor_PoseWithCov.csv']
        topic_list = ['/pose_est', '/pose_gt']

        bagfile = str(SAMPLE_DATA_DIR + '/example.bag')
        self.assertTrue(ROSbag2CSV.extract(bagfile_name=bagfile, topic_list=topic_list,
                                           fn_list=fn_list, result_dir=str(SAMPLE_DATA_DIR + '/results'),
                                           verbose=True, fmt=CSVSpatialFormatType('PoseWithCov')))

if __name__ == "__main__":
     unittest.main()