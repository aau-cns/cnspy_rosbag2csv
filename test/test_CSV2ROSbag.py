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
from cnspy_rosbag2csv.CSV2ROSbag import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class CSV2ROSbag_Test(unittest.TestCase):
    def test_identify(self):
        fn_list = [str(SAMPLE_DATA_DIR + '/ID1-pose-est-cov.csv'), str(SAMPLE_DATA_DIR + '/ID1-pose-gt.csv')]
        topic_list = ['/pose_est', '/pose_gt']
        fmt_list = [ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED, ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED]
        CSV2ROSbag.extract('my.bag', topic_list, fn_list, fmt_list, result_dir=str(SAMPLE_DATA_DIR + '/results'), verbose=True)


if __name__ == "__main__":
     unittest.main()