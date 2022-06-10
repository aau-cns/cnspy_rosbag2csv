#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
#
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
import unittest
from cnspy_rosbag2csv.CSVLine2ROSMsg import *
from cnspy_rosbag2csv.ROSMessageTypes import ROSMessageTypes


class CSVLine2ROSMsg_Test(unittest.TestCase):
    def test_from_TUM(self):
        line = str('0.1, 1.0, 2.0,3.0, 0,0,0,1.0')
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_VECTOR3)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSE)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_QUATERNION)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM)
        print(msg)
        msg = CSVLine2ROSMsg.from_TUM(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED)
        print(msg)
        print('done')

    def test_from_PosOrientWithCov(self):
        line = str('0.1, 1.0, 2.0,3.0, 0,0,0,1.0, 8.0,8.1,8.2,8.7,8.8,8.14, 9.21,9.22,9.23,9.28,9.29,9.35')
        msg = CSVLine2ROSMsg.from_PosOrientWithCov(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_PosOrientWithCov(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print(msg)
        print('done')

    def test_from_PoseWithCov(self):
        line = str('0.1, 1.0, 2.0,3.0, 0,0,0,1.0, 8.0,8.1,8.2,8.7,8.8,8.14, 9.21,9.22,9.23,9.28,9.29,9.35, 7.3,7.4,7.5,7.9,7.10,7.11,7.15,7.16,7.17')
        msg = CSVLine2ROSMsg.from_PoseWithCov(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_PoseWithCov(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print(msg)
        print('done')


if __name__ == "__main__":
    unittest.main()
    print("testing supported ROS msgs types")
