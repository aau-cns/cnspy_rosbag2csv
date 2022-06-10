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
# Requirements:
# enum
########################################################################################################################
import unittest
from cnspy_rosbag2csv.ROSMsg2CSVLine import *
from cnspy_rosbag2csv.ROSMessageTypes import ROSMessageTypes
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType

from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, QuaternionStamped, Transform, TransformStamped


class Time:
    nsecs = 0
    secs = 0

    def __repr__(self):
        return "{0}.{1}])".format(self.secs, self.nsecs)


class ROSMsg2CSVLine_Test(unittest.TestCase):
    def test_get_message_type(self):
        self.assertTrue(ROSMessageTypes.get_message_type(Point()) == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3)
        self.assertTrue(ROSMessageTypes.get_message_type(PointStamped()) == ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED)
        self.assertTrue(ROSMessageTypes.get_message_type(Vector3()) == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3)
        self.assertTrue(
            ROSMessageTypes.get_message_type(Vector3Stamped()) == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED)
        self.assertTrue(ROSMessageTypes.get_message_type(
            PoseWithCovarianceStamped()) == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        self.assertTrue(ROSMessageTypes.get_message_type(
            PoseWithCovariance()) == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        self.assertTrue(ROSMessageTypes.get_message_type(PoseStamped()) == ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED)
        self.assertTrue(ROSMessageTypes.get_message_type(Pose()) == ROSMessageTypes.GEOMETRY_MSGS_POSE)
        self.assertTrue(ROSMessageTypes.get_message_type(Quaternion()) == ROSMessageTypes.GEOMETRY_MSGS_QUATERNION)
        self.assertTrue(
            ROSMessageTypes.get_message_type(QuaternionStamped()) == ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED)
        self.assertTrue(ROSMessageTypes.get_message_type(Transform()) == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM)
        self.assertTrue(
            ROSMessageTypes.get_message_type(TransformStamped()) == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED)

    def check_tumline(self, line):
        self.assertTrue(len(line) == 8)
        self.assertTrue(int(float(line[7])) == 1 or int(float(line[7])) == 0)
        self.assertTrue(int(float(line[0])) == 0)
        self.assertTrue(int(float(line[1])) == 0)
        self.assertTrue(int(float(line[2])) == 0)
        self.assertTrue(int(float(line[3])) == 0)
        self.assertTrue(int(float(line[4])) == 0)

    def test_MESSAGE_TO_TUM(self):
        t = Time()
        t.secs = 0

        self.check_tumline(ROSMsg2CSVLine.to_TUM(Point(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(PointStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(Vector3(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(Vector3Stamped(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(PoseWithCovarianceStamped(), t,
                                                 ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED))
        self.check_tumline(
            ROSMsg2CSVLine.to_TUM(PoseWithCovariance(), t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(PoseStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(Pose(), t, ROSMessageTypes.GEOMETRY_MSGS_POSE))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(Quaternion(), t, ROSMessageTypes.GEOMETRY_MSGS_QUATERNION))
        self.check_tumline(
            ROSMsg2CSVLine.to_TUM(QuaternionStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED))
        self.check_tumline(ROSMsg2CSVLine.to_TUM(Transform(), t, ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM))
        self.check_tumline(
            ROSMsg2CSVLine.to_TUM(TransformStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED))

    def get_lines(self, fmt):
        t = Time()
        t.secs = 0

        # MESSAGE_TO_TUM_SHORT
        line = ROSMsg2CSVLine.to(fmt, Point(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3)
        line = ROSMsg2CSVLine.to(fmt, PointStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED)
        line = ROSMsg2CSVLine.to(fmt, Vector3(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3)
        line = ROSMsg2CSVLine.to(fmt, Vector3Stamped(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED)
        line = ROSMsg2CSVLine.to(fmt, PoseWithCovarianceStamped(), t,
                                 ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        line = ROSMsg2CSVLine.to(fmt, PoseWithCovariance(), t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        line = ROSMsg2CSVLine.to(fmt, PoseStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED)
        line = ROSMsg2CSVLine.to(fmt, Pose(), t, ROSMessageTypes.GEOMETRY_MSGS_POSE)
        line = ROSMsg2CSVLine.to(fmt, Quaternion(), t, ROSMessageTypes.GEOMETRY_MSGS_QUATERNION)
        line = ROSMsg2CSVLine.to(fmt, QuaternionStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED)
        line = ROSMsg2CSVLine.to(fmt, Transform(), t, ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM)
        line = ROSMsg2CSVLine.to(fmt, TransformStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED)
        return line

    def test_MESSAGE_TO_TUM_SHORT(self):
        line = self.get_lines(CSVSpatialFormatType.PositionStamped)

    def test_MESSAGE_TO_PosOrientCov(self):
        t = Time()
        t.secs = 0
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.pose.covariance = range(0, 36, 1)
        line = ROSMsg2CSVLine.to_PosOrientCov(pose_cov, t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print('\nline1:' + str(line))

        pose_cov = PoseWithCovariance()
        pose_cov.covariance = range(0, 36, 1)
        line = ROSMsg2CSVLine.to_PosOrientCov(pose_cov, t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print('\nline2:' + str(line))
        self.assertTrue(len(line) == 13)
        line = self.get_lines(CSVSpatialFormatType.PosOrientCov)
        self.assertTrue(len(line) == 13)

    def test_MESSAGE_TO_PoseCov(self):
        t = Time()
        t.secs = 0
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.pose.covariance = range(0, 36, 1)
        line = ROSMsg2CSVLine.to_PoseCov(pose_cov, t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print('\nline1:' + str(line))

        pose_cov = PoseWithCovariance()
        pose_cov.covariance = range(0, 36, 1)
        line = ROSMsg2CSVLine.to_PoseCov(pose_cov, t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print('\nline2:' + str(line))
        self.assertTrue(len(line) == 13+9)
        line = self.get_lines(CSVSpatialFormatType.PoseCov)
        self.assertTrue(len(line) == 13+9)


    def test_MESSAGE_TO_PosOrientWithCov(self):
        line = self.get_lines(CSVSpatialFormatType.PosOrientWithCov)
        print('\nline:' + str(line))
        self.assertTrue(len(line) == 20)

    def test_MESSAGE_TO_PoseWithCov(self):
        line = self.get_lines(CSVSpatialFormatType.PoseWithCov)
        print('\nline:' + str(line))
        self.assertTrue(len(line) == 29)


if __name__ == "__main__":
    unittest.main()
    print("testing supported ROS msgs types")
