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
from rosbag2csv.ROSMessageTypes import ROSMessageTypes
from spatial_csv_formats.CSVFormatPose import CSVFormatPose


class ROSMsg2CSVLine:
    def __init__(self):
        pass

    @staticmethod
    def to(fmt, msg, t, msg_type):
        if fmt == CSVFormatPose.TUM:
            return ROSMsg2CSVLine.to_TUM(msg, t, msg_type)
        elif fmt == CSVFormatPose.PositionStamped:
            return ROSMsg2CSVLine.to_TUM(msg, t, msg_type)
        elif fmt == CSVFormatPose.PoseCov:
            return ROSMsg2CSVLine.to_PoseCov(msg, t, msg_type)
        elif fmt == CSVFormatPose.PoseWithCov:
            return ROSMsg2CSVLine.to_PoseWithCov(msg, t, msg_type)
        else:
            return None

    @staticmethod
    def to_TUM(msg_, t_, msg_type=ROSMessageTypes.NOT_SUPPORTED):
        """

        :rtype: list of floats
        """

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.point.x), str(msg_.point.y), str(msg_.point.z), "0",
                    "0",
                    "0", "1"]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.x), str(msg_.y), str(msg_.z), "0", "0", "0", "1"]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.vector.x), str(msg_.vector.y),
                    str(msg_.vector.z), "0", "0", "0", "1"]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.pose.position.x), str(msg_.pose.pose.position.y),
                    str(msg_.pose.pose.position.z), str(msg_.pose.pose.orientation.x),
                    str(msg_.pose.pose.orientation.y), str(msg_.pose.pose.orientation.z),
                    str(msg_.pose.pose.orientation.w)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z), str(msg_.pose.orientation.x),
                    str(msg_.pose.orientation.y), str(msg_.pose.orientation.z),
                    str(msg_.pose.orientation.w)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z), str(msg_.pose.orientation.x), str(msg_.pose.orientation.y),
                    str(msg_.pose.orientation.z), str(msg_.pose.orientation.w)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.position.x), str(msg_.position.y),
                    str(msg_.position.z), str(msg_.orientation.x), str(msg_.orientation.y),
                    str(msg_.orientation.z), str(msg_.orientation.w)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_QUATERNION:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), "0", "0", "0", str(msg_.x), str(msg_.y), str(msg_.z), str(msg_.w)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), "0", "0", "0", str(msg_.quaternion.x),
                    str(msg_.quaternion.y), str(msg_.quaternion.z), str(msg_.quaternion.w)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.translation.x), str(msg_.translation.y),
                    str(msg_.translation.z), str(msg_.rotation.x), str(msg_.rotation.y),
                    str(msg_.rotation.z), str(msg_.rotation.w)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.transform.translation.x),
                    str(msg_.transform.translation.y),
                    str(msg_.transform.translation.z), str(msg_.transform.rotation.x),
                    str(msg_.transform.rotation.y),
                    str(msg_.transform.rotation.z), str(msg_.transform.rotation.w)]
        # else:
        return None

    @staticmethod
    def to_TUM_short(msg_, t_, msg_type=ROSMessageTypes.NOT_SUPPORTED):
        """

        :rtype: list of floats
        """

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.point.x), str(msg_.point.y), str(msg_.point.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.x), str(msg_.y), str(msg_.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.vector.x), str(msg_.vector.y),
                    str(msg_.vector.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.pose.position.x), str(msg_.pose.pose.position.y),
                    str(msg_.pose.pose.position.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.position.x), str(msg_.position.y),
                    str(msg_.position.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_QUATERNION:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), "0", "0", "0"]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), "0", "0", "0"]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.translation.x), str(msg_.translation.y),
                    str(msg_.translation.z)]

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.transform.translation.x),
                    str(msg_.transform.translation.y),
                    str(msg_.transform.translation.z)]
        # else:
        return None

    @staticmethod
    def to_PoseCov(msg_, t_, msg_type=ROSMessageTypes.NOT_SUPPORTED):

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED or msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
                P = msg_.pose.covariance
                t = msg_.header.stamp.to_sec()
            else:
                P = msg_.covariance
                t = float(t_.secs) + float(t_.nsecs) * 1e-9

            return ["%f" % (t), P[0], P[1], P[2], P[7], P[8], P[15], P[21], P[22], P[23], P[28], P[29], P[35]]
        elif msg_type != ROSMessageTypes.NOT_SUPPORTED:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return [str(t), '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0']
        # else:
        return None

    @staticmethod
    def to_PoseWithCov(msg_, t_, msg_type=ROSMessageTypes.NOT_SUPPORTED):

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED or msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
                P = msg_.pose.covariance
                p = msg_.pose.pose.position
                q = msg_.pose.pose.orientation
                t = msg_.header.stamp.to_sec()
            else:
                P = msg_.covariance
                p = msg_.pose.position
                q = msg_.pose.orientation
                t = float(t_.secs) + float(t_.nsecs) * 1e-9

            return ["%f" % (t), str(p.x), str(p.y), str(p.z), str(q.x), str(q.y), str(q.z), str(q.w),
                    P[0], P[1], P[2], P[7], P[8], P[14], P[21], P[22], P[23], P[28], P[29], P[35]]
        elif msg_type != ROSMessageTypes.NOT_SUPPORTED:
            line = ROSMsg2CSVLine.to_TUM(msg_, t_, msg_type)
            line = line + ['0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0']
            return line
        # else:
        return None


########################################################################################################################
#################################################### T E S T ###########################################################
########################################################################################################################
import unittest
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
        line = self.get_lines(CSVFormatPose.PositionStamped)

    def test_MESSAGE_TO_PoseCov(self):
        t = Time()
        t.secs = 0
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.pose.covariance = range(0, 36, 1)
        line = ROSMsg2CSVLine.to_PoseCov(pose_cov, t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print('line1:' + str(line))

        pose_cov = PoseWithCovariance()
        pose_cov.covariance = range(0, 36, 1)
        line = ROSMsg2CSVLine.to_PoseCov(pose_cov, t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print('line2:' + str(line))
        self.assertTrue(len(line) == 13)
        line = self.get_lines(CSVFormatPose.PoseCov)
        self.assertTrue(len(line) == 13)

    def test_MESSAGE_TO_PoseWithCov(self):
        line = self.get_lines(CSVFormatPose.PoseWithCov)
        print('line:' + str(line))
        self.assertTrue(len(line) == 20)


if __name__ == "__main__":
    unittest.main()
    print("testing supported ROS msgs types")
