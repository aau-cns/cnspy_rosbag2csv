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
from ros_csv_formats.CSVFormat import CSVFormat

from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, QuaternionStamped, Transform, TransformStamped
from std_msgs.msg import Header, Time


class CSVLine2ROSMsg:
    def __init__(self):
        pass

    @staticmethod
    def to(fmt, line, line_number, msg_type):
        if str(fmt) == 'TUM':
            return CSVLine2ROSMsg.from_TUM(line, line_number, msg_type)
        elif str(fmt) == 'PoseWithCov':
            return CSVLine2ROSMsg.from_PoseWithCov(line, line_number, msg_type)
        else:
            print("CSVLine2ROSMsg.to(...): type {0} not supported".format(str(fmt)))
        return None

    @staticmethod
    def from_TUM(line, line_number, msg_type):
        msg = None
        s = CSVFormat.parse(line, CSVFormat.TUM)
        h = Header()
        h.stamp = h.stamp.from_sec(s.t)
        h.seq = int(line_number)

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED:
            msg = PointStamped()
            msg.header = h
            msg.point = Point(x=s.tx, y=s.ty, z=s.tz)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3:
            msg = Vector3(x=s.tx, y=s.ty, z=s.tz)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED:
            msg = Vector3Stamped()
            msg.header = h
            msg.vector = Vector3(x=s.tx, y=s.ty, z=s.tz)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
            msg = PoseWithCovarianceStamped()
            msg.header = h
            msg.pose.pose.position = Point(x=s.tx, y=s.ty, z=s.tz)
            msg.pose.pose.orientation = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            msg = PoseWithCovariance()
            msg.pose.position = Point(x=s.tx, y=s.ty, z=s.tz)
            msg.pose.orientation = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED:
            msg = PoseStamped()
            msg.header = h
            msg.pose.position = Point(x=s.tx, y=s.ty, z=s.tz)
            msg.pose.orientation = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSE:
            msg = Pose()
            msg.position = Point(x=s.tx, y=s.ty, z=s.tz)
            msg.orientation = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_QUATERNION:
            msg = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED:
            msg = QuaternionStamped()
            msg.header = h
            msg.quaternion = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM:
            msg = Transform()
            msg.translation = Vector3(x=s.tx, y=s.ty, z=s.tz)
            msg.rotation = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        elif msg_type == ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED:
            msg = TransformStamped()
            msg.header = h
            msg.transform.translation = Vector3(x=s.tx, y=s.ty, z=s.tz)
            msg.transform.rotation = Quaternion(x=s.qx, y=s.qy, z=s.qz, w=s.qw)
        # else:
        return msg, s.t

    @staticmethod
    def from_PoseWithCov(line, line_number, msg_type):
        msg, t = CSVLine2ROSMsg.from_TUM(line, line_number, msg_type=msg_type)
        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED or msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            # TODO: inefficient as line is parsed twice!
            s = CSVFormat.parse(line, CSVFormat.PoseWithCov)
            P = [0.] * 36
            P[0] = s.pxx
            P[1] = s.pxy
            P[6] = s.pxy
            P[2] = s.pxz
            P[12] = s.pxz
            P[7] = s.pyy
            P[8] = s.pyz
            P[13] = s.pyz
            P[14] = s.pzz
            P[21] = s.qrr
            P[22] = s.qrp
            P[27] = s.qrp
            P[23] = s.qry
            P[33] = s.qry
            P[28] = s.qpp
            P[29] = s.qpy
            P[34] = s.qpy
            P[35] = s.qyy
            if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
                msg.pose.covariance = P
            else:
                msg.covariance = P

        # else:
        return msg, t


########################################################################################################################
#################################################### T E S T ###########################################################
########################################################################################################################
import unittest


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

    def test_from_PoseWithCov(self):
        line = str('0.1, 1.0, 2.0,3.0, 0,0,0,1.0, 8.0,8.1,8.2,8.7,8.8,8.14, 9.21,9.22,9.23,9.28,9.29,9.35')
        msg = CSVLine2ROSMsg.from_PoseWithCov(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print(msg)
        msg = CSVLine2ROSMsg.from_PoseWithCov(line, 1, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print(msg)
        print('done')


if __name__ == "__main__":
    unittest.main()
    print("testing supported ROS msgs types")
