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
from cnspy_rosbag2csv.ROSMessageTypes import ROSMessageTypes
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType


class ROSMsg2CSVLine:
    def __init__(self):
        pass

    @staticmethod
    def to(fmt, msg, t, msg_type):
        if fmt == CSVSpatialFormatType.TUM or fmt == CSVSpatialFormatType.PoseStamped:
            return ROSMsg2CSVLine.to_TUM(msg, t, msg_type)
        elif fmt == CSVSpatialFormatType.PositionStamped:
            return ROSMsg2CSVLine.to_TUM(msg, t, msg_type)
        elif fmt == CSVSpatialFormatType.PosOrientCov:
            return ROSMsg2CSVLine.to_PosOrientCov(msg, t, msg_type)
        elif fmt == CSVSpatialFormatType.PoseCov:
            return ROSMsg2CSVLine.to_PoseCov(msg, t, msg_type)
        elif fmt == CSVSpatialFormatType.PosOrientWithCov:
            return ROSMsg2CSVLine.to_PosOrientWithCov(msg, t, msg_type)
        elif fmt == CSVSpatialFormatType.PoseWithCov:
            return ROSMsg2CSVLine.to_PoseWithCov(msg, t, msg_type)
        else:
            print("ROSMsg2CSVLine.to(...): type {0} not supported".format(str(fmt)))
            assert False

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
    def to_PosOrientCov(msg_, t_, msg_type=ROSMessageTypes.NOT_SUPPORTED):

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
    def to_PoseCov(msg_, t_, msg_type=ROSMessageTypes.NOT_SUPPORTED):

        if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED or msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            if msg_type == ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
                P = msg_.pose.covariance
                t = msg_.header.stamp.to_sec()
            else:
                P = msg_.covariance
                t = float(t_.secs) + float(t_.nsecs) * 1e-9

            return ["%f" % (t), P[0], P[1], P[2], P[3], P[4], P[5],
                    P[7], P[8], P[9], P[10], P[11],
                    P[14], P[15], P[16], P[17],
                    P[21], P[22], P[23],
                    P[28], P[29],
                    P[35]]
        elif msg_type != ROSMessageTypes.NOT_SUPPORTED:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return [str(t), '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0']
        # else:
        return None

    @staticmethod
    def to_PosOrientWithCov(msg_, t_, msg_type=ROSMessageTypes.NOT_SUPPORTED):

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
                    P[0], P[1], P[2], P[3], P[4], P[5],
                    P[7], P[8], P[9], P[10], P[11],
                    P[14], P[15], P[16], P[17],
                    P[21], P[22], P[23],
                    P[28], P[29],
                    P[35]]
        elif msg_type != ROSMessageTypes.NOT_SUPPORTED:
            line = ROSMsg2CSVLine.to_TUM(msg_, t_, msg_type)
            line = line + ['0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0']
            return line
        # else:
        return None

