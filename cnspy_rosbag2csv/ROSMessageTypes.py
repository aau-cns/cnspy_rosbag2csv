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
from enum import Enum


class ROSMessageTypes(Enum):
    NOT_SUPPORTED = 'NOT_SUPPORTED'
    # GEOMETRY_MSGS_POINT = 3 == VECTOR3
    GEOMETRY_MSGS_POINTSTAMPED = 'GEOMETRY_MSGS_POINTSTAMPED'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PointStamped.html
    GEOMETRY_MSGS_VECTOR3 = 'GEOMETRY_MSGS_VECTOR3'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html
    GEOMETRY_MSGS_VECTOR3STAMPED = 'GEOMETRY_MSGS_VECTOR3STAMPED'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html
    GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED = 'GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
    GEOMETRY_MSGS_POSEWITHCOVARIANCE = 'GEOMETRY_MSGS_POSEWITHCOVARIANCE'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html
    GEOMETRY_MSGS_POSESTAMPED = 'GEOMETRY_MSGS_POSESTAMPED'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
    GEOMETRY_MSGS_POSE = 'GEOMETRY_MSGS_POSE'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
    GEOMETRY_MSGS_QUATERNION = 'GEOMETRY_MSGS_QUATERNION'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html
    GEOMETRY_MSGS_QUATERNIONSTAMPED = 'GEOMETRY_MSGS_QUATERNIONSTAMPED'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/QuaternionStamped.html
    GEOMETRY_MSGS_TRANSFORM = 'GEOMETRY_MSGS_TRANSFORM'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Transform.html
    GEOMETRY_MSGS_TRANSFORMSTAMPED = 'GEOMETRY_MSGS_TRANSFORMSTAMPED'  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html

    def __str__(self):
        return str(self.value)

    @staticmethod
    def get_message_type(msg_):
        """

        :rtype: ROSMessageTypes
        """

        if hasattr(msg_, 'header'):  # STAMPED
            if hasattr(msg_, 'pose') and hasattr(msg_.pose, 'covariance') and hasattr(msg_.pose, 'pose'):
                return ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED

            if hasattr(msg_, 'pose') and hasattr(msg_.pose, 'position'):
                return ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED

            if hasattr(msg_, 'point'):
                return ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED

            if hasattr(msg_, 'vector'):
                return ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED

            if hasattr(msg_, 'transform'):
                return ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED

            if hasattr(msg_, 'quaternion'):
                return ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED

        else:  # NOT STAMPED
            if hasattr(msg_, 'pose') and hasattr(msg_, 'covariance'):
                return ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE

            if hasattr(msg_, 'position') and hasattr(msg_, 'orientation'):
                return ROSMessageTypes.GEOMETRY_MSGS_POSE

            if hasattr(msg_, 'translation') and hasattr(msg_, 'rotation'):
                return ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM

            if hasattr(msg_, 'x') and hasattr(msg_, 'y') and hasattr(msg_, 'z') and hasattr(msg_, 'w'):
                return ROSMessageTypes.GEOMETRY_MSGS_QUATERNION

            if hasattr(msg_, 'x') and hasattr(msg_, 'y') and hasattr(msg_, 'z'):
                return ROSMessageTypes.GEOMETRY_MSGS_VECTOR3

        return ROSMessageTypes.NOT_SUPPORTED
