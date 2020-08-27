from enum import Enum
from ROSMessageType import ROSMessageType

from tum_eval.tum_csv_header import tum_csv_header


class CSVFormat(Enum):
    TUM = 'TUM'
    none = 'none'

    def __str__(self):
        return self.value

    @staticmethod
    def get_header(format):
        if format == 'TUM':
            return tum_csv_header.default()
        else:
            return "# no header "

    @staticmethod
    def message_to_tum(msg_, t_, msg_type=ROSMessageType.NOT_SUPPORTED):
        """

        :rtype: list of floats
        """

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.point.x), str(msg_.point.y), str(msg_.point.z), "0",
                    "0",
                    "0", "1"]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_VECTOR3:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.x), str(msg_.y), str(msg_.z), "0", "0", "0", "1"]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.vector.x), str(msg_.vector.y),
                    str(msg_.vector.z), "0", "0", "0", "1"]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.pose.position.x), str(msg_.pose.pose.position.y),
                    str(msg_.pose.pose.position.z), str(msg_.pose.pose.orientation.x),
                    str(msg_.pose.pose.orientation.y), str(msg_.pose.pose.orientation.z),
                    str(msg_.pose.pose.orientation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z), str(msg_.pose.orientation.x),
                    str(msg_.pose.orientation.y), str(msg_.pose.orientation.z),
                    str(msg_.pose.orientation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z), str(msg_.pose.orientation.x), str(msg_.pose.orientation.y),
                    str(msg_.pose.orientation.z), str(msg_.pose.orientation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.position.x), str(msg_.position.y),
                    str(msg_.position.z), str(msg_.orientation.x), str(msg_.orientation.y),
                    str(msg_.orientation.z), str(msg_.orientation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNION:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), "0", "0", "0", str(msg_.x), str(msg_.y), str(msg_.z), str(msg_.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), "0", "0", "0", str(msg_.pose.quaternion.x),
                    str(msg_.pose.quaternion.y), str(msg_.pose.quaternion.z), str(msg_.pose.quaternion.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORM:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.translation.x), str(msg_.translation.y),
                    str(msg_.translation.z), str(msg_.rotation.x), str(msg_.rotation.y),
                    str(msg_.rotation.z), str(msg_.rotation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED:
            if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
                return ["%f" % msg_.header.stamp.to_sec(), str(msg_.transform.translation.x),
                        str(msg_.transform.translation.y),
                        str(msg_.transform.translation.z), str(msg_.transform.rotation.x),
                        str(msg_.transform.rotation.y),
                        str(msg_.transform.rotation.z), str(msg_.transform.rotation.w)]
        # else:
        return []

    @staticmethod
    def message_to_tum_short(msg_, t_, msg_type=ROSMessageType.NOT_SUPPORTED):
        """

        :rtype: list of floats
        """

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.point.x), str(msg_.point.y), str(msg_.point.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_VECTOR3:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.x), str(msg_.y), str(msg_.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.vector.x), str(msg_.vector.y),
                    str(msg_.vector.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.pose.position.x), str(msg_.pose.pose.position.y),
                    str(msg_.pose.pose.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.position.x), str(msg_.position.y),
                    str(msg_.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNION:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), "0", "0", "0"]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), "0", "0", "0"]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORM:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % str(t), str(msg_.translation.x), str(msg_.translation.y),
                    str(msg_.translation.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED:
            if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
                return ["%f" % msg_.header.stamp.to_sec(), str(msg_.transform.translation.x),
                        str(msg_.transform.translation.y),
                        str(msg_.transform.translation.z)]
        # else:
        return []
