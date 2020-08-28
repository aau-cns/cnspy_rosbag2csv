from enum import Enum
from ROSMessageType import ROSMessageType

from tum_eval.TUMCSVheader import TUMCSVheader


class CSVFormat(Enum):
    TUM = 'TUM'
    TUM_short = 'TUM_short'
    none = 'none'

    def __str__(self):
        return self.value

    @staticmethod
    def get_header(format):
        if format == 'TUM':
            return TUMCSVheader.default()
        elif format == 'TUM_short':
            return TUMCSVheader.pos_stamped()
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
            return ["%f" % (t), str(msg_.x), str(msg_.y), str(msg_.z), "0", "0", "0", "1"]

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
            return ["%f" % (t), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z), str(msg_.pose.orientation.x),
                    str(msg_.pose.orientation.y), str(msg_.pose.orientation.z),
                    str(msg_.pose.orientation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z), str(msg_.pose.orientation.x), str(msg_.pose.orientation.y),
                    str(msg_.pose.orientation.z), str(msg_.pose.orientation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.position.x), str(msg_.position.y),
                    str(msg_.position.z), str(msg_.orientation.x), str(msg_.orientation.y),
                    str(msg_.orientation.z), str(msg_.orientation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNION:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), "0", "0", "0", str(msg_.x), str(msg_.y), str(msg_.z), str(msg_.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), "0", "0", "0", str(msg_.quaternion.x),
                    str(msg_.quaternion.y), str(msg_.quaternion.z), str(msg_.quaternion.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORM:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.translation.x), str(msg_.translation.y),
                    str(msg_.translation.z), str(msg_.rotation.x), str(msg_.rotation.y),
                    str(msg_.rotation.z), str(msg_.rotation.w)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED:
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
            return ["%f" % (t), str(msg_.x), str(msg_.y), str(msg_.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.vector.x), str(msg_.vector.y),
                    str(msg_.vector.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.pose.position.x), str(msg_.pose.pose.position.y),
                    str(msg_.pose.pose.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.pose.position.x), str(msg_.pose.position.y),
                    str(msg_.pose.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSE:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.position.x), str(msg_.position.y),
                    str(msg_.position.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNION:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), "0", "0", "0"]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), "0", "0", "0"]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORM:
            t = float(t_.secs) + float(t_.nsecs) * 1e-9
            return ["%f" % (t), str(msg_.translation.x), str(msg_.translation.y),
                    str(msg_.translation.z)]

        if msg_type == ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED:
            return ["%f" % msg_.header.stamp.to_sec(), str(msg_.transform.translation.x),
                    str(msg_.transform.translation.y),
                    str(msg_.transform.translation.z)]
        # else:
        return []


if __name__ == "__main__":
    print("testing supported ROS msgs types")
    import rospy
    from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped
    from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
    from geometry_msgs.msg import Quaternion, QuaternionStamped, Transform, TransformStamped

    assert (ROSMessageType.get_message_type(Point()) == ROSMessageType.GEOMETRY_MSGS_VECTOR3)
    assert (ROSMessageType.get_message_type(PointStamped()) == ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED)
    assert (ROSMessageType.get_message_type(Vector3()) == ROSMessageType.GEOMETRY_MSGS_VECTOR3)
    assert (ROSMessageType.get_message_type(Vector3Stamped()) == ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED)
    assert (ROSMessageType.get_message_type(
        PoseWithCovarianceStamped()) == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
    assert (ROSMessageType.get_message_type(PoseWithCovariance()) == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
    assert (ROSMessageType.get_message_type(PoseStamped()) == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED)
    assert (ROSMessageType.get_message_type(Pose()) == ROSMessageType.GEOMETRY_MSGS_POSE)
    assert (ROSMessageType.get_message_type(Quaternion()) == ROSMessageType.GEOMETRY_MSGS_QUATERNION)
    assert (ROSMessageType.get_message_type(QuaternionStamped()) == ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED)
    assert (ROSMessageType.get_message_type(Transform()) == ROSMessageType.GEOMETRY_MSGS_TRANSFORM)
    assert (ROSMessageType.get_message_type(TransformStamped()) == ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED)


    class Time:
        nsecs = 0
        secs = 0

        def __repr__(self):
            return "{0}.{1}])".format(self.secs, self.nsecs)


    t = Time()
    t.secs = 0


    def check_tumline(line):
        assert len(line) == 8
        assert int(float(line[7])) == 1 or int(float(line[7])) == 0
        assert int(float(line[0])) == 0
        assert int(float(line[1])) == 0
        assert int(float(line[2])) == 0
        assert int(float(line[3])) == 0
        assert int(float(line[4])) == 0


    # MESSAGE_TO_TUM
    check_tumline(CSVFormat.message_to_tum(Point(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3))
    check_tumline(CSVFormat.message_to_tum(PointStamped(), t, ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED))
    check_tumline(CSVFormat.message_to_tum(Vector3(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3))
    check_tumline(CSVFormat.message_to_tum(Vector3Stamped(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED))
    check_tumline(CSVFormat.message_to_tum(PoseWithCovarianceStamped(), t,
                                           ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED))
    check_tumline(CSVFormat.message_to_tum(PoseWithCovariance(), t, ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE))
    check_tumline(CSVFormat.message_to_tum(PoseStamped(), t, ROSMessageType.GEOMETRY_MSGS_POSESTAMPED))
    check_tumline(CSVFormat.message_to_tum(Pose(), t, ROSMessageType.GEOMETRY_MSGS_POSE))
    check_tumline(CSVFormat.message_to_tum(Quaternion(), t, ROSMessageType.GEOMETRY_MSGS_QUATERNION))
    check_tumline(CSVFormat.message_to_tum(QuaternionStamped(), t, ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED))
    check_tumline(CSVFormat.message_to_tum(Transform(), t, ROSMessageType.GEOMETRY_MSGS_TRANSFORM))
    check_tumline(CSVFormat.message_to_tum(TransformStamped(), t, ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED))

    # MESSAGE_TO_TUM_SHORT
    line = CSVFormat.message_to_tum_short(Point(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3)
    line = CSVFormat.message_to_tum_short(PointStamped(), t, ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED)
    line = CSVFormat.message_to_tum_short(Vector3(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3)
    line = CSVFormat.message_to_tum_short(Vector3Stamped(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED)
    line = CSVFormat.message_to_tum_short(PoseWithCovarianceStamped(), t,
                                          ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
    line = CSVFormat.message_to_tum_short(PoseWithCovariance(), t, ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
    line = CSVFormat.message_to_tum_short(PoseStamped(), t, ROSMessageType.GEOMETRY_MSGS_POSESTAMPED)
    line = CSVFormat.message_to_tum_short(Pose(), t, ROSMessageType.GEOMETRY_MSGS_POSE)
    line = CSVFormat.message_to_tum_short(Quaternion(), t, ROSMessageType.GEOMETRY_MSGS_QUATERNION)
    line = CSVFormat.message_to_tum_short(QuaternionStamped(), t, ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED)
    line = CSVFormat.message_to_tum_short(Transform(), t, ROSMessageType.GEOMETRY_MSGS_TRANSFORM)
    line = CSVFormat.message_to_tum_short(TransformStamped(), t, ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED)
