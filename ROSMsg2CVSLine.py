from ROSMessageTypes import ROSMessageTypes


class ROSMsg2CSVLine:
    def __init__(self):
        pass

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

    def test_MESSAGE_TO_TUM_SHORT(self):
        t = Time()
        t.secs = 0

        # MESSAGE_TO_TUM_SHORT
        line = ROSMsg2CSVLine.to_TUM_short(Point(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3)
        line = ROSMsg2CSVLine.to_TUM_short(PointStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_POINTSTAMPED)
        line = ROSMsg2CSVLine.to_TUM_short(Vector3(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3)
        line = ROSMsg2CSVLine.to_TUM_short(Vector3Stamped(), t, ROSMessageTypes.GEOMETRY_MSGS_VECTOR3STAMPED)
        line = ROSMsg2CSVLine.to_TUM_short(PoseWithCovarianceStamped(), t,
                                           ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        line = ROSMsg2CSVLine.to_TUM_short(PoseWithCovariance(), t, ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        line = ROSMsg2CSVLine.to_TUM_short(PoseStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_POSESTAMPED)
        line = ROSMsg2CSVLine.to_TUM_short(Pose(), t, ROSMessageTypes.GEOMETRY_MSGS_POSE)
        line = ROSMsg2CSVLine.to_TUM_short(Quaternion(), t, ROSMessageTypes.GEOMETRY_MSGS_QUATERNION)
        line = ROSMsg2CSVLine.to_TUM_short(QuaternionStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_QUATERNIONSTAMPED)
        line = ROSMsg2CSVLine.to_TUM_short(Transform(), t, ROSMessageTypes.GEOMETRY_MSGS_TRANSFORM)
        line = ROSMsg2CSVLine.to_TUM_short(TransformStamped(), t, ROSMessageTypes.GEOMETRY_MSGS_TRANSFORMSTAMPED)

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


if __name__ == "__main__":
    unittest.main()
    print("testing supported ROS msgs types")
