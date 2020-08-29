from enum import Enum
from ROSMessageType import ROSMessageType

from tum_eval.TUMCSVheader import TUMCSVheader


class CSVFormat(Enum):
    TUM = 'TUM'
    TUM_short = 'TUM_short'
    PoseCov = 'PoseCov'
    none = 'none'

    def __str__(self):
        return self.value

    @staticmethod
    def get_header(format):
        if str(format) == 'TUM':
            return TUMCSVheader.default()
        elif str(format) == 'TUM_short':
            return TUMCSVheader.pos_stamped()
        elif str(format) == 'PoseCov':
            return ['# t', 'pxx', 'pxy', 'pxz', 'pyy', 'pyz', 'pzz', 'qrr', 'qrp', 'qry', 'qpp', 'qpy', 'qyy']
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
        return None

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
        return None

    @staticmethod
    def msg_to_PoseCov(msg_, t_, msg_type=ROSMessageType.NOT_SUPPORTED):

        if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED or msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE:
            if msg_type == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED:
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
import rospy
from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, QuaternionStamped, Transform, TransformStamped


class Time:
    nsecs = 0
    secs = 0

    def __repr__(self):
        return "{0}.{1}])".format(self.secs, self.nsecs)


class CSVFormat_Test(unittest.TestCase):
    def test_get_message_type(self):
        self.assertTrue(ROSMessageType.get_message_type(Point()) == ROSMessageType.GEOMETRY_MSGS_VECTOR3)
        self.assertTrue(ROSMessageType.get_message_type(PointStamped()) == ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED)
        self.assertTrue(ROSMessageType.get_message_type(Vector3()) == ROSMessageType.GEOMETRY_MSGS_VECTOR3)
        self.assertTrue(
            ROSMessageType.get_message_type(Vector3Stamped()) == ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED)
        self.assertTrue(ROSMessageType.get_message_type(
            PoseWithCovarianceStamped()) == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        self.assertTrue(ROSMessageType.get_message_type(
            PoseWithCovariance()) == ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        self.assertTrue(ROSMessageType.get_message_type(PoseStamped()) == ROSMessageType.GEOMETRY_MSGS_POSESTAMPED)
        self.assertTrue(ROSMessageType.get_message_type(Pose()) == ROSMessageType.GEOMETRY_MSGS_POSE)
        self.assertTrue(ROSMessageType.get_message_type(Quaternion()) == ROSMessageType.GEOMETRY_MSGS_QUATERNION)
        self.assertTrue(
            ROSMessageType.get_message_type(QuaternionStamped()) == ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED)
        self.assertTrue(ROSMessageType.get_message_type(Transform()) == ROSMessageType.GEOMETRY_MSGS_TRANSFORM)
        self.assertTrue(
            ROSMessageType.get_message_type(TransformStamped()) == ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED)

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

        self.check_tumline(CSVFormat.message_to_tum(Point(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3))
        self.check_tumline(CSVFormat.message_to_tum(PointStamped(), t, ROSMessageType.GEOMETRY_MSGS_POINTSTAMPED))
        self.check_tumline(CSVFormat.message_to_tum(Vector3(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3))
        self.check_tumline(CSVFormat.message_to_tum(Vector3Stamped(), t, ROSMessageType.GEOMETRY_MSGS_VECTOR3STAMPED))
        self.check_tumline(CSVFormat.message_to_tum(PoseWithCovarianceStamped(), t,
                                                    ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED))
        self.check_tumline(
            CSVFormat.message_to_tum(PoseWithCovariance(), t, ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE))
        self.check_tumline(CSVFormat.message_to_tum(PoseStamped(), t, ROSMessageType.GEOMETRY_MSGS_POSESTAMPED))
        self.check_tumline(CSVFormat.message_to_tum(Pose(), t, ROSMessageType.GEOMETRY_MSGS_POSE))
        self.check_tumline(CSVFormat.message_to_tum(Quaternion(), t, ROSMessageType.GEOMETRY_MSGS_QUATERNION))
        self.check_tumline(
            CSVFormat.message_to_tum(QuaternionStamped(), t, ROSMessageType.GEOMETRY_MSGS_QUATERNIONSTAMPED))
        self.check_tumline(CSVFormat.message_to_tum(Transform(), t, ROSMessageType.GEOMETRY_MSGS_TRANSFORM))
        self.check_tumline(
            CSVFormat.message_to_tum(TransformStamped(), t, ROSMessageType.GEOMETRY_MSGS_TRANSFORMSTAMPED))

    def test_MESSAGE_TO_TUM_SHORT(self):
        t = Time()
        t.secs = 0

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

    def test_MESSAGE_TO_PoseCov(self):
        t = Time()
        t.secs = 0
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.pose.covariance = range(0, 36, 1)
        line = CSVFormat.msg_to_PoseCov(pose_cov, t, ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED)
        print('line1:' + str(line))

        pose_cov = PoseWithCovariance()
        pose_cov.covariance = range(0, 36, 1)
        line = CSVFormat.msg_to_PoseCov(pose_cov, t, ROSMessageType.GEOMETRY_MSGS_POSEWITHCOVARIANCE)
        print('line2:' + str(line))


if __name__ == "__main__":
    unittest.main()
    print("testing supported ROS msgs types")
