from enum import Enum


class ROSMessageTypes(Enum):
    NOT_SUPPORTED = 0
    # GEOMETRY_MSGS_POINT = 3 == VECTOR3
    GEOMETRY_MSGS_POINTSTAMPED = 1  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PointStamped.html
    GEOMETRY_MSGS_VECTOR3 = 2  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html
    GEOMETRY_MSGS_VECTOR3STAMPED = 3  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html
    GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED = 4  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
    GEOMETRY_MSGS_POSEWITHCOVARIANCE = 5  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html
    GEOMETRY_MSGS_POSESTAMPED = 6  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
    GEOMETRY_MSGS_POSE = 7  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
    GEOMETRY_MSGS_QUATERNION = 8  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html
    GEOMETRY_MSGS_QUATERNIONSTAMPED = 9  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/QuaternionStamped.html
    GEOMETRY_MSGS_TRANSFORM = 10  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Transform.html
    GEOMETRY_MSGS_TRANSFORMSTAMPED = 11  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html

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
