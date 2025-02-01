import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

SCAN_TOPIC = "/scan"
GAZEBO_MODEL_STATES_TOPIC = "/gazebo/model_states"
ODOM_TOPIC = "/odom"
ROBOT_MODEL_NAME = "turtlebot3_burger"


def _orientation2rotation(orientation):
    # type: (Quaternion) -> float
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = euler_from_quaternion(quaternion)
    return euler[2]


class RobotState:
    def __init__(self):  # type: () -> None
        self.laser_scan = None
        self.exact_position = None
        self.exact_rotation = None
        self.proximal_position = None
        self.proximal_rotation = None
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self._laser_scan_callback)
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self._model_states_callback)
        rospy.Subscriber(ODOM_TOPIC, Odometry, self._odom_callback)

    def _laser_scan_callback(self, laser_scan):
        # type: (LaserScan) -> None
        self.laser_scan = laser_scan

    def _model_states_callback(self, model_states):
        # type: (ModelStates) -> None
        index = model_states.name.index(ROBOT_MODEL_NAME)
        position = model_states.pose[index].position
        self.exact_position = (position.x, position.y)
        self.exact_rotation = _orientation2rotation(model_states.pose[index].orientation)

    def _odom_callback(self, odom):
        # type: (Odometry) -> None
        position = odom.pose.pose.position
        self.proximal_position = (position.x, position.y)
        self.proximal_rotation = _orientation2rotation(odom.pose.pose.orientation)

    def received_all_data(self):
        # type: () -> bool
        """ Checks if all necessary data like position, odometry and laser scan were received.

        Returns: Whether all necessary data were received.
        """
        if self.exact_position is None or self.exact_rotation is None:
            return False

        if self.proximal_position is None or self.proximal_rotation is None:
            return False

        if self.laser_scan is None:
            return False

        return True
