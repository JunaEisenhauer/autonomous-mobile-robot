import math
from enum import IntEnum
from math import cos, sin, atan2

import rospy
from geometry_msgs.msg import Twist, Vector3

from robot_state import RobotState

VELOCITY_TOPIC = "/cmd_vel"
MOVEMENT_SPEED = 0.3
STEERING_SPEED = 0.5
STRAIGHT_OFFSET = math.pi * 0.05
FORWARD_TURN_OFFSET = math.pi * 0.15


def _angle2target(target_position, position, rotation):
    target_vector = (target_position[0] - position[0],
                     target_position[1] - position[1])
    direction_vector = (cos(rotation), sin(rotation))

    dot = target_vector[0] * direction_vector[0] + target_vector[1] * direction_vector[1]
    det = target_vector[0] * direction_vector[1] - target_vector[1] * direction_vector[0]
    return atan2(det, dot)


def _action2target(target_position, robot_state):
    angle = _angle2target(target_position, (robot_state.proximal_position[0], robot_state.proximal_position[1]),
                          robot_state.proximal_rotation)

    movement = Movement.FORWARD

    if angle > math.pi / 2:
        angle -= math.pi
        movement = Movement.BACKWARD
    if angle < -math.pi / 2:
        angle += math.pi
        movement = Movement.BACKWARD

    if -STRAIGHT_OFFSET <= angle <= STRAIGHT_OFFSET:
        return movement, Steering.STRAIGHT

    if -FORWARD_TURN_OFFSET <= angle < 0:
        return movement, Steering.LEFT
    if FORWARD_TURN_OFFSET >= angle > 0:
        return movement, Steering.RIGHT

    if angle < 0:
        return Movement.STANDING, Steering.LEFT
    if angle > 0:
        return Movement.STANDING, Steering.RIGHT


class RobotControl:
    def __init__(self):  # type: () -> None
        self._velocity_publisher = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=4)

    def navigate(self, target_position, robot_state):
        # type: ((float, float), RobotState) -> None
        """ Navigates the robot towards the given target position.

        Args:
            target_position: The target position to navigate to.
            robot_state: The robot state.
        """
        movement, steering = _action2target(target_position, robot_state)
        self.move(movement, steering)

    def move(self, movement, steering):
        # type: (Movement, Steering) -> None
        """ Moves the robot in the direction of movement and turns depending on steering.

        Args:
            movement: The movement to apply to the robot.
            steering: The steering to apply to the robot.
        """
        rospy.logdebug("Move robot %s %s", movement, steering)

        move = movement * MOVEMENT_SPEED
        steer = steering * STEERING_SPEED
        self._publish(move, steer)

    def stop(self):  # type: () -> None
        """ Stops the robot. """
        self.move(Movement.STANDING, Steering.STRAIGHT)

    def _publish(self, movement, steering):
        # type: (float, float) -> None
        linear_velocity = Vector3(movement, 0.0, 0.0)
        angular_velocity = Vector3(0.0, 0.0, steering)
        twist = Twist(linear=linear_velocity, angular=angular_velocity)
        self._velocity_publisher.publish(twist)


class Movement(IntEnum):
    BACKWARD = -1
    STANDING = 0
    FORWARD = 1


class Steering(IntEnum):
    RIGHT = -1
    STRAIGHT = 0
    LEFT = 1
