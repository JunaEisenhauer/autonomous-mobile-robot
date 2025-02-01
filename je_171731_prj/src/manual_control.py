#!/usr/bin/python

import rospy

from robot_control import RobotControl, Movement, Steering


def main():  # type: () -> None
    """ Main function for the manual robot control. """
    rospy.init_node("manual_robot_control")

    robot_control = RobotControl()

    while True:
        action = raw_input()

        movement = Movement.STANDING
        steering = Steering.STRAIGHT

        if 'w' in action and 's' not in action:
            movement = Movement.FORWARD
        elif 's' in action and 'w' not in action:
            movement = Movement.BACKWARD

        if 'a' in action and 'd' not in action:
            steering = Steering.LEFT
        elif 'd' in action and 'a' not in action:
            steering = Steering.RIGHT

        robot_control.move(movement, steering)


# entry point for the manual control
if __name__ == "__main__":
    main()
