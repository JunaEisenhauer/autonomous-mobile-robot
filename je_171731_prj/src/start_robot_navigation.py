#!/usr/bin/python

import rospy

from robot_navigation import RobotNavigation


def main():  # type: () -> None
    """ Main function for the robot navigation. """
    # initialize ros node
    rospy.init_node("robot_navigation")
    rospy.loginfo("Starting robot navigation.")

    robot_navigation = RobotNavigation()

    # infinite loop till program is shutdown.
    while not rospy.is_shutdown():
        robot_navigation.update()


# entry point for the robot navigation
if __name__ == "__main__":
    main()
