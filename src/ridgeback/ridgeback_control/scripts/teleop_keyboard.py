#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop():
    rospy.init_node('teleop_keyboard', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()

    try:
        print("Use 'W' to move forward, 'S' to move backward, 'A' to turn left, 'D' to turn right")
        print("Use 'Q' to stop and turn left, 'E' to stop and turn right")
        print("Press 'Ctrl + C' to exit")

        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                twist.linear.x = 1.0 # Set linear velocity for forward movement
                twist.linear.y = 0.0  # Stop linear movement
                twist.angular.z = 0.0  # stop angular movement
            elif key == 's':
                twist.linear.x = -1.0 # Set linear velocity for backward movement
                twist.linear.y = 0.0  # Stop linear movement
                twist.angular.z = 0.0  # stop angular movement
            elif key == 'a':
                twist.linear.y = 1.0  # Set angular velocity for turning left
                twist.linear.x = 0.0  # Stop linear movement
                twist.angular.z = 0.0  # stop angular movement
            elif key == 'd':
                twist.linear.y = -1.0  # Set angular velocity for turning right
                twist.linear.x = 0.0  # Stop linear movement
                twist.angular.z = 0.0  # stop angular movement
            elif key == 'q':
                twist.linear.x = 0.0  # Stop linear movement
                twist.linear.y = 0.0  # Stop linear movement
                twist.angular.z = 1.0  # Set angular velocity for turning left
            elif key == 'e':
                twist.linear.x = 0.0  # Stop linear movement
                twist.linear.y = 0.0  # Stop linear movement
                twist.angular.z = -1.0  # Set angular velocity for turning right
            else:
                twist.linear.x = 0.0  # Stop linear movement
                twist.linear.y = 0.0  # Stop linear movement
                twist.angular.z = 0.0  # Stop angular movement

            pub.publish(twist)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

