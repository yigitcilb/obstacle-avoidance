#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
class ReactiveRobot:
    def __init__(self):
        rospy.init_node('reactive_robot_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.rate = rospy.Rate(10)  
        self.min_distance = 9
        self.start_time = rospy.get_rostime()
    def laser_callback(self, msg):
        ranges = msg.ranges
        front_alt = 15
        front_ust1 = 345
        front_ust2 = 359
        front = msg.ranges[front_alt:front_ust1 + 1] + msg.ranges[:front_ust2 + 1]
        self.front_min = np.mean(front)
        self.front_real_min = min(front)
        rospy.loginfo(self.front_min)
        right = msg.ranges[315:346]
        self.right_min = np.mean(right)
        left = msg.ranges[15:46]
        self.left_min = np.mean(left)
        self.avoid_obstacle(self.front_min)
    def avoid_obstacle(self, front_min):
        if self.front_real_min < 0.05:
            twist_msg = Twist()
            twist_msg.linear.x = -1.0
            twist_msg.angular.z = -1.5
            self.cmd_vel_pub.publish(twist_msg)
            rospy.loginfo("Duvara kafa atildi.")
            rospy.sleep(0.03)
        elif self.front_min > self.min_distance:
            twist_msg = Twist()
            twist_msg.linear.x = 0.6
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
        elif self.front_min < self.min_distance and self.right_min < self.left_min:
            twist_msg = Twist()
            self.velocity_stuff = (self.front_min/4)
            self.angular_stuff = (1/self.right_min)
            twist_msg.linear.x = self.velocity_stuff
            twist_msg.angular.z = self.angular_stuff
            self.cmd_vel_pub.publish(twist_msg)
        elif self.front_min < self.min_distance and self.left_min < self.right_min:
            twist_msg = Twist()
            self.velocity_stuff = (self.front_min/4)
            self.angular_stuff = (1/self.right_min)
            twist_msg.linear.x = self.velocity_stuff
            twist_msg.angular.z = -self.angular_stuff
            self.cmd_vel_pub.publish(twist_msg)
    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            elapsed_time = current_time - self.start_time
            if elapsed_time.to_sec() > 60.0:
                rospy.loginfo("60 sn oldu.Kod duruyor.")
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                break  

            self.rate.sleep()
if __name__ == '__main__':
    try:
        robot = ReactiveRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
