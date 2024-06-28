#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String

class ObstacleMonitor:
    def __init__(self):
        rospy.init_node('obstacle_monitor')

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        self.min_distance = 0.4  # 장애물 감지 임계값 (미터)
        self.stop_cmd = Twist()  # 정지 명령
        self.current_goal = None
        self.obstacle_detected = False

        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)

    def goal_callback(self, goal):
        self.current_goal = goal

    def lidar_callback(self, data):
        min_distance = min(data.ranges)

        if min_distance < self.min_distance:
            if not self.obstacle_detected:
                self.cancel_goal()
                self.cmd_vel_pub.publish(self.stop_cmd)
                rospy.loginfo("Obstacle detected! Stopping the robot.")
                self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                self.publish_goal(self.current_goal)
                rospy.loginfo("Obstacle cleared. Resuming the robot.")
                self.obstacle_detected = False

    def cancel_goal(self):
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)

    def publish_goal(self, goal):
        if goal:
            self.goal_pub.publish(goal)

if __name__ == '__main__':
    try:
        ObstacleMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
