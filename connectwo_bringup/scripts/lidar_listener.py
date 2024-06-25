#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan

def lidar_callback(msg):
    # 여기에 라이다 데이터 처리를 추가하세요
    # 이 예제에서는 라이다 스캔 데이터를 출력합니다
    rospy.loginfo("Lidar Scan Data: {}".format(msg.ranges))

def lidar_listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_listener()
    except rospy.ROSInterruptException:
        pass

