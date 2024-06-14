#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import pymysql

# MariaDB 연결 설정
db = pymysql.connect(
    host="13.124.83.151",
    user="root",
    password="1235",
    database="rpm"
)

def update_coordinates(x, y):
    cursor = db.cursor()
    sql = "UPDATE Turtle SET robot_x = %s, robot_y = %s WHERE name = %s"
    cursor.execute(sql, (x, y, 'turtle1'))
    db.commit()
    cursor.close()

def listener():
    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)  # 1 Hz

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x, y = trans[0], trans[1]
            rospy.loginfo(f"Coordinates: x={x}, y={y}")
            update_coordinates(x, y)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        db.close()
