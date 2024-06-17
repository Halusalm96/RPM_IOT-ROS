#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pymysql
import signal
import tf2_ros
from geometry_msgs.msg import TransformStamped

db = None

def update_robot_position(x, y, z, w):
    global db
    cursor = db.cursor()
    check_sql = "SELECT * FROM robot WHERE robot_name = %s"
    cursor.execute(check_sql, ('RPM-01',))
    existing_row = cursor.fetchone()

    if existing_row is None:
        insert_sql = "INSERT INTO robot (robot_name, robot_x, robot_y, robot_z, robot_w, robot_status) VALUES (%s, %s, %s, %s, %s, %s)"
        cursor.execute(insert_sql, ('RPM-01', x, y, z, w, 'online'))
    else:
        update_sql = "UPDATE robot SET robot_x = %s, robot_y = %s, robot_z = %s, robot_w = %s, robot_status = %s WHERE robot_name = %s"
        cursor.execute(update_sql, (x, y, z, w, 'online', 'RPM-01'))

    db.commit()
    cursor.close()

def update_robot_offline():
    global db
    cursor = db.cursor()
    update_sql = "UPDATE robot SET robot_status = 'offline' WHERE robot_name = 'RPM-01'"
    cursor.execute(update_sql)
    db.commit()
    cursor.close()

def signal_handler(signum, frame):
    global db
    rospy.loginfo("Node is shutting down...")
    update_robot_offline()
    if db:
        db.close()
    rospy.signal_shutdown("Node is Closed")

def main():
    global db
    rospy.init_node('testdb')

    try:
        db = pymysql.connect(
            host="13.124.83.151",
            user="root",
            password="1235",
            database="rpm"
        )
        rospy.loginfo("Database connected successfully")
    except Exception as e:
        rospy.logerr("Failed to connect to database: %s", e)
        return

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
    rospy.loginfo("Node started")

    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            x, y, z, w = transform.transform.translation.x, transform.transform.translation.y, transform.transform.rotation.z, transform.transform.rotation.w
            update_robot_position(x, y, z, w)
            rospy.loginfo("Updated position: x=%f, y=%f, z=%f, w=%f" % (x, y, z, w))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            continue

        rate.sleep()

    db.close()
    rospy.loginfo("Database connection closed")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C 종료 시 시그널 핸들러 등록
    try:
        main()
    except rospy.ROSInterruptException:
        pass
