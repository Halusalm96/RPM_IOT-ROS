#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import pymysql
import signal
from geometry_msgs.msg import TransformStamped

db = None
current_x = 0.0
current_y = 0.0
current_z = 0.0
current_w = 0.0

def update_robot_position():
    global db, current_x, current_y, current_z, current_w
    cursor = db.cursor()
    check_sql = "SELECT * FROM robot WHERE robot_name = %s"
    cursor.execute(check_sql, ('RPM-01',))
    existing_row = cursor.fetchone()

    if existing_row is None:
        insert_sql = "INSERT INTO robot (robot_name, robot_x, robot_y, robot_z, robot_w, robot_status) VALUES (%s, %s, %s, %s, %s, %s)"
        cursor.execute(insert_sql, ('RPM-01', current_x, current_y, current_z, current_w, 'Online'))
    else:
        update_sql = "UPDATE robot SET robot_x = %s, robot_y = %s, robot_z = %s, robot_w = %s, robot_status = %s WHERE robot_name = %s"
        cursor.execute(update_sql, (current_x, current_y, current_z, current_w, 'Online', 'RPM-01'))

    db.commit()
    cursor.close()

def update_robot_offline():
    global db
    cursor = db.cursor()
    update_sql = "UPDATE robot SET robot_status = 'Offline' WHERE robot_name = 'RPM-01'"
    cursor.execute(update_sql)
    db.commit()
    cursor.close()

def tf_callback(tf_msg):
    global current_x, current_y, current_z, current_w
    trans = tf_msg.transform.translation
    rot = tf_msg.transform.rotation

    # 변환 메시지에서 x, y 좌표와 z, w 값을 읽어옴
    current_x, current_y, current_z, current_w = trans.x, trans.y, rot.z, rot.w

def signal_handler(signal, frame):
    global db
    rospy.loginfo("Node is shutting down...")
    update_robot_offline()
    rospy.signal_shutdown("Node is Closed")

def timer_callback(event):
    update_robot_position()
    rospy.loginfo("Updated position to database")

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

    rospy.Subscriber('/tf', TransformStamped, tf_callback)
    rospy.loginfo("Subscribed to /tf")

    rospy.Timer(rospy.Duration(0.5), timer_callback)
    rospy.loginfo("Timer started")

    rospy.spin()

    db.close()
    rospy.loginfo("Database connection closed")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C 종료 시 시그널 핸들러 등록
    try:
        main()
    except rospy.ROSInterruptException:
        pass
