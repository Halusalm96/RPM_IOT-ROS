#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import pymysql
import signal
from geometry_msgs.msg import TransformStamped

db = None

def update_robot_position(x, y, yaw):
    global db
    cursor = db.cursor()
    check_sql = "SELECT * FROM robot WHERE robot_name = %s"
    cursor.execute(check_sql, ('RPM-01',))
    existing_row = cursor.fetchone()

    if existing_row is None:
        insert_sql = "INSERT INTO robot (robot_name, robot_x, robot_y, robot_yaw, robot_status) VALUES (%s, %s, %s, %s, %s)"
        cursor.execute(insert_sql, ('RPM-01', x, y, yaw, 'online'))
    else:
        update_sql = "UPDATE robot SET robot_x = %s, robot_y = %s, robot_yaw = %s, robot_status = %s WHERE robot_name = %s"
        cursor.execute(update_sql, (x, y, yaw, 'online', 'RPM-01'))

    db.commit()
    cursor.close()

def update_robot_offline():
    global db
    cursor = db.cursor()
    update_sql = "UPDATE robot SET robot_status = 'offline' WHERE robot_name = 'RPM-01'"
    cursor.execute(update_sql)
    db.commit()
    cursor.close()

def tf_callback(tf_msg):
    global db
    trans = tf_msg.transform.translation
    rot = tf_msg.transform.rotation

    # 변환 메시지에서 x, y 좌표와 회전(yaw)을 읽어옴
    x, y = trans.x, trans.y

    # 쿼터니언을 오일러 각도로 변환하여 yaw(회전)을 얻음
    quaternion = (rot.x, rot.y, rot.z, rot.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]  # Z 축 방향의 회전 (yaw)을 얻음

    update_robot_position(x, y, yaw)
    rospy.loginfo("Updated position: x=%f, y=%f, yaw=%f" % (x, y, yaw))

def signal_handler(signal, frame):
    global db
    rospy.loginfo("Node is Close..")
    update_robot_offline()
    rospy.signal_shutdown("Node is Closed")

def main():
    global db
    rospy.init_node('testdb')

    db = pymysql.connect(
        host="13.124.83.151",
        user="root",
        password="1235",
        database="rpm"
        )

    rospy.Subscriber('/tf', TransformStamped, tf_callback)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()

    db.close()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C 종료 시 시그널 핸들러 등록
    try:
        main()
    except rospy.ROSInterruptException:
        pass
