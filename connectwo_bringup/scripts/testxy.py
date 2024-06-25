#!/usr/bin/env python

import rospy
import tf
import pymysql

def update_robot_position(db, x, y):
    cursor = db.cursor()
    check_sql = "SELECT * FROM robot WHERE name = %s"
    cursor.execute(check_sql, ('RPM-01'))
    existing_row = cursor.fetchone()

    if existing_row is None:
        insert_sql = "INSERT INTO robot (name, current_x, current_y) VALUES (%s, %s, %s)"
        cursor.execute(insert_sql, ('RPM-01', x, y))
    else:
        update_sql = "UPDATE robot SET current_x = %s, current_y = %s WHERE name = %s"
        cursor.execute(update_sql, (x, y, 'RPM-01'))

    db.commit()
    cursor.close()

def main():
    rospy.init_node('turtlebot3_gazebo_to_db')

    db = pymysql.connect(
        host="192.168.0.118",
        user="rpm",
        password="11223344",
        database="rpm"
    )

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x, y = trans[0], trans[1]
            update_robot_position(db, x, y)
            rospy.loginfo("Updated position: x=%f, y=%f" % (x, y))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            continue

        rate.sleep()

    db.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
