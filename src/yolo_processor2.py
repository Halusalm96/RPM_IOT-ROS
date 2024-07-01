#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.yolo = YOLO("/home/user1/catkin_ws/src/nodelet_hello_world/yolov8s.pt")  # YOLO v8 모델 경로 지정
        self.yolo.to('cpu')  # 모델을 CPU로 전환
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)  # 카메라 이미지 토픽 구독
        self.result_pub = rospy.Publisher("/yolo_results", String, queue_size=1)  # 결과 문자열 토픽 발행

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # YOLO v8을 사용하여 객체 감지 수행
            results = self.yolo(cv_image)

            # 객체 감지 결과 처리
            detected_objects = []
            for result in results:
                for box in result.boxes:
                    obj_name = result.names[int(box.cls)]
                    confidence = box.conf.item()
                    detected_objects.append(f"{obj_name}: {confidence:.2f}")

            # 결과를 문자열로 변환하여 발행
            result_str = "; ".join(detected_objects)
            self.result_pub.publish(result_str)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('yolo_processor', anonymous=True)
    yolo_processor = YOLOProcessor()
    rospy.spin()
