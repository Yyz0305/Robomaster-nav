#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    try:
        # 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV Mat 对象
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 调整图像大小为640x480
        target_size = (640, 480)
        resized_image = cv2.resize(cv_image, target_size, interpolation=cv2.INTER_LINEAR)

        # 显示调整后的图像
        cv2.imshow("Image window", resized_image)
        cv2.waitKey(3)  # 每3毫秒刷新一次图像

    except CvBridgeError as e:
        print(e)

def main():
    # 初始化 ROS 节点
    rospy.init_node('py_test', anonymous=True)

    # 订阅图像话题
    image_subscriber = rospy.Subscriber("/hikrobot_camera/rgb", Image, image_callback)

    # 开始处理消息
    rospy.spin()

if __name__ == '__main__':
    main()