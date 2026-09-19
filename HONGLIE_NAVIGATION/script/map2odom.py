#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import geometry_msgs.msg

def publish_transform():
    # 初始化节点
    rospy.init_node('tf_publisher_node_map')
    
    # 创建一个变换广播器
    br = tf.TransformBroadcaster()
    
    # 设置循环频率
    rate = rospy.Rate(10) # 10 Hz
    
    while not rospy.is_shutdown():
        # 定义变换数据
        translation = (0.0, 0.0, 0.0)  # 平移部分，这里假设没有平移
        # 假设我们有欧拉角表示的旋转（roll, pitch, yaw）
        euler_angles = (0, 0, 0)  # 示例值，根据实际情况更改
        
        # 将欧拉角转换为四元数
        quaternion = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
        
        # 发布变换
        br.sendTransform(translation,
                         quaternion,
                         rospy.Time.now(),
                         "odom",  # 子坐标系名称
                         "map")  # 父坐标系名称
        
        # 控制循环频率
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
