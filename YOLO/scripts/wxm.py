#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import cv2
import rospy
import math
sys.path.append(r".\MvImport")
from MvCameraControl_class import *
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
import torch
from kalman import KalmanFilter
from traditional_vision import VisionCore,visiontest
from yjm_pnp import perform_pnp
from threading import Thread, Lock
import queue


# #pitch的pid参数
# KP_PITCH = -13
# KD_PITCH = -0.1


# #yaw的pid参数
# KP_YAW   = 15.0
# KD_YAW   = 0.2

#pitch的pid参数
KP_PITCH = 0
KD_PITCH = 0


#yaw的pid参数
KP_YAW   = 15.0
KD_YAW   = 0.0

####跟踪模式：0.跟踪原始目标点   1.跟踪预测目标点
chase_mod = 1

def Enum_device(tlayerType, deviceList):
    """
    ch:枚举设备 | en:Enum device
    nTLayerType [IN] 枚举传输层 ，pstDevList [OUT] 设备列表
    """
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0:
        print("enum devices fail! ret[0x%x]" % ret)
        sys.exit()

    if deviceList.nDeviceNum == 0:
        print("find no device!")
        sys.exit()

    print("Find %d devices!" % deviceList.nDeviceNum)

    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\ngige device: [%d]" % i)
            # 输出设备名字
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)
            # 输出设备ID
            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
        # 输出USB接口的信息
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nu3v device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("user serial number: %s" % strSerialNumber)


def enable_device(nConnectionNum):
    """
    设备使能
    :param nConnectionNum: 设备编号
    :return: 相机, 图像缓存区, 图像数据大小
    """
    # ch:创建相机实例 | en:Creat Camera Object
    cam = MvCamera()

    # ch:选择设备并创建句柄 | en:Select device and create handle
    # cast(typ, val)，这个函数是为了检查val变量是typ类型的，但是这个cast函数不做检查，直接返回val
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
        nPacketSize = cam.MV_CC_GetOptimalPacketSize()
        if int(nPacketSize) > 0:
            ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
            if ret != 0:
                print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
        else:
            print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

    # ch:设置触发模式为off | en:Set trigger mode as off
    ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    if ret != 0:
        print("set trigger mode fail! ret[0x%x]" % ret)
        sys.exit()

    # 从这开始，获取图片数据
    # ch:获取数据包大小 | en:Get payload size
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    # MV_CC_GetIntValue，获取Integer属性值，handle [IN] 设备句柄
    # strKey [IN] 属性键值，如获取宽度信息则为"Width"
    # pIntValue [IN][OUT] 返回给调用者有关相机属性结构体指针
    # 得到图片尺寸，这一句很关键
    # payloadsize，为流通道上的每个图像传输的最大字节数，相机的PayloadSize的典型值是(宽x高x像素大小)，此时图像没有附加任何额外信息
    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()

    nPayloadSize = stParam.nCurValue

    # ch:开始取流 | en:Start grab image
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("start grabbing fail! ret[0x%x]" % ret)
        sys.exit()
    #  返回获取图像缓存区。
    data_buf = (c_ubyte * nPayloadSize)()
    #  date_buf前面的转化不用，不然报错，因为转了是浮点型
    return cam, data_buf, nPayloadSize


def get_image(data_buf, nPayloadSize):
    """
    获取图像
    :param data_buf:
    :param nPayloadSize:
    :return: 图像
    """
    # 输出帧的信息
    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    # void *memset(void *s, int ch, size_t n);
    # 函数解释:将s中当前位置后面的n个字节 (typedef unsigned int size_t )用 ch 替换并返回 s
    # memset:作用是在一段内存块中填充某个给定的值，它是对较大的结构体或数组进行清零操作的一种最快方法
    # byref(n)返回的相当于C的指针右值&n，本身没有被分配空间
    # 此处相当于将帧信息全部清空了
    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

    # 采用超时机制获取一帧图片，SDK内部等待直到有数据时返回，成功返回0
    ret = cam.MV_CC_GetOneFrameTimeout(data_buf, nPayloadSize, stFrameInfo, 1000)
    if ret == 0:
        print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
            stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
    else:
        print("no data[0x%x]" % ret)

    image = np.asarray(data_buf)  # 将c_ubyte_Array转化成ndarray得到（3686400，）
    image = image.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, -1))  # 根据自己分辨率进行转化
    return image


def close_device(cam, data_buf):
    """
    关闭设备
    :param cam:
    :param data_buf:
    """
    # ch:停止取流 | en:Stop grab image
    ret = cam.MV_CC_StopGrabbing()
    if ret != 0:
        print("stop grabbing fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()

    # ch:关闭设备 | Close device
    ret = cam.MV_CC_CloseDevice()
    if ret != 0:
        print("close deivce fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()

    # ch:销毁句柄 | Destroy handle
    ret = cam.MV_CC_DestroyHandle()
    if ret != 0:
        print("destroy handle fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()

    del data_buf
integral = 0
def PID_pitch(err, kp, ki, kd, last_err_pitch):
    # 比例部分
    global integral
    # print(last_err_pitch)
    p_out = kp * err
    
    # 积分部分
    integral += err
    i_out = ki * integral
    
    # 微分部分
    d_out = kd * (err - last_err_pitch)
    
    # 计算总的输出
    output = p_out + i_out + d_out
    
    # 更新上一次的误差
    previous_error = err
    
    return output

def PID_yaw(err, kp, ki, kd, last_err_yaw):
    # 比例部分
    global integral
    # print(last_err_pitch)
    p_out = kp * err
    
    # 积分部分
    integral += err
    i_out = ki * integral
    
    # 微分部分
    d_out = kd * (err - last_err_yaw)
    
    # 计算总的输出
    output = p_out + i_out + d_out
    
    # 更新上一次的误差
    previous_error = err
    
    return output

def limit(number,min,max):
    if number <= min:
        number = min+1
    if number >= max:
        number = max-1
    return number

def sort_lightband(lightband):
    # 首先按x坐标升序排序，如果x相同，则按y降序排序（因为图像坐标系中，y向下增加）
    sorted_band = sorted(lightband, key=lambda p: (p[0], -p[1]))
    
    # 分别获取最左边和最右边的两个点
    left_points = sorted_band[:2]
    right_points = sorted_band[2:]
    
    # 对左边和右边的点分别按y值升序排序，以确定上下的位置
    left_points_sorted = sorted(left_points, key=lambda p: p[1])
    right_points_sorted = sorted(right_points, key=lambda p: p[1])
    
    # 组合排序后的结果，使其符合左上、右上、左下、右下的顺序
    result = [left_points_sorted[0], right_points_sorted[0], left_points_sorted[1], right_points_sorted[1]]
    
    return result


def IOU(rect1_top_left, rect1_bottom_right, rect2_top_left, rect2_bottom_right):
    # x1, y1, x2, y2 = box1
    # x3, y3, x4, y4 = box2
    x1, y1 = rect1_top_left
    x2, y2 = rect1_bottom_right
    x3, y3 = rect2_top_left
    x4, y4 = rect2_bottom_right
    x_inter1 = max(x1, x3)#左上点的最大横坐标
    y_inter1 = max(y1, y3)#左上点的最大纵坐标
    x_inter2 = min(x2, x4)#右下点的最小横坐标
    y_inter2 = min(y2, y4)#右下点的最小纵坐标
    width_inter = abs(x_inter2 - x_inter1)
    height_inter = abs(y_inter2 - y_inter1)
    area_inter = width_inter * height_inter

    return area_inter
def iou(A_top_left, A_bottom_right, B_top_left, B_bottom_right):
    # A_top_left, A_bottom_right: Rectangle A's top-left and bottom-right coordinates (tuple or list)
    # B_top_left, B_bottom_right: Rectangle B's top-left and bottom-right coordinates (tuple or list)
    
    # Calculate the (x, y) coordinates of the intersection rectangle
    xA = max(A_top_left[0], B_top_left[0])
    yA = max(A_top_left[1], B_top_left[1])
    xB = min(A_bottom_right[0], B_bottom_right[0])
    yB = min(A_bottom_right[1], B_bottom_right[1])
    
    # Compute the area of intersection rectangle
    interArea = max(0, xB - xA) * max(0, yB - yA)
    
    # Compute the area of both rectangles
    boxAArea = abs((A_bottom_right[0] - A_top_left[0]) * (A_bottom_right[1] - A_top_left[1]))
    boxBArea = abs((B_bottom_right[0] - B_top_left[0]) * (B_bottom_right[1] - B_top_left[1]))
    
    # Compute the intersection over union by taking the intersection area and dividing it by the sum of prediction + ground-truth areas - the intersection area
    iou = interArea / float(boxAArea + boxBArea - interArea)
    
    return iou



MID_X = 1536
MID_Y = 1024
WIDTH = 3072
HEIGHT = 2048
last_err_pitch = 0
last_err_yaw   = 0
last_target_x1,last_target_y1,last_target_x2,last_target_y2 = 0,0,0,0
last_target_exist = 0
model = YOLO("//home//ubuntu//catkin_ws//src//YOLO//scripts//20000best.pt")  # 使用'nano'大小的模型作为示例
if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('camera_publisher', anonymous=True)
    
    # 创建一个Publisher，发布名为'camera_coords'的话题，消息类型为Float32MultiArray
    coords_pub = rospy.Publisher('camera_coords', Float32MultiArray, queue_size=10)

    # 检查是否有可用的GPU
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")


    model.to(device)

    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    Enum_device(tlayerType, deviceList)

    cam, data_buf, nPayloadSize = enable_device(0)

    cam.MV_CC_SetEnumValue("ExposureAuto",0)
    cam.MV_CC_SetFloatValue("ExposureTime",5000)
    cam.MV_CC_SetEnumValue("GainAuto", MV_GAIN_MODE_OFF)
    cam.MV_CC_SetFloatValue("Gain", 25.0)


    kf = KalmanFilter()
    
    data1_last = 0
    data2_last = 0

    while not rospy.is_shutdown():
        try:
            # 尝试获取图像
            image = get_image(data_buf, nPayloadSize)
            
            # 在这里继续处理image_rgb...
            
        except Exception as e:
            # 打印错误信息
            rospy.logerr(f"Error occurred in get_image or image processing: {e}")
            # 发生错误时跳过当前循环的剩余部分，继续下一次循环
            continue
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # 将BGR转换为RGB

        # 获取原始图像尺寸
        original_height, original_width = image_rgb.shape[:2]

        # # 计算新尺寸，长宽都压缩一半
        # new_width = int(original_width)
        # new_height = int(original_height)

        # # 使用OpenCV resize函数调整图像大小
        # resized_image = cv2.resize(image_rgb, (new_width, new_height), interpolation=cv2.INTER_AREA)

        resized_image = cv2.rotate(image_rgb, cv2.ROTATE_180)

        # # 将图像转换为Tensor，并移动到GPU上（如果可用）
        # image_tensor = torch.from_numpy(resized_image).permute(2, 0, 1).float().unsqueeze(0).to(device)

        # 使用YOLOv8进行推理
        results = model(resized_image)  # 推理结果
        print("#################################################")
        
        target_x1,target_y1,target_x2,target_y2 = 0,0,0,0
        
        target_size = 0
        exist_tar = 0
        max_size = 0
        best_light = []
        length = 0 #灯条长度，用来近似估计距离以调整弹道
        chonghe_rate = 0
        exist_temp = 0
        angle1 = 0
        angle2 = 0
        for i, result in enumerate(results):
            # 提取每个检测框的信息
            boxes = result.boxes  # 获取所有检测框
            for box in boxes:
                # 每个box通常包含xyxy(边界框坐标), conf(置信度), cls(类别编号)
                xyxy = box.xyxy  # 边界框坐标 (x1, y1, x2, y2)
                conf = box.conf  # 置信度分数
                cls = box.cls  # 类别编号

                # 如果需要类别的名称而不是编号，可以这样做：
                class_name = model.names[int(cls)]  # 获取对应编号的类别名称
                if class_name == 'blue':

                    # print(f"Detected object: {class_name}, Confidence: {float(conf):.2f}, BBox: {xyxy}")
                    # print(int(xyxy[0][0]))
                    x1 = int(xyxy[0][0])
                    y1 = int(xyxy[0][1])
                    x2 = int(xyxy[0][2])
                    y2 = int(xyxy[0][3])
                    size = abs((y1-y2)*(x1-x2))
                    # print("ori",size,x1)

                    roi = resized_image[y1:y2,x1:x2]
                    roi,light_band = visiontest(roi)
                    print("len_of_the_light:",len(light_band))
                    if len(light_band)>0:   #存在两个灯条
                        light_band = sort_lightband(light_band)##左上右上左下右下
                        # print(light_band)
                        light_band[0][0] = limit(light_band[0][0] + x1,0,WIDTH)
                        light_band[0][1] = limit(light_band[0][1] + y1,0,HEIGHT)
                        light_band[1][0] = limit(light_band[1][0] + x1,0,WIDTH)
                        light_band[1][1] = limit(light_band[1][1] + y1,0,HEIGHT)
                        light_band[2][0] = limit(light_band[2][0] + x1,0,WIDTH)
                        light_band[2][1] = limit(light_band[2][1] + y1,0,HEIGHT)
                        light_band[3][0] = limit(light_band[3][0] + x1,0,WIDTH)
                        light_band[3][1] = limit(light_band[3][1] + y1,0,HEIGHT)
                        

                    # cv2.namedWindow("ROI", cv.2WINDOW_NORMAL)
                    # cv2.imshow("ROI",roi)


                    #筛选条件:1、存在两个灯条   2、与上一个装甲板有重合面积（直接要）   3、不存在与上一个装甲板有重合面积的—>找最大的
                    if len(light_band) > 0:

                        chonghe_area = iou((x1,y1),(x2,y2),(last_target_x1,last_target_y1),(last_target_x2,last_target_y2))
                        

                        if chonghe_area > 0:
                            target_x1 = x1
                            target_y1 = y1
                            target_x2 = x2
                            target_y2 = y2
                            max_size = size
                            best_light = light_band
                            chonghe_rate = chonghe_area
                            exist_temp = 1
                            # break
                        if size > max_size and exist_temp == 0:
                            target_x1 = x1
                            target_y1 = y1
                            target_x2 = x2
                            target_y2 = y2
                            max_size = size
                            best_light = light_band
                            chonghe_rate = chonghe_area
                            
                            
                    # print(x1,y1,x2,y2)
        print("#################################################")
        # WIDTH:3072 HEIGHT:2048
        # WIDTH/2:1536 HEIGHT:1024
        
        
        pre_x = 0
        pre_y = 0
        if max_size == 0: #压根不存在装甲板
            dx = 0
            dy = 0
            pre_dx = 0
            pre_dy = 0
            pitch = 0
            yaw = 0
            last_err_pitch = 0
            last_err_yaw = 0
            exist_tar = 0#目标不存在
            last_target_x1,last_target_y1,last_target_x2,last_target_y2 = 0,0,0,0
            last_target_exist = 0
            
        else:

            last_target_x1 = target_x1
            last_target_y1 = target_y1
            last_target_x2 = target_x2
            last_target_y2 = target_y2
            last_target_exist = 1

            dx = int((target_x1+target_x2)/2 - MID_X)
            dy = int((target_y1+target_y2)/2 - MID_Y)
            
            pre_x,pre_y = kf.predict((target_x1+target_x2)/2, (target_y1+target_y2)/2)
            # pre_dx = pre_x - MID_X
            # pre_dy = pre_y - MID_Y
            pre_dx = int(pre_x - (target_x1+target_x2)/2)
            pre_dy = int(pre_y - (target_y1+target_y2)/2)
            exist_tar = 1
            
            if len(best_light) > 0:
                # 连接四个点形成矩形
                cv2.line(resized_image, (best_light[0][0], best_light[0][1]), (best_light[1][0], best_light[1][1]), (255, 255, 255), 2)  # 左上到右上
                cv2.line(resized_image, (best_light[1][0], best_light[1][1]), (best_light[3][0], best_light[3][1]), (255, 255, 255), 2)  # 右上到右下
                cv2.line(resized_image, (best_light[3][0], best_light[3][1]), (best_light[2][0], best_light[2][1]), (255, 255, 255), 2)  # 右下到左下
                cv2.line(resized_image, (best_light[2][0], best_light[2][1]), (best_light[0][0], best_light[0][1]), (255, 255, 255), 2)  # 左下到左上
                cv2.line(resized_image, (best_light[0][0], best_light[0][1]), (best_light[3][0], best_light[3][1]), (255, 255, 255), 2)  # 左上到右上
                cv2.line(resized_image, (best_light[2][0], best_light[2][1]), (best_light[1][0], best_light[1][1]), (255, 255, 255), 2)  # 右上到右下
                success,angle1,angle2,distance = perform_pnp([
                    [-6.5, 3, 0],   # 左上角
                    [6.5, 3, 0],    # 右上角
                    [-6.5, -3, 0],  # 左下角
                    [6.5, -3, 0]    # 右下角
                ],
                [
                    [best_light[0][0], best_light[0][1]],  # 左上角在图像中的位置
                    [best_light[1][0], best_light[1][1]],  # 右上角在图像中的位置
                    [best_light[2][0], best_light[2][1]],  # 左下角在图像中的位置
                    [best_light[3][0], best_light[3][1]]   # 右下角在图像中的位置
                ]
                )

                my_text1 = "pitch:"+str(angle1)
                my_text2 = "yaw:"+str(angle2)
                my_text3 = "distance"+str(distance)
                my_text4 = "IOU"+str(chonghe_rate)
                my_text5 = "last"+str(last_target_x1)
                if success:
                    cv2.putText(resized_image, my_text1, (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 5)
                    cv2.putText(resized_image, my_text2, (100, 300), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 5)
                    cv2.putText(resized_image, my_text3, (100, 450), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 5)
                    if chonghe_rate == 0:
                        cv2.putText(resized_image, my_text4, (100, 600), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 5)
                        cv2.putText(resized_image, my_text5, (100, 750), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 5)
                    else:
                        cv2.putText(resized_image, my_text4, (100, 600), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 5)

            # pitch = PID_pitch(pre_y,KP_PITCH,0,KD_PITCH,last_err_pitch)
            
            # yaw = PID_pitch(pre_x,KP_YAW,0,KD_YAW,last_err_yaw)
            # if chase_mod == 1:
            #     pitch = KP_PITCH * pre_dy + KD_PITCH * (pre_dy - last_err_pitch)
            #     yaw   = KP_YAW   * pre_dx + KD_YAW   * (pre_dx - last_err_yaw  )

            #     # print("bug",pre_dx,last_err_yaw,KP_YAW   * pre_x,  KD_YAW   * (pre_dx - last_err_yaw  ))
            #     last_err_pitch = pre_dy
            #     last_err_yaw   = pre_dx

            # elif chase_mod == 0:
            #     pitch = KP_PITCH * dy + KD_PITCH * (dy - last_err_pitch)
            #     yaw   = KP_YAW   * dx + KD_YAW   * (dx - last_err_yaw)

            #     last_err_pitch = dy
            #     last_err_yaw   = dx
            

            #roi = resized_image[target_y1:target_y2,target_x1:target_x2]
            #roi,light_band = visiontest(roi)
            #print("len_of_the_light:",len(light_band))
            #cv2.namedWindow("ROI", cv2.WINDOW_NORMAL)
            #cv2.imshow("ROI",roi)

            # print("灯条长度",length)
            # print("yolo高度",abs(target_y1-target_y2))
            # print("PID_PITCH",dy,pre_dy,pitch,last_err_pitch)
            # print("PID_YAW",dx,pre_dx,yaw,last_err_yaw)
           
            
            # print(pitch)
            

        annotated_frame = results[0].plot()
        if exist_tar == 1:
            cv2.circle(annotated_frame, (int((target_x1+target_x2)/2),int((target_y1+target_y2)/2)), 10, (255, 255, 255), -1)
        cv2.circle(annotated_frame, (int(pre_x),int(pre_y)), 10, (255, 0, 0), -1)
        cv2.circle(annotated_frame, (MID_X,MID_Y), 10, (0, 0, 255), -1)
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", annotated_frame)

        # 这里假设你已经有了计算得到的y和z值
        # pitch = 1.0  # 示例数据，请替换为实际获取的数据
        # yaw = 0  # 示例数据，请替换为实际获取的数据
        
        # 构造消息
        # data2 = -math.radians(angle2)*500  #yaw轴数据，转弧度制然后×1000
#         data2 = -math.radians(angle2)

#         data2 = KP_YAW * data2 + KD_YAW * (data2 - data2_last)

#         data2_last = data2

#         data2 = data2 * 1000
# ##############################################
#         data1 = -math.radians(angle1)

#         data1 = KP_PITCH * data1 + KD_PITCH * (data1 - data1_last)

#         data1_last = data1

#         data1 = data1 * 1000

        # data1 = math.radians(angle1)*500  #pitch轴数据，转弧度制然后×1000
        # data1 = 0.0
        array = Float32MultiArray(data=[angle1, angle2])
        coords_pub.publish(array)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    # 关闭设备
    close_device(cam, data_buf)