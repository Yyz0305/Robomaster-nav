import cv2
import numpy as np
import math
DEFAULT_CAMERA_MATRIX = np.array([
    [5.34940377e+03, 0.00000000e+00, 2.06096533e+03],
    [0.00000000e+00, 5.32825871e+03, 9.10977048e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)

DEFAULT_DIST_COEFFS = np.array([
    1.86467053e-01, -2.49590197e+00, -3.39206580e-03, 
    3.27667115e-02, 1.60610570e+01
], dtype=np.float32)

def radians_to_degrees(radian_value):
    """
    将弧度转换为角度。
    
    参数:
    radian_value -- 弧度值(float)。
    
    返回:
    转换后的角度值(float)。
    """
    return math.degrees(radian_value)

# 示例：将π弧度转换为角度
radians = math.pi
degrees = radians_to_degrees(radians)

def perform_pnp(object_points, image_points, camera_matrix=DEFAULT_CAMERA_MATRIX, dist_coeffs=DEFAULT_DIST_COEFFS):
    """
    使用OpenCV执行PnP操作。
    
    参数:
    - object_points: 一个包含4个3D点坐标的列表或numpy数组。
    - image_points: 一个包含4个对应2D点坐标的列表或numpy数组。
    - camera_matrix: (可选) 相机内参矩阵，默认使用预定义的矩阵。
    - dist_coeffs: (可选) 畸变系数，默认使用预定义的系数。
    
    返回:
    - 成功标志。
    - 旋转向量。
    - 平移向量。
    """
    # 确保输入是numpy数组
    object_points = np.array(object_points, dtype=np.float32)
    image_points = np.array(image_points, dtype=np.float32)

    # 使用solvePnP进行PnP求解
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    distance = np.linalg.norm(tvec)
    # print(rvec)
    # print(tvec)
    # print(distance)
    print(tvec[0],tvec[1],tvec[2])
    angle1 = radians_to_degrees(math.atan2(tvec[1],tvec[2]))
    angle2 = radians_to_degrees(math.atan2(tvec[0],tvec[2]))
    print(angle1,angle2)
    return success, round(angle1,3), round(angle2,3),round(distance,3)