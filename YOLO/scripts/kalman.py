import cv2
import numpy as np

class KalmanFilter:
    def __init__(self):
        # 初始化卡尔曼滤波器
        self.kf = cv2.KalmanFilter(4, 2)
        
        # 设置测量矩阵
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        
        # 设置状态转移矩阵
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        
        # 设置过程噪声协方差矩阵，增加对过程噪声的信任度（使预测更灵敏）
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        
        # 设置测量噪声协方差矩阵，减少对测量噪声的信任度（使预测更灵敏）
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1

    def predict(self, coordX, coordY):
        '''This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        
        # 校正卡尔曼滤波器的状态向量
        self.kf.correct(measured)
        
        # 预测下一时刻的状态
        predicted = self.kf.predict()
        
        x, y = int(predicted[0]), int(predicted[1])
        return x, y

# 示例使用
if __name__ == "__main__":
    kf = KalmanFilter()

    # 假设你有一些坐标数据，例如从传感器获得的数据
    measurements = [(100, 50), (110, 60), (120, 70), (130, 80), (140, 90)]

    for coord in measurements:
        coordX, coordY = coord
        predX, predY = kf.predict(coordX, coordY)
        print(f"Measured: ({coordX}, {coordY}), Predicted: ({predX}, {predY})")