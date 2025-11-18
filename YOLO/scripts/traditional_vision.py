import cv2
import numpy as np
def VisionCore(img:cv2.Mat):

    gray_img=img.copy()
    gray_img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    
    thrd=10
    i=0
    binary:cv2.Mat
    while(True):
        r,binary=cv2.threshold(gray_img,thrd,255,0)
        #print(cv2.mean(binary)[0])
        i+=1
        thrd+=10
        if(i==20 or cv2.mean(binary)[0]<10):
            #print(" ")
            break 
    thrd -= 10
    blue, g, r = cv2.split(img)
    #print(cv2.mean(img)[0])
    light_img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    #thrd=GetThrd(img)
    r,binary=cv2.threshold(light_img,thrd,255,0)
    # r,binary= cv2.threshold(light_img, 255, 0, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #第一次腐蚀，去掉小光点
    kernel=np.ones((3,3),np.uint8)
    binary=cv2.erode(binary,kernel)
    #第一次膨胀，方便轮廓检测
    kernel=np.ones((5,5),np.uint8)
    binary=cv2.dilate(binary,kernel)
    #边缘检测
    binary=cv2.Canny(binary,0,255)
    #第二次膨胀，加粗轮廓
    #kernel=np.ones((3,3),np.uint8)
    #binary=cv2.dilate(binary,kernel)
    #cv2.imshow('b', binary)
    #得到轮廓点集
    contours,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    coutours_feature=[]
    area_sum=0
    global last_targets
    light_num = 0
    aver_length = 0
    MAX = 0
    for con in contours:
        #获得轮廓面积
        area=cv2.contourArea(con)
        #面积特别小的，筛掉
        if(area<10):
            continue
        #获得矩形
        rect=cv2.minAreaRect(con)
        # print(rect)
        #获得中点
        cx,cy=rect[0]
        #不是竖着的，筛掉
        if(rect[1][1]>rect[1][0] and rect[2]>70):
            continue
        if(rect[1][1]<rect[1][0] and rect[2]<20):
            continue
        angle:float
        if(rect[1][1]>rect[1][0]):
            max_l=rect[1][1]
            min_l=rect[1][0]
            angle=rect[2]
        else:
            max_l=rect[1][0]
            min_l=rect[1][1]
            angle=rect[2]-90

        #通过宽高比筛选
        if(max_l/min_l<0.9 or max_l/min_l>10):
            continue

        #中点float转int
        cx=int(cx)
        cy=int(cy)
        area_sum+=area
        #添加进轮廓特征集：（面积、中点坐标、高、角度）
        coutours_feature.append((area,(cx,cy),max_l,angle))
        # cv2.circle(img, (int(cx),int(cy)), 50, (0, 255, 0), -1)
        light_num = light_num + 1
        aver_length = aver_length + max_l
        # print("light_length:",max_l)
        if MAX < max_l:
            MAX = max_l

    if light_num != 0:
        aver_length = aver_length / light_num
    
    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
    return binary,MAX
        

def visiontest(img:cv2.Mat):

    binary=img.copy()
    binary=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    thr,binary = cv2.threshold(binary, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #第一次腐蚀，去掉小光点
    kernel=np.ones((3,3),np.uint8)
    binary=cv2.erode(binary,kernel)
    #第一次膨胀，方便轮廓检测
    kernel=np.ones((5,5),np.uint8)
    binary=cv2.dilate(binary,kernel)
    binary=cv2.Canny(binary,0,255)
    # cv2.imshow()
    # cv2.imshow("gray",binary)
    MAX = 0
    contours,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    coutours_feature=[]
    area_sum=0
    global last_targets
    light_num = 0
    aver_length = 0
    MAX = 0
    light_band = []
    for con in contours:
        #获得轮廓面积
        area=cv2.contourArea(con)
        #面积特别小的，筛掉
        if(area<10):
            continue
        #获得矩形
        rect=cv2.minAreaRect(con)
        # print(rect)
        #获得中点
        cx,cy=rect[0]
        #不是竖着的，筛掉
        if(rect[1][1]>rect[1][0] and rect[2]>70):
            continue
        if(rect[1][1]<rect[1][0] and rect[2]<20):
            continue
        angle:float
        if(rect[1][1]>rect[1][0]):
            max_l=rect[1][1]
            min_l=rect[1][0]
            angle=rect[2]
        else:
            max_l=rect[1][0]
            min_l=rect[1][1]
            angle=rect[2]-90

        #通过宽高比筛选
        if(max_l/min_l<0.9 or max_l/min_l>10):
            continue

        p4_rect = cv2.boxPoints(rect)
        # print(p4_rect)
        # 根据纵坐标对点进行排序

        sorted_points = sorted(p4_rect, key=lambda point: point[1])

        # 将点分为上下两组
        upper_group = sorted_points[:2]
        lower_group = sorted_points[2:]

        # 计算上组和下组的平均坐标
        upper_avg = np.mean(upper_group, axis=0)
        lower_avg = np.mean(lower_group, axis=0)

        # 将结果存储在 light 列表中
        light = [upper_avg.tolist(), lower_avg.tolist()]

        # print(f"Upper point: {light[0]}")
        # print(f"Lower point: {light[1]}")

        # 绘制计算得到的两个平均点
        upper_pt = tuple(map(int, upper_avg))
        lower_pt = tuple(map(int, lower_avg))
        light_band.append([upper_pt[0],upper_pt[1]])
        light_band.append([lower_pt[0],lower_pt[1]])
        # cv2.circle(img, upper_pt, 15, (255, 0, 0), -1)
        # cv2.circle(img, lower_pt, 15, (0, 0, 255), -1)

        #中点float转int
        cx=int(cx)
        cy=int(cy)
        area_sum+=area
        #添加进轮廓特征集：（面积、中点坐标、高、角度）
        coutours_feature.append((area,(cx,cy),max_l,angle))
        # cv2.circle(img, (int(cx),int(cy)), 50, (0, 255, 0), -1)

        # print("light_length:",max_l)

    
    # print(len(light_band))
    if len(light_band) == 4:
        return img,light_band
    else:
        return img,[]

        