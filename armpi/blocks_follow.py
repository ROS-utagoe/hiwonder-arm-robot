#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# 色块跟随，约三秒不动就抓取
import cv2
import numpy as np
import time
import threading
import signal
import LeArm
import kinematics as kin
import RPi.GPIO as GPIO

debug = True
ball_color = 'green'

stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
cap = cv2.VideoCapture(stream)

orgFrame = None
Running = False
get_d = False
# 校准按键
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
key = 22
GPIO.setup(key, GPIO.IN, GPIO.PUD_UP)
# 校准标志
correction_flag = False

# 要识别的颜色字典
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              }
# 色块颜色，位置列表
position_color_list = []
# 识别到色块标志
cv_blocks_ok = False
#
x_pix_cm = 0
y_pix_cm = 0
c_angle = 0

sec_3_flag = False
last_x = 0
no_move_count = 0
no_move_last_x = 0
step = 0

# 暂停信号的回调
def cv_stop(signum, frame):
    global Running

    print("Stop face detection")
    if Running is True:
        Running = False
    cv2.destroyAllWindows()

# 继续信号的回调
def cv_continue(signum, frame):
    global stream
    global Running
    global cap
    
    if Running is False:
        cap = cv2.VideoCapture(stream)
        Running = True

#   注册信号回调
signal.signal(signal.SIGTSTP, cv_stop)
signal.signal(signal.SIGCONT, cv_continue)

def setDeviation():
    global get_d
    while True:
        if get_d :
            get_d = False
            LeArm.setServo(1, 700, 500)
            kin.ki_move(n_x, n_y, 750.0, 500)
        else:
            time.sleep(0.01)

th1= threading.Thread(target=setDeviation)
th1.setDaemon(True)
th1.start()       

# 色块3秒不动，进行抓取
def grab_blocks():
    global sec_3_flag, no_move_last_x, x_pix_cm, step
    global position_color_list, y_pix_cm
    if sec_3_flag:
        if -10 < x_pix_cm - no_move_last_x < 10:
            while True:
                if step == 0:
                    xx = position_color_list[0][0]
                    yy = position_color_list[0][1]
                    angle = position_color_list[0][2]
                    # 数据映射
                    n_xx = int(leMap(xx, 0.0, 320.0, -1250.0, 1250.0)) * 1.0
                    n_yy = int(leMap(240 - yy, 0.0, 240.0, 1250, 3250.0)) * 1.0
                    # 需要根据实际情况调整，偏差主要来自舵机的虚位
                    if n_xx < -100:
                        n_xx -= 120  # 偏差
                    LeArm.setServo(1, 700, 500)
                    time.sleep(0.5)
                    step = 1
                elif step == 1:
                    # 机械臂下去
                    if kin.ki_move(n_xx, n_yy, 200.0, 1500):
                        step = 2
                    else:
                        step = 6
                elif step == 2:
                    # 根据方块的角度转动爪子
                    if angle <= -45:
                        angle = -(90 + angle)
                    n_angle = leMap(angle, 0.0, -45.0, 1500.0, 1750.0)
                    if n_xx > 0:
                        LeArm.setServo(2, 3000 - n_angle, 500)
                    else:
                        LeArm.setServo(2, n_angle, 500)
                    time.sleep(0.5)
                    step = 3
                elif step == 3:
                    # 抓取
                    print ('3 ok')
                    LeArm.setServo(1, 1200, 500)
                    time.sleep(0.5)
                    step = 4
                elif step == 4:  # 将方块提起
                    print ('4 ok')
                    kin.ki_move(n_xx, n_yy, 700.0, 1000)
                    step = 5
                elif step == 5:
                    print ('5 ok')
                    LeArm.runActionGroup('green', 1)
                    step = 6
                elif step == 6:
                    print ('6 ok')
                    LeArm.setServo(1, 700, 500)
                    time.sleep(0.5)
                    LeArm.runActionGroup('rest', 1)
                    position_color_list = []
                    sec_3_flag = False
                    step = 0
                    x_pix_cm = 0
                    y_pix_cm = 0
                    break
        no_move_last_x = x_pix_cm
    else:
        time.sleep(0.01)
    # 进入下一次定时器
    threading.Timer(0.5, grab_blocks).start()

timer = threading.Timer(0.5, grab_blocks)
timer.start()

# 数值映射
# 将一个数从一个范围映射到另一个范围
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# 机械臂位置校准
def Arm_Pos_Corr():
    LeArm.setServo(1, 1200, 500)
    time.sleep(0.5)
    kin.ki_move(0, 2250, 200.0, 1500)

if debug:
    Running = True
else:
    Running = False 

# 运行程序前按下KEY2,进入校准机械臂位置， 校准完成后，再按下KEY退出
run_corr_one = 0

# 初始化机械臂位置
LeArm.runActionGroup('rest', 1)
while True:
    if GPIO.input(key) == 0:
        time.sleep(0.1)
        if GPIO.input(key) == 0:
            correction_flag = not correction_flag
            if correction_flag is False:
                LeArm.runActionGroup('rest', 1)
    if correction_flag is False:
        run_corr_one = 0
        if Running:
          if cap.isOpened():
              ret, orgFrame = cap.read()
              if ret:
                  t1 = cv2.getTickCount()
                  try:             
                      orgFrame = cv2.resize(orgFrame, (320,240), interpolation = cv2.INTER_CUBIC) #将图片缩放到 320*240            
                  except Exception as e:
                      print(e)
                      continue
                  if orgFrame is not None:
                    img_h, img_w = orgFrame.shape[:2]
                    # 获取图像中心点坐标x, y
                    img_center_x = img_w / 2
                    img_center_y = img_h / 2
                    if cv_blocks_ok is False:
                        # 高斯模糊
                        gs_frame = cv2.GaussianBlur(orgFrame, (5, 5), 0)
                        # 转换颜色空间
                        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
                        # 查找字典颜色
                        mask = cv2.inRange(hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
                        # 腐蚀
                        mask = cv2.erode(mask, None, iterations=2)
                        # 膨胀
                        kernel = np.ones((5, 5), np.uint8)
                        mask = cv2.dilate(mask, kernel, iterations=2)
                        # 查找轮廓
                        # cv2.imshow('mask', mask)
                        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                        if len(cnts) > 0:
                            # 找出最大的区域
                            c = max(cnts, key=cv2.contourArea)
                            # 返回的值 中心坐标（x, y）,（w，h）,角度
                            rect = cv2.minAreaRect(c)
                            # 获取最小外接矩形的4个顶点
                            box = cv2.boxPoints(rect)
                            # 数据类型转换
                            # 绘制轮廓
                            cv2.drawContours(orgFrame, [np.int0(box)], -1, (0, 255, 255), 2)
                            # 找色块中心点
                            c_x, c_y = rect[0]
                            h, w = rect[1]
                            c_angle = rect[2]
                            if h * w >= 1350:   # 色块面积限制
                                # 绘制中心点
                                cv2.circle(orgFrame, (int(c_x), int(c_y)), 3, (216, 0, 255), -1)
                                x_pix_cm = int(c_x)
                                y_pix_cm = int(c_y)
                                cv_blocks_ok = True  # 开始搬运
                    if cv_blocks_ok and sec_3_flag is False:
                        # 数据映射
                        n_x = int(leMap(x_pix_cm, 0.0, 320.0, -1250.0, 1250.0)) * 1.0
                        n_y = int(leMap(240 - y_pix_cm, 0.0, 240.0, 1250, 3250.0)) * 1.0
                        # 需要根据实际情况调整，偏差主要来自舵机的虚位
                        if n_x < -100:
                            n_x -= 120  # 偏差
                        get_d = True
                        cv_blocks_ok = False
                        if -20 < last_x - n_x < 20:     # 判断色块是否已经不动了
                            no_move_count += 1
                            if no_move_count >= 4:
                                no_move_count = 0
                                sec_3_flag = True   # 是就开启定时器
                                position_color_list.append((x_pix_cm, y_pix_cm, int(c_angle)))
                        else:
                            no_move_count = 0
                        last_x = n_x
                    # 画图像中心点
                    cv2.line(orgFrame, (int(img_w / 2) - 20, int(img_h / 2)), (int(img_w / 2) + 20, int(img_h / 2)), (0, 0, 255), 1)
                    cv2.line(orgFrame, (int(img_w / 2), int(img_h / 2) - 20), (int(img_w / 2), int(img_h / 2) + 20), (0, 0, 255), 1)
                    t2 = cv2.getTickCount()
                    time_r = (t2 - t1) / cv2.getTickFrequency()               
                    fps = 1.0/time_r
                    if debug:
                        cv2.putText(orgFrame, "fps:" + str(int(fps)),
                                (10, orgFrame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)#(0, 0, 255)BGR
                        cv2.imshow("orgFrame", orgFrame)
                        cv2.waitKey(1)
                  else:
                    time.sleep(0.01)
    else:
        if correction_flag and run_corr_one == 0:
            run_corr_one += 1
            Arm_Pos_Corr()
        else:
            time.sleep(0.01)
