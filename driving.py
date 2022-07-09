#!/usr/bin/env python  
# -*- coding: utf-8 -*- # 기본 인코딩이 utf-8 이라고 알려주는 것과 동일하며 코드에 한글이 포함되어 있어도 문제없이 잘 실행됨 
import rospy, rospkg, time #　ros node와 ros 패키지에 대한 정보를 쿼리하기 위한 유틸리티, time 모듈를 사용하기 위함
import numpy as np # numpy 라이브러리를 사용하기 위함, numpy라이브러리을 np로 지정함
import cv2, math # openCV모듈와 math모듈을 사용하기 위함
from sensor_msgs.msg import Image # sensor_msgs.msg모듈에서 Image를 가져옴
from cv_bridge import CvBridge # cv_bridge패키지에서 CvBridge를 가져옴
from xycar_msgs.msg import xycar_motor # xycar_msgs.msg모듈에서 xycar_motor를 가져옴
from math import * # math모듈에서 모두 가져옴
import signal # 신호 처리기를 사용하는 메커니즘을 제공하는 signal모듈을 사용하기 위함 
import sys # 변수와 함수를 직접 제어할 수 있게 해주는 sys모듈을 사용하기 위함
import os # 환경 변수나 디렉터리, 파일 등의 OS 자원을 제어할 수 있게 해주는 os모듈을 사용하기 위함


def signal_handler(sig, frame):    # sig는 발생한 신호의 숫자 값이고 frame은 프로그램을 실행한 스택 프레임이고 
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0) # 프로그램을 정상적으로 종료시킴
signal.signal(signal.SIGINT, signal_handler) # SIGINT신호가 발생할 때 기본 동작을 무시하고 handler함수를 실행하도록 함

image = np.empty(shape=[0]) # 임의의 값으로 초기화된 배열을 생성
bridge = CvBridge() # OpenCV 이미지와 ROS 이미지 메시지 간에 변환하는 개체
motor = None


# 카메라 이미지 토픽 callback
def img_callback(data): # 이미지를 콜백하는 함수를 만듬 
    global image # 전역변수인 image를 만들어 줌
    image = bridge.imgmsg_to_cv2(data, "bgr8") # ROS이미지를 OpenCV로 변환 
    # cv2.imshow('img', image)


# publish xycar_motor msg
######################################################################################
def hough(image): # 차선을 검출하는 함수를 만듬
    import cv2 
    import numpy as np
    img = cv2.imread('line_pic.png', cv2.IMREAD_COLOR) #이미지 파일을 Color로 읽어들임
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # color이미지를 그레이스케일 이미지로 변환
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0) # noise제거를 위해 변환한 이미지를 가우시안 필터링함 
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 70) # 입력 이미지에서 가장자리를 검출함

    # ROI Area
    roi_edge_img = edge_img[240:480, 0:640] # 이미지에서 필요한 일부분만 자름
    # cv2.imshow("roi_edge_img", roi_edge_img)
    # 확률적용 허프변환하여 직선검출
    all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi / 180, 30, 30, 10) # 확률적 허프선 변환을 적용하여 직선을 검출함 
    print("Number of lines : %d" % len(all_lines)) # 확률적 허프선 변환을 통해 검출된 직선의 길이를 나타냄
    # draw lines in ROI area
    line_img = image.copy() # 원본이미지는 건들지 않기 위해 이미지를 복사함
    for line in all_lines: # 검출된 모든 선 순회
        x1, y1, x2, y2 = line[0] # 선의 시작점과 끝점
        cv2.line(line_img, (x1, y1 + 240), (x2, y2 + 240), (0, 255, 0), 2) # 시작점에서 끝점까지 초록색으로 직선을 그림
    # cv2.imshow("roi_lines", line_img)
    # cv2.waitKey(1)
    # 기울기를 계산하고 필터링을 함
    slopes = []
    new_lines = [] 
    for line in all_lines: # 검출된 모든 선 순회 
        x1, y1, x2, y2 = line[0] # 선의 시작점과 끝점
        if (x2 - x1) == 0: # 만약 x축의 변화가 없다면
            slope = 0 # 기울기는 0이다
        else: # 만약 x축의 변화가 있다면
            slope = float(y2 - y1) / float(x2 - x1) # 기울기는 (y2 - y1)/(x2 - x1)이다
        if 0.1 < abs(slope) < 10: # 만약 기울기의 절대값이 0.1이상 10이하면
            slopes.append(slope) # 기울기를 slopes에 추가함
            new_lines.append(line[0]) # line[0]을 new_lines에 추가함
    print("Number of lines after slope filtering : %d" % len(new_lines)) # 기울기의 절대값이 0.1이상 10이하였던 선의 개수를 알려줌

    left_lines = [] # 좌측 차선
    right_lines = [] # 우측 차선 
    for j in range(len(slopes)): # 필터링된 차선에서 
        Line = new_lines[j]
        slope = slopes[j]
        x1, y1, x2, y2 = Line # 선의 시작점과 끝점
        if (slope < 0) and (x2 < 320): # 만약 기울기가 0미만이고 x좌표가 320미만이면
            left_lines.append([Line.tolist()]) # 좌측 차선으로 처리함
        elif (slope > 0) and (x1 > 320): # 만약 기울기가 0초과이고 x좌표가 320초과이면
            right_lines.append([Line.tolist()]) # 우측 차선으로 처리함
    print("Number of left lines : %d" % len(left_lines)) # 좌측 차선의 개수를 알려줌
    print("Number of right lines : %d" % len(right_lines)) # 우측 차선의 개수를 알려줌

    # 좌측 차선과 우측 차선을 다른 색으로 그림 
    line_img = image.copy() # 원본이미지는 건들지 않기 위해 이미지를 복사함
    for line in left_lines: # 좌측 차선에서 
        x1, y1, x2, y2 = line[0] # 선의 시작점과 끝점
        cv2.line(line_img, (x1, y1 + 240), (x2, y2 + 240), (0, 0, 255), 2) # 좌측 차선을 빨간색으로 표현함
    for line in right_lines: # 우측 차선에서
        x1, y1, x2, y2 = line[0] # 선의 시작점과 끝점
        cv2.line(line_img, (x1, y1 + 240), (x2, y2 + 240), (0, 255, 255), 2) # 우측 차선을 노란색으로 표현함
    # cv2.imshow("left & right lines", line_img) 
    # cv2.waitKey(1)

    # 평균 좌측 차선을 구함
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0 # 평균 좌측 차선을 구하기 위한 변수
    size = len(left_lines) # 좌측 차선들의 개수
    for line in left_lines: # 좌측 차선에서
        x1, y1, x2, y2 = line[0] # 선의 시작점과 끝점
        x_sum += x1 + x2 # 선의 x의 좌표들을 더해줌
        y_sum += y1 + y2 # 선의 y의 좌표들을 더해줌
        m_sum += float(y2 - y1) / float(x2 - x1) # 선의 기울기들을 더해줌
        x_avg = x_sum / (size * 2) # 좌측 차선의 평균 x좌표
        y_avg = y_sum / (size * 2) # 좌측 차선의 평균 y좌표
        m_left = m_sum / size # 좌측 차선의 평균 기울기
        b_left = y_avg - m_left * x_avg # 좌측 차선의 y좌표의 오차
    if size != 0: # 좌측 차선을 검출했다면 
        x1 = int((0.0 - b_left) / m_left) # y좌표의 오차를 이용하여 x좌표를 구함
        x2 = int((240.0 - b_left) / m_left) # y좌표의 오차를 이용하여 x좌표를 구함
        x_l = (x1 + x2) // 2 # 이렇게 구해진 x좌표의 평균을 평균 좌측 차선의 x좌표로 함
    else: # 좌측 차선을 검출하지 못 했다면 
        x_l = 0 # 좌측 차선의 x좌표를 0이라 한다 
    cv2.line(line_img, (x1, 0 + 240), (x2, 240 + 240), (255, 0, 0), 2) # 평균 좌측 차선을 파란색으로 표현함

    # 평균 우측 차선을 구함
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0 # 평균 우측 차선을 구하기 위한 변수
    size = len(right_lines) # 우측 차선들의 개수
    for line in right_lines: # 우측 차선에서
        x1, y1, x2, y2 = line[0] # 선의 시작점과 끝점
        x_sum += x1 + x2 # 선의 x의 좌표들을 더해줌
        y_sum += y1 + y2 # 선의 y의 좌표들을 더해줌
        m_sum += float(y2 - y1) / float(x2 - x1) # 선의 기울기들을 더해줌
        x_avg = x_sum / (size * 2) # 우측 차선의 평균 x좌표
        y_avg = y_sum / (size * 2) # 우측 차선의 평균 y좌표
        m_right = m_sum / size # 우측 차선의 평균 기울기
        b_right = y_avg - m_right * x_avg # 우측 차선의 y좌표의 오차
    if size != 0: # 우측 차선을 검출했다면
        x1 = int((0.0 - b_right) / m_right) # y좌표의 오차를 이용하여 x좌표를 구함
        x2 = int((240.0 - b_right) / m_right) # y좌표의 오차를 이용하여 x좌표를 구함
        x_r = (x1 + x2) // 2 # 이렇게 구해진 x좌표의 평균을 평균 우측 차선의 x좌표로 함
    else: # 우측 차선을 검출하지 못 했다면
        x_r = 640 # 우측 차선의 x좌표를 640이라 한다
    cv2.line(line_img, (x1, 0 + 240), (x2, 240 + 240), (255, 0, 0), 2) # 평균 우측 차선을 파란색으로 표현함
    
    x_center = (x_l + x_r) // 2 # 검출된 좌측 차선의 x좌표와 검출된 우측 차선의 x좌표을 이용하여 차선의 중앙을 구함

    cv2.imshow("detected lines", line_img) # 검출된 선을 출력함 
    cv2.waitKey(1) # 입력을 대기함
    return x_center - 320 # 차선의 중앙 


Width, Height = 640, 480 # 이미지 크기 지정


#########################################################################################
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
def drive(Angle, Speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed
    motor.publish(motor_msg)


# ============================================================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
# ============================================================================
def start():
    # 위에서 선언한 변수를 start() 안에서 사용하고자 함	
    global motor
    global image
    global Width, Height
    # =========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    # =========================================
    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    # 임시로 이전 steer_angle값을 저장할 변수생성    
    nn = 0.0
    print
    "---------- Xycar ----------"
    # =========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    # =========================================
    while not rospy.is_shutdown():
        # 첫번째 카메라 토픽이 도착할 때까지 기다림.
        while not image.size == (Width * Height * 3):
            continue
        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장        
        img = image.copy()
        angle = hough(img)

        # 산출된 angle값을 핸들링 값으로 변환        
        steer_angle = angle / 6
        # 급격히 핸들링 값이 변환될 때 속도를 감속하고 급 커브를 방지하기 위해 핸들 각도 조절
        if abs(nn - steer_angle) > 30:
            speed = 2
            steer_angle = steer_angle * 0.3
        else:
            # =========================================
            # 차량의 속도 값인 speed값 정하기.
            # 직선 코스에서는 빠른 속도로 주행하고 
            # 회전구간에서는 느린 속도로 주행하도록 설정함.
            # =========================================
            if abs(steer_angle) < 5.0:
                speed = 25
                steer_angle = steer_angle * 0.5
            else:
                # 핸들링 값이 한계치를 넘어서면 일정한 값을 주도록 조정
                speed = 10.5
                if abs(steer_angle) > 49.0:
                    if steer_angle < 0:
                        steer_angle = -49
                    else:
                        steer_angle = 49

        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(steer_angle, speed)
        # 급커브를 방지하기 위해 steer_angle값 저장
        nn = steer_angle


# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
# =============================================
if __name__ == '__main__':
    start()
# ===================================================================================================
# 실험 결과 기록
# 관심영역(roi)를 적절하게 설정하여 주지 않으면 원하는 차선 이외의 차선을 인식하는 경우가 있음
# 차량의 주행속도를 높이기 위해 속도를 높이니 급커브에 취약하여 급커브 구간에서 차량을 일시적으로 감속
# 직진 구간에서도 차량의 핸들링 값이 계속해서 변하여 차량이 흔들림=> 일정 각도 이내 핸들링 값을 작게 만들어 줌
# 여러 개의 차선이 한꺼번에 감지되는 경우 차선 감지가 어려움=> 차량의 차체 각도에 따라 다르게 인식하므로 적절한 주행값을 넣어줌
# ===================================================================================================

