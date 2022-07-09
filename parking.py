#! /usr/bin/env python
# -*- coding: utf-8 -*-
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy, math
import cv2, time, rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)
#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
arData = {"DX":0.0, "DY":1, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()
#=============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
#=============================================
def callback(msg):
    global arData
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z
        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
#=========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
#=========================================
rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )
#=========================================
# speed, angle 값을 보내는 부분이 중복되어 함수로 변경
def drive(motor_msg, motor_pub, angle, speed):
	motor_msg.angle = angle
	motor_msg.speed = speed
	motor_pub.publish(motor_msg)
#=========================================
# 메인 루프
# 끊임없이 루프를 돌면서
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행"
# 작업을 반복적으로 수행함.
#=========================================
while not rospy.is_shutdown():
    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    # Row 100, Column 500 크기의 배열(이미지) 준비
    img = np.zeros((100, 500, 3))
    # 4개의 직선 그리기
    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)
    # DX 값을 그림에 표시하기 위한 좌표값 계산
    point = int(arData["DX"]) + 250
    if point > 475:
        point = 475
    elif point < 25 :
        point = 25	
    # DX값에 해당하는 위치에 동그라미 그리기
    img = cv2.circle(img,(point,65),15,(0,255,0),-1)
    # DX값과 DY값을 이용해서 거리값 distance 구하기
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    # 그림 위에 distance 관련된 정보를 그려넣기
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
    # DX값 DY값 Yaw값 구하기
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1))
    # 그림 위에 DX값 DY값 Yaw값 관련된 정보를 그려넣기
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
    # 만들어진 그림(이미지)을 모니터에 디스플레이 하기
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)


    # 차량의 앞 부분과 AR태그 사이의 각도를 degree로 계산
    angle = math.degrees(np.arctan(arData["DX"] / arData["DY"])) * 2
    
    # Yaw 값의 차이가 클수록, 더 많은 회전을 해야하기에 먼거리는 빠르게 진입하고 가까우면 속도를 줄여서 진입
    # yaw에 대한 오차가 속도를 낮추면서 진입하는 경우가 더 적었음  
    if(arData["DY"] > 150): #DY값이 150 이상이면 속도 30
        speed = 30
    elif(arData["DY"] > 100): #DY값이 100~150 사이면 속도 20
        speed = 20
    elif(arData["DY"] > 70): #DY값이 70~100 사이면 속도 10
        speed = 10
	# theta를 줄이는 방향으로 진입했기 때문에 틀어진 각도가 일정 범위에 들어오면
	# 가까운 거리에서는 정지선 내부에 들어오거나 해당 구역 근처라고 생각
        if (round(yaw, 1) > 5.0 or round(yaw, 1) < -5.0): # yaw를 줄이기 위해 후진
	    # 반대 방향으로 각도를 줄이면서 회전해야 해서 원래 방향으로 돌아가는데 사용하는 시간의 2배 사용
	    speed = -25
            t_reverse = time.time() + 2.0
	    # 차량이 주차구역과 AR태그를 직교하는 선분과 차이를 줄이기 위해 
	    # 차량이 바라보는 방향과 반대 방향으로 후진을 먼저 수행
            while time.time() < t_reverse: 
                angle = round(yaw, 1) * -4.0
		drive(motor_msg, motor_pub, angle, speed) # 함수로 차량 이동부분 정의
	    # 반대 방향으로 후진하며 직교하는 성분과 틀어진 각도를 다시 줄이기
	    # 원래 방향으로 돌아가는데 사용하는 시간을 1로 설정
            t_reverse = time.time() + 1.0
            while time.time() < t_reverse:
                angle = round(yaw, 1) * 4.0
		drive(motor_msg, motor_pub, angle, speed) # 함수로 차량 이동부분 정의

    else: #DY값이 70 이하면 정지 (앞 부분에서 dx, yaw를 수정하고 진입했기 때문에 정지선 내에 존재함)
        speed = 0
    #print(angle)
#===================================================================================================
# 실험 결과 기록
# time, speed, angle을 다양하게 실험한 결과 처음 진입할 때는 거리에 따라 속도를 낮추는 편이 yaw가 작은 값을 기록함.
# 또한, 주차구역 근처에서 후진하면서 각도를 조율할 때는 속도 25에 angle에 4를 곱한게 실험에서는 최적을 기록
# 3,4번에서 시작한 경우는 yaw값이 커서 속도와 앵글이 큰 경우에 yaw를 줄였고
# 1,2번은 yaw 값이 10을 넘지 않아서 속도를 줄이고 앵글 각도를 크게 실험했을 때, abs(yaw)가 1 이하를 기록
#===================================================================================================
    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    drive(motor_msg, motor_pub, angle, speed) # 함수로 차량 이동부분 정의

# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()