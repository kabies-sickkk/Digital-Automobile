###!/usr/bin/python3 %code này để tham khảo về việc xe tự hành nhận diện làn đường
from this import d
import cv2 as cv
import rospy
import numpy as np
from time import sleep
from time import time
import os

from automobile_data import Automobile_Data     # I/O car manager
#from automobile_data_simulator import AutomobileDataSimulator
from controller3 import Controller              # Lane Keeping
from maneuvers import Maneuvers                 # Maneuvers
from helper_functions import *                  # helper functions
from detection import Detection                 # detection


# PARAMETERS
SAMPLE_TIME = 0.01      # [s]
WAIT_TIME_STOP = 2.0    # [s]
DESIRED_SPEED = 35.0  # [m/s]
OBSTACLE_DISTANCE = 0.20 # [m]

SLOWDOWN_DIST_VALUE = 0.2     # [m]
STOP_DIST_VALUE = 0.6         # [m]

#Xuất hình ảnh
#SHOW_IMGS = True
SHOW_IMGS = False

# Pure pursuit controller parameters
k1 = 0.0 #4.0 gain error parallel to direction (speed)
k2 = 0.0 #2.0 perpedndicular error gain
k3 = 0.99 #1.5 yaw error gain 
#dt_ahea  = 0.5 # [s] how far into the future the curvature is estimated, feedforwarded to yaw controller
ff_curvature = 0.0 # feedforward gain

if __name__ == '__main__':
    l = 0

    #load camera with opencv
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_FPS, 30)

    # init the car flow of data
    car = Automobile_Data(trig_control=True, trig_cam=False, trig_gps=False, trig_bno=True, trig_enc=True, trig_sonar=True)
    maneuver = Maneuvers()
    

    # initialize detector
    detect = Detection()

    if SHOW_IMGS:
        cv.namedWindow('Frame preview', cv.WINDOW_NORMAL)
        cv.resizeWindow('Frame preview', 320, 240)
        #cv.namedWindow('lane_detection', cv.WINDOW_NORMAL)
        #cv.resizeWindow('lane_detection', 200, 200)

    # init controller
    controller = Controller(k1=k1, k2=k2, k3=k3, ff=ff_curvature, training=False)

    start_time_stamp = 0.0      # [s]
    angle_ref = 0.0             # [deg]

    # RESET THE CAR POSITION
    car.stop()
    print("CHUONG TRINH DIEU KHIEN XE TU LAI SELF - DRIVING CAR")
    
    #car.drive(speed=30.0, angle=np.rad2deg(0))
    
    sleep(1)
    
    
    car.drive_speed(speed= DESIRED_SPEED)
    
    try:

        while not rospy.is_shutdown():
            start_time_stamp = time() * 1000.0# [ms] loop start time
            
            #Get the image from the camera
            #frame = car.cv_image.copy()
            #frame = cv.resize(frame,(640,480))
            ret, frame = cap.read()
            if not ret:
                print("Kiem tra lai camera")
                frame = np.zeros((480, 640, 3), np.uint8)
                continue

            # -------------------- LANE KEEPING --------------------
            #Neural network control
            lane_info = detect.detect_lane(frame, show_ROI=SHOW_IMGS)
            e2, e3, point_ahead = lane_info
            curv = 0.0001
            speed_ref, angle_ref = controller.get_control(e2, e3, curv, DESIRED_SPEED)
            
            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
            #print("vongg lap moi")
            current_angle = np.rad2deg(angle_ref)
            
            # #stopping logic
            if 0.0 < dist < 0.3:
                print('Chay cham')
                #speed_ref = DESIRED_SPEED * 0.5
                print(l)
                if 0.0 < dist < 0.2:
                    l +=1
                    
                    if l == 1:
                        print('Re phai lan 1') 
                        car.stop()
                        sleep(3)
                        car.drive(speed=35.0, angle=np.rad2deg(0))
                        sleep(1.6)
                        car.drive(speed=35.0, angle=np.rad2deg(-25))
                        sleep(1.9)
                        car.drive(speed=35.0, angle=np.rad2deg(0))
                        sleep(0.3)
                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                    
                    elif l == 2:
                        print('Co vat can') 
                        #car.drive(speed=35.0, angle=np.rad2deg(0))
                        #sleep(1)
                        #car.stop()
                        #sleep(2)
                        car.drive(speed=35.0, angle=np.rad2deg(-25))
                        sleep(1)
                        car.drive(speed=35.0, angle=np.rad2deg(25))
                        sleep(1.7)
                        #car.drive(speed=35.0, angle=np.rad2deg(0))
                        #sleep(1.2)
                        #car.drive(speed=35.0, angle=np.rad2deg(25))
                        #sleep(1.4)
                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1

                    elif l == 3:
                        print('Re phai lan 2') 
                        car.drive(speed=35.0, angle=np.rad2deg(0))
                        sleep(1.6)
                        car.drive(speed=35.0, angle=np.rad2deg(-25))
                        sleep(1.9)
                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                    
                    elif l == 4:
                        print('Tim bai do xe') 
                        car.drive(speed=30.0, angle=np.rad2deg(0))
                        sleep(2)
                        car.drive(speed=30.0, angle=np.rad2deg(-25))
                        sleep(1)
                        car.drive(speed=-30.0, angle=np.rad2deg(25))
                        sleep(1.7)
                        car.stop()
                        sleep(10)
                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                        
                    #elif l == 5:
                        
                        #while dist < 0.2:
                            #ret, frame = cap.read()
                            #dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        #continue
                        #l +=1
                    
                    elif l == 6:

                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                    
                    elif l == 7:

                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                    
                    elif l == 8:

                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                    
                    elif l == 9:

                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                    
                    elif l == 10:

                        while dist < 0.2:
                            ret, frame = cap.read()
                            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
                        continue
                        l +=1
                if car.obstacle_ahead_median < 0.47:
                    print("Co vat can phia truoc")
                    #print("gia tri bang 5")
                    print(car.obstacle_ahead_median)
                    car.drive(speed=20.0, angle=np.rad2deg(25))
                    sleep(1.2)
                    car.drive(speed=27.0, angle=np.rad2deg(-25))
                    sleep(1.5)
                    
                    print("Da nhan duoc vat the 0.5 m")
                else:
                    print("khong nhan duoc va thoat vat the")
                    continue
                    pass    

            # -------------------- SIGNS ---------------------------
            #sign = detect.detect_sign(frame, show_ROI=SHOW_IMGS)
                   
            #if car.obstacle_ahead_median < 0.4:
                #print('stopping pedestrian for obstacle...')
                #print(f'sonar distance: {car.obstacle_ahead_median}')
                #car.drive(speed=30.0, angle=np.rad2deg(-25))
                #sleep(0.5)
                #car.drive(speed=30.0, angle=np.rad2deg(-25))
            
            #if l == 1 and car.obstacle_ahead_median < 0.45:
                #car.drive(speed=30.0, angle=np.rad2deg(-25))
                #sleep(1.2)
                #l += 1

            #if l == 1 and car.obstacle_ahead_median < 0.45:
                #car.drive(speed=30.0, angle=np.rad2deg(-25))
                #sleep(0.6)
                #car.drive(speed=30.0, angle=np.rad2deg(25))
                #sleep(1.4)
                #l += 1
            #if l == 3 and car.obstacle_ahead_median < 0.45:
                #car.drive(speed=30.0, angle=np.rad2deg(-25))
                #sleep(1.2)
                #l += 1
            #if 0.0 < car.obstacle_ahead_median <0.39:
             #   speed_ref=0.0
              #  angle_ref=0.0
            
            
                    
                
                #if round(rpm) == 3:
                    #car.drive(speed=0.0, angle=np.rad2deg(0))
                #else:
                    #car.drive(speed=speed_ref, angle=np.rad2deg(angle_ref))

            # -------------------- INTERSECTION --------------------

            # -------------------- ACTUATION --------------------
            #if stop_detected:
                #if time() - stop_time >= 3: # wait for 3 seconds
                   #stop_detected = False  # reset the flag
                   #speed_ref = DESIRED_SPEED  # resume driving with desired speed
            car.drive(speed=speed_ref, angle=np.rad2deg(angle_ref))
            
            

            # -------------------- LOOP SAMPLING TIME --------------------
            #r.sleep()
            # sleep(0.1)

            # -------------------- DEBUG --------------------
            #os.system('cls' if os.name=='nt' else 'clear')
            #print(f'Net out:\n {lane_info}')

            #project point ahead
            #print(f'angle_ref: {np.rad2deg(angle_ref)}')
            #print(f"point_ahead: {point_ahead}")
            
            #print(f'Loop Time: {time() * 1000.0 - start_time_stamp} ms')
            print(f'FPS: {20/(time() * 1000.0 - start_time_stamp)*1000.0}')
            print("so line:", l)
            #print(f'current speed: {speed_ref}')
            #print(f'yaw: {car.yaw}')
            print(f'obstacle ahead is {car.obstacle_ahead_median} m')
            #print(f'Lane detection time = {detect.avg_lane_detection_time:.2f} ms')
            #print(f'Sign detection time = {detect.avg_sign_detection_time:.2f} ms')
            #print(f"rpm:{round(rpm)}RPM Pulse:{pulse} Balls:{balls_dropped}")
            #print("Value is {}".format(e1.getValue()))

            if SHOW_IMGS:
                #project point ahead
                frame, proj = project_onto_frame(frame, car, point_ahead, False, color=(200, 200, 100))
                if proj is not None:
                    #convert proj to cv2 point
                    proj = (int(proj[0]), int(proj[1]))
                    #draw line from bottmo half to proj
                    cv.line(frame, (320,479), proj, (200, 200, 100), 2)

                cv.imshow('Frame preview', frame)
                key = cv.waitKey(1)
                if key == 27:
                    car.stop()
                    sleep(1)
                    cv.destroyAllWindows()
                    print("Shutting down...")
                    exit(0)

    except rospy.ROSInterruptException:
        print('inside interrupt exeption')
        pass
      




