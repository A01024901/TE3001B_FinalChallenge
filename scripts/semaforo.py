#!/usr/bin/env python
import cv2 
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class filter() :
    def __init__(self) :
        rospy.on_shutdown(self.cleanup)

        ############    PUBLISHER   ####################### 
        self.image_pub = rospy.Publisher("segmented_image", Image)

        ############    SUBSCRIBERS   ####################### 
        self.camera_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_cb)
        
        self.bridge = CvBridge()
        self.image_received = 0
        r = rospy.Rate(50)

        segmented_image = Image()

        #Verde 
        self.V_H_min , self.V_H_max = 0
        self.V_S_min , self.V_S_max = 151
        self.V_V_min , self.V_V_max = 225

        #Amarillo 
        self.A_H_min , self.A_H_max = 0
        self.A_S_min , self.A_S_max = 151
        self.A_V_min , self.A_V_max = 225

        #Rojo
        self.R_H_min , self.R_H_max = 0
        self.R_S_min , self.R_S_max = 151
        self.R_V_min , self.R_V_max = 225

        self.min_area = 4000


        while not rospy.is_shutdown():
            if self.image_received:
                image = cv2.resize(self.cv_image, (500, 500))
                cv2.imshow("Camera", image)

                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                #Verde
                V_hsv_min = np.array([self.H_min, self.S_min, self.V_min])
                V_hsv_max = np.array([self.H_max, self.S_max, self.V_max])
                V_hsv_mask = cv2.inRange(hsv, V_hsv_min, V_hsv_max)
                V_hsv_mask = cv2.erode(V_hsv_mask , None , iterations = 3)
                V_hsv_mask = cv2.dilate(V_hsv_mask , None , iterations = 3)


                #Amarillo 
                A_hsv_min = np.array([self.H_min, self.S_min, self.V_min])
                A_hsv_max = np.array([self.H_max, self.S_max, self.V_max])
                A_hsv_mask = cv2.inRange(hsv, A_hsv_min, A_hsv_max)
                A_hsv_mask = cv2.erode(A_hsv_mask , None , iterations = 3)
                A_hsv_mask = cv2.dilate(A_hsv_mask , None , iterations = 3)

                #Rojo
                R_hsv_min = np.array([self.H_min, self.S_min, self.V_min])
                R_hsv_max = np.array([self.H_max, self.S_max, self.V_max])
                R_hsv_mask = cv2.inRange(hsv, R_hsv_min, R_hsv_max)
                R_hsv_mask = cv2.erode(R_hsv_mask , None , iterations = 3)
                R_hsv_mask = cv2.dilate(R_hsv_mask , None , iterations = 3)

                

                imgBlur = cv2.GaussianBlur(res , (7 , 7) , 1)
                imgGray = cv2.cvtColor(imgBlur , cv2.COLOR_BGR2GRAY)
                imCanny = cv2.Canny(imgGray , 166 , 171)

                cv2.imshow("Filter", imCanny)

                #self.getContours(imgGray , image)

                segmented_image = self.bridge.cv2_to_imgmsg(res, encoding = "passthrough")
                self.image_pub.publish(segmented_image)

            cv2.waitKey(1)
            r.sleep()
        cv2.destroyAllWindows()

    def camera_cb(self, image_data) :
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding = "bgr8")
        except CvBridgeError as e :
            print(e)
        self.image_received = 1

    def getContours(self , img ,img_tracking):
        contours , hierarchy , _= cv2.findContours (img , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)[0]
            print(area)
            if area > self.min_area:
                per = cv2.arcLength(cnt , True)
                aprox = cv2.approxPolyDP(cnt , 0.02* per , True)
                x , y , w ,h = cv2.boundingRect(aprox)
                cx = int(x + w/2)
                cy = int(y + h/2)

                #mostar informacion 
                cv2.drawContours(img_tracking , cnt , -1 , (255 , 0 , 255))
                cv2.putText(img_tracking, 'cx', (20, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                cv2.putText(img_tracking, str(cx), (80, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                cv2.putText(img_tracking, 'cy', (20, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                cv2.putText(img_tracking, str(cy), (80, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                
                #Control de movimiento
            else: 
                pass
                #Trazar una linea media
        cv2.line(img_tracking,(250,0),(250,500),(255,255,0),3)
        cv2.line(img_tracking,(0,250),(500,250),(255,255,0),3)

    def H_min_cb(self, H_min) :
        self.H_min = cv2.getTrackbarPos('H_min', 'HSV_TrackBars')

    def H_max_cb(self, H_max) :
        self.H_max = cv2.getTrackbarPos('H_max', 'HSV_TrackBars')
    
    def S_min_cb(self, S_min) :
        self.S_min = cv2.getTrackbarPos('S_min', 'HSV_TrackBars')

    def S_max_cb(self, S_max) :
        self.S_max = cv2.getTrackbarPos('S_max', 'HSV_TrackBars')

    def V_min_cb(self, V_min) :
        self.V_min = cv2.getTrackbarPos('V_min', 'HSV_TrackBars')

    def V_max_cb(self, V_max) :
        self.V_max = cv2.getTrackbarPos('V_max', 'HSV_TrackBars')

    def cleanup(self):     
        zero_0 = Twist()
        print("I'm dying, bye bye!!!")
        #self.cmd_vel_pub.publish(zero_0)

if __name__ == "__main__" :
    rospy.init_node("color_filter", anonymous = True)
    filter()