#!/usr/bin/env python 

#import queue
import rospy 

import numpy as np

from sensor_msgs.msg import Image 

from cv_bridge import CvBridge, CvBridgeError 
from geometry_msgs.msg import Twist


import cv2 


class ShowingImage(object): 

 

    def __init__(self): 

        
        rospy.on_shutdown(self.cleanup)
        self.image_sub = rospy.Subscriber('camera/image_raw',Image,self.camera_callback)


        self.bridge_object = CvBridge()
        self.image_received = 0 #Flag to indicate that we have already received an image 
        bandera_imagen1 = 0
        bandera = 0
        bandera1 = False
        bandera2 = 0

        r = rospy.Rate(10) #10Hz  

        while not rospy.is_shutdown(): 
            if self.image_received == 1: 
                self.cv_image = cv2.resize(self.cv_image,(300,300))
                cv2.imshow('imagen sin filtro',self.cv_image)
                bandera=0
                if (cv2.waitKey(5) == 114) and bandera == 0: #r
                    bandera_imagen = bandera_imagen1 + 1
                    nombre_imagen = 'robot_image' + str(bandera_imagen) + '.jpg'
                    cv2.imwrite(nombre_imagen, self.cv_image)
                    print("tome una foto")
                    bandera_imagen1 = bandera_imagen1 + 1
                    print(nombre_imagen)
                    print(bandera_imagen)
                    bandera = 1

                if cv2.waitKey(5) == 108:#l 
                    print("TeclaT")
                    bandera1 = True 
                    bandera2 = 0

                while bandera1 and bandera2 <= 10:
                    bandera_imagen = bandera_imagen1 + 1
                    nombre_imagen = 'robot_image' + str(bandera_imagen) + '.jpg'
                    cv2.imwrite(nombre_imagen, self.cv_image)
                    print("tome una foto")
                    bandera_imagen1 = bandera_imagen1 + 1
                    print(nombre_imagen)
                    print(bandera_imagen)
                    bandera2 = bandera2 + 1
                    
                    

                cv2.waitKey(1) 

                r.sleep()  

        cv2.destroyAllWindows() 

 

    def camera_callback(self,data): 

        try: 

            # We select bgr8 because its the OpenCV encoding by default 

            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") 

        except CvBridgeError as e: 

            print(e) 

        self.image_received=1

    def cleanup(self):
        print("I'm Dying bye bye")

        #cv2.waitKey(0) 

        #cv2.destroyAllWindows() 
        


if __name__ == '__main__': 

    rospy.init_node('opencv_example1', anonymous=True) 

    ShowingImage() 

 

#The window will close after a key press 
