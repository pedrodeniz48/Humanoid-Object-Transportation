import cv2
import numpy as np
import rospy
import sensor_msgs
from cv_bridge import CvBridge
import geometry_msgs
from geometry_msgs.msg import Point
import std_msgs
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
import math

class ObjectDetection:

    def __init__(self, frame) -> None:
        self.frame = frame
        self.center = None

    def hsv_detection(self, lower_range, upper_range, erode=None, dilate=None, multiple=None):

        hsv_frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
        
        hsv = cv2.inRange(hsv_frame, lower_range, upper_range)
        self.mask = cv2.erode(hsv,np.ones((erode,erode), np.uint8))
        self.mask = cv2.dilate(self.mask, np.ones((dilate,dilate), np.uint8))

        self.contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if multiple:
            self.center = []

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(self.mask, 4, cv2.CV_32S)
            
            self.center = [(int(i[0]), int(i[1])) for i in centroids[1:].tolist()]

            self.mask = []
            
            for label in range(0,num_labels-1):
                fake_mask = np.zeros((labels.shape), dtype=np.uint8)

                fake_mask[labels==label+1] = 255
                #print(fake_mask.shape)
                self.mask.append(fake_mask)
               

    def binary_detection(self, mask, binary_frame, threshold=125, erode=None, dilate=None):

        #blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        #gray_frame = cv2.cvtColor(blurred,cv2.COLOR_BGR2GRAY)

        _, umbral = cv2.threshold(binary_frame,threshold,255,cv2.THRESH_BINARY)

        #mask = cv2.erode(umbral,np.ones((erode,erode), np.uint8))
        #mask = cv2.dilate(mask, np.ones((dilate,dilate), np.uint8))

        self.contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    def rectangle_drawing(self, min_area=None, max_area=None, orientation=False, robot=False):
        self.box = []
        
        for idx, cnt in enumerate(self.contours):
            area = cv2.contourArea(cnt)
            
            if area > min_area and area < max_area:
                contour = cv2.approxPolyDP(cnt, 0.001*cv2.arcLength(cnt, True), True)

                if orientation:

                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    cv2.drawContours(self.frame,[box],0,(255,0,255), 2)
                    cv2.circle(self.frame, self.center[idx], 3, color=(255,255,255), thickness=2)
                    self.box.append(box)

                else:
                    M = cv2.moments(cnt)
                    cX = int(M["m10"]/M["m00"])
                    cY = int(M["m01"]/M["m00"])
                    center = (cX, cY)
                    
                    x,y,w,h = cv2.boundingRect(contour)
                    
                    cv2.rectangle(self.frame, (x,y), (x+w,y+h), color=(255,255,0), thickness=3)

                    cv2.circle(frame, center, 3, color=(255,255,255), thickness=2)
                    self.center = center
        
        self.box.reverse()
    
    def potential_fields(self, frame, int_field, mid_field, ext_field, offset=None, offset2=None):

        self.mid_field_mask = np.zeros_like(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
        self.mid_boxes = []

        for obs in range(0,len(self.center)): #obstacle.center
            self.binary_detection(int_field[obs], 250, 1, 1)
            self.frame = frame
            self.rectangle_drawing(1, 1000, True)

            self.binary_detection(mid_field[obs], 250, 1, 1)
            self.frame = frame
            self.rectangle_drawing(1, 10000, True)

            self.binary_detection(mid_field[obs], 250, 1, 1)
            self.frame = self.mid_field_mask
            self.rectangle_drawing(1, 10000, True)
            self.mid_boxes.append(self.box)

            self.binary_detection(ext_field[obs], 250, 1, 1)
            self.frame = frame
            self.rectangle_drawing(1, 100000, True)
                




if __name__ == '__main__':
    
    shot = cv2.VideoCapture(2)
    
    pink_lower = (165, 125, 125)
    pink_upper = (175, 255, 255)

    green_lower = (50, 200, 0)
    green_upper = (85, 255, 255)

    red_lower = (0, 150, 150)
    red_upper = (20, 255, 255)

    blue_lower = (120, 140, 75)
    blue_upper = (140, 255, 175)

    while True:
        ret, frame = shot.read()

        obstacle = ObjectDetection(frame)
        beginning = ObjectDetection(frame)
        end = ObjectDetection(frame)
        robot = ObjectDetection(frame)

        dummy_frame = np.zeros_like(frame)

        final_frame = frame
        dummy_frame = np.zeros_like(frame)

        obstacle.hsv_detection(pink_lower, pink_upper, 3, 5, True)
        obstacle.frame = dummy_frame
        obstacle.rectangle_drawing(50, 1000, True)
        
        beginning.hsv_detection(green_lower, green_upper, 3, 5)
        beginning.rectangle_drawing(50, 500)

        end.hsv_detection(red_lower, red_upper, 1, 3)
        end.rectangle_drawing(100, 500)
        
        try:
            cv2.imshow('Video', frame)
        except:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    shot.release()
    cv2.destroyAllWindows()