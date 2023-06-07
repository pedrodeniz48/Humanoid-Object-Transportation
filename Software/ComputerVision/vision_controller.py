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

    def hsv_detection(self, lower_range, upper_range, erode=None, dilate=None):

        hsv_frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
        
        hsv = cv2.inRange(hsv_frame, lower_range, upper_range)
        self.mask = cv2.erode(hsv,np.ones((erode,erode), np.uint8))
        self.mask = cv2.dilate(self.mask, np.ones((dilate,dilate), np.uint8))

        self.contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    def rectangle_drawing(self, min_area=None, max_area=None, orientation=False):
        
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

        self.field_mask = np.zeros_like(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
        
        if self.box.any():
            box = self.box
        else:
            print("Need to run square_drawing() with orientation first!")
            return

        self.field_mask = np.zeros_like(frame)

        try:
            #Middle potential field
            cv2.line(self.frame, (box[0][0]-offset, box[0][1]+offset), (box[1][0]-offset, box[1][1]-offset), (0,255,0), 2)
            cv2.line(self.frame, (box[1][0]-offset, box[1][1]-offset), (box[2][0]+offset, box[2][1]-offset), (0,255,0), 2)
            cv2.line(self.frame, (box[2][0]+offset, box[2][1]-offset), (box[3][0]+offset, box[3][1]+offset), (0,255,0), 2)
            cv2.line(self.frame, (box[3][0]+offset, box[3][1]+offset), (box[0][0]-offset, box[0][1]+offset), (0,255,0), 2)

            #Drawing it in an empty mask
            cv2.line(self.field_mask, (box[0][0]-offset, box[0][1]+offset), (box[1][0]-offset, box[1][1]-offset), (0,255,0), 2)
            cv2.line(self.field_mask, (box[1][0]-offset, box[1][1]-offset), (box[2][0]+offset, box[2][1]-offset), (0,255,0), 2)
            cv2.line(self.field_mask, (box[2][0]+offset, box[2][1]-offset), (box[3][0]+offset, box[3][1]+offset), (0,255,0), 2)
            cv2.line(self.field_mask, (box[3][0]+offset, box[3][1]+offset), (box[0][0]-offset, box[0][1]+offset), (0,255,0), 2)

            #External potential field
            cv2.line(self.frame, (box[0][0]-offset2, box[0][1]+offset2), (box[1][0]-offset2, box[1][1]-offset2), (255, 0,255), 2)
            cv2.line(self.frame, (box[1][0]-offset2, box[1][1]-offset2), (box[2][0]+offset2, box[2][1]-offset2), (255, 0,255), 2)
            cv2.line(self.frame, (box[2][0]+offset2, box[2][1]-offset2), (box[3][0]+offset2, box[3][1]+offset2), (255, 0,255), 2)
            cv2.line(self.frame, (box[3][0]+offset2, box[3][1]+offset2), (box[0][0]-offset2, box[0][1]+offset2), (255, 0,255), 2)
        except:
            pass

                



if __name__ == '__main__':
    
    shot = cv2.VideoCapture(2)

    green_lower = (50, 200, 0)
    green_upper = (85, 255, 255)

    red_lower = (0, 150, 150)
    red_upper = (20, 255, 255)

    while True:

        ret, frame = shot.read()
        beginning = ObjectDetection(frame)
        end = ObjectDetection(frame)
        
        beginning.hsv_detection(green_lower, green_upper, 3, 5)
        beginning.rectangle_drawing(50, 500)

        end.hsv_detection(red_lower, red_upper, 1, 3)
        end.rectangle_drawing(100, 500)
        
        try:
            cv2.imshow('Video', frame)
        except:
            pass

        #cam_pub.image_pub(topic_name='/Camera', image=video)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    shot.release()
    cv2.destroyAllWindows()