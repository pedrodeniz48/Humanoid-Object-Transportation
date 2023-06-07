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
                    """if robot:
                        x,y,w,h = cv2.boundingRect(contour)
                        cX = int(x+(w/2))
                        cY = int(y+(h/2))
                        
                        self.center = [(cX, cY)]
                        idx = 0"""

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


class TrajectoryPlanner:
    
    def __init__(self, frame=None, init_point = None, final_point = None, obstacle = None):
        self.frame = frame
        self.init_point = init_point
        self.final_point = final_point
        self.obstacle = obstacle

        self.gray_frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
        self.motions = [0, 0, 0, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward

    def linear_path(self):
        self.line_mask = np.zeros_like(self.gray_frame)

        #cv2.line(self.frame, self.init_point.center, self.final_point.center, (255, 255, 0), 2)
        cv2.line(self.line_mask, self.init_point.center, self.final_point.center, (255, 255, 255), 1)
    
    def rotate_obstacles(self, masks):
        or_obs = []
        mid_f = []
        ext_f = []
        
        for mask in range(0,len(self.obstacle.mask)):
            obstacle_mask = np.zeros_like(self.gray_frame)

            obstacle_mask, last_point = self._get_object_orientation(obstacle_mask, self.obstacle, mask)

            ang = self._get_angle(self.obstacle.center[mask], (0, self.obstacle.center[mask][1]), last_point) #First point hast to be the intersection
            #cv2.putText(self.frame, str(ang), (self.obstacle.center[mask][0]+10, self.obstacle.center[mask][1]-20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 255, 255), 1)

            #cv2.imshow('mask', and_mask)
            #cv2.imshow('video', frame)

            if ang is not None:
                original_obs = cv2.getRotationMatrix2D(self.obstacle.center[mask], ang-46, 1)
                orig_rotation = cv2.warpAffine(self.obstacle.mask[mask], original_obs, (frame.shape[1], frame.shape[0]))

                middle_field = cv2.getRotationMatrix2D(self.obstacle.center[mask], ang-46, 4)
                mid_rotation = cv2.warpAffine(self.obstacle.mask[mask], middle_field, (frame.shape[1], frame.shape[0]))

                ext_field = cv2.getRotationMatrix2D(self.obstacle.center[mask], ang-46, 5)
                ext_rotation = cv2.warpAffine(self.obstacle.mask[mask], ext_field, (frame.shape[1], frame.shape[0]))

                or_obs.append(orig_rotation)
                mid_f.append(mid_rotation)
                ext_f.append(ext_rotation)
                #cv2.imshow('test', rotation)

        return or_obs, mid_f, ext_f
    
    def draw_trajectory(self):
        self.trajectory_mask = np.zeros_like(self.gray_frame)
        self.displacement_mask = np.zeros_like(self.gray_frame)

        points = []
        points.append(self.final_point.center)
        obs_pts = self._detect_obstacles()

        for pt in obs_pts:
            points.append(pt)

        points.append(self.init_point.center)
        
        for i in range(0,len(points)-1):
            cv2.line(self.trajectory_mask, points[i], points[i+1], (255, 255, 255), 2)
            cv2.line(self.frame, points[i], points[i+1], (175, 255, 0), 2)

            if i%2 == 0:
                cv2.line(self.displacement_mask, points[i], points[i+1], (255, 255, 255), 2)

    def _detect_obstacles(self):
        self.obstacle.mid_boxes.reverse()

        point_mask = cv2.bitwise_and(self.line_mask, self.obstacle.mid_field_mask)

        contours, _ = cv2.findContours(point_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        centers = []
        output_pts = []
        
        for cnt in contours:
            contour = cv2.approxPolyDP(cnt, 0.001*cv2.arcLength(cnt, True), True)
            x,y,w,h = cv2.boundingRect(contour)

            center = (int(x+w/2), int(y+h/2))

            centers.append(center)

        for i in range(0,len(centers)):
            if ((i+1) % 2 != 0):
                
                #cv2.circle(self.frame, centers[i], 10, (255,0,0), 2)

                dst_right = self._get_distance(centers[i], tuple(self.obstacle.mid_boxes[int(i/2)][0][3].tolist()))
                dst_left = self._get_distance(centers[i], tuple(self.obstacle.mid_boxes[int(i/2)][0][0].tolist()))
                if dst_left < dst_right:
                    #cv2.line(self.frame, centers[i], (tuple(self.obstacle.mid_boxes[i][0][0].tolist())), (0, 0, 255), 2)
                    output_pts.append(centers[i])
                    output_pts.append((tuple(self.obstacle.mid_boxes[int(i/2)][0][0].tolist())))
                    #cv2.line(self.frame, tuple(self.obstacle.mid_boxes[i][0][0].tolist()), tuple(self.obstacle.mid_boxes[i][0][1].tolist()), (0, 0, 255), 2)
                    #cv2.line(self.frame, tuple(self.obstacle.mid_boxes[i][0][1].tolist()), centers[i+1], (0, 0, 255), 2)
                    output_pts.append((tuple(self.obstacle.mid_boxes[int(i/2)][0][1].tolist())))
                    output_pts.append(centers[i+1])
                else:
                    #cv2.line(self.frame, centers[i], (tuple(self.obstacle.mid_boxes[i][0][3].tolist())), (0, 0, 255), 2)
                    output_pts.append(centers[i])
                    output_pts.append((tuple(self.obstacle.mid_boxes[int(i/2)][0][3].tolist())))
                    #cv2.line(self.frame, tuple(self.obstacle.mid_boxes[i][0][3].tolist()), tuple(self.obstacle.mid_boxes[i][0][2].tolist()), (0, 0, 255), 2)
                    #cv2.line(self.frame, tuple(self.obstacle.mid_boxes[i][0][2].tolist()), centers[i+1], (0, 0, 255), 2)
                    output_pts.append((tuple(self.obstacle.mid_boxes[int(i/2)][0][2].tolist())))
                    output_pts.append(centers[i+1])
        
        return output_pts
    
    def _get_object_orientation(self, mask=None, object=None, obj=None):
        min_dist = 100000
        box = object.box[obj]
        points = [box[1], box[3]]

        for cnt, point in enumerate(points):
            dst = self._get_distance(box[0], point)
            if dst < min_dist:
                min_dist = dst
                sec_pnt = points[cnt]
                fth_pnt = points[not cnt]

        low_mid = self._get_mid_point(box[0], sec_pnt)
        high_mid = self._get_mid_point(box[2], fth_pnt)

        slope =  self._get_slope(low_mid, high_mid)

        height, width, _ = self.frame.shape

        for x in range(0,width):
            y=int(slope*(x-low_mid[0])+low_mid[1])
            if(y>=0 and y<height):
                cv2.circle(mask, (x,y), 1, (255,255,255))
                last_point = (x, y)
        
        return mask, last_point

    def _get_mid_point(self, pointA, pointB):
        mid_point = []
        for i in [0,1]:
            mid_point.append(int((pointA[i]+pointB[i])/2))
        return mid_point
    
    def _get_distance(self, initial_point, final_point):
        if initial_point[0] == final_point[0]:
            dst = final_point[1]-initial_point[1]
        elif initial_point[1] == final_point[1]:
            dst = final_point[0]-initial_point[0]
        else:
            dst =  math.sqrt((final_point[0]-initial_point[0])**2 + (final_point[1]-initial_point[1])**2) #(y2-y1)/(x2-x1)
        return dst
    
    def _get_slope(self, initial_point, final_point):
        if initial_point[0] == final_point[0]:
            slope = 0
        elif initial_point[1] == final_point[1]:
            slope = 0
        else:
            slope = (final_point[1]-initial_point[1])/(final_point[0]-initial_point[0]) #(y2-y1)/(x2-x1)
        return slope
        
    def _get_angle(self, pt1=None, pt2=None, pt3=None):
        try:
            m1 = self._get_slope(pt1, pt2)
            m2 = self._get_slope(pt1, pt3)
            angR = math.atan((m2-m1)/(1+(m2*m1)))
            angD = round(math.degrees(angR))
            return angD
        except:
            pass
                




if __name__ == '__main__':
    
    shot = cv2.VideoCapture(2)
    
    pink_lower = (165, 125, 125)
    pink_upper = (175, 255, 255)

    black_lower = (120, 75, 0)
    black_upper = (140, 140, 50)

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

        left_robot = TrajectoryPlanner(frame, beginning, end, obstacle)
        
        left_robot.linear_path()

        rot_obs, mid_field, ext_field = left_robot.rotate_obstacles(obstacle.mask)

        obstacle.potential_fields(frame, rot_obs, mid_field, ext_field)

        left_robot.obstacle = obstacle
        left_robot.draw_trajectory()

        frame = left_robot.frame
        
        try:
            cv2.imshow('Video', frame)
        except:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    shot.release()
    cv2.destroyAllWindows()