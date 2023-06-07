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
                    if robot:
                        x,y,w,h = cv2.boundingRect(contour)
                        cX = int(x+(w/2))
                        cY = int(y+(h/2))
                        
                        self.center = [(cX, cY)]
                        idx = 0

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


class Publisher:

    def __init__(self, node_name='vision_node'): #topic_name='/Camera_vision', msg=sensor_msgs.msg.Image, queue_size=1
        rospy.init_node(node_name)
    
    def image_pub(self, topic_name=None, image=None, image_format="rgb8", queue_size=1):
        topic = rospy.Publisher(topic_name, sensor_msgs.msg.Image, queue_size=queue_size)

        bridge = CvBridge()
        final_img = bridge.cv2_to_imgmsg(image, image_format)
        topic.publish(final_img)

    def point_pub(self, topic_name=None, x_value=None, y_value=None, z_value=None, queue_size=1):
        topic = rospy.Publisher(topic_name, geometry_msgs.msg.Point, queue_size=queue_size)

        position_point = Point()

        position_point.x = x_value
        position_point.y = y_value
        position_point.z = z_value

        topic.publish(position_point)

    def float32_pub(self, topic_name=None, value=None, queue_size=1):
        topic = rospy.Publisher(topic_name, std_msgs.msg.Float32, queue_size=queue_size)

        msg = Float32()

        msg.data = value

        topic.publish(msg)
    
    def bool_pub(self, topic_name=None, value=None, queue_size=1):
        topic = rospy.Publisher(topic_name, std_msgs.msg.Bool, queue_size=queue_size)

        msg = Bool()

        msg.data = value

        topic.publish(msg)
    
    def int_array_pub(self, topic_name=None, value=None, queue_size=1):
        topic = rospy.Publisher(topic_name, std_msgs.msg.Int32MultiArray, queue_size=queue_size)

        msg = Int32MultiArray()

        msg.data = value

        topic.publish(msg)


class TrajectoryPlanner:
    
    def __init__(self, frame=None, init_point = None, final_point = None, robot = None, obstacle = None):
        self.frame = frame
        self.init_point = init_point
        self.final_point = final_point
        self.robot = robot
        self.obstacle = obstacle

        self.gray_frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
        self.motions = [0, 0, 0, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward

    def linear_path(self):
        self.line_mask = np.zeros_like(self.gray_frame)

        #cv2.line(self.frame, self.init_point.center, self.final_point.center, (255, 255, 0), 2)
        cv2.line(self.line_mask, self.init_point.center, self.final_point.center, (255, 255, 255), 1)
    
    def angular_path(self):
        ang = []

        for mask in range(0,len(self.obstacle.mask)):
            robot_line_mask = np.zeros_like(self.gray_frame)

            robot_line_mask, last_point = self._get_object_orientation(robot_line_mask, self.robot, mask)

            #print(centerx, centery, self.robot.center, self.final_point.center)

            ang.append(self._get_angle(self.robot.center[mask], (0, self.robot.center[mask][1]), last_point)) #First point hast to be the intersection
            #cv2.putText(self.frame, str(ang), (self.robot.center[mask]+10, self.robot.center[mask]-20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 255, 255), 1)

        return ang
    
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

    def trajectory_displacement(self):
        dsts = []

        horizontal_lines = self._get_perpendicular_line() #masks with drawn lines

        for horz_line in range(0,len(horizontal_lines)):

            points_mask = cv2.bitwise_and(horz_line, self.line_mask)    #, self.displacement_mask)

            contours, _ = cv2.findContours(points_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                contour = cv2.approxPolyDP(cnt, 0.001*cv2.arcLength(cnt, True), True)
                x,y,w,h = cv2.boundingRect(contour)
                
                point = (int(x+w/2), int(y+h/2))
                dst = self._get_distance(point, self.robot.center[horz_line])

                if point[1] > self.robot.center[horz_line][1]:
                    dst = dst*-1
            
            dsts.append(dst)
            
        return dsts

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
    
    def _get_perpendicular_line(self):
        masks = []

        for robot in range(0,len(self.robot.box)):
            mask = np.zeros_like(self.gray_frame)
            
            box = self.robot.box[robot]
            low_mid = self._get_mid_point(box[0], box[1])
            high_mid = self._get_mid_point(box[2], box[3])

            slope =  self._get_slope(low_mid, high_mid)

            height, width, _ = self.frame.shape

            for x in range(0,width):
                y=int(slope*(x-low_mid[0])+low_mid[1])
                if(y>=0 and y<height):
                    cv2.circle(mask, (x,y), 1, (255,255,255))
            
            masks.append(mask)
        
        return masks

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


class Controller:
    
    def __init__(self, variable=None, ref=None, dst = None):
        self.variable = variable
        self.ref = ref
        self.u1 = 0
        self.ang_error = None

        self.dst = dst
    
    def PID(self, Kp, Ki, Kd, Tm, sat_min=-20, sat_max=20, sat_min_value=-20, sat_max_value=20):
        error0 = self.ref - self.variable

        error1 = 0
        error2 = 0

        u = self.u1 + ( Kp + Kd/Tm)*error0 + (-Kp + Ki*Tm - 2*Kd/Tm)*error1 + (Kd/Tm)*error2
        #ux = ux1 + (Kpx + Kdx/Tm)*errx0 + (-2*Kdx/Tm)*errx1 + (-Kpx + Kdx/Tm)*errx2

        if u >= sat_max: u = sat_max_value
        elif u <= sat_min: u = sat_min_value

        self.u1 = u
        error1 = error0
        error2 = error1

        #if ux > 3 or ux < -3:

        if error0 < -5 or error0 > 5:
            return u
    
    def trajectory_control(self, error_lst, range=(None, None)):

        M = self._orientation_control(error_lst)
        
        error_lst = []
        
        return M, error_lst
    
    def _orientation_control(self, error_lst):
        self.ang_error = self.ref - self.variable
        error_lst.append(self.ang_error)
        lst_size = len(error_lst)
        print(self.ang_error, self.ref, self.variable)
    
        if self.ang_error < -15 or self.ang_error > 15:
            if self.variable > 0:
                print(self.ref, self.variable, self.ang_error)
                if self.ang_error > 0:
                    M = [1, 0, 0, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward
                else:
                    M = [0, 1, 0, 0, 0]
            else:
                if self.ang_error < 0:
                    M = [1, 0, 0, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward
                else:
                    M = [0, 1, 0, 0, 0]
        else:
            M = self._lateral_displacement_control()

        return M
    
    def _lateral_displacement_control(self):
        #M = [0, 0, 0, 0, 0]

        #print(self.variable)
    
        if self.dst < -20 or self.dst > 20:
            if self.dst > 0:
                M = [0, 0, 1, 0, 0] #turn_right, turn_left, right_walk, left_walk, forward
            else:
                M = [0, 0, 0, 1, 0]
        else:
            M = [0, 0 , 0, 0, 1]
            ####M = [0, 0, 0, 0, 0] #Staying still until partner get in position
        
        return M
                




if __name__ == '__main__':
    
    shot = cv2.VideoCapture(2)

    cam_pub = Publisher()

    actions = Publisher()
    
    pink_lower = (165, 125, 125)
    pink_upper = (175, 255, 255)

    green_lower = (50, 200, 0)
    green_upper = (85, 255, 255)

    red_lower = (0, 150, 150)
    red_upper = (20, 255, 255)

    robot_lower = (15, 165, 175)
    robot_upper = (50, 255, 255)

    data = []
    
    exit = False

    while True:
        try:

            ret, frame = shot.read()

            obstacle = ObjectDetection(frame)
            beginning = ObjectDetection(frame)
            end = ObjectDetection(frame)
            robot = ObjectDetection(frame)

            dummy_frame = np.zeros_like(frame)

            final_frame = frame
            dummy_frame = np.zeros_like(frame)

            robot.hsv_detection(robot_lower, robot_upper, 1, 1)
            robot.rectangle_drawing(100, 1000, True, True)

            obstacle.hsv_detection(pink_lower, pink_upper, 3, 5, True)
            obstacle.frame = dummy_frame
            obstacle.rectangle_drawing(50, 1000, True)
            
            beginning.hsv_detection(green_lower, green_upper, 3, 5)
            beginning.rectangle_drawing(50, 500)

            end.hsv_detection(red_lower, red_upper, 1, 3)
            end.rectangle_drawing(100, 500)

            left_robot = TrajectoryPlanner(frame, beginning, end, robot, obstacle)

            left_robot.linear_path()

            try:
                line_ang = left_robot._get_angle(end.center, (0, end.center[1]), beginning.center)
                #cv2.putText(left_robot.frame, str(line_ang), (end.center[0]-50, end.center[1]), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 255, 255), 1)
            except:
                pass

            rot_obs, mid_field, ext_field = left_robot.rotate_obstacles(obstacle.mask)

            obstacle.potential_fields(frame, rot_obs, mid_field, ext_field)

            left_robot.obstacle = obstacle
            left_robot.draw_trajectory()

            frame = left_robot.frame

            robot_ang = left_robot.angular_path()

            displacement = left_robot.trajectory_displacement()
            
            movement = Controller(robot_ang, line_ang, displacement)
        except:
            pass
        
        try:
            motions, data = movement.trajectory_control(data)

            if left_robot._get_distance(beginning.center, robot.center[0]) < 25:
                motions = [0, 0, 0, 0, 0]
                exit = True
            actions.int_array_pub('/motions', motions)
        except:
            pass

        if exit:
            break

        video = left_robot.frame
        
        try:
            cv2.imshow('Video', frame)
        except:
            pass

        cam_pub.image_pub(topic_name='/Camera', image=video)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    shot.release()
    cv2.destroyAllWindows()

    if exit:
        print("LLEGÓOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO :D")