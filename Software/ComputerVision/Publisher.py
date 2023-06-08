import rospy
import sensor_msgs
from cv_bridge import CvBridge
import geometry_msgs
from geometry_msgs.msg import Point
import std_msgs
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray

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