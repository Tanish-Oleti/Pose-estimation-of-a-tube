# percepIroc import mask_red
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from PIL import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from traversal.msg import WheelRpm
import std_msgs.msg as std_msgs
import cv2
import numpy as np
import rospy
import sys
import tf
import math
import sys


from tf.transformations import quaternion_from_euler
from tf import LookupException, ConnectivityException, ExtrapolationException

# run the below five lines to check if masked_red is working or not
# img=cv2.imread("scripts/red_cylinder.jpg")
# mask_red(img)
# cv2.imshow('frame',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


class tube_detector:
    def __init__(self):
        self.tube_pose_pub = rospy.Publisher("tube_pose", Point, queue_size=10)
        self.vel_pub = rospy.Publisher(
            'arm_goal', Float32MultiArray, queue_size=10)
        self.bool_pub = rospy.Publisher('arm_goal_bool', Bool, queue_size=10)
        self.velocity_pub = rospy.Publisher('motion',  WheelRpm, queue_size=10)
        self.is_identified = False

        self.bridge = CvBridge()
        # self.depth_sub=rospy.Subscriber("/galileo/zed2/depth/depth_registered",Image,self.Depth)

        self.p_x, self.p_y = 0, 0
        self.cord_x1, self.cord_y1, self.cord_z1 = 0, 0, 0
        self.cv_image = None
        self.image_arrived = False
        self.image_sub = rospy.Subscriber(
            "/zed2i/zed_node/rgb/image_rect_color", Image, self.callback)
        self.depth_sub = rospy.Subscriber(
            "/zed2i/zed_node/depth/depth_registered", Image, self.Depth)
        self.ik_start_sub = rospy.Subscriber(
            "/main_control", Bool, self.main_callback)
#        self.ik_start=True
        self.main_control = True

    def main_callback(self, data):
        self.main_control = data

    def Depth(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, 'passthrough')

        except CvBridgeError as e:
            print(e)
        # print(np.ndim(cv_depth))
        # print(cv_depth)
        self.depth = cv_depth[self.p_y, self.p_x]
        print(f"Depth: {self.depth}")
        print(self.p_x, self.p_y)
        # print(self.p_y, self.p_x)
        # print(cv_depth.shape)

    def show_coordinates(self, p_x, p_y, cv_image):
        # K matrix values
        f_x = 527.2972398956961
        f_y = 527.2972398956961
        c_x = 300.0
        c_y = 180.0

        self.cord_x = self.depth*(p_x-c_x)/f_x
        self.cord_y = self.depth*(p_y-c_y)/f_y
        self.cord_z = self.depth

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.color = [255, 0, 0]
        self.thickness = 1

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.image_arrived = True
        except CvBridgeError as e:
            print(e)

    def main(self):
        if self.is_identified == False:
            g = WheelRpm()
            g.vel = 20
            self.velocity_pub.publish(g)
        if (self.image_arrived == True):
            self.p_x, self.p_y = self.mask_red(self.cv_image)
            if self.p_x is not None and self.p_y is not None:
                self.show_coordinates(self.p_x, self.p_y, self.cv_image)
                trans = self.tube_frame(self.cord_x, self.cord_y, self.cord_z)
                self.cord_x1 = trans[0]
                self.cord_y1 = trans[1]
                self.cord_z1 = trans[2]
                cv2.putText(self.cv_image, str((self.cord_x, self.cord_y, self.cord_z)), (int(self.p_x), int(
                    self.p_y)), self.font, self.font_scale, self.color, self.thickness, cv2.LINE_AA)

                goal_coord = [self.cord_x, self.cord_y, self.cord_z]
                goal_coord = np.asanyarray(goal_coord)
                if self.main_control :

                    msg = Float32MultiArray()
                    msg.data = [0, 0, 0]
                    msg.layout = std_msgs.MultiArrayLayout()
                    msg.layout.data_offset = 0
                    msg.layout.dim = [std_msgs.MultiArrayDimension()]
                    msg.layout.dim[0].size = msg.layout.dim[0].stride = len(
                        msg.data)
                    msg.layout.dim[0].label = 'write'
                    msg.data = [0, 0, 0]
                    msg.data[1] = self.cord_z - 0.1  # offset for zed camera
                    msg.data[2] = -self.cord_y - 0.36
                    msg.data[0] = self.cord_x + 0.02
                    print("Msg Data = ", msg.data)
                    self.is_identified = True
                    d = math.pow(self.cord_x**2 + self.cord_y **
                                 2 + self.cord_z**2, 0.5)
                    g = WheelRpm()
                    kp = 26.67
                    if msg.data[1] > 0.75:
                        g.vel = 20
                    elif msg.data[1] > 0.5 and msg.data[1] < 0.75:
                        g.vel = int(min(kp * d, 20))
                        # g.vel = 0
                    else:
                        g.vel = 0

                    if self.p_x > 500:
                        g.omega = 50
                    elif self.p_x < 200 and self.p_x > 0:
                        g.omega = -50
                    else:
                        g.omega = 0
                    print("G is amazing", g)
                    # msg.data = [0,0,0,0,0,0]
                    self.velocity_pub.publish(g)
#                    if msg.data[0]!=0 and msg.data[1]!=0 and self.ik_start:
                    if msg.data[0] != 0 and msg.data[1] != 0 and g.vel == 0 and g.omega == 0:
                        # msg.data = [0,0,0,0,0,0]
                        self.vel_pub.publish(msg)
                        self.bool_pub.publish(True)
                        print("this is published")

            else:
                pass
            # cv2.imshow('frame',self.cv_image)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
                # cv2.destroyAllWindows()

    def tube_frame(self, x, y, z):
        # self.tube_pose_pub.publish(t)
        # print(x, y, z)
        br = tf.TransformBroadcaster()
        trans = (0, 0, 0)
        br.sendTransform((x, y, z), (0, 0, 0, 1), rospy.Time.now(
        ), "sample_tube", "zed2i_left_camera_optical_frame")
        listener = tf.TransformListener()
        rospy.sleep(0.1)
        print(2)

        try:
            (trans, rot) = listener.lookupTransform(
                "base_link", "sample_tube", rospy.Time(0))
            # print(trans)
            br.sendTransform(trans, (0, 0, 0, 1), rospy.Time.now(),
                             "sample_tube_base", "base_link")

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)
            pass
        return trans

    def mask_red(self, color_image):
        pose = []
        cX, cY = 0, 0
        red_lower = np.array([0, 180, 100])
        red_upper = np.array([10, 255, 255])
        img_hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(img_hsv, red_lower, red_upper)
        red_result = cv2.bitwise_and(color_image, color_image, mask=red_mask)
        blurred = cv2.GaussianBlur(red_result, (21, 21), 0)
        ret, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)
        img_gray = cv2.cvtColor(thresh, cv2.COLOR_BGR2GRAY)
        ret, thresh2 = cv2.threshold((img_gray), 20, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(
            thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
            else:
                cX, cY = 0, 0
            pose.append([cX, cY])
        cv2.circle(color_image, (cX, cY), 3, (255, 0, 0), -1)
        cv2.drawContours(color_image, contours, -1, (0, 255, 0), 2)
        return cX, cY

    def spin(self):
        while not rospy.is_shutdown():
            # if type(self.cv_image) != None:
            self.main()
            rate.sleep()


if __name__ == '__main__':
    try:
        td = tube_detector()
        rospy.init_node('tube_detector', anonymous=True)
        rate = rospy.Rate(10)
        td.spin()
    except KeyboardInterrupt:
        sys.exit()
