#from tube_detection import mask_red
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from PIL import Image

from std_msgs.msg import String
import cv2
import numpy as np
import rospy
import sys

#run the below five lines to check if masked_red is working or not
#img=cv2.imread("scripts/red_cylinder.jpg")
#mask_red(img)
#cv2.imshow('frame',img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

class tube_detector:
    def __init__(self):
        #self.tube_pose_pub = rospy.Publisher("tube_pose",String)
        self.bridge= CvBridge()
        #self.depth_sub=rospy.Subscriber("/galileo/zed2/depth/depth_registered",Image,self.Depth)
        self.image_sub = rospy.Subscriber("/galileo/zed2/left/image_rect_color",Image,self.callback)
        self.depth_sub=rospy.Subscriber("/galileo/zed2/depth/depth_registered",Image,self.Depth)

    def Depth(self,data):
        try:
            cv_depth=self.bridge.imgmsg_to_cv2(data,'passthrough')

        except CvBridgeError as e:
            print(e)
        #print(np.ndim(cv_depth))
        #print(cv_depth)
        self.depth=cv_depth[int(self.p_y)][int(self.p_x)]

    def show_coordinates(self,p_x,p_y,cv_image):
        #K matrix values
        f_x=527.2972398956961
        f_y=527.2972398956961
        c_x=658.8206787109375
        c_y=372.25787353515625
    

        self.cord_x=self.depth*(p_x-c_x)/f_x
        self.cord_y=self.depth*(p_y-c_y)/f_y
        self.cord_z=self.depth

        self.font=cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale=0.5
        self.color=[255,0,0]
        self.thickness=1
        #print(cord_x,cord_y,cord_z)
        #cv2.putText(cv_image,str((cord_x,cord_y,cord_z)),(p_x,p_y),font,font_scale,color,thickness,cv2.LINE_AA)

    def mask_red(self,frame):
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # lowerLimit, upperLimit = get_limits(color=yellow)
        #for red
        lowerLimit = np.array([0, 200, 150])
        upperLimit = np.array([5, 255, 255])
        #for green
        #lowerLimit=np.array([50,240,240])
        #upperLimit=np.array([70,255,255])
        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        mask_ = Image.fromarray(mask)

        bbox = mask_.getbbox()

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 5)
            p_x=(x1+x2)/2
            p_y=(y1+y2)/2
            return p_x,p_y
        else:
            return None,None


    def callback(self,data):
        try:
            cv_image=self.bridge.imgmsg_to_cv2(data,'bgr8')
        except CvBridgeError as e :
            print(e)

        self.p_x,self.p_y=self.mask_red(cv_image)
        #print(self.p_x,self.p_y)
        if self.p_x is not None:


            self.show_coordinates(self.p_x,self.p_y,cv_image)
            cv2.putText(cv_image,str((self.cord_x,self.cord_y,self.cord_z)),(int(self.p_x),int(self.p_y)),self.font,self.font_scale,self.color,self.thickness,cv2.LINE_AA)
        else:
            pass
        cv2.imshow('frame',cv_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            


def main(args):
    td=tube_detector()
    rospy.init_node('tube_detector',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__ == '__main__':
    main(sys.argv)






