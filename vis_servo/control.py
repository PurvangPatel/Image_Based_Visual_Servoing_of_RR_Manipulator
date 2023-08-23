import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Float64MultiArray
from center_interfaces.msg import Centercolor
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library
import numpy as np
import imutils




class VisControl(Node):

    def __init__(self):
        super().__init__('control')

        self.subscription_2 = self.create_subscription(Image,'/camera1/image_raw',self.listener_callback,10)
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands',10)
        self.br = CvBridge()

        

    def pixel_to_image(self,size, center):
        image_center = [size[0]/2, size[1]/2]
        center_image_frame = []
        for i in range(np.size(center)):
            if (i % 2 == 0):
                center_image_frame.append(image_center[0] - center[i])
            else:
                center_image_frame.append(image_center[1] - center[i])
        return np.array([center_image_frame]).T

    def center(self,current_frame):
        center = []
        low_b = (100, 0, 0)   #blue_thresolding
        high_b = (255,40,40)
        mask_b = cv.inRange(current_frame,low_b,high_b)
        contour_b = cv.findContours(mask_b, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contour_b = imutils.grab_contours(contour_b)
        for i in contour_b:
            M = cv.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                center.append(cx)
                center.append(cy)

        low_r = (0, 0, 100)   #red_thresolding
        high_r = (30,30,255)
        mask_r = cv.inRange(current_frame,low_r,high_r)
        contour_r = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contour_r = imutils.grab_contours(contour_r)
        i = contour_r[0]
        M = cv.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            center.append(cx)
            center.append(cy)

        low_g = (0, 200, 0)   #green_thresolding
        high_g = (15,255,15)
        mask_g = cv.inRange(current_frame,low_g,high_g)
        contour_g = cv.findContours(mask_g, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contour_g = imutils.grab_contours(contour_g)
        i = contour_g[0]
        M = cv.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            center.append(cx)
            center.append(cy)

        low_p = (210, 0, 210)   #pink_thresolding
        high_p = (255,100,255)
        mask_p = cv.inRange(current_frame,low_p,high_p)
        contour_p = cv.findContours(mask_p, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contour_p = imutils.grab_contours(contour_p)
        i = contour_p[0]
        M = cv.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            center.append(cx)
            center.append(cy)
        return np.array([center]).T





    def listener_callback(self, data):
        

        current_frame = self.br.imgmsg_to_cv2(data) 

        # Location of desried color center
        desired_center = np.array([[43, 415, 90, 484, 155, 438, 107, 368]]).T
        desired_center_image = self.pixel_to_image(current_frame.shape, desired_center) #Transforming into image co-ordinates

        # Location of current center
        current_center = self.center(current_frame)
        current_center_image = self.pixel_to_image(current_frame.shape, current_center)

        e =   current_center_image - desired_center_image  #Error

        jacobian = np.array([[0 ,0],[0.925, 0.45],[0 ,0],[0 ,0],[0 ,0],[1,1]])  #Robot Jacobian
        jacobian_inv = np.linalg.pinv(jacobian)

        lx = np.array([[-1, 0], [0, -1],[-1, 0], [0, -1],[-1, 0], [0, -1],[-1, 0], [0, -1]]) #For Image Jacobian
        lx_inv = np.linalg.pinv(lx) 

        lam = -0.3  #Lambda
        lx_inv = np.multiply(lx_inv,lam)

        vc = np.matmul(lx_inv,e) #Camera Velocity
        rot = np.array([[0, -1],[1, 0]]) #Rotation about z-axis 90 deg
        vc_base = np.append(np.matmul(rot,vc),[0,0,0,0]).T #Camera velocity w.r.t Base frame 

        vj = np.matmul(jacobian_inv,vc_base)  #Joint Velocity
        data = [vj[0],vj[1]]

        v_joint = Float64MultiArray(data=data) #Publishing to Velocity Controller
        self.publisher.publish(v_joint)
            
        




def main(args=None):

    rclpy.init(args=args)

    control = VisControl()

    rclpy.spin(control)


    rclpy.shutdown()

if __name__ == '__main__':
  main()