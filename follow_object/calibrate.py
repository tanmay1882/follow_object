import rclpy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

def empty(a):
    pass 

cv2.namedWindow("Trackbars")
cv2.resizeWindow("Trackbars",640,240)
cv2.createTrackbar("Hue Min", "Trackbars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "Trackbars", 179, 179, empty)
cv2.createTrackbar("Sat Min", "Trackbars", 0, 255, empty)
cv2.createTrackbar("Sat Max", "Trackbars", 255, 255, empty)
cv2.createTrackbar("Val Min", "Trackbars", 0, 255, empty)
cv2.createTrackbar("Val Max", "Trackbars", 255, 255, empty) 

class calibrate(Node):
    def __init__(self):
        super().__init__("calibrate_node")
        self.subscriber_ = self.create_subscription(Image, "camera/image_raw", self.callback_function, 10)
        self.bridge = CvBridge()
        
    def callback_function(self, data):
        h_min = cv2.getTrackbarPos("Hue Min", "Trackbars")
        h_max = cv2.getTrackbarPos("Hue Max", "Trackbars")
        s_min = cv2.getTrackbarPos("Sat Min", "Trackbars")
        s_max = cv2.getTrackbarPos("Sat Max", "Trackbars")
        v_min = cv2.getTrackbarPos("Val Min", "Trackbars")
        v_max = cv2.getTrackbarPos("Val Max", "Trackbars")
        self.lower = np.array([h_min,s_min,v_min])
        self.upper = np.array([h_max,s_max,v_max])
        
        frame = self.bridge.imgmsg_to_cv2(data)
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,self.lower,self.upper)
        cv2.imshow("Feed", mask)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = calibrate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()