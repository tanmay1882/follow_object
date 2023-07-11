import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np

width = 640
height = 480
dist = 200

kp = 0.01
kd = 1

class PublisherNode(Node):
    def __init__(self):
        super().__init__("follow_target")
        self.publisher_ = self.create_publisher(Twist, "diff_cont/cmd_vel_unstamped", 10)
        self.subscriber_ = self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        lower = np.array([86,250,102])
        upper = np.array([91,255,138])
        mask = cv2.inRange(hsv,lower,upper)
        params = self.getContours(mask)
        cmd = Twist()
        try:
            x = (params[0])+(params[2]/2)
            z = params[2]
            if (abs((x)-(width/2))>=20):
                cmd.angular.z = kp*((width/2)-(x))
            if (abs((z)-dist)>=10):
                cmd.linear.x = kp*(dist - z)
        except:
            pass
        self.publisher_.publish(cmd)


    def getContours(self, img):
        contours, Hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cv2.approxPolyDP(cnt,0.025*cv2.arcLength(cnt,True),True))
                return [x, y, w, h]

def main(args=None):
    rclpy.init(args=args)
    image_publisher = PublisherNode()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()