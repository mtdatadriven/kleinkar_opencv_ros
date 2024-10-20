import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge  
import cv2 
import numpy as np 

class ImageSubcriber(Node):
    """Create an ImageSubscriber class, which is a subclass of the Node class."""
    
    def __init__(self):       
        super().__init__('image_subscriber')      
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        self.subscription  
        self.br = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        low_yellow = np.array([16,143,104]) #white colour masking process
        high_yellow = np.array([46,249,214])
        yellow_mask = cv2.inRange(hsv, low_yellow, high_yellow)
        
        cv2.imshow('Video Frame', frame_bgr)
        cv2.imshow('Yellow Mask', yellow_mask)               
        cv2.waitKey(1)       
        self.get_logger().info('Received video frame.')

def main(args=None):   
    rclpy.init(args=args)
    image_subscriber = ImageSubcriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()  
