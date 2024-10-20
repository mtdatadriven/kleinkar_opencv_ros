# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type for publishing images
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library for image processing

class ImagePublisher(Node):
    """Create an ImagePublisher class, which is a subclass of the Node class."""
    
    def __init__(self):
        """Class constructor to set up the node."""
        super().__init__('image_publisher')  # Call the constructor of the Node class with the name 'image_publisher'

        # Create a publisher for the 'video_frames' topic with message type Image and a queue size of 10
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

        # Set up a timer to call the timer_callback function every 0.1 seconds
        timer_period = 0.1  # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a VideoCapture object for the webcam (index 2)
        self.cap = cv2.VideoCapture(1)

        # Initialize CvBridge to convert between ROS and OpenCV images
        self.br = CvBridge()

    def timer_callback(self):
        """Callback function called every 0.1 seconds to capture and publish images."""
        # Capture a frame from the webcam
        ret, frame = self.cap.read()

        if ret:  # Check if the frame was captured successfully
            # Convert the captured frame from RGB to BGR format
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Publish the image as a ROS message using CvBridge
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame_bgr, encoding='bgr8'))
            self.get_logger().info('Publishing video frame')  # Log the action of publishing

def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)  # Initialize the rclpy library for ROS 2
    image_publisher = ImagePublisher()  # Create an instance of the ImagePublisher node
    rclpy.spin(image_publisher)  # Keep the node running and processing callbacks
    image_publisher.destroy_node()  # Clean up the node when shutting down
    rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == '__main__':
    main()  # Execute the main function when the script is run
