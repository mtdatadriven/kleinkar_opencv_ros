# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Class for creating ROS 2 nodes
from sensor_msgs.msg import Image  # Import the Image message type for handling image data
from cv_bridge import CvBridge  # Import CvBridge for converting between ROS and OpenCV image formats
import cv2  # Import OpenCV library for image processing tasks

class ImageSubcriber(Node):
    """Create an ImageSubscriber class, which is a subclass of the Node class."""
    
    def __init__(self):
        # Initialize the node with the name 'image_subscriber'
        super().__init__('image_subscriber')
        
        # Create a subscription to the 'video_frames' topic
        self.subscription = self.create_subscription(
            Image,  # Message type
            'video_frames',  # Topic name
            self.listener_callback,  # Callback function to handle incoming messages
            10  # Queue size for incoming messages
        )
        self.subscription  # Prevent unused variable warning

        # Initialize CvBridge to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Convert the incoming ROS Image message to an OpenCV-compatible format
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert the color format from RGB to BGR for OpenCV compatibility
        frame_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        
        # Display the image in a window titled 'Video Frame'
        cv2.imshow('Video Frame', frame_bgr)
        
        # Wait for a short period (1 ms) to allow the image to be displayed
        cv2.waitKey(1)  

        # Log a message indicating a new video frame has been received
        self.get_logger().info('Received video frame.')

def main(args=None):
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the ImageSubscriber class
    image_subscriber = ImageSubcriber()

    # Enter a loop to keep the node active and process incoming messages
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly (optional, as it will be done automatically)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS 2 communication
    rclpy.shutdown()
    
    # Close any OpenCV windows that were opened
    cv2.destroyAllWindows()

# Entry point of the script
if __name__ == '__main__':
    main()  # Run the main function
