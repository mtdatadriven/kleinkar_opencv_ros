# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands

class ControlTurtlesim(Node):
    """Create a ControlTurtlesim class, which is a subclass of the Node class."""
    
    def __init__(self):
        """Class constructor to set up the node."""
        super().__init__('control_turtlesim')  # Initialize the Node class with a name

        # Create a publisher for the '/turtle1/cmd_vel' topic
        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Set up a timer to call publish_velocity every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # Initialize a Twist message for controlling velocities
        self.move_cmd = Twist()  # Create a Twist message
        self.move_cmd.linear.x = 1.0
        self.move_cmd.linear.y = 0.0
        self.move_cmd.linear.z = 0.0
        self.move_cmd.angular.x = 0.0
        self.move_cmd.angular.y = 0.0
        self.move_cmd.angular.z = 1.5

        self.get_logger().info("Turtlesim control started.")  # Log startup message
        self.shutdown_requested = False  # Flag to track shutdown status
        self.create_shutdown_handler()  # Set up shutdown handler

    def publish_velocity(self):
        """Publish velocity commands if shutdown has not been requested."""
        if not self.shutdown_requested:  # Check shutdown flag
            self.cmd_vel.publish(self.move_cmd)  # Publish the move command

    def create_shutdown_handler(self):
        """Set up a handler to manage shutdown events."""
        rclpy.get_default_context().on_shutdown(self.shutdown)  # Register shutdown method

    def shutdown(self):
        """Handle shutdown process."""
        self.shutdown_requested = True  # Set shutdown flag
        self.get_logger().info("Stopping Turtlesim...")  # Log shutdown message
        
        # Publish a stop command (zero velocity)
        self.cmd_vel.publish(Twist())  # Publish a zero velocity command to stop the turtle
        
        # Wait for the message to be processed
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))  # Sleep for 1 second

def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)  # Initialize the rclpy library
    node = ControlTurtlesim()  # Create an instance of ControlTurtlesim
    rclpy.spin(node)  # Spin the node to keep it running
    node.destroy_node()  # Cleanup the node
    rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == "__main__":
    main()  # Execute the main function