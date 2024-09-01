import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import random

class BasketStatusPublisher(Node):
    def __init__(self):
        super().__init__('basket_status_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'basket_status', 10)
        self.run_publisher()

    def run_publisher(self):
        while rclpy.ok():
            # Wait for user input
            user_input = input("Enter 'K' to publish random basket status: ")
            
            if user_input.lower() == 'k':
                self.publish_random_basket_status()

    def publish_random_basket_status(self):
        # Randomly generate the status of five baskets, each basket can hold up to 3 balls, and the color of the balls is random
        basket_status = []
        for _ in range(5):
            ball_count = random.randint(0, 3)  # Number of balls in the basket
            top_ball_color = random.randint(0, 1)  # Color of the top ball, 0 means blue, 1 means red
            basket_status.extend([ball_count, top_ball_color])

        # Create and publish the Int32MultiArray message
        msg = Int32MultiArray()
        msg.data = basket_status
        self.publisher.publish(msg)

        # Explain the topic content and generate the corresponding command line format
        self.explain_basket_status(basket_status)
        self.print_ros2_command(basket_status)

    def explain_basket_status(self, basket_status):
        for i in range(5):
            ball_count = basket_status[2*i]
            top_ball_color = "red" if basket_status[2*i + 1] == 1 else "blue"
            if ball_count > 0:
                self.get_logger().info(f"Basket {i + 1} has {ball_count} ball(s), the top ball is {top_ball_color}")
            else:
                self.get_logger().info(f"Basket {i + 1} has no balls")
        
        self.get_logger().info("Topic has been published\n")

    def print_ros2_command(self, basket_status):
        # Generate and print the ROS 2 topic publish command
        command = f"ros2 topic pub /basket_status std_msgs/msg/Int32MultiArray '{{data: {basket_status}}}' --once"
        print("\nYou can use the following command to publish the same topic content:")
        print(command + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = BasketStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shut down the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
