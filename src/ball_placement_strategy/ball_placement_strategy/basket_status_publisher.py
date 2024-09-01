import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import random
import matplotlib.pyplot as plt
import numpy as np

class BasketStatusPublisher(Node):
    def __init__(self):
        super().__init__('basket_status_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'basket_status', 10)
        print("===========================================================")
        self.run_publisher()
        

    def run_publisher(self):
        while rclpy.ok():
            user_input = input("输入 'K' 以发布并可视化随机的球筐状态: ")

            if user_input.lower() == 'k':
                self.publish_random_basket_status()

    def publish_random_basket_status(self):
        basket_status = []
        for _ in range(5):
            ball_count = random.randint(0, 3)  # 球筐中的球数
            top_ball_color = random.randint(0, 1)  # 顶部球的颜色，0 表示蓝色，1 表示红色
            basket_status.extend([ball_count, top_ball_color])

        # 创建并发布 Int32MultiArray 消息
        msg = Int32MultiArray()
        msg.data = basket_status
        self.publisher.publish(msg)

        self.explain_basket_status(basket_status)
        self.print_ros2_command(basket_status)
        self.visualize_basket_status(basket_status)

    def explain_basket_status(self, basket_status):
        for i in range(5):
            ball_count = basket_status[2*i]
            top_ball_color = "红色" if basket_status[2*i + 1] == 1 else "蓝色"
            if ball_count > 0:
                self.get_logger().info(f"球筐 {i + 1} 有 {ball_count} 个球，顶部球是 {top_ball_color}")
            else:
                self.get_logger().info(f"球筐 {i + 1} 没有球")
        
        self.get_logger().info("话题已发布\n")

    def print_ros2_command(self, basket_status):
        command = f"ros2 topic pub /basket_status std_msgs/msg/Int32MultiArray '{{data: {basket_status}}}' --once"
        print("\n使用以下命令发布相同的话题内容:")
        print(command + "\n")

    def visualize_basket_status(self, basket_status):
        basket_labels = ['Hoop 1', 'Hoop 2', 'Hoop 3', 'Hoop 4', 'Hoop 5']
        ball_counts = [basket_status[2*i] for i in range(5)]
        colors = ['red' if basket_status[2*i + 1] == 1 else 'blue' for i in range(5)]

        # 条形图
        plt.bar(basket_labels, ball_counts, color=colors)
        plt.ylim(0, 3, 1)
        plt.yticks(np.arange(0, 4, 1))  # 设置 y 轴刻度，间隔为 1
        plt.xlabel('Hoop')
        plt.ylabel('Nums of Balls')
        plt.title('Hoop Status Visulization')
        # plt.text(0.5, 2.8, '按 "K" 关闭', horizontalalignment='center', fontsize=12, color='green')

        # 显示图表并等待用户按 "K"
        plt.show(block=False)
        plt.pause(0.1)
        input("按 'K' 关闭图表并生成新的球筐状态: ")
        print("===========================================================")
        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = BasketStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
