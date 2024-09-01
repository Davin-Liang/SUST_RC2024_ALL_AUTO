import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class BallPlacementNode(Node):
    def __init__(self):
        super().__init__('ball_placement_node')
        
        # 订阅视觉节点发布的话题
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'basket_status',
            self.basket_status_callback,
            10)
        
        # 发布最佳放球框和次要放球框的序号
        self.publisher = self.create_publisher(
            Int32MultiArray,
            'best_and_secondary_basket',
            10)
        
        self.get_logger().info("请注意，我方为红色方!!!!!!")
        print("===========================================================")
        self.get_logger().info("等待获取篮筐信息......")

    def basket_status_callback(self, msg):
        basket_data = msg.data  # 读取数据
        # 解析篮框信息
        baskets = [
            {"count": basket_data[0], "top_ball": basket_data[1]},
            {"count": basket_data[2], "top_ball": basket_data[3]},
            {"count": basket_data[4], "top_ball": basket_data[5]},
            {"count": basket_data[6], "top_ball": basket_data[7]},
            {"count": basket_data[8], "top_ball": basket_data[9]},
        ]
        
        # 找到最佳放球框和次要放球框
        best_basket = self.get_best_basket(baskets)
        secondary_basket = self.get_secondary_basket(baskets, best_basket)

        self.get_logger().info(f"最佳放球框: 框 {best_basket + 1}")
        if secondary_basket is not None:
            self.get_logger().info(f"次要放球框: 框 {secondary_basket + 1}")
        else:
            self.get_logger().info("次要放球框: 没有可用的框")

        # 发布最佳放球框和次要放球框
        self.publish_best_and_secondary_basket(best_basket, secondary_basket)
        print("===========================================================")
        self.get_logger().info("等待获取篮筐信息......")
        

    def get_best_basket(self, baskets):
        # 获取每个框的球数
        counts = [basket["count"] for basket in baskets]
        top_balls = [basket["top_ball"] for basket in baskets]
        
        # 找到所有有2个球的框
        two_ball_baskets = [i for i, basket in enumerate(baskets) if basket["count"] == 2]

        # 新策略：如果有2个或更多框有2个球，最佳和次要框从这些框中选
        if len(two_ball_baskets) >= 2:
            # 从有两个球的框中优先选择最上方球为蓝色的框作为最佳放球框
            for i in two_ball_baskets:
                if baskets[i]["top_ball"] == 0:  # 蓝色
                    return i
            return two_ball_baskets[0]  # 如果没有蓝色顶球的框，返回其中任意一个

        # 策略 2：如果一个框有2个球，另一个框为空，选择有2个球的框为最佳放球框
        if len(two_ball_baskets) == 1:
            return two_ball_baskets[0]

        # 默认策略：选择球数最少的框
        min_count = 3  # 每个框最多放3个球
        best_basket = None
        
        for i, basket in enumerate(baskets):
            if basket['count'] < min_count:
                min_count = basket['count']
                best_basket = i
                
        return best_basket
    
    def get_secondary_basket(self, baskets, best_basket):
        # 获取每个框的球数
        counts = [basket["count"] for basket in baskets]

        # 找到所有有2个球的框
        two_ball_baskets = [i for i, basket in enumerate(baskets) if basket["count"] == 2]
        
        # 新策略：如果有2个或更多框有2个球，次要框从这些框中选
        if len(two_ball_baskets) >= 2:
            for i in two_ball_baskets:
                if i != best_basket:  # 确保次要框与最佳框不同
                    return i

        # 默认策略：如果没有2个球的框，允许次要框为空框
        min_count = 3
        secondary_basket = None
        for i, basket in enumerate(baskets):
            if i != best_basket and basket['count'] < min_count:
                min_count = basket['count']
                secondary_basket = i
                
        return secondary_basket
    
    def publish_best_and_secondary_basket(self, best_basket, secondary_basket):
        # 创建要发布的 Int32MultiArray 消息
        msg = Int32MultiArray()
        # 如果没有次要放球框，将其设置为 0
        secondary_basket_value = secondary_basket + 1 if secondary_basket is not None else 0
        msg.data = [best_basket + 1, secondary_basket_value]  # 将框序号转为1-indexed

        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f"发布最佳放球框和次要放球框: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = BallPlacementNode()
    
    rclpy.spin(node)
    
    # 关闭节点
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
