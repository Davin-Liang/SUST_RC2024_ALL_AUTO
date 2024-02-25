#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import String
#from example_interfaces.srv import AddTwoInts # 暂时，防止报错

class VisionDetecter(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        self.Mode = ""
        self.OptimalFrame = ""
        self.Frames = ({'BallNum': 0, 'HighestBall': 'no'},
                    {'BallNum': 0, 'HighestBall': 'no'},
                    {'BallNum': 0, 'HighestBall': 'no'},
                    {'BallNum': 0, 'HighestBall': 'no'},
                    {'BallNum': 0, 'HighestBall': 'no'},)
        self.VisionSubscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.VisionCallback, 10)
        #self.ModeServer_ = self.create_service(AddTwoInts, "add_two_ints_srv", self.ModeCallback)
        self.ModeSubscribe_ = self.create_subscription(String, "mode", self.ModeCallback, 10)
        self.FramePublisher_ = self.create_publisher(String, "OptimalFrame", 10)

    def ModeCallback(self, msg)
        self.Mode = msg.data

    def VisionCallback(self, msg):
        BallLists = []
        FramesLists = []
        Ball = {'Color': '', 'CentralPoint': []}
        Frame = []
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                if msg.targets[i].type != "Frame":
                    Ball['Color'] = msg.targets[i].type
                    Ball['CentralPoint'].append(msg.targets[0].rois[0].rect.x_offset + msg.targets[0].rois[0].rect.height/2)
                    Ball['CentralPoint'].append(msg.targets[0].rois[0].rect.y_offset + msg.targets[0].rois[0].rect.width/2)
                else:
                    Frame.append(msg.targets[0].rois[0].rect.x_offset + msg.targets[0].rois[0].rect.height/2)
                    Frame.append(msg.targets[0].rois[0].rect.y_offset + msg.targets[0].rois[0].rect.width/2)
                BallLists.append(Ball)
                FramesLists.append(Frame)

        if 0 != len(BallLists):
            if "a" == self.Mode:
                pass
            elif "b" == self.Mode:
                pass
        if 0 != len(BallLists) and 0 != len(FramesLists):
            if "c" == self."Mode":
                self.SelectOptimalBallFrame(BallLists, FramesLists, "red")

    def SelectOptimalBallFrame(self, BallLists, FramesLists, MainColor='red', Threshold=5):
        YCoords = [] # 记录球筐的中心坐标的 Y 坐标
        for Frame in FramesLists:
            YCoords.append(Frame[1])

        for Ball in BallLists:
            for Index, YCoord in enumerate(YCoords):
                if abs(Ball['CentralPoint'][1] - YCoord) < Threshold:
                    self.Frames[Index]['BallNum'] += 1
                    self.Frames[Index]['HighestBall'] = Ball['Color']
        
        # 最优放球框的选择

        # --------------------

        msg = String()
        msg.data = self.OptimalFrame
        self.FramePublisher_.publish(msg)

    def DetermineIfBallFrameIsOptimal(self, BallLists, MainColor='red'):
        Frame = {'BallNum': 0, 'HighestBall': 'no'}
        BallX = BallLists[0]['CentralPoint'][0]
        for Ball in BallLists:
            if Ball['CentralPoint'][0] <= BallX:
                Frames['BallNum'] += 1
                Frame['HighestBall'] = Ball['Color']
        if Frame == Frame[int(self.Optimal)]:
            msg = String()
            msg.data = "y"
            self.FramePublisher_.publish(msg)

    def SelectOptimalBall(self):
        pass

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = VisionDetecter("VisionDetecter")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

