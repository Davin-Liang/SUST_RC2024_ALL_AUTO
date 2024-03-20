#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import String

class VisionDetecter(Node):
    def __init__(self, name, MainColor):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        self.MainColor = MainColor
        self.Mode = ""
        self.OptimalFrame = ""
        self.DetectSign = False
        self.DropBallMsg = String()
        self.OptimalMsg = String()
        self.Frames = ({'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},)
        self.VisionSubscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.VisionCallback, 10)
        self.ModeSubscribe_ = self.create_subscription(String, "mode", self.ModeCallback, 10)
        self.FramePublisher_ = self.create_publisher(String, "OptimalFrame", 10)

    def ModeCallback(self, msg):
        self.Mode = msg.data
        self.DetectSign = True

    def VisionCallback(self, msg):
        if self.DetectSign:
            return

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
            if "a" == self.Mode: # 自由识别模式
                pass
            elif "b" == self.Mode: # 局部识别模式
                pass
        if 0 != len(BallLists) and 0 != len(FramesLists):
            if "c" == self.Mode: # 全局识别模式
                self.SelectOptimalBallFrame(BallLists, FramesLists, "red")

    def SelectOptimalBallFrame(self, BallLists, FramesLists, Threshold=5):
        self.DetectSign = False

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

        # 发布最优放球框
        self.OptimalMsg.data = self.OptimalFrame
        self.FramePublisher_.publish(self.OptimalMsg)

    def DetermineIfBallFrameIsOptimal(self, BallLists, Threshold):
        Frame = {'BallNum': 0, 'HighestBall': 'no'} 
        BallX = BallLists[0]['CentralPoint'][0] # 随便选择一个球的横坐标
        for Ball in BallLists:
            if (Ball['CentralPoint'][0] - BallX) <= Threshold:
                Frame['BallNum'] += 1
                Frame['HighestBall'] = Ball['Color']
        if Frame == Frame[int(self.Optimal)]:
            self.DropBallMsg.data = "y"
            self.FramePublisher_.publish(self.DropBallMsg)
        else:
            self.OptimalMsg.data = self.OptimalFrame
            self.FramePublisher_.publish(self.OptimalMsg)

    def SelectOptimalBall(self):
        pass

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = VisionDetecter("VisionDetecter", MainColor="red")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

