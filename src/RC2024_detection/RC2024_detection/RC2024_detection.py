#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import String, Bool, Int16MultiArray
from chassis_info_interfaces.msg import ChassisInfo
import math

class VisionDetecter(Node):
    def __init__(self, name, MainColor):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        self.MainColor = MainColor
        self.ActionPoint = [400, 400]
        self.O_DistanceThreshold = 20
        self.Mode = "a"
        self.PreResult = False
        self.Result = False
        self.OptimalFrame = ""
        self.DetectSign = False
        self.TinyAdjustSign = False
        self.DropBallMsg = Bool()
        self.PickBallMsg = Bool()
        self.OptimalMsg = String()
        self.DistanceXY = Int16MultiArray()
        self.Frames = ({'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},
                        {'BallNum': 0, 'HighestBall': 'no'},)
        self.VisionSubscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.VisionCallback, 10)
        # TODO: create a artifical message that includes mode(String). tiny_adjust(Bool). result(Bool) -> finish
        # TODO: import the artifical message type. -> finish
        self.ChassisInfoSubscribe_ = self.create_subscription(ChassisInfo, "chassis_info", self.ChassisInfoCallback_, 10)

        self.FramePublisher_ = self.create_publisher(String, "optimal_frame", 10)
        self.DropBallPublisher_ = self.create_publisher(Bool, "drop_ball", 10)
        self.PickBallPublisher_ = self.create_publisher(Bool, "pick_ball", 10)
        # TODO: define the artifical message that be named as "List". -> finish
        self.DistanceXYPublisher_ = self.create_publisher(Int16MultiArray, "distance_xy", 10)

    def ChassisInfoCallback_(self, msg):
        self.TinyAdjustSign = msg.tiny_adjust
        self.Result = msg.result
        self.Mode = msg.data
        if self.PreResult != self.Result:
            self.PreResult = self.Result
        if self.PreResult == False and self.Result == True:
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
                if msg.targets[i].type != "Frame": # 去除球筐的识别
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
                self.SelectOptimalBall(BallLists)    
            elif "b" == self.Mode: # 局部识别模式
                self.DetermineIfBallFrameIsOptimal()
        if 0 != len(BallLists) and 0 != len(FramesLists):
            if "c" == self.Mode: # 全局识别模式
                self.SelectOptimalBallFrame(BallLists, FramesLists)

    def SelectOptimalBallFrame(self, BallLists, FramesLists, Threshold=5):
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

        self.DetectSign = False

    def DetermineIfBallFrameIsOptimal(self, BallLists, Threshold):
        # 微调 CAR 位置以便可以成功放球
        if not self.TinyAdjustSign:
            # 根据球筐口的中心点进行微调

            return

        Frame = {'BallNum': 0, 'HighestBall': 'no'} 
        BallX = BallLists[0]['CentralPoint'][0] # 随便选择一个球的横坐标
        for Ball in BallLists:
            if (Ball['CentralPoint'][0] - BallX) <= Threshold:
                Frame['BallNum'] += 1
                Frame['HighestBall'] = Ball['Color']
        if Frame == Frame[int(self.Optimal)]:
            self.DropBallMsg.data = "y" # 发布放球命令
            self.FramePublisher_.publish(self.DropBallMsg) # TODO
        else:
            self.OptimalMsg.data = self.OptimalFrame
            self.FramePublisher_.publish(self.OptimalMsg)

    def SelectOptimalBall(self, BallLists):
        """ By this function, select the optimal ball and pulish distance of X Y. """
        # 得到每个目标球的中心坐标
        BestBallPoint = []
        MinIndex = 0
        MinDistance = self.Calculate_O_Distance(BallLists[0]['CentralPoint'])
        for Index, Ball in enumerate(BallLists):
            if Ball['Color'] == self.MainColor:
                if self.Calculate_O_Distance(Ball['CentralPoint']) < MinDistance:
                    MinIndex = Index
                    MinDistance = self.Calculate_O_Distance(Ball['CentralPoint'])
        print(BallLists[MinIndex]['CentralPoint'][0] - self.ActionPoint[0])
        print(BallLists[MinIndex]['CentralPoint'][1] - self.ActionPoint[1])
        data = []
        data.append(BallLists[MinIndex]['CentralPoint'][0] - self.ActionPoint[0])
        data.append(BallLists[MinIndex]['CentralPoint'][1] - self.ActionPoint[1])
        self.DistanceXY.data = data
        self.DistanceXYPublisher_.publish(self.DistanceXY)

        if self.Calculate_O_Distance(BallLists[MinIndex]['CentralPoint']) < self.O_DistanceThreshold:
            self.PickBallMsg.data = True
            self.PickBallPublisher_.publish(self.PickBallMsg)

    def Calculate_O_Distance(self, CentralPoint):
        """ Calculate O distance. """
        return math.sqrt((self.ActionPoint[0] - CentralPoint[0])**2 + (self.ActionPoint[1] - CentralPoint[1])**2)
    
def main(args=None):
    rclpy.init(args=args) # Initial rclpy
    VisionNode = VisionDetecter("VisionDetecter", MainColor="tv") 
    rclpy.spin(VisionNode) # keep the node ongoing, and detect the "ctrl c"
    rclpy.shutdown() #  rclpy

