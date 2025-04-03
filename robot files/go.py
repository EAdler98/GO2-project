import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import math


class SportModeTest:
    def __init__(self) -> None:
        # Time count
        self.t = 0
        self.dt = 0.01

        # Initial poition and yaw
        self.px0 = 0
        self.py0 = 0
        self.yaw0 = 0

        self.client = SportClient()  # Create a sport client
        self.client.SetTimeout(10.0)
        self.client.Init()

    def GetInitState(self, robot_state: SportModeState_):
        self.px0 = robot_state.position[0]
        self.py0 = robot_state.position[1]
        self.yaw0 = robot_state.imu_state.rpy[2]

    def StandUpDown(self):
        self.client.StandDown()
        print("Stand down !!!")
        time.sleep(1)

        self.client.StandUp()
        print("Stand up !!!")
        time.sleep(1)
        self.client.StandDown()
        print("Stand down !!!")
        time.sleep(1)

        self.client.Damp()

    def VelocityMove(self):
        elapsed_time = 1
        for i in range(int(elapsed_time / self.dt)):
            self.client.Move(0.3, 0, 0.3)  # vx, vy vyaw
            time.sleep(self.dt)
        self.client.StopMove()


    
    
    def BalanceAttitude(self):
        self.client.Euler(0.0, 0.0, 0.5)  # roll, pitch, yaw
        self.client.BalanceStand()


            
    def SpecialMotions(self):
        self.client.RecoveryStand()
        print("RecoveryStand !!!")
        time.sleep(1)
        
        self.client.Stretch()
        print("Sit !!!")
        time.sleep(1)  
        
        self.client.RecoveryStand()
        print("RecoveryStand !!!")
        time.sleep(1)
           
    def pre_go_meters_ahead(self, meters):
        itres_for_function = 0
        time_seg = 0.2
        iters = 100 * meters/ time_seg
        maxd=-10000
        mind=10000
        max_ditance_from_init = -10000
        # while time_for_function <iters:
        flag = True
        while True:
            itres_for_function +=1
            self.t += self.dt
            

            time_temp = self.t - time_seg
            if flag == True:
                print(time_temp)
                flag = False
            path = []
            for i in range(SPORT_PATH_POINT_SIZE):
                time_temp += time_seg
                

                
                # px_local =meters* math.sin(0.5 * time_temp)
                velocity = 0.75/meters
                px_local =meters* math.sin(velocity * time_temp)
                
                py_local = 0
                yaw_local = 0
                # vx_local = meters/2 * math.cos(0.5 * time_temp)
                vx_local = 0.25 * math.cos(0.5 * time_temp)

                vy_local = 0
                vyaw_local = 0
                


                path_point_tmp = PathPoint(0, 0, 0, 0, 0, 0, 0)

                path_point_tmp.timeFromStart = i * time_seg
                path_point_tmp.x = (
                    px_local * math.cos(self.yaw0)
                    - py_local * math.sin(self.yaw0)
                    + self.px0
                )
                path_point_tmp.y = (
                    px_local * math.sin(self.yaw0)
                    + py_local * math.cos(self.yaw0)
                    + self.py0
                )
                path_point_tmp.yaw = yaw_local + self.yaw0
                path_point_tmp.vx = vx_local * math.cos(self.yaw0) - vy_local * math.sin(
                    self.yaw0
                )
                path_point_tmp.vy = vx_local * math.sin(self.yaw0) + vy_local * math.cos(
                    self.yaw0
                )
                path_point_tmp.vyaw = vyaw_local
                

                if i == 1:
                    # print(">>")
                    if maxd < -(path_point_tmp.x-self.px0):
                        maxd = -(path_point_tmp.x-self.px0)
                    if mind > -(path_point_tmp.x-self.px0):
                        mind  = -(path_point_tmp.x-self.px0)
                
                    dis  = math.dist((self.px0, self.py0), 
                                 (path_point_tmp.x, path_point_tmp.y))
                    if abs(dis - meters) < 0.01:
                        return
                    # print("dis: ", dis)
                    max_ditance_from_init = max(dis, max_ditance_from_init)
                    # print("max_ditance_from_init:",max_ditance_from_init)

                path.append(path_point_tmp)

                self.client.TrajectoryFollow(path)

            time.sleep(self.dt)


            
    
    def location(self):
        while True:
            print(self.yaw0)
            self.GetInitState(robot_state)
            # if abs()
    
    def goMeteresAhead(self, meters):
        time.sleep(1)
        print("Lets go ",meters," meters ahead!")
        velocity = 0.5
        iters = 100 * meters /velocity
        
        
        while(iters > 0):
            self.client.Move(velocity, 0.0, 0.0)
            iters = iters - 1
            time.sleep(self.dt)
        self.client.StopMove()
        time.sleep(1)
        print("Finish moving")
        
    def rotate(self, alpha):
        time.sleep(1)
        print(f"Lets rotate {alpha} radii!")
        velocity = 0.6

        iters = abs (51 * alpha /velocity)
        if alpha < 0:
            velocity  = -velocity
            # iters = iters * 1.5      
        
        while(iters > 0):
            self.client.Move(0.0, 0.0, velocity)
            iters = iters - abs(velocity)
            time.sleep(self.dt*1.5)
        self.client.StopMove()
        print("Finish rotate")      # sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        time.sleep(1)
    # sub.Init(HighStateHandler, 10)      
        
# Robot state
robot_state = unitree_go_msg_dds__SportModeState_()
def HighStateHandler(msg: SportModeState_):
    global robot_state
    robot_state = msg



def init_sport():
    ChannelFactoryInitialize(0)        
    # sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    # sub.Init(HighStateHandler, 10)

def rotate_robot(alpha):
    robot_state = unitree_go_msg_dds__SportModeState_()
    ChannelFactoryInitialize(0)
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)

    test = SportModeTest()
    test.GetInitState(robot_state)
    test.rotate(alpha)

def go_ahead(distance):
    robot_state = unitree_go_msg_dds__SportModeState_()
    ChannelFactoryInitialize(0)
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)

    test = SportModeTest()
    test.GetInitState(robot_state)
    test.goMeteresAhead(distance)
    
    # time.sleep(1)
    # print("finish time in go ahead")
    # # ChannelFactoryInitialize(0)
    # # print("ChannelFactoryInitialize(0)")
        
    # sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    # sub.Init(HighStateHandler, 10)
    
    # time.sleep(1)
    # test = SportModeTest()
    # test.GetInitState(robot_state)
    # print("Start move ", distance)
    # test.goMeteresAhead(distance)
    
def init_robot():
    robot_state = unitree_go_msg_dds__SportModeState_()
    ChannelFactoryInitialize(0)
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)

    robot = SportModeTest()
    robot.GetInitState(robot_state)
    return robot

if __name__ == "__main__":
    # init_sport()
    
    # rotate_robot(-math.pi/2)
    ChannelFactoryInitialize(0)
        
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)

    test = SportModeTest()
    test.GetInitState(robot_state)
    test.rotate(math.pi/3)

    # print("Start test !!!")
    # # test.go_x_meters_ahead(0.6)
    # # test.location()
    # # while(True):
    # # test.VelocityMove()
    # test.goMeteresAhead(0.6*5)

    # ChannelFactoryInitialize(0)
        
    # sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    # sub.Init(HighStateHandler, 10)
    # time.sleep(1)
    # # init_sport()
    # test = SportModeTest()
    # test.GetInitState(robot_state)
    # print("Start test !!!")
    # test.go_x_meters_ahead(0.6)
    # test.location()
    # while(True):
    # test.VelocityMove()
    # test.goMeteresAhead2(0.6*5)
    # test.rotate( -1.0472 )
    # test.goMeteresAhead(0.6*5)
    # test.rotate( math.pi/2)
    
    # while True:
    #     test.t += test.dt

    #     # test.StandUpDown()
    #     #test.VelocityMove()
    #     # test.BalanceAttitude()
    #     test.TrajectoryFollow()
    #     # test.SpecialMotions()

    #     time.sleep(test.dt)
