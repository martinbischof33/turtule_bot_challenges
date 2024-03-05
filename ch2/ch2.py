import signal

import rclpy  # ROS client library

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from enum import Enum, auto


class State(Enum):
    TO_THE_FIRST_WALL = auto()
    ROTATING = auto()
    TO_THE_SECOND_WALL = auto()
    STOP = auto()


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
            Twist,  # message type
            'cmd_vel',  # topic name
            1)  # history depth

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,  # function to run upon message arrival
            qos_profile_sensor_data)  # allows packet loss

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

        self.averageFrontDistance = [-1] * 5
        self.index_frontAvg = 0

        self.averageRightDistance = [-1] * 5
        self.index_rightAvg = 0

        self.minDistanceFront = [500] * 30
        self.minDistanceBack = [500] * 30
        self.minDistanceLeft = [500] * 30
        self.minDistanceRight = [500] * 30
        self.minDistanceIndex = 0

        self.ROBOT_WIDTH = 0.287
        self.TOLERANCE = 0.17
        self.MAX_LIN_VEL = 0.26  # m/s
        self.MAX_ANG_VEL = 1.82  # rad/s

        self.st = State.TO_THE_FIRST_WALL

        self.previousFront = 0

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = self.MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received"""

        match self.st:
            case State.TO_THE_FIRST_WALL:
                if self.move_forward(msg):
                    self.st = State.ROTATING
                    for i in range(3):
                        self.add_AverageFront(msg)
                    self.previousFront = self.get_AverageFront()
            case State.ROTATING:
                if self.rotate_counterclockwise(msg):
                    self.st = State.TO_THE_SECOND_WALL
            case State.TO_THE_SECOND_WALL:
                if self.move_forward(msg):
                    self.st = State.STOP
            case State.STOP:
                print("Hi")
        

    def move_forward(self, msg):
        MAX_INCREASE_PERCENTAGE = 5
        MAX_DECREASE_PERCENTAGE = 15
        self.add_AverageFront(msg)
        remaining_Distance = self.get_AverageFront() - self.ROBOT_WIDTH / 2

        speedPercentage = min(40, remaining_Distance / self.MAX_LIN_VEL * 100)

        is_increasing = speedPercentage > self.lin_vel_percent

        if is_increasing:
            speedPercentage = min(speedPercentage, self.lin_vel_percent + MAX_INCREASE_PERCENTAGE)
        else:
            speedPercentage = max(speedPercentage, self.lin_vel_percent - MAX_DECREASE_PERCENTAGE)

        self.vel(speedPercentage)

        if (remaining_Distance <= self.TOLERANCE):
            self.vel(0)
            return True

    def rotate_counterclockwise(self, msg):
        tolerance = 0
        self.add_AverageRight(msg)
        averageRightDistance = self.get_AverageRight()
        self.vel(0, 40)
        remainingDistance = averageRightDistance - self.previousFront
        if remainingDistance < tolerance:
            return True

    def add_AverageFront(self, msg):
        self.averageFrontDistance[self.index_frontAvg] = msg.ranges[0]
        if self.index_frontAvg == len(self.averageFrontDistance) - 1:
            self.index_frontAvg = 0
        else:
            self.index_frontAvg += 1

    def get_AverageFront(self):
        sublist = [value for value in self.averageFrontDistance if value > -1]
        return sum(sublist) / len(sublist)

    def add_AverageRight(self, msg):
        self.averageRightDistance[self.index_rightAvg] = msg.ranges[-90]
        if self.index_rightAvg == len(self.averageRightDistance) - 1:
            self.index_rightAvg = 0
        else:
            self.index_rightAvg += 1

    def get_AverageRight(self):
        sublist = [value for value in self.averageRightDistance if value > -1]
        return sum(sublist) / len(sublist)

    def getMinDistances(self, msg):
        if self.minDistanceIndex <= 29:
            self.minDistanceFront[self.minDistanceIndex] = msg.ranges[0]
            self.minDistanceBack[self.minDistanceIndex] = msg.ranges[180]
            self.minDistanceLeft[self.minDistanceIndex] = msg.ranges[90]
            self.minDistanceRight[self.minDistanceIndex] = msg.ranges[-90]
            self.minDistanceIndex += 1
        if self.minDistanceIndex == 30:
            print()
            print("Minimal Distances of the first 30 measurements: ")
            print("Min Distance Front: " + str(round(min(self.minDistanceFront), 3)))
            print("Min Distance Back : " + str(round(min(self.minDistanceBack), 3)))
            print("Min Distance Left : " + str(round(min(self.minDistanceLeft), 3)))
            print("Min Distance Right: " + str(round(min(self.minDistanceRight), 3)))
            self.minDistanceIndex += 1


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print("Waiting for messages...")

    def stop_robot(sig, frame):
        tb3.vel(0, 0)
        tb3.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT
    rclpy.spin(tb3)



if __name__ == '__main__':
    main()