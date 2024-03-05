import signal
import math
import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from transforms3d.euler import quat2euler

from enum import Enum, auto

class State(Enum):
    DRIVE_FORWARD_ONE = auto()
    DRIVE_FORWARD_TWO = auto()
    DRIVE_FORWARD_THREE = auto()
    DRIVE_FORWARD_FOUR = auto()
    ROTATING_ONE = auto()
    ROTATING_TWO = auto()
    ROTATING_THREE = auto()
    ROTATING_FOUR = auto()
    

class Tb3(Node):
    def __init__(self):
        super().__init__("tb3")

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 1  # message type  # topic name
        )  # history depth
        
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        
        self.target_position = None
        self.target_angle = None
        self.MAX_LIN_VEL = 0.26
        
        self.state = State.DRIVE_FORWARD_ONE

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """Publishes linear and angular velocities in percent"""
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent


    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.orientation = msg.pose.pose.orientation
        _, _, self.yaw = quat2euler([self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z])
        
        self.vel(0, 20)
        my_angle = 1.5707963
        my_distance = 0.15
        
        match self.state:
            case State.DRIVE_FORWARD_ONE:
                self.drive_along_axis(my_distance, 'y')
            case State.DRIVE_FORWARD_TWO:
                self.drive_along_axis(my_distance, 'x')
            case State.DRIVE_FORWARD_THREE:
                self.drive_along_axis(my_distance, 'y')
            case State.DRIVE_FORWARD_FOUR:
                self.drive_along_axis(my_distance, 'x')
            case State.ROTATING_ONE:
                self.rotate(my_angle)
            case State.ROTATING_TWO:
                self.rotate(my_angle)
            case State.ROTATING_THREE:
                self.rotate(my_angle)
            case State.ROTATING_FOUR:
                self.rotate(my_angle)
        
    def drive_along_axis(self, total_distance: int, axis: str):
        MAX_INCREASE_PERCENTAGE = 5
        MAX_DECREASE_PERCENTAGE = 15
        TOLERANCE = 0.001
        if axis == 'y':
            if self.target_position == None:
                self.target_position = (self.position[0], self.position[1] + total_distance)            
            remaining_distance = abs(self.position[1] - self.target_position[1])
                        
        elif axis == 'x':
            if self.target_position == None:
                self.target_position = (self.position[0] + total_distance, self.position[1])            
            remaining_distance = abs(self.position[0] - self.target_position[0])
            
        speedPercentage = min(40, remaining_distance / self.MAX_LIN_VEL * 100)

        if speedPercentage > self.lin_vel_percent:
            speedPercentage = min(speedPercentage, self.lin_vel_percent + MAX_INCREASE_PERCENTAGE)
        else:
            speedPercentage = max(speedPercentage, self.lin_vel_percent - MAX_DECREASE_PERCENTAGE)
        self.vel(speedPercentage)

        if (remaining_distance <= TOLERANCE):
            self.vel(0)
            match self.state:
                case State.DRIVE_FORWARD_ONE:
                    self.state = State.ROTATING_ONE
                case State.DRIVE_FORWARD_TWO:
                    self.state = State.ROTATING_TWO
                case State.DRIVE_FORWARD_THREE:
                    self.state = State.ROTATING_THREE
                case State.DRIVE_FORWARD_FOUR:
                    self.state = State.ROTATING_FOUR
            self.reset_environment()
            return True
            
            
    def rotate(self, total_angle):
        '''
        Args:
            total_angle (float): Angle in RAD
        '''
        #_, _, self.yaw = quat2euler([orientation.x, orientation.y, orientation.z, orientation.w])
        #target_yaw = self.normalize_angle(target_yaw + self.yaw)
        
        if self.target_angle == None:
            self.target_angle = (self.yaw - total_angle) % (2 * math.pi)
        print(f"{self.target_angle=}")   
        remaining_angle =   self.subtract_angles(self.target_angle, self.yaw)
        print(f"{remaining_angle=}")
        if abs(remaining_angle) < math.radians(2):
            self.vel(0, 0)
            match self.state:
                case State.ROTATING_ONE:
                    self.state = State.DRIVE_FORWARD_TWO
                case State.ROTATING_TWO:
                    self.state = State.DRIVE_FORWARD_THREE
                case State.ROTATING_THREE:
                    self.state = State.DRIVE_FORWARD_FOUR
                case State.ROTATING_FOUR:
                    self.state = State.DRIVE_FORWARD_ONE
            self.reset_environment()
            return True
        else:
            self.vel(0, -5)


    def subtract_angles(self, angle1, angle2):
        result = (angle1 - angle2) % (2 * math.pi)
        if result > math.pi:
            result -= 2 * math.pi
        elif result < -math.pi:
            result += 2 * math.pi
        return result

    def reset_environment(self):
        self.target_angle = None
        self.target_position = None


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


if __name__ == "__main__":
    main()
