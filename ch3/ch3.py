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
    ROTATING_ONE = auto()
    STOP = auto()
    
class DriveState(Enum):
    ACC = auto()
    DEACC = auto()
    STOP = auto()
    
    
class Tb3(Node):
    def __init__(self):
        super().__init__("tb3")

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 1  # message type  # topic name
        )  # history depth
        
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.ang_vel_percent = 0
        self.last_speed_percentage = 0
        self.lin_vel_percent = 0
        
        self.target_position = None
        self.target_angle = None
        self.MAX_LIN_VEL = 0.26
        
        self.state = State.DRIVE_FORWARD_ONE
        self.transformation = None
        
        self.drive_state = DriveState.STOP
        self.deacc_threshold = 0.1

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
        self.last_speed_percentage = lin_vel_percent


    def odom_callback(self, msg):        
        
        
        if self.transformation == None:
            self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            origin_position_in_maze = (0.5, 0.5)
            self.transformation = (origin_position_in_maze[0] - self.position[0], origin_position_in_maze[1] - self.position[1])
        self.position = self.translate_to_maze((msg.pose.pose.position.x, msg.pose.pose.position.y))     
        
        self.orientation = msg.pose.pose.orientation    
        _, _, self.yaw = quat2euler([self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z])
        
        self.vel(0, 20)
        my_angle = math.radians(90)
        my_distance = 0.3
        print(f"{self.state=}\n{self.drive_state=}\n{self.last_speed_percentage=}\n")
        match self.state:
            case State.DRIVE_FORWARD_ONE:
                self.drive_along_axis(my_distance, 'y')
            case State.DRIVE_FORWARD_TWO:
                self.drive_along_axis(my_distance, 'x')
            case State.ROTATING_ONE:
                self.rotate(my_angle)
            case State.STOP:
                self.vel(0,0)
                print('FINISHED')
    
    
    def drive(self, error):
        ACC = 5
        DEACC = 10
        TOLERANCE = 0.01
        VEL_MAX = 50
        VEL_MIN = 10
        
        # tolaranz guad, breche ab wenn alles fertig 
        if (error <= TOLERANCE):
            self.vel(0)
            self.drive_state = DriveState.STOP
            return True
        if self.deacc_threshold >= error:
            self.drive_state = DriveState.DEACC
        else:
            self.drive_state = DriveState.ACC
        match self.drive_state:
            case DriveState.STOP:
                return True
            case DriveState.ACC:
                self.lin_vel_percent += ACC
                self.lin_vel_percent = min(VEL_MAX, self.lin_vel_percent)  # cap to max speed
            case DriveState.DEACC:
                self.lin_vel_percent -= DEACC
                self.lin_vel_percent = max(VEL_MIN, self.lin_vel_percent)  # cap to min speed   
        
            
        self.vel(self.lin_vel_percent)
        return False
    
        
    def drive_along_axis(self, total_distance: int, axis: str):
        if axis == 'y':
            if self.target_position == None:
                self.target_position = (self.position[0], self.position[1] + total_distance)            
            remaining_distance = abs(self.position[1] - self.target_position[1])
                        
        elif axis == 'x':
            if self.target_position == None:
                self.target_position = (self.position[0] + total_distance, self.position[1])            
            remaining_distance = abs(self.position[0] - self.target_position[0])
        
        self.drive(remaining_distance)
        if (self.drive(remaining_distance)):
            match self.state:
                case State.DRIVE_FORWARD_ONE:
                    self.state = State.ROTATING_ONE
                case State.DRIVE_FORWARD_TWO:
                    self.state = State.STOP
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
            self.vel(0, -7)

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
        self.lin_vel_percent = 0
        self.vel(0,0)
            
    def translate_to_maze(self, point: tuple):
        '''
        This method translates the meassured value into the maze frame.
        '''
        return (point[0] + self.transformation[0], point[1] + self.transformation[1])

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
