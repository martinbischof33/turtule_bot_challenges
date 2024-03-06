import signal
import math
import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from transforms3d.euler import quat2euler

from c4_graph_node import GraphNode

from enum import Enum, auto

class State(Enum):
    FIND_WALLS = auto()
    DECIDE = auto()
    STOP = auto()
    ROTATE = auto()
    DRIVE = auto()
    

class Tb3(Node):
    def __init__(self):
        super().__init__("tb3")

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 1  # message type  # topic name
        )  # history depth
        
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.ang_vel_percent = 0
        self.last_speed_percentage = 0
        
        self.target_position = None
        self.target_angle = None
        self.MAX_LIN_VEL = 0.26
        
        self.state = State.FIND_WALLS
        self.transformation = None
        
        self.last_scans = []
        self.number_of_last_scans = 10
        self.target_direction = 0
        
        self.rotation_for_direction = [0, 90, 180, -90]
        self.nodge = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        
        self.graph = GraphNode(0, 0)
        self.rotation_shift = 0

    def scan_callback(self, msg):
        if self.state == State.FIND_WALLS:
            if len(self.last_scans) < self.number_of_last_scans:
                self.msg_to_last_scans(msg)
            else:
                four_distances = self.get_average_of_direction()
                self.no_walls = []
                for i, direction in enumerate(four_distances):
                    if direction >= 1:
                        self.no_walls.append(i)
                        
                print(self.no_walls)
                neighbours = self.generate_neighbours()
                
                self.graph.add_neighbours(neighbours)
                self.graph.is_explored = True
                self.state = State.DECIDE
                self.last_scans = []
        pass
        
    def odom_callback(self, msg):       
        
        if self.transformation == None:
            self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            origin_position_in_maze = (0.5, 0.5)
            self.transformation = (origin_position_in_maze[0] - self.position[0], origin_position_in_maze[1] - self.position[1])
        self.position = self.translate_to_maze((msg.pose.pose.position.x, msg.pose.pose.position.y))     
        
        self.orientation = msg.pose.pose.orientation    
        _, _, self.yaw = quat2euler([self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z])
        
        match self.state:
            case State.DECIDE:
                self.decide()
            case State.ROTATE:
                self.rotate()
            case State.DRIVE:
                self.drive_to_next_tile()
        
        print(f"\n\n{self.state}")
    
    def decide(self):
        print(self.graph)
        # find direction
        self.target_direction = self.no_walls.pop()
        # react to decision:
        self.state = State.ROTATE
        
    def generate_neighbours(self):
        neighbours = []
        for i in self.no_walls:
            index = (i-self.rotation_shift)%4
            x = self.graph.x + self.nodge[index][0]
            y = self.graph.y + self.nodge[index][1]
            new_node = GraphNode(x, y)
            neighbours.append(new_node)
        return neighbours
        
    
    def msg_to_last_scans(self, msg):
        n = len(msg.ranges)
        current_scan = [
            msg.ranges[0],
            msg.ranges[-n//4], 
            msg.ranges[n//2],
            msg.ranges[n//4]
        ]
        self.last_scans.append(current_scan)
        self.last_scans = self.last_scans[-self.number_of_last_scans:]
    
    def get_average_of_direction(self):
        return [
            sum(x[0] for x in self.last_scans) / self.number_of_last_scans,
            sum(x[1] for x in self.last_scans) / self.number_of_last_scans,
            sum(x[2] for x in self.last_scans) / self.number_of_last_scans,
            sum(x[3] for x in self.last_scans) / self.number_of_last_scans,
        ]
       
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
            
    def drive_to_next_tile(self):
        total_distance = 1
        MAX_INCREASE_PERCENTAGE = 5
        MAX_DECREASE_PERCENTAGE = 15
        TOLERANCE = 0.01
        
        if self.target_position == None:
            target_x = self.position[0] + total_distance * math.cos(self.yaw)
            target_y  = self.position[1] + total_distance * math.sin(self.yaw)
            self.target_position = (target_x, target_y)
            pass
        
        dx = self.target_position[0] - self.position[0]
        dy = self.target_position[1] - self.position[1]
        remaining_distance = math.sqrt(dx**2 + dy**2)
            
        speedPercentage = min(40, remaining_distance / self.MAX_LIN_VEL * 100)
        print(f"{speedPercentage=}\n{self.last_speed_percentage=}\n{remaining_distance=}")
        if speedPercentage > self.last_speed_percentage:
            speedPercentage = min(speedPercentage, self.last_speed_percentage + MAX_INCREASE_PERCENTAGE)
        else:
            speedPercentage = max(speedPercentage, self.last_speed_percentage - MAX_DECREASE_PERCENTAGE)
        print(speedPercentage)
        #self.lin_vel_percent = speedPercentage
        self.last_speed_percentage = speedPercentage
        self.vel(speedPercentage, 0)

        if (remaining_distance <= TOLERANCE):
            self.vel(0)
            self.state = State.FIND_WALLS
            self.reset_environment()
            return True
            
    def rotate(self):
        '''
        Args:
            total_angle (float): Angle in RAD
        '''
        total_angle = self.rotation_for_direction[self.target_direction]
        self.rotation_shift = (self.rotation_shift + self.target_direction) % 4
        turning_speed = -10
        if total_angle < 0:
            turning_speed = 10
        
        total_angle = math.radians(total_angle)
        if self.target_angle == None:
            self.target_angle = (self.yaw - total_angle) % (2 * math.pi)
        print(f"{self.target_angle=}")   
        remaining_angle =   self.subtract_angles(self.target_angle, self.yaw)
        print(f"{remaining_angle=}")
        if abs(remaining_angle) < math.radians(2):
            self.vel(0, 0)
            self.state = State.DRIVE
            self.reset_environment()
            return True
        else:
            self.vel(0, turning_speed)

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
