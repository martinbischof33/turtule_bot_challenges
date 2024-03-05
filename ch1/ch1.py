import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,
                'cmd_vel',
                1)

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,
                qos_profile_sensor_data)

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

        self.MAX_LIN_VEL_M_S = 0.26
        self.MAX_ANG_VEL_RAD_S = 1.82
        self.TB_WIDTH_METERS = 0.287
        self.acc = 4
        self.vel = 0

        self.num_prev_values = 10
        self.current_warmup_round = 0
        self.prev_ranges = np.zeros(self.num_prev_values)
        self.prev_index = 0

    def vel_in_percent(self, lin_vel_percent, ang_vel_percent=0):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.MAX_LIN_VEL_M_S * lin_vel_percent / 100
        cmd_vel_msg.angular.z = self.MAX_ANG_VEL_RAD_S * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        if msg.ranges[0] == np.inf:
            return
        if self.current_warmup_round < self.num_prev_values:
            self.current_warmup_round += 1
            r= self.smooth_forward(msg.ranges[0])
            print("Warmup", r)
            return

        schnautze = self.smooth_forward(msg.ranges[0]) - (self.TB_WIDTH_METERS / 2)
        print(schnautze)
        if schnautze <= 0.2:
            self.vel -= (self.acc*10)
            self.vel = max(0, self.vel)
        else:
            self.vel += self.acc
            self.vel = min(100, self.vel)
        self.vel_in_percent(self.vel)

    def smooth_forward(self, sensor_forward_value):
        self.prev_ranges[self.prev_index] = sensor_forward_value
        self.prev_index = (self.prev_index + 1) % self.num_prev_values
        moving_average = np.mean(self.prev_ranges)
        return moving_average

def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

