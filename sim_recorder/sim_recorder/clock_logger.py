from operator import le
import rclpy
import sys
import os
from rclpy.node import Node
import signal
from rosgraph_msgs.msg import Clock
from rclpy.time import Time


class ClockLogger(Node):

    def __init__(self, simulator, path, idx):
        super().__init__('clock_logger')
        self.subscription = self.create_subscription(
            Clock,
            'clock',
            self.listener_callback,
            10)
        self.get_logger().info("Clock logger initialized")
        self.realtime = []
        self.simtime = []
        self.sim_start = None
        self.ros_start = None
        self.sim_name = simulator
        self.idx = idx
        self.path = path

    def listener_callback(self, msg: Clock):
        stamp = self.get_clock().now()
        stamp = stamp.nanoseconds
        self.realtime.append(str(stamp))

        stamp = Time.from_msg(msg.clock)
        stamp = stamp.nanoseconds
        self.simtime.append(str(stamp))

    def dump_values(self):
        print("Dumping values")
        with open(self.path + f"/{self.sim_name}/clock/clock_{self.idx}.csv", "w") as f:
            f.write(",".join(self.realtime) + "\n")
            f.write(",".join(self.simtime) + "\n")
        sys.exit(0)


def run(simulator, path, idx=0):
    rclpy.init(args=None)
    clock_logger = ClockLogger(simulator, path, idx)
    signal.signal(signal.SIGINT, lambda sig, frame: clock_logger.dump_values())
    signal.signal(signal.SIGTERM, lambda sig, frame: clock_logger.dump_values())

    rclpy.spin(clock_logger)
    rclpy.shutdown()
