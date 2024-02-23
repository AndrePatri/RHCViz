#!/usr/bin/env python

import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
import numpy as np

from rhcviz.utils.namings import NamingConventions

from perf_sleep.pyperfsleep import PerfSleep

class RobotStatePublisher():

    def __init__(self, robot_type):

        self.perf_timer = PerfSleep()

        self.names = NamingConventions()
        self.basename = "RHCViz_test"

        name = self.names.global_ns(basename=self.basename, namespace=robot_type) + "RobotStatePublisher"

        self.node = rclpy.create_node(name)

        self.topic_name = self.names.robot_q_topicname(basename=self.basename, 
                                namespace=robot_type)

        self._qos_settings = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, # BEST_EFFORT
                durability=DurabilityPolicy.TRANSIENT_LOCAL, # VOLATILE
                history=HistoryPolicy.KEEP_LAST, # KEEP_ALL
                depth=10,  # Number of samples to keep if KEEP_LAST is used
                liveliness=LivelinessPolicy.AUTOMATIC,
                # deadline=1000000000,  # [ns]
                # partition='my_partition' # useful to isolate communications
                )
        self.publisher = self.node.create_publisher(Float64MultiArray, 
                            self.topic_name, 
                            qos_profile=self._qos_settings)
        
        self.sleep_dt = 0.1  # s
        # self.rate = self.create_rate(self.rate_value)

        # Set number of joints based on robot type
        if robot_type == "aliengo":
            self.n_joints = 12
        elif robot_type == "centauro":
            self.n_joints = 39

    def publish_robot_state(self):

        while rclpy.ok():
            
            # Create a matrix with null base pose and random joint positions
            base_pose = np.zeros(7)  # Null pose (3 pos + 4 quat)
            base_pose[6] = 1  # Ensure valid quaternion
            joint_positions = np.random.uniform(-3.14, 3.14, (self.n_joints, 1))
            matrix = np.vstack((np.tile(base_pose, (1, 1)).T, joint_positions))

            # Publish the matrix
            msg = Float64MultiArray(data=matrix.flatten())

            self.publisher.publish(msg)
            # rclpy.spin_once(self.node)
            self.perf_timer.thread_sleep(int((self.sleep_dt) * 1e+9)) 

def main(args=None):

    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Robot State Publisher")
    parser.add_argument('robot_type', choices=['aliengo', 'centauro'], help="Type of the robot ('aliengo' or 'centauro')")
    args = parser.parse_args()

    robot_state_publisher = RobotStatePublisher(args.robot_type)

    try:

        robot_state_publisher.publish_robot_state()

    except KeyboardInterrupt:

        pass

    finally:

        rclpy.shutdown()

if __name__ == '__main__':

    main()

