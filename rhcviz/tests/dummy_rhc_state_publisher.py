#!/usr/bin/env python

import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from rhcviz.utils.handshake import RHCVizHandshake
from rhcviz.utils.namings import NamingConventions

from perf_sleep.pyperfsleep import PerfSleep

class RHCPublisher():

    def __init__(self, robot_type):

        self.perf_timer = PerfSleep()

        self.names = NamingConventions()
        self.basename = "RHCViz_test"

        name = self.names.global_ns(basename=self.basename, namespace=robot_type) + "RHCPublisher"

        self.node = rclpy.create_node(name)

        self.handshake_topicname = self.names.handshake_topicname(basename=self.basename, namespace=robot_type)
        self.topic_name = self.names.rhc_q_topicname(basename=self.basename, namespace=robot_type)

        self.handshaker = RHCVizHandshake(name=name,
                            node = self.node,
                            handshake_topic=self.handshake_topicname, 
                            is_server=False)

        self.publisher = self.node.create_publisher(Float64MultiArray, 
                        self.topic_name, 
                        10)
        self.sleep_dt = 0.1  # s
        # self.rate = self.create_rate(self.rate_value)

        # Set number of joints based on robot type
        if robot_type == "aliengo":
            self.n_joints = 12
        elif robot_type == "centauro":
            self.n_joints = 39

    def handshake(self):
        # Wait for handshake to complete

        while rclpy.ok() and not self.handshaker.handshake_done():
            
            rclpy.spin_once(self.node)

            self.perf_timer.thread_sleep(int((self.sleep_dt) * 1e+9)) 

        if self.handshaker.n_nodes is None:

            self.get_logger().error("Handshake not completed. Exiting.")

            rclpy.shutdown()

            return

        return self.handshaker.n_nodes

    def publish_rhc_state(self):
        
        x_offset = 1.2  # Increment along the x-axis for each node

        self.n_nodes = self.handshake()

        while rclpy.ok():
            # Create a matrix for base pose and random joint positions
            base_poses = []

            for i in range(self.n_nodes):
                base_pose = np.zeros(7)  # Null pose (3 pos + 4 quat)
                base_pose[6] = 1  # Ensure valid quaternion
                base_pose[0] = i * x_offset  # Increment x position for each node
                base_poses.append(base_pose)

            # Stack all base poses and joint positions to form the matrix
            joint_positions = np.random.uniform(-3.14, 3.14, (self.n_joints, self.n_nodes))
            matrix = np.vstack((np.array(base_poses).T, joint_positions))

            # Publish the matrix
            msg = Float64MultiArray(data=matrix.flatten())

            self.publisher.publish(msg)
            rclpy.spin_once(self.node)
            self.perf_timer.thread_sleep(int((self.sleep_dt) * 1e+9)) 

def main(args=None):

    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="RHC State Publisher")
    parser.add_argument('robot_type', choices=['aliengo', 'centauro'], help="Type of the robot ('aliengo' or 'centauro')")
    args = parser.parse_args()

    rhc_publisher = RHCPublisher(args.robot_type)

    try:
        
        rhc_publisher.publish_rhc_state()

    except KeyboardInterrupt:

        pass

    finally:

        rclpy.shutdown()

if __name__ == '__main__':
    main()


