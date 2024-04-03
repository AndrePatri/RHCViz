#!/usr/bin/env python

import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.qos import QoSProfile
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

        self.topic_name_refs = self.names.rhc_refs_topicname(basename=self.basename, namespace=robot_type)

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
                        self.topic_name_refs, 
                        qos_profile=self._qos_settings)

        self.sleep_dt = 0.1  # s
        # self.rate = self.create_rate(self.rate_value)

    def publish_rhc_refs(self):
        
        base_pose = np.zeros(7 + 6)  # Null pose (3 pos + 4 quat + 6 twist)
        base_pose[2] = 0.5
        quaternion = np.random.rand(4)
        quaternion /= np.linalg.norm(quaternion)
        base_pose[3:7] = quaternion  # Ensure valid quaternion
        base_pose[7:13] = np.random.uniform(-0.1, 0.1, size=6)
        
        while rclpy.ok():
            # Create a matrix for base pose and random joint positions
            
            msg_refs = Float64MultiArray(data=base_pose.flatten())

            self.publisher.publish(msg_refs)

            self.perf_timer.thread_sleep(int((self.sleep_dt) * 1e+9)) 

def main(args=None):

    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="RHC State Publisher")
    parser.add_argument('robot_type', choices=['aliengo', 'centauro'], help="Type of the robot ('aliengo' or 'centauro')")
    args = parser.parse_args()

    rhc_publisher = RHCPublisher(args.robot_type)

    try:
        
        rhc_publisher.publish_rhc_refs()

    except KeyboardInterrupt:

        pass

    finally:

        rclpy.shutdown()

if __name__ == '__main__':
    main()

