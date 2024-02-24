#!/usr/bin/env python

import rclpy
from std_msgs.msg import String
import argparse

from rhcviz.utils.handshake import RHCVizHandshake
from rhcviz.utils.namings import NamingConventions

from perf_sleep.pyperfsleep import PerfSleep

def publish_handshake(n_rhc_nodes: int, robot_type: str):
    rclpy.init()

    node = rclpy.create_node('handshake_publisher')

    names = NamingConventions()

    basename = "RHCViz_test"

    handshake_topicname = names.handshake_topicname(basename=basename, namespace=robot_type)

    # Define the rate of publishing handshake info (low rate)
    
    sleep_dt = 0.1
    perf_timer = PerfSleep()

    handshake = RHCVizHandshake(handshake_topic=handshake_topicname, 
                            node=node,
                            is_server=True)

    try:
        while rclpy.ok():

            # Set and publish the handshake information
            handshake.set_n_nodes(n_rhc_nodes)

            # node.get_logger().info(f"Publishing handshake data: n_robots = {n_rhc_nodes}")

            # Sleep for the rate duration
            perf_timer.thread_sleep(int((sleep_dt) * 1e+9)) 

    finally:
        # Cleanup when the node is shutting down
        handshake.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="RHC State Publisher")
    parser.add_argument('n_rhc_nodes', type=int, help="Number of RHC nodes", default=2)
    parser.add_argument('robot_type', choices=['aliengo', 'centauro'], help="Type of the robot ('aliengo' or 'centauro')")

    args = parser.parse_args()

    try:
        publish_handshake(n_rhc_nodes=args.n_rhc_nodes, robot_type=args.robot_type)
    except KeyboardInterrupt:
        pass



