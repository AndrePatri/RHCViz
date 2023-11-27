#!/usr/bin/env python
import rospy
import random
import argparse
from std_msgs.msg import Float64MultiArray

import numpy as np

from rhcviz.utils.handshake import RHCVizHandshake

def publish_handshake(n_rhc_nodes: int, 
                robot_type: str):
    rospy.init_node('handshake_publisher')
    
    basename = "RHCViz_test"
    global_ns = f"{basename}_{robot_type}"
    handshake_basename = "HandShake"

    handshake_topicname = f"/{global_ns}_{handshake_basename}"

    # Define the rate of publishing handshake info (low rate)
    handshake_rate = rospy.Rate(0.5) 

    handshake = RHCVizHandshake(handshake_topicname, is_server=True)
    
    while not rospy.is_shutdown():
        # Set and publish the handshake information
        handshake.set_n_nodes(n_rhc_nodes)
        
        rospy.loginfo(f"Publishing handshake data: n_robots = {n_rhc_nodes}")

        # Sleep for the rate duration
        handshake_rate.sleep()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="RHC State Publisher")
    parser.add_argument('n_rhc_nodes', help="Number of RHC nodes")
    parser.add_argument('robot_type', choices=['aliengo', 'centauro'], help="Type of the robot ('aliengo' or 'centauro')")

    args = parser.parse_args()

    try:

        publish_handshake(n_rhc_nodes=args.n_rhc_nodes, 
                        robot_type=args.robot_type)

    except rospy.ROSInterruptException:

        pass


