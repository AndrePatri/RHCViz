#!/usr/bin/env python
import rospy

import argparse

from rhcviz.utils.handshake import RHCVizHandshake
from rhcviz.utils.namings import NamingConventions

def publish_handshake(n_rhc_nodes: int, 
                robot_type: str):
    rospy.init_node('handshake_publisher')
    
    names = NamingConventions()

    basename = "RHCViz_test"

    handshake_topicname = names.handshake_topicname(basename=basename, 
                                            namespace=robot_type) 

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


