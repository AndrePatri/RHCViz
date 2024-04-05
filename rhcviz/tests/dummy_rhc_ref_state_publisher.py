#!/usr/bin/env python

import rospy
import random
import argparse
from std_msgs.msg import Float64MultiArray
import numpy as np

from rhcviz.utils.namings import NamingConventions

def publish_rhc_refs_state(robot_type):
    rospy.init_node('rhc_refs_publisher')

    names = NamingConventions()
    
    topic_name_refs = names.rhc_refs_topicname(basename="RHCViz_test", namespace=robot_type)
    topic_name_high_lev_refs = names.hl_refs_topicname(basename="RHCViz_test", namespace=robot_type)

    pub = rospy.Publisher(topic_name_refs, Float64MultiArray, queue_size=10)
    pub_hl =  rospy.Publisher(topic_name_high_lev_refs, Float64MultiArray, queue_size=10)

    rate_value = 1 # Hz
    rate = rospy.Rate(rate_value)

    # rhc refs
    base_full = np.zeros(7 + 6)  # Null pose (3 pos + 4 quat + 6 twist)
    base_full[2] = 0.5
    quaternion = np.random.rand(4)
    quaternion /= np.linalg.norm(quaternion)
    base_full[3:7] = quaternion  # Ensure valid quaternion
    base_full[7:13] = np.random.uniform(-0.1, 0.1, size=6)
    
    # high lev refs
    base_full_hl = np.zeros(7 + 6)  # Null pose (3 pos + 4 quat + 6 twist)
    base_full_hl[0] = 0.5
    base_full_hl[2] = 0.3
    quaternion_hl = np.random.rand(4)
    quaternion_hl /= np.linalg.norm(quaternion_hl)
    base_full_hl[3:7] = quaternion_hl  # Ensure valid quaternion
    base_full_hl[7:13] = np.random.uniform(-0.1, 0.1, size=6)

    while not rospy.is_shutdown():
        msg_refs = Float64MultiArray(data=base_full.flatten())
        msg_refs_hl = Float64MultiArray(data=base_full_hl.flatten())

        pub.publish(msg_refs)
        pub_hl.publish(msg_refs_hl)

        rate.sleep()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Robot State Publisher")

    parser.add_argument('robot_type', 
                    choices=['aliengo', 'centauro'], 
                    help="Type of the robot ('aliengo' or 'centauro')")
    
    args = parser.parse_args()

    try:
        publish_rhc_refs_state(args.robot_type)
    except rospy.ROSInterruptException:
        pass


