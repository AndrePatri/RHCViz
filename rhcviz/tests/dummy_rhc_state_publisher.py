#!/usr/bin/env python
import rospy
import random
import argparse
from std_msgs.msg import Float64MultiArray
import numpy as np

from rhcviz.utils.handshake import RHCVizHandshake
from rhcviz.utils.namings import NamingConventions

def handshake(handshaker: RHCVizHandshake):
        
    # Wait for handshake to complete
    while not rospy.is_shutdown() and not handshaker.handshake_done():

        rospy.sleep(0.1)

    if handshaker.n_nodes is None:

        rospy.logerr("Handshake not completed. Exiting.")

        return
    
    return handshaker.n_nodes
    
def publish_rhc_state(robot_type):
    rospy.init_node('rhc_state_publisher')
    
    names = NamingConventions()

    basename = "RHCViz_test"
    handshake_topicname = names.handshake_topicname(basename=basename, 
                                        namespace=robot_type)

    topic_name = names.rhc_q_topicname(basename=basename, 
                                    namespace=robot_type)
    
    pub = rospy.Publisher(topic_name, 
                        Float64MultiArray, 
                        queue_size=10)
    
    rate_value = 0.1  # Hz
    rate = rospy.Rate(rate_value)

    # Set number of joints based on robot type
    if robot_type == "aliengo":
        n_joints = 12
    elif robot_type == "centauro":
        n_joints = 39

    handshaker = RHCVizHandshake(handshake_topicname, 
                            is_server=False)
    
    n_nodes = handshake(handshaker)
    x_offset = 1.2  # Increment along the x-axis for each node

    while not rospy.is_shutdown():
        # Create a matrix for base pose and random joint positions
        base_poses = []

        for i in range(n_nodes):
            base_pose = np.zeros(7)  # Null pose (3 pos + 4 quat)
            base_pose[6] = 1  # Ensure valid quaternion
            base_pose[0] = i * x_offset  # Increment x position for each node
            base_poses.append(base_pose)

        # Stack all base poses and joint positions to form the matrix
        joint_positions = np.random.uniform(-3.14, 3.14, (n_joints, n_nodes))
        matrix = np.vstack((np.array(base_poses).T, joint_positions))

        # Publish the matrix
        msg = Float64MultiArray(data=matrix.flatten())
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="RHC State Publisher")
    parser.add_argument('robot_type', choices=['aliengo', 'centauro'], help="Type of the robot ('aliengo' or 'centauro')")
    
    args = parser.parse_args()

    try:
        publish_rhc_state(args.robot_type)
    except rospy.ROSInterruptException:
        pass

