#!/usr/bin/env python
import rospy
import random
import argparse
from std_msgs.msg import Float64MultiArray
import numpy as np

def publish_rhc_state(robot_type):
    rospy.init_node('rhc_state_publisher')

    topic_name = f"/{robot_type}_rhc_q"
    pub = rospy.Publisher(topic_name, Float64MultiArray, queue_size=10)
    
    rate_value = 1  # Hz
    rate = rospy.Rate(rate_value)

    # Set number of joints based on robot type
    if robot_type == "aliengo":
        n_joints = 12
    elif robot_type == "centauro":
        n_joints = 39
    n_nodes = 10  # Number of nodes

    while not rospy.is_shutdown():
        # Create a matrix with null base pose and random joint positions
        base_pose = np.zeros(7)  # Null pose (3 pos + 4 quat)
        base_pose[6] = 1  # Ensure valid quaternion
        joint_positions = np.random.uniform(-3.14, 3.14, (n_joints, n_nodes))
        matrix = np.vstack((np.tile(base_pose, (n_nodes, 1)).T, joint_positions))

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

