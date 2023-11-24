#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Float64MultiArray
import numpy as np

def publish_joint_positions():
    rospy.init_node('joint_position_publisher')

    robot_name = "aliengo"  # Replace with actual robot name
    topic_name = f"/{robot_name}_q"
    pub = rospy.Publisher(topic_name, Float64MultiArray, queue_size=10)
    
    rate_value = 10  # Hz
    rate = rospy.Rate(rate_value)

    n_joints = 12  # Number of joints, adjust as necessary
    n_nodes = 3  # Number of nodes

    while not rospy.is_shutdown():
        # Create a matrix with null base pose and random joint positions
        base_pose = np.zeros(7)  # Null pose (3 pos + 4 quat)
        base_pose[6] = 1
        joint_positions = np.random.uniform(-3.14, 3.14, (n_joints, n_nodes))
        matrix = np.vstack((np.tile(base_pose, (n_nodes, 1)).T, joint_positions))

        # Publish the matrix
        msg = Float64MultiArray(data=matrix.flatten())
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_positions()
    except rospy.ROSInterruptException:
        pass
