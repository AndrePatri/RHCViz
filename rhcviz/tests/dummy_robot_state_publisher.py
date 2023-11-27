#!/usr/bin/env python
import rospy
import random
import argparse
from std_msgs.msg import Float64MultiArray
import numpy as np

def publish_robot_state(robot_type):
    rospy.init_node('robot_state_publisher')

    basename = "RHCViz_test"
    namespace = robot_type
    global_ns = f"{basename}_{namespace}"
    
    topic_name = f"/{global_ns}_robot_q"

    pub = rospy.Publisher(topic_name, Float64MultiArray, queue_size=10)
    
    rate_value = 1  # Hz
    rate = rospy.Rate(rate_value)

    # Set number of joints based on robot type
    if robot_type == "aliengo":
        n_joints = 12
    elif robot_type == "centauro":
        n_joints = 39

    while not rospy.is_shutdown():
        # Create a matrix with null base pose and random joint positions
        base_pose = np.zeros(7)  # Null pose (3 pos + 4 quat)
        base_pose[6] = 1  # Ensure valid quaternion
        joint_positions = np.random.uniform(-3.14, 3.14, (n_joints, 1))
        matrix = np.vstack((np.tile(base_pose, (1, 1)).T, joint_positions))

        # Publish the matrix
        msg = Float64MultiArray(data=matrix.flatten())
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Robot State Publisher")
    parser.add_argument('robot_type', choices=['aliengo', 'centauro'], help="Type of the robot ('aliengo' or 'centauro')")
    
    args = parser.parse_args()

    try:
        publish_robot_state(args.robot_type)
    except rospy.ROSInterruptException:
        pass
