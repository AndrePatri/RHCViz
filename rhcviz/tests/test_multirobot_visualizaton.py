#!/usr/bin/env python
import rospy
import random
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf2_ros

import subprocess
import argparse

def read_urdf_file(urdf_file_path):
    """
    Read the URDF file from the given path and return its content as a string.
    """
    with open(urdf_file_path, 'r') as file:
        return file.read()

def get_joint_info(urdf_robot):
    """
    Parse the URDF model and extract joint names and their state dimensions.
    Returns a list of joint names and a corresponding list of state dimensions (1 for revolute and prismatic, 0 for fixed).
    """
    joint_names = []
    state_dimensions = []

    for joint in urdf_robot.joints:
        if joint.type not in ['fixed', 'floating', 'planar']:
            joint_names.append(joint.name)
            state_dimensions.append(1 if joint.type in ['revolute', 'prismatic'] else 0)

    return joint_names, state_dimensions

def generate_random_joint_positions(joint_names):
    """
    Generate random joint positions. Here we assume each joint can move between -pi and pi.
    This can be adjusted based on the actual joint limits of the robot.
    """
    return [random.uniform(-3.14, 3.14) for _ in joint_names]

def start_robot_state_publisher(urdf, robot_ns):
    """
    Start a robot_state_publisher for a robot namespace.
    """
    # Set the robot description for each namespace
    full_param_name = '/{}/robot_description'.format(robot_ns)
    rospy.set_param(full_param_name, urdf)

    # Launch the robot_state_publisher for each robot
    rsp_command = [
        'rosrun', 'robot_state_publisher', 'robot_state_publisher',
        '__ns:=' + robot_ns,
        '_tf_prefix:=' + robot_ns
    ]
    rsp_process = subprocess.Popen(rsp_command)
    return rsp_process

def publish_static_transforms_for_robots(n_robots):
    """
    Publish static transforms from world to base_link for each robot.
    """
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    for i in range(n_robots):
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = 'world'
        static_transform_stamped.child_frame_id = 'robot_{}/base_link'.format(i)
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        static_broadcaster.sendTransform(static_transform_stamped)

def publish_robot_state(publishers, pose, joint_names, joint_positions, ns):
    """
    Publish a single robot state to a given namespace.
    """
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = joint_names
    js.position = joint_positions

    # Publish joint states to the robot's namespace
    publishers[ns].publish(js)

def launch_rviz(rviz_config_path=None):
    rviz_command = ['rviz']
    if rviz_config_path is not None:
        rviz_command += ['-d', rviz_config_path]
    subprocess.Popen(rviz_command)

def main(urdf_file_path, n_robots, rviz_config_path=None):

    rospy.init_node('RHCViz', anonymous=True)

    robot_description = read_urdf_file(urdf_file_path)
    joint_names, _ = get_joint_info(URDF.from_xml_string(robot_description))

    # Launch RViz in a separate process
    launch_rviz(rviz_config_path)

    # Start a robot_state_publisher for each robot namespace
    rsp_processes = []
    for i in range(n_robots):
        ns = 'robot_{}'.format(i)
        rsp_processes.append(start_robot_state_publisher(robot_description, ns))

    # Publish static transforms for each robot
    publish_static_transforms_for_robots(n_robots)

    # Publishers for each robot's joint states
    publishers = {}
    for i in range(n_robots):
        ns = 'robot_{}'.format(i)
        publishers[ns] = rospy.Publisher('/{}/joint_states'.format(ns), JointState, queue_size=10)

    # Give time for the robot_state_publishers to start
    rospy.sleep(2)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        for i in range(n_robots):
            ns = 'robot_{}'.format(i)
            publishers[ns].publish(JointState(
                header=rospy.Header(stamp=rospy.Time.now()),
                name=joint_names,
                position=generate_random_joint_positions(joint_names),
                velocity=[0]*len(joint_names),
                effort=[0]*len(joint_names)
            ))
            # Publish static transforms for each robot
            publish_static_transforms_for_robots(n_robots)
        rate.sleep()

    # Cleanup robot_state_publisher processes
    for rsp_process in rsp_processes:
        rsp_process.terminate()
    rviz_process.terminate()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('urdf_file_path', type=str, help="Path to the URDF file")
    parser.add_argument('n_robots', type=int, help="Number of robots")
    parser.add_argument('--rviz_config', type=str, help="Path to the RViz configuration file", default=None)

    args = parser.parse_args()

    main(args.urdf_file_path, args.n_robots, args.rviz_config)
