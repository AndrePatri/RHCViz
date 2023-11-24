#!/usr/bin/env python
import rospy
import random
import subprocess
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import TransformStamped
import tf2_ros

import yaml
import os
import tempfile

from rhcviz.utils.sys_utils import PathsGetter

# import time

class RHCViz:

    def __init__(self, 
            urdf_file_path, 
            n_robots, 
            rviz_config_path=None, 
            namespace: str = "", 
            basename: str = "RHCViz", 
            rate: float = 100,
            cpu_cores: list = None):
        
        self.syspaths = PathsGetter()

        self.namespace = namespace
        self.basename = basename

        self.cpu_cores = cpu_cores

        self.baselink_name = "base_link"

        self.rate = rate

        self.nodes_ns = []
        self.robot_description_name = f'{self.basename}_{self.namespace}/robot_description' # one robot
        # description for all
        self.nodes_tf_prefixes = []
        for i in range(0, n_robots):
            
            self.nodes_ns.append(f'{self.basename}_{self.namespace}_node_{i}')
            self.nodes_tf_prefixes.append(f'{self.nodes_ns[i]}')

        self.state_ns = f'{self.basename}_{self.namespace}_state'
        self.state_tf_prefix = self.state_ns

        self.urdf_file_path = urdf_file_path
        self.n_robots = n_robots
        self.rviz_config_path = rviz_config_path or self.rviz_config_path_default()
        self.robot_description = self.read_urdf_file(urdf_file_path)
        self.joint_names, _ = self.get_joint_info(URDF.from_xml_string(self.robot_description))

    def get_taskset_command(self):
        """
        Generate the taskset command based on the specified CPU cores.
        """
        if self.cpu_cores:
            core_string = ','.join(map(str, self.cpu_cores))
            return ['taskset', '-c', core_string]
        else:
            return []

    def rviz_config_path_default(self):

        return self.syspaths.DEFAULT_RVIZ_CONFIG_PATH
    
    def read_urdf_file(self, urdf_file_path):
        """
        Read the URDF file from the given path and return its content as a string.
        """
        with open(urdf_file_path, 'r') as file:
            return file.read()

    def update_rviz_config(self):
        with open(self.rviz_config_path, 'r') as file:
            config = yaml.safe_load(file)

        # add robot models for each node
        for i in range(self.n_robots):
            
            rhcnode_config = {
                'Class': 'rviz/RobotModel',
                'Name': 'RHCNode{}'.format(i),
                'Enabled': True,
                'Robot Description': f'{self.robot_description_name}',
                'TF Prefix': f'{self.nodes_tf_prefixes[i]}'
            }
            config['Visualization Manager']['Displays'].append(rhcnode_config)

        # add a robot model for the true robot state
        # (either coming from the simulator or the real robot)
        robotstate_config = {
            'Class': 'rviz/RobotModel',
            'Name': 'RobotState',
            'Enabled': True,
            'Robot Description': f'{self.robot_description_name}',
            'TF Prefix': f'{self.state_tf_prefix}'
        }
        config['Visualization Manager']['Displays'].append(robotstate_config)

        temp_config_path = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz').name
        with open(temp_config_path, 'w') as file:
            yaml.safe_dump(config, file)
        
        return temp_config_path
    
    def get_joint_info(self, urdf_robot):
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

    def generate_random_joint_positions(self, joint_names):
        """
        Generate random joint positions. Here we assume each joint can move between -pi and pi.
        This can be adjusted based on the actual joint limits of the robot.
        """
        return [random.uniform(-3.14, 3.14) for _ in joint_names]

    def start_robot_state_publisher(self, urdf, robot_ns):
        """
        Start a robot_state_publisher for a robot namespace with specified CPU affinity.
        """

        # Set the robot description for each namespace
        full_param_name = '/{}/robot_description'.format(robot_ns)
        rospy.set_param(full_param_name, urdf)

        taskset_command = self.get_taskset_command()
        rsp_command = [
            'rosrun', 'robot_state_publisher', 'robot_state_publisher',
            '__ns:=' + robot_ns,
            '_tf_prefix:=' + robot_ns
        ]
        full_command = taskset_command + rsp_command
        rsp_process = subprocess.Popen(full_command)
        return rsp_process

    def publish_static_tf_rhc_nodes(self, 
                                index: int):

        """
        Publish static transforms from world to base_link for each robot.
        """
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # RHC node
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = 'world'
        static_transform_stamped.child_frame_id = f'{self.nodes_tf_prefixes[index]}/{self.baselink_name}'
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        static_broadcaster.sendTransform(static_transform_stamped)
    
    def publish_static_tf_robot_state(self):
        
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # robot actual state
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = 'world'
        static_transform_stamped.child_frame_id = f'{self.state_tf_prefix}/{self.baselink_name}'
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        static_broadcaster.sendTransform(static_transform_stamped)

    def launch_rviz(self):
        """
        Launch RViz with specified CPU affinity.
        """
        updated_config_path = self.update_rviz_config()
        taskset_command = self.get_taskset_command()
        rviz_command = ['rviz', '-d', updated_config_path]
        full_command = taskset_command + rviz_command
        return subprocess.Popen(full_command)
        
    def run(self):

        rospy.init_node('RHCViz', anonymous=True)

        robot_description = self.read_urdf_file(self.urdf_file_path)
        # Set the robot description for each namespace
        rospy.set_param(self.robot_description_name, robot_description)

        joint_names, _ = self.get_joint_info(URDF.from_xml_string(robot_description))

        # Launch RViz in a separate process
        rviz_process = self.launch_rviz()

        # Start a robot_state_publisher for each RHC node
        rsp_processes = []
        for i in range(self.n_robots):

            rsp_processes.append(self.start_robot_state_publisher(robot_description, self.nodes_ns[i]))

        # Start another robot_state_publisher for the robot state
        rsp_processes.append(self.start_robot_state_publisher(robot_description, self.state_ns))

        # # Publish static transforms for each robot
        # self.publish_static_transforms_for_robots(self.n_robots)

        # Publishers for RHC nodes
        publishers = {}
        for i in range(self.n_robots):
            publishers[self.nodes_ns[i]] = rospy.Publisher('/{}/joint_states'.format(self.nodes_ns[i]), JointState, queue_size=10)
        # Pubilisher for robot state
        publishers[self.state_ns] = rospy.Publisher('/{}/joint_states'.format(self.state_ns), JointState, queue_size=10)

        # Give time for the robot_state_publishers to start
        rospy.sleep(2)

        rate = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            
            # loop_start_time = time.time()  # Start time of the loop

            # publish RHC nodes states
            for i in range(self.n_robots):

                # random joint state
                publishers[self.nodes_ns[i]].publish(JointState(
                    header=rospy.Header(stamp=rospy.Time.now()),
                    name=joint_names,
                    position=self.generate_random_joint_positions(joint_names),
                    velocity=[0]*len(joint_names),
                    effort=[0]*len(joint_names)
                ))
                # constant pose
                self.publish_static_tf_rhc_nodes(i)
            
            # publish robot state
            publishers[self.state_ns].publish(JointState(
                    header=rospy.Header(stamp=rospy.Time.now()),
                    name=joint_names,
                    position=self.generate_random_joint_positions(joint_names),
                    velocity=[0]*len(joint_names),
                    effort=[0]*len(joint_names)))
            # Publish static transforms for each robot
            self.publish_static_tf_robot_state()
            
            rate.sleep()

            # loop_end_time = time.time()  # End time of the loop
            # loop_duration = loop_end_time - loop_start_time
            # expected_duration = 1.0 / self.rate
            # if loop_duration > expected_duration:
            #     rospy.logwarn(f"Loop overran desired rate: Duration {loop_duration:.4f}s > Expected {expected_duration:.4f}s")

        rviz_process.terminate()

        # Cleanup robot_state_publisher processes
        for rsp_process in rsp_processes:
            rsp_process.terminate()

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('urdf_file_path', type=str, help="Path to the URDF file")
    parser.add_argument('n_robots', type=int, help="Number of robots")
    parser.add_argument('--rviz_config', type=str, help="Path to the RViz configuration file", default=None)

    args = parser.parse_args()

    rhcviz = RHCViz(urdf_file_path=args.urdf_file_path, 
           n_robots=args.n_robots, 
           rviz_config_path=args.rviz_config, 
           namespace="", 
           basename="RHCViz_test")
    
    rhcviz.run()