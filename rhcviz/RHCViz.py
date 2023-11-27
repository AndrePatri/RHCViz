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
from rhcviz.utils.handshake import RHCVizHandshake

from std_msgs.msg import Float64MultiArray

import numpy as np

# import time

class RHCViz:

    def __init__(self, 
            urdf_file_path: str, 
            rviz_config_path=None, 
            namespace: str = "", 
            basename: str = "RHCViz", 
            handshake_basename: str = "HandShake",
            rate: float = 100,
            cpu_cores: list = None):
        
        self.syspaths = PathsGetter()
        
        self.namespace = namespace
        self.basename = basename

        self.cpu_cores = cpu_cores

        self.baselink_name = "base_link"

        self.rate = rate

        self.global_ns = f"{self.basename}_{self.namespace}"

        self.n_rhc_nodes = -1

        self.nodes_ns = []
        self.robot_description_name = f'{self.global_ns}/robot_description' # one robot
        # description for all
        self.nodes_tf_prefixes = []
        
        self.state_ns = f'{self.global_ns}_state'
        self.state_tf_prefix = self.state_ns

        self.urdf_file_path = urdf_file_path
        self.rviz_config_path = rviz_config_path or self.rviz_config_path_default()
        self.robot_description = self.read_urdf_file(urdf_file_path)
        self.joint_names, _ = self.get_joint_info(URDF.from_xml_string(self.robot_description))
        
        self.rhc_state_topicname = f"/{self.global_ns}_rhc_q"
        self.robot_state_topicname = f"/{self.global_ns}_robot_q"
        self.handshake_basename = handshake_basename
        self.handshake_topicname = f"/{self.global_ns}_{self.handshake_basename}"

        self.rsp_processes = []

        self.handshaker = RHCVizHandshake(self.handshake_topicname, 
                                    is_server=False)
    
    def handshake(self):
        
        # Wait for handshake to complete
        while not rospy.is_shutdown() and not self.handshaker.handshake_done():
            
            rospy.loginfo(f"Waiting for handshake data...")

            rospy.sleep(0.3)

        if self.handshaker.n_nodes is None:

            rospy.logerr("Handshake not completed. Exiting.")

            return
        
        self.n_rhc_nodes = self.handshaker.n_nodes
    
    def finalize_init(self):

        # to be called after all the handshake info is
        # available

        for i in range(0, self.n_rhc_nodes):
            
            self.nodes_ns.append(f'{self.global_ns}_node_{i}')
            self.nodes_tf_prefixes.append(f'{self.nodes_ns[i]}')

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

        # Transparency level for RHC nodes
        alpha_value_start = 0.9  
        alpha_value_end = 0.2

        import math  
        alpha_decay_rate = -math.log(alpha_value_end / alpha_value_start) / self.n_rhc_nodes

        # add robot models for each node
        for i in range(self.n_rhc_nodes):
            
            alpha_value = alpha_value_start * math.exp(-alpha_decay_rate * i)

            rhcnode_config = {
                'Class': 'rviz/RobotModel',
                'Name': 'RHCNode{}'.format(i),
                'Enabled': True,
                'Visual Enabled': True,
                'Collision Enabled': False,
                'Robot Description': f'{self.robot_description_name}',
                'TF Prefix': f'{self.nodes_tf_prefixes[i]}',
                'Alpha': alpha_value
            }
            config['Visualization Manager']['Displays'].append(rhcnode_config)

        # add a robot model for the true robot state
        # (either coming from the simulator or the real robot)
        robotstate_config = {
            'Class': 'rviz/RobotModel',
            'Name': 'RobotState',
            'Enabled': True,
            'Visual Enabled': False,
            'Collision Enabled': True, # to better distinguish the state from the nodes
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

    def initialize_rhc_subscriber(self, 
                        topic_name: str):
        """
        Initialize the subscriber to listen to the rhc state data.
        """
        self.subscriber = rospy.Subscriber(topic_name, 
            Float64MultiArray, 
            self.rhc_state_callback)

    def initialize_robot_state_subscriber(self, 
                        topic_name: str):
        """
        Initialize the subscriber to listen to robot state data.
        """
        self.subscriber = rospy.Subscriber(topic_name, 
            Float64MultiArray, 
            self.robot_state_callback)
        
    def rhc_state_callback(self, data):
        """
        Callback function for processing incoming RHC state data.
        """
        # Convert data to numpy array and reshape
        matrix = np.array(data.data).reshape((-1, self.n_rhc_nodes))
        n_rows, n_cols = matrix.shape

        # Check if number of joints match
        expected_joints = len(self.joint_names)
        if (n_rows - 7) != expected_joints:
            rospy.logerr(f"Number of actuated joints in the message {n_rows - 7} does not match the robot model ({expected_joints}).")
            return

        for i in range(self.n_rhc_nodes):
            # Extract base pose and joint positions for node i
            base_pose = matrix[0:7, i]  # First 7 elements (position + quaternion)
            joint_positions = matrix[7:, i]  # Rest are joint positions

            # Publish base pose and joint positions for this node
            self.publish_rhc_state_to_rviz(i, base_pose, joint_positions)
    
    def robot_state_callback(self, data):
        """
        Callback function for processing incoming robot state data.
        """
        # Convert data to numpy array and reshape
        matrix = np.array(data.data).reshape((-1, 1))
        n_rows, n_cols = matrix.shape

        # Check if number of joints match
        expected_joints = len(self.joint_names)
        if (n_rows - 7) != expected_joints:
            rospy.logerr(f"Number of actuated joints in the message {n_rows - 7} does not match the robot model ({expected_joints}).")
            return

        base_pose = matrix[0:7, 0]  # First 7 elements (position + quaternion)
        joint_positions = matrix[7:, 0]  # Rest are joint positions

        # Publish base pose and joint positions for this node
        self.publish_robot_state_to_rviz(base_pose, joint_positions)

    def publish_rhc_state_to_rviz(self, node_index, base_pose, joint_positions):
        """
        Publish rhc state to rviz
        """
        # Publish base pose
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'world'
        transform.child_frame_id = f'{self.nodes_tf_prefixes[node_index]}/{self.baselink_name}'
        transform.transform.translation.x = base_pose[0]
        transform.transform.translation.y = base_pose[1]
        transform.transform.translation.z = base_pose[2]
        transform.transform.rotation.x = base_pose[3]
        transform.transform.rotation.y = base_pose[4]
        transform.transform.rotation.z = base_pose[5]
        transform.transform.rotation.w = base_pose[6]

        self.tf_broadcaster.sendTransform(transform)

        # Publish joint positions
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = joint_positions

        self.publishers[self.nodes_ns[node_index]].publish(joint_state)

    def publish_robot_state_to_rviz(self, base_pose, joint_positions):
        """
        Publish robot state to rviz
        """
        # Publish base pose
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'world'
        transform.child_frame_id = f'{self.state_tf_prefix}/{self.baselink_name}'
        transform.transform.translation.x = base_pose[0]
        transform.transform.translation.y = base_pose[1]
        transform.transform.translation.z = base_pose[2]
        transform.transform.rotation.x = base_pose[3]
        transform.transform.rotation.y = base_pose[4]
        transform.transform.rotation.z = base_pose[5]
        transform.transform.rotation.w = base_pose[6]

        self.tf_broadcaster.sendTransform(transform)

        # Publish joint positions
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = joint_positions

        self.publishers[self.state_ns].publish(joint_state)

    # def start_robot_state_publisher(self, urdf, robot_ns):
    #     """
    #     Start a robot_state_publisher for a robot namespace with specified CPU affinity.
    #     """

    #     # Set the robot description for each namespace
    #     full_param_name = '/{}/robot_description'.format(robot_ns)
    #     rospy.set_param(full_param_name, urdf)

    #     taskset_command = self.get_taskset_command()
    #     rsp_command = [
    #         'rosrun', 'robot_state_publisher', 'robot_state_publisher',
    #         '__ns:=' + robot_ns,
    #         '_tf_prefix:=' + robot_ns
    #     ]
    #     full_command = taskset_command + rsp_command
    #     rsp_process = subprocess.Popen(full_command)
    #     return rsp_process

    def start_robot_state_publisher(self, urdf, robot_ns, node_index):
        """
        Start a robot_state_publisher for a robot namespace with specified CPU affinity,
        based on the node index.
        """
        # Calculate the appropriate CPU core for this node
        if self.cpu_cores and len(self.cpu_cores) > 0:
            core_index = node_index % len(self.cpu_cores)
            selected_core = self.cpu_cores[core_index]
            taskset_command = ['taskset', '-c', str(selected_core)]
        else:
            taskset_command = []

        # Set the robot description for each namespace
        full_param_name = '/{}/robot_description'.format(robot_ns)
        rospy.set_param(full_param_name, urdf)

        rsp_command = [
            'rosrun', 'robot_state_publisher', 'robot_state_publisher',
            '__ns:=' + robot_ns,
            '_tf_prefix:=' + robot_ns
        ]
        full_command = taskset_command + rsp_command
        rsp_process = subprocess.Popen(full_command)
        return rsp_process
    
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

        self.handshake() # blocks, waits for handshake data to be available

        self.finalize_init()

        robot_description = self.read_urdf_file(self.urdf_file_path)
        # Set the robot description for each namespace
        rospy.set_param(self.robot_description_name, robot_description)

        # Launch RViz in a separate process
        rviz_process = self.launch_rviz()

        # Start a robot_state_publisher for each RHC node and for the robot state
        total_nodes = self.n_rhc_nodes + 1  # Including robot state
        for i in range(total_nodes):
            node_ns = self.nodes_ns[i] if i < self.n_rhc_nodes else self.state_ns
            self.rsp_processes.append(self.start_robot_state_publisher(robot_description, node_ns, i))

        # Publishers for RHC nodes
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.publishers = {}
        for ns in self.nodes_ns:
            self.publishers[ns] = rospy.Publisher(f'/{ns}/joint_states', JointState, queue_size=10)
        # Pubilisher for robot state
        self.publishers[self.state_ns] = rospy.Publisher('/{}/joint_states'.format(self.state_ns), JointState, queue_size=10)
        
        # subscribers to rhc states and robot state
        self.initialize_rhc_subscriber(topic_name=self.rhc_state_topicname)
        self.initialize_robot_state_subscriber(topic_name=self.robot_state_topicname)

        # give some time for the robot_state_publishers to start
        rospy.sleep(3)
    
        rate = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            
            rate.sleep()

        rviz_process.terminate()

        # Cleanup robot_state_publisher child processes
        for rsp_process in self.rsp_processes:
            rsp_process.terminate()

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('urdf_file_path', type=str, help="Path to the URDF file")
    parser.add_argument('--rviz_config', type=str, help="Path to the RViz configuration file", default=None)

    args = parser.parse_args()

    rhcviz = RHCViz(urdf_file_path=args.urdf_file_path, 
           rviz_config_path=args.rviz_config, 
           namespace="", 
           basename="RHCViz_test")
    
    rhcviz.run()