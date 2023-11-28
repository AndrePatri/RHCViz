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

from std_msgs.msg import Float64MultiArray, String

import numpy as np

from rhcviz.utils.namings import NamingConventions
from rhcviz.utils.string_list_encoding import StringArray

# import time

class RHCViz:

    def __init__(self, 
            urdf_file_path: str, 
            rviz_config_path=None, 
            namespace: str = "", 
            basename: str = "RHCViz", 
            rate: float = 100,
            cpu_cores: list = None, 
            use_only_collisions = False, 
            nodes_perc: int = 100):
        
        self.syspaths = PathsGetter()
        
        self.names = NamingConventions()

        self.string_list_decoder = StringArray()

        self.use_only_collisions = use_only_collisions

        self.nodes_perc = max(0, min(nodes_perc, 100))  # Ensure within 0-100 range

        self.namespace = namespace
        self.basename = basename

        self.cpu_cores = cpu_cores

        self.baselink_name = "base_link"

        self.rate = rate

        self.n_rhc_nodes = -1
        self.rhc_indeces = [] # might be a subset of [0, ..., self.n_rhc_nodes - 1]

        self.nodes_ns = []
        self.robot_description_name = self.names.robot_description_name(basename=self.basename, 
                                                    namespace=self.namespace)
        
        self.rhc_state_subscriber = None
        self.robot_state_subscriber = None
        self.jnt_names_subscriber = None

        # description for all
        self.nodes_tf_prefixes = []
        
        self.state_ns = self.names.robot_state_ns(basename=self.basename, 
                                                namespace=self.namespace)
        self.state_tf_prefix = self.names.robot_state_tf_pref(basename=self.basename, 
                                                namespace=self.namespace)

        self.urdf_file_path = urdf_file_path
        self.rviz_config_path = rviz_config_path or self.rviz_config_path_default()
        self.robot_description = self.read_urdf_file(urdf_file_path)
        self.joint_names, _ = self.get_joint_info(URDF.from_xml_string(self.robot_description))
        
        self.joint_names_rhc = []
        self.joint_names_acquired = False
        self.joint_names_topicname = self.names.robot_jntnames(basename=self.basename, 
                                                    namespace=self.namespace)
        self.rhc_state_topicname = self.names.rhc_q_topicname(basename=self.basename, 
                                                    namespace=self.namespace)
        self.robot_state_topicname = self.names.robot_q_topicname(basename=self.basename, 
                                                    namespace=self.namespace)
        
        self.handshake_topicname = self.names.handshake_topicname(basename=self.basename, 
                                                    namespace=self.namespace)

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
    
    def calculate_nodes_indices(self, total_nodes, percentage):
        """Calculate and return indices of nodes to display."""
        if percentage >= 100 or total_nodes <= 1:
            return list(range(total_nodes))
        
        num_nodes_to_display = max(1, total_nodes * percentage // 100)
        step = total_nodes / float(num_nodes_to_display)
        return [int(step * i) for i in range(num_nodes_to_display)]
    
    def finalize_init(self):

        # to be called after all the handshake info is
        # available

        # Calculate indices of nodes to display
        self.rhc_indeces = self.calculate_nodes_indices(self.n_rhc_nodes, 
                                                self.nodes_perc)

        for i in range(0, self.n_rhc_nodes):
            
            self.nodes_ns.append(self.names.rhc_state_ns(basename=self.basename, 
                                                    namespace=self.namespace, 
                                                    index=i))
            self.nodes_tf_prefixes.append(self.names.rhc_state_tf_pref(basename=self.basename, 
                                                    namespace=self.namespace, 
                                                    index=i))

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
        alpha_decay_rate = -math.log(alpha_value_end / alpha_value_start) / len(self.rhc_indeces)

        # add robot models for each node
        for i in range(len(self.rhc_indeces)):
            
            alpha_value = alpha_value_start * math.exp(-alpha_decay_rate * i)

            rhcnode_config = {
                'Class': 'rviz/RobotModel',
                'Name': 'RHCNode{}'.format(self.rhc_indeces[i]),
                'Enabled': True,
                'Visual Enabled': False if self.use_only_collisions else True,
                'Collision Enabled': True if self.use_only_collisions else False,
                'Robot Description': f'{self.robot_description_name}',
                'TF Prefix': f'{self.nodes_tf_prefixes[self.rhc_indeces[i]]}',
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

    def initialize_joint_names_subscriber(self, 
                        topic_name: str):
        """
        Initialize the subscriber to listen to joint names
        """
        self.jnt_names_subscriber = rospy.Subscriber(topic_name, 
            String, 
            self.jnt_names_callback)
        
    def initialize_rhc_subscriber(self, 
                        topic_name: str):
        """
        Initialize the subscriber to listen to the rhc state data.
        """
        self.rhc_state_subscriber = rospy.Subscriber(topic_name, 
            Float64MultiArray, 
            self.rhc_state_callback)

    def initialize_robot_state_subscriber(self, 
                        topic_name: str):
        """
        Initialize the subscriber to listen to robot state data.
        """
        self.robot_state_subscriber = rospy.Subscriber(topic_name, 
            Float64MultiArray, 
            self.robot_state_callback)
    
    def jnt_names_callback(self, data):
        
        if not self.joint_names_acquired:

            self.joint_names_rhc = self.string_list_decoder.decode(data.data) 
            
            self.joint_names_acquired = True

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
            rospy.logerr(f"rhc_state_callback: Number of actuated joints in the message {n_rows - 7} " + \
                    f"does not match the robot model ({expected_joints}).")
            return
        if n_cols != self.n_rhc_nodes:
            rospy.logerr(f"rhc_state_callback: Number available rhc nodes in the message {n_cols} " + \
                    f"does not match {self.n_rhc_nodes}, which is the expected one.")
            return
        
        for i in range(len(self.rhc_indeces)):

            # Extract base pose and joint positions for node i

            base_pose = matrix[0:7, self.rhc_indeces[i]]  # First 7 elements (position + quaternion)
            joint_positions = matrix[7:, self.rhc_indeces[i]]  # Rest are joint positions

            # Publish base pose and joint positions for this node
            self.publish_rhc_state_to_rviz(self.rhc_indeces[i], base_pose, joint_positions)
    
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
            rospy.logerr(f"robot_state_callback: Number of actuated joints in the message {n_rows - 7} " + \
                    f"does not match the robot model ({expected_joints}).")
            return
        if n_cols != 1:
            rospy.logerr(f"robot_state_callback: received a robot state matrix with n. cols {n_cols}. " + \
                    f"But the expeted n. cols is {1}")
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
        joint_state.name = self.joint_names_rhc
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
        joint_state.name = self.joint_names_rhc
        joint_state.position = joint_positions

        self.publishers[self.state_ns].publish(joint_state)

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
    
    def check_jnt_names_consistency(self):

        return sorted(self.joint_names) == sorted(self.joint_names_rhc)

    def run(self):

        rospy.init_node('RHCViz', anonymous=True)
        rate = rospy.Rate(self.rate) 

        self.handshake() # blocks, waits for handshake data to be available

        self.finalize_init()

        robot_description = self.read_urdf_file(self.urdf_file_path)
        # Set the robot description for each namespace
        rospy.set_param(self.robot_description_name, robot_description)

        # subscribers to joint names
        self.initialize_joint_names_subscriber(topic_name=self.joint_names_topicname)
        while not self.joint_names_acquired:
            
            rospy.loginfo(f"Waiting for joint names data from RHC controller...")

            rospy.sleep(0.5)

        # check consistency between joint list parsed from urdf and the one 
        # provided by the controller

        if not self.check_jnt_names_consistency():

            rospy.logerr("Not all joints in the parsed URDF where found in the RHC list, or viceversa.")

            return 

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