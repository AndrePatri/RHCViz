import rospy

def start_robot_state_publisher(urdf, robot_ns, node_index):
        """
        Start a robot_state_publisher on a separate process, based on the node index.
        """
        rospy.init_node(f'RHCViz_robot_state_publisher_n{node_index}')

        # Set the robot description for each namespace
        full_param_name = '/{}/robot_description'.format(robot_ns)
        rospy.set_param(full_param_name, urdf)

        rsp_command = [
            'rosrun', 'robot_state_publisher', 'robot_state_publisher',
            '__ns:=' + robot_ns,
            '_tf_prefix:=' + robot_ns
        ]

        import subprocess
        rsp_process = subprocess.Popen(rsp_command)
        rsp_process.wait()