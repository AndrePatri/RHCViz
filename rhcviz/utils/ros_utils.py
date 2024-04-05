import rclpy

def start_robot_state_publisher(urdf, robot_ns, node_index):
        """
        Start a robot_state_publisher on a separate process, based on the node index.
        """

        # Set the robot description for each namespace
        full_param_name = '/{}/robot_description'.format(robot_ns)

        rclpy.init()

        node = rclpy.create_node(robot_ns
                            + f"_RHCViz_robot_state_publisher_n{node_index}")

        node.declare_parameter(full_param_name, urdf)

        rsp_command = [
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args',
            '-r', f'__ns:=/{robot_ns}',
            # '-r', f'/tf:=/{robot_ns}/tf',
            '-p', f'robot_description:={urdf}',
            '-p', f'frame_prefix:={robot_ns}/',
            # /RHCViz_test_aliengo_rhc_node0/joint_states
            # '--param', f'robot_state_publisher:__ns:=/aAAAAAAAAAAAAAa',
            # '-p', f'robot_state_publisher:prefix:=Pippo',
            # '__ns:=' + robot_ns,
            # '_tf_prefix:=' + robot_ns
        ]

        import subprocess
        rsp_process = subprocess.Popen(rsp_command)
        rsp_process.wait()