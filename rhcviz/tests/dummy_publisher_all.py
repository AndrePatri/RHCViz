#!/usr/bin/env python
import subprocess
import argparse
import os

def main(robot_type, n_rhc_nodes):
    # Get the directory of the current script
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # Define the commands to run each script with full path
    handshaker_cmd = ['python3', os.path.join(dir_path, 'dummy_handshaker.py'), str(n_rhc_nodes), robot_type]
    rhc_state_pub_cmd = ['python3', os.path.join(dir_path, 'dummy_rhc_state_publisher.py'), robot_type]
    robot_state_pub_cmd = ['python3', os.path.join(dir_path, 'dummy_robot_state_publisher.py'), robot_type]

    # Start each script as a subprocess
    handshaker_process = subprocess.Popen(handshaker_cmd)
    rhc_state_process = subprocess.Popen(rhc_state_pub_cmd)
    robot_state_process = subprocess.Popen(robot_state_pub_cmd)

    # Wait for the subprocesses to finish (optional)
    handshaker_process.wait()
    rhc_state_process.wait()
    robot_state_process.wait()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run all RHCViz related scripts")
    parser.add_argument('--robot_type', choices=['aliengo', 'centauro'], 
                        help="Type of the robot ('aliengo' or 'centauro')")
    parser.add_argument('--n_rhc_nodes', type=int, help="Number of RHC nodes", default=10)

    args = parser.parse_args()
    main(args.robot_type, args.n_rhc_nodes)
