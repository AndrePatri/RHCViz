#!/usr/bin/env python
from rhcviz.RHCViz import RHCViz
import argparse

if __name__ == '__main__':

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