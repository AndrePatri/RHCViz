#!/usr/bin/env python
from rhcviz.RHCViz import RHCViz
from rhcviz.tests.urdf_gen_examples import RoboUrdfGen

import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('n_robots', type=int, help="Number of robots")
    parser.add_argument('--rviz_config', type=str, help="Path to the RViz configuration file", default=None)
    parser.add_argument('--robot_type', type=str, choices=['centauro', 'aliengo'], default='centauro',
                    help="robot type to be visualized.")

    args = parser.parse_args()
    
    # generating urdf
    
    urdf_gen = RoboUrdfGen(robotname=args.robot_type, 
                     name= args.robot_type + "Urdf")
    
    rhcviz = RHCViz(urdf_file_path=urdf_gen.urdf_path, 
           n_robots=args.n_robots, 
           rviz_config_path=args.rviz_config,
           namespace=args.robot_type, 
           basename="RHCViz_test", 
           rate = 50           
           )
    
    rhcviz.run()