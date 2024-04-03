#!/usr/bin/env python
from rhcviz.RHCViz import RHCViz
from rhcviz.tests.urdf_gen_examples import RoboUrdfGen

import argparse

if __name__ == '__main__':

    
    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('--dpath', type=str, help="description path")
    parser.add_argument('--rviz_config', type=str, help="Path to the RViz configuration file", default=None)
    parser.add_argument('--robot_type', type=str, default='MyRobot',
                    help="robot name")

    args = parser.parse_args()
    
    if args.dpath is None:

       raise Exception("dpath was not provided")

    # generating urdf
    
    urdf_gen = RoboUrdfGen(robotname=args.robot_type, 
                     name= args.robot_type,
                     descr_path = args.dpath)
    
    rhcviz = RHCViz(urdf_file_path=urdf_gen.urdf_path, 
           rviz_config_path=args.rviz_config,
           namespace=args.robot_type, 
           basename="RHCViz_test", 
           rate = 100,
           cpu_cores = [0],
           use_only_collisions=False,
           check_jnt_names = False, # just to avoid manual publishing of joints (normaly should be set to true)       
           )
    
    rhcviz.run()