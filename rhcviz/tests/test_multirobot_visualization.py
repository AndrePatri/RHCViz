#!/usr/bin/env python
from rhcviz.RHCViz import RHCViz
from rhcviz.tests.centauro_urdf_gen import CentauroUrdfGen

import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('n_robots', type=int, help="Number of robots")
    parser.add_argument('--rviz_config', type=str, help="Path to the RViz configuration file", default=None)

    args = parser.parse_args()
    
    # generating urdf
    centauro_urdf_gen = CentauroUrdfGen(robotname="centauro", 
                                   name="CentauroUrdf")
    
    rhcviz = RHCViz(urdf_file_path=centauro_urdf_gen.urdf_path, 
           n_robots=args.n_robots, 
           rviz_config_path=args.rviz_config,
           namespace="Centauro", 
           basename="RHCViz_test", 
           rate = 100
           )
    
    rhcviz.run()