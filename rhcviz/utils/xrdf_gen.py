from abc import abstractmethod

from typing import Dict

import rospkg

import subprocess

class UrdfGenerator:

    def __init__(self, 
            robotname: str,
            name: str = "UrdfGenerator"):

        self.robotname = robotname
        
        self.output_path = "/tmp/"
        self.name = name

        self.descr_dump_path = self.output_path + f"{name}"

        self.generated = False

        self.urdf_path = ""

    def generate_urdf(self):

        rospackage = rospkg.RosPack()
        descr_path = rospackage.get_path(self.robotname + "_urdf")
        urdf_path = descr_path + "/urdf"
        xacro_name = self.robotname
        xacro_path = urdf_path + "/" + xacro_name + ".urdf.xacro"
        
        self.urdf_path = self.descr_dump_path + "/" + self.robotname + ".urdf"

        if self._xrdf_cmds() is not None:
            
            cmds = self._xrdf_cmds()[self.robotname]

            if cmds is None:
                
                xacro_cmd = ["xacro"] + [xacro_path] + ["-o"] + [self.urdf_path]

            else:

                xacro_cmd = ["xacro"] + [xacro_path] + cmds + ["-o"] + [self.urdf_path]

        if self._xrdf_cmds() is None:

            xacro_cmd = ["xacro"] + [xacro_path] + ["-o"] + [self.urdf_path]

        try:

            xacro_gen = subprocess.check_call(xacro_cmd)
            
            # we also generate an updated SRDF (used by controllers)

        except:

            raise Exception(f"[{self.__class__.__name__}]" + 
                            f"[Exception]" + 
                            ": failed to generate " + self.robotname + "\'s URDF!!!")

        self.generated = True

    @abstractmethod
    def _xrdf_cmds(self) -> Dict:

        # this has to be implemented by the user depending on the arguments
        # the xacro description of the robot takes. The output is a list 
        # of xacro commands.
        # Example implementation: 

        # def _xrdf_cmds():

        #   cmds = {}
        #   cmds{self.robot_names[0]} = []
        #   xrdf_cmd_vals = [True, True, True, False, False, True]

        #   legs = "true" if xrdf_cmd_vals[0] else "false"
        #   big_wheel = "true" if xrdf_cmd_vals[1] else "false"
        #   upper_body ="true" if xrdf_cmd_vals[2] else "false"
        #   velodyne = "true" if xrdf_cmd_vals[3] else "false"
        #   realsense = "true" if xrdf_cmd_vals[4] else "false"
        #   floating_joint = "true" if xrdf_cmd_vals[5] else "false" # horizon needs a floating joint

        #   cmds.append("legs:=" + legs)
        #   cmds.append("big_wheel:=" + big_wheel)
        #   cmds.append("upper_body:=" + upper_body)
        #   cmds.append("velodyne:=" + velodyne)
        #   cmds.append("realsense:=" + realsense)
        #   cmds.append("floating_joint:=" + floating_joint)

        #   return cmds
    
        pass