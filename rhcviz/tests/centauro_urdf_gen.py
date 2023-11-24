from rhcviz.utils.xrdf_gen import UrdfGenerator

class CentauroUrdfGen(UrdfGenerator):

    def __init__(self, 
            robotname: str,
            name: str = "CentauroUrdf"):
        
        super().__init__(
            robotname = robotname,
            name = name)

        self.generate_urdf() # actually generated urdf

    def _xrdf_cmds(self):
        
        # implements parent method 

        cmds = {} 

        cmds.update(self._get_xrdf_cmds_centauro())

        return cmds
    
    def _get_xrdf_cmds_centauro(self):
        
        cmds = {}
        cmds_aux = []
        
        xrdf_cmd_vals = [True, True, True, False, False, False]

        legs = "true" if xrdf_cmd_vals[0] else "false"
        big_wheel = "true" if xrdf_cmd_vals[1] else "false"
        upper_body ="true" if xrdf_cmd_vals[2] else "false"
        velodyne = "true" if xrdf_cmd_vals[3] else "false"
        realsense = "true" if xrdf_cmd_vals[4] else "false"
        floating_joint = "true" if xrdf_cmd_vals[5] else "false"

        cmds_aux.append("legs:=" + legs)
        cmds_aux.append("big_wheel:=" + big_wheel)
        cmds_aux.append("upper_body:=" + upper_body)
        cmds_aux.append("velodyne:=" + velodyne)
        cmds_aux.append("realsense:=" + realsense)
        cmds_aux.append("floating_joint:=" + floating_joint)
        
        cmds["centauro"] = cmds_aux

        return cmds