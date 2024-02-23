from rhcviz.utils.xrdf_gen import UrdfGenerator

class RoboUrdfGen(UrdfGenerator):

    def __init__(self, 
            robotname: str,
            descr_path: str,
            name: str = "RobotUrdf"):
        
        super().__init__(
            robotname = robotname,
            descr_path = descr_path,
            name = name)

        self.generate_urdf() # actually generated urdf

    def _xrdf_cmds(self):
        
        # implements parent method 

        cmds = {} 

        if self.robotname == "centauro":

            cmds.update(self._get_xrdf_cmds_centauro(root=self.descr_path))

        if self.robotname == "aliengo":

            cmds.update(self._get_xrdf_cmds_aliengo(root=self.descr_path))

        if self.robotname != "aliengo" and \
            self.robotname != "centauro":

            raise Exception("Unsupported robot type provided.")
        
        return cmds
    
    def _get_xrdf_cmds_centauro(self,
                root: str):
        
        cmds = {}
        cmds_aux = []
        cmds["centauro"] = cmds_aux

        xrdf_cmd_vals = [True, True, True, False, False, False]

        root_centauro = "centauro_root:=" + root
        cmds_aux.append(root_centauro)

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
        
        cmds_aux.append("use_abs_mesh_paths:=true")
        cmds_aux.append("use_local_filesys_for_meshes:=true")

        

        return cmds
    
    def _get_xrdf_cmds_aliengo(self,
                root: str):
        
        # no particular configuration needed
        
        cmds = {}
        cmds_aux = []
        
        cmds["aliengo"] = cmds_aux

        aliengo_root = "aliengo_root:=" + root
        cmds_aux.append(aliengo_root)
        
        cmds_aux.append("use_abs_mesh_paths:=true")
        cmds_aux.append("use_local_filesys_for_meshes:=true")

        return cmds
        