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

        cmds_aux.append("centauro_root:=" + root)
        cmds_aux.append("legs:=true")
        cmds_aux.append("big_wheel:=true")
        cmds_aux.append("upper_body:=true")
        cmds_aux.append("velodyne:=false")
        cmds_aux.append("realsense:=false")
        cmds_aux.append("floating_joint:=false")
        cmds_aux.append("use_abs_mesh_paths:=true")
        cmds_aux.append("use_local_filesys_for_meshes:=true")

        return cmds
    
    def _get_xrdf_cmds_aliengo(self,
                root: str):
        
        # no particular configuration needed
        
        cmds = {}
        cmds_aux = []

        cmds["aliengo"] = cmds_aux

        cmds_aux.append("aliengo_root:=" + root)
        cmds_aux.append("use_abs_mesh_paths:=true")
        cmds_aux.append("use_local_filesys_for_meshes:=true")

        return cmds
        