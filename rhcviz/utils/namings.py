class NamingConventions:

    # utility class which defines naming conventions for 
    # RHCViz topics

    def __init__(self):
        
        self.ROBOT_ACT_JNTS_NAMES = "robot_actuated_jointnames"
        self.RHC_ACT_JNTS_NAMES = "rhc_actuated_jointnames"

        self.ROBOT_Q_NAME = "robot_q"
        self.RHC_Q_NAME = "rhc_q"

        self.RHC_REFS_NAME = "rhc_refs"

        self.HANDSHAKE_BASENAME = "HandShake"

        self.ROB_DESCR_BASENAME = "robot_description"

        self.ROB_STATE_NS_BASE = "state"
        self.ROB_STATE_TF_PREFIX_BASE = "state"

        self.RHC_STATE_NS_BASE = "rhc_node"
        self.RHC_STATE_TF_PREFIX_BASE = "rhc_node"

        self.RHC_POSE_REF_NS_BASE = "rhc_pose_ref_rviz"
        self.RHC_TWIST_REF_NS_BASE = "rhc_twist_ref_rviz"

    def global_ns(self, 
            basename: str, 
            namespace: str):

        return f"{basename}_{namespace}"
    
    def robot_jntnames(self, 
                    basename: str, 
                    namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)
    
        return  f"/{global_ns}_{self.ROBOT_ACT_JNTS_NAMES}"

    def rhc_jntnames(self, 
                    basename: str, 
                    namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)
    
        return  f"/{global_ns}_{self.RHC_ACT_JNTS_NAMES}"

    def robot_q_topicname(self, 
                    basename: str, 
                    namespace: str):

        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)

        topic_name = f"/{global_ns}_{self.ROBOT_Q_NAME}"

        return topic_name
    
    def rhc_q_topicname(self, 
                    basename: str, 
                    namespace: str):

        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)

        topic_name = f"/{global_ns}_{self.RHC_Q_NAME}"

        return topic_name

    def rhc_refs_topicname(self, 
                    basename: str, 
                    namespace: str):

        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)

        topic_name = f"/{global_ns}_{self.RHC_REFS_NAME}"

        return topic_name

    def rhc_twist_ref_topicname(self, 
                    basename: str, 
                    namespace: str):

        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)

        topic_name = f"/{global_ns}_{self.RHC_REFS_TWIST_NAME}"

        return topic_name

    def handshake_topicname(self,
                        basename: str, 
                        namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)
        
        return f"/{global_ns}_{self.HANDSHAKE_BASENAME}"
    
    def robot_description_name(self,
                        basename: str, 
                        namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)
        
        return f'{global_ns}/{self.ROB_DESCR_BASENAME}'
    
    def robot_state_ns(self,
                    basename: str, 
                    namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)
        
        return f'{global_ns}_{self.ROB_STATE_NS_BASE}'

    def robot_state_tf_pref(self,
                    basename: str, 
                    namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)
        
        return f'{global_ns}_{self.ROB_STATE_NS_BASE}'
    
    def rhc_pose_ref_ns(self,
                    basename: str, 
                    namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                    namespace=namespace)
        
        return f'{global_ns}_{self.RHC_POSE_REF_NS_BASE}'

    def rhc_twist_ref_ns(self,
                    basename: str, 
                    namespace: str):
        
        global_ns = self.global_ns(basename = basename, 
                                    namespace=namespace)
        
        return f'{global_ns}_{self.RHC_TWIST_REF_NS_BASE}'

    def rhc_state_ns(self,
                    basename: str, 
                    namespace: str, 
                    index: int = None):
        
        global_ns = self.global_ns(basename = basename, 
                                    namespace=namespace)
        
        if index is None:

            return f'{global_ns}_{self.RHC_STATE_NS_BASE}'
        
        else:

            return f'{global_ns}_{self.RHC_STATE_NS_BASE}{index}'
    
    def rhc_state_tf_pref(self,
                    basename: str, 
                    namespace: str, 
                    index: int = None):
        
        global_ns = self.global_ns(basename = basename, 
                                    namespace=namespace)
        
        if index is None:

            return f'{global_ns}_{self.RHC_STATE_TF_PREFIX_BASE}'
        
        else:

            return f'{global_ns}_{self.RHC_STATE_TF_PREFIX_BASE}{index}'
