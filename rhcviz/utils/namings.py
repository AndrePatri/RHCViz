class NamingConventions:

    # utility class which defines naming conventions for 
    # RHCViz topics

    def __init__(self):

        self.ROBOT_Q_NAME = "robot_q"
        self.RHC_Q_NAME = "rhc_q"

        self.HANDSHAKE_BASENAME = "HandShake"

        self.ROB_DESCR_BASENAME = "robot_description"

        self.ROB_STATE_NS_BASE = "state"
        self.ROB_STATE_TF_PREFIX_BASE = "state"

        self.RHC_STATE_NS_BASE = "rhc_node"
        self.RHC_STATE_TF_PREFIX_BASE = "rhc_node"

    def global_ns(self, 
            basename: str, 
            namespace: str):

        return f"{basename}_{namespace}"
    
    def robot_q_topicname(self, 
                    basename: str, 
                    namespace: str):

        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)

        topic_name = f"/{global_ns}_{self.RHC_Q_NAME}"

        return topic_name
    
    def rhc_q_topicname(self, 
                    basename: str, 
                    namespace: str):

        global_ns = self.global_ns(basename = basename, 
                                namespace=namespace)

        topic_name = f"/{global_ns}_{self.ROBOT_Q_NAME}"

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
