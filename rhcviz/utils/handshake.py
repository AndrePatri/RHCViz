import rospy
from std_msgs.msg import Float64MultiArray

class RHCVizHandshake:

    PARAM_INDEX_N_ROBOTS = 0  # Index for the number of robots

    def __init__(self, 
                handshake_topic, 
                is_server=False):
        
        self.handshake_topic = handshake_topic

        self.is_server = is_server

        self.n_nodes = None  # Number of robots

        if self.is_server:

            self.publisher = rospy.Publisher(handshake_topic, 
                                            Float64MultiArray, 
                                            queue_size=10, 
                                            latch=True)
        
        else:

            self.subscriber = rospy.Subscriber(handshake_topic, 
                                            Float64MultiArray, 
                                            self.handshake_callback)

    def handshake_callback(self, data):

        self.n_nodes = int(data.data[self.PARAM_INDEX_N_ROBOTS])

    def set_n_nodes(self, n_nodes):

        if self.is_server:

            self.n_nodes = n_nodes
            msg = Float64MultiArray()
            msg.data = [float(self.n_nodes)]
            self.publisher.publish(msg)

        else:

            rospy.logwarn("set_n_nodes called on a non-server instance of RHCVizHandshake.")

    def get_n_nodes(self):

        return self.n_nodes

    def handshake_done(self):
        """
        Returns True if handshake data has been received in client mode.
        Raises an error if called in server mode.
        """
        if self.is_server:

            raise RuntimeError("handshake_done method should not be called in server mode.")
        
        return self.n_nodes is not None
