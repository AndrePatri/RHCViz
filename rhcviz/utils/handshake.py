import rclpy
from std_msgs.msg import Float64MultiArray, String
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.qos import QoSProfile

class RHCVizHandshake():

    PARAM_INDEX_N_ROBOTS = 0  # Index for the number of robots

    def __init__(self, 
            name: str,
            node: rclpy.node.Node,
            handshake_topic: str, 
            is_server=False):

        self._node = node

        self._queue_size = 10

        self.handshake_topic = handshake_topic
        self.is_server = is_server
        self.n_nodes = None  # Number of robots

        self._qos_settings = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, # BEST_EFFORT
                durability=DurabilityPolicy.TRANSIENT_LOCAL, # VOLATILE
                history=HistoryPolicy.KEEP_LAST, # KEEP_ALL
                depth=self._queue_size,  # Number of samples to keep if KEEP_LAST is used
                liveliness=LivelinessPolicy.AUTOMATIC,
                # deadline=1000000000,  # [ns]
                # partition='my_partition' # useful to isolate communications
                )

        if self.is_server:

            self.publisher = self._node.create_publisher(msg_type=Float64MultiArray, 
                                    topic=handshake_topic,
                                    qos_profile=self._qos_settings)

        else:

            self.subscription = self._node.create_subscription(msg_type=Float64MultiArray, 
                topic=handshake_topic, 
                callback=self.handshake_callback, 
                qos_profile=self._qos_settings)

    def handshake_callback(self, msg):
        
        self.n_nodes = int(msg.data[self.PARAM_INDEX_N_ROBOTS])

    def set_n_nodes(self, n_nodes):

        if self.is_server:
            self.n_nodes = n_nodes
            msg = Float64MultiArray()
            msg.data = [float(self.n_nodes)]

            self.publisher.publish(msg)
            
        else:
            self.get_logger().warn("set_n_nodes called on a non-server instance of RHCVizHandshake.")

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


def main(args=None):
    rclpy.init(args=args)
    node = RHCVizHandshake("your_handshake_topic_name", is_server=True)  # Update the topic name accordingly
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

