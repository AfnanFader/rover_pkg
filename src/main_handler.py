import rospy
from std_msgs.msg import String
import time, zmq

class ZmqServer():

    def __init__(self, debug=False, port=5600):

        #ROS instance
        rospy.init_node('Zmq_Server',disable_signals=True)
        self.data_frame = String
        self.ros_pub_motor_control = rospy.Publisher("/cmd_val", String, queue_size=2)
        rospy.loginfo("[ZmqServer] Motor arduino publisher initialized")
        
        #ZMQ instance
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.bind("tcp://*:{}".format(port))
        self.data = ""

    def run(self):

        rospy.loginfo("[ZmqServer] Waiting for Client")
        self.socket.send(b'1')

        rate = rospy.Rate(10)

        try:

            while not rospy.is_shutdown():
                    
                self.data = self.socket.recv_json()
                self.ros_pub_motor_control.publish(self.data_frame)

                if self.data == -999:
                    rospy.loginfo("[ZmqServer] Goodbye")
                    rospy.signal_shutdown("Goodbye")
                    break
        
        finally:
            self.socket.close()
                
if __name__ == '__main__':
    server = ZmqServer()
    server.run()