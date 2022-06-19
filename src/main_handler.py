#!/usr/bin/env

import rospy
from std_msgs.msg import String
import time, zmq


class movement_hander():
    def __init__(self) -> None:
        pass

class ZmqServer():

    def __init__(self, debug=False, port=5600):

        #ROS instance
        rospy.init_node('Zmq_Server',disable_signals=True)
        self.data = String()
        self.ros_pub_motor_control = rospy.Publisher("/cmd_val", String, queue_size=2)
        rospy.loginfo("[ZmqServer] Motor arduino publisher initialized")
        
        #ZMQ instance
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.bind("tcp://*:{}".format(port))

    def run(self):

        rospy.loginfo("[ZmqServer] Waiting for Client")
        self.socket.send(b'1')
        rospy.loginfo("[ZmqServer] Client Connected!")
        rate = rospy.Rate(10)

        try:

            while not rospy.is_shutdown():
    
                self.data.data = self.socket.recv_json()

                if self.data.data == "p":
                    rospy.loginfo("[ZmqServer] Goodbye")
                    rospy.signal_shutdown("Goodbye")
                    break

                self.ros_pub_motor_control.publish(self.data)
        
        finally:
            self.socket.close()
                
if __name__ == '__main__':
    server = ZmqServer()
    server.run()