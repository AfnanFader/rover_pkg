from ast import Sub
import rospy
from rover_pkg.msg import motorMsg
from std_msgs.msg import String

def talker():
    rospy.init_node('Tester_Node')
    pub = rospy.Publisher('motor_control', motorMsg, queue_size=2)
    rate = rospy.Rate(1)

    msg = motorMsg()

    msg.speed = 180
    msg.turn = 28
    msg.type = 1
    msg.delay = 100

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


class ListenerNode():

    def __init__(self):
        rospy.init_node('Tester_Node')
        self.sub = rospy.Subscriber('cmd_val', String, self.response_node)

    def response_node(self, msg):
        rospy.loginfo("[Tester] Data - {}".format(msg.data))

    def run(self):

        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':

    listener = ListenerNode()

    try:
        # talker()
        listener.run()
    except rospy.ROSInterruptException:
        pass