import rospy
from rover_pkg.msg import motorMsg

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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass