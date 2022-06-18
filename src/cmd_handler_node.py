"""
Class for message distribution and handles all message transmisison to the arduino class.
Also provides fine movement calculations.
"""
import rospy
from std_msgs.msg import UInt16, UInt16MultiArray, UInt8, String
from rover_pkg.msg import motorMsg
import time

class CommandHandler():
    
    def __init__(self):

        self.is_idle = True

        #--- Init ROS Node
        rospy.init_node('cmd_handler', disable_signals=True)
        
        #--- Create the Subscriber to listen
        self.ros_sub_wifi_node = rospy.Subscriber("/cmd_val", String, self.set_actuators_from_cmdvel)
        # add later ... auto
        self.ros_sub_voltage_sensor = rospy.Subscriber("/vBatt", UInt16, self.set_actuators_from_cmdvel)
        rospy.loginfo("[CmdHndlr] Command Subscribers has been initialized ...")

        #--- Create the motor arduino publisher
        self.ros_pub_motor_control = rospy.Publisher("/motor_control", UInt16MultiArray, queue_size=2)
        rospy.loginfo("[CmdHndlr] Motor arduino publisher initialized")
        
        #--- Create the gimbal arduino publisher
        self.ros_pub_servo_throttle = rospy.Publisher("/servo_control", UInt16MultiArray, queue_size=2)
        self.ros_pub_linear_direction = rospy.Publisher("/linear_control", UInt8, queue_size=1)
        rospy.loginfo("[CmdHndlr] Gimbal Arduino publisher initialized")

    def set_status_idle(self):
        pass

    def run(self):

        rate = rospy.Rate(10)

        while not rospy.signal_shutdown("[CmdHandler] Shutdown .."):

            if self.is_idle:
                self.set_status_idle()

            rate.sleep()


if __name__ == "__main__":
    cmdhndlr = CommandHandler()
    cmdhndlr.run()
