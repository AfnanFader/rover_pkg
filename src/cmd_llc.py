#!/usr/bin/env python3

"""
Class for message distribution and handles all message transmisison to the arduino class.
Also provides fine movement calculations.
"""
import rospy
from std_msgs.msg import UInt8, String
from rover_pkg.msg import motorMsg, servo
import time

class CommandHandler():
    
    def __init__(self):

        self.is_idle = True

        #--- Init ROS Node
        rospy.init_node('cmd_handler', disable_signals=True)
        
        #--- Create the Subscriber to listen
        self.ros_sub_wifi_node = rospy.Subscriber("/cmd_val", String, self.command_received)
        rospy.loginfo("[CmdHndlr] Command mode has been initialized ...")

        #--- Create the motor arduino publisher
        self.ros_pub_motor_control = rospy.Publisher("/motor_control", motorMsg, queue_size=2)
        rospy.loginfo("[CmdHndlr] Motor arduino publisher initialized")
        
        #--- Create the gimbal arduino publisher
        self.ros_pub_servo_position = rospy.Publisher("/servo_control", servo, queue_size=2)
        self.ros_pub_linear_direction = rospy.Publisher("/linear_control", UInt8, queue_size=1)
        rospy.loginfo("[CmdHndlr] Gimbal Arduino publisher initialized")

        #--- Commands to be sent
        self.motor_cmd = motorMsg()
        self.linear_cmd = UInt8()
        self.servo_cmd = servo()

    def update_motor_cmd(self, speed, turn, type, delay):
        self.motor_cmd.speed = speed
        self.motor_cmd.turn = turn
        self.motor_cmd.type = type
        self.motor_cmd.delay = delay

    def update_servo_cmd(self, pan, tilt):
        self.servo_cmd.pan = pan
        self.servo_cmd.tilt = tilt

    def send_motor_cmd(self, data):

        if data == "w":
            self.update_motor_cmd(180,0,0,0)
        elif data == "s":
            self.update_motor_cmd(-180,0,0,0)
        elif data == "a":
            self.update_motor_cmd(0,180,0,0)
        elif data == "d":
            self.update_motor_cmd(0,-180,0,0)

        self.ros_pub_motor_control.publish(self.motor_cmd)
    
    def send_servo_cmd(self, data):
        if data == "i":
            self.update_servo_cmd(0,2)
        elif data == "j":
            self.update_servo_cmd(-2,0)
        elif data == "k":
            self.update_servo_cmd(0,-2)
        elif data == "l":
            self.update_servo_cmd(2,0)
        
        self.ros_pub_servo_position.publish(self.servo_cmd)

    def send_linear_actuator_cmd(self, data):
        if data == "q":
            self.linear_cmd = 0
        else:
            self.linear_cmd = 255
            
        self.ros_pub_linear_direction.publish(self.linear_cmd)

    def set_status_idle(self):
        return False

    def command_received(self, message):

        rospy.loginfo("[CmdHndlr] DATA - {}".format(message.data))

        if message.data == "w" or message.data == "a" or message.data == "s" or message.data == "d":
            self.send_motor_cmd(message.data)

        if message.data == "q" or message.data == "e":
            self.send_linear_actuator_cmd(message.data)

        if message.data == "i" or message.data == "j" or message.data == "k" or message.data == "l":
            self.send_servo_cmd(message.data)

    def run(self):

        rate = rospy.Rate(10)
        rospy.loginfo("[CmdHndlr] Main Loop Running ..")

        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    cmdhndlr = CommandHandler()
    cmdhndlr.run()
