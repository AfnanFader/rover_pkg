#include <Arduino.h>
#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>

// Define I/O channel pins
static const int SERVO_PAN_PIN = 1;
static const int SERVO_TILT_PIN = 2;
static const int LINEAR_UP_PIN = 3;
static const int LINEAR_DOWN_PIN = 4;

// Servo control callback for ROS Node
void servoControlCallback(const std_msgs::UInt16MultiArray& cmd_msg) {

    

}

// Linear control callback for Ros Node
void linearControlCallback(const std_msgs::UInt8& cmd_msg) {

    if (cmd_msg.data > 128) {

        // Assuming this is up
        digitalWrite(LINEAR_UP_PIN, LOW);
        digitalWrite(LINEAR_DOWN_PIN, HIGH);

    }
    
    if (cmd_msg.data < 128) {

        digitalWrite(LINEAR_UP_PIN, HIGH);
        digitalWrite(LINEAR_DOWN_PIN, LOW);

    }

    if (cmd_msg.data == 128) {

        digitalWrite(LINEAR_UP_PIN, HIGH);
        digitalWrite(LINEAR_DOWN_PIN, LOW);

    }

}

// Initialize ROS Node Handler
ros::NodeHandle nh;

/*!
 * \brief Motor commands cmd in from the pi in an UInt8MultiArray.
 * \details Array of 2 data
 * 0 - Pan --> 0 - 180 degree
 * 1 - Tilt --> 0 - 180 degree
 */
ros::Subscriber<std_msgs::UInt16MultiArray> servoControlSubscriber("servo_control", servoControlCallback);

/*!
 * \brief Motor commands cmd in from the pi in an UInt8MultiArray.
 * \details Uint8 for direction value
 * UP - 255
 * DOWN - 0
 */
ros::Subscriber<std_msgs::UInt8> linearControlSubscriber("linear_control", linearControlCallback);

// Servo Objects
Servo tiltServo;
Servo panServo;

void setup() {

    // Initiallize linear actuator
    pinMode(LINEAR_UP_PIN, OUTPUT);
    pinMode(LINEAR_DOWN_PIN, OUTPUT);

    // Initiallize servo objects
    tiltServo.attach(SERVO_TILT_PIN);
    panServo.attach(SERVO_PAN_PIN);

    // Init ROS Nodes
    nh.initNode();
    nh.subscribe(linearControlSubscriber);
    nh.subscribe(servoControlSubscriber);

}

void loop() {
    nh.spinOnce();
    delay(5);
}
