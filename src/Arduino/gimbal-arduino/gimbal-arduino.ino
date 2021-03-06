#include <Arduino.h>
#include <ros.h>
#include <Servo.h>
#include <rover_pkg/servo.h>

// Define I/O channel pins
static const int SERVO_PAN_PIN = 6;
static const int SERVO_TILT_PIN = 5;

// Servo Objects
Servo tiltServo;
Servo panServo;

// Servo value

// Initialize ROS Node Handler
ros::NodeHandle nh;

bool calculate_servo_location(uint8_t& curr_pos, int new_val) {

    nh.loginfo("update enter");
    bool update = false;
    int update_val = curr_pos + new_val;

    if (update_val >= 0 || update_val <= 180) {
        nh.loginfo("update is true");
        update = true;
    }

    return update;
}

// Servo control callback for ROS Node
void servoControlCallback(const rover_pkg::servo& cmd_msg) {
    
    // Read current position
    uint8_t curr_pan_val = panServo.read();
    uint8_t curr_tilt_val = tiltServo.read();

    if (calculate_servo_location(curr_pan_val, cmd_msg.pan)) {
        panServo.write((curr_pan_val + cmd_msg.pan));
    }

    if (calculate_servo_location(curr_tilt_val, cmd_msg.tilt)) {
        tiltServo.write((curr_tilt_val + cmd_msg.tilt));
    }

}

/*!
 * \brief Motor commands cmd in from the pi in an UInt8MultiArray.
 * \details Array of 2 data
 * 0 - Pan --> 0 - 180 degree
 * 1 - Tilt --> 0 - 180 degree
 */
ros::Subscriber<rover_pkg::servo> servoControlSubscriber("servo_control", servoControlCallback);

void setup() {

    // Initiallize servo objects
    tiltServo.attach(SERVO_TILT_PIN);
    panServo.attach(SERVO_PAN_PIN);

    // Init ROS Nodes
    nh.initNode();
    nh.subscribe(servoControlSubscriber);

}

void loop() {
    nh.spinOnce();
    delay(5);
}
