#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <rover_pkg/motorMsg.h>

// Define I/O channel pins
static const uint8_t IN_MOTOR_LEFT_A = 8;
static const uint8_t IN_MOTOR_LEFT_B = 7;
static const uint8_t IN_MOTOR_RIGHT_A = 5;
static const uint8_t IN_MOTOR_RIGHT_B = 4;
static const uint8_t EN_MOTOR_LEFT = 9;
static const uint8_t EN_MOTOR_RIGHT = 3;
static const uint8_t VOLTAGE_SENSOR_PIN = A0;

// Motor Varible
int maxSpeed = 255; //Max speed
int leftSpeed, rightSpeed;

// Voltage Sensor Variable
uint16_t vBatt;
float vIn, vOut, voltageSensorVal;
const long interval = 1000; // 1000ms/1s to update voltage
long previousMillis = 0;


// Initialize ROS Node Handler
ros::NodeHandle nh;

// Handle all logic for motor controls
void mainMotorController(int mySpeed, int myTurn, int type= 0, int time = 0) {

    // Calculation done on Pi
    // mySpeed *= maxSpeed;
    // myTurn *= maxSpeed;

    // individual speeds
    leftSpeed = int(mySpeed - myTurn);
    rightSpeed = int(mySpeed + myTurn);

    // limit
    leftSpeed = constrain(leftSpeed, -maxSpeed,maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed,maxSpeed);

    // Send Speed
    analogWrite(EN_MOTOR_LEFT, abs(leftSpeed));
    analogWrite(EN_MOTOR_RIGHT, abs(rightSpeed));

    if (leftSpeed > 0) {
        digitalWrite(IN_MOTOR_LEFT_A, HIGH);
        digitalWrite(IN_MOTOR_LEFT_B, LOW);
        digitalWrite(IN_MOTOR_RIGHT_A, LOW);
        digitalWrite(IN_MOTOR_RIGHT_B, HIGH);
    } else {
        digitalWrite(IN_MOTOR_LEFT_A, 0);
        digitalWrite(IN_MOTOR_LEFT_B, 1);
        digitalWrite(IN_MOTOR_RIGHT_A, 1);
        digitalWrite(IN_MOTOR_RIGHT_B, 0);
    }

    if (rightSpeed > 0) {
        digitalWrite(IN_MOTOR_LEFT_A, 1);
        digitalWrite(IN_MOTOR_LEFT_B, 0);
        digitalWrite(IN_MOTOR_RIGHT_A, 0);
        digitalWrite(IN_MOTOR_RIGHT_B, 1);

    } else {
        digitalWrite(IN_MOTOR_LEFT_A, 0);
        digitalWrite(IN_MOTOR_LEFT_B, 1);
        digitalWrite(IN_MOTOR_RIGHT_A, 1);
        digitalWrite(IN_MOTOR_RIGHT_B, 0);
    }
}

// Motor control callback for Ros Node
void motorControlCallback(const rover_pkg::motorMsg& cmd_msg) {
        mainMotorController(cmd_msg.speed, cmd_msg.turn, cmd_msg.type, cmd_msg.delay);
}

/*!
 * \brief Battery Voltage Ros Node. Subscribe under topic "vBatt"
 * \return Uint16_t Vin * 100
 */
std_msgs::UInt16 voltageData;
ros::Publisher voltageSensorPublisher("vBatt", &voltageData);

/*!
 * \brief Motor commands cmd in from the pi in an Int16MultiArray.
 * \details Array of 4 data
 * 0 - MotorSpeed --> max 255 pwm
 * 1 - TurnSpeed --> Negative right/Positive left
 * 2 - MovementType --> FUTURE IMPLEMENTATION FOR MECANUM WHEELS
 * 4 - Time --> milliseconds/Burst movement (May be removed)
 */
ros::Subscriber<rover_pkg::motorMsg> motorControlSubscriber("motor_control", motorControlCallback);

// Voltage Reading Calculation
uint16_t readVoltageSensor() {

    const float vFactor = 5.128; // Assumption only needs to be calibrated later on
    const float vCC = 5.00; // Arduino input voltage (Need to confirm by voltmeter may be +/-0.2v)
    
    voltageSensorVal = analogRead(VOLTAGE_SENSOR_PIN);
    vOut = (voltageSensorVal / 1024) * vCC;
    vIn = vOut * vFactor;
    
    return (vIn * 100);
}

void setup() {

    // Setup all I/O pins
    pinMode(IN_MOTOR_LEFT_A, OUTPUT);
	pinMode(IN_MOTOR_LEFT_B, OUTPUT);
	pinMode(IN_MOTOR_RIGHT_A, OUTPUT);
	pinMode(IN_MOTOR_RIGHT_B, OUTPUT);
	pinMode(EN_MOTOR_LEFT, OUTPUT);
	pinMode(EN_MOTOR_RIGHT, OUTPUT);

    // Turn off motors - Initial State
    digitalWrite(IN_MOTOR_LEFT_A, LOW);
    digitalWrite(IN_MOTOR_LEFT_B, LOW);
    digitalWrite(IN_MOTOR_RIGHT_A, LOW);
    digitalWrite(IN_MOTOR_RIGHT_B, LOW);

    // Get initial voltage ready to avoid null value
    vBatt = readVoltageSensor();

    // Init ROS Nodes
    nh.initNode();
    nh.advertise(voltageSensorPublisher);
    nh.subscribe(motorControlSubscriber);
}

void loop() {

    unsigned long currentMillis = millis();
    
    // Only run the voltage reading at 1 sec interval
    if (currentMillis - previousMillis >= interval) {

        vBatt = readVoltageSensor();
        voltageData.data = vBatt;
        voltageSensorPublisher.publish(&voltageData);

        previousMillis = currentMillis;
    }

    nh.spinOnce();
    delay(5); // 1ms delay
}
