#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
MagneticSensorSPI sensor = MagneticSensorSPI(53, 14, 0x3FFF);
// #include "encoders/as5048a/MagneticSensorAS5048A.h"

//init our encoder
// #define SENSOR1_CS 53 // For Unofficoal way of encoder 
// MagneticSensorAS5048A sensor1(SENSOR1_CS);

// init BLDC motor
BLDCMotor motor = BLDCMotor( 1 );

// init driver
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);

// angle set point variable
float target_angle = 0;

// commander interface
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }

void setup() {
  pinMode(12,OUTPUT);
  Serial.begin(115200);
  SimpleFOCDebug::enable();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  Serial.println("motor linked");
  
  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 24;

  driver.init();
  // link the motor to the driver                      
  motor.linkDriver(&driver);
  Serial.println("driver linked");

  // set control loop to be used
  motor.controller = MotionControlType::angle;
  
  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 10;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 300;
  
  //default voltage_power_supply
  motor.voltage_limit = 5.6;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller 
  // default P=20
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  // default 20
  motor.velocity_limit = 20;
  
  // initialize motor1
  motor.init();

  // align encoder and start FOC
  // motor.sensor_direction=Direction::CW;
  // motor.zero_electric_angle=5.9630;
  motor.initFOC();


  // add target command T
  command.add('T', onTarget, "target angle");

  // monitoring port
  // Serial.begin(115200);
  Serial.println("Motor ready.");
  Serial.println("Set the target angle using serial terminal:");
  _delay(1000);
}

void loop() {
  // iterative FOC function
  motor.loopFOC();

  // function calculating the outer position loop and setting the target position 
  motor.move(target_angle);
  // Serial.println(target_angle);

  // user communication
  command.run();
}