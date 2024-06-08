#include <SimpleFOC.h>
// #include <gimbal_pinouts.h>
#include "SimpleFOCDrivers.h"
// #include "imu_helpers.h"
#include <MYUM7.h>

// Setup one UM7 on Serial1 (TX1/RX1)
MYUM7 imu(Serial);
long startTime;

//init our encoder
MagneticSensorSPI sensor1 = MagneticSensorSPI(52, 14);

// init BLDC motor
BLDCMotor roll_motor = BLDCMotor(14);

// init driver
// BLDCDriver3PWM driver = BLDCDriver3PWM(3, 4, 5, 2); //CS51
BLDCDriver3PWM driver = BLDCDriver3PWM(7, 8, 9, 6); //CS52
// BLDCDriver3PWM driver = BLDCDriver3PWM(11, 12, 13, 10); //CS53


// angle set point variable
float target_angle = -1.5;
float initial_angle = 1.5; //TODO: Current value is hard coded, not ideal!
float error_in_angle = -1.5;

// Define the range of allowed motor movement
float min_angle = -1.5;
float max_angle = 1.5;

// commander interface
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }

void setup() {
  SimpleFOCDebug::enable(); 
  // initialise magnetic sensor hardware
  sensor1.init();

  Serial.begin(115200);
  Serial1.begin(115200);
  SimpleFOCDebug::enable();

  _delay(100);
  // imu init and configure
  // if ( !initIMU() ) {
  //   Serial.println(F("IMU connection problem... Disabling!"));
  //   return;
  // }
   // Set euler and all processed datasets to 255 Hz
  imu.set_euler_rate(100);
  delay(100);
  imu.set_all_processed_rate(100);
  delay(100);

  // Set the UM7 baud rate to output at 115200 bps to match the MCU
  imu.set_sensor_baud_rate(115200);
  delay(100);

  // link the motor to the sensor
  roll_motor.linkSensor(&sensor1);
  Serial.println("motor linked");
  
  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 22;

  driver.init();
  // link the motor to the driver                      
  roll_motor.linkDriver(&driver);
  Serial.println("driver linked");

  // set control loop to be used
  roll_motor.controller = MotionControlType::angle;
  
  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  roll_motor.PID_velocity.P = 0.7;
  roll_motor.PID_velocity.I = 3;
  // roll_motor.PID_velocity.D = 0.1;
  
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  roll_motor.PID_velocity.output_ramp = 300;
  
  //default voltage_power_supply
  roll_motor.voltage_limit = 12.0;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  roll_motor.LPF_velocity.Tf = 0.01;

  // angle P controller 
  // default P=20
  roll_motor.P_angle.P = 20;
  //  maximal velocity of the position control
  // default 20
  roll_motor.velocity_limit = 10;
  
  // initialize motor1
  roll_motor.init();

  // align encoder and start FOC
  // roll_motor.sensor_direction=Direction::CCW;
  // roll_motor.zero_electric_angle=2.9061;

  roll_motor.sensor_direction=Direction::CW;
  roll_motor.zero_electric_angle=2.360;

  roll_motor.initFOC();


  // startTime = micros();

  // add target command T
  // command.add('T', onTarget, "target angle");

  // monitoring port
  // Serial.begin(115200);
  Serial.println("Motor ready.");
  // Serial.println("Set the target angle using serial terminal:");
  _delay(1000);
}

void loop() {
  // iterative FOC function
  roll_motor.loopFOC();
  // Read data only when it's available from Serial1 on the MCU
  if (Serial1.available()) {  
    
    if (imu.decode(Serial1.read())) { // when IMU has received the package
        // read roll from the IMU
        float roll = imu.roll*0.01745;
        // error_in_angle = (target_angle - roll ) - sensor1.getAngle();
        error_in_angle = -(target_angle + roll);

        // Move the motor to the constrained position 
        roll_motor.move(error_in_angle);

        Serial.print("error_in_angle:");
        Serial.print(error_in_angle);
        Serial.print(", sensor_angle:");
        Serial.print(sensor1.getAngle());
        Serial.print(", roll:");
        Serial.println(roll);
    }     
  }
  // user communication
  command.run();
}