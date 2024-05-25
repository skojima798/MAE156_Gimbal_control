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
BLDCMotor pitch_motor = BLDCMotor(14);

// init driver
BLDCDriver3PWM driver = BLDCDriver3PWM(9,10,11,8);

// angle set point variable
float target_angle = 0;

// commander interface
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }

void setup() {
    // initialise magnetic sensor hardware
  sensor1.init();
  pinMode(2,INPUT);
  Serial.begin(115200);

  Serial1.begin(115200);
  // SimpleFOCDebug::enable();

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
  pitch_motor.linkSensor(&sensor1);
  Serial.println("motor linked");
  
  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 22;

  driver.init();
  // link the motor to the driver                      
  pitch_motor.linkDriver(&driver);
  Serial.println("driver linked");

  // set control loop to be used
  pitch_motor.controller = MotionControlType::angle;
  
  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  pitch_motor.PID_velocity.P = 0.2;
  pitch_motor.PID_velocity.I = 10;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  pitch_motor.PID_velocity.output_ramp = 400;
  
  //default voltage_power_supply
  pitch_motor.voltage_limit = 22;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  pitch_motor.LPF_velocity.Tf = 0.05;

  // angle P controller 
  // default P=20
  pitch_motor.P_angle.P = 20;
  //  maximal velocity of the position control
  // default 20
  pitch_motor.velocity_limit = 20;
  
  // initialize motor1
  pitch_motor.init();

  // align encoder and start FOC
  // pitch_motor.sensor_direction=Direction::CW;
  // pitch_motor.zero_electric_angle=1.4903;
  pitch_motor.initFOC();
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
  pitch_motor.loopFOC();
  // Read data only when it's available from Serial1 on the MCU
  if (Serial1.available()) {  
    
    if (imu.decode(Serial1.read())) { // when IMU has received the package
        // read pitch from the IMU
        float pitch = imu.pitch*0.01745;
        // float pitch_deg=pitch*57.29;
        target_angle = -pitch;
        
        // function calculating the outer position loop and setting the target position 
        pitch_motor.move(target_angle);
        Serial.println(target_angle);
    }     
  }
  // user communication
  command.run();
}