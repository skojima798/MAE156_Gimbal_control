#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include <MYUM7.h>

#define CSn1 52 //Pitch
#define CSn2 51 //Roll
#define CSn3 53 //Yaw

int posA=0;
int posB=0;
int posC=0;

// create the sensor
GenericSensor PitchSensor = GenericSensor(readPitchSensorCallback);
GenericSensor RollSensor = GenericSensor(readRollSensorCallback);
GenericSensor YawSensor = GenericSensor(readYawSensorCallback);

// Create Motor
BLDCMotor Pitch_motor = BLDCMotor(14);
// BLDCMotor Roll_motor = BLDCMotor(14);
// BLDCMotor Yaw_motor = BLDCMotor(14);

// Create Driver
BLDCDriver3PWM driverP = BLDCDriver3PWM(9,10,11,8);
// BLDCDriver3PWM driverR = BLDCDriver3PWM(9,10,11,8);
// BLDCDriver3PWM driverY = BLDCDriver3PWM(9,10,11,8);

// Setup one UM7 on Serial1 (TX1/RX1)
MYUM7 imu(Serial);
long startTime;

// angle set point variable
float target_angle_P = 0;
// float target_angle_R = 0;
// float target_angle_Y = 0;

//Set Commander
Commander command = Commander(Serial);
void onTargetP(char* cmd){ command.scalar(&target_angle_P, cmd); }
// void onTargetR(char* cmd){ command.scalar(&target_angle_R, cmd); }
// void onTargetY(char* cmd){ command.scalar(&target_angle_Y, cmd); }


void setup() {
  Serial.begin(115200);

  // IMU SETUP BEGIN
  Serial1.begin(115200);
  _delay(100);
  // Set euler and all processed datasets to 100 Hz
  imu.set_euler_rate(100);
  delay(100);
  imu.set_all_processed_rate(100);
  delay(100);
  // Set the UM7 baud rate to output at 115200 bps to match the MCU
  imu.set_sensor_baud_rate(115200);
  delay(100);
  // IMU SETUP END

  // ENCODER SETUP BEGIN
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CSn1, OUTPUT);
  pinMode(CSn2, OUTPUT);
  pinMode(CSn3, OUTPUT);
  digitalWrite(CSn1, HIGH);
  digitalWrite(CSn2, HIGH);
  digitalWrite(CSn3, HIGH);

  // initialize sensor hardware
  PitchSensor.init();
  RollSensor.init();
  YawSensor.init();

  Serial.println("My sensor ready");
  _delay(1000);
  // ENCODER SETUP END

  //MOTOR AND MOTOR DRIVER SETUP BEGIN
  // Linking Motors to Drivers
  Pitch_motor.linkSensor(&PitchSensor);
  // Roll_motor.linkSensor(&RollSensor);
  // Yaw_motor.linkSensor(&YawSensor);
  // Serial.println("P motor linked");

  driverP.voltage_power_supply = 22;
  // driverR.voltage_power_supply = 22;
  // driverY.voltage_power_supply = 22;
  
  driverP.init();
  // driverR.init();
  // driverY.init();

  Pitch_motor.linkDriver(&driverP);
  // Roll_motor.linkDriver(&driverR);
  // Yaw_motor.linkDriver(&driverY);
  //MOTOR AND MOTOR DRIVER SETUP END

  //MOTOR PID SETUP BEGIN
  Pitch_motor.controller = MotionControlType::angle;
  // Roll_motor.controller = MotionControlType::angle;
  // Yaw_motor.controller = MotionControlType::angle;

  Pitch_motor.PID_velocity.P = 0.2;
  Pitch_motor.PID_velocity.I = 10;
  // Roll_motor.PID_velocity.P = 0.2;
  // Roll_motor.PID_velocity.I = 10;
  // Yaw_motor.PID_velocity.P = 0.2;
  // Yaw_motor.PID_velocity.I = 10;

  //Jerk Control
  Pitch_motor.PID_velocity.output_ramp = 400;
  // Roll_motor.PID_velocity.output_ramp = 400;
  // Yaw_motor.PID_velocity.output_ramp = 400;

  Pitch_motor.voltage_limit = 22;
  // Roll_motor.voltage_limit = 22;
  // Yaw_motor.voltage_limit = 22;

  Pitch_motor.LPF_velocity.Tf = 0.05;
  // Roll_motor.LPF_velocity.Tf = 0.05;
  // Yaw_motor.LPF_velocity.Tf = 0.05;

  Pitch_motor.P_angle.P = 20;  
  // Roll_motor.P_angle.P = 20;  
  // Yaw_motor.P_angle.P = 20;  

  Pitch_motor.velocity_limit = 20;  
  // Roll_motor.velocity_limit = 20;  
  // Yaw_motor.velocity_limit = 20;  
  //MOTOR PID SETUP END

  //MOTOR INIT BEGIN
  Pitch_motor.init();
  Pitch_motor.initFOC();
  // Roll_motor.init();
  // Roll_motor.initFOC();
  // Yaw_motor.init();
  // Yaw_motor.initFOC();
  _delay(1000);
  //MOTOR INIT END
}

void loop() {
  // iterative FOC function
  Pitch_motor.loopFOC();
  // Read data only when it's available from Serial1 on the MCU
  if (Serial1.available()) {  
    
    if (imu.decode(Serial1.read())) { // when IMU has received the package
        // read pitch from the IMU
        float pitch = imu.pitch*0.01745;
        // float pitch_deg=pitch*57.29;
        target_angle_P = -pitch;
        
        // function calculating the outer position loop and setting the target position 
        Pitch_motor.move(target_angle_P);
        Serial.println(target_angle_P);
    }     
  }
  // user communication
  command.run();
}



//Functions Called in the loop to read Encoder data
float readPitchSensorCallback(){
  int EncPin = CSn1;
  digitalWrite(EncPin, LOW);
  byte d = shiftIn(MISO, SCK, MSBFIRST);
  byte d1 = shiftIn(MISO, SCK, MSBFIRST);
  digitalWrite(EncPin, HIGH);
  d &=0b00111111;
  int posEnc = d << 8;
  posEnc = posEnc | d1;
  posEnc = map(posEnc, 0,16384,0,359);
  float  rad_posEnc= posEnc * 0.017453;
  return rad_posEnc;
}

float readRollSensorCallback(){
  int EncPin = CSn2;
  digitalWrite(EncPin, LOW);
  byte d = shiftIn(MISO, SCK, MSBFIRST);
  byte d1 = shiftIn(MISO, SCK, MSBFIRST);
  digitalWrite(EncPin, HIGH);
  d &=0b00111111;
  int posEnc = d << 8;
  posEnc = map(posEnc, 0,16384,0,359);
  float  rad_posEnc= posEnc * 0.017453;
  return rad_posEnc;
}


float readYawSensorCallback(){
  int EncPin = CSn3;
  digitalWrite(EncPin, LOW);
  byte d = shiftIn(MISO, SCK, MSBFIRST);
  byte d1 = shiftIn(MISO, SCK, MSBFIRST);
  digitalWrite(EncPin, HIGH);
  d &=0b00111111;
  int posEnc = d << 8;
  posEnc = map(posEnc, 0,16384,0,359);
  float  rad_posEnc= posEnc * 0.017453;
  return rad_posEnc;
}