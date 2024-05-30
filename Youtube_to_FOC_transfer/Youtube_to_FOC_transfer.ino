#include <SimpleFOC.h>
#define CSn1 52
#define CSn2 51
#define CSn3 53


int posA=0;
int posB=0;
int posC=0;


float readYawSensorCallback(){
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

float readPitchSensorCallback(){
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


float readRollSensorCallback(){
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


// void initMySensorCallback(){
//   // do the init
// }

// create the sensor
GenericSensor YawSensor = GenericSensor(readYawSensorCallback);
GenericSensor PitchSensor = GenericSensor(readPitchSensorCallback);
GenericSensor RollSensor = GenericSensor(readRollSensorCallback);

void setup() {
  Serial.begin(9600);
  
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CSn1, OUTPUT);
  pinMode(CSn2, OUTPUT);
  pinMode(CSn3, OUTPUT);
  digitalWrite(CSn1, HIGH);
  digitalWrite(CSn2, HIGH);
  digitalWrite(CSn3, HIGH);

  // monitoring port
  // Serial.begin(115200);

  // initialize sensor hardware
  YawSensor.init();
  PitchSensor.init();
  RollSensor.init();

  Serial.println("My sensor ready");
  _delay(1000);
}

void loop() {
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  YawSensor.update();
  PitchSensor.update();
  RollSensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print("PosY: ");
  Serial.print(YawSensor.getAngle());
  Serial.print(", PosP: ");
  Serial.print(PitchSensor.getAngle());
  Serial.print(", PosR: ");
  Serial.println(RollSensor.getAngle());
  // Serial.println(YawSensor.getVelocity());
}