#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"

#define SENSOR1_CS 53 
MagneticSensorAS5048A sensor1(SENSOR1_CS);

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// cs              - SPI chip select pin 
// bit_resolution  - sensor resolution
// angle_register  - (optional) angle read register - default 0x3FFF
// MagneticSensorSPI as5047u = MagneticSensorSPI(10, 14, 0x3FFF);
// or quick config
// MagneticSensorSPI as5047u = MagneticSensorSPI(10, AS4147_SPI);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
 
  sensor1.init();

  Serial.println("as5047u ready");
  _delay(1000);
}

void loop() {
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  sensor1.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor1.getAngle());
  Serial.print("\t");
  Serial.println(sensor1.getVelocity());
}