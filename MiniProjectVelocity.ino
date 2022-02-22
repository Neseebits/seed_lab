// this code outputs the angular velocity of the wheel in degrees per second
// change which print statements are active to output position or integral
// the position of the wheel is sampled every [period] milliseconds
// the velocity is printed to the serial monitor at the same rate

#include <Encoder.h>

Encoder wheel(2,3);

// position in degrees
long oldPos = 0;
long newPos = 0;

// integral of position
long sumPos = 0;

// period in milliseconds
int period = 10;

// current system time in milliseconds
unsigned long time_now = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  time_now = millis();
  // set newPos as current rotational position (degrees)
  newPos = (float)(wheel.read()) * 0.1125;

  // print velocity [degrees/second]
  Serial.println(( 1000 * (newPos - oldPos) / period ));
  
  // print position [degrees]
  //Serial.println(newPos);
  
  // print integral of position
  //sumPos += (newPos * period) / 1000;
  //Serial.println(sumPos);
  
  oldPos = newPos;
  
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    wheel.write(0);
  }
  // wait until sampling period is up
  while(millis() < time_now + period) {
    // :)
  }
}
