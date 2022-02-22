// This code implements a proportional-integral controller

#include <Encoder.h>

Encoder wheel(2,3);

// Position input pin, use ratio to adjust range of analog_in
int input_pin = 0;
float ratio = 1;

// Variables Kp and Ki allow for adjustment of PI control
float Kp = 0.45;
float Ki = 13.04;

// Values for position and integral
float current_position = 0;
float requested_position = 0;
float e = 0;
float I = 0;

// controller output
float u = 0;

// period in milliseconds
int period = 10;

// current system time in milliseconds
unsigned long time_now = 0;

float pi = 3.1415;

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  time_now = millis();

  // read current position
  current_position = (float)(wheel.read()) * ((2*pi)/3200);
  // read requested position
  requested_position = analogRead(input_pin) * ratio;
  
  // calculate error
  e = requested_position - current_position
  
  // calculate I
  I += e * period;

  // set output
  u = Kp * e + Ki * I;
  if (u > 7.5) {
    u = 7.5;
  }
  if (u < -7.5) {
    u = -7.5;
  }
  u = (255/7.5) * u;
  analogWrite(9,u);
  
  while (millis < time_now + period) {
    // :)
  }
}
