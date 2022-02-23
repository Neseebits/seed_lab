// This code implements a proportional-integral controller

#include <Encoder.h>
#define tol 0.01

Encoder wheel(2,3);

// Position input pin, use ratio to adjust range of analog_in
int input_pin = 0;
float ratio = 1;

// Variables Kp and Ki allow for adjustment of PI control
float Kp = 1;
float Ki = 5;


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
  pinMode(7,OUTPUT);
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
}

void loop() {
  time_now = millis();

  // read current position
  current_position = (float)(wheel.read()) * ((2*pi)/3200);
  if (current_position > pi) {
    current_position -= 2*pi;
  } else if (current_position < -pi) {
    current_position += 2*pi;
  }
  // read requested position
  //requested_position = analogRead(input_pin) * ratio;
  requested_position = 1.57;

  // calculate error
  e = requested_position - current_position;
  
  // calculate I
  if (abs(e) < tol) {
    I = 0;
  } else {
    I += e * ((float)period / 1000);
  }

  // set output
  u = Kp * e + Ki * I;

  if (u > 0) {
    digitalWrite(7,HIGH);
  } else {
    digitalWrite(7,LOW);
    u *= -1;
  }
  if (u > 7.5) {
    u = 7.5;
  }
  u = (255/7.5) * u;
  analogWrite(9,u);

  Serial.print(current_position);
  Serial.print("\t");
  Serial.print(requested_position);
  Serial.print("\t");
  Serial.print(e);
  Serial.print("\t");
  Serial.print(I);
  Serial.print("\t");
  Serial.println(u);
  
  while (millis() < time_now + period) {
    // :)
  }
}
