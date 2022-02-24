// This code implements a proportional-integral controller

#include <Encoder.h>
#include <stdlib.h>
#include <Wire.h>

byte data[32];
char split[5];
int write_to = 0;
int read_len = 0;

Encoder wheel(2,3);

// Position input pin, use ratio to adjust range of analog_in
int input_pin = 0;
float ratio = 1;

// Variables Kp and Ki allow for adjustment of PI control
float Kp = 7;
float Ki = 0.2;


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
  
  Wire.begin(ADDRESS); //define the send and receive functions for I2C comms
  Wire.onRequest(send_data);
  Wire.onReceive(receive_data);
  
  pinMode(7,OUTPUT);
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
}

void loop() {
  time_now = millis();

  // read current position
  current_position = (float)(wheel.read()) * ((2*pi)/3200);

  // calculate error
  e = requested_position - current_position;
  
  // calculate I
  I += e * ((float)period / 1000);

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

void receive_data(int num_byte){
  address = Wire.read(); //read the address as not to overwrite data on read
 
  while(Wire.available()){ //read while data is available
    data[read_len] = Wire.read();
    read_len++;
  }
  read_len = 0;
  //reconcatenate the split byte back into 1  floating point number
  requested_position = atof(data);
  
}

void send_data(){
  //Split up the angle into 4 bytes to be sent back to the pi
  dtostrf(current_position, 4, 2, split)
  Wire.write(split, 4); 
}
