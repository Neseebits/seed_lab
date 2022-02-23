// This code implements a proportional-integral controller

#include <Encoder.h>
#include <Wire.h>

#define tol 0.01
#define ADDRESS 0x04

//I2C data reading setup
byte data[32];
byte split[4];
int write_to = 0;
int read_len = 0;

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
  Wire.begin(ADDRESS); //define the send and receive functions for I2C comms
  Wire.onRequest(send_data);
  Wire.onReceive(receive_data);
  
  pinMode(7,OUTPUT);  //define pins for PWM output and motor direction
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

void receive_data(int num_byte){
  write_to = Wire.read(); //read the address as not to overwrite data on read
 
  while(Wire.available()){ //read while data is available
    data[read_len] = Wire.read();
    read_len++;
  }
  read_len = 0;
  requested_position = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]); //Reconcatenate the 4 read bytes for requested postion
}

void send_data(){
  //split the floating point number into 4 bytes and send each
  split[0] = current_position >> 24;
  split[1] = current_position << 8) >> 24;
  split[2] = (current_position << 16) >> 24;
  split[3] = (current_position << 24) >> 24;
  Wire.write(split, 4); 
}

