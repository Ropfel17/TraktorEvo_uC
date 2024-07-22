#include <Arduino.h>
const int HALL_PIN = 2;
const float d_wheel = 0.35;
const float res_hall_perturn = 3;

float wheel_circ_res = 0;

//volatile bool HallStatus = false;
volatile int hall_time_old = 0;
volatile float speed = 0; // in km/h

// put function declarations here:
void HallTriggered();

void setup() {
  // put your setup code here, to run once:
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(0, HallTriggered ,FALLING);
  Serial.begin(9600);

  wheel_circ_res = PI*d_wheel*1/res_hall_perturn; // calculate length of wheel circumference over resolution of hall sensor
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
void HallTriggered(){
  int time_new = millis();  // get millis since program start
  int time_diff = time_new - hall_time_old; // calculate time diff to last trigger
  hall_time_old = time_new; // store new timestamp as last trigger
  speed = wheel_circ_res*3.6*1000/time_diff;  // calculate speed in km/h
  Serial.println(speed);

  /*
  if(HallStatus == true){
    HallStatus = false;
  }
  else{
    HallStatus = true;
  }
  digitalWrite(3,HallStatus);
  */


}