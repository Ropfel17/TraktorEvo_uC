#include <Arduino.h>
#include <Wire.h>
#include "RoboClaw.h"
#include <SoftwareSerial.h>

// SOFTWARE DEFINES
#define UPPER_ADC_LIMIT 810.0
#define LOWER_ADC_LIMIT 240.0
#define ROBOCLAW_ADDR 0x80   // see value in studio
const int SPEED_0_DELAY = 1000;
const uint8_t LOCK_MAX_FORW_SPEED = 100;
const uint8_t LOCK_MAX_BACK_SPEED = 70;
const double d_wheel = 0.35;
const double res_hall_perturn = 3;

// HARDWARE DEFINES
#define FORW_BACK_SWITCH_PIN    7
#define HALL_PIN                2
#define GAS_PIN                 A0

// GLOBAL VARIABLES
int adc_value = 0;
uint8_t pwm_value = 0;
uint8_t speed_value = 0;
uint8_t dir_speed_value = 0;
bool forward = true;
double wheel_circ_res = 0;
volatile int hall_time_old = 0;
volatile double measured_speed = 0; // in km/h
bool speed_locked = false;

SoftwareSerial serial(11, 10);	// rxPin, txPin
RoboClaw roboclaw(&serial,10000);   // serial pins, timeout

// FUNCTION DECLARATIONS
void HallTriggered();
void AdcToSpeed(void);
void SendSpeedToRoboClaw(void);
void SendMeasuredSpeedToRP(void);


void setup() {
    // Set Pins for Drive Switch
    pinMode(F_R_SWITCH_PIN, INPUT);
    pinMode(HALL_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), HallTriggered ,FALLING);
    Serial.begin(9600);

    wheel_circ_res = PI*d_wheel*1/res_hall_perturn; // calculate length of wheel circumference over resolution of hall sensor

    //Open roboclaw serial ports
    roboclaw.begin(38400);
}

void loop() {
    adc_value = analogRead(GAS_PIN);
    AdcToSpeed();

    // set measured speed to 0, if no interrupt by hall sensor since long time
    int tim = millis();  // get millis since program start
    if((tim - hall_time_old) > SPEED_0_DELAY){
        measured_speed = 0;
    }

    // allow direction change only while standing // no input on gas pedal
    if(speed_value == 0 && measured_speed == 0){
        forward = digitalRead(F_R_SWITCH_PIN);
    }

    SendSpeedToRoboClaw();
    SendMeasuredSpeedToRP();
}

void AdcToSpeed(void){
    if(adc_value >= UPPER_ADC_LIMIT){
        speed_value = 255;
    }else if(adc_value <= LOWER_ADC_LIMIT){
        speed_value = 0;
    }else{
        double buf = (adc_value-LOWER_ADC_LIMIT)/(UPPER_ADC_LIMIT-LOWER_ADC_LIMIT)*255;
        speed_value = (uint8_t) buf;
    }

    if(!forward){
        dir_speed_value = speed_value/2;
    }else{
        dir_speed_value = speed_value;  
    }
}

void HallTriggered(){
  int time_new = millis();  // get millis since program start
  int time_diff = time_new - hall_time_old; // calculate time diff to last trigger
  hall_time_old = time_new; // store new timestamp as last trigger
  measured_speed = wheel_circ_res*3.6*1000/time_diff;  // calculate speed in km/h
}

void SendSpeedToRoboClaw(void){
    // send current speed target value to roboclaw
    if(forward){
        // if not authorized by RP, speed value is limited
        if(speed_locked && (dir_speed_value >= LOCK_MAX_FORW_SPEED)){
            dir_speed_value = LOCK_MAX_FORW_SPEED;
        }
        roboclaw.ForwardM1(ROBOCLAW_ADDR, dir_speed_value);
        roboclaw.ForwardM2(ROBOCLAW_ADDR, dir_speed_value);
    }else{
        // if not authorized by RP, speed value is limited
        if(speed_locked && (dir_speed_value >= LOCK_MAX_BACK_SPEED)){
            dir_speed_value = LOCK_MAX_BACK_SPEED;
        }
        roboclaw.BackwardM1(ROBOCLAW_ADDR,dir_speed_value);
        roboclaw.BackwardM2(ROBOCLAW_ADDR,dir_speed_value);
    }
}

void SendMeasuredSpeedToRP(void){
    int rounded_speed = (int) measured_speed;
    char message[10];
    snprintf(message, sizeof(message), ">S%03d-%01d", rounded_speed, forward);
    Serial.print(message);
}