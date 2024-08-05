#include <Arduino.h>
#include <Wire.h>
#include "RoboClaw.h"
#include <SoftwareSerial.h>

// SOFTWARE DEFINES
#define UPPER_ADC_LIMIT 810.0
#define LOWER_ADC_LIMIT 240.0
#define ROBOCLAW_ADDR 0x80   // see value in studio
const double d_wheel = 0.35;
const double res_hall_perturn = 3;

// HARDWARE DEFINES
#define FORW_BACK_SWITCH_PIN    6
#define HALL_PIN                2
#define GAS_PIN                 A0

// GLOBAL VARIABLES
int adc_value = 0;
uint8_t pwm_value = 0;
uint8_t speed_value = 0;
uint8_t dir_speed_value = 0;
bool forward = false;
double wheel_circ_res = 0;
volatile int hall_time_old = 0;
volatile double speed = 0; // in km/h

SoftwareSerial serial(11, 10);	// rxPin, txPin
RoboClaw roboclaw(&serial,10000);   // serial pins, timeout

// FUNCTION DECLARATIONS
void HallTriggered();
void adcToSpeed(void);


void setup() {
    // Set Pins for Drive Switch
    pinMode(FORW_BACK_SWITCH_PIN, INPUT);
    pinMode(HALL_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), HallTriggered ,FALLING);
    Serial.begin(9600);

    wheel_circ_res = PI*d_wheel*1/res_hall_perturn; // calculate length of wheel circumference over resolution of hall sensor

    //Open roboclaw serial ports
    roboclaw.begin(38400);
}

void loop() {
    adcToSpeed();

    adc_value = analogRead(GAS_PIN);

    uint16_t roboclaw_logic_bat_v;
    roboclaw_logic_bat_v = roboclaw.ReadLogicBatteryVoltage(ROBOCLAW_ADDR);


    uint16_t roboclaw_main_bat_v;
    roboclaw_main_bat_v = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_ADDR);

    Serial.println(roboclaw_logic_bat_v);

    if(forward){
        roboclaw.ForwardM1(ROBOCLAW_ADDR, dir_speed_value);
        roboclaw.ForwardM2(ROBOCLAW_ADDR, dir_speed_value);
    }else{
        roboclaw.BackwardM1(ROBOCLAW_ADDR,dir_speed_value);
        roboclaw.BackwardM2(ROBOCLAW_ADDR,dir_speed_value);
    }

    if(digitalRead(PIN6) == HIGH){
        forward = true;
    }
    else{
        forward = false;
    }
}

void adcToSpeed(void){
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
        //Serial.println("backwards");
    }else{
        dir_speed_value = speed_value;
        //Serial.println("forward");    
    }

    //Serial.println(dir_speed_value);
}

void HallTriggered(){
  int time_new = millis();  // get millis since program start
  int time_diff = time_new - hall_time_old; // calculate time diff to last trigger
  hall_time_old = time_new; // store new timestamp as last trigger
  speed = wheel_circ_res*3.6*1000/time_diff;  // calculate speed in km/h
  Serial.println(speed);
}