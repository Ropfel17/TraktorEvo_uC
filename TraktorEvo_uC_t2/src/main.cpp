#include <Arduino.h>

#define RPWM_PIN  11
#define LPWM_PIN  10
#define GAS_PIN   A0
#define FORW_BACK_SWITCH_PIN  7

#define UPPER_ADC_LIMIT 810.0
#define LOWER_ADC_LIMIT 240.0
#define SPEED_GRADIENT 40
const int FORW_BACK_SWITCH_DELAY = 1000;

int adc_value = 0;
uint8_t speed_value = 0;
uint8_t dir_speed_value = 0;
uint8_t actual_speed = 0;
bool forward = true;

// put function declarations here:
void AdcToSpeed(void);

void setup() {
  // put your setup code here, to run once:
  pinMode(RPWM_PIN,OUTPUT);
  pinMode(LPWM_PIN,OUTPUT);
  pinMode(FORW_BACK_SWITCH_PIN, INPUT);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

      // allow direction change only while standing // no input on gas pedal
    if(actual_speed == 0){
        forward = digitalRead(FORW_BACK_SWITCH_PIN);
    }

  adc_value = analogRead(GAS_PIN);
  AdcToSpeed();

  if(forward){
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, actual_speed);
  }else{
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, actual_speed);
  }
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

  static uint8_t enter_count = 0;
  enter_count++;
  if(enter_count > SPEED_GRADIENT){
    if(dir_speed_value < actual_speed){
      actual_speed--;
    }else if(dir_speed_value > actual_speed){
      actual_speed++;
    }
    enter_count = 0;
  }
}