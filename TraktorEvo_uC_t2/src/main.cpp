#include <Arduino.h>

// PIN DEFINITION
#define RPWM_PIN        11  // RPWM Pin of IBT_2 Module (reverse)
#define LPWM_PIN        10  // LPWM Pin of IBT_2 Module (forward)
#define F_R_SWITCH_PIN  7   // Forward/Reverse Switch
#define UNLOCK_PIN      5   // Full Speed Unlock
#define HALL_PIN        2   // Hall Sensor Pin
#define GAS_PIN         A0  // Gas Pedal Pin

// SOFTWARE DEFINES
#define UPPER_ADC_LIMIT 810.0   // Everything over this value will be treated as full speed
#define LOWER_ADC_LIMIT 240.0   // Everything under this value will be treated as no input
#define SPEED_GRADIENT 60       // Higher number => slower acceleration and decelaration 
  // safe mode:
#define SAFE_MODE_REVERSE_MAX 50  // max pwm outputs in safe mode
#define SAFE_MODE_FORWARD_MAX 120 // do not use values over 126!
  // calculation values
const double d_wheel = 0.35;      // diameter of the wheel
const double res_hall_perturn = 3; // magnet pairs in hall sensor ring
const int SPEED_0_DELAY = 1000;   // millis delay after measured speed will be set to 0

// GLOBAL VARIABLES
bool forward = true;          // forward or reverse gear engaged?
bool unlocked_mode = false;   // max speed unlocked?
double wheel_circ_res = 0;    // will be calculated at setup (wheel circumference)
volatile double measured_speed = 0; // in km/h
volatile int hall_time_old = 0; // last time hall interrupt was triggered

// FUNCTION DECLARATIONS
uint8_t AdcToSpeed(int adc_value);
void HallTriggered(void);
void SendMeasuredSpeedToRP(void);

void setup() {
  // PIN SETUP
    // OUTPUTS:
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
    // INPUTS:
  pinMode(F_R_SWITCH_PIN, INPUT);
  pinMode(UNLOCK_PIN, INPUT);
  pinMode(GAS_PIN, INPUT);
    // INTERRUPTS
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN),HallTriggered,FALLING);

  // reset PWM Pins
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  // calculate length of wheel circumference over resolution of hall sensor
  wheel_circ_res = PI*d_wheel*1/res_hall_perturn; 
  
  // Open Serial Port to RP
  Serial.begin(9600);
}

void loop() {
  // read out gas pedal
  int adc_value = analogRead(GAS_PIN);
  // convert adc value to ibt_2_pwm value
  uint8_t ibt_2_pwm_output = AdcToSpeed(adc_value);

  // allow major changes (gear / safe mode) only while standing
  if(ibt_2_pwm_output == 0 && measured_speed == 0){
    // read out Forward/Reverse Switch
    forward = digitalRead(F_R_SWITCH_PIN);
    // read out if safe mode is (de-)activated
    unlocked_mode = digitalRead(UNLOCK_PIN);  // high => unlock
  }

  // if safe mode activated
  if(unlocked_mode != true){
    ibt_2_pwm_output = ibt_2_pwm_output/2;  // scale down throttle input
    if(forward){  // cap throttle to max value
      if(ibt_2_pwm_output > SAFE_MODE_FORWARD_MAX){
        ibt_2_pwm_output = SAFE_MODE_FORWARD_MAX;
      }
    }else{
        if(ibt_2_pwm_output > SAFE_MODE_REVERSE_MAX){
        ibt_2_pwm_output = SAFE_MODE_REVERSE_MAX;
      }
    }
  }

  // set PWM to connected motor drivers
  if(forward){
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, ibt_2_pwm_output);
  }else{
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, ibt_2_pwm_output);
  }

  // set measured speed to 0, if no interrupt by hall sensor longer than SPEED_0_DELAY in millis
  // reason: if not driving => no information from hall sensor
  int tim = millis();  // get millis since program start
  if((tim - hall_time_old) > SPEED_0_DELAY){
      measured_speed = 0; // reset measured speed
  }

  // send measured speed to RP every x cycles
  static int counter = 0;
  counter++;
  if(counter > 1000){
      SendMeasuredSpeedToRP();
      counter = 0;
  }
}

// Converts adc value to actual pwm output
uint8_t AdcToSpeed(int adc_value){
    // set measured speed to 0, if no interrupt by hall sensor since long time
    int tim = millis();  // get millis since program start
    if((tim - hall_time_old) > SPEED_0_DELAY){
        measured_speed = 0;
    }

  static uint8_t pwm_output = 0;  // pwm output value
  uint8_t input_speed_value = 0;  // adc input value converted to a scale of 0 to 255

  // cut of lower and upper ends of gas pedal, so you can reach stand still and max speed
  if(adc_value >= UPPER_ADC_LIMIT){
      input_speed_value = 255;
  }else if(adc_value <= LOWER_ADC_LIMIT){
      input_speed_value = 0;
  }else{
      double buf = (adc_value-LOWER_ADC_LIMIT)/(UPPER_ADC_LIMIT-LOWER_ADC_LIMIT)*255;
      input_speed_value = (uint8_t) buf;
  }

  // if backwards decrease speed
  if(!forward){
    input_speed_value = input_speed_value/2;
  }
  
  // input_speed_value now holds a requested input of 0 to 255
  static uint8_t enter_count = 0;  // counts function calls 
  enter_count++;

  // slow down the acceleration curve in a non blocking way
  if(enter_count > SPEED_GRADIENT){ 
    if(input_speed_value < pwm_output){
      pwm_output = pwm_output-2;
      if(pwm_output < 10){
        pwm_output = 0;
      }
    }else if(input_speed_value > pwm_output){
      pwm_output++;
    }
    enter_count = 0;
  }
  return pwm_output;
}

// Interrupt triggered by Hall Sensor
void HallTriggered(){
  int time_new = millis();      // get millis since program start
  int time_diff = time_new - hall_time_old; // calculate time diff to last trigger
  hall_time_old = time_new;     // store new timestamp as last trigger
  measured_speed = wheel_circ_res*3.6*1000/time_diff;  // calculate speed in km/h
}

// Send measured speed via usb to RP in a "formatted" way
void SendMeasuredSpeedToRP(void){
    static int rounded_speed = 0;
    if(measured_speed < 50){ // if value unplausible keep old value
      rounded_speed = (int) measured_speed;
    }
    char message[6];
    snprintf(message, sizeof(message), ">%02d<\n", rounded_speed);
    Serial.print(message);
}