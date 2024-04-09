#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RoboClaw.h"
#include <SoftwareSerial.h>

#define UPPER_ADC_LIMIT 810.0
#define LOWER_ADC_LIMIT 240.0

LiquidCrystal_I2C lcd(0x38,16,2);
int adc_value = 0;
uint8_t pwm_value = 0;
uint8_t speed_value = 0;
uint8_t dir_speed_value = 0;
bool forward = false;
bool backward = false;

#define address 0x80   // see value in studio

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	// rxPin, txPin
RoboClaw roboclaw(&serial,10000);   // serial pins, timeout

void setup() {
    lcd.init();                 // initialize the lcd 
    lcd.backlight();

    // Set Pins for Drive Switch
    pinMode(PIN7, INPUT);
    pinMode(PIN6, INPUT);

    //Open roboclaw serial ports
    roboclaw.begin(38400);
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

    if(forward){
        dir_speed_value = speed_value;
    }else if(backward){
        dir_speed_value = speed_value/2;
    }else{
        speed_value= 0;
    }
}

void loop() {
    adcToSpeed();

    lcd.setCursor(0,0);
    lcd.print("ADC:");

    adc_value = analogRead(A0);
 
    lcd.setCursor(4, 0);
    lcd.print(adc_value,DEC);

    uint16_t roboclaw_logic_bat_v;
    roboclaw_logic_bat_v = roboclaw.ReadLogicBatteryVoltage(address);
    lcd.setCursor(8, 0);
    lcd.print("LB:");
    lcd.setCursor(11, 0);
    lcd.print(roboclaw_logic_bat_v,DEC);

    uint16_t roboclaw_main_bat_v;
    roboclaw_main_bat_v = roboclaw.ReadMainBatteryVoltage(address);
    lcd.setCursor(8, 1);
    lcd.print("MB:");
    lcd.setCursor(11, 1);
    lcd.print(roboclaw_main_bat_v,DEC);

    lcd.setCursor(0, 1);
    lcd.print("DSV:");
    lcd.setCursor(4, 1);
    lcd.print("   ");
    lcd.setCursor(4, 1);
    lcd.print(dir_speed_value,DEC);

    if(forward){
        roboclaw.ForwardM1(address, dir_speed_value);
        roboclaw.ForwardM2(address, dir_speed_value);
    }else if(backward){
        roboclaw.BackwardM1(address,dir_speed_value);
        roboclaw.BackwardM2(address,dir_speed_value);
    }

    if(digitalRead(PIN6)== HIGH && digitalRead(PIN7)== HIGH){
        lcd.setCursor(14,0);
        lcd.print("E=");
        forward = false;
        backward = false;
    }
    else if(digitalRead(PIN6)==HIGH){
        lcd.setCursor(14,0);
        lcd.print("F>");
        forward = true;
        backward = false;
    }
    else if (digitalRead(PIN7)==HIGH){
        lcd.setCursor(14,0);
        lcd.print("B<");
        forward = false;
        backward = true;
    }
    else{
        lcd.setCursor(14,0);
        lcd.print("--");
        forward = false;
        backward = false;
    }
}
