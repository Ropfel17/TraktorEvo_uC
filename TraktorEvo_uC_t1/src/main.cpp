#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x38,16,2);

void setup() {
    lcd.init();                      // initialize the lcd 
    // Print a message to the LCD.
    lcd.backlight();

}

void loop() {
    lcd.setCursor(0,0);
    lcd.print("Hello, world!");
    delay(500);
 
    for(int i = 0; i <= 9999999999999999; i++){
        lcd.setCursor(0,1);
        lcd.print(i,DEC);
    }
}
