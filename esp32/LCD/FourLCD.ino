#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd1(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x26, 16, 2);
LiquidCrystal_I2C lcd3(0x25, 16, 2);
LiquidCrystal_I2C lcd4(0x24, 16, 2);

void setup() {
  lcd1.init();
  lcd2.init();
  lcd3.init();
  lcd4.init();

  lcd1.backlight();
  lcd2.backlight();
  lcd3.backlight();
  lcd4.backlight();

  lcd1.setCursor(0, 0);
  lcd1.print("HYE");

  lcd2.setCursor(0, 0);
  lcd2.print("BYE");

  lcd3.setCursor(0, 0);
  lcd3.print("WHAT THE");

  lcd4.setCursor(0, 0);
  lcd4.print("HOME");
}

void loop() {
}
