#include <LiquidCrystal_I2C.h>
#include <wire.h>
LiquidCrystal_I2C lcd(0x27 ,16 ,2);

void setup() {
  // put your setup code here, to run once:
  lcd.init();
  lcd.backlight();
}

void loop() {
  // put your main code here, to run repeatedly:
lcd.setCursor(4,0);
lcd.print ("Aku Cinta");
delay(900);

lcd.setCursor(3,1);
lcd.print ("Yadikuy");
delay(2000);
lcd.clear();

lcd.setCursor(2,0);
lcd.print ("tapi boong");
delay(5000);
lcd.clear();

}
