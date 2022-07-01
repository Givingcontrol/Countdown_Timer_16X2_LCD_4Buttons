#include <LiquidCrystal.h>
#include "Countimer.h"
Countimer tdown;
//LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

int show = -1;
#include <EEPROM.h>

#define bt_set    D3
#define bt_up     D6
#define bt_down   D7
#define bt_start  D5

int time_s = 0;
int time_m = 0;
int time_h = 0;

int set = 0;
int flag1=0, flag2=0;

int relay = D8;
int buzzer = D4;

//pre-define functions
void print_time();
void tdownComplete();
void eeprom_write();
void eeprom_read();

void setup() {
  int error;
  
 Serial.begin(115200);
  delay(100);
  Serial.println("LCD...");

   // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  delay(100);
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(16, 2); // initialize the lcd

pinMode(bt_set,   INPUT_PULLUP);
pinMode(bt_up,    INPUT_PULLUP);
pinMode(bt_down,  INPUT_PULLUP);
pinMode(bt_start, INPUT_PULLUP);
//pinMode(LED_BUILTIN, OUTPUT);


pinMode(relay, OUTPUT);
pinMode(buzzer, OUTPUT);

lcd.begin(16, 2);
lcd.setBacklight(255);
lcd.clear();
lcd.setCursor(0,0);
lcd.print("   Welcome To   ");
lcd.setCursor(0,1);
lcd.print("Countdown  Timer");
tdown.setInterval(print_time, 999);
//eeprom_read();
delay(1000);
lcd.clear();
}
}

void print_time(){
time_s = time_s-1;
if(time_s<0){time_s=59; time_m = time_m-1;}
if(time_m<0){time_m=59; time_h = time_h-1;}
}

void tdownComplete(){Serial.print("ok");}

//tdown.stop(); 

void loop(){
tdown.run();

if(digitalRead (bt_set) == 0){
if(flag1==0 && flag2==0){flag1=1;
set = set+1;
if(set>3){set=0;}
delay(100); 
}
}else{flag1=0;}

if(digitalRead (bt_up) == 0){
if(set==0){tdown.start(); flag2=1;}
if(set==1){time_s++;}
if(set==2){time_m++;}
if(set==3){time_h++;}
if(time_s>59){time_s=0;}
if(time_m>59){time_m=0;}
if(time_h>99){time_h=0;}
//if(set>0){eeprom_write();}
delay(200); 
}

if(digitalRead (bt_down) == 0){
if(set==0){tdown.stop(); flag2=0;}
if(set==1){time_s--;}
if(set==2){time_m--;}
if(set==3){time_h--;}
if(time_s<0){time_s=59;}
if(time_m<0){time_m=59;}
if(time_h<0){time_h=99;}
//if(set>0){eeprom_write();}
delay(200); 
}

if(digitalRead (bt_start) == 0){ flag2=1; 
  //eeprom_read(); 
  digitalWrite(relay, HIGH); 
  //tdown.restart(); 
  tdown.start();
}

lcd.setCursor(0,0);
if(set==0){lcd.print("      Timer     ");}
if(set==1){lcd.print("  Set Timer SS  ");}
if(set==2){lcd.print("  Set Timer MM  ");}
if(set==3){lcd.print("  Set Timer HH  ");}

lcd.setCursor(4,1);
if(time_h<=9){lcd.print("0");}
lcd.print(time_h);
lcd.print(":");
if(time_m<=9){lcd.print("0");}
lcd.print(time_m);
lcd.print(":");
if(time_s<=9){lcd.print("0");}
lcd.print(time_s);
lcd.print("   ");

if(time_s==0 && time_m==0 && time_h==0 && flag2==1){flag2=0;
tdown.stop(); 
digitalWrite(relay, LOW);
digitalWrite(buzzer, HIGH);
delay(300);
digitalWrite(buzzer, LOW);
delay(200);
digitalWrite(buzzer, HIGH);
delay(300);
digitalWrite(buzzer, LOW);
delay(200);
digitalWrite(buzzer, HIGH);
delay(300);
digitalWrite(buzzer, LOW);
}

if(flag2==1){digitalWrite(relay, HIGH);}
else{digitalWrite(relay, LOW);}

delay(1);
}
/*
void eeprom_write(){
EEPROM.write(1, time_s);  
EEPROM.write(2, time_m);  
EEPROM.write(3, time_h);  
}

void eeprom_read(){
time_s =  EEPROM.read(1);
time_m =  EEPROM.read(2);
time_h =  EEPROM.read(3);

}
*/
