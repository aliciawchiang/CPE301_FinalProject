//DHT Temp, humidity
#include "Adafruit_Sensor.h"
#include "DHT.h"
#define DHTPIN 44
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//LCD
#include <LiquidCrystal.h>
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Stepper
#include <Stepper.h>
#define STEPS 32
Stepper stepper(STEPS, 8, 10, 9, 11);
int previousPos = 0;

//Clock
#include <RTClib.h>
//#include <TimeLib.h>
RTC_DS1307 rtc;

//UART Print Functions
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
/*
//define Port D register pointers for LEDs
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29; */

// Define Port A Register Pointers for LEDs 
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

//D0 = BLUE, D1 = RED, D2 = GREEN, D3 = YELLOW
//set D0, D1, D2, D3 to output

//turn on BLUE led to show system is running

float thresholdTemp = 70;

void setup() {
  // put your setup code here, to run once:
  U0init(9600);
  Serial.begin(9600);
  stepper.setSpeed(200);
  lcd.begin(16,2);
  dht.begin();
  running();

  //*ddr_d |= 0xFF;

  //CLOCK STUFF
  /*#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif*/

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }else{
    Serial.println("hhh");
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }else if (rtc.isrunning()){
    Serial.println("runin");
  }
}

void loop(){
  
  delay(2000);
  
  //RTC
  DateTime now = rtc.now();
  Serial.print(now.hour());
  Serial.print(':');
  Serial.print(now.minute());
  Serial.print(':');
  Serial.print(now.second());
  Serial.println();
  delay(1000);
  

  //set port for fan to output
  *ddr_a |= 0xFF;
  
  
  //Stepper motor
  //while(!disabled()){
  int currentPos = analogRead(0); //changes the direction of the stepper
  reportTransition();
  stepper.step(currentPos - previousPos);
  int previousPos = currentPos;
  //}

  //compare current Temp to threshold 
  float currentTemp = dht.readTemperature(true);
  if(currentTemp < thresholdTemp || currentTemp == thresholdTemp){ 
    reportTransition();
    idle();
    
  }
  else{
    reportTransition();
    running(); 
  }

  //UART
  unsigned char cs1;
  
  while (U0kbhit()==0){}; // wait for RDA = true
  cs1 = U0getchar();    // read character

  //DateTime now = rtc.now();

  Serial.println(now.hour());
  U0putchar(':');
  U0putchar(now.minute());
  U0putchar(':');
  U0putchar(now.second());
  //U0putchar();

}


//FUNCTIONS!!!! 

//LCD monitor function
void printTempHumidity(){

  float h = dht.readHumidity();
  float f = dht.readTemperature(true);
  

  if (isnan(h) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  lcd.clear();
  lcd.setCursor(0,0); //prints temperature on first line
  lcd.print("Temp: ");
  lcd.print(f);
  lcd.print((char) 223);
  lcd.print("F");

  lcd.setCursor(0,1); //prints humidity on the second line
  lcd.print("Humidity: ");
  lcd.print(h);
  lcd.print("%");
}

void running(){
  *port_a |= 0b01000010;
  *port_a &= 0b01000010;
  
  printTempHumidity();
}

void idle(){
 /* *port_d |= 0b00000100;
  *port_d &= 0b00000100;*/
  *port_a |= 0b10000000;
  *port_a &= 0b10000000;

 
  
}


void disabled(){
 /* *port_d |= 0b00001000;
  *port_d &= 0b00001000;*/

  *port_a |= 0b00100000;
  *port_a &= 0b00100000;
 
}

void error(){
  /**port_d |= 0b00000010;
  *port_d &= 0b00000010;*/
  *port_a |= 0b00001000;
  *port_a &= 0b00001000;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Error: Water level low");
}
 

void reportTransition(){

}

void U0init(unsigned long U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit(){
  return (RDA & *myUCSR0A);
}

unsigned char U0getchar(){
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata){
  while(!(TBE & *myUCSR0A));
  *myUDR0 = U0pdata;
}

