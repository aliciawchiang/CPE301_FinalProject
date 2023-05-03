
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
#include <RTC.h>
#include <TimeLib.h>
DS3231 rtc;

#define RDA 0x80
#define TBE 0x20 
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//define Port D register pointers for LEDs
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29; 

// Define Port A Register Pointers for fan 
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 



void setup() {
  // put your setup code here, to run once:
  U0init(9600);
  Serial.begin(9600);
  stepper.setSpeed(200);
  lcd.begin(16,2);
  dht.begin();
  //rtc.begin(); // Initialize the RTC module
  //setTime(0); // Set the time to 00:00:00
}

void loop() {

  delay(2000);

  //D0 = BLUE, D1 = RED, D2 = GREEN, D3 = YELLOW
  //set D0, D1, D2, D3 to output
  *ddr_d |= 0xFF;
  //turn on BLUE led to show system is running
  

  //set port for fan to output
  *ddr_a |= 0xFF;
  *port_a |= 0b00000001;

  //Stepper motor
  int currentPos = analogRead(0); //changes the direction of the stepper
  stepper.step(currentPos - previousPos);
  int previousPos = currentPos;

  //unsigned char cs1;
  //while (U0kbhit()==0){};
  
  float thresholdTemp = 70; //could be a macro?

  //compare current Temp to threshold 
  float currentTemp = printTempHumidity();
  
  if(currentTemp < thresholdTemp){ 
    *port_d |= 0b00000100;
  }
  else{
    *port_d |= 0b00000001;
  }

  // Report an event every hour
 /* if (minute() == 0 && second() == 0) {
    Serial.print("Event reported at ");
    Serial.print(hour(now()));
    Serial.print(":");
    Serial.print(minute(now()));
    Serial.print(":");
    Serial.println(second(now()));
  }*/

}
  

//LCD monitor function
float printTempHumidity(){

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

  return f;
}


//
// function to initialize USART0 to "int" Baud, 8 data bits,
// no parity, and one stop bit. Assume FCPU = 16MHz.
//
void U0init(unsigned long U0baud){
//  Students are responsible for understanding
//  this initialization code for the ATmega2560 USART0
//  and will be expected to be able to intialize
//  the USART in differrent modes.
//
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
//
// Read USART0 RDA status bit and return non-zero true if set
//
unsigned char U0kbhit(){
  return(*myUCSR0A & RDA);
}
//
// Read input character from USART0 input buffer
//
unsigned char U0getchar(){
  return *myUDR0;
}
//
// Wait for USART0 TBE to be set then write character to
// transmit buffer
//
void U0putchar(unsigned char U0pdata){
  while(!(*myUCSR0A & TBE)){}; //wait for TBE to be true
    *myUDR0 = U0pdata;
}