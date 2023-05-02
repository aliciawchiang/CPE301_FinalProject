//DHT Temp, humidity
#include <DHT.h>
#include <DHT_U.h>
#define DHT11_PIN 22
DHT dht(DHT11_PIN, DHT11);

//LCD
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

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


void setup() {
  // put your setup code here, to run once:
  stepper.setSpeed(200);
  lcd.begin(16,2);
  U0init(9600);

  rtc.begin(); // Initialize the RTC module
  setTime(0); // Set the time to 00:00:00
}

//declare function before called in loop
int printTempHumidity();

void loop() {
  //D0 = BLUE, D1 = RED, D2 = GREEN, D3 = YELLOW
  //set D0, D1, D2, D3 to output
  *ddr_d |= 0xFF;
  //turn on BLUE led to show system is running
  *port_d |= 0b00000001;

  //Stepper motor
  int currentPos = analogRead(0); //changes the direction of the stepper
  stepper.step(currentPos - previousPos);
  int previousPos = currentPos;

  unsigned char cs1;
  while (U0kbhit()==0){};
  
  int thresholdTemp = 70; //could be a macro?

  //compare current Temp to threshold 
  int currentTemp = printTempHumidity();
  if(currentTemp < thresholdTemp){ 
  }

  // Report an event every hour
  if (minute() == 0 && second() == 0) {
    Serial.print("Event reported at ");
    Serial.print(hour(now()));
    Serial.print(":");
    Serial.print(minute(now()));
    Serial.print(":");
    Serial.println(second(now()));
  }
}

//LCD monitor function
int printTempHumidity(){
  float sensorData = dht.read(DHT11_PIN); //reads sensor data and converts temperature to Farenheit
  int farenheit = sensorData * 9/5 + 32;

  lcd.setCursor(0,0); //prints temperature on first line
  lcd.print("Temp: ");
  lcd.print(farenheit);
  lcd.print((char) 223);
  lcd.print("F");

  lcd.setCursor(0,1); //prints humidity on the second line
  lcd.print("Humidity: ");
  lcd.print(sensorData);
  lcd.print("%");

  return farenheit;
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