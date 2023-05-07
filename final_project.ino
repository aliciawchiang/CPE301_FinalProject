/****************************************
 * Name: Alicia Chiang, Haley Marquez, Tayshawn Williams
 * Team: HAT
 * Assignment: Lab Final
 * Date: May 9, 2023
 ****************************************/

//DHT TEMP, HUMIDITY
#include "Adafruit_Sensor.h"
#include "DHT.h"
#define DHTPIN 44
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//LCD
#include <LiquidCrystal.h>
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//STEPPER
#include <Stepper.h>
#define STEPS 32
Stepper stepper(STEPS, 37, 41, 39, 43);
int previousPos = 0;

//CLOCK
#include <RTClib.h>
RTC_DS1307 rtc;

//WATER
//unsigned char WATER_LEVEL_PORT = 0;
#define WATER_LEVEL_PIN 2 // digital pin connected to water level sensor

//UART
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//Define Port A Register Pointers for LEDs 
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

//my_delay Timer Pointers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
volatile unsigned char *portB =    (unsigned char *) 0x25;

//Threshold variables
float thresholdTemp = 72;
float thresholdWater = 100;

void setup() {
  //Interrupt
  //set PB4 to output
  *portDDRB |= 0b00001000;
  //set PB4 LOW
  *portB &= 0b11110111;
  //setup the Timer for Normal Mode, with the TOV interrupt enabled
  setup_timer_regs();
  //Start the UART
  U0init(9600);

  Serial.begin(9600);
  stepper.setSpeed(200);
  lcd.begin(16,2);
  dht.begin();
  running();

  //CLOCK
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) my_delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

byte in_char;
//This array holds the tick values
unsigned int ticks[16]= {440,440, 494, 494, 523, 523, 587, 587, 659, 659, 698, 698, 784, 784, 0, 0};
//This array holds the characters to be entered, index echos the index of the ticks
unsigned char input[16]= {'a', 'A', 'b', 'B', 'c', 'C', 'd', 'D', 'e', 'E', 'f', 'F', 'g', 'G', 'q', 'Q'};

//global ticks counter
int currentTicks = 0;
int timer_running = 0;

void loop(){
  //check for temp, water level, if the shut off button is pressed
  //if the temp falls below threshold, check water, check button
  //running: all things about threshold
  //idle: temp is below
  //disable: button is pressed
  //error: water is below

  // if we recieve a character from serial
  if (U0kbhit()){
    // read the character
    in_char = U0getChar();
    // echo it back
    U0putChar(in_char);
    // if it's the quit character
    if(in_char == 'q' || in_char == 'Q'){
      // set the current ticks to the max value
      currentTicks = 65535;
      // if the timer is running
      if(timer_running){
        // stop the timer
        *myTCCR1B &=  0xF8;
        // set the flag to not running
        timer_running = 0;
        // set PB4 LOW
        *portB &= 0b11110111;
      }
    }
    // otherwise we need to look for the char in the array
    else{
      // look up the character
      for(int i=0; i < 12; i++){
        // if it's the character we received...
        if(in_char == input[i]){
          // set the ticks
           double period = 1.0/double(ticks[i]);
          // 50% duty cycle
          double half_period = period/ 2.0f;
          // clock period def
          double clk_period = 0.0000000625;
          // calc ticks
          unsigned int t = half_period / clk_period;
          currentTicks = t;
          // if the timer is not already running, start it
          if(!timer_running){
              // start the timer
              *myTCCR1B |= 0b00000001;
              // set the running flag
              timer_running = 1;
          }
        }
      }
    }
  }
  my_delay(2000);
  
  //set port for fan to output
  *ddr_a |= 0xFF;
  
  //Stepper motor
  int currentPos = analogRead(0); //changes the direction of the stepper
  reportTransition();
  stepper.step(currentPos - previousPos);
  int previousPos = currentPos;

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
  cs1 = U0getChar();    // read character
}

//FUNCTIONS!!!! 
//LCD monitor
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

//PROGRAM STATES
void running(){
  *port_a |= 0b01000010;
  *port_a &= 0b01000010;
  
  printTempHumidity();
}

void idle(){
  *port_a |= 0b10000000;
  *port_a &= 0b10000000;
}

void disabled(){
  *port_a |= 0b00100000;
  *port_a &= 0b00100000;
}

void error(){
  *port_a |= 0b00001000;
  *port_a &= 0b00001000;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Error: Water level low");
}

//RTC to serial monitor
void reportTransition(){
  DateTime now = rtc.now();
  char timeStr[9];// 8 characters + null terminator
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  for(int i = 0; i < 9; i++){
    U0putChar(timeStr[i]);
  }
  U0putChar('\n');

  my_delay(1000);
}

//my_delay
void my_delay(unsigned int freq){
  double period = 1.0/double(freq);
  double half_period = period/ 2.0f;
  double clk_period = 0.0000000625;
  unsigned int ticks = half_period / clk_period;
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  * myTCCR1B |= 0b00000001;
  while((*myTIFR1 & 0x01)==0);
  *myTCCR1B &= 0xF8;        
  *myTIFR1 |= 0x01;
}

//Timer setup function
void setup_timer_regs(){
  //setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  //reset the TOV flag
  *myTIFR1 |= 0x01;
  //enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}

//TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect){
//Stop the Timer
  *myTCCR1B &= 0xF8;
  //Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  //Start the Timer
  *myTCCR1B |= 0b00000001;
  //if it's not the STOP amount
  if(currentTicks != 65535)
  {
    //XOR to toggle PB6
    *portB ^= 0x40;
  }
}

//check water level
unsigned int water_level() {
  int sensorValue = digitalRead(WATER_LEVEL_PIN);
  if (sensorValue == HIGH) {
    return 100; // water level is high
  } else {
    return 0; // water level is low
  }
}

//UART FUNCTIONS
void U0init(unsigned long U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit(){
  return (RDA & *myUCSR0A);
}

unsigned char U0getChar(){
  return *myUDR0;
}

void U0putChar(unsigned char U0pdata){
  while(!(TBE & *myUCSR0A));
  *myUDR0 = U0pdata;
}
