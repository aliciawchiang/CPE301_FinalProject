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

//UART
#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Define Port A Register Pointers for LEDs 
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

//Define Port F Register Pointers for Analog
volatile unsigned char* port_f = (unsigned char*) 0x31; 
volatile unsigned char* ddr_f  = (unsigned char*) 0x30; 
volatile unsigned char* pin_f  = (unsigned char*) 0x2F;


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
float thresholdTemp = 76.30;
float thresholdWater = 33;

//interupt buttons
const byte stopButtonPin = 18;
const byte resetButtonPin = 19;

volatile bool stat;

void setup() {
  //Interrupt
  //set PB4 to output
  *portDDRB |= 0b11110000;
  //set PB4 LOW
  *portB &= 0b11101111;
  //set analog ports to output
  *ddr_f |= 0b00000001;

  attachInterrupt(digitalPinToInterrupt(stopButtonPin), stopButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(resetButtonPin), resetButton, FALLING);

  //setup the Timer for Normal Mode, with the TOV interrupt enabled
  setup_timer_regs();
  //Start the UART
  U0init(9600);
  // Serial.begin(9600); for testing
  stepper.setSpeed(200);
  lcd.begin(16,2);
  dht.begin();
  adc_init();

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

//global ticks counter
int timer_running = 0;

void loop(){

  //get water sensor values and current temp
  int sensorValue = adc_read(5);
  float currentTemp = dht.readTemperature(true);

  if(sensorValue < 100){
    reportTransition();
    error_state();
  }
  else{
    //checks if current temp is less than or equal to threshold
      if(currentTemp <= thresholdTemp){
          reportTransition();
          idle_state();
          stepperMotor();
      }
      else{
          reportTransition();
          running_state();
          stepperMotor();
      }
  }

  my_delay(1000);

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

  my_delay(1000);
}

//PROGRAM STATES
void running_state(){
  //blue LED on
  *port_a |= 0b01000010;
  *port_a &= 0b01000010;
  //set ports for fan on
  *portB |= 0b10100000;
  printTempHumidity();
}

void idle_state(){
  //green LED on
  *port_a |= 0b10000000;
  *port_a &= 0b10000000;
  //set ports for fan off
  *portB &= 0b01011111;
}

void disabled_state(){
  //yellow LED on
  *port_a |= 0b00100000;
  *port_a &= 0b00100000;
  //set ports for fan off
  *portB &= 0b01011111;
  //button #2 code to idle
  if(water_level() > 100){
    stat = true;
  }
}

void error_state(){
  //red LED on
  *port_a |= 0b00001000;
  *port_a &= 0b00001000;
  //set ports for fan off
  *portB &= 0b01011111;
  //LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Error: ");
  lcd.setCursor(0,1);
  lcd.print("Water level low");

  if(stat == false){
    stat = true;
  }
}

void stopButton(){
  if(digitalRead(stopButtonPin) == LOW){
    stat = !stat;
  }
  
}

void resetButton(){
  if(digitalRead(resetButtonPin) == LOW){
    stat = !stat;
  }
}

void stepperMotor(){
  //Stepper motor
  int currentPos = adc_read(0); //changes the direction of the stepper
  reportTransition();
  stepper.step(currentPos - previousPos);
  int previousPos = currentPos;
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

//check water level
unsigned int water_level() {
  int sensorValue = adc_read(5);
  my_delay(1000);
  return sensorValue;
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
  //*myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  //Start the Timer
  *myTCCR1B |= 0b00000001;
  //if it's not the STOP amount
  //if(currentTicks != 65535){
    //XOR to toggle PB6
    //*portB ^= 0x40;
  //}
}

void adc_init(){
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num){
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7){
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
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