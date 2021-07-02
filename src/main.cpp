#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PotPin A3

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const byte OC1A_PIN = 9;
const byte OC1B_PIN = 10;

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

int NbTopsFan; 
int Calc;
int Speed;
//The pin location of the sensor
int hallsensor = 2; typedef struct{
 
//Defines the structure for multiple fans and 
//their dividers 
char fantype;
unsigned int fandiv; }fanspec;
 
//Definitions of the fans
//This is the varible used to select the fan and it's divider,
//set 1 for unipole hall effect sensor
//and 2 for bipole hall effect sensor
fanspec fanspace[3]={{0,1},{1,2},{2,8}}; char fan = 1;

void setPwmDuty(byte duty) {
  OCR1A = (word) (duty*TCNT1_TOP)/100;
}

//This is the function that the interupt calls
void rpm ()
{ 
  NbTopsFan++; 
}

void drawsreen() {
  display.clearDisplay();

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  display.print(String(Calc));
  display.print(" rpm");

  display.setCursor(16, 16);
  display.print(String(Speed));
  display.print(" %");

  display.display();
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("Starting"));
  attachInterrupt(0, rpm, RISING);
  
  pinMode(hallsensor, INPUT_PULLUP);
  pinMode(OC1A_PIN, OUTPUT);
  pinMode(PotPin, INPUT);

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Set Timer1 configuration
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
}

void loop() {
  Speed = map(analogRead(PotPin), 0, 1024, 0, 100);
  if(Speed > 96) {
    Speed = 100;
  }
  setPwmDuty(Speed);

  NbTopsFan = 0;
  //Enables interrupts
  sei();
  //Wait 1 second
  delay(1000);
  //Disable interrupts
  cli();
  
  //Times NbTopsFan (which is apprioxiamately the fequency the fan 
  //is spinning at) by 60 seconds before dividing by the fan's divider
  Calc = ((NbTopsFan * 60)/fanspace[fan].fandiv);
  
  Serial.print(Speed);
  Serial.print(F("% | "));

  Serial.print (Calc, DEC);
  Serial.println(F(" rpm"));

  sei();
  drawsreen();
  cli();
}