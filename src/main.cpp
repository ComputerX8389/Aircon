#include <Arduino.h>

#define PotPin A5

const byte OC1A_PIN = 9;
const byte OC1B_PIN = 10;

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

int NbTopsFan; 
int Calc;
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

void setup() {
  Serial.begin(9600);
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
}

void loop() {
  int speed = map(analogRead(PotPin), 0, 1024, 0, 100);
  if(speed > 96) {
    speed = 100;
  }
  setPwmDuty(speed);

  NbTopsFan = 0;
  //Enables interrupts
  sei();
  
  //Wait 1 second
  delay (1000);
  
  //Disable interrupts
  cli();
  
  //Times NbTopsFan (which is apprioxiamately the fequency the fan 
  //is spinning at) by 60 seconds before dividing by the fan's divider
  Calc = ((NbTopsFan * 60)/fanspace[fan].fandiv);
  
  Serial.print(speed);
  Serial.print("% | ");

  Serial.print (Calc, DEC);
  Serial.println(" rpm");
}