#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

#define PotPin A3
#define hallsensor 2
#define OC1A_PIN 9
#define FanMinimumProcent 6
#define DHTPIN 5
#define DHTTYPE DHT11

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
DHT dht(DHTPIN, DHTTYPE);

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

int NbTopsFan; 
int RPM;
int Speed;
float Temp;
float Humi;

int Screeninterval=100;
int RPMinterval=1000;
int DHTinterval=2000;
unsigned long PreviousScreenMillis=0;
unsigned long PreviousRPMMillis=0;
unsigned long PreviousDHTMillis=0;

// 'Fan1', 16x16px
const unsigned char epd_bitmap_Fan1 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0xe0, 0x01, 0xe0, 0x00, 0xe6, 0x00, 0x6f, 0x0f, 0xff, 
	0x0f, 0xff, 0x0f, 0x60, 0x06, 0x70, 0x00, 0x78, 0x00, 0x78, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00
};
// 'Fan2', 16x16px
const unsigned char epd_bitmap_Fan2 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x3c, 0xe0, 0x01, 0xe0, 0x01, 0xe0, 0x00, 0xe6, 0x00, 0x6f, 0x0f, 0xff, 
	0x0f, 0xff, 0x0f, 0x60, 0x06, 0x70, 0x00, 0x78, 0x3c, 0x78, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00
};
// 'Fan3', 16x16px
const unsigned char epd_bitmap_Fan3 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x7c, 0xe0, 0x01, 0xe0, 0x01, 0xe0, 0x00, 0xe6, 0x70, 0x6f, 0x0f, 0xff, 
	0x0f, 0xff, 0x6f, 0x60, 0x06, 0x70, 0x00, 0x78, 0x7c, 0x78, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00
};
// 'Fan4', 16x16px
const unsigned char epd_bitmap_Fan4 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0xfe, 0xe0, 0x01, 0xe0, 0xfd, 0xe0, 0x00, 0xe6, 0xf0, 0x6f, 0x0f, 0xff, 
	0xef, 0xff, 0x0f, 0x60, 0xf6, 0x70, 0x00, 0x78, 0xff, 0x78, 0x00, 0x70, 0xff, 0x80, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 192)
const int epd_bitmap_allArray_LEN = 4;
const unsigned char* epd_bitmap_allArray[4] = {
	epd_bitmap_Fan1,
	epd_bitmap_Fan2,
	epd_bitmap_Fan3,
	epd_bitmap_Fan4
};

#define FanRPMX 64
#define FanRPMY 0

void setPwmDuty() {
  Speed = map(analogRead(PotPin), 0, 1024, 0, 100);
  if(Speed > 96) {
    Speed = 100;
  }
  else if (Speed < FanMinimumProcent)
  {
    Speed = 0;
  }
  
  OCR1A = (word) (Speed*TCNT1_TOP)/100;
}

void drawsreen() {
  display.clearDisplay();

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text

  // Fan RPM
  display.setCursor(FanRPMX + 16, FanRPMY);     
  display.print(String(RPM));
  
  int fanindex = 0;

  if(Speed >= 80) {
    fanindex = 3;
  }
  else if (Speed > 40) {
    fanindex = 2;
  }
  else if ( Speed > 0) {
    fanindex = 1;
  }

  const unsigned char * fan = epd_bitmap_allArray[fanindex];

  display.drawBitmap(FanRPMX, FanRPMY, fan, 16, 16, 1);

  // Pot pos
  display.setCursor(0, 0);
  display.print(String(Speed));
  display.print("%");

  display.display();
}

void ReadFanRPM() {
  if(Speed != 0) {
    noInterrupts();
    unsigned long interval = pulseIn(hallsensor, HIGH) + pulseIn(hallsensor, LOW); // Pulse interval in microseconds
    interrupts();
    int RPS = 1000000UL / interval; // Pulses per second
    RPM = ((RPS * 60)/2);
  }
  else {
    RPM = 0;
  }
}

void ReadDHT() {
  Temp = dht.readTemperature();
  Humi = dht.readHumidity();
}

void setup() {
  Serial.begin(9600);
  
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
  dht.begin();
}

void loop() {
  // Get snapshot of time
  unsigned long currentMillis = millis();

  // How much time has passed, accounting for rollover with subtraction!
  if ((unsigned long)(currentMillis - PreviousScreenMillis) >= Screeninterval) {
    PreviousScreenMillis = currentMillis;

    setPwmDuty();
    drawsreen();
  }

  if ((unsigned long)(currentMillis - PreviousRPMMillis) >= RPMinterval) {
    PreviousRPMMillis = currentMillis;

    ReadFanRPM();
  }

  if ((unsigned long)(currentMillis - PreviousDHTMillis) >= DHTinterval) {
    PreviousDHTMillis = currentMillis;

    ReadDHT();
  }
}