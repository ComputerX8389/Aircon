#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// Pins
#define PotPin A3
#define hallsensor 2
#define OC1A_PIN 9
#define DHTPIN 4
#define DHTTYPE DHT11
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Settings
#define FanMinimumProcent 8
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHTPIN, DHTTYPE);

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);

int NbTopsFan;
int RPM;
int Speed;
int LastSpeed;
float Temp;
float Humi;
bool AjustMode;

unsigned int Screeninterval = 100;
unsigned int RPMinterval = 1000;
unsigned int DHTinterval = 2000;
unsigned long PreviousScreenMillis = 0;
unsigned long PreviousRPMMillis = 0;
unsigned long PreviousDHTMillis = 0;
unsigned long lastSpeedChange = 0;

// 'Fan1', 16x16px
const unsigned char epd_bitmap_Fan1[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0xe0, 0x01, 0xe0, 0x00, 0xe6, 0x00, 0x6f, 0x0f, 0xff,
    0x0f, 0xff, 0x0f, 0x60, 0x06, 0x70, 0x00, 0x78, 0x00, 0x78, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00};
// 'Fan2', 16x16px
const unsigned char epd_bitmap_Fan2[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x3c, 0xe0, 0x01, 0xe0, 0x01, 0xe0, 0x00, 0xe6, 0x00, 0x6f, 0x0f, 0xff,
    0x0f, 0xff, 0x0f, 0x60, 0x06, 0x70, 0x00, 0x78, 0x3c, 0x78, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00};
// 'Fan3', 16x16px
const unsigned char epd_bitmap_Fan3[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x7c, 0xe0, 0x01, 0xe0, 0x01, 0xe0, 0x00, 0xe6, 0x70, 0x6f, 0x0f, 0xff,
    0x0f, 0xff, 0x6f, 0x60, 0x06, 0x70, 0x00, 0x78, 0x7c, 0x78, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00};
// 'Fan4', 16x16px
const unsigned char epd_bitmap_Fan4[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0xfe, 0xe0, 0x01, 0xe0, 0xfd, 0xe0, 0x00, 0xe6, 0xf0, 0x6f, 0x0f, 0xff,
    0xef, 0xff, 0x0f, 0x60, 0xf6, 0x70, 0x00, 0x78, 0xff, 0x78, 0x00, 0x70, 0xff, 0x80, 0x00, 0x00};
// 'celsius', 16x16px
const unsigned char epd_bitmap_celsius[] PROGMEM = {
    0x00, 0x00, 0x70, 0xf8, 0x53, 0xfe, 0x53, 0x06, 0x73, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00,
    0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x06, 0x03, 0xfe, 0x00, 0xf8, 0x00, 0x00};
// 'Humidity', 16x16px
const unsigned char epd_bitmap_Humidity[] PROGMEM = {
    0x01, 0x80, 0x03, 0xc0, 0x03, 0xc0, 0x07, 0xe0, 0x0f, 0xf0, 0x0f, 0xf0, 0x13, 0xc8, 0x33, 0x8c,
    0x3f, 0x1c, 0x7e, 0x3e, 0x7c, 0x7e, 0x78, 0xfe, 0x71, 0xce, 0x33, 0xcc, 0x1f, 0xf8, 0x07, 0xe0};
// Array of all fans bitmaps for convenience
const unsigned char *epd_bitmap_allArray[4] = {
    epd_bitmap_Fan1,
    epd_bitmap_Fan2,
    epd_bitmap_Fan3,
    epd_bitmap_Fan4};

#define SpeedChangeTimeout 5000

#define PotX 28
#define PotY 16
#define PotR 14
#define PotWidth 4
#define StartAngle 2.5
#define EndAngle 6.9
#define StepAngle 0.075
#define StepPreFill 2

#define FanRPMX 32
#define FanRPMY 0

#define TempX 0
#define TempY 16

#define HumiX 78
#define HumiY 16

#define Tolorance 5

void drawsreen()
{
  display.clearDisplay();

  if (AjustMode)
  {
    display.setTextSize(3);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text

    // Pot
    float step = 8;
    int big = map(Speed, 0, 100, StartAngle * step, EndAngle * step);

    float end_angle = big / step;
    int rtwo = PotR - PotWidth;

    for (float i = StartAngle; i < end_angle; i = i + StepAngle)
    {
      display.fillTriangle(
          PotX + cos(i) * PotR,
          PotY + sin(i) * PotR,
          PotX + cos(i) * rtwo,
          PotY + sin(i) * rtwo,
          PotX + cos(i + StepAngle * StepPreFill) * PotR,
          PotY + sin(i + StepAngle * StepPreFill) * PotR,
          WHITE);

      display.fillTriangle(
          PotX + cos(i) * PotR,
          PotY + sin(i) * PotR,
          PotX + cos(i) * rtwo,
          PotY + sin(i) * rtwo,
          PotX + cos(i + StepAngle * StepPreFill) * rtwo,
          PotY + sin(i + StepAngle * StepPreFill) * rtwo,
          WHITE);
    }

    // Border
    for (float i = StartAngle; i < EndAngle; i = i + StepAngle)
    {
      display.drawPixel(PotX + cos(i) * PotR, PotY + sin(i) * PotR, WHITE);
      display.drawPixel(PotX + cos(i) * rtwo, PotY + sin(i) * rtwo, WHITE);
    }

    display.drawLine(PotX + cos(EndAngle) * PotR, PotY + sin(EndAngle) * PotR, PotX + cos(EndAngle) * rtwo, PotY + sin(EndAngle) * rtwo, WHITE);

    // Text
    display.setCursor(PotX + PotR * 2, PotY - PotR);
    display.print(Speed);
    display.print("%");
  }
  else
  {
    display.setTextSize(2);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text

    // Fan RPM
    display.setCursor(FanRPMX, FanRPMY);
    display.print(RPM);

    int fanindex = 0;

    if (Speed >= 80)
    {
      fanindex = 3;
    }
    else if (Speed > 50)
    {
      fanindex = 2;
    }
    else if (Speed > 10)
    {
      fanindex = 1;
    }
    const unsigned char *fan = epd_bitmap_allArray[fanindex];
    display.drawBitmap(display.getCursorX(), display.getCursorY(), fan, 16, 16, 1);

    // Temp
    display.setCursor(TempX, TempY);
    display.print(Temp, 1);
    display.drawBitmap(display.getCursorX(), display.getCursorY(), epd_bitmap_celsius, 16, 16, 1);

    // Humi
    display.setCursor(HumiX, HumiY);
    display.print(Humi, 0);
    display.drawBitmap(display.getCursorX(), display.getCursorY(), epd_bitmap_Humidity, 16, 16, 1);
  }

  // Display screen
  display.display();
}

void setPwmDuty()
{
  Speed = map(analogRead(PotPin), 0, 1024, FanMinimumProcent, 100);

  if (Speed > LastSpeed + Tolorance || Speed < LastSpeed - Tolorance)
  {
    LastSpeed = Speed;
    lastSpeedChange = millis();
  }

  if (Speed > 95)
  {
    Speed = 100;
  }

  if (AjustMode)
  {
    OCR1A = (word)(Speed * TCNT1_TOP) / 100;
  }
}

void ReadFanRPM()
{
  if (Speed != 0)
  {
    noInterrupts();
    unsigned long interval = pulseIn(hallsensor, HIGH) + pulseIn(hallsensor, LOW); // Pulse interval in microseconds
    interrupts();
    int RPS = 1000000UL / interval; // Pulses per second
    RPM = ((RPS * 60) / 2);
  }
  else
  {
    RPM = 0;
  }
}

void ReadDHT()
{
  Temp = dht.readTemperature();
  Humi = dht.readHumidity();
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("And HeErReE we go"));

  pinMode(hallsensor, INPUT_PULLUP);
  pinMode(OC1A_PIN, OUTPUT);
  pinMode(PotPin, INPUT);
  Serial.println(F("Pinmodes set"));

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Set Timer1 configuration
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
  Serial.println(F("Timer set"));

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  Serial.println(F("Display Ready"));
  dht.begin();
  Serial.println(F("DHT Ready"));
}

void loop()
{
  // Get snapshot of time
  unsigned long currentMillis = millis();

  // How much time has passed, accounting for rollover with subtraction!
  if ((unsigned long)(currentMillis - PreviousScreenMillis) >= Screeninterval)
  {
    PreviousScreenMillis = currentMillis;
    setPwmDuty();
    drawsreen();
  }

  if ((unsigned long)(currentMillis - PreviousRPMMillis) >= RPMinterval)
  {
    PreviousRPMMillis = currentMillis;
    ReadFanRPM();
  }

  if ((unsigned long)(currentMillis - PreviousDHTMillis) >= DHTinterval)
  {
    PreviousDHTMillis = currentMillis;
    ReadDHT();
  }

  if (lastSpeedChange + SpeedChangeTimeout > currentMillis)
  {
    AjustMode = true;
  }
  else
  {
    AjustMode = false;
  }
}