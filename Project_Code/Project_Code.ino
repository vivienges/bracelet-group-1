
//------------ Libraries
#include "Wire.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"

//------------ Pins for Display
//  DIN - green - 11
//  CLK - orange - 13
//  CS - yellow - 10
//  DC - blue - 9
//  RST - brown - 8
//  BL - grey - 7
//  VCC 5V - purple/red
//  GND - white /black


//------------ Variables for Display
#define TFT_DC 9    
#define TFT_CS 10
unsigned int currentBarLevel = 60;
int percentage = 0; // Increase or decrease bar level by 5%
int defaultPercentage = 5;

//------------ Variables for Pulse Sensor
const int pulsePin = 2; 


Adafruit_MPU6050 mpu;

//------------ Variables for Simulated Microphone
const int switchPin = 4;

//--------------- Variables for Gyroscope
int GyroscopeThreshold = 10;
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_z; // variables for accelerometer raw data
sensors_event_t a, g, temp;


//------------ Variables for sensor logic
static const unsigned long refreshInterval = 200;
static unsigned long lastRefreshTimer = 0; 
bool increasing = true;

//------------ Hardware SPI on Feather or other boards
Adafruit_GC9A01A tft(TFT_CS, TFT_DC);

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);
  pinMode(switchPin, INPUT);
  //pulseSensor.begin();
  //pulseSensor.analogInput(pulsePin);   
  //auto-magically blink Arduino's LED with heartbeat.
  //pulseSensor.setThreshold(Threshold); 

  tft.begin();

  delay(500);

  tft.fillScreen(GC9A01A_BLACK);
  
  setBarLevel(currentBarLevel, increasing);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
}

void loop() {

//Serial.println(increasing);
//TODO: Fake pulse input?
//TODO: Decrease bar after defined time interval if no signal inputs were detected 


  mpu.getEvent(&a, &g, &temp);

  // Serial.print("Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  
lastRefreshTimer = sensorLogic(lastRefreshTimer, refreshInterval, checkPulse());
setBarLevel(currentBarLevel, increasing);    
}

unsigned long sensorLogic(unsigned long _lastRefreshTimer, const unsigned long _refreshInterval, int bpmValue)
{
  if ((millis() - _lastRefreshTimer) >= _refreshInterval) {
  _lastRefreshTimer += _refreshInterval;

    if(checkMicrophone())
    {
      increasing = true;
      if(checkAcceleration() && checkPulse())
      {
      percentage = defaultPercentage * 3;
      }
      else if (checkAcceleration() || checkPulse())
      {
      percentage = defaultPercentage * 2;
      }
      else
      {
       percentage = defaultPercentage;
      }

      if(currentBarLevel < 120)
        {
          currentBarLevel += percentage;
        }
    }
    else
    {
    increasing = false;
    percentage = defaultPercentage;
      if(currentBarLevel > 0)
      {
      currentBarLevel -= percentage;
      }
    }
    return _lastRefreshTimer; 
  }  
}

boolean checkAcceleration()
{
  if (a.acceleration.z > GyroscopeThreshold || a.acceleration.z < (GyroscopeThreshold*-1)) {
      return true;
  } else {
      return false;
  }
}


void setBarLevel(unsigned int barLevel, bool increasingState) {
  
  

  uint16_t color;
  int w = tft.width();
  int h = ((tft.height()/120)*barLevel) + 10;

  if (increasingState == false) {
    color = GC9A01A_BLACK;
    if (color == GC9A01A_BLACK) {
        for(int y=h; y>=(h-10); y--) {
          tft.drawFastHLine(0, y, w, color);
          }
          }
         }
  else  {
    color = GC9A01A_RED;
    h = ((tft.height()/120)*barLevel);
      if (color != NULL) {
        for(int y=0; y<h; y++) {
          tft.drawFastHLine(0, y, w, color);
         }
      }
  }
}


bool checkMicrophone() {

if (digitalRead(switchPin) == HIGH) {
    return true;
}
else {
    return false;
}
}

bool checkPulse() {

  if (digitalRead(pulsePin) == HIGH) {
    return true;
}
else {
    return false;
}
}
