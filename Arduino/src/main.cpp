
 /*///////////////////////////////////////////////////////////////////////////////////////////////////
 
      Water rocket logging altimeter with Arduino Nano / programmed with Arduino Version 1.6.12
      Logs altitude with 20Hz
      Stores 200 data points before detected apogee and 300 data points after in EEPROM
      Transmits 500 data points to Processing Sketch via Serial Port for analysis and visualization
      Hardware Setup:
      Power: 3x 3V button Cells in parallel to Vbat with switch in power line
      Voltage readout: 2K-2K voltage divider between +Vbat and GND connected to A6
      Sensor: Barometer breakout board connected via I²C
      User Interface: push-button connected between pin 2 and GND
      Published under the Beer Ware License by Niklas Roy (www.niklasroy.com)

    // Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
    // Connect GND to Ground
    // Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
    // Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
    // EOC is not used, it signifies an end of conversion
    // XCLR is a reset pin, also not used here
      
  ///////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <EEPROM.h>

#define BUTTON_PIN 2
#define LED_PIN LED_BUILTIN // On board led

#define BMP_SENSOR_ID 42

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(BMP_SENSOR_ID);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float altitude;
float initialAltitude;
float maxAltitude = 0;
uint8_t altiByte[501]; //altitude in a byte
unsigned long nextSampleTime; //time when next sample has to be taken
int ringBufferPosition = 0;
boolean apogee = false;
int apogeeCount = 0; //number of samples after apogee;

volatile boolean hasBeenTriggered = false;

void trigger()
{
  hasBeenTriggered = true;
  Serial.println("Button pushed interrupt");
}

//////////////////////////////////////// SETUP

void initSensor()
{
  if (!bmp.begin())
  {
    Serial.print("Ooops, no BMP085 detected... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void setup(void)
{
  Serial.begin(9600);
  
  initSensor();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), trigger, RISING);
}

////////////////////////////////////////FUNCTIONS

void blink1Hz() {
  unsigned long tm = millis();
  tm = tm / 500;
  digitalWrite(LED_PIN, tm % 2);
}

void blink10Hz() {
  unsigned long tm = millis();
  tm = tm / 50;
  digitalWrite(LED_PIN, tm % 2);
}

//////////////////////////////////////// LOOP

// Return true if completed, false otherwise.
boolean writePreviousDataToSerial()
{
  Serial.write(255);
  for (uint16_t address = 0; address < EEPROM.length(); ++address)
  {
    if (hasBeenTriggered) return false;

    uint8_t value = EEPROM.read(address);
    Serial.write(value < 255 ? value : 254);
    delay(20);
  }

  return true;
}

void waitForTrigger()
{
  while (!hasBeenTriggered)
  {
    Serial.println("Writing to Serial/Processing");
    if (writePreviousDataToSerial())
    {
      Serial.println("\nTransfer successful");
      Serial.println("Waiting for button to be pushed");
      delay(20);
    }   
  }
}

void loop(void)
{
  digitalWrite(LED_PIN, HIGH);
  
  waitForTrigger();

  Serial.println("\nButton has been pushed!");

  Serial.print("MEASURE INITIAL ALTITUDE:");
  initialAltitude = 0;
  for (int i = 0; i < 10; i++) {
    sensors_event_t event;
    bmp.getEvent(&event);
    initialAltitude += bmp.pressureToAltitude(seaLevelPressure, event.pressure);
  }
  initialAltitude = initialAltitude / 10;

  Serial.print(initialAltitude);
  Serial.print("m");

  // button is pushed: device is armed

  while (apogeeCount <= 300) {  //after apogee: write still 300 samples into ringbuffer
    if (!apogee) {
      blink1Hz(); //blink with 1Hz in order to indicate that device is armed but apogee has not been detected
    } else {
      blink10Hz(); //blink with 10Hz in order to indicate that device is armed and apogee has been detected
    }

    if (millis() >= nextSampleTime) {
      nextSampleTime = nextSampleTime + 50; // take altitude samples with 20Hz
      sensors_event_t event;
      bmp.getEvent(&event);
      altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure) - initialAltitude;

      //detect apogee
      if (altitude > maxAltitude) {
        maxAltitude = altitude;           //save highest altitude
      }
      if (maxAltitude > 4 && altitude < (maxAltitude - 3.0)) { //if height is lower than maximum height -2m
        apogee = true;
      }

      if (apogee) {
        apogeeCount++;  //keep track of how many values have been sampled since apogee
      }

      //convert altitude in a byte value
      altitude = altitude + 10;
      altitude = altitude * 2;
      if (altitude < 0) {
        altitude = 0;
      }
      if (altitude > 254) {
        altitude = 254;
      }
      altiByte[ringBufferPosition] = int(altitude); //write altitude samples into ringbuffer
      ringBufferPosition++;
      if (ringBufferPosition == 500) {
        ringBufferPosition = 0;
      }
    }
  }

  //recording has ended - save values in EEPROM
  digitalWrite(LED_PIN, HIGH);
  byte value = EEPROM.read(0);
  if (value >= 199) {
    value = 0;
  }
  value++;
  EEPROM.write(0, value); //write number of recording in address 0

  int IVin = analogRead(A6); //read battery voltage
  IVin = IVin / 4;
  uint8_t Vwrite = IVin;
  if (Vwrite == 255) {
    Vwrite = 254;
  }
  EEPROM.write(1, Vwrite); //write battery voltage in address 1

  for (int i = 2; i < 502; i++) { //write 500 samples in addresses 2...501
    ringBufferPosition++;
    if (ringBufferPosition == 500) {
      ringBufferPosition = 0;
    }
    delay(1);
    uint8_t writeVal = altiByte[ringBufferPosition];
    if (writeVal == 255) {
      writeVal = 254;
    }
    EEPROM.write(i, writeVal);
  }

  while (1) {
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
    delay(1000);
  }
}