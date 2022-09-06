/** =========================================================================
   @file 03c_SFEI_NoCellular_pump.ino
   @brief Example for SFEI pumping sampler without cellular service.
   @author Donald Yee <donald@sfei.org>

   @built on DWRINoCell by Sara Geleskie Damiano <sdamiano@stroudcenter.org>
   @
   @copyright (c) 2017-2022 Stroud Water Research Center (SWRC)
                            and the EnviroDIY Development Team
              This example is published under the BSD-3 license.

   Build Environment: Visual Studios Code with PlatformIO
   Hardware Platform: EnviroDIY Mayfly Arduino Datalogger

   DISCLAIMER:
   THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
   ======================================================================= */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>
/** End [includes] */


// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "03c_SFEI_NoCellular_pump.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "SFEI2";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 5;
// Your logger's timezone.
const int8_t timeZone = -8;  // Pacific Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 57600;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = 31;  // MCU interrupt/alarm pin to wake from sleep
// Mayfly 0.x D31 = A7
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */

// ==========================================================================
//  Sample Collection Options
// ==========================================================================
// How frequently (in minutes) to sample
const uint32_t sampleInterval = 5;
// Total pumptime (in sec) to fill bottle.  1L = 4000mL / 200ml/min = 20min *60sec/min = 1200 sec
const uint32_t totalFillSec = 600;
uint16_t remainFillSec = totalFillSec; //keeps track of time left to completed fill
// Flush pumptime (in sec) to sample
const uint32_t stdFlushSec = 30; //standard flush seconds
long adjFlushSec = stdFlushSec;  //adjusted flush seconcds
// Sip pumptime (in sec) to sample
const uint32_t stdSipSec = 30; //standard sip seconds
const uint32_t sipSec = 30; // sip seconds
long adjSipSec  = sipSec; //adjusted flush seconcds
long currDepth = 0;
// Sip minDepth (in mm on Hydros21) to sample
const long sipMinDepth = 100;
// Sip maxCond (in uS/cm on Hydros21) to sample
const uint32_t sipMaxCond = 1000;
unsigned long currTime;  // holder for current time


// Set up pins for the PUMP MOSFETs. D7 already used for Hydros21
const uint8_t  pinPump1  = 4;               // MOSFET Pump1
const uint8_t  pinSolen1  = 5;               // MOSFET Solenoid1

// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v1.1";
ProcessorStats mcuBoard(mcuBoardVersion);
/** End [processor_sensor] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);
/** End [ds3231] */


// ==========================================================================
//  Campbell OBS 3 / OBS 3+ Analog Turbidity Sensor
// ==========================================================================
/** Start [obs3] */
#include <sensors/CampbellOBS3.h>

const int8_t  OBS3Power = sensorPowerPin;  // Power pin (-1 if unconnected)
const uint8_t OBS3NumberReadings = 10;
const uint8_t ADSi2c_addr        = 0x48;  // The I2C address of the ADS1115 ADC
// Campbell OBS 3+ *Low* Range Calibration in Volts
const int8_t OBSLowADSChannel = 0;  // ADS channel for *low* range output
const float  OBSLow_A         = 0.000E+00;  // "A" value (X^2) [*low* range]
const float  OBSLow_B         = 1.000E+00;  // "B" value (X) [*low* range]
const float  OBSLow_C         = 0.000E+00;  // "C" value [*low* range]

// Create a Campbell OBS3+ *low* range sensor object
CampbellOBS3 osb3low(OBS3Power, OBSLowADSChannel, OBSLow_A, OBSLow_B, OBSLow_C,
                     ADSi2c_addr, OBS3NumberReadings);


// Campbell OBS 3+ *High* Range Calibration in Volts
const int8_t OBSHighADSChannel = 1;  // ADS channel for *high* range output
const float  OBSHigh_A         = 0.000E+00;  // "A" value (X^2) [*high* range]
const float  OBSHigh_B         = 1.000E+00;  // "B" value (X) [*high* range]
const float  OBSHigh_C         = 0.000E+00;  // "C" value [*high* range]

// Create a Campbell OBS3+ *high* range sensor object
CampbellOBS3 osb3high(OBS3Power, OBSHighADSChannel, OBSHigh_A, OBSHigh_B,
                      OBSHigh_C, ADSi2c_addr, OBS3NumberReadings);
/** End [obs3] */


// ==========================================================================
//  Meter Hydros 21 Conductivity, Temperature, and Depth Sensor
// ==========================================================================
/** Start [hydros21] */
#include <sensors/MeterHydros21.h>

const char*   hydrosSDI12address = "1";  // The SDI-12 Address of the Hydros 21
const uint8_t hydrosNumberReadings = 6;  // The number of readings to average
const int8_t  SDI12Power = sensorPowerPin;  // Power pin (-1 if unconnected)
const int8_t  SDI12Data  = 7;               // The SDI12 data pin

// Create a Meter Hydros 21 sensor object
MeterHydros21 hydros(*hydrosSDI12address, SDI12Power, SDI12Data,
                     hydrosNumberReadings);
/** End [hydros21] */


// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
Variable* variableList[] = {
  new MeterHydros21_Cond(&hydros),
  new MeterHydros21_Temp(&hydros),
  new MeterHydros21_Depth(&hydros),
  new CampbellOBS3_Turbidity(&osb3low, "", "TurbLow"),
  new CampbellOBS3_Turbidity(&osb3high, "", "TurbHigh"),
  new ProcessorStats_Battery(&mcuBoard),
  new MaximDS3231_Temp(&ds3231),
};

// All UUID's, device registration, and sampling feature information can be
// pasted directly from Monitor My Watershed.  To get the list, click the "View
// token UUID list" button on the upper right of the site page.
// Even if not publishing live data, this is needed so the logger file will be
// "drag-and-drop" ready for manual upload to the portal.

// *** CAUTION --- CAUTION --- CAUTION --- CAUTION --- CAUTION ***
// Check the order of your variables in the variable list!!!
// Be VERY certain that they match the order of your UUID's!
// Rearrange the variables in the variable list if necessary to match!
// *** CAUTION --- CAUTION --- CAUTION --- CAUTION --- CAUTION ***
/* clang-format off */
const char* UUIDs[] = {
  "12345678-abcd-1234-ef00-1234567890a1",  // Electrical conductivity (Hydros21_Cond)
  "12345678-abcd-1234-ef00-1234567890a2",  // Temperature (Hydros21_Temp)
  "12345678-abcd-1234-ef00-1234567890a3",  // Water depth (Hydros21_Depth)
  "12345678-abcd-1234-ef00-1234567890a4",  // Turbidity (Campbell_OBS3_Turb)
  "12345678-abcd-1234-ef00-1234567890a5",  // Turbidity (Campbell_OBS3_Turb)
  "12345678-abcd-1234-ef00-1234567890a6",  // Battery voltage (EnviroDIY_Mayfly_Batt)
  "12345678-abcd-1234-ef00-1234567890a7"   // Temperature (EnviroDIY_Mayfly_Temp)
};
const char* registrationToken = "12345678-abcd-1234-ef00-1234567890a8";  // Device registration token
const char* samplingFeature = "12345678-abcd-1234-ef00-1234567890a9";  // Sampling feature UUID
/* clang-format on */

// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray(variableCount, variableList, UUIDs);
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);
/** End [loggers] */


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
  for (uint8_t i = 0; i < numFlash; i++) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
    delay(rate);
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
    delay(rate);
  }
  digitalWrite(redLED, LOW);
}

// Reads the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
  if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
  return mcuBoard.sensorValues[0];
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
  // Start the primary serial connection
  Serial.begin(serialBaud);

  // Print a start-up note to the first serial port
  Serial.print(F("Now running "));
  Serial.print(sketchName);
  Serial.print(F(" on Logger "));
  Serial.println(LoggerID);
  Serial.println();

  Serial.print(F("Using ModularSensors Library version "));
  Serial.println(MODULAR_SENSORS_VERSION);

  // Set up pins for the LED's
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, LOW);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, LOW);
  // Blink the LEDs to show the board is on and starting up
  greenredflash();

  // Set up pins for the PUMP MOSFETs. D7 already used for Hydros21
  pinMode(pinPump1, OUTPUT);
  digitalWrite(pinPump1, LOW);
  pinMode(pinSolen1, OUTPUT);
  digitalWrite(pinSolen1, LOW);

  // Set the timezones for the logger/data and the RTC
  // Logging in the given time zone
  Logger::setLoggerTimeZone(timeZone);
  // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
  Logger::setRTCTimeZone(0);

  // Attach information pins to the logger
  dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                           greenLED);
  dataLogger.setSamplingFeatureUUID(samplingFeature);

  // Begin the logger
  dataLogger.begin();

  // Note:  Please change these battery voltages to match your battery
  // Set up the sensors, except at lowest battery level
  if (getBatteryVoltage() > 3.4) {
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();
  }

  // Create the log file, adding the default header to it
  // Do this last so we have the best chance of getting the time correct and
  // all sensor names correct
  // Writing to the SD card can be power intensive, so if we're skipping
  // the sensor setup we'll skip this too.
  if (getBatteryVoltage() > 3.4) {
    Serial.println(F("Setting up file on SD card"));
    dataLogger.turnOnSDcard(
      true);  // true = wait for card to settle after power up
    dataLogger.createLogFile(true);  // true = write a new header
    dataLogger.turnOffSDcard(
      true);  // true = wait for internal housekeeping after write
  }

  // Call the processor sleep
  Serial.println(F("Putting processor to sleep\n"));
  dataLogger.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
// Use this short loop for simple data logging and sending
void loop() {

  // Note:  Please change these battery voltages to match your battery
  // At very low battery, just go back to sleep
  if (getBatteryVoltage() < 3.4) {
    dataLogger.systemSleep();
  }
  // If the battery is OK, log data
  else {
    dataLogger.setRTCWakePin(-1);
    dataLogger.logData();
    //if here to jump the pumping stuff unless time matches logging interval
    //takes 10 seconds to get readings exit dataLogger so want remainder == 15
    currTime =  rtc.now().getEpoch();
    //Serial.println(currTime);
    if (currTime % (sampleInterval * 60) == 15) {
      Serial.println(currTime % (sampleInterval * 60));
      //if it gets past this it's after one of the loggingIntervals
      Serial.println("THIS IS 15sec AFTER A LOGGING INTERVAL ");
      Serial.print(" Battery V:");
      Serial.print(getBatteryVoltage());
      Serial.print(" Cond uS/cm:");
      //Serial.print(hydros.sensorValues[0]);
      Serial.print(variableList[0]->getValue());
      Serial.print(" Temp C:");
      Serial.print(variableList[1]->getValue());
      Serial.print(" Depth mm:");
      Serial.println(variableList[2]->getValue());
      //If depth > sipMinDepth && cond < sipMaxDepth
      //&& remainFillSec > stdSipSec (at least 1 sip left)
      if (variableList[2]->getValue() > sipMinDepth
          && variableList[0]->getValue() < sipMaxCond
          && remainFillSec > 10) {
            currDepth = variableList[2]->getValue();
          //limits sip fill < interval -1min Gsheet&flush -15sec SDwrite
            Serial.println(" PICK MIN of adjSipSec, nextLogTime ");
            Serial.println(sipSec*currDepth/sipMinDepth);
            Serial.println(60*sampleInterval-stdFlushSec-15);
            adjSipSec = min(stdSipSec*currDepth/sipMinDepth,60*sampleInterval-stdFlushSec-15); 
          //limits the sip fill to  max of remaining vol
            Serial.println(" PICK MIN of adjSipSec, remainFillSec ");
            Serial.println(adjSipSec);
            Serial.println(remainFillSec);
            adjSipSec = min(adjSipSec,remainFillSec); 
          //flush Pump1 on, Solen1 off
            Serial.print("Flushing Line Pump1=HIGH Solen1=LOW Sec ");
            Serial.println(stdFlushSec);
            digitalWrite(pinPump1, HIGH);
            digitalWrite(pinSolen1, LOW);
          //delay(flushmSec); // instead of delay jam LTE write in here
            Serial.println("STICK GOOGLE SHEET WRITE HERE");
          //if LTE write fast, wait remainder of stdFlushSec
            adjFlushSec = (currTime+stdFlushSec-rtc.now().getEpoch());
            if (adjFlushSec>0) {delay(adjFlushSec*1000);}
          //collect Pump1 on, Solen1 ON
            Serial.print("Filling Line Pump1=HIGH Solen1=HIGH Sec ");
        digitalWrite(pinSolen1, HIGH);
            Serial.println(adjSipSec);
            delay(adjSipSec*1000);
            remainFillSec = remainFillSec - adjSipSec;
            Serial.print(" remaining FillSec ");
        Serial.println(remainFillSec);
        //all off  Pump1 OFF, Solen1 OFF
        Serial.println("Turn all off Pump1=LOW Solen1=LOW ");
        digitalWrite(pinPump1, LOW);
        digitalWrite(pinSolen1, LOW);
      }
      else {
        Serial.println("THIS PUMP ROUND SKIPPED");
        Serial.print(" remainFillSec:");
        Serial.print(remainFillSec);
        Serial.print(" stdFlushSec:");
        Serial.print(stdFlushSec);
        Serial.print(" stdSipSec:");
        Serial.print(stdSipSec);
        Serial.print(" prev sipSec:");
        Serial.println(adjSipSec);
        Serial.print(variableList[0]->getValue());
        Serial.print("  ");
        Serial.print(variableList[1]->getValue());
        Serial.print("  ");
        Serial.print(variableList[2]->getValue());
        Serial.print("  ");
        Serial.print(variableList[3]->getValue());
        Serial.print("  ");
        Serial.print(variableList[4]->getValue());
        Serial.print("  ");
        Serial.println(variableList[5]->getValue());
      }
        dataLogger.setRTCWakePin(wakePin);
        dataLogger.systemSleep();

    }
  }

}





/** End [loop] */
