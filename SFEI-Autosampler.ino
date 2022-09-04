/** =========================================================================
   @file DRWI_NoCellular.ino
   @brief Example for DRWI CitSci without cellular service.

   @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
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

#include "LoggerModem.h"
#include "dataPublisherBase.h"

const int8_t THREE_WAY_VALVE_PIN = 8;
const int8_t PUMP_PIN = 9;

// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "DRWI_NoCellular.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "SFEI2";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
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

const uint8_t ADSi2c_addr        = 0x48;  // The I2C address of the ADS1115 ADC

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
  "12345678-abcd-1234-ef00-1234567890a1",  // Electrical conductivity (Decagon_CTD-10_Cond)
  "12345678-abcd-1234-ef00-1234567890a2",  // Temperature (Decagon_CTD-10_Temp)
  "12345678-abcd-1234-ef00-1234567890a3",  // Water depth (Decagon_CTD-10_Depth)
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

class PumpingPublisher : public dataPublisher {
  public:
    int16_t publishCount = 0;

    explicit PumpingPublisher() {}
    explicit PumpingPublisher(Logger& baseLogger, uint8_t sendEveryX = 1,
                              uint8_t sendOffset = 0)
      : dataPublisher(baseLogger, sendEveryX, sendOffset) { }
    String getEndpoint(void) override {
      return String("Pumping Publisher");
    }

    int16_t getPublishCount();

    int16_t publishData(Client* outClient) override {};

    int16_t publishData() override {
      Serial.println("publishing...");
      publishCount++;
    }
};

class NullLoggerModem : public loggerModem {
  public:
    explicit NullLoggerModem(boolean x) :
      loggerModem(0, 0, false, 0,
                  false, false, 0,
                  false, 0, 0,
                  0, 0,
                  0) {}

    bool isModemAwake() override {
      return true;
    }
    bool extraModemSetup() override { }
    bool modemWakeFxn() override {}
    bool modemSleepFxn() override {}
    bool isInternetAvailable() override {}
    float getModemChipTemperature() override {}
    bool getModemBatteryStats(uint8_t& chargeState, int8_t& percent,
                              uint16_t& milliVolts) {
      return false;
    }
    bool getModemSignalQuality(int16_t& rssi, int16_t& percent) {
      return false;
    }
    uint32_t getNISTTime() {
      return 0;
    }
    void disconnectInternet() {}
    bool connectInternet() {
      return true;
    }
    bool connectInternet(uint32_t maxConnectionTime = 50000L) {
      return true;
    }
    bool modemWake() {
      return true;
    }

};

int16_t PumpingPublisher::getPublishCount() {
  return publishCount;
}

PumpingPublisher pumpingPublisher(dataLogger);
NullLoggerModem nullLoggerModem(true);


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {

  dataLogger.registerDataPublisher(&pumpingPublisher);
  dataLogger.attachModem(nullLoggerModem);
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

  // Setup PIN for Solenoid and motor
  pinMode(THREE_WAY_VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

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
  //    Serial.println(F("Putting processor to sleep\n"));
  //    dataLogger.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
// Use this short loop for simple data logging and sending
int16_t lastPublishCount = 0;
void loop() {
  // Note:  Please change these battery voltages to match your battery
  // At very low battery, just go back to sleep
  if (getBatteryVoltage() < 3.4) {
    dataLogger.systemSleep();
  }
  // If the battery is OK, log data
  else {
    dataLogger.logDataAndPublish();
    pumpingPublisher.getPublishCount();
    if (pumpingPublisher.publishCount > lastPublishCount) {
      Serial.print(lastPublishCount);
      Serial.print(": values: ");
      Serial.println(variableList[0]->getValue(), 3);
      Serial.println(variableList[1]->getValue(), 3);
      Serial.println(variableList[2]->getValue(), 3);
      lastPublishCount = pumpingPublisher.publishCount;
    }
  }
}
/** End [loop] */
