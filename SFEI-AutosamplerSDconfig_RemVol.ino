/** =========================================================================
   @file SFEI-AutosamplerSDconfig_RemVol.ino
   @brief Example for SFEI pumping sampler without cellular service.
   @author Donald Yee <donald@sfei.org>

   @built on DWRINoCell by Sara Geleskie Damiano <sdamiano@stroudcenter.org>
   @
   @copyright (c) 2017-2022 Stroud Water Research Center (SWRC)
                            and the EnviroDIY Development Team
              This example is published under the BSD-3 license.

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

// XXX ========= these includes & defines for SDcard json reads
// Include json file, move config.cpp into this local ino
#define ARDUINOJSON_USE_LONG_LONG 1
#define ARDUINOJSON_ENABLE_COMMENTS 1
#include <ArduinoJson.h>
StaticJsonDocument<1280> doc; //hoping declaring here makes it global
#include <SdFat.h>
#include <TimeLib.h>

// declare variables & starting values in case SD doesn't read
const char *boxName = "SFEIn";
int pumpInterval = 15;
float inletVol = 100.0;
float pumpHeight = 2000.0;
// nValves for 1pump system. =0 for all pump system
int8_t nValve = 2; 
const char *pumpName[5] = {"flush", "bot1", "bot2", "bot3", "bot4"};
int8_t pumpPin[5] = { 4, 5, 6, 9, 10};
long starTime[5] = { 1000111222, 1675152001, 1675152002, 1675152003, 1675152004};
long stopTime[5] = { 1999888777, 1675756801, 1675756802, 1675756803, 1675756804};
// maxVol is max volume of bottle in mL. vol <10mL will get bypassed
float maxVol[5] = { 0, 1991, 1.2, 1.3, 1.4};
// minVol is min volume of acceptable sample
float minVol[5] = { 0, 1111, 0.2, 0.3, 0.4};
float minCond[5] = { 0, 1, 2, 10003, 20004};
float maxCond[5] = { 99888,1001,1002,9993,9994};
float minDepth[5] = { 300, 301, 302, 303, 304};
float maxDepth[5] = { 0, 2001, 2002, 2003, 2004};
float stdSip[5] = { 100.0, 100.1, 100.2, 100.3, 100.4};
// pumpSpeed is constant in 1 pump system, entries for valves are just
// to indicate flow speed to that bottle.
float pumpSpeed[5] = { 7.50, 7.501, 7.502, 7.503, 7.504};
//========== end of initial variable setup
// XXX 


// ==========================================================================
//  Sample Collection Options
// ==========================================================================
// pumpInterval How frequently (in minutes) to sample
// inletVol = 100.0; help estimate time of flush tubing: 1m of 6mm tube is ~30mL
// pumpHeight = 2000.0; pumpHeight in mm
// starTime & stopTime in epoch seconds
// Total remain volume to fill bottle
float remainVol[5] = {0};
// Standard Flush vol (in mL) = stdSip[0]
float sipVol[5] = {100};
float sipSec[5] = {100};
float stdFlushSec = 60; //standard flush seconds
float adjFlushSec = stdFlushSec;  //adjusted flush seconcds
// Sip pumptime (in sec) to sample
float stdSipSec = 60; //standard sip seconds
float adjSipSec  = stdSipSec; //adjusted flush seconcds
// Sip minDepth maxDepth (in mm on Hydros21) to sample
// Sip minCond maxCond (in uS/cm on Hydros21) to sample
unsigned long currTime;  // holder for current time
float currCond = 0;
float currTemp = 0;
float currDepth = 0;



// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "SFEI-AutosamplerSDconfig.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
char* LoggerID = boxName;
// How frequently (in minutes) to log data
uint8_t loggingInterval = 5;
// Your logger's timezone.
int8_t timeZone = -8;  // Pacific Standard Time
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

// =XXX=========================================================================
//  Calculated RemVol1 "Sensor"
// ==========================================================================
/** Start [remaining volume] */
// Create the function to calculate the remain vol
// remain vol= totVol for a bottle - vol pumped so far
float calculateRemVol1(void) {
    float remVol = -9999;
    remVol = remainVol[1];
    return remVol;
}
float calculateRemVol2(void) {
    float remVol = -9999;
    remVol = remainVol[2];
    return remVol;
}
float calculateRemVol3(void) {
    float remVol = -9999;
    remVol = remainVol[3];
    return remVol;
}
float calculateRemVol4(void) {
    float remVol = -9999;
    remVol = remainVol[4];
    return remVol;
}

// Properties of the calculated remain Vol variable
const char* remVolVarName = "volume";  
        // Must be a value in http://vocabulary.odm2.org/variablename/
const char* remVolVarUnit = "mL";  
        // Must be a value in http://vocabulary.odm2.org/units/
int         remVolVarResolution = 0;
const char* remVol1UUID          = "12345678-abcd-1234-ef00-1234567890a6";
const char* remVol1VarCode       = "remVol1";
const char* remVol2UUID          = "12345678-abcd-1234-ef00-1234567890a7";
const char* remVol2VarCode       = "remVol2";
const char* remVol3UUID          = "12345678-abcd-1234-ef00-1234567890a8";
const char* remVol3VarCode       = "remVol3";
const char* remVol4UUID          = "12345678-abcd-1234-ef00-1234567890a9";
const char* remVol4VarCode       = "remVol4";

// Create the calculated remain vol variable objects and return variable
// pointers
Variable* calcRemVol1 = new Variable(
    calculateRemVol1, remVolVarResolution, remVolVarName,
    remVolVarUnit, remVol1VarCode, remVol1UUID);
Variable* calcRemVol2 = new Variable(
    calculateRemVol2, remVolVarResolution, remVolVarName,
    remVolVarUnit, remVol2VarCode, remVol2UUID);
Variable* calcRemVol3 = new Variable(
    calculateRemVol3, remVolVarResolution, remVolVarName,
    remVolVarUnit, remVol3VarCode, remVol3UUID);
Variable* calcRemVol4 = new Variable(
    calculateRemVol4, remVolVarResolution, remVolVarName,
    remVolVarUnit, remVol4VarCode, remVol4UUID);
/** End [remaining volume]XXX */


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
  calcRemVol1,/*XXX*/
  calcRemVol2,/*XXX*/
  calcRemVol3,/*XXX*/
  calcRemVol4,/*XXX*/
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
  "12345678-abcd-1234-ef00-1234567890a4",  // Battery voltage (EnviroDIY_Mayfly_Batt)
  "12345678-abcd-1234-ef00-1234567890a5",   // Temperature (EnviroDIY_Mayfly_Temp)
  remVol1UUID,  // RemVolOne)/*XXX*/
  remVol2UUID,  // RemVolTwo)/*XXX*/
  remVol3UUID,  // RemVolThree)/*XXX*/
  remVol4UUID,  // RemVolFour)/*XXX*/
//  "12345678-abcd-1234-ef00-1234567890a7"  // Turbidity (Campbell_OBS3_Turb)

};
const char* registrationToken = "12345678-abcd-1234-ef00-1234567890aa";  // Device registration token
const char* samplingFeature = "12345678-abcd-1234-ef00-1234567890ab";  // Sampling feature UUID
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
//  ========== Flashes the LED's on the primary board ========== 
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

// ========== Reads the battery voltage ========== 
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
  if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
  return mcuBoard.sensorValues[0];
}


// XXX ========== Reads SDcard file ========== 
const char *filename = "config.txt"; // <- SD library uses 8.3 filenames
SdFat sdFat;
void setupConfig()
{
    Serial.print(F("Initializing sd library\n"));
    // Initialize SD library
    while (!sdFat.begin(sdCardSSPin, SPI_FULL_SPEED))
    {
        PRINTOUT(F("Failed to initialize SD library..."));
        delay(1000);
    }
}
// XXX 

// XXX ========== Prints the content of a file to the Serial ========== 
void printFile(const char *filename)
{
    PRINTOUT(F("printFile opening file"), filename);
    File file = sdFat.open(filename);
    if (!file)
    {
        PRINTOUT(F("Failed to read file"));
        return;
    }
    while (file.available())
    {
        Serial.print((char)file.read());
    }
    file.close();
}
// XXX 


// XXX ========== Parse a JsonDocument ========== 
// Don't forget to change the capacity to match your requirements.
// Use arduinojson.org/v6/assistant to compute the capacity.
void loadConfiguration(const char *filename)
{
    setupConfig();
    printFile(filename);
    File file = sdFat.open(filename);

    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
        PRINTOUT(F("Failed to read configuration file: "), error.c_str());
    }

    Serial.println("printing from inside loadConfiguration");
    int pumpInterval = doc["pumpInterval"];    
    Serial.print( pumpInterval );
    Serial.println(" pumpInterval ");

    file.close();
  return;
}
// XXX

// XXX========== calc Pump[N] volume based on conditions ========== 
void pumpVol(int8_t N, float nowTime, float nowCond, 
                float nowTemp, float nowDepth)
{
  //set sipVol to 0 unless it passes all conditions
  sipVol[N] = 0;
  if (nowTime < starTime[N]) {Serial.print("<starTime ");return;}
  if (nowTime > stopTime[N]) {Serial.print(">stopTime ");return;}  
  if (nowCond < minCond[N]) {Serial.print("<minCond ");return;}  
  if (nowCond > maxCond[N]) {Serial.print(">maxCond ");return;}  
  if (nowDepth < minDepth[N]) {Serial.print("<minDepth ");return;}  
  if (nowDepth > maxDepth[N]) {Serial.print(">maxDepth ");return;}  
  //  dont' bother collecting <30mL
  if (30 > remainVol[N]) {return;}  
  
  //if passed gauntlet set sipVol adjusted to level
  //simple linear can be adjusted in future
  sipVol[N] = stdSip[N] * nowDepth / minDepth[N]; 
  //adjust sipVol if remainVol or maxVol/4 is smaller
  sipVol[N] = min(sipVol[N],remainVol[N]);
  return;
  
}
// XXX

// XXX========== calc Pump[N] seconds based on conditions ========== 
//void pumpSec(int8_t N, float nowTime, float nowCond, 
//                float nowTemp, float nowDepth)
//{
  //set sipSec to 0 
//  sipSec[N] = 0;
//  if (30 > sipVol[N]) {return;}  
  // assuming pumpSpeed at max
//  adjSipSec = sipVol[N]/pumpSpeed[N];
  // adjust based on currDepth, 1+0 if at maxDepth
  // 1+1 if currDepth ~0, can add empirical tweak factor
//  sipSec[N] = adjSipSec * (1+ (maxDepth[N]/nowDepth)/minDepth[N] ); 
//  return;
//}
// XXX

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

//=============================================================
// XXX Json SDcard config read, if it fails hardcorded defaults  
  loadConfiguration("/config.txt"); 
 
  Serial.println("printing after loadConfiguration in setup");
  boxName = doc["boxName"];
  pumpInterval = doc["pumpInterval"];
  inletVol = doc["inletVol"];
  pumpHeight = doc["pumpHeight"];
  nValve = doc["nValve"];

  Serial.print(boxName);   
  Serial.println("  boxName "); 
  Serial.print(pumpInterval);   
  Serial.println("  pumpInterval "); 
  Serial.print(inletVol);   
  Serial.println("  inletVol "); 
  Serial.print(pumpHeight);   
  Serial.println("  pumpHeight "); 
  Serial.print(nValve );   
  Serial.println("  nValve "); 

  //this replaces starting array values with SDcard values
  for (int i=0; i<5; i++) {
    pumpName[i] = doc["pumpName"][i];
    pumpPin[i] = doc["pumpPin"][i];  
    starTime[i] = doc["starTime"][i];  
    stopTime[i] = doc["stopTime"][i];  
    maxVol[i] = doc["maxVol"][i];  
    minVol[i] = doc["minVol"][i];  
    minCond[i] = doc["minCond"][i];  
    maxCond[i] = doc["maxCond"][i];  
    minDepth[i] = doc["minDepth"][i];  
    maxDepth[i] = doc["maxDepth"][i];  
    stdSip[i] = doc["stdSip"][i];  
    pumpSpeed[i] = doc["pumpSpeed"][i];  
    // this sets the initial bottle remainVol to maxVol
    remainVol[i] = maxVol[i] ;  
  }
  
  Serial.println("********** this is all pumps ");   
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(pumpName[i]);}   
  Serial.println("  pumpName "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(pumpPin[i]);}
  Serial.println("  pumpPin "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(starTime[i]);}   
  Serial.println("  starTime "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(stopTime[i]);}   
  Serial.println("  stopTime "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(maxVol[i]);}   
  Serial.println("  maxVol "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(minVol[i]);}   
  Serial.println("  minVol "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(minCond[i]);}   
  Serial.println("  minCond "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(maxCond[i]);}   
  Serial.println("  maxCond "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(minDepth[i]);}   
  Serial.println("  minDepth "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(maxDepth[i]);}   
  Serial.println("  maxDepth "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(stdSip[i]);}   
  Serial.println("  stdSip "); 
  for (int i=0; i<5; i++) {Serial.print(" "); Serial.print(pumpSpeed[i]);}   
  Serial.println("  pumpSpeed "); 

  Serial.println("does anything print inside setup");
  LoggerID = boxName;
  
  // Set up pins for the LED's
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, LOW);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, LOW);
  // Blink the LEDs to show the board is on and starting up
  greenredflash();

  // Set up pins for the PUMP MOSFETs. D7 already used for Hydros21
  for (int i=0; i<5; i++) {
    pinMode(pumpPin[i], OUTPUT);
    digitalWrite(pumpPin[i], LOW);
  }
  
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
    //turn off all solenoids;
    for (int i=0; i<5; i++) {
      digitalWrite(pumpPin[i], LOW);
    }
    dataLogger.systemSleep();
  }
  // If the battery is OK, log data
  else {
      //dataLogger.setRTCWakePin(-1);
    currTime =  rtc.now().getEpoch();
    dataLogger.logData();
    //if here to jump the pumping stuff unless time matches logging interval
    //takes 10 seconds to get readings exit dataLogger so want remainder == 15
      
    //XXX turn off all solenoids;
    if (currTime % 60 < 1) {     
      for (int i=0; i<5; i++) {
        digitalWrite(pumpPin[i], LOW);
      }
    }

    //Serial.println(currTime);        
    if (currTime % (pumpInterval * 60) < 30) {
      currCond = hydros.sensorValues[2];
      currTemp = hydros.sensorValues[1];
      currDepth = hydros.sensorValues[0];
      Serial.println(" got new curr Values ");    

      Serial.print(" LoggerID "); 
      Serial.print(LoggerID); 
      Serial.println("datetime,HydrosEC,temp,depth,battV,brdTemp,bot1-4RemVols");    
      Serial.print(currTime % (pumpInterval * 60));
      //if it gets past this it's after one of the loggingIntervals
      Serial.println(" sec AFTER A LOGGING INTERVAL ");
      Serial.print(" Battery V:");
      Serial.print (getBatteryVoltage());
      Serial.print("   Cond uS/cm:");
      Serial.print(currCond); //current Cond
      //Serial.print(variableList[0]->getValue()); //last Cond
      Serial.print("   Temp C:");
      Serial.print(currTemp); //current Temp
      //Serial.print(variableList[1]->getValue()); //last Temp
      Serial.print("   Depth mm:");
      Serial.println(currDepth); //current Depth
      //Serial.println(variableList[2]->getValue()); //last Depth
      Serial.println("hydros.sensorValues[0-2]");
      
      /// this decides whether or not pump[i] runs this interval
      // pumpVol calc for non-flush pumps this interval
      float sumVol = 0;
        for (int i=1; i<5; i++) {
          pumpVol(i, currTime, currCond, currTemp, currDepth);
          sumVol += sipVol[i] ;        
          Serial.print(sipVol[i]);
          Serial.print(" mL sipVol pump ");
          Serial.print(i);
          Serial.print(" , ");
          }
        Serial.println(".");

      //if sumVol > 30mL collect, so need to run flush pump[0]
      sipVol[0] = 0;
      if (sumVol > 30) {sipVol[0] = inletVol;}
              
      //All should be off already,but turn off all solenoids;
      for (int i=0; i<5; i++) {digitalWrite(pumpPin[i], LOW);}

      // flush sec adjustment- if currDepth = maxDepth, adds 0
      // if currDepth = 0, pump time doubles, can add tweak factor
      // only need sipSec locally in loop. actual tracking is volume
      sipSec[0]=  sipVol[0]/pumpSpeed[0] *
                (1+ (maxDepth[0]/currDepth)/minDepth[0] );
      adjFlushSec = sipSec[0];

      // figure out the time the rest of the bottles need
      // adjust based on currDepth, 1 if at pumpHeight
      // multiply sipSec by heightFactor
      float heightFactor;
      heightFactor = 1/(1-0.00033*(pumpHeight-currDepth));
      Serial.print(" heightFactor multiply sipSec by ");
      Serial.println(heightFactor);
      float sumSec = 0;
        for (int i=1; i<5; i++) {
          //set sipSec to 0 
          sipSec[i] = 0;
          if (30 > sipVol[i]) {sipVol[i]=0;}  
          // assuming pumpSpeed at max
          adjSipSec = sipVol[i]/pumpSpeed[i];   
          sipSec[i] = adjSipSec*heightFactor; 
          sumSec += sipSec[i] ;
        }

      // figure out remaining time after flush
      int intervalSec;
      intervalSec = (pumpInterval-1)*60-adjFlushSec;
        if (intervalSec < sumSec) {
        // we need to squeeze sipSec and sipVol to fit
          float adjFactor = intervalSec / sumSec;
          for (int i=1; i<5; i++) {
            sipSec[i] = sipSec[i] * adjFactor;
            sipVol[i] = sipVol[i] * adjFactor;
          }
          sumSec = adjFactor * sumSec;
        }
        
      // Now we do actual pumping 
      // start flush pump[0] on
      digitalWrite(pumpPin[0], HIGH);
      Serial.println("start of flush");
      delay (adjFlushSec*1000);
      Serial.println("end of flush");
      //  turns off flush pump if all pump system
      if (nValve == 0){digitalWrite(pumpPin[0], LOW);}
      
      // now cycle through all the pumps or valves
      // this is setup for system where HIGH = flow to bottle
        for (int i=1; i<5; i++) {
          digitalWrite(pumpPin[i], HIGH);
          Serial.print("pump "); 
          Serial.print(i);
          Serial.print(" filling sipVol ");
          Serial.print(sipVol[i]);
          Serial.print(", ");
          delay (sipSec[i]*1000);          
          digitalWrite(pumpPin[i], LOW);
          // increment down remaining volume
          remainVol[i] = remainVol[i] - sipVol[i];
        }
        Serial.println("Pump DONE ");  
       //  turns off flush pump[0] after all valves
       //  if it was nValve==0 all pump system, low already       
        digitalWrite(pumpPin[0], LOW);      
      }

//      dataLogger.setRTCWakePin(wakePin);
//      dataLogger.systemSleep();

    
  }
  /** end of check of battery level to log/pump options */
    
  if (currTime % 60 == 0 ) {
    Serial.print(currTime%(pumpInterval * 60));
    Serial.print(" sec past pumpInterval ");
    delay(100);
    //Serial.println(F("Putting processor to sleep\n"));
//  rtc.enableInterrupts(EveryMinute);
//  dataLogger.systemSleep();
  }

}
/** End [loop] */
