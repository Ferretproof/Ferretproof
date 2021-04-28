#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include <TaskScheduler.h>


#define ALL    1000
#define TRACE   600
#define DEBUG   500
#define INFO    400
#define WARN    300
#define ERROR   200
#define FATAL   100
#define OFF       0

#define logLevel DEBUG

// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN    // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass 
//#define _TASK_STATUS_REQUEST     // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations

#define _TASK_TIMEOUT

#define D0 16   // IO
#define D1 5    // IO, SCL
#define D2 4    // IO, SDA
#define D3 0    // IO, 10k Pull-up
#define D4 2    // IO, 10k Pull-up, build in LED
#define D5 14   // IO, SCK
#define D6 12   // IO, MISO
#define D7 13   // IO, MOSI
#define D8 15   // IO, 10k Pull-down, SS

#define buildInLed D4

#ifndef STASSID
#define STASSID "Not Found"
#define STAPSK  "Tv Radio Water Fiets Auto Boot Stuur"
#endif

#define RELAY1  D5
#define RELAY2  D6
#define RELAY3  D7
#define RELAY4  D8

#define relayOn true
#define relayOff false

#define veryShortDelay 100
#define shortDelay     250
#define mediumDelay    1000
#define longDelay      2000
#define veryLongDelay  5000

#define Red   D0
#define Green D4
#define Yellow -1

#define ledOff     0
#define ledOn      1
#define ledToggle  2

#define ONE_WIRE_BUS D3  // D5


OneWire oneWire(ONE_WIRE_BUS  );
DallasTemperature sensors(&oneWire);

#define numberOfSensors  2;

float* getTemperatures(float[]);
float *temperatures;
float temperatureArray[3] = {0.0, 0.0, 0.0};

float criticalTemperature[3] = {31.0, 31.0, 31.0};
float warningTemperature[3]  = {29.0, 29.0, 29.0};
float okTemperature[3]       = {27.0, 27.0, 27.0};

int glitchProtection = 0;   // should prevent problems when sensor hits 85C (this is usually a short glitch).

boolean ledStatus = false;  // off

const char* ssid = STASSID;
const char* password = STAPSK;

void determineAlertStatus();
void determineAlertStatusDisabled();

void flashRedLed();
void flashRedLedDisabled();

void flashGreenLed();
void flashGreenLedDisabled();

void flashYellowLed();
void flashYellowLedDisabled();

void relayGymnastics();

void setLedStatus(int, int);

void alarmLed();
void alarmLedDisable();

void readTemp();
void readTempDisable();

void switchRelay(int, bool);

void switchPowerRelays(int);

void oled(String, float*);

void log(int,  String);
void logNL(int,  String);

Scheduler ts;

#define statusOK        1
#define statusWARNING   2
#define statusFAILURE   3
#define statusCRITICAL  4
#define statusUNKNOWN   -1

int previousStatus = statusUNKNOWN;

Task task_determineAlertStatus(5 * TASK_SECOND, TASK_FOREVER, &determineAlertStatus, &ts, false, NULL, &determineAlertStatusDisabled);

void critical_ON();
void critical_OFF();

void failure_ON();
void failure_OFF();

void warning_ON();
void warning_OFF();


Task tCritical ( shortDelay * TASK_MILLISECOND, TASK_FOREVER, &critical_ON, &ts, false );
Task tFailure ( mediumDelay * TASK_MILLISECOND, TASK_FOREVER, &failure_ON, &ts, false );
Task tWarning ( mediumDelay * TASK_MILLISECOND, TASK_FOREVER, &warning_ON, &ts, false );


void setup() {
  Serial.begin(115200);
  logNL(ALL, ""); logNL(ALL, "");
  logNL(ALL, "/----------------------------------------------------------\\");
  logNL(ALL, "| Starting...                                              |");
  logNL(ALL, "\\----------------------------------------------------------/");

  pinMode(Red,    OUTPUT);
  pinMode(Green,  OUTPUT);

  pinMode(RELAY1,  OUTPUT); switchRelay(RELAY1, relayOff);  
  pinMode(RELAY2,  OUTPUT); switchRelay(RELAY2, relayOff);  
  pinMode(RELAY3,  OUTPUT); switchRelay(RELAY3, relayOff);  
  pinMode(RELAY4,  OUTPUT); switchRelay(RELAY4, relayOff);  
  
  setLedStatus(Red, ledOn);
  setLedStatus(Green, ledOn);

  for (int i=0; i<5; i++)
  { 
    temperatures    = getTemperatures(temperatureArray);
    printTemperature(temperatures);
    delay(500);
  }
  
  task_determineAlertStatus.enable();
}

void loop() {
  ts.execute();
}

void flashRedLed(){
  setLedStatus(Red, ledToggle);
}

void flashRedLedDisabled(){
  setLedStatus(Red, ledOff);
}


void printTemperature(float sensorTemperature[]){
  int nrOfSensors = numberOfSensors;
  for(int i=0; i<nrOfSensors; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" temp: ");
    Serial.print(temperatures[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void setLedStatus(int color, int status) {

  bool newStatus = false;

  if (status == ledOff)    { newStatus = false;               }
  if (status == ledOn)     { newStatus = true;                }
  if (status == ledToggle) { newStatus = !digitalRead(color); }

  if (color == Yellow) 
  {
    if (status == ledToggle)
    {
      digitalWrite(Red,   !digitalRead(Red));
      digitalWrite(Green, !digitalRead(Red));
    }
    else
    {
      digitalWrite(Red,   newStatus);
      digitalWrite(Green, newStatus);  
    }
  }
  else
  {
    digitalWrite(color, newStatus);
  }
}

void switchRelay(int relay, bool status) {
  Serial.println("switchRelay::");
  digitalWrite(relay, status);
}

void relayGymnastics() {
  switchRelay(RELAY1, relayOff);
  switchRelay(RELAY2, relayOff);
  switchRelay(RELAY3, relayOff);
  switchRelay(RELAY4, relayOff);

  for(int i=0; i<2; i++)
  {
    switchRelay(RELAY1, relayOn);
    delay(veryShortDelay);
    switchRelay(RELAY1, relayOff);
    switchRelay(RELAY2, relayOn);
    delay(veryShortDelay);
    switchRelay(RELAY2, relayOff);
    switchRelay(RELAY3, relayOn);
    delay(veryShortDelay);
    switchRelay(RELAY3, relayOff);
    switchRelay(RELAY4, relayOn);
    delay(veryShortDelay);
    switchRelay(RELAY4, relayOff);
    delay(veryShortDelay);
  }
}

float* getTemperatures(float sensorTemperature[]) {
  int nrOfSensors= numberOfSensors;

  sensors.requestTemperatures(); // Send the command to get temperatures
  for (int i=0; i<nrOfSensors; i++)
  {
    sensorTemperature[i] = sensors.getTempCByIndex(i);
  }
  
  return sensorTemperature;
}

// ----------------------------------------------------------------------

// ----------------------------------------------------------------------


void determineAlertStatus() {
  int nrOfSensors           = numberOfSensors;
  int newStatus             = statusUNKNOWN;
  int okStatusLevel         = 0;
  int warningStatusLevel    = 0;
  int criticalStatusLevel   = 0;
  int failureStatusLevel    = 0;
  String statusText         = "?";
  int i                     = 0; 
  
  Serial.println("determineAlertStatus:: start");
  temperatures    = getTemperatures(temperatureArray);
  printTemperature(temperatures);

  for (i=0; i<nrOfSensors; i++)
  {
    float temp = temperatures[i];
 /*
  *                              s0      s1       s2     s3          s4
                                 OK      WARN    CRIT    FAILURE     UNKNOWN --> previous status
                                 -------------------------------------------
t0   t = Fail                    FAIL    FAIL    CRIT    FAIL        FAIL    --\
t1   t > critical                CRIT    CRIT    CRIT    CRIT        CRIT       \
t2   t > warning, t <= critical  WARN    WARN    CRIT    FAIL        WARN        -> new status
t3   t > ok, t<= warning         OK      WARN    CRIT    FAIL        WARN       /
t4   t <= ok                     OK      OK      CRIT    FAIL        OK      --/

    
    to go to the OK status, all Sensors should match the conditions.
    To go to any other status, just one sensor has to match the condition.
*/

    if (temp == 85){
      glitchProtection++;
      if (glitchProtection < 3)
      {   break;    }
    }
    else {   glitchProtection = 0;    }

    bool t0 = (temp < -120);
    bool t1 = (temp > criticalTemperature[i]);
    bool t2 = ((temp > warningTemperature[i]) && (temp <= criticalTemperature[i])) ;
    bool t3 = ((temp > okTemperature[i]) && (temp <= warningTemperature[i]));
    bool t4 = (temp <= okTemperature[i]);

    bool s0 = (previousStatus == statusOK);
    bool s1 = (previousStatus == statusWARNING);
    bool s2 = (previousStatus == statusCRITICAL);
    bool s3 = (previousStatus == statusFAILURE);
    bool s4 = (previousStatus == statusUNKNOWN);

    Serial.print("s0: ");
    Serial.print(s0);
    
    Serial.print("  s1: ");
    Serial.print(s1);

    Serial.print("  s2: ");
    Serial.print(s2);
    
    Serial.print("  s3: ");
    Serial.print(s3);

    Serial.print("  s4: ");
    Serial.println(s4);


    if (t0 && (s0 || s1 || s3 || s4))     {    failureStatusLevel  = 1;  break;   }
    if (t0 && !s2)                        {    criticalStatusLevel = 1;  break;   }
    
    if (t1)                               {    criticalStatusLevel = 1;  break;   }
    
    if (t2 && (!s2 && !s3 && !s4))        {    warningStatusLevel  = 1;           }
    if (t2 && s2)                         {    criticalStatusLevel = 1;  break;   }
    if (t2 && s3)                         {    failureStatusLevel  = 1;           }
    if (t2 && s4)                         {    warningStatusLevel  = 1;           }
    
    if (t3 && s0)                         {    okStatusLevel++;                   }
    if (t3 && s1)                         {    warningStatusLevel  = 1;           }
    if (t3 && s2)                         {    criticalStatusLevel = 1;  break;   }
    if (t3 && s3)                         {    failureStatusLevel  = 1;           }
    if (t3 && s4)                         {    warningStatusLevel  = 1;           }
  
    if (t4 && (s0 || s1 || s4))           {    okStatusLevel++;                   }
    if (t4 && s2)                         {    criticalStatusLevel = 1;  break;   }
    if (t4 && s3)                         {    failureStatusLevel  = 1;           }
  }
  
  if (criticalStatusLevel != 0)
  {
    Serial.println(" == CRITICAL ==");
    newStatus = statusCRITICAL;
  }
  else if (failureStatusLevel != 0)
  {
    Serial.println(" == FAILURE =="); 
    newStatus = statusFAILURE;
  }
  else if (warningStatusLevel != 0)
  {
    Serial.println(" == WARNING ==");
    newStatus = statusWARNING;
  }
  else if (okStatusLevel == nrOfSensors)
  {
    Serial.println(" == OK ==");
    newStatus = statusOK;
  }
  else
  {
    Serial.println(" == UNKNOWN ==");
    newStatus = statusUNKNOWN;
  }

  if (newStatus != previousStatus)
  {
    LEDOff(Red);
    LEDOff(Green);
    tCritical.disable();
    tFailure.disable();
    tWarning.disable();
  }
  if (newStatus == statusCRITICAL)
  {
    tCritical.enableIfNot();
    statusText = "CRITICAL";
    switchPowerRelays(relayOff);
  }
  else if (newStatus == statusFAILURE)
  {
    tFailure.enableIfNot();
    statusText = "Failure";
    switchPowerRelays(relayOff);
  }
  else if (newStatus == statusWARNING)
  {
    LEDOn(Yellow);
    statusText = "Warning";
    switchPowerRelays(relayOn);
  }
  else if (newStatus == statusOK)
  {
    LEDOn(Green);
    statusText = "OK";
    switchPowerRelays(relayOn);
  }
  else
  {
    LEDOn(Red);
    statusText = "INTERNAL ERROR";
    switchPowerRelays(relayOff);
  }

  oledData(statusText, temperatures);

  previousStatus = newStatus;
}

// ----------------------------------------------------------------------

void determineAlertStatusDisabled() {
  Serial.println("determineAlertStatusDisabled:: start");
}

void oledData(String statusText, float temperatures[])
{
  Serial.println("STATUS: " + statusText);
  printTemperature(temperatures);
}

void switchPowerRelays(int relayStatus) {
  switchRelay(RELAY1, relayStatus);
  switchRelay(RELAY2, relayStatus);
  switchRelay(RELAY3, relayStatus);
  switchRelay(RELAY4, relayStatus);
}




inline void LEDOn(int color) {
  if (color == Yellow)
  {
    digitalWrite( Red, HIGH );
    digitalWrite( Green, HIGH );
  }
  digitalWrite( color, HIGH );
}

inline void LEDOff(int color) {
  if (color == Yellow)
  {
    digitalWrite( Red, LOW);
    digitalWrite( Green, LOW );
  }
  digitalWrite( color, LOW );
}

void critical_ON() {
  LEDOn(Red);
  tCritical.setCallback( &critical_OFF );
}

void critical_OFF() {
  LEDOff(Red);
  tCritical.setCallback( &critical_ON );
}

void failure_ON() {
  LEDOn(Yellow);
  tFailure.setCallback( &failure_OFF );
}

void failure_OFF() {
  LEDOff(Yellow);
  tFailure.setCallback( &failure_ON );
}

void warning_ON() {
  LEDOn(Green);
  tWarning.setCallback( &warning_OFF );
}

void warning_OFF() {
  LEDOff(Green);
  tWarning.setCallback( &warning_ON );
}



void log(int requestedLogLevel,  String text)
{
  if (requestedLogLevel >= logLevel)
  {
    Serial.print(text);
  }
}

void logNL(int requestedLogLevel,  String text)
{
  text = text + "\n";
  log(requestedLogLevel, text);
}
