#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include <TaskScheduler.h>
#include "Arduino_DebugUtils.h"
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"


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

#define numberOfSensors  3;

float* getTemperatures(float[]);
float *temperatures;
float temperatureArray[4] = {0.0, 0.0, 0.0, 0.0};

float criticalTemperature[4] = {31.0, 31.0, 31.0, 31.0};
float warningTemperature[4]  = {29.0, 29.0, 29.0, 29.0};
float okTemperature[4]       = {27.0, 27.0, 27.0, 27.0};
String sensorLocation[4]     = {"top", "noz", "pwr", "cpu"};

int glitchProtection = 0;   // should prevent problems when sensor hits 85C (this is usually a short glitch). - not implemented 20210430

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

void printDisplay(String, int);
void printDisplay(String, int, int);
Task tCritical (  shortDelay * TASK_MILLISECOND, TASK_FOREVER, &critical_ON, &ts, false );
Task tFailure  ( mediumDelay * TASK_MILLISECOND, TASK_FOREVER, &failure_ON,  &ts, false );
Task tWarning  ( mediumDelay * TASK_MILLISECOND, TASK_FOREVER, &warning_ON,  &ts, false );

int screenwidth = 126;
int screenheight = 64;
int textPositionLeft    = 0;
int textPositionRight   = 70;
int textPositionCenter  = 20;

SSD1306Wire display(0x3c, D2, D1);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h



void setup() {
  Serial.begin(115200);
  Debug.timestampOff();
  
  
  display.init();

//  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.display();
  
  printDisplay("Booting...", 0, 0);
  

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Debug.print(DBG_ERROR, "WIFI Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("Fireproof");

  // No authentication by default
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3

  // generated using echo -n 'the password' | md5sum.exe
  //ArduinoOTA.setPasswordHash("5163ef35d45511844f246a691356dea4");

  ArduinoOTA.onStart([]()   {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) 
    {
      type = "sketch";
    }
    else 
    { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("OTA: Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA: Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("OTA: Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("OTA: Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("OTA: Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("OTA: Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("OTA: End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP().toString());

  printDisplay("ip: " + WiFi.localIP().toString(), 2, 0);
   
  
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

  Debug.setDebugLevel(DBG_DEBUG);
  
  Debug.print(DBG_ERROR, "setup:: \n\n");
  Debug.print(DBG_ERROR, "setup:: /----------------------------------------------------------\\");
  Debug.print(DBG_ERROR, "setup:: | Starting...                                              |");
  Debug.print(DBG_ERROR, "setup:: \\----------------------------------------------------------/");

  pinMode(Red,    OUTPUT);
  pinMode(Green,  OUTPUT);

  pinMode(RELAY1,  OUTPUT); switchRelay(RELAY1, relayOff);  
  pinMode(RELAY2,  OUTPUT); switchRelay(RELAY2, relayOff);  
  pinMode(RELAY3,  OUTPUT); switchRelay(RELAY3, relayOff);  
  pinMode(RELAY4,  OUTPUT); switchRelay(RELAY4, relayOff);  
  
  setLedStatus(Red, ledOn);
  setLedStatus(Green, ledOn);

  for (int i=0; i<3; i++)
  { 
    temperatures    = getTemperatures(temperatureArray);
//    printTemperature(temperatures);
    delay(250);
  }
  
  task_determineAlertStatus.enable();
}

void loop() {
  ts.execute();
  ArduinoOTA.handle();
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
    Debug.print(DBG_ERROR, "printTemperature:: Sensor %d: temp: %.2f  ", i, temperatures[i]);
//    printDisplay("Sensor " + String(i) + ": " + String(temperatures[i]), i);
  }
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
  Debug.print(DBG_INFO, "switchRelay:: %d, %s", relay, status ? "ON":"OFF");
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
    Debug.print(DBG_DEBUG, "getTemperatures:: sensor %d = %.2f", sensorTemperature[i]);
//    printDisplay("Sensor" + i + " = " + sensorTemperature[i], i);
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

  
  
  Debug.print(DBG_DEBUG, "determineAlertStatus:: start");
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
      glitchProtection++;      if (glitchProtection < 3)    {   break;    }
    }
    else {   glitchProtection = 0;    }

    bool t0 = (temp  < -120);
    bool t1 = (temp  > criticalTemperature[i]);
    bool t2 = ((temp >  warningTemperature[i]) && (temp <= criticalTemperature[i])) ;
    bool t3 = ((temp >       okTemperature[i]) && (temp <= warningTemperature[i]));
    bool t4 = ((temp <=       okTemperature[i]) && (temp >= -120));

    bool s0 = (previousStatus == statusOK);
    bool s1 = (previousStatus == statusWARNING);
    bool s2 = (previousStatus == statusCRITICAL);
    bool s3 = (previousStatus == statusFAILURE);
    bool s4 = (previousStatus == statusUNKNOWN);

    Debug.print(DBG_VERBOSE, "determineAlertStatus:: t0 = %s", t0 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: t1 = %s", t1 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: t2 = %s", t2 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: t3 = %s", t3 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: t4 = %s", t4 ? "true":"false");

    Debug.print(DBG_VERBOSE, "determineAlertStatus:: s0 = %s", s0 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: s1 = %s", s1 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: s2 = %s", s2 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: s3 = %s", s3 ? "true":"false");
    Debug.print(DBG_VERBOSE, "determineAlertStatus:: s4 = %s", s4 ? "true":"false");
  
    int hit=0;
  
    if (t0 && s2)                         {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 01") ; criticalStatusLevel = 1;  break;   }
    if (t0 && !s2)                        {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 02") ; failureStatusLevel  = 1;           }
    if (t1)                               {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 03") ; criticalStatusLevel = 1;  break;   }
    if (t2 && (s0 || s1 || s4))           {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 04") ; warningStatusLevel  = 1;           }
    if (t2 && s2)                         {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 05") ; criticalStatusLevel = 1;  break;   }
    if (t2 && s3)                         {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 06") ; failureStatusLevel  = 1;           }
    if (t3 && (s0 || s4))                 {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 07") ; okStatusLevel++;                   }
    if (t3 && (s1))                       {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 08") ; warningStatusLevel  = 1;           }
    if (t3 && (s2))                       {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 09") ; criticalStatusLevel = 1;  break;   }
    if (t3 && (s3))                       {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 10") ; failureStatusLevel  = 1;           }
    if (t4 && (s0 || s1 || s4))           {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 11") ; okStatusLevel++;                   }
    if (t4 && (s2))                       {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 12") ; criticalStatusLevel = 1;  break;   }
    if (t4 && (s3))                       {    hit++; Debug.print(DBG_DEBUG, "determineAlertStatus:: 13") ; failureStatusLevel  = 1;           }

    if (hit != 1)
    {
      Debug.print(DBG_ERROR, "---------------------------------------------------------------------------");
      Debug.print(DBG_ERROR, "Hit != 1 !!! Hit = %d", hit);
      Debug.print(DBG_ERROR, "---------------------------------------------------------------------------");
    }
  }
  
  if (criticalStatusLevel != 0)  {
    Debug.print(DBG_WARNING, "determineAlertStatus::  == CRITICAL ==");
    newStatus = statusCRITICAL;
  }
  else if (failureStatusLevel != 0)  {
    Debug.print(DBG_WARNING, "determineAlertStatus::  == FAILURE =="); 
    newStatus = statusFAILURE;
  }
  else if (warningStatusLevel != 0)  {
    Debug.print(DBG_WARNING, "determineAlertStatus::  == WARNING ==");
    newStatus = statusWARNING;
  }
  else if (okStatusLevel == nrOfSensors)  {
    Debug.print(DBG_WARNING, "determineAlertStatus::  == OK ==");
    newStatus = statusOK;
  }
  else  {
    Debug.print(DBG_ERROR, "determineAlertStatus::  == UNKNOWN STATUS ==");
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
    statusText = "ERROR";
    switchPowerRelays(relayOff);
    newStatus = statusCRITICAL;
  }
//
//  display.setFont(ArialMT_Plain_16);
//  printDisplay(statusText, 5);
//  display.display();
  previousStatus = newStatus;

  printToOled(statusText, temperatures);
}

// ----------------------------------------------------------------------

void determineAlertStatusDisabled() {
  Debug.print(DBG_VERBOSE, "determineAlertStatusDisabled:: start");
}

void printToOled(String statusText, float temperatures[])
{
  int nrOfSensors           = numberOfSensors;

  display.setFont(ArialMT_Plain_10);
  for (int i=0; i<nrOfSensors; i=i+2)
  {
    printDisplay(sensorLocation[i] +":" + String(temperatures[i]), (i/2), textPositionLeft);
  }
  for (int i=1; i<nrOfSensors; i=i+2)
  {
    printDisplay(sensorLocation[i] +":"  + String(temperatures[i]), (i/2), textPositionRight);
  }
  
  printDisplay("Status:", 4, 0);
  
  display.setFont(ArialMT_Plain_24);
  printDisplay(statusText, 4, 40);
  display.setFont(ArialMT_Plain_10);
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



void printDisplay(String message, int lineNr)
{
  Serial.print("printDisplay 1:");
  Serial.print(message);
  Serial.print(" on 0, " );
  Serial.println(lineNr*10 );

  printDisplay(message, lineNr, 0);
}

void printDisplay(String message, int lineNr, int position)
{
  int x = position;
 
  Serial.print("printDisplay 2:");
  Serial.print(message);
  Serial.print(" on " );
  Serial.print(x);
  Serial.print("," );
  Serial.println(lineNr*10 );
  
  if ((lineNr == 0) && (x==0))
  {
    display.clear();
    display.display();
  }
 
  display.drawString(x, lineNr*10, message);
  display.display();
}



//
//sendDataToInflux()
//{
//
//}
//
//
//
//sendMessageToIFTTT(status, temperatures)
//{
//  
//}
//
