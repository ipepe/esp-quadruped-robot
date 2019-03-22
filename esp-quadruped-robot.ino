#include <ESP8266WiFi.h>
#include "ESPAsyncWebServer.h"
#include <ArduinoOTA.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SPIFFSEditor.h>
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include "animations.h"

AsyncWebServer server(80);
DNSServer dns;

const uint8_t FRONT_RIGHT_FOOT = D7;
const uint8_t FRONT_RIGHT_LEG = D6;

const uint8_t REAR_RIGHT_LEG = D5;
const uint8_t REAR_RIGHT_FOOT = D8;

const uint8_t FRONT_LEFT_FOOT = D3;
const uint8_t FRONT_LEFT_LEG = D2;

const uint8_t REAR_LEFT_LEG = D1;
const uint8_t REAR_LEFT_FOOT = D4;

const int numberOfServos = 8; // Number of servos

//calibration legs FRF, FRL, RRL, RRF, FLF, FLL, RLL, RLF
int servoCal[] = {   0,   0,   0,   0,   0,   0,   0,   0 }; // Servo calibration data
int servoPos[] = {   135,  45, 135,  45,  45, 135,  45, 135 }; // Servo current position
const int servoPrgPeriod = 20; // 20 ms
int nextAnimation = -1;
int newAction = -1;
Servo servo[numberOfServos];

// EEPROM Clear (For debug only)
void eepromClear()
{
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
}

// Get servoCal from EEPROM
void getServoCal()
{
  int eeAddress = 0;
  for (int i = 0; i < numberOfServos; i++) {
    EEPROM.get(eeAddress, servoCal[i]);
    eeAddress += sizeof(servoCal[i]);
  }
}

// Put servoCal to EEPROM
void putServoCal()
{
  int eeAddress = 0;
  for (int i = 0; i < numberOfServos; i++) {
    EEPROM.put(eeAddress, servoCal[i]);
    eeAddress += sizeof(servoCal[i]);
  }
}

// Clear Servo calibration data
void clearCal()
{
  for (int i = 0; i < numberOfServos; i++) {
    servoCal[i] = 0;
  }
  putServoCal(); // Put servoCal to EEPROM
  runServoPrg(servoPrg00, servoPrg00step); // zero position
}

// Calibration
void calibration(int i, int change)
{
  servoCal[i] = servoCal[i] + change;
  servo[i].write(servoAct00[i] + servoCal[i]);
  putServoCal(); // Put servoCal to EEPROM
  delay(400);
}

void runServoPrg(int servoPrg[][numberOfACE], int step)
{
  for (int i = 0; i < step; i++) { // Loop for step

    int totalTime = servoPrg[i][numberOfACE - 1]; // Total time of this step

    // Get servo start position
    for (int s = 0; s < numberOfServos; s++) {
      servoPos[s] = servo[s].read() - servoCal[s];
    }

    for (int j = 0; j < totalTime / servoPrgPeriod; j++) { // Loop for time section
      for (int k = 0; k < numberOfServos; k++) { // Loop for servo
        servo[k].write((map(j, 0, totalTime / servoPrgPeriod, servoPos[k], 180 - servoPrg[i][k])) + servoCal[k]);
      }
      delay(servoPrgPeriod);
    }
  }
}

void setup() {
  servo[0].attach(FRONT_RIGHT_FOOT);
  servo[1].attach(FRONT_RIGHT_LEG);
  servo[2].attach(REAR_RIGHT_LEG);
  servo[3].attach(REAR_RIGHT_FOOT);
  servo[4].attach(FRONT_LEFT_FOOT);
  servo[5].attach(FRONT_LEFT_LEG);
  servo[6].attach(REAR_LEFT_LEG);
  servo[7].attach(REAR_LEFT_FOOT);
  runServoPrg(servoPrg01, servoPrg01step); // zero position

  Serial.begin(115200);
  Serial.println("Booted");
  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  AsyncWiFiManager wifiManager(&server,&dns);
  wifiManager.autoConnect();
  ArduinoOTA.setHostname("quadruped-v1-wemos-d1-mini");
  ArduinoOTA.begin();

  server.on("/api/move", [](AsyncWebServerRequest *request){
    if(request->hasParam("action")){
        newAction = request->arg("action").toInt();
        if(newAction >= 0 && newAction <= 15){
            nextAnimation = newAction;
            request->send(200, "text/plain", "ok");
        }else{
            request->send(404, "text/plain", "error");
        }
    }else{
        request->send(404, "text/plain", "error");
    }
  });
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.begin();

  delay(300);
  runServoPrg(servoPrg01, servoPrg01step); // zero position

  Serial.println("End of setup");
}

void loop() {
  ArduinoOTA.handle();
  if(nextAnimation != -1){
    switch(nextAnimation){
    case 0:
        runServoPrg(servoPrg00, servoPrg00step);
        break;
    case 1:
        runServoPrg(servoPrg01, servoPrg01step);
        break;
    case 2:
        runServoPrg(servoPrg02, servoPrg02step);
        break;
    case 3:
        runServoPrg(servoPrg03, servoPrg03step);
        break;
    case 4:
        runServoPrg(servoPrg04, servoPrg04step);
        break;
    case 5:
        runServoPrg(servoPrg05, servoPrg05step);
        break;
    case 6:
        runServoPrg(servoPrg06, servoPrg06step);
        break;
    case 7:
        runServoPrg(servoPrg07, servoPrg07step);
        break;
    case 8:
        runServoPrg(servoPrg08, servoPrg08step);
        break;
    case 9:
        runServoPrg(servoPrg09, servoPrg09step);
        break;
    case 10:
        runServoPrg(servoPrg10, servoPrg10step);
        break;
    case 11:
        runServoPrg(servoPrg11, servoPrg11step);
        break;
    case 12:
        runServoPrg(servoPrg12, servoPrg12step);
        break;
    case 13:
        runServoPrg(servoPrg13, servoPrg13step);
        break;
    case 14:
        runServoPrg(servoPrg14, servoPrg14step);
        break;
    case 15:
        runServoPrg(servoPrg15, servoPrg15step);
        break;
    default:
        break;
    }
    nextAnimation = -1;
  }
}
