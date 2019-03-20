#include <ESP8266WiFi.h>
#include "ESPAsyncWebServer.h"
#include <ArduinoOTA.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SPIFFSEditor.h>

#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager

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
const int numberOfACE = 9; // Number of action code elements
//calibration legs FRF, FRL, RRL, RRF, FLF, FLL, RLL, RLF
int servoCal[] = {   0,   0,   0,   0,   0,   0,   0,   0 }; // Servo calibration data
int servoPos[] = {   135,  45, 135,  45,  45, 135,  45, 135 }; // Servo current position
const int servoPrgPeriod = 20; // 20 ms
int nextAnimation = -1;
int newAction = -1;
Servo servo[numberOfServos];

// Servo calibration position
// -------------------------- P02, P03, P05, P15, P07, P08, P11, P16
int servoAct00 [] PROGMEM = { 135,  45, 135,  45,  45, 135,  45, 135 };

// Zero
int servoPrg00step = 1;
int servoPrg00 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {  135,  45, 135,  45,  45, 135,  45, 135,  400  }, // zero position
};

// Standby
int servoPrg01step = 2;
int servoPrg01 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  200  }, // servo center point
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
};

// Forward
int servoPrg02step = 11;
int servoPrg02 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  90,  90, 110, 110,  90,  45,  90,  100  }, // leg1,4 up; leg4 fw
  {   70,  90,  90, 110, 110,  90,  45,  70,  100  }, // leg1,4 dn
  {   70,  90,  90,  90,  90,  90,  45,  70,  100  }, // leg2,3 up
  {   70,  45, 135,  90,  90,  90,  90,  70,  100  }, // leg1,4 bk; leg2 fw
  {   70,  45, 135, 110, 110,  90,  90,  70,  100  }, // leg2,3 dn
  {   90,  90, 135, 110, 110,  90,  90,  90,  100  }, // leg1,4 up; leg1 fw
  {   90,  90,  90, 110, 110, 135,  90,  90,  100  }, // leg2,3 bk
  {   70,  90,  90, 110, 110, 135,  90,  70,  100  }, // leg1,4 dn
  {   70,  90,  90, 110,  90, 135,  90,  70,  100  }, // leg3 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg3 fw dn
};

// Backward
int servoPrg03step = 11;
int servoPrg03 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  45,  90, 110, 110,  90,  90,  90,  100  }, // leg4,1 up; leg1 fw
  {   70,  45,  90, 110, 110,  90,  90,  70,  100  }, // leg4,1 dn
  {   70,  45,  90,  90,  90,  90,  90,  70,  100  }, // leg3,2 up
  {   70,  90,  90,  90,  90, 135,  45,  70,  100  }, // leg4,1 bk; leg3 fw
  {   70,  90,  90, 110, 110, 135,  45,  70,  100  }, // leg3,2 dn
  {   90,  90,  90, 110, 110, 135,  90,  90,  100  }, // leg4,1 up; leg4 fw
  {   90,  90, 135, 110, 110,  90,  90,  90,  100  }, // leg3,1 bk
  {   70,  90, 135, 110, 110,  90,  90,  70,  100  }, // leg4,1 dn
  {   70,  90, 135,  90, 110,  90,  90,  70,  100  }, // leg2 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg2 fw dn
};

// Move Left
int servoPrg04step = 11;
int servoPrg04 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  45,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg2 fw
  {   70,  90,  45, 110, 110,  90,  90,  70,  100  }, // leg3,2 dn
  {   90,  90,  45, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  45,  90,  90,  100  }, // leg3,2 bk; leg1 fw
  {   70, 135,  90, 110, 110,  45,  90,  70,  100  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg3 fw
  {   70,  90,  90,  90,  90,  90, 135,  70,  100  }, // leg1,4 bk
  {   70,  90,  90, 110, 110,  90, 135,  70,  100  }, // leg3,2 dn
  {   70,  90,  90, 110, 110,  90, 135,  90,  100  }, // leg4 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg4 fw dn
};

// Move Right
int servoPrg05step = 11;
int servoPrg05 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  90,  90,  90,  45,  90,  70,  100  }, // leg2,3 up; leg3 fw
  {   70,  90,  90, 110, 110,  45,  90,  70,  100  }, // leg2,3 dn
  {   90,  90,  90, 110, 110,  45,  90,  90,  100  }, // leg4,1 up
  {   90,  90,  45, 110, 110,  90, 135,  90,  100  }, // leg2,3 bk; leg4 fw
  {   70,  90,  45, 110, 110,  90, 135,  70,  100  }, // leg4,1 dn
  {   70,  90,  90,  90,  90,  90, 135,  70,  100  }, // leg2,3 up; leg2 fw
  {   70, 135,  90,  90,  90,  90,  90,  70,  100  }, // leg4,1 bk
  {   70, 135,  90, 110, 110,  90,  90,  70,  100  }, // leg2,3 dn
  {   90, 135,  90, 110, 110,  90,  90,  70,  100  }, // leg1 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1 fw dn
};

// Turn left
int servoPrg06step = 8;
int servoPrg06 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  90,  90, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  90, 135,  90,  100  }, // leg1,4 turn
  {   70, 135,  90, 110, 110,  90, 135,  70,  100  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90, 135,  70,  100  }, // leg2,3 up
  {   70, 135, 135,  90,  90, 135, 135,  70,  100  }, // leg2,3 turn
  {   70, 135, 135, 110, 110, 135, 135,  70,  100  }, // leg2,3 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Turn right
int servoPrg07step = 8;
int servoPrg07 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  90,  90,  90,  90,  90,  70,  100  }, // leg2,3 up
  {   70,  90,  45,  90,  90,  45,  90,  70,  100  }, // leg2,3 turn
  {   70,  90,  45, 110, 110,  45,  90,  70,  100  }, // leg2,3 dn
  {   90,  90,  45, 110, 110,  45,  90,  90,  100  }, // leg1,4 up
  {   90,  45,  45, 110, 110,  45,  45,  90,  100  }, // leg1,4 turn
  {   70,  45,  45, 110, 110,  45,  45,  70,  100  }, // leg1,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Lie
int servoPrg08step = 1;
int servoPrg08 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {  110,  90,  90,  70,  70,  90,  90, 110,  500  }, // leg1,4 up
};

// Say Hi
int servoPrg09step = 4;
int servoPrg09 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
};

// Fighting
int servoPrg10step = 11;
int servoPrg10 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 2 down
  {  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
  {  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  200  }, // leg1, 2 up ; leg3, 4 down
  {   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
  {   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  200  }  // leg1, 2 up ; leg3, 4 down
};

// Push up
int servoPrg11step = 11;
int servoPrg11 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // start
  {  100,  90,  90,  80,  80,  90,  90, 100,  400  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70,  500  }, // up
  {  100,  90,  90,  80,  80,  90,  90, 100,  600  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70,  700  }, // up
  {  100,  90,  90,  80,  80,  90,  90, 100, 1300  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70, 1800  }, // up
  {  135,  90,  90,  45,  45,  90,  90, 135,  200  }, // fast down
  {   70,  90,  90,  45,  60,  90,  90, 135,  500  }, // leg1 up
  {   70,  90,  90,  45, 110,  90,  90, 135,  500  }, // leg2 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  500  }  // leg3, leg4 up
};

// Sleep
int servoPrg12step = 2;
int servoPrg12 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  200  }, // leg1,4 dn
  {   30,  45, 135, 150, 150, 135,  45,  30,  200  }, // protect myself
};

// Dancing 1
int servoPrg13step = 10;
int servoPrg13 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1,2,3,4 up
  {   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
  {   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up; leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up
};

// Dancing 2
int servoPrg14step = 9;
int servoPrg14 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  45, 135, 110, 110, 135,  45,  70,  300  }, // leg1,2,3,4 two sides
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   75,  45, 135, 105, 110, 135,  45,  70,  300  }, // leg1,2 dn
};

// Dancing 3
int servoPrg15step = 10;
int servoPrg15 [][numberOfACE] PROGMEM = {
  // P02, P03, P05, P15, P07, P08, P11, P16,  ms
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3,4 bk
  {  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
  {  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
  {  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
  {  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // standby
};

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
