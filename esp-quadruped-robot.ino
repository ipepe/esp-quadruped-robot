// Q1 lite 2.0 - Simple Quadruped Robot (Designed by Jason Workshop)
//
// Firmware version 2.0.0
// Last Update: 28 Mar 2018
//
// Jason Workshop
// Website: http://jasonworkshop.com
// FB page: http://fb.com/jasonworkshop
//
// Related documents and software
// Website: http://q1.jasonworkshop.com
// FB page: http://fb.com/Q1.JasonWorkshop
//
// 3D parts
// Website: http://thingiverse.com/thing:2732957
//
// ---------------------------------------------------------------------------------------------------------------
//
// This Firmware licensed under the Attribution-NonCommercial-ShareAlike 4.0 (CC-BY-NC-SA 4.0)
//
// Attribution: You must give appropriate credit, provide a link to the license, and indicate if changes were made.
// You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
//
// ShareAlike: If you remix, transform, or build upon the material,
// you must distribute your contributions under the same license as the original.
//
// License Deed
// http://creativecommons.org/licenses/by-sa/4.0/
//
// ---------------------------------------------------------------------------------------------------------------
//
//  -----               -----
// |  5  |             |  1  |
// | P07 |             | P02 |
//  ----- -----   ----- -----
//       |  6  | |  2  |
//       | P08 | | P03 |
//        -----   -----
//       |  7  | |  3  |
//       | P11 | | P05 |
//  ----- -----   ----- -----
// |  8  |             |  4  |
// | P16 |             | P15 |
//  -----               -----  (Top View)
//

#include <EEPROM.h>
#include <Servo.h>
#include <LRemote.h>

// ---------------------------------------------------------------------------------------------------------------

String robotName = "Q1 lite"; // Robot name

const int enableCalibration = true; // Enable calibration button

// ---------------------------------------------------------------------------------------------------------------

const int numberOfServos = 8; // Number of servos
const int numberOfACE = 9; // Number of action code elements
const int buzzerPin = 14; // Robot shield onboard buzzer pin
int servoCal[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoPrgPeriod = 20; // 20 ms
Servo servo[numberOfServos]; // Servo object

// LinkIt remote interface object
int sliderX;
int sliderWidth;
LRemoteButton button01;
LRemoteButton button02;
LRemoteButton button03;
LRemoteButton button04;
LRemoteButton button05;
LRemoteButton button06;
LRemoteButton button07;
LRemoteButton button08;
LRemoteButton button09;
LRemoteButton button10;
LRemoteButton button11;
LRemoteButton button12;
LRemoteButton button13;
LRemoteButton button14;
LRemoteButton button15;
LRemoteButton buttonZero;
LRemoteButton buttonClear;
LRemoteButton buttonC01a;
LRemoteButton buttonC01b;
LRemoteButton buttonC02a;
LRemoteButton buttonC02b;
LRemoteButton buttonC03a;
LRemoteButton buttonC03b;
LRemoteButton buttonC04a;
LRemoteButton buttonC04b;
LRemoteButton buttonC05a;
LRemoteButton buttonC05b;
LRemoteButton buttonC06a;
LRemoteButton buttonC06b;
LRemoteButton buttonC07a;
LRemoteButton buttonC07b;
LRemoteButton buttonC08a;
LRemoteButton buttonC08b;
LRemoteSlider slider01;
LRemoteSlider slider02;
LRemoteSlider slider03;
LRemoteSlider slider04;
LRemoteSlider slider05;
LRemoteSlider slider06;
LRemoteSlider slider07;
LRemoteSlider slider08;



// Action code
// --------------------------------------------------------------------------------

// Servo zero position
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

// --------------------------------------------------------------------------------



// Setup
// --------------------------------------------------------------------------------

void setup()
{
  // EEPROM Clear (For Debug Only)
  // eepromClear();

  // Serial.begin(9600); // Open serial communications

  getServoCal(); // Get servoCal from EEPROM

  // Servo Pin Set
  servo[0].attach(2);
  servo[1].attach(3);
  servo[2].attach(5);
  servo[3].attach(15);
  servo[4].attach(7);
  servo[5].attach(8);
  servo[6].attach(11);
  servo[7].attach(16);

  // Setup UI canvas
  LRemote.setName(robotName);
  LRemote.setOrientation(RC_PORTRAIT);
  LRemote.setGrid(12, 21);

  // Add a push button
  // ButtonLabel, PosX, PosY, SizeW, SizeH, Color, ObjectName
  addButton("Turn Left",  0,  0, 4, 2, RC_GREEN, button01);
  addButton("Forward",    4,  0, 4, 2, RC_BLUE, button02);
  addButton("Turn Right", 8,  0, 4, 2, RC_GREEN, button03);
  addButton("Left",       0,  2, 4, 2, RC_BLUE, button04);
  addButton("Backward",   4,  2, 4, 2, RC_BLUE, button05);
  addButton("Right",      8,  2, 4, 2, RC_BLUE, button06);
  addButton("Standby",    0,  4, 4, 2, RC_PINK, button07);
  addButton("Say Hi!",    4,  4, 4, 2, RC_ORANGE, button08);
  addButton("Push Up",    8,  4, 4, 2, RC_ORANGE, button09);
  addButton("Lie",        0,  6, 4, 2, RC_ORANGE, button10);
  addButton("Fighting",   4,  6, 4, 2, RC_ORANGE, button11);
  addButton("Sleep",      8,  6, 4, 2, RC_ORANGE, button12);
  addButton("Dancing 1",  0,  8, 4, 2, RC_ORANGE, button13);
  addButton("Dancing 2",  4,  8, 4, 2, RC_ORANGE, button14);
  addButton("Dancing 3",  8,  8, 4, 2, RC_ORANGE, button15);
  addButton("Zero",       0, 20, 2, 1, RC_ORANGE, buttonZero);

  if (enableCalibration == true) {
    sliderX = 0;
    sliderWidth = 5;
  } else {
    sliderX = 13;
    sliderWidth = 6;
  }
  addButton("Clr Cal",  2 + sliderX, 20, 2, 1, RC_PINK, buttonClear);
  addButton("+",        5 + sliderX, 11, 1, 1, RC_PINK, buttonC05a);
  addButton("-",        5 + sliderX, 12, 1, 1, RC_PINK, buttonC05b);
  addButton("+",       11 + sliderX, 11, 1, 1, RC_PINK, buttonC01a);
  addButton("-",       11 + sliderX, 12, 1, 1, RC_PINK, buttonC01b);
  addButton("+",        5 + sliderX, 13, 1, 1, RC_PINK, buttonC06a);
  addButton("-",        5 + sliderX, 14, 1, 1, RC_PINK, buttonC06b);
  addButton("+",       11 + sliderX, 13, 1, 1, RC_PINK, buttonC02a);
  addButton("-",       11 + sliderX, 14, 1, 1, RC_PINK, buttonC02b);
  addButton("+",        5 + sliderX, 15, 1, 1, RC_PINK, buttonC07a);
  addButton("-",        5 + sliderX, 16, 1, 1, RC_PINK, buttonC07b);
  addButton("+",       11 + sliderX, 15, 1, 1, RC_PINK, buttonC03a);
  addButton("-",       11 + sliderX, 16, 1, 1, RC_PINK, buttonC03b);
  addButton("+",        5 + sliderX, 17, 1, 1, RC_PINK, buttonC08a);
  addButton("-",        5 + sliderX, 18, 1, 1, RC_PINK, buttonC08b);
  addButton("+",       11 + sliderX, 17, 1, 1, RC_PINK, buttonC04a);
  addButton("-",       11 + sliderX, 18, 1, 1, RC_PINK, buttonC04b);

  // Add a slider
  // SliderLabel, PosX, PosY, SizeW, SizeH, RangeMin, RangeMax, RangInit, Color, ObjectName
  addSlider("P07", 0, 11, sliderWidth, 2, 45, 135, servoAct00[4], RC_BLUE, slider05);
  addSlider("P02", 6, 11, sliderWidth, 2, 45, 135, servoAct00[0], RC_BLUE, slider01);
  addSlider("P08", 0, 13, sliderWidth, 2, 45, 135, servoAct00[5], RC_BLUE, slider06);
  addSlider("P03", 6, 13, sliderWidth, 2, 45, 135, servoAct00[1], RC_BLUE, slider02);
  addSlider("P11", 0, 15, sliderWidth, 2, 45, 135, servoAct00[6], RC_BLUE, slider07);
  addSlider("P05", 6, 15, sliderWidth, 2, 45, 135, servoAct00[2], RC_BLUE, slider03);
  addSlider("P16", 0, 17, sliderWidth, 2, 45, 135, servoAct00[7], RC_BLUE, slider08);
  addSlider("P15", 6, 17, sliderWidth, 2, 45, 135, servoAct00[3], RC_BLUE, slider04);

  LRemote.begin(); // Start broadcasting remote controller

  runServoPrg(servoPrg00, servoPrg00step); // zero position
}

// --------------------------------------------------------------------------------



// Loop
// --------------------------------------------------------------------------------

void loop()
{
  // Check connection
  if(!LRemote.connected()) {
    delay(1000);
  }

  // Process the incoming BLE write request
  LRemote.process();

  // When button pressed
  if (button01.getValue()) {
    runServoPrg(servoPrg06, servoPrg06step); // turnLeft
  } else if (button02.getValue()) {
    runServoPrg(servoPrg02, servoPrg02step); // forward
  } else if (button03.getValue()) {
    runServoPrg(servoPrg07, servoPrg07step); // turnRight
  } else if (button04.getValue()) {
    runServoPrg(servoPrg04, servoPrg04step); // moveLeft
  } else if (button05.getValue()) {
    runServoPrg(servoPrg03, servoPrg03step); // backward
  } else if (button06.getValue()) {
    runServoPrg(servoPrg05, servoPrg05step); // moveRight
  } else if (button07.getValue()) {
    runServoPrg(servoPrg01, servoPrg01step); // standby
  } else if (button08.getValue()) {
    runServoPrg(servoPrg09, servoPrg09step); // sayHi
  } else if (button09.getValue()) {
    runServoPrg(servoPrg11, servoPrg11step); // pushUp
  } else if (button10.getValue()) {
    runServoPrg(servoPrg08, servoPrg08step); // lie
  } else if (button11.getValue()) {
    runServoPrg(servoPrg10, servoPrg10step); // fighting
  } else if (button12.getValue()) {
    runServoPrg(servoPrg12, servoPrg12step); // sleep
  } else if (button13.getValue()) {
    runServoPrg(servoPrg13, servoPrg13step); // dancing1
  } else if (button14.getValue()) {
    runServoPrg(servoPrg14, servoPrg14step); // dancing2
  } else if (button15.getValue()) {
    runServoPrg(servoPrg15, servoPrg15step); // dancing3
  } else if (buttonZero.getValue()) {
    runServoPrg(servoPrg00, servoPrg00step); // zero position
  } else if (buttonClear.getValue()) {
    clearCal(); // Clear Servo calibration data
  } else if (buttonC01a.getValue()) {
    calibration(0, 1);
  } else if (buttonC01b.getValue()) {
    calibration(0, -1);
  } else if (buttonC02a.getValue()) {
    calibration(1, 1);
  } else if (buttonC02b.getValue()) {
    calibration(1, -1);
  } else if (buttonC03a.getValue()) {
    calibration(2, 1);
  } else if (buttonC03b.getValue()) {
    calibration(2, -1);
  } else if (buttonC04a.getValue()) {
    calibration(3, 1);
  } else if (buttonC04b.getValue()) {
    calibration(3, -1);
  } else if (buttonC05a.getValue()) {
    calibration(4, 1);
  } else if (buttonC05b.getValue()) {
    calibration(4, -1);
  } else if (buttonC06a.getValue()) {
    calibration(5, 1);
  } else if (buttonC06b.getValue()) {
    calibration(5, -1);
  } else if (buttonC07a.getValue()) {
    calibration(6, 1);
  } else if (buttonC07b.getValue()) {
    calibration(6, -1);
  } else if (buttonC08a.getValue()) {
    calibration(7, 1);
  } else if (buttonC08b.getValue()) {
    calibration(7, -1);
  }

  // When slider change
  if (slider01.isValueChanged()) {
    servo[0].write(slider01.getValue() + servoCal[0]);
  } else if (slider02.isValueChanged()) {
    servo[1].write(slider02.getValue() + servoCal[1]);
  } else if (slider03.isValueChanged()) {
    servo[2].write(slider03.getValue() + servoCal[2]);
  } else if (slider04.isValueChanged()) {
    servo[3].write(slider04.getValue() + servoCal[3]);
  } else if (slider05.isValueChanged()) {
    servo[4].write(slider05.getValue() + servoCal[4]);
  } else if (slider06.isValueChanged()) {
    servo[5].write(slider06.getValue() + servoCal[5]);
  } else if (slider07.isValueChanged()) {
    servo[6].write(slider07.getValue() + servoCal[6]);
  } else if (slider08.isValueChanged()) {
    servo[7].write(slider08.getValue() + servoCal[7]);
  }
}

// --------------------------------------------------------------------------------



// Function
// --------------------------------------------------------------------------------

// EEPROM Clear (For debug only)
void eepromClear()
{
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
  randomSound();
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

// Add a push button
void addButton(String label, int posX, int posY, int sizeW, int sizeH, RCColorType colorType, LRemoteButton &button)
{
  button.setText(label);
  button.setPos(posX, posY);
  button.setSize(sizeW, sizeH);
  button.setColor(colorType);
  LRemote.addControl(button);
}

// Add a slider
void addSlider(String label, int posX, int posY, int sizeW, int sizeH, int rangeMin, int rangeMax, int rangeInit, RCColorType colorType, LRemoteSlider &slider)
{
  slider.setText(label);
  slider.setPos(posX, posY);
  slider.setSize(sizeW, sizeH);
  slider.setColor(colorType);
  slider.setValueRange(rangeMin, rangeMax, rangeInit);
  LRemote.addControl(slider);
}

void randomSound()
{
  for (int i = 1; i < 20; i = i + 1) {
    tone(buzzerPin, random(50, 1000));
    delay(40);
  }
  noTone(buzzerPin);
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
        servo[k].write((map(j, 0, totalTime / servoPrgPeriod, servoPos[k], servoPrg[i][k])) + servoCal[k]);
      }
      delay(servoPrgPeriod);
    }
  }
}

// --------------------------------------------------------------------------------

