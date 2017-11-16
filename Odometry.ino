#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Servo.h>

/*
 * Title: Odometry Task
 * Client: Aero 2 - Group 17/Q{
 * Team Lead:       James Wilshaw
 * 2nd in Command:  Luka Zabotto-Devitt
 *                - Ho (Phoebus) Wong
 *                - Wen (William) Yu
 *                - Aaron Adekoya
 *                }
 * Author: James Wilshaw -- jrw1n15@soton.ac.uk
 * Date: 2017-11-06
 * =============================================
 * Objectives:
 * Navigate route and carry out tasks at waypoints using only encoder data as input.
 * notify at all way points.
 * Dispense Chocolate at certain way points.
 * 
 * CODE STRUCTURE
 * -Instruct: Instruction sent to the MD25 and receives information to decode it
 * -halt: Function to stop robot
 * -enc target: function that will translate the distance in mm to computer readable information
 * -Turn(theta): function that will tell the MD25 if the turn is sweep or spot.
 * -Sweep: Function that will describe an arc using theta
 * -Notify: indicates the robot has arrived at a point
 * -DriveTo: Manages the speed of the robot
 * -Target: intakes the information of the next trajectory and outputs information for enc target
 * -kmn: stops the loop
 * -
 */

#define debug 1  //switch for Software Serial
#define pi 3.1415926 //saves any errors typing

#if debug == 1 // NOT THE SERIAL SWITCH DON'T CHANGE
    SoftwareSerial MD25(10, 11); //Software Serial MD25 RX, TX
    #define DEBUG Serial
#else
  #define MD25 Serial
#endif
Servo Carouselle;
const int track = 23500; //trackwidth of robot in mm x100
const int wheel_dia = 9450; //wheel diameter of robot in mm x100
const int wheel_base = 15000; //distance from axle to M&M dispenser in mm x100
const byte sPos[6] = {20, 0, 40, 73, 114, 150}; //defines servo drive positions for M&Ms 

    
/* multi dimension array describing waypoints 
 *  execution order: distance and radius to waypoint -ve radius indicates ccw
 *                   once waypoint achieved notify (and M&M if 1) then turn theta
 *                   -ve indicates ccw before executing next waypoint. 
                                 wpID, distance, radius, theta, M&M
                                       (x10mm)   (x10mm) (x10deg) bool*/ 
const int waypoints[13][5] ={
                                {12,   4280,     0,      0,     0},
                                {11,   2833,     0,      1426,  0},
                                {10,   8482,     -1800,  -900,  1},
                                {9,    1800,     0,      1400,  0},
                                {8,    6223,     0,      500,   1},
                                {7,    4000,     0,      900,   0},
                                {6,    4000,     0,      900,   1},
                                {5,    4000,     0,      900,   0},
                                {4,    6600,     0,      -900,  1},
                                {3,    4084,     -2600,  900,   0},
                                {2,    5000,     0,      900,   1},
                                {1,    2600,     0,      -900,  0},
                                {0,    3400,     0,      0,     0}
                                };
/*
 * serial control register lookup table
 */
enum registers:byte
  {
    getS1   = 0x21,
    getS2   = 0x22,
    getE1   = 0x23,
    getE2   = 0x24,
    getEs   = 0x25,
    getV    = 0x26,
    getI1   = 0x27,
    getI2   = 0x28,
    getVer  = 0x29,
    getAcc  = 0x2A,
    getMod  = 0x2B,
    getPow  = 0x2C,
    setS1   = 0x31,
    setS2   = 0x32,
    setAcc  = 0x33,
    setMod  = 0x34,
    reset   = 0x35,
    disReg  = 0x36,
    enReg   = 0x37,
    disTimO = 0x38,
    enTimO  = 0x39
  };

#include "OdometryTasks.h" 
OdometryTasks robot(sPos, wheel_dia, wheel_base, track, *Carouselle);

void setup() {
  // put your setup code here, to run once:
  Carouselle.attach(9);
  pinMode(13, OUTPUT);
  pinMode(4, INPUT);
    #if debug == 1
      MD25.begin(9600);
      DEBUG.begin(9600);  
    #else
      MD25.begin(38400);
    #endif
    robot.instruct(setMod, 1); // sets motors with 0 being stop and each independent of the other.
    Carouselle.write(sPos[0]);
    robot.notify();
    bool go = 0;
    #if debug == 1
    DEBUG.print("Awaiting all clear @ ");
    DEBUG.println((int)millis, DEC);
    #endif
     while(!go){
      go = !digitalRead(4);
    }
    #if debug == 1
      DEBUG.print("Setup Complete @ ");
      DEBUG.println((int)millis, DEC);
    #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  byte MandMstock = 5;
  for(int i = 0; i < 13; i++){ // for loop to work through waypoints
    int wp[5];
   robot.instruct(reset);
    for(int j = 0; j < 5; j++){
      wp[j] = waypoints[i][j]; // takes data about next waypoint "off the shelf"
    }
#if debug == 1
      DEBUG.print("Next WP: ");
      DEBUG.println(wp[0], DEC);
#endif
    robot.target(wp[1], wp[2]);
    if(wp[4] == 1){
      robot.MandMrelease(MandMstock);
      MandMstock--;
    }
    if(wp[3] > 0){robot.turn(wp[3]);}
}
  kmn();
}
