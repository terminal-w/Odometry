#include <SoftwareSerial.h>
#include "Arduino.h"

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
 */

#define debug  1 //switch for Software Serial

#if debug == 1
	  SoftwareSerial MD25(10, 11); //Software Serial MD25 RX, TX
	  #define DEBUG Serial
#else
	#define MD25 Serial
#endif
const int track = 21000; //trackwidth of robot in mm x100
const int wheel_dia = 10000; //wheel diameter of robot in mm x100
const int wheel_base = 15000; //distance from axle to M&M dispenser in mm x100
union dec {
      struct enc{
          int degs :9;
          byte turns :7;
          } enc;
      int val;
    };
    
/* multi dimension array describing waypoints 
 *  execution order: distance and radius to waypoint -ve radius indicates ccw
 *                   once waypoint achieved notify (and M&M if 1) then turn theta
 *                   -ve indicates ccw before executing next waypoint. 
                                 wpID, distance, radius, theta, M&M
                                       (x10mm)   (x10mm) (x10deg) bool*/ 
const int instructions[13][5] ={
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

int instruct(byte reg, byte val = 0){
  if(reg == getPow || reg == getEs){
    #if debug == 1
    DEBUG.println("Sorry this function doesn't support that register");
    #endif
    return 0;
  }
  MD25.write((byte)0x00);
  MD25.write(reg);
  #if debug == 1
  DEBUG.println("Register Accesed");
  #endif
  if(reg > 0x34){return 0;}
  if(reg < 0x30){byte b[9];
    if(reg <= 0x25 && reg >= 0x23){
      //encoders
      MD25.flush();
      MD25.readBytes(b, 5);
      long r = b[1] << 24;    // (0x00 shifted 24 bits left, effectively * 16777216) 
      r += b[2] << 16;        // (0x10 shifted 16 bits left, effectively * 65536) 
      r += b[3] << 8;         // (0x56 shifted 8 bits left, effectively * 256) 
      r += b[4];              // (0x32)
      dec d;
      d.enc.degs = r % 360;
      d.enc.turns = (byte) r / 360;
      #if debug == 1
      DEBUG.print("Recieved: ");
      DEBUG.print(reg, HEX);
      DEBUG.print(" with turns:");
      DEBUG.print(d.enc.turns, DEC);
      DEBUG.print(" and degrees:");
      DEBUG.println(d.enc.degs, DEC);
      #endif
      return d.val; 
    }
    else{
      //gets
      MD25.flush();
      MD25.readBytes(b, 2);
      #if debug == 1
      DEBUG.print("Recieved: ");
      DEBUG.print(reg, HEX);
      DEBUG.print(" with value:");
      DEBUG.println(b[1], DEC);
      #endif
      return b[1];
    }
  }
  else if(reg <= 0x34 && reg > 0x30){
    //sets
    MD25.write(val);
    return 0;
  }
  #if debug == 1
  DEBUG.println("FATAL ERROR: instruct");
  #endif
  return 0;
}

void halt(){
  //function to stop robot.
	instruct(setAcc, 10);
	instruct(setS1);
  instruct(setS2); 
	instruct(setAcc, 5);
  return;
}

int turn(int theta){
	  /* takes two arguments a target angle, theta (degrees x10), and a switch, spot,
		to determine whether the angle is to describe an arc or a spot turn.
		returns the distance (by outer wheel) that must be traveled x10. */
		float distance; //distance to be traveled per in mm
		distance = (theta/3600)*3.1415962*(track);
		#if debug == 1
			DEBUG.print("turn: ");
			DEBUG.println(distance);
		#endif 
		return distance*10;
}

int enc_target(int distance) {
	/* takes the required travel distance in mm x10 an converts it to an encoder target*/
 int out =distance*3600/(3.1415962*wheel_dia);
 dec d;
 d.enc.turns = out / 360;
 d.enc.degs  = out % 360;
 #if debug == 1
    DEBUG.print("Encoder Target:");
    DEBUG.print(out, DEC);
    DEBUG.print(" degrees, or:");
    DEBUG.print(d.enc.turns, DEC);
    DEBUG.print(" turns and ");
    DEBUG.print(d.enc.degs, DEC);
    DEBUG.println(" degrees.");
 #endif
 return d.val;
}

int sweep(int distance, int radius, bool rat = 0){
  /* code to allow robot to describe an arc
     returns the distance/velocity ratio x1000 between the two encoders and the outer arc length in mm x10*/
  int Ri = radius - track/20;
  int Ro = radius + track/20;
  int ratio = (Ri/Ro)*1000;
  int Do = (distance/radius)*Ro;
  #if debug == 1
    DEBUG.print("Ratio = ");
    DEBUG.print(ratio/1000, DEC);
    DEBUG.print(", D(o) = ");
    DEBUG.print(Do/10, DEC);
    if(rat){DEBUG.println(" Return: Ratio");}
    else{DEBUG.println(" Return: D(o)");}
  #endif
  if(rat){return ratio;}
  else{return Do;}
}

void notify(){
	halt();
	tone(13, 4000, 500);
	delay(500);
	#if debug == 1
		DEBUG.println("Waypoint Notification");
	#endif
	return;
}
 
void setup() {
  // put your setup code here, to run once:
	  #if debug == 1
			MD25.begin(9600);
			DEBUG.begin(9600);  
	  #else
			MD25.begin(38400);
	  #endif
    instruct(reset);
    instruct(setMod, 3);
}


void loop() {
  // put your main code here, to run repeatedly:

}
