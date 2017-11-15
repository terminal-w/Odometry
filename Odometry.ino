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
 */

#define debug 0  //switch for Software Serial
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

int instruct(byte reg, char val = 0){
  if(reg == getPow || reg == getEs){
    #if debug == 1
    DEBUG.println("Sorry this function doesn't support that register");
    #endif
    return 0;
  }
  MD25.write((byte)0x00);
  MD25.write(reg);
  #if debug == 1
	DEBUG.print("Register: ");
	DEBUG.print(reg, HEX);
	DEBUG.println(" Accessed");
  #endif
  if(reg > 0x34){return 0;}
  if(reg < 0x30){byte b[5];
    if(reg <= 0x25 && reg >= 0x23){
      //encoders
      MD25.flush();
      MD25.readBytes(b, 5);
      int r = b[2] << 8;         // (0x56 shifted 8 bits left, effectively * 256) 
      r += b[3];              // (0x32)
      #if debug == 1
      DEBUG.print("Serial Buffer: ");
      for(byte i = 0; i<5; i++){
        DEBUG.print(b[i], DEC);
        DEBUG.print(", ");
      }
      DEBUG.println();
      DEBUG.print("Recieved: ");
      DEBUG.print(reg, HEX);
      DEBUG.print(" with:");
      DEBUG.print(r, DEC);
      DEBUG.println(" degrees");
      #endif
      return r; 
    }
    else{
      //gets
      MD25.flush();
      MD25.readBytes(b, 2);
      #if debug == 1
      DEBUG.print("Serial Buffer: ");
      for(byte i; i<5; i++){
        DEBUG.print(b[i], DEC);
        DEBUG.print(", ");
      }
      DEBUG.println();
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
  #if debug == 1
    DEBUG.println("ACHTUNG!!! Ich habe gehaltet!");
  #endif
  return;
}

int enc_target(int distance) {
  /* takes the required travel distance in mm x10 an converts it to an encoder target*/
 float den = pi*wheel_dia;
 float frac = 3600/den;
 int out = distance * frac;

 #if debug == 1
    DEBUG.println(distance, DEC); DEBUG.println(den, DEC); DEBUG.println(frac, DEC);
    DEBUG.print("Encoder Target:");
    DEBUG.print(out, DEC);
    DEBUG.println(" degrees");
 #endif
 return out;
}

void turn(int theta){
	  /* takes two arguments a target angle, theta (degrees x10), and a switch, spot,
		to determine whether the angle is to describe an arc or a spot turn.
		executes turn */
		float distance; //distance to be traveled per in mm
		distance = (theta/3600)*pi*(track);
		#if debug == 1
			DEBUG.print("turn: ");
			DEBUG.println(distance);
		#endif 
    int E2tar = enc_target((int)distance*10);
    int E1tar = enc_target(-(int)distance*10);
    
    DriveTo(E1tar, E2tar);
    return;
}

int sweep(int distance, int radius, bool in = 0){
  /* code to allow robot to describe an arc
     returns the inner & outer arc lengths in mm x10*/
  int Ri = radius - track/20;
  int Ro = radius + track/20;
  int Di = (distance/radius)*Ri;
  int Do = (distance/radius)*Ro;
  #if debug == 1
    DEBUG.print("D(i) = ");
    DEBUG.print(Di/10, DEC);
    DEBUG.print(", D(o) = ");
    DEBUG.print(Do/10, DEC);
    if(in){DEBUG.println(" Return: D(i)");}
    else{DEBUG.println(" Return: D(o)");}
  #endif
  if(in){return Di;}
  else{return Do;}
}

void notify(){
	halt();
	tone(13, 4000, 500);
	delay(500);
	#if debug == 1
		DEBUG.println("Waypoint Notification");
	#endif
  instruct(reset);
	return;
}
void DriveTo(int E1tar, int E2tar) {
	bool happy = 0; int E1cur; int E2cur; char S1; char S2; float E1diff; float E2diff;
 #if debug ==1
  DEBUG.print(E1tar, DEC);
  DEBUG.println(E2tar, DEC);
  #endif
	while (!happy) {
    float E1prog; float E2prog; 
		E1cur = instruct(getE1);
		E2cur = instruct(getE2);
    E1diff = E1tar-E1cur;
    E2diff = E2tar-E2cur;
    E1prog = E1diff/E1tar;
    E2prog = E2diff/E2tar;
		S1 = 100 * E1prog;
		S2 = 100 * E2prog;
#if debug == 1
   DEBUG.print(S1, DEC);
   DEBUG.println(S2, DEC);
   #endif
		if (E1cur == E1tar) {
			happy = 1;
			S1 = 0;
		}
		else if (E1tar - E1cur < 0) {
			S1 -= 27;
		}
		else {
			S1 += 27;
		}
		if (E2cur == E2tar) {
			happy = 1;
			S2 = 0;
		}
		else if (E2tar - E2cur < 0) {
			S2 -= 27;
		}
		else {
			S2 += 27;
		}
		instruct(setS1, S1);
		instruct(setS2, S2);
#if debug == 1
		DEBUG.println("Speed Adjustment: S1, S2");
		DEBUG.print(S1, DEC);
		DEBUG.println(S2, DEC);
#endif
	}
 instruct(setAcc, 10);
 #if debug == 1
 DEBUG.println("GOES LOOPY");
 #endif
 while(E1cur != E1tar || E2cur != E2tar){
  S1 = 50 * (E1tar -E1cur)/90;
  S2 = 50 * (E2tar-E2cur)/90;
  instruct(setS1, S1);
  instruct(setS1, S2);
 }
 instruct(setAcc, 5);
#if debug ==1
	DEBUG.println("Because I'm Happy");
#endif
	return;
}
void target(int distance, int radius) {
#if debug == 1
	DEBUG.println("Targeting...");
#endif
	bool ccw = 0;
	int E1Tar;
	int E2Tar;
	if (radius == 0) {
		E1Tar = enc_target(distance);
		E2Tar = E1Tar;
#if debug == 1
		DEBUG.print("Straight Line, length: ");
		DEBUG.println(distance, DEC);
#endif
	}
	else {
		int Do = sweep(distance, abs(radius));
		int Di = sweep(distance, abs(radius), 1);
		int EoTar = enc_target(Do);
		int EiTar = enc_target(Di);
#if debug == 1
		DEBUG.print("Swept Radius, length, radius: ");
		DEBUG.print(distance, DEC);
		DEBUG.print(", ");
		DEBUG.print(radius, DEC);
#endif
		if (radius < 0) {
			E1Tar = EoTar;
			E2Tar = EiTar;
#if debug == 1
			DEBUG.println(" Anti-Clockwise");
#endif
		}
		else {
			E1Tar = EiTar;
			E2Tar = EoTar;
#if debug == 1
			DEBUG.println(" Clockwise");
#endif
		}
	}
	DriveTo(E1Tar, E2Tar);
		notify();
	return;
}

void MandMrelease(byte remaining){
  /*This Function drops M&Ms
   * Author: Luka - lzd1u16@soton.ac.uk
   */
   Carouselle.write(sPos[remaining]);
   #if debug == 1
    DEBUG.print("M&M Deployed: ");
    DEBUG.println(remaining, DEC);
   #endif
   return;
  } //function prototype for releasing M&Ms


void kmn(){bool a=0;} //function than never returns to provide stop

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
    instruct(setMod, 1); // sets motors with 0 being stop and each independent of the other.
    Carouselle.write(sPos[0]);
    notify();
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
   instruct(reset);
    for(int j = 0; j < 5; j++){
      wp[j] = waypoints[i][j]; // takes data about next waypoint "off the shelf"
    }
#if debug == 1
      DEBUG.print("Next WP: ");
      DEBUG.println(wp[0], DEC);
#endif
    target(wp[1], wp[2]);
    if(wp[4] == 1){
      MandMrelease(MandMstock);
      MandMstock--;
    }
    if(wp[3] > 0){turn(wp[3]);}
}
  kmn();
}
