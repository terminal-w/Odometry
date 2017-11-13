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

#define debug 1  //switch for Software Serial

#if debug == 1
	  SoftwareSerial MD25(10, 11); //Software Serial MD25 RX, TX
	  #define DEBUG Serial
#else
	#define MD25 Serial
#endif
Servo Carouselle;
const int track = 21000; //trackwidth of robot in mm x100
const int wheel_dia = 10000; //wheel diameter of robot in mm x100
const int wheel_base = 15000; //distance from axle to M&M dispenser in mm x100
const byte sPos[6] = {0, 51, 102, 153, 204, 255}; //defines servo drive positions for M&Ms 
union dec { // definition of magical data class with many variables sharing the same bit of memory so we can do funky encoding to save memory and stuff.
      struct enc{
          int degs :9; //bitfields make this integer 9 bits long within the structure
          byte turns :7; //bitfields make this byte 7 bits long within the structure
          } enc;
      int val;
    };
    
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
  DEBUG.println("Register Accesed");
  #endif
  if(reg > 0x34){return 0;}
  if(reg < 0x30){byte b[5];
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
  #if debug == 1
    DEBUG.println("ACHTUNG!!! Ich habe gehaltet!");
  #endif
  return;
}

void turn(int theta){
	  /* takes two arguments a target angle, theta (degrees x10), and a switch, spot,
		to determine whether the angle is to describe an arc or a spot turn.
		executes turn */
		float distance; //distance to be traveled per in mm
		distance = (theta/3600)*3.1415962*(track);
		#if debug == 1
			DEBUG.print("turn: ");
			DEBUG.println(distance);
		#endif 
    int E2tar = enc_target((int)distance*10);
    int E1tar = enc_target(-(int)distance*10);
    if(E1tar > E2tar){
      instruct(setS1, (byte)127);
      instruct(setS2, (byte)-127);
    }
    else{
      instruct(setS2, (byte)127);
      instruct(setS1, (byte)-127);
    }
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
  instruct(reset);
	return;
}

void MandMrelease(byte remaining){
  /*This Function drops M&Ms
   * Author: Luka - lzd1u16@soton.ac.uk
   */
   Carouselle.write(sPos[remaining]);
   return;
  } //function prototype for releasing M&Ms

byte overshootOrFine(int tt, dec wheel_decoder, bool wheel = 0){
  /* to be used when robot is moving at 'cruise speed' to determine when to slow down
     takes a wheel encoder value and target turns. Wheel is high for encoder 2;
     returns:
     0 is no change
     1 is overshoot
     2 is fine
  */
  if(wheel_decoder.enc.turns > tt){halt(); return (byte)1;}
  else if(wheel_decoder.enc.turns = tt){
    if(!wheel){instruct(setS1, (char)50);}
    else{instruct(setS2, (char)50);}
    return (byte)2;
  }
  else if(wheel_decoder.enc.turns >= 0.9*tt){
    if(!wheel){instruct(setS1, (char)90);}
    else{instruct(setS2, (char)90);}
    return (byte)2;
  }
  return 0;
}

void wiggle(int decTar, bool wheel = 0){ //fine adjustment prototype
  #if debug == 1
    DEBUG.println("Wiggle Wiggle Wiggle, Yeah");
  #endif
  dec decoder;
  decoder.val = decTar;
  bool happy = 0;
  long target_degs = decoder.enc.turns*360 + decoder.enc.degs;
  long current_degs;
  instruct(setAcc, (byte)10);
  while(!happy){
    if(wheel){decoder.val = instruct(getE2);}
    else{decoder.val = instruct(getE1);}
    current_degs = decoder.enc.turns*360 + decoder.enc.degs;
    if(target_degs-current_degs == 0){happy = 1; halt(); break;}
    char velocity = (target_degs - current_degs)*127/360;
    if(wheel){instruct(setS2, velocity);}
    else{instruct(setS1, velocity);}
  }
  #if debug == 1
    DEBUG.println("Clap along if you feel like happiness is the truth");
  #endif
  return;
}

void straightAndNarrow(int distance){
  /*drive in a straightline*/
  //code to achieve straight line
  int E1cur; //current encoder value of E1
  int E2cur; //current encoder value of E2
  bool e; // 1 if wheel_decoder currently contains encoder 2
  dec wheel_decoder;
    int Etar = enc_target(distance);
    bool overshoot = 0;
    bool fine = 0;
    #if debug == 1
      DEBUG.print("Straight line target:");
      DEBUG.print(distance/10, DEC);
      DEBUG.println("mm");
    #endif
    
    wheel_decoder.val = Etar;
    int tt = wheel_decoder.enc.turns;
    int td = wheel_decoder.enc.degs;
    instruct(setS1, (char)127);
    instruct(setS2, (char)127);
    while(!overshoot && !fine){
      E1cur = instruct(getE1);
      E2cur = instruct(getE2);
      wheel_decoder.val = E1cur;
      e = 0;
      switch(overshootOrFine(tt, wheel_decoder, e)){
        case 0: break;
        case 1: {overshoot = 1; break;}
        case 2: {fine = 1; break;}
      }
      wheel_decoder.val = E2cur;
      e = 1;
      switch(overshootOrFine(tt, wheel_decoder, e)){
        case 0: break;
        case 1: {overshoot = 1; break;}
        case 2: {fine = 1; break;}
      }
    }
    while(overshoot && !fine){
      #if debug == 1
        DEBUG.println("OVERSHOOT");
      #endif
      bool reverse = 0;
      int error = 360;
      if(abs(wheel_decoder.enc.turns-tt)-tt <= 1){
        instruct(setS1, (char)-50);
        instruct(setS2, (char)-50);
        fine = 1;
        overshoot =1;
      }
      else if(abs(wheel_decoder.enc.turns-tt)/tt >= 0.9 && !reverse){
        #if debug == 1
          DEBUG.println("IS HUGE");
        #endif
        instruct(setS1, (char)-120);
        instruct(setS2, (char)-120);
        reverse = 1;
      }
      else if(abs(wheel_decoder.enc.turns-tt)/tt < 0.9 ){
        #if debug == 1
          DEBUG.println("is moderate");
        #endif
        instruct(setS1, (char)-90);
        instruct(setS2, (char)-90);
        reverse = 1;
      }
      E1cur = instruct(getE1);
      E2cur = instruct(getE2);
      if(e){wheel_decoder.val = E1cur;}
      else{wheel_decoder.val = E2cur;}
    }
    wiggle(Etar, 0);
    wiggle(Etar, 1);
    #if debug == 1 
      DEBUG.println("The way of the righteous is narrow");
    #endif
    return;
}
void raidersOfTheLostARC(int ratio, int Do, bool ccw){
  int EoCur; int EiCur; int EoTar; int EiTar;
  int Di = ratio/100 * Do;
  #if debug == 1
    if(ccw){DEBUG.println("Describing Anti-Clockwise Arc:");}
    else{DEBUG.println("Describing Clockwise Arc:");}
    DEBUG.print("Inner Circumference:");
    DEBUG.print(Di/10, DEC);
    DEBUG.print("mm Outer Circumference:");
    DEBUG.print(Do/10, DEC);
    DEBUG.println("mm");
  #endif
}
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
     while(!go){
      go = !digitalRead(4);
    }
    #if debug == 1
      DEBUG.println("Setup Complete");
    #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  byte MandMstock = 5;
  for(int i = 0; i < 13; i++){ // for loop to work through waypoints
    int wp[5];
   
    for(int j = 0; j < 5; j++){
      wp[j] = waypoints[i][j]; // takes data about next waypoint "off the shelf"
    }
    #if debug == 1
      DEBUG.print("Next WP: ");
      DEBUG.println(wp[0], DEC);
    #endif
    if(wp[2] == 0){straightAndNarrow(wp[1]);} //if the radius is zero draw a straight line
    else{bool ccw;
      if(wp[2] < 0){ccw=1;} //if the radius is negative the arc is anti-clockwise 
      else{ccw=0;} //if it isn't it's not <- love a good tortology
      int ratio = sweep(wp[1], wp[2], 1); //find ratio and outer target should probably be in raidersOfTheLostARC function... oh well.
      int Do = sweep(wp[1], wp[2]);
      raidersOfTheLostARC(ratio, Do, ccw);
    }
    notify();
    if((bool)wp[4]){MandMrelease(MandMstock); MandMstock--;}
    if(wp[3] != 0){
                    turn(wp[3]);
                    #if debug == 1
                    DEBUG.print("Turn executed: "); DEBUG.print(wp[3]/10, DEC); DEBUG.println(" Degrees");
                    #endif
                   }
  }
  kmn();
}
