#include <iostream>
#include <cmath>
#include "vex.h"
#include "Drive.hpp"
 
 bool a = false;

//cmmnt
bool isClosed = true;
bool isThingyActive = true;
double left_speed = 0; 
//go to gulags
double right_speed = 0;
int UserControlMode = 0;
int AutonomousMode =6;
int AutonMin = 0;
int AutonMax = 8;
bool isred = true;
bool isSortingColors = true;
void Drive_Autonomous(int, int, int);
void Drive_Autonomous_Volt(int, int, int);
void DriveBrake();
void DriveToRing();
void GyroTurnABS(float, int, int, float);
#pragma once
using namespace vex;
 
double goal = 0;


float OneStick = 0;
float TwoStick = 0;
float ThreeStick = 0;
float FourStick = 0;


void drawGUI()
{ 
  // Draws 2 buttons to be used for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
  Brain.Screen.printAt(320, 50, "0. Blue pos");
  Brain.Screen.printAt(320, 70, "1. Red pos");
  Brain.Screen.printAt(320, 90, "2. Red Rush");
  Brain.Screen.printAt(320, 110, "3. Red Awp");
  Brain.Screen.printAt(320, 130, "4. Red 5-Ring E");
  Brain.Screen.printAt(320, 150, "5. Blue Rush");
  Brain.Screen.printAt(320, 170, "6. Blue Awp");
  Brain.Screen.printAt(320, 190, "7. Blue 5-Ring E");
  Brain.Screen.printAt(320, 210, "8. Drive 30 Inches");
  Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
  Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonomousMode);
  Brain.Screen.setFillColor(vex::red);
  Brain.Screen.drawRectangle(20, 50, 100, 100);
  Brain.Screen.drawCircle(300, 75, 25);
  Brain.Screen.printAt(25, 75, "Select");
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(170, 50, 100, 100);
  Brain.Screen.printAt(175, 75, "GO");
  Brain.Screen.setFillColor(black);
}
void selectAuton()
{
  bool selectingAuton = true;

  int xx = Brain.Screen.xPosition(); // get the x position of last touch of the screen
  int yy = Brain.Screen.yPosition(); // get the y position of last touch of the screen

  // check to see if buttons were pressed
  if (xx >= 20 && xx <= 120 && yy >= 50 && yy <= 150)
  { // select button pressed
    AutonomousMode++;
    if (AutonomousMode > AutonMax)
    {
      AutonomousMode = AutonMin; // rollover
    }
    Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonomousMode);
  }

  if (xx >= 170 && xx <= 270 && yy >= 50 && yy <= 150)
  {
    selectingAuton = false; // GO button pressed
    Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           ", AutonomousMode);
  }

  if (!selectingAuton)
  {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(300, 75, 25);
  }
  else
  {
    Brain.Screen.setFillColor(vex::red);
    Brain.Screen.drawCircle(300, 75, 25);
  }

  wait(10, msec); // slow it down
  Brain.Screen.setFillColor(black);
}
bool flip = false;
double counter = 0;

bool backside = false;

double targetL = 0;
double targetA = 10;
double targetAA = 5;

float cap = 100;

enum state {    
    idle,
    loading,
    loading2,
    loading3,
    loading4,
    loading5,
    loading6,
    loading7,
    loading8,
    loading9,
    loading10,
    loading11,
    loading12,
    loading13,
    loading14,
    loading15,
    loading16,
    loading17,
    loading18,
    loading19,
    loading20,
    loading21,
    loading22,
    loading23,
    loading24,
    loading25,
    loading26,
    loading27,
    loading28,
    loading29,
    loading30,
    loading31,
    loading32,
    loading33,
    loading34,
    loading35,
    loading36,
    loading37,
    loading38,
    loading39,
    loading40,
    loading41,
    loading42,
    loading43,
    loading44,
    loading45,
    loading46,
    loading47,
    loading48,
    loading49,
    loading50,

    low,
    low2,
    middle,
    middle2,
    high,
    high2,
};
double Akp = 0.15;
double Akd = 0.55;
double Aki = 0.0;

double kp = 0.7;
double kd = 3;
bool height = false;
state currentState = idle;

void toggleState() {
    switch(currentState){
              case idle:
            
            LeftArm.setVelocity(20,pct);
            RightArm.setVelocity(20,pct);
            Intake.setVelocity(100,pct);
            cap = 25;
            counter = 0;
            kp = 0.15;
            kd = 0;  
          Akp = 0.15;
            Akd = 0.55;
            Aki = 0.0;
            targetL = 0;
          


            if (!backside) {
          if (!flip) {
              if (!height) {
                  currentState = loading;
              } else {
                  currentState = loading7;
              }
          } else {
              if (!height) {
                  currentState = low;
              } else {
                  currentState = loading11;
              }
          }
      } else {
          if (!flip) {
              if (!height) {
                  currentState = loading19;
              } else {
                  currentState = loading31;
              }
          } else { 
              if (!height) {
                  currentState = loading25;
              } else {
                  currentState = loading35;
              }
          }
      }

            break;

        case loading:
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            currentState = loading3;
            counter = 1;

            cap = 65;
            Akp = 0.09;
            kp = 0.3;
            kd = 0;

            targetA = 295;
            targetL = 80;
            Controller.rumble(".");
            
            break; // old 24
          case loading2:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 0;

            wait(250,msec);

            targetA = 360;
            inchDriveC(-2,250,0.6,40);
            DriveBrake();

            clamp.open();

            DriveBrake();
            break;
            case loading3:
            currentState = loading5;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL =440;



            targetA = 295;
            wait(100,msec);
            Rotatedown.open();
            Controller.rumble("..");
            counter = 2;
            break; 
            case loading4:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;


            kp = 0.2;
            kd = 0;
            targetL = 280;
            wait(100,msec);
            targetA = 310;
            wait(200,msec);
            inchDriveC(-2,250,1,40);
            DriveBrake();


            clamp.open();

            break; 
            case loading5:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            // targetL = 0;
            // targetA = 187;
            Akp = 0.25;
            Akd = 0.55;
            Aki = 0.0;

            targetL = 400;
            targetA = 260;
            Rotatedown.open();

            counter = 3;
            Controller.rumble("...");

            break; 
            case loading6:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.05;
            targetL = 390;
            targetA = 265;

            wait(150,msec);
            clamp.open();
        
            break;             
        case low:
            currentState = middle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 185;
            counter = 4;

            // Controller.rumble("....");
            Akp = 0.15;
            Akd = 1.35;
            Rotatedown.close();
          targetL = 0;
          targetA = 160;
            break;
        case low2:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 185;
            Akp = 0.15;
            Akd = 0.55;


          targetA = 175;
            wait(250,msec);
            clamp.open();
            break;
        case middle:
            currentState = high;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;

            Controller.rumble(".....");
            kp = 0.3;
            kd =0;

            targetA = 155;
            targetL = 330;
            Rotatedown.close();


            counter = 5;
            break;
        case middle2:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            targetL= 330;
            Akp = 0.15;
            Akd = 0.55;
            targetA = 170;
            wait(350,msec);
            clamp.open();


            break;
          case high:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            Controller.rumble("......");

            Rotatedown.close();
                kp = 3.2;
                kd = 40;
            targetL = 700;
            targetA = 155;
            counter = 6;
            break;
          case high2:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            Akp = 0.05;
            Akd = 0.0;
            targetA = 169;
            wait(400,msec);
            clamp.open();
            break;
          case loading7:
          currentState = loading9;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble(".");
            counter = 7;

            targetA = 300;
            targetL = 270;

            break;
          case loading8:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 100;
            wait(100,msec);
            targetA = 310;
            wait(100,msec);
            clamp.open();
            break;
          case loading9:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("..");
                kp = 1.2;
                kd = 20;

            targetA = 290;
            targetL = 550;

            wait(100,msec);
            Rotatedown.open();
            counter = 8;
            break;
          case loading10:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            kp = 0.3;
            kd = 0;
            targetL = 360;
            wait(100,msec);
            targetA = 300;
            wait(100,msec);
            clamp.open();
            break;
          case loading11:
            currentState = loading13;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("...");

            targetA = 185;
            targetL = 0;

            counter = 9;
            break;
            case loading12:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("...");

            targetA = 195;
            targetL = 0;
            inchDriveC3(-2,250,1);
            wait(100,msec);
            clamp.open();            
            break;
            case loading13:
          currentState = loading15;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("....");
            counter = 10;
            kp = 0.25;
            kd = 0;
          

            targetL = 160;
            targetA = 160;
            wait(100,msec);
            Rotatedown.close();
            break;
            case loading14:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            
            targetL = 150;
   
            targetA = 173;
            wait(200,msec);
            clamp.open();
            break;
            case loading15:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble(".....");

            targetL = 450;
            targetA = 160;
            wait(100,msec);
            Rotatedown.close();
            counter = 11;
            break;
            case loading16:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            
            targetL = 430;
            targetA = 174;
            wait(200,msec);
            clamp.open();
            break;
            case loading17:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 600;
            targetA = 150;
            Controller.rumble("......");

            wait(100,msec);
            Rotatedown.close();
            counter = 12;
            break;
            case loading18:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 500;
            targetA = 165;
            wait(100,msec);
            clamp.open();
            break;
          case loading19:
          currentState = loading21;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            counter = 13;

            cap = 65;
            Akp = 0.25;
            Akd=0.55;
            targetL = 0;
            targetA = 16;

            Controller.rumble(".");

            break;
          case loading20:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 0;
            targetA = 10;

            wait(100,msec);
            clamp.open();
            break;
          case loading21:
          currentState = loading23;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            counter = 14; 
            kp = 0.45;
            kd = 3;

            Controller.rumble("..");
          targetA = 10.3;
            targetL = 450;

            break;
          case loading22:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetA = 10.3;

            targetL = 230;

            wait(300,msec);
            clamp.open();
            break;
          case loading23:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            Controller.rumble("...");

            targetL = 350;
            targetA = 42.5;
            wait(100,msec);
            Rotatedown.close();
            counter = 15;
            break;
            case loading24:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetA = 42.3;
            targetL = 280;
            wait(100,msec);
            clamp.open();
            break;
            case loading25:
            currentState = loading27;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("....");
            counter = 16;
            targetL = 0;
            targetA = 130;

            break;
            case loading26:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;

            targetA = 120;
            wait(100,msec);
            clamp.open();
            break;
            case loading27:
          currentState = loading29;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble(".....");
            
            targetA = 135;
            targetL =280;
            kp = 0.3;
            kd = 0;
            wait(100,msec);
            Rotatedown.open();
            counter = 17;
            break;
            case loading28:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 95;

            targetL = 290;
            targetA = 120;
            wait(100,msec);
            clamp.open();
            break;
            case loading29:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            Controller.rumble("......");
                kp = 2.0;
                kd = 40;
            targetA = 139;
            targetL = 670;
            wait(100,msec);
            Rotatedown.open();
            counter = 18;
            break;
            case loading30:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 95;
            targetA = 120;
            wait(200,msec);
            clamp.open();
            break;
          case loading31:
          currentState = loading33;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble(".");
            counter = 19;

            targetA = 16;
            targetL = 500;
            if(Rotatedown.value()==1){
              wait(100,msec);
              Rotatedown.close();
            }

            break;
          case loading32:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;

            targetL = 300;
            wait(200,msec);
            clamp.open();
            break;
          case loading33:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("..");

            targetA = 43;
            targetL = 480;
            wait(100,msec);
            counter = 20;
            break;
          case loading34:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetA = 42;
            targetL = 400;
            wait(200,msec);
            clamp.open();
            break;
          case loading35:
          currentState = loading37;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("...");
            kp = 0.005;
            kd = 0;
            Akp = 0.15;
            Akd = 0.75;
            targetA = 153;
            targetL = 0;

            counter = 21;
            break;
            case loading36:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;

            targetA = 125;
            wait(100,msec);
            Rotatedown.open();
            clamp.open();
            break;
            case loading37:
          currentState = loading39;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("....");
            counter = 22;
            targetL = 375;
            kp = 0.5;
            kd = 2;
            targetA = 145;
            wait(100,msec);
            Rotatedown.open();
            break;
            case loading38:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 330;
            targetA = 125;
            wait(100,msec);
            Rotatedown.open();
            clamp.open();
            break;
            case loading39:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            Controller.rumble(".....");
                kp = 5.2;
                kd = 50;
            targetL = 680;
            targetA = 150u                                         ;
            wait(100,msec);
            Rotatedown.open();
            counter = 23;
            break;
            case loading40:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            kp = 0.45;
            kd=0;
            targetL = 560;
            targetA = 125;
            wait(200,msec);
            Rotatedown.open();
            clamp.open();
            break;
            case loading41:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Controller.rumble("......");

            targetA = 185;
            wait(100,msec);
            Rotatedown.open();
            counter = 24;
            break;
            case loading42:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;

            targetA = 185;
            wait(100,msec);
            Rotatedown.open();

            break;
            case loading43:
            currentState = loading45;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            counter = 25;
            kp = 0.2;
            kd = 0.2;
            cap = 65;
            targetL = 300;
            targetA = -12;
            wait(500,msec);
            Rotatedown.open();

            targetL = 250;


            break; 
            case loading44:
            Akp = 0.25;
            Akd=0.55;
            if (!backside) {
                if (!flip) {
                    if (!height) {
                        currentState = loading;
                    } else {
                        currentState = loading7;
                    }
                } else {
                    if (!height) {
                        currentState = low;
                    } else {
                        currentState = loading11;
                    }
                }
            } else {
                if (!flip) {
                    if (!height) {
                        currentState = loading19;
                    } else {
                        currentState = loading31;
                    }
                } else {
                    if (!height) {
                        currentState = loading25;
                    } else {
                        currentState = loading35;
                    }
                }
            }
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 0;
            targetA = 19;
            Rotatedown.close();
            counter = 26;

            break;
            case loading45:
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            targetL = 340;
            wait(300,msec);
            targetA = 90;
            wait(1000,msec);
            counter = 27;
            kd = 16;
            if (!backside) {
                if (!flip) {
                    if (!height) {
                        currentState = loading;
                    } else {
                        currentState = loading7;
                    }
                } else {
                    if (!height) {
                        currentState = low;
                    } else {
                        currentState = loading11;
                    }
                }
            } else {
                if (!flip) {
                    if (!height) {
                        currentState = loading21;
                    } else {
                        currentState = loading31;
                    }
                } else {
                    if (!height) {
                        currentState = loading25;
                    } else {
                        currentState = loading35;
                    }
                }
            }
            break;
            case loading46:
            currentState = loading47;
            counter = 28;
            targetA = 215;
            targetL = 0;
            Rotatedown.close();
            break;
            case loading47:
            currentState = loading46;
            Akp = 0.2;
            Akd = 0.55;
            Aki = 0.0;
            counter = 29;
            targetA = 234;
            targetL = 0;
            counter = 29;

            break;
            case loading48:
            currentState = idle;
            Akp = 0.2;
            Akd = 0.55;
            Aki = 0.00;
            counter = 29;
            targetA = 248.5;
            targetL = 0;
            Rotatedown.open();

            break;
            



    }
}
bool kyle = 0;
bool autorun = false;
float acuracy = 1.8;
void idlePosition(){
    if(counter == 0||counter == 26|| counter== 27){
    if (!backside) {
          if (!flip) {
              if (!height) {
                  currentState = loading;
              } else {
                  currentState = loading7;
              }
          } else {
              if (!height) {
                  currentState = low;
              } else {
                  currentState = loading11;
              }
          }
      } else {
          if (!flip) {
              if (!height) {
                  currentState = loading21;
              } else {
                  currentState = loading31;
              }
          } else { 
              if (!height) {
                  currentState = loading25;
              } else {
                  currentState = loading35;
              }
          }
      }
    }
    if(counter == 1&&Rotatedown.value() == 0){
        wait(600,msec);
        Rotatedown.open();
    }
    
    if(Rotatedown.value() == 1){
      if(counter == 14||counter == 11){
        wait(100,msec);
        Rotatedown.close();
      }
    }
    if(counter == 16||counter == 7||counter ==21){
      if(Rotatedown.value()== 0)
        wait(100,msec);
        Rotatedown.open();
    }
    if(counter == 0 && clawSensor.objectDistance(inches)<=acuracy&&clamp.value() == 1){
      clamp.set(false);
      wait(100,msec);
      currentState = loading44;
      toggleState();

    }
    if (counter == 0&&targetA!=4){
      wait(480,msec);
      if(counter ==0){
      targetA = 4;
      Rotatedown.close();
      }
    }
    if(autorun == true&& statechanger == 1){
      currentState = loading46;
      toggleState();
    }
    if(autorun == true&& statechanger == 2){
      currentState = loading47;
      toggleState();
    }
    if(autorun == true&& statechanger == 3){
      currentState = idle;
      toggleState();
    }


}
void idle2(){
  
}
    double intergal = 0;
    double lasterror = 0;

    double Alasterror = 0;
void liftControl() {

    double kg = 0;
    double x = (LeftArm.position(deg)+RightArm.position(deg)/2);
    double error = targetL - x;
    double speed = error * kp+kd*(error-lasterror)+ kg;
    double Akg = 0.0;

    double Ax = (Arm.position(deg));
    double Aerror = targetA - Ax;

    if (fabs(Aerror) < 7 && fabs(Aerror) > 1)
    {
      intergal += Aerror;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 25)
    {
      intergal = 25;
    }
    else if (intergal <= -25)
    {
      intergal = -25;
    }
    if(fabs(error) >=3){
    std::cout << error << "\n";

    }

    double Aspeed = Aerror * Akp+Akd*(Aerror-Alasterror)+ Aki *intergal;
      Brain.Screen.printAt(25,200,"Aerror:%.2f",(Aerror));
      Brain.Screen.printAt(25,225,"Ax:%.2f",(Ax));
      
    lasterror = error;

    if(counter ==29){
      if(Ax>=236){
        Aspeed = -1;
      // }else if (AX<=)
    }
  }
    

    LArm.spin(fwd, -Aspeed, volt);
    RArm.spin(fwd, -Aspeed, volt);
    LeftArm.spin(fwd, speed, pct);
    RightArm.spin(fwd, speed, pct);
    Alasterror = Aerror;
}



enum Astate {    
    lowest,
    store,
    Slow,
    Smiddle,
    Shigh,
};
Astate Armstate = lowest;

void AState() {
    switch(Armstate){
        case lowest:
            Armstate = store;

            cap = 65;
            
            targetAA = 5;


            break;

        case store:
            Armstate = Slow;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            targetAA = 80;
            break; // old 24
        case low:
            Armstate = Smiddle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;

          targetAA = 28;
            break;
        case Smiddle:
            Armstate = Shigh;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            targetAA = 90;

            break;
        case Shigh:
            Armstate = lowest;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            targetL = 120;

            break;

    }
}


void ArmControl() {
  
    double kp = 0.05;
    double kd = 0.05;
    double kg = 0.0;
    double lasterror = 0;
    double x = (Arm.position(deg));
    double error = targetA - x;
    double speed = error * kp+kd*(error-lasterror)+ kg;
      // while (true){
      // if(a==true){
    x = (Arm.position(deg));
    error = targetA - x;
    speed = error * kp+kd*(error-lasterror)+ kg;
    Brain.Screen.printAt(25,200,"speed:%.2f",(speed));
    
  

    LArm.spin(fwd, -speed, volt);
    RArm.spin(fwd, -speed, volt);
    lasterror = error;
      // }
      // }
}
bool Aa = false;
void AArmControl(){
  
    double kp = 0.1;
    double kd = 0.05;
    double kg = 0.0;
    double lasterror = 0;
    double x = (Arm.position(deg));
    double error = targetA - x;
    double speed = error * kp+kd*(error-lasterror)+ kg;
    while (true){
      if(Aa==true){
        break;
      }
    x = (Arm.position(deg));
    error = targetAA - x;
    speed = error * kp+kd*(error-lasterror)+ kg;
    Brain.Screen.printAt(25,200,"speed:%.2f",(speed));
    
  

    LArm.spin(fwd, -speed, volt);
    RArm.spin(fwd, -speed, volt);
    lasterror = error;
    }
      // }
}

void doubleToggle(){
  if(counter == 1){
  currentState = loading2;
  toggleState();
  } else if(counter == 2){
  currentState = loading4;
  toggleState();
  } else if(counter == 3){
  currentState = loading6;
  toggleState();
  } else if(counter == 4){
  currentState = low2;
  toggleState();
  } else if(counter == 5){
  currentState = middle2;
  toggleState();
  } else if(counter == 6){
  currentState = high2;
  toggleState();
  }else if(counter == 7){
  currentState = loading8;
  toggleState();
  }else if(counter == 8){
  currentState = loading10;
  toggleState();
}else if(counter == 9){
  currentState = loading12;
  toggleState();
  }else if(counter == 10){
  currentState = loading14;
  toggleState();
}else if(counter == 11){
  currentState = loading16;
  toggleState();
  }else if(counter == 12){
  currentState = loading18;
  toggleState();
}else if(counter == 13){
  currentState = loading20;
  toggleState();
  }else if(counter == 14){
  currentState = loading22;
  toggleState();
}else if(counter == 15){
  currentState = loading24;
  toggleState();
  }else if(counter == 16){
  currentState = loading26;
  toggleState();
}else if(counter == 17){
  currentState = loading28;
  toggleState();
  }else if(counter == 18){
  currentState = loading30;
  toggleState();
}else if(counter == 19){
  currentState = loading32;
  toggleState();
}else if(counter == 20){
  currentState = loading34;
  toggleState();
  }else if(counter == 21){
  currentState = loading36;
  toggleState();
}else if(counter == 22){
  currentState = loading38;
  toggleState();
  }else if(counter == 23){
  currentState = loading40;
  toggleState();
}else if(counter == 24){
  currentState = loading42;
  toggleState();
  }
}
void ToggleRight(){
  currentState = loading43;
  toggleState();
}
void ToggleLeft(){
  currentState = loading46;
  toggleState();
}
void Toggle(){
  currentState = idle;
  toggleState();
}

void LIFTWork(){
if(counter == 0){
  clamp.set(false);
  currentState = loading44;
  toggleState();
}else if(counter == 26){
  clamp.set(true);
  wait(500,msec);

  currentState = idle;
  toggleState();
  kyle = true;
  kyle = false;
}else{
  clamp.set(!clamp.value());
}



  }
void position(){
  flip = false;
}

void position2(){
  flip = true;
}
void position3(){
  backside = false;
}
void position4(){
  backside = true;
}
void AntlerDEscore(){

    Rotatedown.set(!Rotatedown.value());
}
void Setup_UserControl()
{
  //set all the drive motors brake modes to coast.
  LeftFront.setBrake(coast);
  LeftMiddle.setBrake(coast);
  LeftBack.setBrake(coast);
  RightFront.setBrake(coast);
  RightMiddle.setBrake(coast);
  RightBack.setBrake(coast);

  // Set all of the drive motors to 100 percent velocity.
  LeftFront.setVelocity(100, pct);
  LeftMiddle.setVelocity(100, pct);
  LeftBack.setVelocity(100, pct);
  RightFront.setVelocity(100, pct);
  RightMiddle.setVelocity(100, pct);
  RightBack.setVelocity(100, pct);
  // Controller.ButtonL1.pressed(toggleState);
  // Controller.ButtonL2.pressed(doubleToggle);

}
void load(){
  clamp.set(!clamp.value());
}
void doinkerRtoggle()
{
  height = !height;
  Controller.rumble(".");
}


void driver_doinker()
{
  Controller.ButtonDown.pressed(doinkerRtoggle);
}

enum Color
{
  blue,
  red
};
bool colorDetected(Color color)
{
  int hue = OpticalSensor.hue();
  switch (color)
  {
  case Color::red:
    if (hue >= 4 && hue <= 27)
    {
      return true;
    }
    break;
  case Color::blue:
    if (hue >= 185 && hue <= 225)
    {
      return true;
    }
    break;
  }
  return false;
} 
//
int intakeState = 0;
bool sortColor = false;
int ladybrownposition = 0;


// void liftmacro(){
//   lift.setVelocity(100,pct);


//   if(ladybrownposition == 0){
//     lift.spinTo(540.5, degrees, true);
//     ladybrownposition = 1;
//   }
//   else{
//     lift.spinTo(640, degrees, true);
//     ladybrownposition = 2;
//   }
// }

    

// void liftrest(){
//   lift.spinTo(0, degrees, true);
//   ladybrownposition = 0;
// }
float Outake_state = 0;
void intakeControl()
{
  float count = 0;
  Outake.setVelocity(100,pct);
  while (true)
  {
    if (intakeState == 1) {
          OpticalSensor.setLightPower(100, percent);
    }
    else {
          OpticalSensor.setLightPower(0, percent);
    }

    if (isSortingColors)
    {
      if (isred)
      {
        if (colorDetected(Color::blue) && OpticalSensor.isNearObject())
        {
          sortColor = true;
        }
      }
      else if (colorDetected(Color::red) && OpticalSensor.isNearObject())
      {
        sortColor = true;
      }
    }
    



    if (intakeState == 1){
      Intake.spin(fwd);
      Outake.spin(fwd);
        // if(Intake.efficiency(pct)<=12){
        //   count++;
        // }
      wait(10, msec);
    }
    else if(intakeState == -1){
    Intake.spin(reverse);
    Outake.spin(reverse);

    wait(10, msec);
    }
    else {
      Intake.stop();
      Outake.stop();
    }

  }
}
void Spin_Intake(bool direction) {
  //78846781b
  // If the direction is true...
  if (direction) {

    // Spin the intake forward.
    intakeState = -1;

    // If the direction is false...
  }
  else if (direction == false)
  {

    // Spin the intake backward.
    intakeState = 1;
  }
}
void Spin_Outake(bool direction) {
  //78846781b
  // If the direction is true...
  if (direction) {

    // Spin the intake forward.
    Outake_state = -1;

    // If the direction is false...
  }
  else if (direction == false)
  {

    // Spin the intake backward.
    Outake_state = 1;
  }
}    
void Stop_Intake()
{
  intakeState = 0;
}
void Stop_Outake()
{
  Outake_state = 0;
}
void Intake_UserControl()
{  
 
  // If the button R2 is being pressed...
  if (Controller.ButtonR2.pressing())
  {

    // Spin the intake forward.
    Spin_Intake(false);

    // AutoUnjam_Intake();

    // If the button R1 is being pressed...
  }
  else if (Controller.ButtonR1.pressing())
  {

    // Spin the intake backward.
    Spin_Intake(true);
    // ColorSorting();
    // vex::thread colorSortingThread(ColorSorting);
    // AutoUnjam_Intake();
    // If neither button R2 nor R1 is being pressed...
  }
  else
  {

    // Stop the intake motor from spinning.
    Stop_Intake();
  } 

  // If the button R2 is being pressed...
  if (Controller.ButtonL2.pressing())
  {

    // Spin the intake forward.
    Spin_Outake(false);

    // AutoUnjam_Intake();

    // If the button R1 is being pressed...
  }
  else if (Controller.ButtonL1.pressing())
  {

    // Spin the intake backward.
    Spin_Outake(true);
    // ColorSorting();
    // vex::thread colorSortingThread(ColorSorting);
    // AutoUnjam_Intake();
    // If neither button R2 nor R1 is being pressed...
  }
  else
  {

    // Stop the intake motor from spinning.
    Stop_Outake();
  }
  
}

void Drive_UserControl()
{

  // Set the stick variables to the axis's of the controller sticks.
  OneStick = Controller.Axis1.position();
  TwoStick = Controller.Axis2.position();
  ThreeStick = Controller.Axis3.position();
  FourStick = Controller.Axis4.position();

  // Calculate left_speed and right_speed.
  left_speed = (ThreeStick + (OneStick * fabs(OneStick)) / 100) * 12 / 100;
  right_speed = (ThreeStick - (OneStick * fabs(OneStick)) / 100) * 12 / 100;

  // left_speed = (ThreeStick + (OneStick));
  // right_speed = (ThreeStick - (OneStick));
  // double mag = std::max(fabs(left_speed),fabs(right_speed))/100.0;
  // // Spin the motors at left_speed in volts.
  //   std::cout <<left_speed  << std::endl;

  // if(mag > 1.0){
  //   left_speed=left_speed/mag;
  //   right_speed=right_speed/mag;    
    
  // }
  // left_speed = left_speed*(12.0 / 100.0);
  // right_speed = right_speed*(12.0 / 100.0);
    // std::cout <<mag  << std::endl;
    // std::cout <<left_speed  << std::endl;

  LeftFront.spin(fwd, left_speed, volt);
  LeftMiddle.spin(fwd, left_speed, volt);
  LeftBack.spin(fwd, left_speed, volt);

  // Spin the motors at right_speed in volts.
  RightFront.spin(fwd, right_speed, volt);
  RightMiddle.spin(fwd, right_speed, volt);
  RightBack.spin(fwd, right_speed , volt);

}

void loop(){
  while(true){
  idlePosition();
  }
}

void test (){
}
void autonomous(void)
{
  float START_ANGLE;
  float HEADING;
  vex::thread(intakeControl);
  vex::thread odom (icc_tracking);
  vex::thread states(loop);



  switch (AutonomousMode)
  { 
    case 0:
    LeftArm.resetPosition();
    RightArm.resetPosition();
    Intake.setVelocity(10,pct);
    Gyro.setRotation(0,deg);
    backside = true;
    currentState = loading19;
    toggleState();
    Akp = 0.0;
    Akd = 0.0;
  
    x=0;
    y=0;

    inchDriveC2(-15.3,1200,52,-60,5,-56,0,-56);

    Rotatedown.open();
    clamp.open();
    wait(300,msec);




    inchDriveC(8,250,0.8,60);
    gyroTurnF(-25,1,0.2,550);    
    Akp = 0.15;
    Akd = 0.55;


    ToggleLeft();
    clamp.open();
    inchDriveC3(20,400,0.6);
    gyroTurnF(0,1,1,800);

    toggleState();
    Akp = 0.6;
    Akd = 1;
    Aki = 0;

    wait(200,msec);
    inchDriveC(-17,1000,0.6,40);
    DriveBrake();
    currentState = idle;
    toggleState();









    



    
    //launch loader
    


    







    DriveBrake();
    // gyropivotRC(45,true);
    // DriveBrake();

    break;
    case 1:
    LeftArm.resetPosition();
    RightArm.resetPosition();
    Intake.setVelocity(10,pct);
    Gyro.setRotation(0,deg);
    backside = true;
    currentState = loading19;
    toggleState();
    Akp = 0.0;
    Akd = 0.0;
  
    x=0;
    y=0;

    inchDriveC2(-15.3,1200,52,-60,5,-56,0,-56);

    Rotatedown.open();
    clamp.open();
    wait(300,msec);




    inchDriveC(8,250,0.8,60);
    gyroTurnF(-25,1,0.2,550);    
    Akp = 0.15;
    Akd = 0.55;


    ToggleLeft();
    clamp.open();
    inchDriveC3(20,400,0.6);
    gyroTurnF(0,1,1,800);

    toggleState();
    Akp = 0.6;
    Akd = 1;
    Aki = 0;

    wait(200,msec);
    inchDriveC(-17,1000,0.6,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    acuracy = 2.8;
    TTP(22,-45.5,10000,180);
    inchDriveCE(-27,1100,0.6,10);
    currentState = loading21;
    toggleState();
    MTPB(20,-12,1200,50,35);
    DriveBrake();
    wait(200,msec);
    doubleToggle();

    wait(300,msec);
    gyroTurnF(180,1,0.7,800);
    inchDriveC(14,1300,0.8,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    acuracy = 2.30;

    TTP(48.7,-11,13000,180);
    inchDriveCE(-33,1500,0.8,10);
    currentState = loading31;
    toggleState();
    Akp = 0.15;
    Akd = 0.55;
    kp = 0.75;
    kd = 0;
    MTPB(13,-30,1500,100,80);
    MTPB(-27,-13,1300,80,0);
    doubleToggle();
    kp = 0.45;



    break;
    case 2:
    LeftArm.resetPosition();
    RightArm.resetPosition();
    Intake.setVelocity(10,pct);
    Gyro.setRotation(0,deg);
    backside = true;
    currentState = loading19;
    toggleState();
    Akp = 0.0;
    Akd = 0.0;
  
    x=0;
    y=0;

    inchDriveC2(-22.3,1200,52,60,5,51,0,51);

    Rotatedown.open();
    clamp.open();
    wait(300,msec);




    inchDriveC(8,250,0.8,60);
    gyroTurnF(25,1,0.2,550);    
    Akp = 0.15;
    Akd = 0.55;


    ToggleLeft();
    clamp.open();
    inchDriveC3(20,400,0.6);
    gyroTurnF(0,1,1,800);

    toggleState();
    Akp = 0.3;
    Akd = 1;
    Aki = 0;

    wait(200,msec);
    inchDriveC(-17,1000,0.6,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    acuracy = 2.8;
    TTP(-22,-45,10000,180);
    inchDriveCE(-27,1100,0.6,10);
    currentState = loading21;
    toggleState();
    MTPB(-20,-12,1200,50,35);
    DriveBrake();
    wait(200,msec);
    doubleToggle();

    wait(300,msec);
    gyroTurnF(180,1,0.7,800);
    inchDriveC(14,1300,0.8,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    kp = 0.45;
    kd = 0;
    acuracy = 2.30;

    TTP(-45.4,-11,13000,180);
    inchDriveCE(-33,1500,0.8,10);
    currentState = loading31;
    toggleState();
    Akp = 0.15;
    Akd = 0.55;
    kp = 0.75;
    kd = 0;
    MTPB(-13,-30,1500,100,80);
    MTPB(29,-16,1500,80,40);
    doubleToggle();
    kp = 0.45;
    break;
    case 3:
    LeftArm.resetPosition();
    RightArm.resetPosition();
    Intake.setVelocity(10,pct);
    Gyro.setRotation(0,deg);
    backside = true;
    currentState = loading19;
    toggleState();
    Akp = 0.0;
    Akd = 0.0;
  
    x=0;
    y=0;

     inchDriveC2(-22.3,1200,52,60,5,51,0,51);

    Rotatedown.open();
    clamp.open();
    wait(300,msec);




    inchDriveC(8,250,0.8,60);
    gyroTurnF(25,1,0.2,550);    
    Akp = 0.15;
    Akd = 0.55;


    ToggleLeft();
    clamp.open();
    inchDriveC3(20,400,0.6);
    gyroTurnF(0,1,1,800);

    toggleState();
    Akp = 0.35;
    Akd = 1;
    Aki = 0;

    wait(200,msec);
    inchDriveC(-17,1000,0.6,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    acuracy = 2.8;
    TTP(-22,-45,10000,180);
    inchDriveCE(-27,1100,0.6,10);
    currentState = loading21;
    toggleState();
    MTPB(-20,-12,1200,50,35);
    DriveBrake();
    wait(200,msec);
    doubleToggle();

    wait(300,msec);
    gyroTurnF(180,1,0.7,800);
    inchDriveC(14,1300,0.8,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    kp = 0.45;
    kd = 0;
    acuracy = 2.30;

    TTP(-45.4,-11,13000,180);
    inchDriveCE(-33,1500,0.8,10);
    toggleState();
    toggleState();
    Akp = 0.15;
    Akd = 0.55;
    kp = 0.75;
    kd = 0;
    TTP(-22,-14.5,1000000,180);



    wait(300,msec);
    inchDriveC(-8.5,1400,0.8,40);
    DriveBrake();
    doubleToggle();
    wait(300,msec);
    gyroTurnF(-90,1,0.5,500);
    inchDriveC(30,1200,0.6,40);
    backside = false;
    acuracy = 2.3;


    toggleState();
            kp = 0.15;
            kd = 0; 
    gyroTurnF(180,1.0,0.2,1500);
    inchDriveD(1.6,1000,1,40);
    wait(100,msec);
    inchDriveC(10,500,1,60);

    TTP(-43.5,-33,3000);
    currentState = loading;
    toggleState(); 
    wait(100,msec);
    
    kp = 0.45;
    kd = 0;
    inchDriveC(17.5,1500,0.5,20);
    DriveBrake();
    doubleToggle();
    wait(200,msec);
    DriveBrake();
    inchDriveC(-5,500,0.6,40);
    currentState = idle;
    toggleState();
    kp = 0.45;
    kd = 0;

    inchDriveC(-16,1500,0.6,40);
    DriveBrake();
    clamp.open();

    gyroTurnF(180);
    inchDriveD(1.6,1000,1,40);
    clamp.close();
    wait(100,msec);
    inchDriveC(10,500,1,60);

    TTP(-43.5,-33,3000);
    currentState = loading3;
    toggleState(); 
    wait(500,msec);

    
    kp = 0.45;
    kd = 0;
    inchDriveC(19,1500,0.5,15);
    DriveBrake();
    doubleToggle();
    wait(300,msec);
    DriveBrake();
    inchDriveC(-5,500,0.6,40);
    currentState = idle;
    toggleState();
    kp = 0.45;
    kd = 0;
    inchDriveC(-10,1500,0.6,40);
    DriveBrake();
    clamp.open();


    gyroTurnF(180);
    inchDriveD(1.6,1000,1,40);

    clamp.close();
    wait(100,msec);
    inchDriveC(10,500,1,60);

    TTP(-43.5,-32,3000);
    currentState = loading5;
    toggleState(); 
    wait(800,msec);


    
    kp = 0.45;
    kd = 0;
    inchDriveC(18,1500,0.5,15);
    inchDriveC(-7,1000,0.5,10);
    DriveBrake();
    doubleToggle();
    wait(100,msec);
    DriveBrake();
    inchDriveC(-5,500,0.6,40);
    currentState = idle;
    toggleState();
    clamp.open();

    kp = 0.45;
    kd = 0;
    inchDriveC(-10,1100,0.6,40);
    clamp.open();

    DriveBrake();

    gyroTurnF(180);
    inchDriveC(-12,1000,0.5,30);
    clamp.close();
    wait(100,msec);
    inchDriveC(10,500,1,60);

    TTP(-43,-32,3000);
    currentState = low;
    toggleState(); 

    
    kp = 0.45;
    kd = 0;
    inchDriveC(20,1500,0.4,10);
    DriveBrake();
    doubleToggle();
    wait(100,msec);
    DriveBrake();
    inchDriveC(-5,500,0.6,40);
    currentState = idle;
    toggleState();
    kp = 0.45;
    kd = 0;
    clamp.open();

    inchDriveC(-13,1500,0.6,40);
    DriveBrake();

    gyroTurnF(180);
    inchDriveD(1.7,1000,1,40);
    clamp.close();
    wait(100,msec);
    inchDriveC(10,500,1,60);

    TTP(-43.5,-32,3000);
    currentState = middle;
    toggleState(); 

    
    kp = 0.45;
    kd = 0;
    inchDriveC(20,1500,0.4,10);
    DriveBrake();
    doubleToggle();
    wait(100,msec);
    DriveBrake();
    inchDriveC(-5,500,0.6,40);

    toggleState();
    kp = 0.45;
    kd = 0;
    inchDriveC(-13,1000,0.6,40);
    DriveBrake();

    gyroTurnF(180);
    MTP(-58,-60,2400);
    gyroTurnF(-90);
    ToggleLeft();
    toggleState();
    Akp = 0.4;
    Akd = 1;
    Aki = 0;
    wait(900,msec);
    inchDriveC(-40,1000,0.8,40);
    DriveBrake();









    break;
    case 4:
    LeftArm.resetPosition();
    RightArm.resetPosition();
    Intake.setVelocity(30,pct);
    Gyro.setRotation(0,deg);
    backside = true;

    Akp = 0.15;
    Akd = 0.55;
  
    x=0;
    y=0;

    currentState = loading3;
    toggleState();
    wait(150,msec);
    currentState = loading;
    toggleState();
    inchDriveC(8,300,1,70);
    doubleToggle();

    wait(100,msec);
    currentState = idle;
    toggleState();
    clamp.open();
    inchDriveC3(-10,680,1);

    gyroTurnF(80,1,1,1200);

    toggleState();
    Akp = 0.2;
    Akd = 0.0;
    Aki = 0;
    MTP(18, 4, 1000);
    MTPB(-4,8,1000,60);


    doubleToggle();
    wait(300,msec);
    inchDriveC3(10,300,1);
    ToggleLeft();
    gyroTurnF(180,1,0.2,900);
    inchDriveC3(20,500,0.6);

    toggleState();
    Akp = 0.15;
    Akd = 0.0;
    Aki = 0;
    wait(200,msec);
    inchDriveC3(-18,1000,1);

    toggleState();
    TTP(1,30.5,1000,180);
    inchDriveC3(-20,1400,0.6,false);
    height = true;
    currentState = loading31;
    toggleState();
    kp = 0.45;
    kd = 0;
    TTP(-24,80.5,1500,180);
    MTPB(-21,80,1600,60,30);
    doubleToggle();
    wait(300,msec);
    inchDriveC(6,300,1,60);
    MTP(-58,60,1200);
    ToggleLeft();
    toggleState();
    Akp = 0.35;
    Akd = 0.0;
    Aki = 0;
    wait(800,msec);
    MTPB(-24,60,600,100,80);




    

    break;
    case 5:
    Intake.setVelocity(30,pct);
    Gyro.setRotation(0,deg);
    x=0;
    y=0;



    inchDriveC3(5,450,0.6);
    DriveBrake();
    intakeState = -1;
    wait(500,msec);



    inchDriveC3(-5,250,1);
    wait(200,msec);
    currentState = loading7;
    toggleState();
    gyroTurnF(45);

    MTP(23, 7, 2000);

    gyroTurnF(180);
    // targetL = 350;

    inchDriveC3(20,700,0.8);
    inchDriveC3(-5,250,1);
    inchDriveC3(10,600,1);

    MTPB(55, 14, 2000,50);
    targetA = 28;
    wait(700,msec);
    LIFTWork();
    inchDriveC3(10,600,1);
    targetA = 5;
    MTPB(-3, 22.5, 2000,60);
    inchDriveC3(-2,600,0.4);
    DriveBrake();

    LIFTWork();
    targetA = 70;
    MTPB(0, 10, 2000,60);
















    


    break;
    case 6:
    LeftArm.resetPosition();
    RightArm.resetPosition();
    Intake.setVelocity(10,pct);
    Gyro.setRotation(0,deg);
    backside = true;
    currentState = loading19;
    toggleState();
    Akp = 0.0;
    Akd = 0.0;
  
    x=0;
    y=0;

    inchDriveC2(-22.3,1200,52,60,5,48,0,48);

    Rotatedown.open();
    clamp.open();
    wait(300,msec);




    inchDriveC(8,250,0.8,60);
    gyroTurnF(25,1,0.2,550);    
    Akp = 0.15;
    Akd = 0.55;


    ToggleLeft();
    clamp.open();
    inchDriveC3(20,400,0.6);
    gyroTurnF(0,1,1,800);

    toggleState();
    Akp = 0.3;
    Akd = 1;
    Aki = 0;

    wait(200,msec);
    inchDriveC(-17,1000,0.6,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    acuracy = 2.8;
    TTP(-22,-45,10000,180);
    inchDriveCE(-27,1100,0.6,10);
    currentState = loading21;
    toggleState();
    MTPB(-20,-10,1200,50,35);
    DriveBrake();
    wait(200,msec);
    doubleToggle();

    wait(300,msec);
    gyroTurnF(180,1,0.7,800);
    inchDriveC(14,1300,0.8,40);
    DriveBrake();
    currentState = idle;
    toggleState();
    kp = 0.45;
    kd = 0;
    acuracy = 2.30;
    MTPB(-3,-14,1200,80,0.6);
    gyroTurnF(-90);
    inchDriveC2(42.3,1500,52,-90,5,180,0,180);

    // currentState = loading31;
    // toggleState();
    // MTP(0,-14,1200);
    // MTPB(29,-16,1500,80,40);
    // doubleToggle();




    



    break;
    case 7:
    Intake.setVelocity(100,pct);
    Gyro.setRotation(0,deg);
    x=0;
    y=0;
    targetA = 50;
    inchDriveC3(-5,250,1);
    doinkerR.open();

    wait(250,msec);
    AntlerDEscore();
    inchDriveC3(10,1000,1);
    inchDriveC3(-9,500,1);
    intakeState = -1;
    doinkerR.close();

    MTP(-12.5, 0, 2000);
    gyropivotL(55,true);
    inchDriveC3(-10,1100,1);
    MTP(-16, 1, 1500);
    inchDriveC3(-10,1100,1);
    AntlerDEscore();
    targetA = 50;


    
    DriveBrake();




    

    // MTP(24,48,1000000);
    // MTP(0,0,10000);

    break;
    case 8:

    Intake.setVelocity(30,pct);
    Gyro.setRotation(0,deg);
    x=0;
    y=0;
    kp = 0.45;
    kd = 0;
    TTP(24,24,10000);
    DriveBrake();
    intakeState = -1;
    break; 
    
  }
} 
void abc(){
  wait(200,msec);
  if(Controller.ButtonDown.pressing()){
    ToggleLeft();
  } else{
    ToggleRight();
  }
}
void pre_auton(void)
{
  Gyro.calibrate();
  Gyro.setRotation(0,deg);
  drawGUI();
  Brain.Screen.pressed(selectAuton);
  vex::thread intakeThread(intakeControl);
  vex::thread states(loop);



    {
while (true) {
  liftControl();
 Brain.Screen.printAt(1, 20, "Gyro Rotation: %f", Gyro.rotation());
wait(10, msec);
} };
}
void usercontrol(void)
{

  Brain.Screen.clearScreen();
  Controller.Screen.clearScreen();
  
  wait(50, msec);

  Controller.Screen.setCursor(1, 0);  


  Controller.Screen.print("CLR:ON");


// Default Control Scheme
    // Call the function "Setup_UserControl".
    Setup_UserControl();

    //  vex::thread colorSortingThread(ColorSorting);
    Intake.setVelocity(100,pct);
    Outake.setVelocity(100,pct);
    vex::thread odom(icc_tracking);
    //trollol lol 
    AIVision1.tagDetection(true);
    Controller.ButtonA.pressed(LIFTWork);
    // Controller.ButtonUp.pressed(position);
    // Controller.ButtonRight.pressed(position2);
    Controller.ButtonLeft.pressed(position3);



    Controller.ButtonX.pressed(Toggle);
    Controller.ButtonL2.pressed(doubleToggle); 
    Controller.ButtonL1.pressed(toggleState); 
    Controller.ButtonY.pressed(doinkerRtoggle); 
    Controller.ButtonB.pressed(abc);
  
    // Controller.ButtonDown.pressed(Align);  
    Intake.setVelocity(30,pct);
    Gyro.setRotation(0,deg);

    Akp = 0.15;
    Akd = 0.55;
  
    x=0;
    y=0;







   Aa = true;
    
    Intake.setVelocity(100,pct);

    acuracy = 1.8;
    Gyro.setRotation(0,deg);

    while (true)
    {
      // if(Controller.ButtonL1.pressing()){

      //   LeftArm.spin(reverse,100,pct);
      //   RightArm.spin(reverse,100,pct);


      // }else if (Controller.ButtonL2.pressing()){
      //   if (LeftArm.position(deg)>=-10){
      //   LeftArm.stop(hold);
      //   RightArm.stop(hold);
      //   }else{
      //   LeftArm.spin(fwd,100,pct);
      //   RightArm.spin(fwd,100,pct);
      //   }

      // }else{
      //   LeftArm.stop(hold);
      //   RightArm.stop(hold);

      // }
      // if(Controller.ButtonL2.pressing()){
      //   LArm.spin(fwd,70,pct);
      //   RArm.spin(fwd,70,pct);

      //   a = false;

      // }else if (Controller.ButtonL1.pressing()){
      //   LArm.spin(reverse,60,pct);
      //   RArm.spin(reverse,60,pct);
      //   a = false;
      // }
      // else{
      //   LArm.spin(reverse,0.5,pct);
      //   RArm.spin(reverse,0.5,pct);
      // }
      // if(Controller.ButtonX.pressing()){
      //   LArm.spin(fwd,100,pct);
      //   RArm.spin(fwd,100,pct);

      // }else if (Controller.ButtonY.pressing()){
      //   LArm.spin(reverse,100,pct);
      //   RArm.spin(reverse,100,pct);

      // }else{
      //   LArm.stop(hold);
      //   RArm.stop(hold);

      // }
    if(height == true){
      Controller.Screen.setCursor(5, 0);
      Controller.Screen.print("Height: True");
    }
    else{
      Controller.Screen.setCursor(5, 0);
      Controller.Screen.print("Height: False");
    }

    if(Controller.ButtonUp.pressing()){
      position();
    }
    if(Controller.ButtonRight.pressing()){
      position2();
    }
    if(Controller.ButtonDown.pressing()&&!Controller.ButtonB.pressing()){
      position4();
    }


    
    Brain.Screen.printAt(25,100,"x:%.2f",(x));
    Brain.Screen.printAt(25,125,"y:%.2f",(y));
    Brain.Screen.printAt(25,150,"Gyro:%.2f",(Gyro.rotation(deg)));
    Brain.Screen.printAt(25,175,"Ox:%.2f",(odomX.position(rev)));
    Brain.Screen.printAt(25,175,"position:%.2f",(counter));

      liftControl();
      Drive_UserControl();
      Intake_UserControl();


  }
}

int main()
{
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
}