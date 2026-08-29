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
int AutonomousMode =8;
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
void LIFTWork(){

  clamp.set(!clamp.value());

  }
void position(){
  
}

void AntlerDEscore(){

    Rotatedown.set(!Rotatedown.value());
}
double targetL = 2;
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
    low,
    low2,
    middle,
    middle2,
    high,
    high2,
};
double Akp = 0.09;
double kp = 0.3;
double kd = 0.0;
state currentState = idle;

void toggleState() {
    switch(currentState){
        case idle:
            currentState = loading;
            LeftArm.setVelocity(20,pct);
            RightArm.setVelocity(20,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Rotatedown.close();
            Akp = 0.05;
            
            kp = 0.1;
            targetA = 10;
            targetL = 0;


            break;

        case loading:
            currentState = loading3;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.05;
            kp = 0.3;

            targetL = 80;
            targetA = 280;
            wait(100,msec);
            Rotatedown.open();

            break; // old 24
          case loading2:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.09;
            targetL = 0;
            targetA = 290;
            wait(250,msec);
            clamp.close();

            break;
            case loading3:
            currentState = loading5;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.05;
            targetL = 400;
            kp = 0.3;


            targetA = 260;
            wait(100,msec);
            Rotatedown.open();

            break; 
            case loading4:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.05;
            targetL = 250;

            wait(250,msec);
            clamp.close();

            break; 
            case loading5:
            currentState = low;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.05;
            targetL = 490;
            kp = 0.8;

            targetA = 250;
            wait(100,msec);
            Rotatedown.open();

            break; 
            case loading6:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.05;

            targetA = 290;


            wait(250,msec);
            clamp.close();

            break;             
        case low:
            currentState = middle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 185;
            Akp = 0.07;
            kp = 0.2;

            Rotatedown.close();
          targetL = 0;
          targetA = 160;
            break;
        case low2:
            currentState = middle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 185;
            Akp = 0.09;

          targetA = 185;
            wait(250,msec);
            clamp.close();
            break;
        case middle:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            Akp = 0.09;
            kp = 0.3;

            targetA = 160;
            Rotatedown.close();



            break;
        case middle2:
            currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 100;
            Akp = 0.09;
            kp = 0.1;

            targetA = 185;
            wait(250,msec);
            clamp.close();


            break;
          case high:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.09;
            Rotatedown.close();
            kp = 0.5;

            targetA = 160;

            break;
          case high2:
          currentState = idle;
            LeftArm.setVelocity(100,pct);
            RightArm.setVelocity(100,pct);
            Intake.setVelocity(100,pct);
            cap = 65;
            Akp = 0.09;

            targetA = 185;
            wait(100,msec);
            Rotatedown.open();
            break;
    }
}
void liftControl() {

    double kg = 0;
    double lasterror = 0;
    double x = (LeftArm.position(deg)+RightArm.position(deg)/2);
    double error = targetL - x;
    double speed = error * kp+kd*(error-lasterror)+ kg;
    double Akd = 0;
    double Akg = 0.0;
    double Alasterror = 0;
    double Ax = (Arm.position(deg));
    double Aerror = targetA - Ax;
    double Aspeed = Aerror * Akp+Akd*(Aerror-Alasterror)+ Akg;
      Brain.Screen.printAt(25,200,"Aerror:2%.2f",(Aerror));
      Brain.Screen.printAt(25,225,"Ax:%.2f",(Ax));



    
  


    lasterror = error;
    if(speed >= cap){
      speed = cap;
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
  if(currentState == loading3){
  currentState = loading2;
  toggleState();
  } else if(currentState == loading5){
  currentState = loading4;
  toggleState();
  } else if(currentState == low){
  currentState = loading6;
  toggleState();
  } else if(currentState == middle){
  currentState = low2;
  toggleState();
  } else if(currentState == high){
  currentState = middle2;
  toggleState();
  } else{
  currentState = high2;
  toggleState();
  }
}
void ToggleRight(){
  currentState = high;
  toggleState();
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
  if (isClosed)
  {
    Controller.rumble(".");
    doinkerR.open();
    isClosed = !isClosed;
  }
  else
  {
    Controller.rumble(".");
    doinkerR.close();
 isClosed = !isClosed;
  }
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
float position = 0;



void test (){
}
void autonomous(void)
{
  float START_ANGLE;
  float HEADING;
  vex::thread(intakeControl);
  vex::thread odom (icc_tracking);



  switch (AutonomousMode)
  { 
    case 0:
    Gyro.setRotation(0,deg);
    Intake.setVelocity(100,pct);
    Outake.setVelocity(100,pct);
    intakeState = 1;
    inchDriveC2(49,2900,45,0,18,-90,19,-90);
    inchDriveC(20,1250,0.6,40);

    
    gyroTurnF(-90);
    inchDriveC(-36.5,1600,0.6,40);
    intakeState =1;
    DriveBrake();
    intakeState = 1;
    Outake_state = 1;
    wait(2000,msec);
    Outake_state = 0;
    gyroTurnF(170);
    inchDriveC(18,1200,0.6,30);
    gyroTurnF(-45);
    inchDriveC(-18,1100,0.6,30);
    clamp.open();
    wait(2000,msec);
    inchDriveO(37,1200,1,60);
    gyroTurnF(-90);
    inchDriveC(-17,1500,0.8,50);
    DriveBrake();










    



    
    //launch loader
    


    







    DriveBrake();
    // gyropivotRC(45,true);
    // DriveBrake();

    break;
    case 1:
    Intake.setVelocity(100,pct);
    Outake.setVelocity(100,pct);
    intakeState = 1;
    inchDriveC2(37,2100,45,0,14,45,30,45);
    DriveBrake();

    wait(200,msec);
    gyroTurnF(-80);
    inchDriveO(-28,700,1,60);
    gyropivotL(180,false,0.3,1000);
    // wait(200,msec);
    // inchDriveC(-15,900,1,40);
    // gyroTurnF(180);
    intakeState = 1;
    Outake_state = 1;
    wait(2000,msec);
    Outake_state = 0;
    gyroTurnF(180);

    inchDriveC(30,1200,0.6,50);
    DriveBrake();
    wait(1200,msec);
    inchDriveC(-10,1400,1,60);
    gyroTurnF (-45);
    inchDriveC(14,800,1,60);
    gyropivotLC(0,true);
    inchDriveC(20,1000,1,60);
    gyroTurnF(-45);



    DriveBrake();
    break;
    case 2:
    Intake.setVelocity(100,pct);
    clamp.open();

    intakeState = 1;
    inchDriveC2(48,2900,45,0,17,90,19,90);
    gyroTurnF(90);
    inchDriveC(-30,1200,0.6,40);
    Outake_state = 1;

    wait(900,msec);
    inchDriveC(5.3,800,1,0.6);
    gyropivotLC(-133.5,true,0.3,800);
    inchDriveC(33.2,2000,0.6,40);
    Outake_state = 0;
    inchDriveC(-8,1200,0.6,40);
    gyropivotL(180,false,0.5,600);
    clamp.close();
    inchDriveC(49,1850,0.7,50);
    clamp.open();
    gyroTurnF(135);
    intakeState = 0;
    inchDriveC(-19,1500,0.8,40);
    intakeState = 1;
    clamp.close();

    DriveBrake();





    break;
    case 3:
    Intake.setVelocity(100,pct);

    intakeState = 1;
    inchDriveC2(49,2900,45,0,19,90,20,90);
    gyroTurnF(90,1,0.7);
    inchDriveC(-5,100,1,60);
    inchDriveC(7,250,1,60);
    DriveBrake();
    wait(1400,msec);
    inchDriveC(-14,1200,0.8,40);
  
    gyroTurnF(-45);
    inchDriveO(28,900,0.8,40);
    gyroTurnF(-89);
    inchDriveO(65,3500,0.8,40,-90);
    gyroTurnF(180);
    inchDriveO(14,1000,0.4,40);
    gyroTurnF(-90);
    inchDriveC(-20,900,0.7,40);
    DriveBrake();
    Outake_state = 1;
    wait(3000,msec);
    Outake_state = 0;
    gyroTurnF(-90,1,0.7);

    inchDriveC(47,1000,0.4,30);
    wait(1500,msec);
    gyroTurnF(-90);
    inchDriveC(-35,1500,0.6,40);
    DriveBrake();
    Outake_state = 1;
    wait(3000,msec);
    Outake_state = 0;
    inchDriveO(10,1500,0.8,40);
    gyroTurnF(180);

    inchDriveO(95,4000,0.8,70);
    gyroTurnF(-90);
    inchDriveC(32,1800,0.5,30);
    inchDriveC(8,250,1,60);
    inchDriveC(-5,100,1,60);
    inchDriveC(8,250,1,60);
    wait(700,msec);
    gyroTurnF(-90);
    inchDriveC(-10,2100,0.7,40);
    gyroTurnF(135);
    inchDriveC(23,1000,0.6,40);
    gyroTurnF(91);

    inchDriveC(82,3500,0.8,30);
    gyroTurnF(0);
    inchDriveC(10 ,2000,0.7,40);
    gyroTurnF(90);
    inchDriveC(-28,1200,1,40);
    DriveBrake();
    Outake_state = 1;
    wait(2100,msec);
    gyroTurnF(90);
    Outake_state = 0;
    inchDriveC(38,1000,0.5,40);
    wait(1500,msec);
    gyroTurnF(90);
    inchDriveC(-35,1500,0.6,40);
    Outake_state = 1;
    wait(2000,msec);
    Outake_state = 0;
    inchDriveO(13,1200,1,60);

    gyroTurnF(45);
    inchDriveC(26,1000,1,60);
    gyroTurnF(10);
    inchDriveC(45,2200,0.7,65);
    DriveBrake();






    break;
    case 4:
    LeftArm.resetPosition();
    RightArm.resetPosition();
    Intake.setVelocity(30,pct);
    Gyro.setRotation(0,deg);
  
    x=0;
    y=0;
    doinkerR.open();
    Rotatedown.open();
    RotateUP.close();


    inchDriveC3(5,450,0.6);
    DriveBrake();
    intakeState = -1;
    wait(500,msec);
    doinkerR.close();



    inchDriveC3(-10,750,1);
    clamp.open();
    wait(500,msec);
    targetA= 80;
    currentState = low;
    toggleState();
    gyroTurnF(90);

    MTP(12, 4, 2000);
    MTPB(0,12,1200,60);
    targetA= 28;

    wait(700,msec);
    clamp.close();

    doinkerR.open();
    currentState = idle;
    toggleState();
    targetA= 5;

    wait(300,msec);



    gyroTurnF(150);

    inchDriveC3(20,700,0.8);
    inchDriveC3(-5,250,1);

    inchDriveC3(10,600,1);

    inchDriveC3(-10,1200,1,true);

    MTPB(3, 35.5, 2000,40);
    clamp.open();




    

    break;
    case 5:
    Intake.setVelocity(30,pct);
    Gyro.setRotation(0,deg);
    x=0;
    y=0;
    doinkerR.open();
    Rotatedown.open();
    RotateUP.close();


    inchDriveC3(5,450,0.6);
    DriveBrake();
    intakeState = -1;
    wait(500,msec);
    doinkerR.close();



    inchDriveC3(-5,250,1);
    LIFTWork();
    wait(200,msec);
    targetA = 70;
    gyroTurnF(45);

    MTP(23, 7, 2000);
    doinkerR.open();

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
    Intake.setVelocity(100,pct);
    Gyro.setRotation(0,deg);
    x=0;
    y=0;

    inchDriveC2(-10,1800,80,0,1,-85,2,1);
    gyropivotR(0,false,0.5,1200);
    // gyropivotL(0,true,0.5,1200);
    // inchDriveC3(-9,500,1);

    



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
    MTP(24,24,1500);
  }
} 
void pre_auton(void)
{
  Gyro.calibrate();
  Gyro.setRotation(0,deg);
  drawGUI();
  Brain.Screen.pressed(selectAuton);
  vex::thread intakeThread(intakeControl);



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
  Controller.Screen.setCursor(5, 0);
  Controller.Screen.print("MGO:OPEN");
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
    Controller.ButtonB.pressed(LIFTWork);

    Controller.ButtonX.pressed(AntlerDEscore);
    Controller.ButtonL2.pressed(doubleToggle); 
    Controller.ButtonL1.pressed(toggleState); 
    Controller.ButtonY.pressed(doinkerRtoggle); 
  
    // Controller.ButtonDown.pressed(Align);  
    LeftArm.setPosition(0,deg);
   Aa = true;
    

  
    while (true)
    {
      if(!Gyro.isCalibrating()){
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
    
    Brain.Screen.printAt(25,100,"x:%.2f",(x));
    Brain.Screen.printAt(25,125,"y:%.2f",(y));
    Brain.Screen.printAt(25,150,"Gyro:%.2f",(Gyro.rotation(deg)));
    Brain.Screen.printAt(25,175,"Ox:%.2f",(odomX.position(rev)));



      liftControl();
      Drive_UserControl();
      Intake_UserControl();


      }
  }
}

int main()
{
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
}