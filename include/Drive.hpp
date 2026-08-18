#include <iostream>
#include <cmath>
#include "vex.h"
extern vex::competition Competition;
extern vex::brain Brain;
extern vex::controller Controller;
extern vex::motor Intake;
extern vex::motor LeftArm;
extern vex::motor RightArm;
extern vex::motor LArm;
extern vex::motor RArm;

extern vex::motor LeftFront;
extern vex::motor LeftMiddle;
extern vex::motor LeftBack;
extern vex::motor RightFront;
extern vex::motor RightMiddle;
extern vex::motor RightBack;
extern vex::distance liftSensor; 
extern vex::distance clawSensor;
extern vex::rotation Arm;

extern vex::rotation odom;
extern vex::pneumatics clamp;
extern vex::pneumatics doinkerR;
extern vex::pneumatics RotateUP;
extern vex::pneumatics Rotatedown;
extern vex::inertial Gyro;
extern vex::optical OpticalSensor;
extern vex::aivision AIVision1;
extern vex::motor Outake;
extern float x;
extern float y;
void Drive_Autonomous(int LeftSpeed, int RightSpeed, int WaitTime);
void drive(double lspeed, double rspeed, int wt);
void DriveBrake();
void Drivecoast();
void Drive_Autonomous_Volt(int LeftSpeed, int RightSpeed, int WaitTime);
void inchDriveC(float target, float timeLimit, float mspeed, float chainspeed = 40,double target2 = Gyro.rotation(), double target3 = Gyro.rotation(), int c = 0);
void inchDriveC3(float target, float timeLimit, float mspeed,bool chained = false, double target2 = Gyro.rotation());
void inchDriveC2(float target, float timeLimit,double mspeed = 100, double target2 = Gyro.rotation(), double distbetweentarget=0, double target3 = Gyro.rotation(),double distbetweentarget2=0,double target4 = Gyro.rotation(), int c = 0);
void inchDriveO(float target, float timeLimit, float mspeed, float chainspeed = 40,double target2 = Gyro.rotation(), double target3 = Gyro.rotation(), int c = 0);
void arcturn(float target, float arcdegree, float timeLimit, int b = 1.5, int c = 0);
void icc_tracking();
int sign(float a);
void MTP (float Tx, float Ty, double timeLimit);
void TTP (float Tx, float Ty, double timeLimit,double flip =0);

void MTPB (float Tx, float Ty, double timeLimit, double Tspeed =100, double Mspeed = 0, double acuracy = 2.0);

void Align();
void gyroTurnF(float target, double mspeed =1, double accuracy = 0.3, float b = 2.4);
void gyropivotR(float target, bool direction, double accuracy = 0.5,  int timeLimit = 1200, float b = 2.4);
void gyropivotL(float target, bool direction, double accuracy = 0.5,  int timeLimit = 1200, float b = 2.4);
void gyropivotRC(float target, bool direction, double accuracy = 0.5,  int timeLimit = 1200, float b = 2.4);
void gyropivotLC(float target, bool direction, double accuracy = 0.5,  int timeLimit = 1200, float b = 2.4);
void arcturnLH(int r, float arcdeg, int timeLimit, float max_drift = 0.15);
void arcturn3(int r, float targetA, double timeLimit, bool Left = true);
void arcturnL(int r, float arcdeg, int timeLimit, float max_drift = 0.15);
void arcturnR(float r, float arcdeg, int timeLimit, float max_drift = 0.1);
void arcturnLC(float r, float arcdeg, int timeLimit, float max_drift = 0.15);
void arcturnRC(float r, float arcdeg, int timeLimit, float max_drift = 0.1);


