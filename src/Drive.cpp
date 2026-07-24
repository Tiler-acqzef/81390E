#include <iostream>
#include <cmath>
#include "vex.h"
#include "Drive.hpp"
using namespace vex;

competition Competition;
brain Brain;
controller Controller;
motor Intake = motor(PORT15, ratio6_1, true);
motor LeftArm     = motor(PORT20, ratio18_1, false);
motor RightArm    = motor(PORT11, ratio18_1, true);
motor LArm = motor(PORT5,ratio18_1,false);
motor RArm = motor(PORT4,ratio18_1,true);

// motor LeftFront   = motor(PORT2, ratio6_1, true);
// motor LeftMiddle  = motor(PORT3, ratio6_1, false);
// motor LeftBack    = motor(PORT1, ratio6_1, true);
// motor RightFront  = motor(PORT5, ratio6_1, false);
// motor RightMiddle = motor(PORT6, ratio6_1, true);
// motor RightBack   = motor(PORT4, ratio6_1, false); bajBRWuab meow

motor LeftFront = motor(PORT19, ratio6_1, true); //11
motor LeftMiddle = motor(PORT18, ratio18_1, true); //12
motor LeftBack = motor(PORT17, ratio6_1, true); //13
motor Outake = motor(PORT21,ratio6_1,false);
motor RightFront = motor(PORT12, ratio6_1, false); //1
motor RightMiddle = motor(PORT13, ratio18_1, false); //4
motor RightBack = motor(PORT14, ratio6_1, false); //14
distance liftSensor = (PORT21);
distance clawSensor = (PORT21);

rotation odom = rotation(PORT11); 
pneumatics clamp(Brain.ThreeWirePort.H);
pneumatics doinkerR(Brain.ThreeWirePort.E);
pneumatics RotateUP (Brain.ThreeWirePort.G);
pneumatics Rotatedown (Brain.ThreeWirePort.F);
inertial Gyro = inertial(PORT9);
optical OpticalSensor = optical(PORT21);
vex::aivision AIVision1(PORT21, aivision::ALL_AIOBJS);
gps GPS = gps(PORT21);
float pi = 3.14159;
float dia = 3.25;
float diaO = 2;
float gearRatio = 0.75;
float width = 14;
float target2 = 0;
float x = 0;
float y = 0;

void Drive_Autonomous(int LeftSpeed, int RightSpeed, int WaitTime)
{
  LeftFront.spin(fwd, LeftSpeed, pct);
  LeftMiddle.spin(fwd, LeftSpeed, pct);
  LeftBack.spin(fwd, LeftSpeed, pct);
  RightFront.spin(fwd, RightSpeed, pct);
  RightMiddle.spin(fwd, RightSpeed, pct);
  RightBack.spin(fwd, RightSpeed, pct);
  wait(WaitTime, msec);
}
void drive(double lspeed, double rspeed, int wt)
{
  lspeed *= 0.12;
  rspeed *= 0.12;

  LeftFront.spin(fwd, lspeed, volt);
  LeftMiddle.spin(fwd, lspeed, volt);
  LeftBack.spin(fwd, lspeed, volt);
  RightFront.spin(fwd, rspeed,volt);
  RightMiddle.spin(fwd, rspeed,volt);
  RightBack.spin(fwd, rspeed, volt);
  wait(wt, msec);
}
void DriveBrake()
{
  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightMiddle.stop(brake);
  RightBack.stop(brake);
}


void Drivecoast()
{
  LeftFront.stop(coast);
  LeftMiddle.stop(coast);
  LeftBack.stop(coast);
  RightFront.stop(coast);
  RightMiddle.stop(coast);
  RightBack.stop(coast);  
}


void Drive_Autonomous_Volt(int LeftSpeed, int RightSpeed, int WaitTime)
{
  LeftFront.spin(fwd, LeftSpeed / 100 * 120, volt);
  LeftMiddle.spin(fwd, LeftSpeed / 100 * 120, volt);
  LeftBack.spin(fwd, LeftSpeed / 100 * 120, volt);
  RightFront.spin(fwd, RightSpeed / 100 * 120, volt);
  RightMiddle.spin(fwd, RightSpeed / 100 * 120, volt);
  RightBack.spin(fwd, RightSpeed / 100 * 120, volt);
  wait(WaitTime, msec);
}




void inchDriveC(float target, float timeLimit, float mspeed, float chainspeed, double target2 , double target3 , int c ){
  float heading = 0;
  float angle_error = 0;
  float angle_last_error = 0;
  float angleP = 2;
  float turn_speed = 0;
  float angleD = 0.2;
  float aacuracy = 0.5; 
  RightFront.setPosition(0, rev);
  RightBack.setPosition (0,rev);
  float x = 0;
  float error2 = target;
  float error = target;
  float kp = 7;
  float speed = kp * error;
  float accuracy = 0.05; // was 0.05
  float kd = 0.4;
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
  vex::timer timer; // Create a timer object
  timer.clear();
  while (fabs(error) >= accuracy)
  {
    heading = Gyro.rotation();
    if (target2 > 180)
    {
      target - 360;
    }
    else if (target2 < -180)
    {
      target + 360;
    }

    x = ((RightFront.position(rev)+RightBack.position(rev))/2) * pi * dia * gearRatio;
    //  std::cout << x << "\n"; // stope the drive
    error = target - x;
    angle_error = target2 - heading;
    speed = kp * error + kd * (error - last_error) / dt;
    turn_speed = angleP * angle_error + angleD * (angle_error - angle_last_error) / dt;
    if (speed >= 100)
    {
      speed = 100;
    }
    else if (speed <= -100)
    {
      speed = -100;
    }
    if(mspeed >=0&&mspeed <= chainspeed){
    if (fabs(speed) < chainspeed )
    {
      if (speed > 0)
      {
        speed = chainspeed;
      }
      else
      {
        speed = (-1*chainspeed);
      }
    }
    }

    drive(mspeed*(speed+turn_speed), mspeed*(speed-turn_speed), 10);
    last_error = error;
    angle_last_error = angle_error;
    if(fabs(angle_error)<= aacuracy){
      target2 = target3;
    }
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
    std::cout << x << std::endl;
  }
}
void inchDriveC3(float target, float timeLimit, float mspeed,bool chained, double target2)
{
  float heading = 0;
  float angle_error = 0;
  float angle_last_error = 0;
  float angleP = 2;
  float turn_speed = 0;
  float angleD = 0.2;
  float aacuracy = 0.5; 
  RightFront.setPosition(0, rev);
  RightBack.setPosition (0,rev);
  float x = 0;
  float error2 = target;
  float error = target;
  float kp = 7;
  float speed = kp * error;
  float accuracy = 0.05; // was 0.05
  float kd = 0.4;
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
  double Ftarget = 0;
  vex::timer timer; // Create a timer object
  timer.clear();
  if(chained = false){
  while (fabs(error) >= accuracy)
  {
    heading = Gyro.rotation();
    if (target2 > 180)
    {
      target - 360;
    }
    else if (target2 < -180)
    {
      target + 360;
    }

    x = ((RightFront.position(rev)+RightBack.position(rev))/2) * pi * dia * gearRatio;
    //  std::cout << x << "\n"; // stope the drive
    error = target - x;
    angle_error = target2 - heading;
    speed = kp * error + kd * (error - last_error) / dt;
    turn_speed = angleP * angle_error + angleD * (angle_error - angle_last_error) / dt;
    if (speed >= 100)
    {
      speed = 100;
    }
    else if (speed <= -100)
    {
      speed = -100;
    }


    drive(mspeed*(speed+turn_speed), mspeed*(speed-turn_speed), 1);
    last_error = error;
    angle_last_error = angle_error;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
    std::cout << x << std::endl;
  }
  DriveBrake();
}else{
    while (fabs(error) >= accuracy){

      heading = Gyro.rotation();
      if (target2 > 180)
      {
        target - 360;
      }
      else if (target2 < -180)
      {
        target + 360;
      }
      if (target >0){
        Ftarget = target + 2;
      }
      else if (target <0){
        Ftarget = target -2;
      }
      x = ((RightFront.position(rev)+RightBack.position(rev))/2) * pi * dia * gearRatio;
      //  std::cout << x << "\n"; // stope the drive
      error = Ftarget - x;
      angle_error = target2 - heading;
      speed = kp * error + kd * (error - last_error) / dt;
      turn_speed = angleP * angle_error + angleD * (angle_error - angle_last_error) / dt;
      if (speed >= 100)
      {
        speed = 100;
      }
      else if (speed <= -100)
      {
        speed = -100;
      }


      drive(mspeed*(speed+turn_speed), mspeed*(speed-turn_speed), 1);
      last_error = error;
      angle_last_error = angle_error;

      if (timer.time(vex::timeUnits::msec) >= timeLimit)
      {
        break;
      }
      std::cout << x << std::endl;
  }
}
}
void inchDriveC2(float target, float timeLimit,double mspeed , double target2, double distbetweentarget, double target3 ,double distbetweentarget2,double target4 , int c )
{
  float heading = 0;
  float angle_error = target2;
  float angle_last_error = 0;
  float angleP = 2;
  float turn_speed = 0;
  float angleD = 0.2;
  float aacuracy = 0.7; 
  RightFront.setPosition(0, rev);
  float x = 0;
  float error2 = target;
  float error = target;
  float kp = 7;
  float speed = kp * error;
  float accuracy = 0.05; // was 0.05
  float kd = 0.4;
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
  vex::timer timer; // Create a timer object
  timer.clear();
  bool s = false;
  int count;

  while (fabs(error) >= accuracy)
  {
    heading = Gyro.rotation();
    if (target2 > 180)
    {
      target2 - 360;
    }
    else if (target2 < -180)
    {
      target2 + 360;
    }
    x = RightFront.position(rev) * pi * dia * gearRatio;
    error = target - x;
    if(fabs(error<=distbetweentarget)&&s==false){
      target2 = target3;

      s=true;
    }
    if(fabs(angle_error)<=aacuracy&&s==true&&fabs(error)<=distbetweentarget2){
      target2 = target4;
      s=true;
    }


    angle_error = target2 - heading;
    speed = kp * error + kd * (error - last_error) / dt;
    turn_speed = angleP * angle_error + angleD * (angle_error - angle_last_error) / dt;
    if(fabs(mspeed) <= fabs(speed)){
      if(speed >= 0){
        speed = mspeed;
      }
      else if(speed <=0){
        speed = -mspeed;
      }
      else{
        speed = speed;
      }
    }
    if (speed >= 100)
    {
      speed = 100;
    }
    else if (speed <= -100)
    {
      speed = -100;
    }


    drive((speed+turn_speed),(speed-turn_speed), 10);
    last_error = error;
    angle_last_error = angle_error;
    std::cout << count <<", " << error <<", " << speed + turn_speed<<", " << speed - turn_speed <<", " << heading << std::endl;


    count++;
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }
}
void inchDriveO(float target, float timeLimit, float mspeed, float chainspeed ,double target2 , double target3 , int c )
{
  float heading = 0;
  float angle_error = 0;
  float angle_last_error = 0;
  float angleP = 2;
  float turn_speed = 0;
  float angleD = 0.4;
  float aacuracy = 0.5; 
  double startposition = odom.position(rev);
  float x = 0;
  float error2 = target;
  float error = target;
  float kp = 7;
  float speed = kp * error;
  float accuracy = 0.05; // was 0.05
  float kd = 1.2;
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
  vex::timer timer; // Create a timer object
  timer.clear();

  while (fabs(error) >= accuracy)
  {
    heading = Gyro.rotation();
    if (target2 > 180)
    {
      target - 360;
    }
    else if (target2 < -180)
    {
      target + 360;
    }
    x = (odom.position(rev)- startposition) * diaO*pi;
    //  std::cout << x << "\n"; // stope the drive
    error = target - x;
    angle_error = target2 - heading;
    speed = kp * error + kd * (error - last_error) / dt;
    turn_speed = angleP * angle_error + angleD * (angle_error - angle_last_error) / dt;
    if (speed >= 100)
    {
      speed = 100;
    }
    else if (speed <= -100)
    {
      speed = -100;
    }
    if(mspeed >=0&&mspeed <= chainspeed){
    if (fabs(speed) < chainspeed )
    {
      if (speed > 0)
      {
        speed = chainspeed;
      }
      else
      {
        speed = (-1*chainspeed);
      }
    }
    }

    drive(mspeed*(speed+turn_speed), mspeed*(speed-turn_speed), 10);
    last_error = error;
    angle_last_error = angle_error;
    if(fabs(angle_error)<= aacuracy){
      target2 = target3;
    }
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
    std::cout << x << std::endl;
  }
}
void arcturn(float target, float arcdegree, float timeLimit, int b , int c )
{
  float heading = 0;
  float angle_error = 0;
  float angle_last_error = 0;
  float angleP = 1.2;
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  LeftMiddle.setPosition(0, rev);
  float x = 0;
  float error = target;
  float kp = 6;
  float speed = kp * error;
  float accuracy = 0.1; // was 0.05
  float kd = 0.5;
  double last_error = 0;
  double dt = 0.01;
  vex::timer timer; // Create a timer object
  timer.clear();
  while (fabs(error) > accuracy or fabs(angle_error) > angle_accuracy)
  {
    heading = Gyro.rotation();
    x = LeftMiddle.position(rev) * pi * dia * gearRatio;
    if (angle_error > 180)
    {
      error = error - 360;
    }
    else if (angle_error < -180)
    {
      error = error + 360;
    }
    error = target - x;
    angle_error = arcdegree - heading;
    speed = kp * error + kd * (error - last_error) / dt + b * error / fabs(error);
    turn_speed = angleP * angle_error + c * angle_error / fabs(angle_error);
    if (speed >= 100)
    {
      speed = 100;
    }
    else if (speed <= -100)
    {
      speed = -100;
    }

    drive(speed + turn_speed, speed - turn_speed, 10);
    last_error = error;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }
  DriveBrake();
}
void icc_tracking() {

    float prevHeading = Gyro.rotation() * pi / 180.0;
    float prevLrev = 0;
    float prevRrev = 0;
    float DRight =0;
    float Dleft = 0;



    const float dt = 0.01;

    while(true){

        float heading = Gyro.rotation() * pi / 180.0;

        Dleft = -LeftFront.position(rev) - prevLrev;
        DRight = (-RightFront.position(rev) - prevRrev);
        float distance = (((Dleft+DRight)/2.0)*gearRatio) *dia*pi;

        float dTheta = heading - prevHeading;


        if(fabs(dTheta) != 0.0){

            float R = distance / dTheta;

            x += R * (cos(heading)-cos(prevHeading));
            y += R * (sin(prevHeading)-sin(heading));

        }
        else{

            x += distance * sin(heading);
            y += distance * cos(heading);

        }

        prevHeading = heading;
        prevRrev = -RightFront.position(rev);
        prevLrev = -LeftFront.position(rev);

        wait(dt, seconds);
    }
}
int sign(float a){
    if(a>0){
       return 1;
    }else if(a<0)
    {
      return  -1;
    }
    else{
      return 0;
    }
}
void MTP (float Tx, float Ty, double timeLimit){
    float LKP = 0.0;
    float LKD = 0.0;
    float LKI = 0;
    float AKP = 1.0;
    float AKD = 0.0;
    float AKI = 0;
    float speed =0;
    float ASpeed =0;
    float errorX = Tx-x;
    float ErrorY = Ty - y;
    float hypot = 0;
    double last_hypot = 0;
    float heading = Gyro.rotation();  
    float Aerror = 0;
    float Last_Aerror = 0;
    timer timer;
    int xsign = sign(errorX);
    int ysign = sign(ErrorY);


    while (true){
    heading = Gyro.rotation();
    errorX = Tx-x;
    ErrorY = Ty - y;
    hypot = sqrt(pow(errorX,2) + pow(ErrorY,2));
    bool overshot =
    sign(errorX) != xsign &&
    sign(ErrorY) != ysign;

      float directionFlip = 0.0;

      if (overshot) {
          hypot = -hypot;
          directionFlip = 180.0;
      }

      float targetHeading =
          atan2(errorX, ErrorY) * 180.0 / M_PI;

      Aerror = targetHeading - directionFlip - heading;
    if(hypot<=3){
      Aerror = 0;
    }
    if (Aerror > 180){
      Aerror -= 360;
    }
    else if (Aerror < -180){
      Aerror += 360;
    }

    // if(sign(errorX)!= xsign && sign(ErrorY)!= ysign ){
    //   Controller.Screen.print("done");
    //   hypot = -hypot;
    // }
    speed = hypot*LKP+LKD*(hypot-last_hypot);
    if(hypot <= 7){
      speed = speed * cos(Aerror*M_PI/180);
    } else{
      speed = speed* std::fmax(cos(Aerror*M_PI/180),0.0);
    }

    if (speed >= 50)
    {
      speed = 50;
    }
    else if (speed <= -50)
    {
      speed = -50;
    }

    ASpeed = Aerror*AKP + AKD*(Aerror-Last_Aerror);
    
    drive(speed+ASpeed,speed-ASpeed,0);

    Last_Aerror = Aerror;
    last_hypot = hypot;
    printf(
        "x: %.2f  y: %.2f  "
        "errorX: %.2f  errorY: %.2f  "
        "distance: %.2f  "
        "heading: %.2f  angleError: %.2f  "
        "speed: %.2f  turn: %.2f  "
        "left: %.2f  right: %.2f  "
        "overshot: %d\n",
        x,
        y,
        errorX,
        ErrorY,
        hypot,
        heading,
        Aerror,
        speed,
        ASpeed,
        speed + ASpeed,
        speed - ASpeed,
        overshot
    );
    if(hypot <=2.0){
      break;
    }
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }

      wait(10,msec);
    }
  Controller.Screen.print("done");
  DriveBrake();

}
void Align(){
    AIVision1.takeSnapshot(aivision::ALL_TAGS);
    double Tkp = 6; 
    double Tki = 0; 
    double Tkd = 0; 
    double kp =1;
    double ki =0;
    double kd =0;
    double last_error = 0;
    double Tlast_error = 0;
    double accuracy = 0.05;
    double realWidth = 5.5;
    double focalLength = 271.6;
    double pixelWidth = AIVision1.objects[0].centerX;
    double distance = (realWidth * focalLength) / pixelWidth;
    double error = 0-distance;
    Brain.Screen.print(AIVision1.objectCount);
  if (AIVision1.objectCount > 0) {

    pixelWidth = AIVision1.objects[0].centerX;
    aivision::object targetTag = AIVision1.objects[0];
    while(fabs(error)>=accuracy){

    distance = (realWidth * focalLength) / pixelWidth;
    error = 0-distance;
    double heading = Gyro.rotation();
    
    float Terror = targetTag.angle - heading;
    float speed = error*kp + kd * (error - last_error);
    float turnVelocity = Terror * Tkp ;
    drive(turnVelocity,-turnVelocity,1);

    // if (AIVision1.objects[0].width > 100) {
    //     DriveBrake();
    //     break; 
    // }
    std::cout  << targetTag.angle <<std::endl;
    std::cout  << turnVelocity<<std::endl;



  last_error = error;
  Tlast_error = Terror;
  }

}
}
void gyroTurnF(float target, double mspeed , double accuracy , float b )
{
  float heading = 0.0; // initialize a variable for heading
  double error = target - heading;
  double ki = 0.5;
  double intergal = 0;

  double kp = 5; // 7.85;//was 6
  double speed = 0;
  double kd = 40; // 0.65;//was 0.3
  double last_error = 0;
  double dt = 0.01; // reset Gyro to zero degrees
  int count = 0;
  vex::timer timer; // Create a timer object

  timer.clear(); // Clear any previous timer value
  int timeLimit = 1200;
  while (fabs(error) >= accuracy or count <= 5)
  {
    heading = Gyro.rotation(); // measure the heading of the robot
    std::cout << heading << "\n";
    error = target - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = error + 360;
    }

    if (fabs(error) < 15 && fabs(error) > 0.25)
    {
      intergal += error;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 50)
    {
      intergal = 50;
    }
    else if (intergal <= -50)
    {
      intergal = -50;
    }


    speed = kp * error + kd * (error - last_error)  + ki * intergal;

    drive(speed*mspeed, -speed*mspeed,1); // turn right at speed
    last_error = error;
    if (fabs(error) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }

  DriveBrake();
  std::cout << "newline" << "\n"; // stope the drive
}
void gyropivotR(float target, bool direction, double accuracy ,  int timeLimit, float b )  {
  float heading = Gyro.rotation(); // initialize a variable for heading
  double error = target - heading;
  double ki = 0.95;
  double intergal = 0;

  double kp = 7; // 7.85;//was 6
  double speed = 0;
  double kd = 0.45; // 0.65;//was 0.3
  double last_error = 0;
  double dt = 0.01; // reset Gyro to zero degrees
  int count = 0;
  vex::timer timer; // Create a timer object

  timer.clear(); // Clear any previous timer value
  while (fabs(error) >= accuracy )
  {
    heading = Gyro.rotation(); // measure the heading of the robot
    std::cout << heading << "\n";
    error = target - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = error + 360;
    }

    if (fabs(error) < 10 && fabs(error) > 1)
    {
      intergal += error;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 50)
    {
      intergal = 50;
    }
    else if (intergal <= -50)
    {
      intergal = -50;
    }

    // calculate error
    speed = kp * error + kd * (error - last_error) / dt + ki * intergal;
    if (direction == true){
      if (speed>=0){
        speed *=-1;
      }
    }
    else if (direction == false){
      if (speed<=0){
        speed *=-1;
      }
    }
    drive(0, -speed*0.5, dt * 1000); // turn right at speed
    last_error = error;
    if (fabs(error) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }

  DriveBrake();
  std::cout << "newline" << "\n"; // stope the drive
}
void gyropivotL(float target, bool direction, double accuracy ,  int timeLimit, float b )
{
  float heading = Gyro.rotation(); // initialize a variable for heading
  double error = target - heading;
  double ki = 0.95;
  double intergal = 0;

  double kp = 7; // 7.85;//was 6
  double speed = 0;
  double kd = 0.72; // 0.65;//was 0.3
  double last_error = 0;
  double dt = 0.01; // reset Gyro to zero degrees
  int count = 0;
  vex::timer timer; // Create a timer object

  timer.clear(); // Clear any previous timer value

  while (fabs(error) >= accuracy )
  {
    heading = Gyro.rotation(); // measure the heading of the robot
    std::cout << heading << "\n";
    error = target - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = error + 360;
    }

    if (fabs(error) < 10 && fabs(error) > 1)
    {
      intergal += error;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 50)
    {
      intergal = 50;
    }
    else if (intergal <= -50)
    {
      intergal = -50;
    }

    // calculate error
    speed = kp * error + kd * (error - last_error) / dt + ki * intergal;
    if (direction == true){
      if (speed<=0){
        speed *=-1;
      }
    }
    else if (direction == false){
      if (speed>=0){
        speed *=-1;
      }
    }
    drive(speed*0.5, 0, dt * 1000); // turn right at speed
    last_error = error;
    if (fabs(error) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }

  DriveBrake();
  std::cout << "newline" << "\n"; // stope the drive
}
void gyropivotRC(float target, bool direction, double accuracy ,  int timeLimit, float b)
{
  float heading = 0.0; // initialize a variable for heading
  double error = target;
  double ki = 0.95;
  double intergal = 0;

  double kp = 7; // 7.85;//was 6
  double speed = 0;
  double kd = 0.72; // 0.65;//was 0.3
  double last_error = 0;
  double dt = 0.01; // reset Gyro to zero degrees
  int count = 0;
  vex::timer timer; // Create a timer object

  timer.clear(); // Clear any previous timer value
  while (fabs(error) >= accuracy )
  {
    heading = Gyro.rotation(); // measure the heading of the robot
    std::cout << heading << "\n";
    error = target - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = error + 360;
    }

    if (fabs(error) < 10 && fabs(error) > 1)
    {
      intergal += error;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 50)
    {
      intergal = 50;
    }
    else if (intergal <= -50)
    {
      intergal = -50;
    }

    // calculate error
    speed = kp * error + kd * (error - last_error) / dt + ki * intergal;
    if (direction == true){
      if (speed>=0){
        speed *=-1;
      }
    }
    else if (direction == false){
      if (speed<=0){
        speed *=-1;
      }
    }
    drive(0, -speed*0.5, dt * 1000); // turn right at speed
    last_error = error;
    if (fabs(error) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }

  Drivecoast();
  std::cout << "newline" << "\n"; // stope the drive
}
void gyropivotLC(float target, bool direction, double accuracy ,  int timeLimit , float b )
{
  float heading = Gyro.rotation(); // initialize a variable for heading
  double error = target - heading;
  double ki = 0.95;
  double intergal = 0;

  double kp = 7; // 7.85;//was 6
  double speed = 0;
  double kd = 0.45; // 0.65;//was 0.3
  double last_error = 0;
  double dt = 0.01; // reset Gyro to zero degrees
  int count = 0;
  vex::timer timer; // Create a timer object

  timer.clear(); // Clear any previous timer value
  while (fabs(error) >= accuracy )
  {
    heading = Gyro.rotation(); // measure the heading of the robot
    std::cout << heading << "\n";
    error = target - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = error + 360;
    }

    if (fabs(error) < 10 && fabs(error) > 1)
    {
      intergal += error;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 50)
    {
      intergal = 50;
    }
    else if (intergal <= -50)
    {
      intergal = -50;
    }

    // calculate error
    speed = kp * error + kd * (error - last_error) / dt + ki * intergal;
    if (direction == true){
      if (speed<=0){
        speed *=-1;
      }
    }
    else if (direction == false){
      if (speed>=0){
        speed *=-1;
      }
    }
    drive(speed*0.4, 0, dt * 1000); // turn right at speed
    last_error = error;
    if (fabs(error) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }

  Drivecoast();
  std::cout << "newline" << "\n"; // stope the drive
}
void arcturnLH(int r, float arcdeg, int timeLimit, float max_drift )
{
float startHeading = Gyro.rotation();
float targetHeading = arcdeg;

float turnAmount = targetHeading - startHeading;
while (turnAmount < 0) turnAmount += 360;
while (turnAmount >= 360) turnAmount -= 360;

float heading = Gyro.rotation();
float angle_last_error = 0;

float angleP = 1.2;
float angleD = 0.05;

float turn_speed = 0;
float angle_accuracy = 0.3;
float robotwidth = 15;

LeftMiddle.setPosition(0, rev);
RightMiddle.setPosition(0, rev);

float Lx = 0;
float Rx = 0;
float settletime = 150;

double Ltarget = 2 * pi * r * fabs(turnAmount / 360.0);
double Rtarget = 2 * pi * (r + robotwidth) * fabs(turnAmount / 360.0);

float Lerror = Ltarget;
float Rerror = Rtarget;
float Lkp = 7;
float count = 0;
float Rkp = 7;
float accuracy = 0.1;
float Lkd = 0.35;
float Rkd = 0.35;
double Llast_error = 0;
double Rlast_error = 0;
double dt = 0.01;
double Lki = 0.5;
double Lintergal = 0;
double Rki = 0.5;
double Rintergal = 0;

float angle_target = startHeading;
float Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
float Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
float angle_error = angle_target - heading;

vex::timer timer;
vex::timer errortimer;
timer.clear();

while (true)
{
heading = Gyro.rotation();

Lx = LeftMiddle.position(rev) * pi * dia * gearRatio;
Rx = RightMiddle.position(rev) * pi * dia * gearRatio;

angle_target = startHeading + (Lx * 360) / (2 * pi * r);

angle_error = angle_target - heading;

if (angle_error > 180)
{
angle_error = angle_error - 360;
}
else if (angle_error < -180)
{
angle_error = angle_error + 360;
}

Lerror = Ltarget - Lx;
Rerror = Rtarget - Rx;

if (fabs(Lerror) < 10 && fabs(Lerror) > 1)
{
Lintergal += Lerror;
}
else
{
Lintergal = 0;
}if (Lintergal >= 40)
{
Lintergal = 40;
}
else if (Lintergal <= -40)
{
Lintergal = -40;
}

if (fabs(Rerror) < 10 && fabs(Rerror) > 1)
{
Rintergal += Rerror;
}
else
{
Rintergal = 0;
}

if (Rintergal >= 40)
{
Rintergal = 40;
}
else if (Rintergal <= -40)
{
Rintergal = -40;
}

Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;

Brain.Screen.printAt(1, 20, "Lspeed = %.2f Rspeed = %.2f ", Lspeed, Rspeed);

turn_speed = angleP * angle_error + angleD * (angle_error - angle_last_error) / dt;

float maxTurn = fabs(Lspeed) * max_drift;

if (turn_speed > maxTurn)
{
turn_speed = maxTurn;
}
else if (turn_speed < -maxTurn)
{
turn_speed = -maxTurn;
}

if (Lspeed >= 100)
{
Lspeed = 100;
}
else if (Lspeed <= -100)
{
Lspeed = -100;
}

if (Rspeed >= 100)
{
Rspeed = 100;
}
else if (Rspeed <= -100)
{
Rspeed = -100;
}

if (fabs(angle_error) > angle_accuracy)
{
errortimer.clear();
}

if (fabs(Lerror) <= accuracy + 0.3 && fabs(Rerror) <= accuracy + 0.3)
{
count++;
}
else
{
count = 0;
}

drive(Lspeed - turn_speed, Rspeed + turn_speed, 10);

Llast_error = Lerror;
Rlast_error = Rerror;
angle_last_error = angle_error;

if (count >= 20 || timer.time(vex::timeUnits::msec) >= timeLimit)
{
break;
}

vex::wait(10, vex::timeUnits::msec);
}

DriveBrake();
}
void arcturn3(int r, float targetA, double timeLimit, bool Left ){
  float width = 14;
  float angle_accuracy = 0.3;
  float heading = Gyro.rotation();
  double ratio = fabs(r+width/2/(r-width/2));
  float error = targetA-heading;
  float kp = 7;
  float speed = kp * error;
  float accuracy = 0.05; // was 0.05
  float kd = 1.2;
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
   double ki = 0.5;
  double intergal = 0;
  double count =0;
    vex::timer timer;
    timer.clear();
  if(true){
  while((targetA - heading)<=angle_accuracy){

    heading = Gyro.rotation(); // measure the heading of the robot
    std::cout << heading << "\n";
    error = targetA - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = error + 360;
    }



    if (fabs(error) < 15 && fabs(error) > 0.25)
    {
      intergal += error;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 50)
    {
      intergal = 50;
    }
    else if (intergal <= -50)
    {
      intergal = -50;
    }


    speed = kp * error + kd * (error - last_error)  + ki * intergal;

    drive(speed*(1/ratio), speed*ratio,1); // turn right at speed
    last_error = error;
    if (fabs(error) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }
  }else{
  while((targetA - heading)<=angle_accuracy){
    heading = Gyro.rotation(); // measure the heading of the robot
    std::cout << heading << "\n";
    error = targetA - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = error + 360;
    }



    if (fabs(error) < 15 && fabs(error) > 0.25)
    {
      intergal += error;
    }
    else
    {
      intergal = 0;
    }

    if (intergal >= 50)
    {
      intergal = 50;
    }
    else if (intergal <= -50)
    {
      intergal = -50;
    }


    speed = kp * error + kd * (error - last_error)  + ki * intergal;

    drive(speed*(ratio), speed*(1/ratio),1); // turn right at speed
    last_error = error;
    if (fabs(error) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }

  DriveBrake();
}
}
void arcturnL(int r, float arcdeg, int timeLimit, float max_drift )
{
  float heading = Gyro.rotation();
  float Initheading = Gyro.rotation();

  float angle_last_error = 0;
  float angleP = 0;
  float angleD = 0;
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  float robotwidth = 14;
  LeftMiddle.setPosition(0, rev);
  RightMiddle.setPosition(0, rev);
  float Lx = 0;
  float Rx = 0;
  float settletime = 150;
  double Ltarget = 2 * pi * (r)*fabs(arcdeg / 360.0);
  double Rtarget = 2 * pi * (r + robotwidth) * fabs(arcdeg / 360.0);
  float Lerror = Ltarget;
  float Rerror = Rtarget;
  float Lkp = 7;
  float count = 0;
  float Rkp = 7;
  float accuracy = 0.1; // was 0.05
  float Lkd = 0.35;
  float Rkd = 0.35;
  double Llast_error = 0;
  double Rlast_error = 0;
  double dt = 0.01;
  double Lki = 0.5;
  double Lintergal = 0;
  double Rki = 0.5;
  double Rintergal = 0;
  float angle_target = (Rx * 360) / (2 * pi * r);
  float Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
  float Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
  float angle_error = angle_target;
  vex::timer timer;
  vex::timer errortimer; // Create a timer object
  timer.clear();
  while (count >= 2)
  {

    heading = Gyro.rotation();
    Lx = LeftMiddle.position(rev) * pi * dia * gearRatio;
    Rx = RightMiddle.position(rev) * pi * dia * gearRatio;
    angle_target = (Lx * 360) / (2 * pi * r);

    Lerror = Ltarget - Lx;
    Rerror = Rtarget - Rx;
    angle_error = angle_target - heading;

    if (angle_error > 180)
    {
      angle_error = angle_error - 360;
    }
    else if (angle_error < -180)
    {
      angle_error = angle_error + 360;
    }
    if (fabs(Lerror) < 10 && fabs(Lerror) > 1)
    {
      Lintergal += Lerror;
    }
    else
    {
      Lintergal = 0;
    }
    if (Lintergal >= 40)
    {
      Lintergal = 40;
    }
    else if (Lintergal <= -40)
    {
      Lintergal = -40;
    }
    if (fabs(Rerror) < 10 && fabs(Rerror) > 1)
    {
      Rintergal += Rerror;
    }
    else
    {
      Rintergal = 0;
    }
    if (Rintergal >= 40)
    {
      Rintergal = 40;
    }

    else if (Rintergal <= -40)
    {
      Rintergal = -40;
    }
    Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
    Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
    Brain.Screen.printAt(1, 20, "Lspeed = %.2f     Rspeed = %.2f  ", Lspeed, Rspeed);
    turn_speed = angleP * angle_error + angleD * (angle_error - angle_last_error) / dt;
    // turn_speed = std::min(std::fabs(turn_speed), std::fabs(Lspeed) * max_drift);

    if (Lspeed >= 100)
    {
      Lspeed = 100;
    }
    else if (Lspeed <= -100)
    {
      Lspeed = -100;
    }
    if (Rspeed >= 100)
    {
      Rspeed = 100;
    }
    else if (Rspeed <= -100)
    {
      Rspeed = -100;
    }

    if (fabs(angle_error) > angle_accuracy)
    {
      errortimer.clear();
    }
    if (fabs(Lerror) <= accuracy + 0.3 || fabs(Rerror) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    drive(Lspeed - turn_speed, Rspeed + turn_speed, 10);
    Llast_error = Lerror;
    Rlast_error = Rerror;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }
  DriveBrake();
}
void arcturnR(float r, float arcdeg, int timeLimit, float max_drift )
{
  float heading = Gyro.rotation();

  float angle_last_error = 0;
  float angleP = 0;
  float angleD = 0;
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  float robotwidth = 10.5;
  LeftMiddle.setPosition(0, rev);
  RightMiddle.setPosition(0, rev);
  float Lx = 0;
  float Rx = 0;
  float settletime = 150;
  double Ltarget = 2 * pi * (r) * fabs(arcdeg / 360.0);
  double Rtarget = 2 * pi * (r+robotwidth)*fabs(arcdeg / 360.0);
  float Lerror = Ltarget;
  float Rerror = Rtarget;
  float Lkp = 7;
  float count = 0;
  float Rkp = 7;
  float accuracy = 0.1; // was 0.05
  float Lkd = 0.35;
  float Rkd = 0.35;
  double Llast_error = 0;
  double Rlast_error = 0;
  double dt = 0.01;
  double Lki = 0;
  double Lintergal = 0;
  double Rki = 0;
  double Rintergal = 0;
  float angle_target = (Lx * 360) / (2 * pi * r);
  float Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
  float Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
  float angle_error = angle_target;
  vex::timer timer;
  vex::timer errortimer; // Create a timer object
  timer.clear();
  while (fabs(Lerror) >= accuracy || fabs(Rerror) >= accuracy || count <= 5)
  {

    heading = Gyro.rotation();
    Lx = LeftMiddle.position(rev) * pi * dia * gearRatio;
    Rx = RightMiddle.position(rev) * pi * dia * gearRatio;
    angle_target = (Lx * 360) / (2 * pi * r);

    Lerror = Ltarget - Lx;
    Rerror = Rtarget - Rx;
    angle_error = angle_target - heading;

  
    if (angle_error > 180)
    {
      angle_error = angle_error - 360;
    }
    else if (angle_error < -180)
    {
      angle_error = angle_error + 360;
    }


    if (fabs(Lerror) < 10 && fabs(Lerror) > 1)
    {
      Lintergal += Lerror;
    }
    else
    {
      Lintergal = 0;
    }
    
    
    if (Lintergal >= 40)
    {
      Lintergal = 40;
    }
    else if (Lintergal <= -40)
    {
      Lintergal = -40;
    }



    if (fabs(Rerror) < 10 && fabs(Rerror) > 1)
    {
      Rintergal += Rerror;
    }
    else
    {
      Rintergal = 0;
    }
    
    
    if (Rintergal >= 40)
    {
      Rintergal = 40;
    }
    else if (Rintergal <= -40)
    {
      Rintergal = -40;
    }


    Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
    Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
    Brain.Screen.printAt(1, 20, "Lspeed = %.2f     Rspeed = %.2f  ", Lspeed, Rspeed);
    turn_speed = angleP * angle_error + angleD * (angle_error, angle_last_error) / dt;
    turn_speed = std::min(std::fabs(turn_speed), std::fabs(Lspeed) * max_drift);
    Lspeed = Lspeed*0.3;
    Rspeed = Rspeed*0.3;

    if (fabs(angle_error) > angle_accuracy)
    {
      errortimer.clear();
    }
    if (fabs(Lerror) <= accuracy + 0.3 || fabs(Rerror) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;

    drive(Lspeed - turn_speed, Rspeed + turn_speed, 10);
    Llast_error = Lerror;
    Rlast_error = Rerror;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }
  DriveBrake();
}
void arcturnLC(float r, float arcdeg, int timeLimit, float max_drift )
{
  float heading = Gyro.rotation();

  float angle_last_error = 0;
  const float angleP {12};
  const float angleD {0.8};
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  float robotwidth = 12;
  LeftMiddle.setPosition(0, rev);
  RightMiddle.setPosition(0, rev);
  float Lx = 0;
  float Rx = 0;
  float settletime = 150;
  double Ltarget = 2 * pi * (r)*fabs(arcdeg / 360.0);
  double Rtarget = 2 * pi * (r+ robotwidth) * fabs(arcdeg / 360.0);
  float Lerror = Ltarget;
  float Rerror = Rtarget;
  float Lkp = 6;
  float count = 0;
  float Rkp = 6;
  float accuracy = 0.1; // was 0.05
  float Lkd = 0.5;
  float Rkd = 0.5;
  double Llast_error = 0;
  double Rlast_error = 0;
  double dt = 0.01;
  double Lki = 0.5;
  double Lintergal = 0;
  double Rki = 0.5;
  double Rintergal = 0;
  float angle_target = (Rx * 360) / (2 * pi * r);
  float Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
  float Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
  float angle_error = angle_target;
  vex::timer timer;
  vex::timer errortimer; // Create a timer object
  timer.clear();
  while (fabs(Lerror) >= accuracy || fabs(Rerror) >= accuracy)
  {

    heading = Gyro.rotation();
    Lx = LeftMiddle.position(rev) * pi * dia * gearRatio;
    Rx = RightMiddle.position(rev) * pi * dia * gearRatio;
    angle_target = (Lx * 360) / (2 * pi * r);

    Lerror = Ltarget - Lx;
    Rerror = Rtarget - Rx;
    angle_error = angle_target - heading;

    if (angle_error > 180)
    {
      angle_error = angle_error - 360;
    }
    else if (angle_error < -180)
    {
      angle_error = angle_error + 360;
    }


    if (fabs(Lerror) < 10 && fabs(Lerror) > 1)
    {
      Lintergal += Lerror;
    }
    else
    {
      Lintergal = 0;
    }


    if (Lintergal >= 40)
    {
      Lintergal = 40;
    }
    else if (Lintergal <= -40)
    {
      Lintergal = -40;
    }

  
    if (fabs(Rerror) < 10 && fabs(Rerror) > 1)
    {
      Rintergal += Rerror;
    }
    else
    {
      Rintergal = 0;
    }


    if (Rintergal >= 40)
    {
      Rintergal = 40;
    }

    else if (Rintergal <= -40)
    {
      Rintergal = -40;
    }


    Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
    Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
    Brain.Screen.printAt(1, 20, "Lspeed = %.2f     Rspeed = %.2f  ", Lspeed, Rspeed);
    turn_speed = angleP * angle_error + angleD * (angle_error, angle_last_error) / dt;
    turn_speed = std::min(std::fabs(turn_speed), std::fabs(Lspeed) * max_drift);



    std::cout <<"Heading: "<< heading << '\n';
    std::cout <<"angle Error "<<angle_error<<'\n';
    std::cout <<"LSpeed: " << Lspeed << '\n';
    std::cout <<"RSpeed: " << Rspeed << '\n';
    std::cout <<"Lerror: " << Lerror << '\n';
    std::cout <<"Rerror: " << Rerror << '\n';
    std::cout <<"turn_speed: "<< turn_speed << '\n';
    std::cout << "\n";

    if (Lspeed >= 100)
    {
      Lspeed = 100;
    }
    else if (Lspeed <= -100)
    {
      Lspeed = -100;
    }


    if (Rspeed >= 100)
    {
      Rspeed = 100;
    }
    else if (Rspeed <= -100)
    {
      Rspeed = -100;
    }


    Lspeed = Rspeed*(Ltarget/Rtarget);


    if (fabs(angle_error) > angle_accuracy)
    {
      errortimer.clear();
    }


    if (fabs(Lerror) <= accuracy + 0.3 || fabs(Rerror) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;


    if (fabs(Lspeed) < 70 && Lerror > 3)
    {
      if (Lspeed > 0)
      {
        Lspeed = 70;
      }
      else
      {
        Lspeed = -70;
      }
    }


    if (fabs(Rspeed) < 70 && Rerror > 3)
    {
      if (Rspeed > 0)
      {
        Rspeed = 70;
      }
      else
      {
        Rspeed = -70;
      }
    }


    drive(Lspeed-turn_speed, Rspeed+turn_speed, 10);


    Llast_error = Lerror;
    Rlast_error = Rerror;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }
  Drivecoast();
}
void arcturnRC(float r, float arcdeg, int timeLimit, float max_drift )
{
  float heading = Gyro.rotation();

  float angle_last_error = 0;
  float angleP = 6.5;
  float angleD = 0.1;
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  float robotwidth = 15;
  LeftMiddle.setPosition(0, rev);
  RightMiddle.setPosition(0, rev);
  float Lx = 0;
  float Rx = 0;
  float settletime = 150;
  double Ltarget = 2 * pi * (r ) * fabs(arcdeg / 360.0);
  double Rtarget = 2 * pi * (r- robotwidth)*fabs(arcdeg / 360.0);
  float Lerror = Ltarget;
  float Rerror = Rtarget;
  float Lkp = 6;
  float count = 0;
  float Rkp = 6;
  float accuracy = 0.1; // was 0.05
  float Lkd = 0.5;
  float Rkd = 0.5;
  double Llast_error = 0;
  double Rlast_error = 0;
  double dt = 0.01;
  double Lki = 0.5;
  double Lintergal = 0;
  double Rki = 0.5;
  double Rintergal = 0;
  float angle_target = (Rx * 360) / (2 * pi * r);
  float Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
  float Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
  float angle_error = angle_target;
  vex::timer timer;
  vex::timer errortimer; // Create a timer object
  timer.clear();
  while (fabs(Lerror) >= accuracy || fabs(Rerror) >= accuracy)
  {

    heading = Gyro.rotation();
    Lx = LeftMiddle.position(rev) * pi * dia * gearRatio;
    Rx = RightMiddle.position(rev) * pi * dia * gearRatio;
    angle_target = (Lx * 360) / (2 * pi * r);

    Lerror = Ltarget - Lx;
    Rerror = Rtarget - Rx;
    angle_error = angle_target - heading;

    if (angle_error > 180)
    {
      angle_error = angle_error - 360;
    }
    else if (angle_error < -180)
    {
      angle_error = angle_error + 360;
    }
    if (fabs(Lerror) < 10 && fabs(Lerror) > 1)
    {
      Lintergal += Lerror;
    }
    else
    {
      Lintergal = 0;
    } 
    if (Lintergal >= 40)
     {
      Lintergal = 40;
    } 
    else if (Lintergal <= -40)
    {
      Lintergal = -40;
    } 
    if (fabs(Rerror) < 10 && fabs(Rerror) > 1)
    {
      Rintergal += Rerror;
    }
    else    
    { 
      Rintergal = 0; 
    }
    if (Rintergal >= 40)
    {
      Rintergal = 40;
    }

    else if (Rintergal <= -40)
    {
      Rintergal = -40;
    }
    Lspeed = Lkp * Lerror + Lkd * (Lerror - Llast_error) / dt + Lintergal * Lki;
    Rspeed = Rkp * Rerror + Rkd * (Rerror - Rlast_error) / dt + Rintergal * Rki;
    Brain.Screen.printAt(1, 20, "Lspeed = %.2f     Rspeed = %.2f  ", Lspeed, Rspeed);
    turn_speed = angleP * angle_error + angleD * (angle_error, angle_last_error) / dt;
    turn_speed = std::min(std::fabs(turn_speed), std::fabs(Lspeed) * max_drift);

    if (Lspeed >= 100)
    {
      Lspeed = 100;
    }
    else if (Lspeed <= -100)
    {
      Lspeed = -100;
    }
    if (Rspeed >= 100)
    {
      Rspeed = 100;
    }
    else if (Rspeed <= -100)
    {
      Rspeed = -100;
    }
    Rspeed = Rtarget/Ltarget;


    if (fabs(angle_error) > angle_accuracy)
    {
      errortimer.clear();
    }
    if (fabs(Lerror) <= accuracy + 0.3 || fabs(Rerror) <= accuracy + 0.3)
    {
      count++;
    }
    else
      count = 0;
    if (fabs(Lspeed) < 70 && Lerror > 3)
    {
      if (Lspeed > 0)
      {
        Lspeed = 70;
      }
      else
      {
        Lspeed = -70;
      }
    }
    if (fabs(Rspeed) < 70 && Rerror > 3)
    {
      if (Rspeed > 0)
      {
        Rspeed = 70;
      }
      else
      {
        Rspeed = -70;
      }
    }
       
    drive(Lspeed, Rspeed, 10);
    Llast_error = Lerror;
    Rlast_error = Rerror;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit)
    {
      break;
    }
  }
} 
