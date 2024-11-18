/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\10340                                            */
/*    Created:      Tue Oct 08 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Left1                motor         10              
// Left2                motor         9               
// Left3                motor         8               
// Right1               motor         7               
// Right2               motor         6               
// Right3               motor         5               
// Roller1              motor         3               
// Roller2              motor         4               
// Arm                  motor         2               
// A                    digital_out   A               
// Controller1          controller                    
// InertialSensor       inertial      11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void EncoderInit()
{
  Left1.setMaxTorque(100,percent);
  Left2.setMaxTorque(100,percent);
  Left3.setMaxTorque(100,percent);
  Right1.setMaxTorque(100,percent);
  Right2.setMaxTorque(100,percent);
  Right3.setMaxTorque(100,percent);
  Roller1.setMaxTorque(100,percent);
  Roller2.setMaxTorque(100,percent);
  A.set(false); 
  
}

void EncoderReset()
{
  Left1.resetPosition();
  Left2.resetPosition();
  Left3.resetPosition();
  Right1.resetPosition();
  Right2.resetPosition();
  Right3.resetPosition();
  Roller1.resetPosition();
  Roller2.resetPosition();
  Arm.resetPosition();
  
}
void pre_auton(void) 
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  EncoderReset();
  EncoderInit();
  wait(2000, timeUnits::msec);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// 函数将编码器的角度转换为线性距离（毫米）
double degreesToMillimeters(double distanceMM) {
  const double wheelDiameterMM = 800.0; // 例如，轮子直径为100毫米
    double circumferenceMM = M_PI * wheelDiameterMM; // 计算轮子周长（毫米）
    double distancedgree = (distanceMM*360) / circumferenceMM; // 计算线性距离（毫米）
    return distancedgree;
}
void Move(int left_power,int right_power)
{
  Left1.spin(forward, 0.128*left_power ,volt);
  Left2.spin(forward, 0.128*left_power ,volt);
  Left3.spin(forward, 0.128*left_power ,volt);

  Right1.spin(forward, 0.128*right_power ,volt);
  Right2.spin(forward, 0.128*right_power ,volt);
  Right3.spin(forward, 0.128*right_power ,volt);
}
void Move_stop(void)
{
  Left1.stop(brake);
  Left2.stop(brake);
  Left3.stop(brake);
  Right1.stop(brake);
  Right2.stop(brake);
  Right3.stop(brake);
}
void moveForwardPID(double targetDistance) {
  targetDistance=degreesToMillimeters(targetDistance);
  double kP_forward = 1.4;  // 比例常数
  double kI_forward = 0.01; // 积分常数
  double kD_forward = 0.8;  // 微分常数
  double forwardIntegral = 0;
  double forwardPreviousError = 0;
  // 假设使用编码器来测量距离
  double currentDistance = (Left1.position(degrees) + Right1.position(degrees)) / 2.0;
  double error = targetDistance - currentDistance;
  
  forwardIntegral += error;
  double derivative = error - forwardPreviousError;
  forwardPreviousError = error;
  
  double forwardSpeed = kP_forward * error + kI_forward * forwardIntegral + kD_forward * derivative;
  
  // 控制电机前进
  Move(forwardSpeed, forwardSpeed);
  
  // 等待一段时间以使控制器生效
  wait(20, msec);
}

void turnToAngle(double targetAngle) {
  // PID 控制参数
double kP = 1.5;  // 比例常数
double kI = 0.0092; // 积分常数
double kD = 3.2;  // 微分常数

double previousError = 0;
double integral = 0;
  double error = targetAngle - InertialSensor.rotation(degrees);
  integral = 0;
  previousError = error;

  while (fabs(error) > 1) { // 当误差小于1度时停止
    double currentAngle = InertialSensor.rotation(degrees);
    error = targetAngle - currentAngle;

    integral += error;
    double derivative = error - previousError;
    previousError = error;

    double turnSpeed = kP * error + kI * integral + kD * derivative;

    // 控制电机转向
    Move(turnSpeed, -turnSpeed);

    wait(20, msec);
  }

  Move_stop(); // 停止机器人
}


void autonomous(void) 
{
  EncoderReset(); // 确保编码器从零开始
  InertialSensor.setRotation(0, degrees); // 重置惯性传感器

  // 示例：让机器人前进1000（编码器单位）
  while (true) {
    double targetDistance = 10000; // 目标距离

    moveForwardPID(targetDistance);

    // 检查是否到达目标距离
    double currentDistance = (Left1.position(degrees) + Right1.position(degrees)) / 2.0;
    if (fabs(targetDistance - currentDistance) < 5) { // 误差小于5度时停止
      break;
    }
  }

  Move_stop(); // 停止机器人
  
  // 完成后停止所有运动
  Move_stop();
/*
Move(50,50);//底盘运动,Move(左边底盘速度，右边底盘速度)
wait(1000,msec);//等待，单位毫秒
Move_stop();//底盘停止（仅控制底盘，不控制其他电机）注意：底盘运动后一定要写停止，不然会一直往前走
wait(200,msec);
Move(50,-50);//右转
wait(500,msec);
Move_stop();
wait(200,msec);
Arm.spin(fwd,50,pct);//电机运动控制，反转在数字处加负号
wait(500,msec);
Arm.stop();//电机运动停止，在电机工作后也一定要写停止，不然也会一直转
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
*/
}




void thread_Move()
{
  int Axis_1,Axis_2,Axis_3,Axis_4;
  bool Button_R1,Button_R2;
  bool Button_L1;
  while (1) 
  {
    Axis_1=Controller1.Axis1.value();
    Axis_2=Controller1.Axis2.value();
    Axis_3=Controller1.Axis3.value();
    Axis_4=Controller1.Axis4.value();
    Button_R1=Controller1.ButtonR1.pressing();
    Button_R2=Controller1.ButtonR2.pressing();
    Button_L1=Controller1.ButtonL1.pressing();
    
    //====================================================手柄定义

    if (abs(Axis_3)<5 && abs(Axis_4)>5)
    {
      Move(0.7 * Axis_4,- 0.7 * Axis_4);
    }
    else if (abs(Axis_3)>5)
    {
      Move(Axis_3 + 0.7 * Axis_4,Axis_3 - 0.7 * Axis_4);
    }
    else
    {
      Move_stop();
    }

    if (Button_R2)
    {
     Roller1.spin(fwd,100,pct);
     Roller2.spin(fwd,100,pct);
    }
    else if (Button_R1) 
    {
     Roller1.spin(fwd,-100,pct);
     Roller2.spin(fwd,-100,pct);
     
    }
    
    else 
    {
      Roller1. stop(coast);
      Roller2.stop(coast);
    }//滚筒

    
  }
}//底盘控制

void drivercontrol(void) 
{
  int f = 0;
  thread task1,task2;
  bool Button_A,Button_B,Button_Y,Button_X;
  bool Button_L1,Button_L2;
  bool Button_Up,Button_Down,Button_Left,Button_Right;
    
  //====================================================手动参数定义
  // User control code here, inside the loop
  while (1) 
  {
    Button_A=Controller1.ButtonA.pressing();
    Button_Y=Controller1.ButtonY.pressing();
    
    Button_L1=Controller1.ButtonL1.pressing();    
    Button_L2=Controller1.ButtonL2.pressing();

    Button_Up=Controller1.ButtonUp.pressing();
    Button_Down=Controller1.ButtonDown.pressing();
    Button_Left=Controller1.ButtonLeft.pressing();
    Button_Right=Controller1.ButtonRight.pressing();

    Button_X=Controller1.ButtonX.pressing();
    Button_A=Controller1.ButtonA.pressing();
    Button_B=Controller1.ButtonB.pressing();
    //====================================================手柄定义
     task1 = thread(thread_Move);

     if(Button_A)
    {
      Arm.spinTo(13, deg, 200, rpm);
    }
   
    if (Button_X)
    {
     Arm.spin(fwd,100,pct); 
    }
     else if (Button_L1)
    {
     Arm.spin(fwd,100,pct); 
    }
    else if (Button_B) 
    {
      Arm.spin(fwd,-100,pct);
    }
    else 
    {
     Arm.stop(hold);
    }//机械臂

    

    
     
     if(Button_L2 and f == 0){
       A.set(1);
       waitUntil(!Controller1.ButtonL2.pressing());
       f = 1;
     }
     else if(Button_L2 and f == 1){
       A.set(0);
       waitUntil(!Controller1.ButtonL2.pressing());
       f = 0;
     }
     else if(f == 0){
     A.set(0);
     }//夹子

     

    //余下部分逻辑在这之下续写
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//
int main() 
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.drivercontrol(drivercontrol);
  printf("build time： %s\n\n",__TIME__);
  Competition.autonomous(autonomous);
  // Run the pre-autonomous function.
  
}
