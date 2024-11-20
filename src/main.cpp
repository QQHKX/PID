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
// InertialSensor       inertial      20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include<PID.h>
#include<robot-config.h>
#include<motorControl.h>

using namespace vex;

// 创建比赛的全局实例
competition Competition;

// 定义全局设备的实例
// 请确保所有设备的配置与VEX配置工具中一致
/*---------------------------------------------------------------------------*/
/*                          初始化相关函数                                   */
/*---------------------------------------------------------------------------*/

// 初始化电机的最大扭矩
void EncoderInit() {
  Left1.setMaxTorque(100, percent);
  Left2.setMaxTorque(100, percent);
  Left3.setMaxTorque(100, percent);
  Right1.setMaxTorque(100, percent);
  Right2.setMaxTorque(100, percent);
  Right3.setMaxTorque(100, percent);
  Roller1.setMaxTorque(100, percent);
  Roller2.setMaxTorque(100, percent);
  A.set(false);  // 初始化电磁夹关闭
}

// 比赛前的初始化函数
void pre_auton(void) {
  vexcodeInit();  // 初始化VEX设备
  EncoderInit();  // 初始化电机参数
  wait(2000, timeUnits::msec);  // 等待2秒，确保初始化完成
}

/*---------------------------------------------------------------------------*/
/*                          自动化部分代码                                   */
/*---------------------------------------------------------------------------*/

// 将旋转角度转换为线性距离（毫米）

// 使用PID控制机器人转向
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
    ChassisControl(turnSpeed, -turnSpeed);

    wait(20, msec);
  }

    Stop(); // 停止机器人
}






/*---------------------------------------------------------------------------*/
/*                          自动驾驶控制部分                                   */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  EncoderInit(); 
  InertialSensor.setRotation(0, degrees); // 重置惯性传感器
 

  
  
 
  
  //turnToAngle(90);       // 右转90度
}



/*---------------------------------------------------------------------------*/
/*                          驾驶员控制部分                                   */
/*---------------------------------------------------------------------------*/

// 驾驶员底盘控制线程
void thread_Move() {
  int Axis_3, Axis_4;
  while (true) {
    Axis_3 = Controller1.Axis3.value(); // 前后轴
    Axis_4 = Controller1.Axis4.value(); // 转向轴

    if (abs(Axis_3) > 5 || abs(Axis_4) > 5) {
      ChassisControl(Axis_3 + Axis_4, Axis_3 - Axis_4); // 混合控制
    } else {
      Stop(); // 停止底盘
    }
    wait(20, msec);
  }
}

// 驾驶员控制函数
void drivercontrol(void) {
  thread task1(thread_Move); // 启动底盘控制线程
  bool Button_A, Button_L1, Button_B;
  bool Button_R1, Button_R2;

  while (true) {
    Button_A = Controller1.ButtonA.pressing();
    Button_L1 = Controller1.ButtonL1.pressing();
    Button_B = Controller1.ButtonB.pressing();
    Button_R1 = Controller1.ButtonR1.pressing();
    Button_R2 = Controller1.ButtonR2.pressing();

    if (Button_A) {
      Arm.spin(fwd, 50, pct); // 上升
    } else if (Button_B) {
      Arm.spin(reverse, 50, pct); // 下降
    } else {
      Arm.stop(hold); // 保持位置
    }

    if (Button_L1) {
      A.set(true);  // 打开夹子
    } else {
      A.set(false); // 关闭夹子
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

    

    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                             主函数                                        */
/*---------------------------------------------------------------------------*/

int main() {
  Competition.autonomous(autonomous); // 设置自动化回调
  Competition.drivercontrol(drivercontrol); // 设置驾驶员控制回调
  pre_auton(); // 调用初始化函数
}
