#include <vex.h>
#include <motorControl.h>
#include <PID.h>
#include <ctime>
#include <string>
float POW;
timer T1;    //时间片计时器
timer T2;
timer T3;
timer TACC;
int act=0;
double now_angle=0;

/*2023-03-20
更新MoveTo函数V0.1
@author unibeam
*/

/*输出信息
@param s 需要显示的信息
@param controller 需要显示信息的控制器
在手柄屏幕上显示控制信息
*/
void out(std::string s, vex::controller controller){
  controller.Screen.clearScreen();
  controller.Screen.print(s.c_str());
}

/*输出信息(重载)
@param s 需要显示的信息
@param controller 需要显示信息的控制器
在主控屏幕上显示控制信息
*/
void out(std::string s, vex::brain brain){
  brain.Screen.clearScreen();
  brain.Screen.print(s.c_str());
}

/*控制电机移动位置，
@param vex::motor 选择需要移动的电机
@param rotation 移动的距离，现在是degree，可以修改vex::rotationUnits来改变单位
@param velocity 设置移动速度，现在为rpm，可以修改vex::velocityUnits来改变单位
封装的是vex::motor::spinTo函数，最后一个true代表是否等待完成
*/
void MoveTo(vex::motor motor, double rotation, double velocity){
  motor.resetPosition();
  if(motor.spinTo(rotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::rpm, true)){
    //out("motor move to target success", Controller1);
    Stop(vex::brakeType::hold);
  }else{
    out("motor move to target false", Controller1);
  }
}


//====================================================================================

void ChassisControl(float lPower, float rPower)
{
  Left1. spin(fwd, 0.128 * lPower, volt);
  Left2. spin(fwd, 0.128 * lPower, volt);
  Left3. spin(fwd, 0.128 * lPower, volt);

  Right1.spin(fwd, 0.128 * rPower, volt);
  Right2.spin(fwd, 0.128 * rPower, volt);
  Right3.spin(fwd, 0.128 * rPower, volt);
}


//====================================================================================

void Stop(brakeType type)
{
  Left1.stop(type);
  Left2.stop(type);
  Left3.stop(type);

  Right1.stop(type);
  Right2.stop(type);
  Right3.stop(type);
}

//====================================================================================



//====================================================================================




//====================================================================================


  



