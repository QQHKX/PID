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

// 重置电机编码器的位置
void EncoderReset() {
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

// 比赛前的初始化函数
void pre_auton(void) {
  vexcodeInit();  // 初始化VEX设备
  EncoderReset(); // 重置电机编码器
  EncoderInit();  // 初始化电机参数
  wait(2000, timeUnits::msec);  // 等待2秒，确保初始化完成
}

/*---------------------------------------------------------------------------*/
/*                          自动化部分代码                                   */
/*---------------------------------------------------------------------------*/

// 将旋转角度转换为线性距离（毫米）
double degreesToMillimeters(double distanceMM) {
  const double wheelDiameterMM = 100.0; // 假设轮径为100毫米
  double circumferenceMM = M_PI * wheelDiameterMM; // 计算轮子周长
  double distancedegree = (distanceMM * 360) / circumferenceMM; // 转换为角度
  return distancedegree;
}

// 控制底盘以指定的功率移动
void Move(int left_power, int right_power) {
  Left1.spin(forward, 0.128 * left_power, volt);
  Left2.spin(forward, 0.128 * left_power, volt);
  Left3.spin(forward, 0.128 * left_power, volt);

  Right1.spin(forward, 0.128 * right_power, volt);
  Right2.spin(forward, 0.128 * right_power, volt);
  Right3.spin(forward, 0.128 * right_power, volt);
}

// 停止底盘
void Move_stop(void) {
  Left1.stop(brake);
  Left2.stop(brake);
  Left3.stop(brake);
  Right1.stop(brake);
  Right2.stop(brake);
  Right3.stop(brake);
}

// 使用PID控制机器人前进（以毫米为单位）
void moveForwardPID(double targetDistanceMM) {
  const double wheelDiameterMM = 100.0; // 假设轮子直径为100毫米
  const double wheelCircumferenceMM = M_PI * wheelDiameterMM; // 计算轮子周长

  // PID控制参数
  double kP = 1.4;  // 比例增益
  double kI = 0.01; // 积分增益
  double kD = 0.8;  // 微分增益

  double integral = 0;               // 积分项
  double previousError = 0;          // 上一次误差
  double error = targetDistanceMM;   // 初始化误差为目标距离
  double toleranceMM = 5.0;          // 容忍误差范围（毫米）

  while (fabs(error) > toleranceMM) {
    // 获取当前平均距离（将编码器角度转换为毫米）
    double leftPositionMM = (Left1.position(degrees) / 360.0) * wheelCircumferenceMM;
    double rightPositionMM = (Right1.position(degrees) / 360.0) * wheelCircumferenceMM;
    double currentDistanceMM = (leftPositionMM + rightPositionMM) / 2.0;

    // 计算误差
    error = targetDistanceMM - currentDistanceMM;

    // 计算积分项并防止积分饱和
    integral += error;
    if (fabs(integral) > 5000) { // 限制积分累计值
      integral = (integral > 0) ? 5000 : -5000;
    }

    // 计算微分项
    double derivative = error - previousError;
    previousError = error;

    // 计算输出速度
    double outputSpeed = kP * error + kI * integral + kD * derivative;

    // 限制速度输出范围
    if (outputSpeed > 100) outputSpeed = 100;
    if (outputSpeed < -100) outputSpeed = -100;

    // 控制电机前进
    Move(outputSpeed, outputSpeed);

    // 等待一段时间（20ms）以确保控制器的稳定性
    wait(20, msec);
  }

  // 停止机器人
  Move_stop();
}

// 使用PID控制机器人转向
void turnToAngle(double targetAngle) {
  double kP = 1.5, kI = 0.0092, kD = 3.2; // PID控制参数
  double previousError = 0, integral = 0;

  while (true) {
    double currentAngle = InertialSensor.rotation(degrees);
    double error = targetAngle - currentAngle;

    if (fabs(error) < 1) break;  // 当误差小于1度时退出

    integral += error;
    double derivative = error - previousError;
    previousError = error;

    double turnSpeed = kP * error + kI * integral + kD * derivative;

    Move(turnSpeed, -turnSpeed);  // 控制底盘转向
    wait(20, msec);
  }
  Move_stop(); // 停止底盘
}

// 自动化任务
void autonomous(void) {
  EncoderReset(); // 重置编码器
  InertialSensor.setRotation(0, degrees); // 重置惯性传感器

  moveForwardPID(1000);  // 前进1000毫米
  wait(20, msec);
  turnToAngle(90);       // 右转90度
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
      Move(Axis_3 + Axis_4, Axis_3 - Axis_4); // 混合控制
    } else {
      Move_stop(); // 停止底盘
    }
    wait(20, msec);
  }
}

// 驾驶员控制函数
void drivercontrol(void) {
  thread task1(thread_Move); // 启动底盘控制线程
  bool Button_A, Button_L1, Button_B;

  while (true) {
    Button_A = Controller1.ButtonA.pressing();
    Button_L1 = Controller1.ButtonL1.pressing();
    Button_B = Controller1.ButtonB.pressing();

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
