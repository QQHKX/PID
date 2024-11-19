#include <vex.h>
#include <motorControl.h>
#include <PID.h>
#include <ctime>
#include <cmath>
#include <bits/stdc++.h>

#define LOC_DEAD_ZONE 60 /*位置环死区*/
#define LOC_INTEGRAL_START_ERR 200 /*积分分离时对应的误差范围*/
#define LOC_INTEGRAL_MAX_VAL 800   /*积分范围限定，防止积分饱和*/
const int NEGATIVE = -1;


/*2023-03-20
PID算法更新V0.1
@author unibeam
*/

/*初始化PID对象
@param PID 要计算的pid
@param Kp, Ki, Kd PID三个参数值
*/

void pid_init(double kp, double ki, double kd, PID *pid)
{
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_pre= 0;
    pid->error_pre_pre = 0;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->set_value = 0;
    pid->integral = 0;
    pid->P = 0;
    pid->I = 0;
    pid->D = 0;
}

/*位置式pid
@param pid  要计算的pid
@param set_value 想要设定的值
返回 增量
*/
float position_pid(PID *pid)
{
    pid->error = pid->set_value - pid->actual_value;
    pid->integral += pid->error;   /*误差累积*/

    //比例项
    pid->P = pid->Kp*pid->error;

    //积分项
    pid->I = pid->Ki*pid->integral;

    //微分项
    pid->D =pid->Kd*(pid->error-pid->error_pre);

    pid->error_pre_pre = pid->error_pre;
    pid->error_pre = pid->error;

    float uk = pid->P + pid->I +pid->D;
    pid->actual_value += uk;

    return uk;
}

/*增量式PID算法
@param pid  要计算的pid
@param set_value 想要设定的值 可以作为入参
返回给执行器的输出值
*/
float incremental_pid(PID *pid)
{
    float P, I, D,increment;
    pid->error = pid->set_value - pid->actual_value;

    P = pid->Kp*(pid->error-pid->error_pre);
    I = pid->Ki*pid->error;
    D = pid->Kd*(pid->error-2*pid->error_pre + pid->error_pre_pre);
    
    increment = P + I + D;   /*pid计算得到的增量*/
    pid->actual_value += increment;     /*实际反馈值加上增量*/

    pid->error_pre_pre = pid->error_pre;
    pid->error_pre = pid->error;

    return increment;
}

/*编码器与陀螺仪重置
返回空
PS：api文档另有public void vex::InertialSensor::startCalibration(int32_t value=0)的函数用于陀螺仪校准（非重置）
可以试一试什么效果
*/
  void EncoderReset()
  {
      Left1.resetPosition();
      Left2.resetPosition();
      Left3.resetPosition();
      Right1.resetPosition();
      Right2.resetPosition();
      Right3.resetPosition();
      InertialSensor.resetRotation();
  }

/*画出输出坐标系的框架
返回空
*/
void drawFram(){
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawRectangle(20, 10, 400, 200);
  Brain.Screen.drawLine(20, 120, 420, 120);
  Brain.Screen.printAt(20, 30, "200");
  Brain.Screen.printAt(430, 240, "8");
  Brain.Screen.setPenColor(green);
}

/*画出输出图形，用于循环中，所以需要像函数中传输循环次数才能画出连续图形
图形是根据循环次数与目标值自适应比例，无需调整
@param this_time 已经循环的次数
@param all_time 总循环次数
@param value 实际测量的值
@param target 目标值
返回空
*/
void drawResult(int this_time, int all_time, int value, int target){
  int x = 20 + (this_time * 400 / all_time);
  int y = 220 - (value * 220 / (target * 2));
  Brain.Screen.drawPixel(x, y);
}

/*PID算法直行
@param degree 目标值需要转动的角度
@param compute_time 需要计算的次数，可以修改wait时间调整精度
@param power PID输出数值对应马达功率的比率
@param Kp，Ki，Kd PID参数
返回空
*/
void pidForward(int degree, int compute_time, int power, float Kp, float Ki, float Kd){
  PID pos_pid;
  pid_init(Kp, Ki, Kd, &pos_pid);

  pos_pid.set_value = degree;
  EncoderReset();
  drawFram();

  for(int i = 0; i < compute_time; i++){
    pos_pid.actual_value = fabs(Left1.position(vex::rotationUnits::deg));
    //位置式的PID算法
    float uk = position_pid(&pos_pid)/degree * power;
    //增量式的PID算法
    //float uk = incremental_pid(&pos_pid)/degree * power;
    drawResult(i, compute_time, pos_pid.actual_value, degree);
    ChassisControl(uk, uk);
    //可修改时间增加调整精度
    wait(1, vex::timeUnits::msec);
  }
  //运行结束后选择停止方式
  Stop(vex::brakeType::hold);
}

/*PID算法转弯，参数需要调整，结构与pidForward相同
@param degree 目标值需要转动的角度
@param compute_time 需要计算的次数，可以修改wait时间调整精度
@param power PID输出数值对应马达功率的比率
@param Kp，Ki，Kd PID参数
返回空
*/
void pidTurn(int degree, int compute_time, int power, float Kp, float Ki, float Kd){
  PID pos_pid;
  pid_init(Kp, Ki, Kd, &pos_pid);

  pos_pid.set_value = degree;
  EncoderReset();
  drawFram();

  for(int i = 0; i < compute_time; i++){
    pos_pid.actual_value = fabs(InertialSensor.rotation(vex::rotationUnits::deg));
    //位置式的PID算法
    float uk = position_pid(&pos_pid)/degree * power;
    //增量式的PID算法
    //float uk = incremental_pid(&pos_pid)/degree * power;
    drawResult(i, compute_time, pos_pid.actual_value, degree);
    ChassisControl(uk, uk * NEGATIVE);
    wait(1, vex::timeUnits::msec);
  }
}






//old

int sgn(double d)
{
  if (d<0) return -1;
  else if (d==0) return 0;
  else return 1;
}
void turn(int turnpwm,int forwardpwm) //编码转向
{
  float turnpower = turnpwm*12/127;
  float forwardpower = forwardpwm*12/127;
  Left1.spin(directionType::fwd, forwardpower + turnpower, voltageUnits::volt);
  Left2.spin(directionType::fwd, forwardpower + turnpower, voltageUnits::volt);
  Left3.spin(directionType::fwd, forwardpower + turnpower, voltageUnits::volt);
  Right1.spin(directionType::fwd, forwardpower +- turnpower, voltageUnits::volt);
  Right2.spin(directionType::fwd, forwardpower +- turnpower, voltageUnits::volt);
  Right3.spin(directionType::fwd, forwardpower +- turnpower, voltageUnits::volt);
}

void InertialSensorTurn(float target) //pid转向
{ 
  int timeused = 0;
  float kp = 5;
  float kd = 30;
  float dtol = 0.1;
  float errortolerance = 1;
  float lim =127;
  float error = target - InertialSensor.rotation();
  float lasterror;
  float v = 0;
  bool arrived;
  float timetol = fabs(error) < 300 ? 700 : fabs(error) * 1.2;//2.7
  float pow;
  int endp = 0;
  lasterror = error;
  arrived = error == 0;
  if(target>=0)
  {
    endp=-5;
  }
  else
  {
    endp=5;
  }
  while (!arrived)
  {
    timeused += 10;
    error = target - InertialSensor.rotation();
    v = error - lasterror;
    lasterror = error;
    //if ((fabs(error) < errortolerance && fabs(v) <= dtol) || timeused > timetol){arrived = true;}
    if ((fabs(error) < errortolerance && fabs(v) <= dtol) || timeused > timetol)
    {
      arrived = true;
      turn(0,0);
    }
    pow = kp * error + kd * v ;
    pow = fabs(pow) > lim ? sgn(pow) * lim : pow;
    turn(pow,0);
    wait(10, msec);
  }
  turn(0, 0);
}

void encodemove(int deg, double end_power, int time, bool rev) //编码移动
{
    EncoderReset();
    double deg_to_rotate = deg;
    if (rev == false)
    {
        Left1.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Left2.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Left3.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Right1.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Right2.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Right3.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
    }
    else
    {
        Left1.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Left2.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Left3.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Right1.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Right2.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
        Right3.spinFor(deg_to_rotate,vex::rotationUnits::deg,end_power,vex::velocityUnits::pct,false);
    }
    wait(time, msec);
}

void forward_(double leftpower,double rightpower)
{
  double left = leftpower *12/127;
  double right = rightpower*12/127;
  Left1.spin(directionType::fwd, left, voltageUnits::volt);
  Left2.spin(directionType::fwd, left, voltageUnits::volt);
  Left3.spin(directionType::fwd, left, voltageUnits::volt);
  Right1.spin(directionType::fwd, right, voltageUnits::volt);
  Right2.spin(directionType::fwd, right, voltageUnits::volt);
  Right3.spin(directionType::fwd, right, voltageUnits::volt);
}


void InertialSensorMove( double power, double enc , float g) //陀螺仪移动
{ 
  //bool finish;
  int time=0;
  float menc;
  float turnpower;
  EncoderReset();
  //int timeout_ =enc<300?500:enc*1.5;
  while(true)
  {
    time += 1;
    menc = (Left1.position(rotationUnits::deg) + Right1.position(rotationUnits::deg)+Left2.position(rotationUnits::deg) + Right2.position(rotationUnits::deg)+Left3.position(rotationUnits::deg) + Right3.position(rotationUnits::deg))/6;
    turnpower = 1.16*(g - InertialSensor.rotation());
    //if (time>timeout){finish = true;}
    if (fabs(enc)-fabs(menc)<3 )
    {
      break;
    }
    if (enc > 0)
    {
      forward_(power+turnpower,power-turnpower);
    }
    else
    {
      forward_(-power+turnpower,-power-turnpower);
    }
    wait(10, msec);
  }
  forward_(0,0);
}





