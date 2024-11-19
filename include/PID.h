/*2023-03-20
PID算法更新V0.1
@auther unibeam
*/
typedef struct PID_
{
    float actual_value;      //实际值 每次反馈回来的值
    float error;           //偏差值
    float Kd, Ki, Kp;         //比例、积分、微分常数
    float P, I, D;             //比例项 积分项 微分项
    float error_pre;            //E[k-1]  //上一次误差
    float error_pre_pre;        //E[k-2]  //上上次误差
    float set_value;              //设定值 你期望系统达到的值
    float integral;         //积分值
}PID;
void pid_init(double kp, double ki, double kd, PID *pid);
void pidForward(int degree, int compute_time, int power, float Kp, float ki, float kd);
void pidTurn(int degree, int compute_time, int power, float Kp, float ki, float kd);
void drawFram();
void drawResult(int this_time, int all_time, int value, int target);

//old
void Forward_Distance(int Aim,int Max_Power,int Exit_Time);
void Backward_Distance(int Aim,int Max_Power,int Exit_Time);
void InertialSensorTurn(float target);
void InertialSensorMove(double power, double enc , float g);
float position_pid(PID *pid);