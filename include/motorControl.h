void MoveTo(vex::motor motor, double rotation, double velocity);

//old
void gyroInit();
void ChassisControl(float lPower, float rPower);
void Stop(vex::brakeType type = vex::brake);
void RollersControl(int power);
void CatapultControl(int power);
void InertialSensorTurn(float Lspeed,float Rspeed,float degree);
void InertialSensorMove(float Lspeed,float Rspeed,float degree);

double return_angle();
void inertial_clear();
int sgn(double output);
void just_stop(int stopmod);
void just_right_spin(int output);
void pid_turn(double target_degree,int max_speed);
void pid_move(int mubiao,int target_degree,double speed_limit,double speed_limit2);
void GyroBack(int Power);
