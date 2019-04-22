
//往右打方向为减小，往左打方向为增大

typedef struct PID
{
    double Angle_Target;//要达到的目标位置
    double current_Angle;//现在的位置

    double Kp;
    double Ti;
    double Td;
    
    double now_error;
    double last_error;
    double pre_error;

    double Angle_output;//增量式PID的输出，为角度的输出
    uint16_t turn_output;//将角度的输出转化为PWM的输出

    double T;//采样周期

    int i;//设置初始航偏角为小车要保持的位置
}kPID;

static kPID sPID;


void PID_Param_Init()
{
    sPID.Angle_Target = 0;//要从IMU读取初始值

    sPID.now_error = 0;
    sPID.last_error = 0;
    sPID.pre_error = 0;

    sPID.Kp = 4;
    sPID.Ti = 0;
    sPID.Td = 0;

    sPID.T = 0.05;//根据实际设置采样频率

    sPID.i = 0;
}


// 采用增量式PID,只需要采用3次误差的值
uint16_t PID_control(const double Angle_fact)
{
    sPID.now_error = Angle_fact;
    // sPID.Angle_output = sPID.Kp*(1 + sPID.T/sPID.Ti + sPID.Td/sPID.T)*sPID.now_error - 
    //                     sPID.Kp*(1 + 2*sPID.Td/sPID.T)*sPID.last_error + 
    //                     sPID.Kp*sPID.Td*sPID.pre_error/sPID.T;
    
    //sPID.Angle_output = sPID.Kp * sPID.now_error;//纯比例调节


    if(sPID.Angle_output > 45)//对输出进行限幅
    {
        sPID.Angle_output = 45;
    }
    else if(sPID.Angle_output < -45)
    {
        sPID.Angle_output = -45;
    }

    //进行前后误差的工作交接，保存3次误差
    sPID.pre_error = sPID.last_error;
    sPID.last_error = sPID.Angle_Target;

    sPID.turn_output = (int)(10.0/9.0*sPID.Angle_output + 90);

    return sPID.turn_output;//返回的值为40到140
}