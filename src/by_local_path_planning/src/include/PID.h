#ifndef __PID_H__
#define __PID_H__

#define KP 0
#define KI 1
#define KD 2
#define KT 3
#define KB 4
#define KF 5


typedef struct PID
{
  float SumError;	//误差累计	
  int LastError;	//Error[-1]
  int PrevError;	//Error[-2]	
  int LastData;	//Speed[-1]
}PID;

float Direct_x[4]  = {0.0035, 0.00001, 0.0001, 0};	
float Direct_angle[4]  = {0.005, 0.00001, 0.0001,0.01};	
float straight_second[4]  = {0.002, 0.00001, 0.0001,0};	
float Direct_angle1[4]  = {0.02, 0.00001, 0.0001,0};	
float stop[4]  = {0.1, 0, 0,0.001};	
float straight[4]  = {1, 0, 0,0};	
PID Direct_x_PID,Direct_angle_PID,stop_PID,straight_PID,straight_PID_second;

float stop3_angle_pid[4]  = {0.0035, 0.00001, 0.0001, 0};	
PID stop3_angle_PID;
/////////寻迹部分pid
float angle_pid_xj[4]  ={0.004, 0.00001, 0.0001,0};	

PID Angle_pid_param_xj;
//////////////////
// PID参数初始化
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	//Error[-1]
	sptr->PrevError = 0;	//Error[-2]	
	sptr->LastData  = 0;
}

	//PID_Realize1(&Direct_PID, Direct, (int32)(IMC_Treated.gyro.z)*100, (int32)(Speed_Min*Radius));
// 位置式PID控制
float PID_Realize1(PID *sptr, float *PID, float NowData, float Point)   //转向环
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	float iError,	// 当前误差
		  Realize;	// 最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差
	sptr->SumError += PID[KI] * iError;	// 误差积分
	if (sptr->SumError >= PID[KT])
	{
		sptr->SumError = PID[KT];
	}
	else if (sptr->SumError <= -PID[KT])
	{
		sptr->SumError = -PID[KT];
	}
	
	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据

	return Realize;	// 返回实际值
}

// 增量式PID电机控制
float PID_Increase1(PID *sptr, float *PID, float NowData, float Point)  //角速度内环
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	float iError,	//当前误差
		Increase;	//最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差

	Increase =  PID[KP] * (iError - sptr->LastError)
			  + PID[KI] * iError
			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
	
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据
	
	return Increase;	// 返回增量
}


// 位置式动态PID控制
float PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint)
{
	//定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	float iError,	//当前误差
		  Actual;	//最后得出的实际输出值
	float Kp;		//动态P
	
	iError = SetPoint - NowPiont;	//计算当前误差
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <= -PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
//        Kp = 1.0 * abs(iError) / PID[KP] + PID[KI];
	Kp =  (iError*iError) / PID[KP] + PID[KI];	//P值与差值成二次函数关系，此处P和I不是PID参数，而是动态PID参数
	
	Actual = Kp * iError
		  + PID[KD] * (0.8*iError- 0.8*sprt->LastError);//只用PD
	sprt->LastError = iError;		//更新上次误差

	return   Actual;
}



#endif
