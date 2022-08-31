#include "PID.h"
///

///
datatype Upright(datatype Med,datatype theta,datatype gro)
{

		static float Upright_Kp=90,Upright_Kd=16;//36,24
		static datatype Upright_out=0;
    Upright_out = Upright_Kp*(theta-Med)+Upright_Kd*(gro);
	
		if(Upright_out>1000) Upright_out=1000;
		if(Upright_out<-1000) Upright_out=-1000;
	
    return Upright_out;
}


datatype velocity(int target,int left_speed,int right_speed)
{
	static datatype velocity_out=0;
	static float velocity_Kp=0.1,velocity_Ki=0.1;
	static int speed,velocity_S,low_out_last,low_out;
//	
//	float a=0.7;
	speed = ((left_speed+right_speed)-target);
//	low_out = (1-a)*speed+a*low_out_last;
//	low_out_last=low_out;
//	
//	velocity_S+=low_out;
//	velocity_out = velocity_Kp*speed+velocity_Ki*velocity_S;
	velocity_out = velocity_Kp*speed;
	
	return velocity_out;
}
