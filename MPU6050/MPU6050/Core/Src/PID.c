#include "PID.h"
///

///
datatype Upright(int Med,datatype theta,datatype gro)
{

		static float Upright_Kp=50,Upright_Kd=24;
		static datatype Upright_out=0;
	
	if(theta-Med<5 || theta-Med>-5)	Upright_Kp=100;
	else	Upright_Kp=200;
    Upright_out = Upright_Kp*(theta-Med)+Upright_Kd*(gro);
    return Upright_out;
}


datatype velocity(int target,int left_speed,int right_speed)
{
	static datatype velocity_out=0;
	static float velocity_Kp=16,velocity_Ki=1.6;
	static int speed,velocity_S,low_out_last,low_out;
	
	float a=0.7;
	speed = ((left_speed+right_speed)-target);
	low_out = (1-a)*speed+a*low_out_last;
	low_out_last=low_out;
	
	velocity_S+=low_out;
	velocity_out = velocity_Kp*speed+velocity_Ki*velocity_S;
	velocity_out = velocity_Kp*speed;
	return velocity_out;
}
