#include "pid.h"

void pid_increment_type(pid_typedef *pid,float desired,float actual)//pid参数，期望值，实际值
{
	pid->dx = desired - actual;
	pid->P=(pid->dx-pid->previous_dx)*pid->kp;
	pid->I=pid->dx*pid->ki;
	pid->D=(pid->dx-2.0f*pid->previous_dx+pid->pre_previous_dx)*pid->kd;
	pid->output = pid->P+pid->I+pid->D;
	pid->pre_previous_dx = pid->previous_dx;
	pid->previous_dx = pid->dx;
}

void pid_deinit(pid_typedef *pid)
{
	pid->previous_dx=0;
	pid->pre_previous_dx=0;
}

void pid_init(pid_typedef *pid,float kp,float ki,float kd)
{
	pid->kp=kp;
	pid->ki=ki;
	pid->kd=kd;
}
