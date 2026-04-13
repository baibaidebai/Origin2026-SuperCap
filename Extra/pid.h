#include "main.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	float dx;
	float previous_dx;
	float pre_previous_dx;
	float output;
	float P;
	float I;
	float D;
}pid_typedef;

void pid_init(pid_typedef *pid,float kp,float ki,float kd);
void pid_deinit(pid_typedef *pid);
float pid_position_type(pid_typedef *pid,float desired,float actual);//位置式pid
void pid_increment_type(pid_typedef *pid,float desired,float actual);//增量式pid

