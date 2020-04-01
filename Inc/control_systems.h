
#ifndef CONTROL_SYSTEMS_H_
#define CONTROL_SYSTEMS_H_

typedef struct
{
	float kp;
	float ki;
	float target;
	float current_integral_value;
	float upper_boundary;
	float lower_boundary;
}pi_regulator;

typedef struct
{
	float kp;
	float target;
	float upper_boundary;
	float lower_boundary;
}p_regulator;

typedef struct
{
	float kp;
}pid_regulator;

typedef struct
{
	float kp;
}pd_regulator;



#endif /* CONTROL_SYSTEMS_H_ */
