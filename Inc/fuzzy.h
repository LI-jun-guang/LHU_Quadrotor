#include "stm32f4xx_hal.h"

typedef struct
{
	float setpoint;
	float kp;
	float ki;
	float kd;	
	float lasterror;
	float preerror;
	float deadband;
	float output;
	float result;
	float maximum;
	float minimum;
	
	float maxdKp;
	float maxdKi;
	float maxdKd;
	
	float mindKp;
	float mindKi;
	float mindKd;
	
	float qKp;
	float qKi;
	float qKd;

}FUZZYPID;

static void LinearQuantization(FUZZYPID * vPID,float pv,float *qValue);
static void CalcMembership(float *ms,float qv,int * index);

void FuzzyComputat(FUZZYPID * vPID,float pv,float *deltaK);



