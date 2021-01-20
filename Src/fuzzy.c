#include "fuzzy.h"
#include "stm32f4xx_hal.h"
#include "LED.h"
#include "PID.h"
#include "Math.h"
#include "eeprom.h"
#include "MS5611.h"
#include "Kalman.h"
#include "Battery.h"
#include "MPU9250.h"
#include "attitude.h"
#include "arm_math.h"
#include "vl53l1_api.h"
#include "math_helper.h"
#define PB  3 
#define PM  2
#define PS  1
#define ZO  0
#define NS -1
#define NM -2
#define NB -3

 /**/
 static const float ruleKp[7][7]={
	 PB, PB, PM, PM, PS, ZO, ZO,
	 PB, PB, PM, PS, PS, ZO, NS,
	 PM, PM, PM, PS, ZO, NS, NS,
	 PM, PM, PS, ZO, NS, NM, NM,
	 PS, PS, ZO, NS, NS, NM, NM,
	 PS, ZO, NS, NM, NM, NM, NB,
	 ZO, ZO, NM, NM, NM, NB, NB
 };
 
 /**/
 static const float ruleKi[7][7]={
	 NB, NB, NM, NM, NS, ZO, ZO,
	 NB, NB, NM, NS, NS, ZO, ZO,
	 NB, NM, NS, NS, ZO, PS, PS,
	 NM, NM, NS, ZO, PS, PM, PM,
	 NM, NS, ZO, PS, PS, PM, PB,
	 ZO, ZO, PS, PS, PM, PB, PB,
	 ZO, ZO, PS, PM, PM, PB, PB
 };
 
 /*OK*/
  static const float ruleKd[7][7]={
	 PS, NS, NB, NB, NB, NM, PS,
	 PS, NS, NB, NM, NM, NS, ZO,
	 ZO, NS, NM, NM, NS, NS, ZO,
	 ZO, NS, NS, NS, NS, NS, ZO,
	 ZO, ZO, ZO, ZO, ZO, ZO, ZO,
	 PB, NS, PS, PS, PS, PS, PB,
	 PB, PM, PM, PM, PS, PS, PB
 };
	

 static void LinearQuantization(FUZZYPID * vPID,float pv,float *qValue)
 {
	 float thisError;
	 float deltaError;
	 
	 thisError = vPID->setpoint-pv;
	 deltaError = thisError-vPID->lasterror;
	 
	 qValue[0]=6.0*thisError/(vPID->maximum-vPID->minimum);
	 qValue[1]=3.0*deltaError/(vPID->maximum-vPID->minimum);
 }
 
 static void CalcMembership(float *ms,float qv,int * index)
 {
	if((qv>=-NB)&&(qv<-NM))
  {
    index[0]=0;
    index[1]=1;
    ms[0]=-0.5*qv-2.0;  //y=-0.5x-2.0
    ms[1]=0.5*qv+3.0;   //y=0.5x+3.0
  }
	else if((qv>=-NM)&&(qv<-NS))
  {
    index[0]=1;
    index[1]=2;
    ms[0]=-0.5*qv-1.0;  //y=-0.5x-1.0
    ms[1]=0.5*qv+2.0;   //y=0.5x+2.0
	}
	else if((qv>=-NS)&&(qv<ZO))
  {
    index[0]=2;
    index[1]=3;
    ms[0]=-0.5*qv;      //y=-0.5x
    ms[1]=0.5*qv+1.0;   //y=0.5x+1.0
	}
	else if((qv>=ZO)&&(qv<PS))
  {
    index[0]=3;
    index[1]=4;
    ms[0]=-0.5*qv+1.0;  //y=-0.5x+1.0
    ms[1]=0.5*qv;       //y=0.5x
	}
	else if((qv>=PS)&&(qv<PM))
  {
    index[0]=4;
    index[1]=5;
    ms[0]=-0.5*qv+2.0;  //y=-0.5x+2.0
    ms[1]=0.5*qv-1.0;   //y=0.5x-1.0
	}
	else if((qv>=PM)&&(qv<=PB))
  {
    index[0]=5;
    index[1]=6;
    ms[0]=-0.5*qv+3.0;  //y=-0.5x+3.0
    ms[1]=0.5*qv-2.0;   //y=0.5x-2.0
	}
 }
	
 
 
void FuzzyComputat(FUZZYPID * vPID,float pv,float *deltaK)
 {
	 
	float qValue[2]={0,0};  //偏差及增量的量化值
	int indexE[2]={0,0};		//偏差隸屬度索引
	float msE[2]={0,0};			//偏差隸屬度
	int indexEC[2]={0,0};		//偏差增量隸屬度索引
	float msEC[2]={0,0};		//偏差增量隸屬度
	float qValueK[3];       //KP,KI,KD
	/**/
	LinearQuantization(vPID,pv,qValue);
	/*計算隸屬度*/
	CalcMembership(msE,qValue[0],indexE);
  CalcMembership(msEC,qValue[1],indexEC);

	qValueK[0]=msE[0]*(msEC[0]*ruleKp[indexE[0]][indexEC[0]]+msEC[1]*ruleKp[indexE[0]][indexEC[1]])+msE[1]*(msEC[0]*ruleKp[indexE[1]][indexEC[0]]+msEC[1]*ruleKp[indexE[1]][indexEC[1]]);
	qValueK[1]=msE[0]*(msEC[0]*ruleKi[indexE[0]][indexEC[0]]+msEC[1]*ruleKi[indexE[0]][indexEC[1]])+msE[1]*(msEC[0]*ruleKi[indexE[1]][indexEC[0]]+msEC[1]*ruleKi[indexE[1]][indexEC[1]]);
	qValueK[2]=msE[0]*(msEC[0]*ruleKd[indexE[0]][indexEC[0]]+msEC[1]*ruleKd[indexE[0]][indexEC[1]])+msE[1]*(msEC[0]*ruleKd[indexE[1]][indexEC[0]]+msEC[1]*ruleKd[indexE[1]][indexEC[1]]);

 }
	
	
	 