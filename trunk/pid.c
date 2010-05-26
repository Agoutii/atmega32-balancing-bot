#include "pid.h"

double UpdatePID(PID_T *pid, signed double error)
{
	double pTerm, dTerm, iTerm;
	double output;
	// Calculate the proportional term
	pTerm = pid->pGain*error;


	// Calculate the derivative term
	dTerm = pid->dGain*(error - pid->dState);
	pid->dState = error;


	pid->iState += error;

	// Integral anti-windup
	if(pid->iState > pid->iMax)
		pid->iState = pid->iMax;
	else if(pid->iState < pid->iMin)
		pid->iState = pid->iMin;

	// Calculate the integral term
	iTerm = pid->iGain*pid->iState;
	
	// Trial output
	output = pTerm + iTerm + dTerm;
	
	// Integral anti-windup
	//if (output > pid->iMax)
	//	output = pid->iMax;
	//else if (output < pid->iMin)
	//	output = pid->iMin;
	//else
		//pid->iState += error;
	
	return output;
}
