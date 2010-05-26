#ifndef PID_H
#define PID_H

typedef struct
{
	double iMax, iMin;		// Maximum and minimum allowable integrator state

	double 	dState,			// Last position input
					iState;			// Integrator state
	double 	iGain,    		// integral gain
		   			pGain,    		// proportional gain
           			dGain;   	  	// derivative gain
} PID_T;

double UpdatePID(PID_T *pid, double error);

#endif
