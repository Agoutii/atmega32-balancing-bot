#ifndef PID_H
#define PID_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
#include <math.h>

#include "kalman.h"
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
