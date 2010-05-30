#ifndef KALMAN_INCLUDED
#define KALMAN_INCLUDED
#include <math.h>
#include "uart.h"
#include "adc.h"

//void setKVar(char *str, char *subStr);
//double getKVar(char *str);
double getAng(void);
double getRate(void);
void doKalman(void);
void startKalman(void);
char kalmanRunning(void);
//void kalman_print(void);
//void kalman_print_headers(void);
double getPVar(char *str);
void setPVar(char *str, char *subStr);
int anglePID(void);
typedef struct
{
	double iMax, iMin;		// Maximum and minimum allowable integrator state

	double 	dState,			// Last position input
					iState;			// Integrator state
	double 	iGain,    		// integral gain
		   			pGain,    		// proportional gain
           			dGain;   	  	// derivative gain
} PID_T;

double updatePID(double error);
#endif
