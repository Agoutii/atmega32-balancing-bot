#ifndef KALMAN_INCLUDED
#define KALMAN_INCLUDED
#include <math.h>
#include "uart.h"
#include "adc.h"

void setVar(char *str, char *subStr);
double getVar(char *str);

void doKalman(void);
void kalman_print(void);
#endif
