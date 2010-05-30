#include "kalman.h"
const double T = 1.0/112.5;
double y_angle, y_gyro; 
double R_angle = 10d, R_gyro = 0.0005;
double x_angle = 0, x_rate = 0, x_bias = 38;
double Q_angle = 0.0001, Q_rate = 0.05, Q_bias = 0.00001;

//gyro_data = 0.83mV/deg/sec=0.0145 mV/rad/sec @ 3V 
// supply voltage is 3.13V... datasheet for IMU board says 3.3V supply
// however the datasheets for all 3 chips specify 3.0V nominal
// Sensitivity is not ratiometric to supply voltage
// ADC range 0-1024 = 0-3130mV
// 1 ADC div approx 3.0566mV
// 1 ADC div approx 0.04654 rad/sec +- ratiometric differences...
double p_gyro = .5*0.04654;
double p_rate = 0.005;
double P[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
double K[3];
double y_err, K_denom;
double r = 0;


double p_ang = 3400;
double d_ang = 70;
double i_ang = 0;
double pTerm, dTerm, iTerm;
double pGain=0, iGain=1, dGain=0;
double iMax = 20, iMin = -20;
double iState, pState, dState;
double iAngle = 0;
double pos_ref=0;

double updatePID(double error) //velocity control
{
	double output;
	// Calculate the proportional term
	pTerm = pGain*error;


	// Calculate the derivative term
	//dTerm = dGain*(error - dState);
	//dState = error;


	iState += error;

	// Integral anti-windup
	if(iState > iMax)
		iState = iMax;
	else if(iState < iMin)
		iState = iMin;

	// Calculate the integral term
	iTerm = iGain*iState;
	
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

void setPVar(char *str, char *subStr)
{
	if(!strncmp_P(str,PSTR("p"),1))
	{
		p_ang=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("d"),1))
	{
		d_ang=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("r"),1))
	{
		r=atof(subStr+1);
	}	
	else if(!strncmp_P(str,PSTR("i"),1))
	{
		i_ang=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("q_bias"),6))
	{
		Q_bias=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("q_rate"),6))
	{
		Q_rate=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("q_angle"),7))
	{
		Q_angle=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("r_angle"),7))
	{
		R_angle=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("r_gyro"),6))
	{
		R_gyro=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("p_gyro"),6))
	{
		p_gyro=atof(subStr+1);
	}
}

double getPVar(char *str)
{
	if(!strncmp_P(str,PSTR("p"),1))
	{
		return p_ang;
	}
	else if(!strncmp_P(str,PSTR("d"),1))
	{
		return d_ang;
	}
	else if(!strncmp_P(str,PSTR("r"),1))
	{
		return r;
	}	
	else if(!strncmp_P(str,PSTR("i"),1))
	{
		return i_ang;
	}
	else if(!strncmp_P(str,PSTR("q_bias"),6))
	{
		return Q_bias;
	}
	else if(!strncmp_P(str,PSTR("x_rate"),6))
	{
		return x_rate;
	}
	else if(!strncmp_P(str,PSTR("x_angle"),7))
	{
		return x_angle;
	}
	else if(!strncmp_P(str,PSTR("q_rate"),6))
	{
		return Q_rate;
	}
	else if(!strncmp_P(str,PSTR("q_angle"),7))
	{
		return Q_angle;
	}
	else if(!strncmp_P(str,PSTR("r_angle"),7))
	{
		return R_angle;
	}
	else if(!strncmp_P(str,PSTR("r_gyro"),6))
	{
		return R_gyro;
	}
	else if(!strncmp_P(str,PSTR("p_gyro"),6))
	{
		return p_gyro;
	}	
	else if(!strncmp_P(str,PSTR("accel_y"),6))
	{
		return adc_fetch(1);
	}	
	else if(!strncmp_P(str,PSTR("accel_z"),6))
	{
		return adc_fetch(2);
	}
	return 0;
}

int anglePID(void)
{
	int output;
//	iAngle += x_angle*i_ang;
//	if(iAngle > 100)
//		iAngle = 100;
//	else if(iAngle<-100)
//		iAngle = -100;

	output = (int)(x_angle*p_ang+ x_rate*d_ang);// -iAngle);
	if(output>255)
		output = 255;
	else if(output<-255)
		output = -255;

	return output;
}

double getAng(void)
{
	return x_angle;
}
double getRate(void)
{
	return x_rate;
}

/*void kalman_print_headers(void)
{
	printf_P(PSTR("x_angle,x_rate,y_angle,y_gyro_c,x_bias,R_angle,R_gyro,Q_angle,Q_rate,Q_bias\n"));
	printf_P(PSTR("%G,%G,%G,%G,%G,%G,%G,%G,%G,%G\n"),x_angle,x_rate,y_angle,y_gyro-x_bias,x_bias,R_angle,R_gyro,Q_angle,Q_rate,Q_bias);

}


void setKVar(char *str, char *subStr)
{
	if(!strncmp_P(str,PSTR("q_bias"),6))
	{
		Q_bias=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("q_rate"),6))
	{
		Q_rate=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("q_angle"),7))
	{
		Q_angle=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("r_angle"),7))
	{
		R_angle=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("r_gyro"),6))
	{
		R_gyro=atof(subStr+1);
	}
	else if(!strncmp_P(str,PSTR("p_gyro"),6))
	{
		p_gyro=atof(subStr+1);
	}
}



double  getKVar(char *str)
{
	if(!strncmp_P(str,PSTR("q_bias"),6))
	{
		return Q_bias;
	}
	else if(!strncmp_P(str,PSTR("x_rate"),6))
	{
		return x_rate;
	}
	else if(!strncmp_P(str,PSTR("x_angle"),7))
	{
		return x_angle;
	}
	else if(!strncmp_P(str,PSTR("q_rate"),6))
	{
		return Q_rate;
	}
	else if(!strncmp_P(str,PSTR("q_angle"),7))
	{
		return Q_angle;
	}
	else if(!strncmp_P(str,PSTR("r_angle"),7))
	{
		return R_angle;
	}
	else if(!strncmp_P(str,PSTR("r_gyro"),6))
	{
		return R_gyro;
	}
	else if(!strncmp_P(str,PSTR("p_gyro"),6))
	{
		return p_gyro;
	}	
	else if(!strncmp_P(str,PSTR("accel_y"),6))
	{
		return adc_fetch(1);
	}	
	else if(!strncmp_P(str,PSTR("accel_z"),6))
	{
		return adc_fetch(2);
	}
	return 0;
}

void kalman_print(void)
{
	printf_P(PSTR("%G,%G,%G,%G,%G,%G\n"),x_angle,x_rate,y_angle,y_gyro-x_bias,0.005*((adc_fetch(2)-543)- 100*sin(x_angle)),x_bias);
}
 */

void doKalman(void)
{
	int gyro_data = adc_fetch(4); 
	int acc_y = adc_fetch(1);
	int acc_z = adc_fetch(2);
//	printf_P(PSTR("%i,%i\n"),acc_y,acc_z);

	y_gyro = p_gyro*(double)(gyro_data);
	//xt = 9.81*10.0/1024.0*(double)(acc_Z - 0x1FF);
	//yt = 9.81*10.0/1024.0*(double)(acc_Y - 0x1FF);
	y_angle = atan2((double)(acc_z - 543),-(double)(512-acc_y))+r;
	/*
	* Sequential Kalman Filter
	*/
	// Covariance prediction
	// P = A*P*A' + Q
	P[0][0] += (P[0][1] + P[1][0] + P[1][1]*T)*T + Q_angle;
	P[1][1] += Q_rate;
	P[2][2] += Q_bias;
	P[0][1] += P[1][1]*T;
	P[0][2] += P[1][2]*T;
	P[1][0] += P[1][1]*T;
	P[2][0] += P[2][1]*T;
	// State prediction
	// x = A*x
	// 1 ADC LSB approx 0.005 rad/sec^2
	// 100 ADC counts approx 1g on accel
//	x_rate -= p_rate*((acc_z-543) - 100*sin(y_angle));

	x_angle += x_rate*T; // - 0.005*((acc_z-543) - 100*sin(y_angle));
//	x_angle -= 0.005*((adc_fetch(2)-543)- 100*sin(x_angle));
	// Measurement update due to accelerometer data
	// Kalman gain
	K_denom = P[0][0] + R_angle;
	K[0] = P[0][0] / K_denom;
	K[1] = P[1][0] / K_denom;
	K[2] = P[2][0] / K_denom;
	// State correction
	y_err = y_angle - x_angle;
	x_angle += K[0]*y_err;
	x_rate += K[1]*y_err;
	x_bias += K[2]*y_err;
	// Covariance update
	// P = (I - K*C)*P
	P[0][0] -= K[0]*P[0][0];
	P[0][1] -= K[0]*P[0][1];
	P[0][2] -= K[0]*P[0][2];
	P[1][0] -= K[1]*P[0][0];
	P[1][1] -= K[1]*P[0][1];
	P[1][2] -= K[1]*P[0][2];
	P[2][0] -= K[2]*P[0][0];
	P[2][1] -= K[2]*P[0][1];
	P[2][2] -= K[2]*P[0][2];
	// Measurement update due to gyro data
	// Kalman gain
	K_denom = P[1][1] + P[1][2] + P[2][1] + P[2][2] + R_gyro;
	K[0] = (P[0][1] + P[0][2]) / K_denom;
	K[1] = (P[1][1] + P[1][2]) / K_denom;
	K[2] = (P[2][1] + P[2][2]) / K_denom;
	// State correction
	y_err = y_gyro - x_rate - x_bias;
	x_angle += K[0]*y_err;
	x_rate += K[1]*y_err;
	x_bias += K[2]*y_err;
	// Covariance update
	// P = (I - K*C)*P
	P[0][0] -= K[0]*(P[1][0] + P[2][0]);
	P[0][1] -= K[0]*(P[1][1] + P[2][1]);
	P[0][2] -= K[0]*(P[1][2] + P[2][2]);
	P[1][0] -= K[1]*(P[1][0] + P[2][0]);
	P[1][1] -= K[1]*(P[1][1] + P[2][1]);
	P[1][2] -= K[1]*(P[1][2] + P[2][2]);
	P[2][0] -= K[2]*(P[1][0] + P[2][0]);
	P[2][1] -= K[2]*(P[1][1] + P[2][1]);
	P[2][2] -= K[2]*(P[1][2] + P[2][2]);

	int u = anglePID();;
	if(u>255)
		u=255;
	else if(u<-255)
		u= -255;

	printf_P(PSTR("%G, %G,%G, %i\n"),y_angle,x_angle,x_rate, u);
//	adc_print();
	if(u>0)
	{
		PORTC |= (1<<PORTC1);
		PORTC |= (1<<PORTC0);
		OCR0 = u;
		OCR2 = u;
	}
	else
	{
		PORTC &= ~(1<<PORTC0);
		PORTC &= ~(1<<PORTC1);
		OCR0 = -1*u;
		OCR2 = -1*u;
	}
}


