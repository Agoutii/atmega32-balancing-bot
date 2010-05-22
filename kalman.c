#include "kalman.h"
const double T = 1.0/112.5;
double y_angle, y_gyro;
double R_angle = 1.0, R_gyro = 0.0001;
double x_angle = 0, x_rate = 0, x_bias = -11;
double Q_angle = 0.001, Q_rate = 0.005, Q_bias = 0.0001;
double P[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
double K[3];
double y_err, K_denom;

void setVar(char *str, char *subStr)
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
}

double  getVar(char *str)
{
	if(!strncmp_P(str,PSTR("q_bias"),6))
	{
		return Q_bias;
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
	return 0;
}

void doKalman(void)
{
	int gyro_data = adc_fetch(4);
	int acc_y = adc_fetch(1);
	int acc_z = adc_fetch(2);

	y_gyro = -3300.0*3.14159265359/(2.0*1024.0*180.0)*(double)(gyro_data);
	//xt = 9.81*10.0/1024.0*(double)(acc_Z - 0x1FF);
	//yt = 9.81*10.0/1024.0*(double)(acc_Y - 0x1FF);
	y_angle = atan2((double)(acc_z - 540),-(double)(511-acc_y));
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
	x_angle += x_rate*T;
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
}

void kalman_print(void)
{
	printf_P(PSTR("%G,%G,%G,%G,%G\n"),x_angle,x_rate,x_bias,y_angle,y_gyro);
}
