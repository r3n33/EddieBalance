/*
 * pid.h
 *
 *  Created on: Jul 26, 2013
 *      Author: Renee
 */
#ifndef PID_H_
#define PID_H_

//For I Term Saturation
#define PID_I_LIMIT  15.0 //Ilimit is before process gain

//p gain oscillation measured at 28
//Ku measured at 133ms
//Classic PID 16.8, 252, 1.9
//Pessen Integral PID 19.6, 368.4, 0.39 ** works on carpet

//p gain oscillation measured at 35.0 and balanced on carpet with p only
//Ku measured at 56ms
//Pessen Integral PID 24.5 1093.0 0.2

//My best PID 27.15 1100.0 0.16 3ema 20limit
//My new best 20.15 25 1 4ema 100 limit
// 21.15 5 0.8 5ema
//Last night's final values 17.15 5.9 0.8 5.0ema 10000limit
 
//Pitch PID Configuration
#define PIDP_P_GAIN 14.0f //13.7f
#define PIDP_I_GAIN 7.0f //10.1f //>10.9
#define PIDP_D_GAIN .675f
#define PIDP_EMA_SAMPLES 2.0f

typedef struct
{
	float *processGain;
	float *integralTime;
	float *derivateTime;

	double error;
	double accumulatedError;
	double differentialError;
	double lastFeedbackReading;

	float *iLimit;

	float *EMAnumberSamples; //Determines the EMAalpha;
} PID_t;

double PIDUpdate( double setpoint, double actual_position, double dTmilliseconds, PID_t* pPID);
void PIDinit(PID_t * pid, float *pgain, float *igain, float *dgain, float *ilimit, float *numsamples);

#endif /*PID_H_*/
