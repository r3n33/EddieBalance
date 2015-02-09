/*
 * pid.h
 *
 *  Created on: Jul 26, 2013
 *      Author: Renee
 */
#ifndef PID_H_
#define PID_H_

//For I Term Saturation
#define PID_I_LIMIT  10.0 //Ilimit is before process gain

//Setpoint PID Configuration
#define PIDS_P_GAIN 0.4f //0.005f
#define PIDS_I_GAIN 100.0f //800.0f
#define PIDS_D_GAIN 10.0f //0.6f
#define PIDS_EMA_SAMPLES 2.0f

//Pitch PID Configuration
#define PIDP_P_GAIN 15.0f
#define PIDP_I_GAIN 400.0f //120.0f
#define PIDP_D_GAIN 3.0f //0.6f
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
