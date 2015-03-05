/*
 * pid.h
 *
 *  Created on: Jul 26, 2013
 *      Author: Renee
 */
#ifndef PID_H_
#define PID_H_

//Speed PID Configuration
#define PIDS_P_GAIN 0.02f
#define PIDS_I_GAIN 800.0f
#define PIDS_D_GAIN 340.0f
#define PIDS_EMA_SAMPLES 10.0f
#define PIDS_I_LIMIT  450.0 //Ilimit is before process gain

//Pitch PID Configuration
#define PIDP_P_GAIN 6.5f
#define PIDP_I_GAIN 600.0f
#define PIDP_D_GAIN 30.0f
#define PIDP_EMA_SAMPLES 2.0f
#define PIDP_I_LIMIT  10.0 //Ilimit is before process gain

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
