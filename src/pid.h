/*
 * pid.h
 *
 *  Created on: Jul 26, 2013
 *      Author: Renee
 */
#ifndef PID_H_
#define PID_H_




//Speed PID Configuration
#define PIDS_P_GAIN 0.032f
#define PIDS_I_GAIN 100.0f
#define PIDS_D_GAIN 120.0f
#define PIDS_EMA_SAMPLES 1.0f
#define PIDS_I_LIMIT  25.0 //Ilimit is before process gain

//Pitch PID Configuration
#define PIDP_P_GAIN 5.0f
#define PIDP_I_GAIN 160.0f
#define PIDP_D_GAIN 30.0f
#define PIDP_EMA_SAMPLES 1.0f
#define PIDP_I_LIMIT  5.0 //Ilimit is before process gain

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
