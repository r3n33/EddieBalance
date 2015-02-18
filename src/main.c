/* 
 Eddie the balance bot. Copyright (C) 2015 Renee Glinski. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file LICENSE included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
*/

//TODO: Remove network reset at startup when Intel fixes the confirmed bug in GPIO
//TODO: Allow for multiple Eddies on the same network
//TODO: If not connected to WiFi start a hotspot for ones self
//TODO: Add command line arguments for switching console/udp output... maybe camera etc..

#include "imu/imu.h"
#include "motordriver/MotorDriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include "mraa.h"
#include "pid.h"
#include <stdarg.h> //print function
#include "Kalman.h"
#include "encoder.h"
#include "udp.h"

#include <sys/time.h>

double last_gy_ms;
double last_PID_ms;
double current_milliseconds() 
{
    struct timeval c_time; 
    gettimeofday(&c_time, NULL);
    double milliseconds = c_time.tv_sec*1000 + c_time.tv_usec/1000;
    return milliseconds;
}

//#define DISABLE_MOTORS

enum
{
	CONSOLE=0,
	UDP //Will send all print() calls to UDP port 4243, all printf() will still go to console (:)
};
int outputto = UDP; //Change to fit current need.

int Running = 1;
int inFalloverState = 0; //Used to flag when Eddie has fallen over and disables motors
int inRunAwayState = 0;
int inSteadyState = 0; //Used to flag how long Eddie is being held upright in a stead state and will enable motors

PID_t pitchPID; //PID Controller for pitch angle
float pitchPIDoutput = 0;
float pidP_P_GAIN,pidP_I_GAIN,pidP_D_GAIN,pidP_I_LIMIT,pidP_EMA_SAMPLES;

PID_t speedPID; //PID Controller for wheel speed
float speedPIDoutput = 0;
float pidS_P_GAIN,pidS_I_GAIN,pidS_D_GAIN,pidS_I_LIMIT,pidS_EMA_SAMPLES;

enum 
{
	DRIVE_IDLE = 0,
	DRIVE_FORWARD,
	DRIVE_REVERSE
};
int currentDriveMode = DRIVE_IDLE;

enum
{
	TURN_IDLE = 0,
	TURN_RIGHT,
	TURN_LEFT
};
int currentTurnMode = TURN_IDLE;

float filteredPitch;
float filteredRoll;

float driveTrim = 0;
float turnTrim = 0;

int print(const char *format, ...)
{
	static char buffer[2048];
	
	va_list arg;
	int len;

	va_start(arg, format);
	len = vsprintf(buffer, format, arg);
	va_end(arg);

	if( len <= 0 ) return len;
	
	switch( outputto )
	{
		case CONSOLE:
			printf("%s",buffer);
		break;
		case UDP:
			UDPSend(buffer,len);
		break;
	}

   return len;
}

// Define the exit signal handler
void signal_callback_handler(int signum)
{
	print("Exiting program; Caught signal %d\r\n",signum);
	Running = 0;
}

/* Current UDP Commands:
 *
 * DRIVE[value]	=	+ is Forwards, - is Reverse, 0.0 is IDLE
 * TURN[value]	=	+ is Right, - is Left, 0.0 is STRAIGHT
 *
 * SETPIDS = Changes all PIDs for speed and pitch controllers
 * GETPIDS = Returns all PIDs for speed and pitch controllers via UDP
 *
 * PIDP[P,I,D][value] = adjust pitch PIDs
 * PISS[P,I,D][value] = adjust speed PIDs
 * KALQ[value] = adjust Kalman Q Bias
 * KALR[value] = adjust Kalman R Measure
 *
 * STOPUDP	= Will stop Eddie from sending UDP to current recipient
 *
 */
void UDP_Data_Handler( char * p_udpin )
{
	/* DRIVE commands */
	if( !memcmp( p_udpin, "DRIVE", 5 ) )
	{
		float driveSpeed = atof( &p_udpin[5] );
		if ( driveSpeed > 0.0 )
		{
			driveTrim = driveSpeed;
			currentDriveMode = DRIVE_FORWARD;
			//print( "Drive Mode: DRIVE FORWARD command at %0.2f rate\r\n", driveTrim );
		}
		else if ( driveSpeed < 0.0 )
		{
			driveTrim = driveSpeed;
			currentDriveMode = DRIVE_REVERSE;
			//print( "Drive Mode: DRIVE REVERSE command at %0.2f rate\r\n", driveTrim );
		}
		else
		{
			currentDriveMode = DRIVE_IDLE;
			//print( "Drive Mode: DRIVE 0 (IDLE) command received\r\n" );
		}
	}
	/* TURN commands */
	else if( !memcmp( p_udpin, "TURN", 4 ) )
	{
		float turnSpeed = atof( &p_udpin[4] );
		if ( turnSpeed > 0.0 )
		{
			turnTrim = turnSpeed;
			currentTurnMode = TURN_RIGHT;
			//print( "Turn Mode: TURN RIGHT command at %0.2f speed received\r\n", driveRightTrim );
		}
		else if ( turnSpeed < 0.0 )
		{
			turnTrim = turnSpeed;
			currentTurnMode = TURN_LEFT;
			//print( "Turn Mode: TURN LEFT command at %0.2f speed received\r\n", driveLeftTrim );
		}
		else
		{
			currentTurnMode = TURN_IDLE;
			//print( "Turn Mode: TURN 0 (STRAIGHT) command received\r\n" );
		}
	}
	/* Get/Set all PID quick commands*/
	else if ( strncmp( p_udpin, "SETPIDS:", 8 ) == 0 )
	{
		char * pch;
		
		pch = strtok ( &p_udpin[8], "," );
		pidP_P_GAIN=atof(pch);
		
		pch = strtok (NULL, ",");
		pidP_I_GAIN=atof(pch);
		
		pch = strtok (NULL, ",");
		pidP_D_GAIN=atof(pch);
		
		pch = strtok (NULL, ",");
		pidS_P_GAIN=atof(pch);
		
		pch = strtok (NULL, ",");
		pidS_I_GAIN=atof(pch);
		
		pch = strtok (NULL, ",");
		pidS_D_GAIN=atof(pch);		
	}
	else if ( strncmp( p_udpin, "GETPIDS", 7 ) == 0 )
	{
		print( "CURRENTPIDS:%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", pidP_P_GAIN, pidP_I_GAIN, pidP_D_GAIN, pidS_P_GAIN, pidS_I_GAIN, pidS_D_GAIN );
	}
	/* Individual Pitch PID Controller commands */
	else if ( strncmp( p_udpin, "PPIDP", 5 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[5] );
		print( "New Pitch PID P Gain Received: Changing %0.3f to %0.3f\r\n", pidP_P_GAIN, newGain );
		pidP_P_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "PPIDI", 5 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[5] );
		print( "New Pitch PID I Gain Received: Changing %0.3f to %0.3f\r\n", pidP_I_GAIN, newGain );
		pidP_I_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "PPIDD", 5 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[5] );
		print( "New Pitch PID D Gain Received: Changing %0.3f to %0.3f\r\n", pidP_D_GAIN, newGain );
		pidP_D_GAIN = newGain;
	}
	/* Individual Speed PID Controller commands*/
	else if ( strncmp( p_udpin, "SPIDP", 5 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[5] );
		print( "New Speed PID P Gain Received: Changing %0.3f to %0.3f\r\n", pidS_P_GAIN, newGain );
		pidS_P_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "SPIDI", 5 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[5] );
		print( "New Speed PID I Gain Received: Changing %0.3f to %0.3f\r\n", pidS_I_GAIN, newGain );
		pidS_I_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "SPIDD", 5 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[5] );
		print( "New Speed PID D Gain Received: Changing %0.3f to %0.3f\r\n", pidS_D_GAIN, newGain );
		pidS_D_GAIN = newGain;
	}
	/* Pitch Kalman filter tuning commands */
	else if ( strncmp( p_udpin, "KALQA", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "Setting Kalman Q Angle to: %0.4f\r\n", newGain );
		setQkalmanangle( newGain );
	}
	else if ( strncmp( p_udpin, "KALQB", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "Setting Kalman Q Bias to: %0.4f\r\n", newGain );
		setQbias( newGain );
	}
	else if ( strncmp( p_udpin, "KALR", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "Setting Kalman R Measure to: %0.4f\r\n", newGain );
		setRmeasure( newGain );
	}
	/* UDP Hangup command */
	else if ( strncmp( p_udpin, "STOPUDP", 7 ) == 0 )
	{
		bsock=-1;
	}
}

int main(int argc, char **argv)
{
	//Register signal and signal handler
	signal(SIGINT, signal_callback_handler);
	
	//Init UDP with callback and pointer to run status
	initUDP( &UDP_Data_Handler, &Running );
	
	print("Eddie starting...\r\n");

	double EncoderAverage;
	double EncoderPos[2] = {0};
	
	initEncoders( 183, 46, 45, 44 );
	print("Encoders activated.\r\n");

	imuinit();
	print("IMU Started.\r\n");

	float kalmanAngle;
	InitKalman();
	
#ifndef DISABLE_MOTORS
	print( "Starting motor driver (and resetting wireless) please be patient..\r\n" );
	if ( motor_driver_enable() < 1 )
		{
			print("Startup Failed; Error starting motor driver.\r\n");
			motor_driver_disable();
			return -1;
		}
	print("Motor Driver Started.\r\n");
#endif
	
	print("Eddie is starting the UDP network thread..\r\n");
	pthread_create( &udplistenerThread, NULL, &udplistener_Thread, NULL );
	
	print( "Eddie is Starting PID controller\r\n" );
	/*Set default PID values and init pitchPID controller*/
	pidP_P_GAIN = PIDP_P_GAIN;	pidP_I_GAIN = PIDP_I_GAIN;	pidP_D_GAIN = PIDP_D_GAIN;	pidP_I_LIMIT = PIDP_I_LIMIT; pidP_EMA_SAMPLES = PIDP_EMA_SAMPLES;
	PIDinit( &pitchPID, &pidP_P_GAIN, &pidP_I_GAIN, &pidP_D_GAIN, &pidP_I_LIMIT, &pidP_EMA_SAMPLES );
	
	/*Set default values and init speedPID controller*/
	pidS_P_GAIN = PIDS_P_GAIN;	pidS_I_GAIN = PIDS_I_GAIN;	pidS_D_GAIN = PIDS_D_GAIN;	pidS_I_LIMIT = PIDS_I_LIMIT; pidS_EMA_SAMPLES = PIDS_EMA_SAMPLES;
	PIDinit( &speedPID, &pidS_P_GAIN, &pidS_I_GAIN, &pidS_D_GAIN, &pidS_I_LIMIT, &pidS_EMA_SAMPLES );
	
	//Get estimate of starting angle and specify complementary filter and kalman filter start angles
	getOrientation();
	kalmanAngle = filteredPitch = i2cPitch;
	setkalmanangle( filteredPitch );
	filteredRoll = i2cRoll;
	
	print( "Eddie startup complete. Hold me upright to begin\r\n" );
	
	float gy_scale = 0.01;
	last_PID_ms = last_gy_ms = current_milliseconds();

	float test_smoothDriveTrim;
	
	while(Running)
	{
		EncoderAverage = GetEncoder();
		
		if( fabs(EncoderAverage) > 1000 && !inRunAwayState )
		{
			print( "Help! I'm running and not moving.\r\n");
			ResetEncoders();
			inRunAwayState=1;
		}
		//printf( "Encoder L: %4lld R: %4lld\r\n", EncoderPos[0], EncoderPos[1] );
		
		/*Read IMU and calculate rough angle estimates*/
		getOrientation();
		
		/*Calculate time since last IMU reading and determine gyro scale (dt)*/
		gy_scale = (float)( current_milliseconds() - last_gy_ms ) / 1000.0f;
		//printf( "gy scale: %0.2f last_gy_ms: %d\r\n", gy_scale, current_milliseconds() - last_gy_ms );
		last_gy_ms = current_milliseconds();
		
		/*Complementary filters to smooth rough pitch and roll estimates*/
		filteredPitch = 0.98 * ( filteredPitch + ( gy * gy_scale ) ) + ( 0.02 * i2cPitch );
		filteredRoll = 0.98 * ( filteredRoll + ( gx * gy_scale ) ) + ( 0.02 * i2cRoll );
		
		/*Kalman filter for most accurate pitch estimates*/	
		kalmanAngle = -getkalmanangle(filteredPitch, gy, gy_scale /*dt*/);
		
		/* Monitor angles to determine if Eddie has fallen too far... or if Eddie has been returned upright*/
		if ( ( inRunAwayState || ( fabs( kalmanAngle ) > 50 || fabs( filteredRoll ) > 45 ) ) && !inFalloverState ) 
		{
#ifndef DISABLE_MOTORS
			motor_driver_standby(1);
#endif
			inFalloverState = 1;
			print( "Help! I've fallen over and I can't get up =)\r\n");
		} 
		else if ( fabs( kalmanAngle ) < 10 && inFalloverState && fabs( filteredRoll ) < 20 )
		{
			if ( ++inSteadyState == 100 )
			{
				inRunAwayState = 0;
				inSteadyState = 0;
#ifndef DISABLE_MOTORS
				motor_driver_standby(0);
#endif
				inFalloverState = 0;
				print( "Thank you!\r\n" );
			}
		}
		else
		{
			inSteadyState = 0;
		}

		if ( !inFalloverState )
		{
			/* Drive operations */
			test_smoothDriveTrim = ( 0.97 * test_smoothDriveTrim ) + ( 0.03 * driveTrim );
			switch( currentDriveMode )
			{
				case DRIVE_FORWARD:
					Move(test_smoothDriveTrim);
				break;
				case DRIVE_REVERSE:
					Move(test_smoothDriveTrim);
				break;
			}
			
			//Wheel Speed PIDs
			speedPIDoutput = PIDUpdate( 0, EncoderAverage, current_milliseconds() - last_PID_ms, &speedPID);
			//Pitch Angle PIDs
			pitchPIDoutput = PIDUpdate( speedPIDoutput, kalmanAngle, current_milliseconds() - last_PID_ms, &pitchPID);
			
			last_PID_ms = current_milliseconds();
			
			//Limit pitchPID output to 100% motor throttle
			if ( pitchPIDoutput > 100.0 ) pitchPIDoutput = 100.0;
			if ( pitchPIDoutput < -100.0 ) pitchPIDoutput = -100.0;
		}
		else //We are inFalloverState
		{
			ResetEncoders();
			pitchPID.accumulatedError = 0;
			speedPID.accumulatedError = 0;
		}
		
		/* Testing: Turn operations */	
		float testTurnTrim;
		switch( currentTurnMode )
		{
			case TURN_RIGHT:
				testTurnTrim = turnTrim;
			break;
			case TURN_LEFT:
				testTurnTrim = turnTrim;
			break;
			default:
				testTurnTrim = 0;
			break;				
		}
	
#ifndef DISABLE_MOTORS
		set_motor_speed_right( pitchPIDoutput - testTurnTrim );
		set_motor_speed_left( pitchPIDoutput + testTurnTrim );
#endif

		if ( !inFalloverState || outputto == UDP )
		{
			print( "PIDout: %0.2f,%0.2f\tcompPitch: %6.2f kalPitch: %6.2f\tPe: %0.3f\tIe: %0.3f\tDe: %0.3f\tPe: %0.3f\tIe: %0.3f\tDe: %0.3f\r\n",
				speedPIDoutput, 
				pitchPIDoutput, 
				filteredPitch, 
				kalmanAngle,
				pitchPID.error, 
				pitchPID.accumulatedError, 
				pitchPID.differentialError, 
				speedPID.error, 
				speedPID.accumulatedError, 
				speedPID.differentialError 
				);
		}

	} //--while(Running)
	
	print( "Eddie is cleaning up...\r\n" );
	
CloseEncoder();
	
	pthread_join(udplistenerThread, NULL);
	print( "UDP Thread Joined..\r\n" );

#ifndef DISABLE_MOTORS
	motor_driver_disable();
	print( "Motor Driver Disabled..\r\n" );
#endif
	
	print( "Eddie cleanup complete. Good Bye!\r\n" );
	return 0;
}