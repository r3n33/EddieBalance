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

#include "udp.h"

//#define DISABLE_MOTORS

//TODO: Remove network reset at startup when Intel fixes the confirmed bug in GPIO
//TODO: Finish IMU complementary filter and Kalman filter tuning
//TODO: Automatically detect camera and enable

#define DEFAULT_TRIM -5.9f //Eddie's balance point

#define DEFAULT_FWD_TRIM -1.25f //Trim used to drive forwards by changing angular setpoint
#define DEFAULT_REV_TRIM 1.25f //Trim used to drive backwards by changing angular setpoint
#define DEFAULT_TRN_TRIM 20.0f //Default trim for turning expressed in motor speed %

#define MOTION_COMPENSATE_RATE 0.08

#define UDP_LISTEN_PORT 4242 //UDP Port for receiving commands

int Running = 1;
int inFalloverState = 0; //Used to flag when Eddie has fallen over and disables motors
int inSteadyState = 0; //Used to flag how long Eddie is being held upright in a stead state and will enable motors

PID_t pitchPID;
float pitchPIDoutput = 0;
float pidP_P_GAIN,pidP_I_GAIN,pidP_D_GAIN,pidP_I_LIMIT,pidP_EMA_SAMPLES;

float kalmanAngle;

enum 
{
	DRIVE_IDLE = 0,
	DRIVE_FORWARD,
	DRIVE_REVERSE,
	DRIVE_RIGHT,
	DRIVE_LEFT
};

float filteredPitch;
float currentTrim = DEFAULT_TRIM;
int currentDriveMode = DRIVE_IDLE;
float driveForwardTrim = DEFAULT_FWD_TRIM;
float driveReverseTrim = DEFAULT_REV_TRIM;
float driveRightTrim = DEFAULT_TRN_TRIM;
float driveLeftTrim = -DEFAULT_TRN_TRIM;

enum
{
	CONSOLE=0,
	UDP
};
int outputto = UDP;

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
			broadcast(buffer,len);
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

void UDP_Data_Handler( char * p_udpin )
{
	if( !memcmp( p_udpin, "TRIM+", 5 ) )
	{
		currentTrim -= 0.1;
		print( "Trim forwards command received and set to: %0.2f\r\n", currentTrim );
	}
	else if( !memcmp( p_udpin, "TRIM-", 5 ) )
	{
		currentTrim += 0.1;
		print( "Trim backwards command received and set to: %0.2f\r\n", currentTrim );
	}
	else if( !memcmp( p_udpin, "IDLE", 4 ) )
	{
		currentDriveMode = DRIVE_IDLE;
		print( "Drive Mode: IDLE command received\r\n" );
	}
	else if( !memcmp( p_udpin, "FORWARD", 7 ) )
	{
		float driveSpeed = atof( &p_udpin[7] );
		if ( driveSpeed ) driveForwardTrim = -driveSpeed;
		currentDriveMode = DRIVE_FORWARD;
		print( "Drive Mode: FORWARD command at %0.2f deg angle\r\n", driveForwardTrim );
	}
	else if( !memcmp( p_udpin, "REVERSE", 7 ) )
	{
		float driveSpeed = atof( &p_udpin[7] );
		if ( driveSpeed ) driveReverseTrim = driveSpeed;
		currentDriveMode = DRIVE_REVERSE;
		print( "Drive Mode: REVERSE command at %0.2f deg angle\r\n", driveReverseTrim );
	}
	else if( !memcmp( p_udpin, "RIGHT", 5 ) )
	{
		float driveSpeed = atof( &p_udpin[5] );
		if ( driveSpeed ) driveRightTrim = driveSpeed;
		currentDriveMode = DRIVE_RIGHT;
		print( "Drive Mode: RIGHT command at %0.2f speed received\r\n", driveRightTrim );
	}
	else if( !memcmp( p_udpin, "LEFT", 4 ) )
	{
		float driveSpeed = atof( &p_udpin[4] );
		if ( driveSpeed ) driveLeftTrim = -driveSpeed;
		currentDriveMode = DRIVE_LEFT;
		print( "Drive Mode: LEFT command at %0.2f speed received\r\n", driveRightTrim );
	}
	else if ( strncmp( p_udpin, "PIDP", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "New PID P Gain Received: Changing %0.3f to %0.3f\r\n", pidP_P_GAIN, newGain );
		pidP_P_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "PIDI", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "New PID I Gain Received: Changing %0.3f to %0.3f\r\n", pidP_I_GAIN, newGain );
		pidP_I_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "PIDD", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "New PID D Gain Received: Changing %0.3f to %0.3f\r\n", pidP_D_GAIN, newGain );
		pidP_D_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "KALQ", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "Setting Kalman Q Bias to: %0.2f\r\n", newGain );
		setQbias( newGain );
	}
	else if ( strncmp( p_udpin, "KALR", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "Setting Kalman R Measure to: %0.2f\r\n", newGain );
		setRmeasure( newGain );
	}	
}

int main(int argc, char **argv)
{
	// Register signal and signal handler
	signal(SIGINT, signal_callback_handler);
	initUDP(&UDP_Data_Handler, &Running );
	print("Eddie starting...\r\n");

	imuinit();
	print("IMU Started.\r\n");

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
	pidP_P_GAIN = PIDP_P_GAIN;	pidP_I_GAIN = PIDP_I_GAIN;	pidP_D_GAIN = PIDP_D_GAIN;	pidP_I_LIMIT = PID_I_LIMIT; pidP_EMA_SAMPLES = PIDP_EMA_SAMPLES;
	PIDinit( &pitchPID, &pidP_P_GAIN, &pidP_I_GAIN, &pidP_D_GAIN, &pidP_I_LIMIT, &pidP_EMA_SAMPLES );
	
	//Get estimate of starting angle and specify complementary filter and kalman filter start angles
	getOrientation();
	filteredPitch=-i2cPitch;
	setkalmanangle( filteredPitch );
	
	
	float testMotionTrim=0;
	float testTrimApplied = 0;
	
	print( "Eddie startup complete. Hold me upright to begin\r\n" );
	while(Running)
	{
		/*
		//readSensors();
		//print("gx:%6.2f gy:%6.2f gz:%6.2f  ax:%6.2f ay:%6.2f az:%6.2f  mx:%6.2f my:%6.2f mz:%6.2f  temp:%0.0f\n",gx,gy,gz,ax,ay,az,mx,my,mz,temp);
		*/
		
		getOrientation(); //print("i2cPitch: %6.2f i2cRoll: %6.2f\n",i2cPitch,i2cRoll);
		
		/*Complementary filter*/
		filteredPitch = 0.992 * ( filteredPitch+(gy*.01)) + 0.008*(-i2cPitch);
		/*Kalman filter*/	
		kalmanAngle = -getkalmanangle(filteredPitch, gy, 0.01 /*dt*/);
		
		/* Monitor angles to determine if Eddie has falled too far or if Eddie has been returned upright*/
		if ( fabs( filteredPitch ) > 30 && !inFalloverState )
		{
#ifndef DISABLE_MOTORS
			motor_driver_standby(1);
#endif
			inFalloverState = 1;
			print( "Help! I've fallen over and I can't get up =)\r\n");
		} 
		else if ( fabs( kalmanAngle ) < 10 && inFalloverState )
		{
			if ( ++inSteadyState == 200 )
			{
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
			/* Testing adjusting setpoint when motor speeds exceed threshold
			 * Working quite nicely as motion compensation
			 * Now expanded to three thresholds with different rates. I'm sure there is a 'smarter' way
			 * to handle this but for now I'll stick with what works.
			 */
			if      ( pitchPIDoutput >  60.0 && testMotionTrim <  4.0 ) testMotionTrim += MOTION_COMPENSATE_RATE*2;
			else if ( pitchPIDoutput < -60.0 && testMotionTrim > -4.0 ) testMotionTrim -= MOTION_COMPENSATE_RATE*2;
			
			else if ( pitchPIDoutput >  40.0 && testMotionTrim <  3.0 ) testMotionTrim += MOTION_COMPENSATE_RATE*2;
			else if ( pitchPIDoutput < -40.0 && testMotionTrim > -3.0 ) testMotionTrim -= MOTION_COMPENSATE_RATE*2;
			
			else if ( pitchPIDoutput >  25.0 && testMotionTrim <  1.5 ) testMotionTrim += MOTION_COMPENSATE_RATE;
			else if ( pitchPIDoutput < -25.0 && testMotionTrim > -1.5 ) testMotionTrim -= MOTION_COMPENSATE_RATE;
			
			else if ( testMotionTrim > 0 ) testMotionTrim -= MOTION_COMPENSATE_RATE;
			else if ( testMotionTrim < 0 ) testMotionTrim += MOTION_COMPENSATE_RATE;
			
			/* Testing drive operations */
			switch( currentDriveMode )
			{
				case DRIVE_FORWARD:
					testTrimApplied = driveForwardTrim + testMotionTrim;
				break;
				case DRIVE_REVERSE:
					testTrimApplied = driveReverseTrim + testMotionTrim;
				break;
				case DRIVE_IDLE: default: //Bleed off applied trim ( or drive trim ) to match motion compensation trim
					if ( testTrimApplied < testMotionTrim ) testTrimApplied += MOTION_COMPENSATE_RATE;
					else if ( testTrimApplied > testMotionTrim ) testTrimApplied -= MOTION_COMPENSATE_RATE;
				break;
			}
			
			pitchPIDoutput = PIDUpdate( currentTrim+testTrimApplied, kalmanAngle, 10 /*ms*/, &pitchPID);
		}
		else //We are inFalloverState and the PID i term should remain 0
		{
			pitchPID.accumulatedError = 0;
		}
		
		/* Testing turn operations */	
		float testTurnTrim;
		switch( currentDriveMode )
		{
			case DRIVE_RIGHT:
				testTurnTrim = driveRightTrim;
			break;
			case DRIVE_LEFT:
				testTurnTrim = driveLeftTrim;
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
			print( "PID Output: %0.2f\tfilteredPitch: %6.2f kalman: %6.2f\tPe: %0.2f\tIe: %0.2f\tDe: %0.2f\r\n",pitchPIDoutput,i2cPitch, -filteredPitch, kalmanAngle,pitchPID.error, pitchPID.accumulatedError, pitchPID.differentialError);
		}
		
		usleep(10000); //Wait 10ms

	} //--while(Running)
	
	print( "Eddie is cleaning up...\r\n" );
	
	pthread_join(udplistenerThread, NULL);
	print( "UDP Thread Joined..\r\n" );

#ifndef DISABLE_MOTORS
	motor_driver_disable();
	print( "Motor Driver Disabled..\r\n" );
#endif
	
	print( "Eddie cleanup complete. Good Bye!\r\n" );
	return 0;
}
