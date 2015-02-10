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

#include <sys/time.h>

long long last_gy_ms;
long long last_PID_ms;

long long current_milliseconds() 
{
    struct timeval c_time; 
    gettimeofday(&c_time, NULL);
    long long milliseconds = c_time.tv_sec*1000LL + c_time.tv_usec/1000;
    return milliseconds;
}

//#define DISABLE_MOTORS

//TODO: Remove network reset at startup when Intel fixes the confirmed bug in GPIO
//TODO: Finish IMU complementary filter and Kalman filter tuning
//TODO: Finish motion compensation testing
//TODO: Finish remote control testing
//TODO: measure every interval and wait appropriate time

#define DEFAULT_TRIM -2.0f //Eddie's balance point

#define DEFAULT_FWD_TRIM -1.25f //Trim used to drive forwards by changing angular setpoint
#define DEFAULT_REV_TRIM 1.25f //Trim used to drive backwards by changing angular setpoint
#define DEFAULT_TRN_TRIM 20.0f //Default trim for turning expressed in motor speed %

#define MOTION_COMPENSATE_RATE 0.3 //Maximum
#define DRIVE_TRIM_BLEED_RATE 0.8 //Degrees per iteration

#define UDP_LISTEN_PORT 4242 //UDP Port for receiving commands

enum
{
	CONSOLE=0,
	UDP //Will send all print() calls to UDP port 4243, all printf() will still go to console (:)
};
int outputto = UDP; //Change to fit current need.

int Running = 1;
int inFalloverState = 0; //Used to flag when Eddie has fallen over and disables motors
int inSteadyState = 0; //Used to flag how long Eddie is being held upright in a stead state and will enable motors

PID_t pitchPID; //PID Controller for holding angle on pitch axis
float pitchPIDoutput = 0;
float pidP_P_GAIN,pidP_I_GAIN,pidP_D_GAIN,pidP_I_LIMIT,pidP_EMA_SAMPLES;

PID_t setpointPID; //PID Controller for pitchPID input
float setpointPIDoutput = 0;
float pidS_P_GAIN,pidS_I_GAIN,pidS_D_GAIN,pidS_I_LIMIT,pidS_EMA_SAMPLES;

float kalmanAngle;

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
float currentTrim = DEFAULT_TRIM;
float driveForwardTrim = DEFAULT_FWD_TRIM;
float driveReverseTrim = DEFAULT_REV_TRIM;
float driveRightTrim = DEFAULT_TRN_TRIM;
float driveLeftTrim = -DEFAULT_TRN_TRIM;

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

/* Current UDP Commands:
 *
 * IDLE 	= No forward or reverse, just balance.
 * FORWARD 	= Drive forwards; 	FORWARD2.5 = Drive forwards at 2.5 degree angle.
 * REVERSE 	= Drive backwards; 	REVERSE3.5 = Drive reverse at 3.5 degree angle.
 * RIGHT 	= Turn right; 		RIGHT25.0 = Turn right at 25% speed.
 * LEFT 	= Turn left; 		LEFT22.0 = Turn left at 22% speed.
 * STRAIGHT = Cancel turning.
 *
 * TRIM+ = Trim forwards at 0.1 degree increment.
 * TRIM- = Trim backwards at 0.1 degree increment.
 *
 * PID[P,I,D][value] = adjust PIDs
 * PIS[P,I,D][value] = adjust setpoint PIDs
 * KALQ[value] = adjust Kalman Q Bias
 * KALR[value] = adjust Kalman R Measure
 *
 */
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
		currentTurnMode = TURN_RIGHT;
		print( "Turn Mode: RIGHT command at %0.2f speed received\r\n", driveRightTrim );
	}
	else if( !memcmp( p_udpin, "LEFT", 4 ) )
	{
		float driveSpeed = atof( &p_udpin[4] );
		if ( driveSpeed ) driveLeftTrim = -driveSpeed;
		currentTurnMode = TURN_LEFT;
		print( "Turn Mode: LEFT command at %0.2f speed received\r\n", driveRightTrim );
	}
	else if( !memcmp( p_udpin, "STRAIGHT", 8 ) )
	{
		currentTurnMode = TURN_IDLE;
		print( "Turn Mode: SRAIGHT command received\r\n" );
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
	
	
	else if ( strncmp( p_udpin, "PISP", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "New PIDS P Gain Received: Changing %0.3f to %0.3f\r\n", pidS_P_GAIN, newGain );
		pidS_P_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "PISI", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "New PIDS I Gain Received: Changing %0.3f to %0.3f\r\n", pidS_I_GAIN, newGain );
		pidS_I_GAIN = newGain;
	}
	else if ( strncmp( p_udpin, "PISD", 4 ) == 0 )
	{
		float newGain = 0;
		newGain = atof( &p_udpin[4] );
		print( "New PIDS D Gain Received: Changing %0.3f to %0.3f\r\n", pidS_D_GAIN, newGain );
		pidS_D_GAIN = newGain;
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

void motion_comp( float p_speed, float * p_trim, float p_threshold, float p_maxAngle, float p_maxRate )
{
	float fabsSpeed = fabs(p_speed);
	if ( fabsSpeed >  100.0 ) fabsSpeed =  100.0;
	int sign = p_speed < 0 ? -1 : 1;
	float compensationLimit = (p_maxAngle / 100.0) * fabsSpeed;
	float compensationRate = (p_maxRate / 100.0) * fabsSpeed;
	
	if ( fabsSpeed < p_threshold )
	{
		if ( *p_trim > 0 ) *p_trim -= p_maxRate*3;
		else if ( *p_trim < 0 ) *p_trim += p_maxRate*3;
	}
	else if ( fabs(*p_trim) < compensationLimit )
	{
		printf( "Speed %0.2f Limit %0.2f Rate %0.2f Trim %0.2f \t DFtrim: %0.2f DRtrim: %0.2f\r\n", fabsSpeed, compensationLimit, compensationRate, *p_trim, driveForwardTrim, driveReverseTrim );
		*p_trim += sign * compensationRate;
	}
}

int main(int argc, char **argv)
{
	//Register signal and signal handler
	signal(SIGINT, signal_callback_handler);
	
	//Init UDP with callback and pointer to run status
	initUDP( &UDP_Data_Handler, &Running );
	
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
	//Set default PID values and init pitchPID controller
	pidP_P_GAIN = PIDP_P_GAIN;	pidP_I_GAIN = PIDP_I_GAIN;	pidP_D_GAIN = PIDP_D_GAIN;	pidP_I_LIMIT = PID_I_LIMIT; pidP_EMA_SAMPLES = PIDP_EMA_SAMPLES;
	PIDinit( &pitchPID, &pidP_P_GAIN, &pidP_I_GAIN, &pidP_D_GAIN, &pidP_I_LIMIT, &pidP_EMA_SAMPLES );
	//Set default values and init setpointPID controller
	pidS_P_GAIN = PIDS_P_GAIN;	pidS_I_GAIN = PIDS_I_GAIN;	pidS_D_GAIN = PIDS_D_GAIN;	pidS_I_LIMIT = PID_I_LIMIT; pidS_EMA_SAMPLES = PIDS_EMA_SAMPLES;
	PIDinit( &setpointPID, &pidS_P_GAIN, &pidS_I_GAIN, &pidS_D_GAIN, &pidS_I_LIMIT, &pidS_EMA_SAMPLES );

	
	//Get estimate of starting angle and specify complementary filter and kalman filter start angles
	getOrientation();
	filteredPitch = i2cPitch;
	setkalmanangle( filteredPitch );
	filteredRoll = i2cRoll;
	
	float testMotionTrim=0;
	float testDriveTrim = 0;
	
	print( "Eddie startup complete. Hold me upright to begin\r\n" );
	
	float gy_scale = 0.01;
	last_PID_ms = last_gy_ms = current_milliseconds();
	
	int skipOutput = 0;
	while(Running)
	{		
		getOrientation(); //print("i2cPitch: %6.2f i2cRoll: %6.2f\n",i2cPitch,i2cRoll);
		
		gy_scale = (float)( current_milliseconds() - last_gy_ms ) / 1000.0f;
		//printf( "gy scale: %0.2f last_gy_ms: %d\r\n", gy_scale, current_milliseconds() - last_gy_ms );
		last_gy_ms = current_milliseconds();
		
		/*Complementary filters*/
		filteredPitch = 0.97 * ( filteredPitch + ( gy * gy_scale ) ) + ( 0.03 * i2cPitch );
		filteredRoll = 0.98 * ( filteredRoll + ( gx * gy_scale ) ) + ( 0.02 * i2cRoll );
		
		/*Kalman filter*/	
		kalmanAngle = -getkalmanangle(filteredPitch, gy, gy_scale /*dt*/);
		
		/* Monitor angles to determine if Eddie has fallen too far or if Eddie has been returned upright*/
		if ( ( fabs( filteredPitch ) > 30 || fabs( filteredRoll ) > 30 ) && !inFalloverState )
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
			 * Now made into a function with adjustable parameters.
			 * If you are PID tuning for balance I would recommend commenting this function.
			 */
			motion_comp( pitchPIDoutput, &testMotionTrim, 77.0 /*threshold*/, 7.5 /*max angle*/, MOTION_COMPENSATE_RATE );	 
			 
			/* Testing drive operations */
			switch( currentDriveMode )
			{
				case DRIVE_FORWARD:
					testDriveTrim = driveForwardTrim + testMotionTrim;
				break;
				case DRIVE_REVERSE:
					testDriveTrim = driveReverseTrim + testMotionTrim;
				break;
				case DRIVE_IDLE: default: //Bleed off drive trim to match motion compensation trim
					if ( testDriveTrim < testMotionTrim ) 
					{
						if ( testDriveTrim + DRIVE_TRIM_BLEED_RATE < 0 ) testDriveTrim += DRIVE_TRIM_BLEED_RATE;
						else testDriveTrim = 0;
					}
					else if ( testDriveTrim > testMotionTrim )
					{
						if ( testDriveTrim - DRIVE_TRIM_BLEED_RATE > 0 ) testDriveTrim -= DRIVE_TRIM_BLEED_RATE;
						else testDriveTrim = 0;
					}
				break;
			}
			
			//Use setpointPID to determine input for pitchPID controller
			setpointPIDoutput = PIDUpdate( currentTrim+testDriveTrim, kalmanAngle, current_milliseconds() - last_PID_ms, &setpointPID );
			pitchPIDoutput = PIDUpdate( setpointPIDoutput+currentTrim+testDriveTrim, kalmanAngle, current_milliseconds() - last_PID_ms, &pitchPID);
			
			last_PID_ms = current_milliseconds();
			
			//Limit pitchPID output to 100% motor throttle
			if ( pitchPIDoutput > 100.0 ) pitchPIDoutput = 100.0;
			if ( pitchPIDoutput < -100.0 ) pitchPIDoutput = -100.0;
		}
		else //We are inFalloverState and the PID i term should remain 0
		{
			pitchPID.accumulatedError = 0;
			setpointPID.accumulatedError = 0;
		}
		
		/* Testing turn operations */	
		float testTurnTrim;
		switch( currentTurnMode )
		{
			case TURN_RIGHT:
				testTurnTrim = driveRightTrim;
			break;
			case TURN_LEFT:
				testTurnTrim = driveLeftTrim;
			break;
			default:
				testTurnTrim = 0;
			break;				
		}
	
		float testDrivePower = 0;
		/* Testing forcing motor output on drive command 
		 * PID Output shows that it can compensate for 60%?@!$@#
		switch( currentDriveMode )
		{
			case DRIVE_FORWARD:
				testDrivePower = 25.0;
			break;
			case DRIVE_REVERSE:
				testDrivePower = -25.0;
			break;
		}
		*/
		
#ifndef DISABLE_MOTORS
		set_motor_speed_right( pitchPIDoutput + testDrivePower - testTurnTrim );
		set_motor_speed_left( pitchPIDoutput + testDrivePower + testTurnTrim );
#endif

		if ( !inFalloverState || outputto == UDP )
		{
			//if ( ++skipOutput == 5 )
			//{
			print( "PIDout: %0.2f,%0.2f\tcompPitch: %6.2f kalPitch: %6.2f\tPe: %0.3f\tIe: %0.3f\tDe: %0.3f\tPe: %0.3f\tIe: %0.3f\tDe: %0.3f\r\n",
				setpointPIDoutput, 
				pitchPIDoutput, 
				filteredPitch, 
				kalmanAngle,
				pitchPID.error, 
				pitchPID.accumulatedError, 
				pitchPID.differentialError, 
				setpointPID.error, 
				setpointPID.accumulatedError, 
				setpointPID.differentialError 
				);
			
			//skipOutput = 0;
			//}
		}
		
//TODO: measure every interval and wait appropriate time
//usleep(3000); //Wait 6ms with (measured) 4ms of processing for 10ms intervals

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
