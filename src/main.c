#include "imu/imu.h"
#include "motordriver/MotorDriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include "mraa.h"
#include "pid.h"

#include "Kalman.h"

//#define DISABLE_MOTORS

//TODO: Remove network reset at startup when Intel fixes the confirmed bug in GPIO
//TODO: Fix Kalman.h since I harshly converted C++ class to C
//TODO: Finish IMU complementary filter and Kalman filter tuning
//TODO: Expand network command interface, currently using UDP to simplify the connection process
//TODO: Give network interface a proper home
//TODO: Enhance movement parameters, currently set desired angle +/- to move forward backward
//TODO: Automatically detect camera and enable
//TODO: On camera webpage add buttons to control Eddie
//TODO: Clean up clean up clean up.. as always

#define DEFAULT_TRIM -4.0f //Eddie's balance point

#define DEFAULT_FWD_TRIM -3.0f //Trim used to drive forwards in motor speed %
#define DEFAULT_REV_TRIM 3.0f //Trim used to drive backwards in motor speed %

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
	DRIVE_REVERSE
};

float filteredPitch;
float currentTrim = DEFAULT_TRIM;
int currentDriveMode = DRIVE_IDLE;
float driveForwardTrim = DEFAULT_FWD_TRIM;
float driveReverseTrim = DEFAULT_REV_TRIM;

// Define the exit signal handler
void signal_callback_handler(int signum)
{
	printf("Exiting program; Caught signal %d\r\n",signum);

	Running = 0;
}

/* UDP Receive bits and bytes that need a proper home */
	#include <sys/socket.h>
	#include <sys/ioctl.h>
	#include <netinet/in.h>
	#include <net/if.h>
	#include <string.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <sys/prctl.h>
	#define MAXMESSAGESIZE 64
	int bsock, sockfd, RXsockfd, fd_SERIAL;
	struct sockaddr_in broadcastAddr,servaddr,cliaddr;
	pthread_t udplistenerThread;
	void initListener( unsigned short udpListenPort )
	{
		sockfd = socket( AF_INET, SOCK_DGRAM, 0 );
		bzero( &servaddr, sizeof( servaddr ) );
		servaddr.sin_family = AF_INET;
		servaddr.sin_addr.s_addr = htonl( INADDR_ANY );
		servaddr.sin_port = htons( udpListenPort );
		bind( sockfd,(struct sockaddr *)&servaddr, sizeof( servaddr ) );
	}

	int udpMsgLen=0;
	int startup = 1;
	int checkUDPReady( char * udpBuffer )
	{
		int bytesAv = 0;
		if( ioctl( sockfd, FIONREAD, &bytesAv ) > 0 || bytesAv > 0 )
		{
			socklen_t len = sizeof( cliaddr );
			udpMsgLen = recvfrom( sockfd, udpBuffer, MAXMESSAGESIZE, 0, ( struct sockaddr * ) &cliaddr, &len );
			udpBuffer[ udpMsgLen ] = 0;
		
			return 1;
		}
		
		return 0;
	}

	void* udplistener_Thread( void *arg )
	{  
		printf("Eddie is starting the UDP network thread..\r\n");
		signal( SIGINT, signal_callback_handler );
		signal( SIGHUP, signal_callback_handler );
		signal( SIGTERM, signal_callback_handler );
		prctl(PR_SET_NAME,"updlistener",0,0,0);
	
		initListener( UDP_LISTEN_PORT );
	
		char incomingUDP[MAXMESSAGESIZE+1];
	
		while(Running)
		{
			usleep( 20000 );
	
			while( checkUDPReady( incomingUDP ) )
			{		
				if( !memcmp( incomingUDP, "TRIM+", 5 ) )
				{
					currentTrim -= 0.1;
					printf( "Trim forwards command received and set to: %0.2f\r\n", currentTrim );
				}
				else if( !memcmp( incomingUDP, "TRIM-", 5 ) )
				{
					currentTrim += 0.1;
					printf( "Trim backwards command received and set to: %0.2f\r\n", currentTrim );
				}
				else if( !memcmp( incomingUDP, "IDLE", 4 ) )
				{
					currentDriveMode = DRIVE_IDLE;
					printf( "Drive Mode: IDLE command received\r\n" );
				}
				else if( !memcmp( incomingUDP, "FORWARD", 7 ) )
				{
					currentDriveMode = DRIVE_FORWARD;
					printf( "Drive Mode: FORWARD command received\r\n" );
				}
				else if( !memcmp( incomingUDP, "REVERSE", 7 ) )
				{
					currentDriveMode = DRIVE_REVERSE;
					printf( "Drive Mode: REVERSE command received\r\n" );
				}
				else if ( strncmp( incomingUDP, "PIDP", 4 ) == 0 )
				{
					float newGain = 0;
					newGain = atof( &incomingUDP[4] );
					printf( "New PID P Gain Received: Changing %0.3f to %0.3f\r\n", pidP_P_GAIN, newGain );
					pidP_P_GAIN = newGain;
				}
				else if ( strncmp( incomingUDP, "PIDI", 4 ) == 0 )
				{
					float newGain = 0;
					newGain = atof( &incomingUDP[4] );
					printf( "New PID I Gain Received: Changing %0.3f to %0.3f\r\n", pidP_I_GAIN, newGain );
					pidP_I_GAIN = newGain;
				}
				else if ( strncmp( incomingUDP, "PIDD", 4 ) == 0 )
				{
					float newGain = 0;
					newGain = atof( &incomingUDP[4] );
					printf( "New PID D Gain Received: Changing %0.3f to %0.3f\r\n", pidP_D_GAIN, newGain );
					pidP_D_GAIN = newGain;
				}
				else if ( strncmp( incomingUDP, "KALQ", 4 ) == 0 )
				{
					float newGain = 0;
					newGain = atof( &incomingUDP[4] );
					printf( "Setting Kalman QBias to: %0.2f\r\n", newGain );
					setQbias( newGain );
				}
				else if ( strncmp( incomingUDP, "KALR", 4 ) == 0 )
				{
					float newGain = 0;
					newGain = atof( &incomingUDP[4] );
					printf( "Setting Kalman RMeasure to: %0.2f\r\n", newGain );
					setRmeasure( newGain );
				}
				incomingUDP[ 0 ] = 0;
			}
		}

		return NULL;
	}
/* UDP Receive bits and bytes */

int main(int argc, char **argv)
{
	// Register signal and signal handler
	signal(SIGINT, signal_callback_handler);

	printf("Eddie starting...\r\n");

	imuinit();
	printf("IMU Started.\r\n");

	InitKalman();
	
#ifndef DISABLE_MOTORS
	printf( "Starting motor driver (and resetting wireless) please be patient..\r\n" );
	if ( motor_driver_enable() < 1 )
		{
			printf("Startup Failed; Error starting motor driver.\r\n");
			motor_driver_disable();
			return -1;
		}
	printf("Motor Driver Started.\r\n");
#endif
	
	//Start network thread
	pthread_create( &udplistenerThread, NULL, &udplistener_Thread, NULL );
	
	printf( "Starting PID controller\r\n" );
	pidP_P_GAIN = PIDP_P_GAIN;	pidP_I_GAIN = PIDP_I_GAIN;	pidP_D_GAIN = PIDP_D_GAIN;	pidP_I_LIMIT = PID_I_LIMIT; pidP_EMA_SAMPLES = PIDP_EMA_SAMPLES;
	PIDinit( &pitchPID, &pidP_P_GAIN, &pidP_I_GAIN, &pidP_D_GAIN, &pidP_I_LIMIT, &pidP_EMA_SAMPLES );
	
	//Get estimate of starting angle and specify complementary filter and kalman filter start angles
	getOrientation();
	filteredPitch=-i2cPitch;
	setkalmanangle( filteredPitch );
	
	printf( "Eddie startup complete. Hold me upright to begin\r\n" );
	while(Running)
	{
		/*
		//readSensors();
		//printf("gx:%6.2f gy:%6.2f gz:%6.2f  ax:%6.2f ay:%6.2f az:%6.2f  mx:%6.2f my:%6.2f mz:%6.2f  temp:%0.0f\n",gx,gy,gz,ax,ay,az,mx,my,mz,temp);
		*/
		
		getOrientation(); //printf("i2cPitch: %6.2f i2cRoll: %6.2f\n",i2cPitch,i2cRoll);
		
		/*Complementary filter*/
		filteredPitch = 0.993 * ( filteredPitch+(gy*.01)) + 0.007*(-i2cPitch);
		/*Kalman filter*/	
		kalmanAngle = -getkalmanangle(filteredPitch, gy, 0.01 /*dt*/);
		
		if ( fabs( filteredPitch ) > 30 && !inFalloverState )
		{
			motor_driver_standby(1);
			inFalloverState = 1;
			printf( "Help! I've fallen over and I can't get up =)\r\n");
		} 
		else if ( fabs( kalmanAngle ) < 10 && inFalloverState )
		{
			motor_driver_standby(0);
			inFalloverState = 0;
			printf( "Thank you!\r\n" );
		}
		
		if ( !inFalloverState )
		{
			pitchPIDoutput = PIDUpdate( currentTrim, kalmanAngle, 10 /*ms*/, &pitchPID);
		}
		else
		{
			pitchPID.accumulatedError = 0;
		}
		
		switch( currentDriveMode )
		{
			case DRIVE_FORWARD:
				pitchPIDoutput += driveForwardTrim;
			break;
			case DRIVE_REVERSE:
				pitchPIDoutput += driveReverseTrim;
			break;
		}
			
#ifndef DISABLE_MOTORS
		set_motor_speed_right( pitchPIDoutput );
		set_motor_speed_left( pitchPIDoutput );
#endif

		if ( !inFalloverState )
		{
			printf( "PID Output: %0.2f\t", pitchPIDoutput );
			printf( "filteredPitch: %6.2f kalman: %6.2f\t",i2cPitch, -filteredPitch, kalmanAngle );
			printf( "Pe: %0.2f\tIe: %0.2f\tDe: %0.2f\r\n", pitchPID.error, pitchPID.accumulatedError, pitchPID.differentialError );
		}
		
		usleep(10000);

	} //--while(Running)
	
	printf( "Eddie is cleaning up...\r\n" );
	
	pthread_join(udplistenerThread, NULL);
	printf( "UDP Thread Joined..\r\n" );

#ifndef DISABLE_MOTORS
	motor_driver_disable();
	printf( "Motor Driver Disabled..\r\n" );
#endif
	
	printf( "Eddie cleanup complete. Good Bye!\r\n" );
	return 0;
}
