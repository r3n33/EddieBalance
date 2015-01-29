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

//Eddie's balance point without head and with current IMU filtering is -8.7
//Eddie's balance point with a $5 camera head and current IMU filtering is -3.2
#define DEFAULT_TRIM -3.2f 

#define DEFAULT_FWD_TRIM -1.25f //Trim used to drive forwards
#define DEFAULT_REV_TRIM 1.5f //Trim used to drive backwards

#define UDP_LISTEN_PORT 4242

int Running = 1;
int inStandbyState = 0;

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
		
		/*Complementary filter example:	//angle = 0.98 *(angle+gyro*dt) + 0.02*accAngle*/
		filteredPitch = 0.993 * ( filteredPitch+(gy*.01)) + 0.007*(-i2cPitch); //0.993 is last known best with PID tune
				
		kalmanAngle = -getkalmanangle(filteredPitch, gy, 0.01 /*dt*/);
		
		if ( fabs( filteredPitch ) > 30 && !inStandbyState)
		{
			motor_driver_standby(1);
			inStandbyState = 1;
			printf( "Help! I've fallen over and I can't get up =)\r\n");
		} 
		else if ( fabs( filteredPitch ) < 10 && inStandbyState )
		{
			motor_driver_standby(0);
			inStandbyState = 0;
			printf( "Thank you!\r\n" );
		}
		
		if ( !inStandbyState )
		{
			switch( currentDriveMode )
			{
				case DRIVE_IDLE:
					pitchPIDoutput = PIDUpdate( currentTrim, kalmanAngle, 1 /*ms*/, &pitchPID);
				break;
				case DRIVE_FORWARD:
					pitchPIDoutput = PIDUpdate( currentTrim+driveForwardTrim, -filteredPitch, 1 /*ms*/, &pitchPID);
				break;
				case DRIVE_REVERSE:
					pitchPIDoutput = PIDUpdate( currentTrim+driveReverseTrim, -filteredPitch, 1 /*ms*/, &pitchPID);
				break;
				default:
					pitchPIDoutput = PIDUpdate( currentTrim, -filteredPitch, 1 /*ms*/, &pitchPID);
				break;
			}
		}
		else
		{
			pitchPID.accumulatedError = 0;
		}
		
#ifndef DISABLE_MOTORS
		set_motor_speed_right( pitchPIDoutput );
		set_motor_speed_left( pitchPIDoutput );
#endif

		if ( !inStandbyState )
		{
			//printf("PID Output: %0.2f\r\n", pitchPIDoutput );
			//printf("i2cPitch: %6.2f filteredPitch: %6.2f kalman: %6.2f \r\n",i2cPitch, -filteredPitch, kalmanAngle );
			//printf( "Pe: %0.2f Ie: %0.2f De: %0.2f\r\n", pitchPID.error, pitchPID.accumulatedError, pitchPID.differentialError );
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
