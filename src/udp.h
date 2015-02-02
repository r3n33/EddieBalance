#ifndef UDP_H
#define UDP_H


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

#define UDP_LISTEN_PORT 4242 //UDP Port for receiving commands

int * isRunning;

// Define the exit signal handler
void udp_signal_handler(int signum)
{
	(*isRunning) = 0;
}

int bsock, sockfd, RXsockfd, fd_SERIAL;
struct sockaddr_in broadcastAddr,servaddr,cliaddr;
pthread_t udplistenerThread;

void (*functionPtr)(char *);

void initUDP( void * p_funptr, int * p_running )
{
	functionPtr = p_funptr;
	isRunning = p_running;
}

void initBroadCast(char *broadcastIP, unsigned short broadcastPort)
{
	bsock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);  						// Create socket for sending/receiving datagrams 
	int broadcastPermission = 1;																	// Set socket to allow broadcast 
	setsockopt( bsock, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof( broadcastPermission ) );
	memset( &broadcastAddr, 0, sizeof( broadcastAddr ) );   					// Zero out structure 
	broadcastAddr.sin_family = AF_INET;                 					// Internet address family 
	broadcastAddr.sin_addr.s_addr = inet_addr( broadcastIP );				// Broadcast IP address 
	broadcastAddr.sin_port = htons(broadcastPort);         				// Broadcast port 
}

void broadcast(char * data, int len)
{
	sendto( bsock, data, len, 0, ( struct sockaddr * )&broadcastAddr, sizeof( broadcastAddr ) );
}

void initListener( unsigned short udpListenPort )
{
	sockfd = socket( AF_INET, SOCK_DGRAM, 0 );
	bzero( &servaddr, sizeof( servaddr ) );
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl( INADDR_ANY );
	servaddr.sin_port = htons( udpListenPort );
	bind( sockfd,(struct sockaddr *)&servaddr, sizeof( servaddr ) );
	
	initBroadCast( "255.255.255.255", udpListenPort+1 );
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
	signal( SIGINT, udp_signal_handler );
	signal( SIGHUP, udp_signal_handler );
	signal( SIGTERM, udp_signal_handler );
	prctl(PR_SET_NAME,"updlistener",0,0,0);

	initListener( UDP_LISTEN_PORT );

	char incomingUDP[MAXMESSAGESIZE+1];

	while((*isRunning))
	{
		usleep( 20000 );

		while( checkUDPReady( incomingUDP ) )
		{
			(*functionPtr)(incomingUDP);
			/*
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
			*/
			incomingUDP[ 0 ] = 0;
		}
	}

	return NULL;
}

#endif