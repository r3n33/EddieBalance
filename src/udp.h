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

int bsock=-1;
int sockfd, RXsockfd, fd_SERIAL;
struct sockaddr_in sendtoAddress,servaddr,cliaddr;
pthread_t udplistenerThread;

void (*functionPtr)(char *);

void initUDP( void * p_funptr, int * p_running )
{
	functionPtr = p_funptr;
	isRunning = p_running;
}

void initUDPSend(char *sendtoIP, unsigned short sendtoPort)
{
	bsock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);  							// Create socket for sending/receiving 
	memset( &sendtoAddress, 0, sizeof( sendtoAddress ) );   				// Zero out structure 
	sendtoAddress.sin_family = AF_INET;                 						// Internet address family 
	sendtoAddress.sin_addr.s_addr = inet_addr( sendtoIP );					// Destination IP address 
	sendtoAddress.sin_port = htons(sendtoPort);         						// Destination port 
}
/* NOTE: To broadcast UDP you must:
//int broadcastPermission = 0;																		// Set socket to allow broadcast 
//setsockopt( bsock, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof( broadcastPermission ) );
*/
void UDPSend(char * data, int len)
{
	if(bsock>=0)
		sendto( bsock, data, len, 0, ( struct sockaddr * )&sendtoAddress, sizeof( sendtoAddress ) );
}

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
		
		initUDPSend( (char*)inet_ntoa( cliaddr.sin_addr ), UDP_LISTEN_PORT + 1 );
		
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
			incomingUDP[ 0 ] = 0;
		}
	}

	return NULL;
}

#endif