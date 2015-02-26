#ifndef UDP_H
#define UDP_H

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/prctl.h>

#define MAXMESSAGESIZE 64

#define UDP_COMMAND_PORT 4242 //UDP Port for receiving commands
#define UDP_CONTROL_PORT 4240 //UDP Port for receive control data
#define UDP_RESPOND_PORT 4243 //UDP Port for returning data to user

int * isRunning;

// Define the exit signal handler
void udp_signal_handler(int signum)
{
	(*isRunning) = 0;
}

int tx_socketfd = -1; //Socket file descriptor used to send data from Eddie
int rx_command_socketfd; //Socket file descriptor used to receive commands from user
int rx_control_socketfd; //Socket file descriptor used to receive control data from user

struct sockaddr_in sendtoAddress;
struct sockaddr_in rx_cmd_addr;
struct sockaddr_in rx_ctrl_addr;

pthread_t udplistenerThread;

void (*commandFunctionPtr)(char *);
void (*controlFunctionPtr)(char *);

void UDPCloseTX();

void initUDP( void * p_cmdFuncPtr, void * p_ctrlFuncPtr, int * p_running )
{
	commandFunctionPtr = p_cmdFuncPtr;
	controlFunctionPtr = p_ctrlFuncPtr;
	isRunning = p_running;
}

void initUDPSend(char *sendtoIP, unsigned short sendtoPort)
{
	/* NOTE: To broadcast UDP you must:
	//int broadcastPermission = 0;	// Set socket to allow broadcast 
	//setsockopt( tx_socketfd, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof( broadcastPermission ) );
	*/
	tx_socketfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);	// Create socket for sending/receiving 
	memset( &sendtoAddress, 0, sizeof( sendtoAddress ) );	// Zero out structure 
	sendtoAddress.sin_family = AF_INET;						// Internet address family 
	sendtoAddress.sin_addr.s_addr = inet_addr( sendtoIP );	// Destination IP address 
	sendtoAddress.sin_port = htons(sendtoPort);				// Destination port 
}

void UDPSend(char * data, int len)
{
	if(tx_socketfd>=0)
		sendto( tx_socketfd, data, len, 0, ( struct sockaddr * )&sendtoAddress, sizeof( sendtoAddress ) );
}

void initListener( unsigned short udpListenPort, int * p_socket, struct sockaddr_in * p_addr )
{
	*p_socket = socket( AF_INET, SOCK_DGRAM, 0 );
	bzero( p_addr, sizeof( *p_addr ) );
	(*p_addr).sin_family = AF_INET;
	(*p_addr).sin_addr.s_addr = htonl( INADDR_ANY );
	(*p_addr).sin_port = htons( udpListenPort );
	bind( *p_socket, (struct sockaddr *)p_addr, sizeof( *p_addr ) );
}

int udpMsgLen=0;
int startup = 1;
int checkUDPReady( char * udpBuffer, int * p_socket )
{
	int bytesAv = 0;
	
	/* If there is data to be read on the socket bring it in and capture the source IP */
	if( ioctl( *p_socket, FIONREAD, &bytesAv ) > 0 || bytesAv > 0 )
	{
		struct sockaddr_in rx_from_addr;
		socklen_t len = sizeof( rx_from_addr );
		udpMsgLen = recvfrom( *p_socket, udpBuffer, MAXMESSAGESIZE, 0, ( struct sockaddr * )&rx_from_addr, &len );
		udpBuffer[ udpMsgLen ] = 0;
		
//If we received UDP data on control socket close the TX socket so a response can be sent to whomever we received from
		if ( udpMsgLen && p_socket == &rx_control_socketfd ) UDPCloseTX();
		
//If the TX socket is closed init sending socket in case a response is to be sent		
		if ( tx_socketfd < 0 ) initUDPSend( (char*)inet_ntoa( rx_from_addr.sin_addr ), UDP_RESPOND_PORT );
		
		return 1;
	}
	
	return 0;
}

void UDPCloseTX()
{
	close( tx_socketfd );
	tx_socketfd = -1;
}

void* udplistener_Thread( void * arg )
{  
	signal( SIGINT, udp_signal_handler );
	signal( SIGHUP, udp_signal_handler );
	signal( SIGTERM, udp_signal_handler );
	prctl(PR_SET_NAME,"updlistener",0,0,0);

	initListener( UDP_COMMAND_PORT, &rx_command_socketfd, &rx_cmd_addr );
	initListener( UDP_CONTROL_PORT, &rx_control_socketfd, &rx_ctrl_addr );
	
	char incomingUDP[ MAXMESSAGESIZE + 1 ];

	while( (*isRunning) )
	{
		usleep( 20000 ); //Give this thread a break between iterations to keep CPU usage down

		/* Check for UDP data on Control port */
		while( checkUDPReady( incomingUDP, &rx_control_socketfd ) )
		{
			(*controlFunctionPtr)(incomingUDP);
			bzero( incomingUDP, sizeof( incomingUDP ) );
		}
		
		/* Check for UDP data on Command port */
		while( checkUDPReady( incomingUDP, &rx_command_socketfd ) )
		{
			(*commandFunctionPtr)(incomingUDP);
			bzero( incomingUDP, sizeof( incomingUDP ) );
		}
	}

	return NULL;
}

#endif