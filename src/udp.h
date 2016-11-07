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

#define UDP_COMMAND_PORT 4242 //UDP Port for receiving command packets
#define UDP_CONTROL_PORT 4240 //UDP Port for receiving control packets
#define UDP_RESPOND_PORT 4243 //UDP Port for returning data to user

int * isRunning;

// Define the exit signal handler
void udp_signal_handler(int signum)
{
	(*isRunning) = 0;
}

int tx_command_socketfd = -1; 	//Socket descriptor used to send data to bound client
int tx_control_socketfd = -1; 	//Socket descriptor used to response to control packets
int rx_command_socketfd; 		//Socket descriptor used to receive commands from user
int rx_control_socketfd; 		//Socket descriptor used to receive control data from user

struct sockaddr_in tx_cmd_addrin;
struct sockaddr_in tx_ctrl_addrin;
struct sockaddr_in rx_cmd_addrin;
struct sockaddr_in rx_ctrl_addrin;

pthread_t udplistenerThread;

void (*commandFunctionPtr)(char *);
void (*controlFunctionPtr)(char *);

void UDPCloseTX();
void UDPCloseCtrlTX();
void initUDPCmdSend( char * sendtoIP, unsigned short sendtoPort );

void initUDP( void * p_cmdFuncPtr, void * p_ctrlFuncPtr, int * p_running )
{
	commandFunctionPtr = p_cmdFuncPtr;
	controlFunctionPtr = p_ctrlFuncPtr;
	isRunning = p_running;
}

char lastRXAddress[16] = {0}; //The last address a UDP message was received from
char commandBindAddress[16] = {0}; //This is the address we are bound to receive commands from
int isBoundToClient = 0;

void setCommandBindAddress()
{
	//Set the bind address to the last address received from
	strcpy( commandBindAddress, lastRXAddress );

	//Init the TX command socket with the new bind address
	initUDPCmdSend( commandBindAddress, UDP_RESPOND_PORT );

	isBoundToClient = 1;
}

void initUDPCtrlSend( char * sendtoIP, unsigned short sendtoPort )
{
	tx_control_socketfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);	// Create socket for sending
	memset( &tx_ctrl_addrin, 0, sizeof( tx_ctrl_addrin ) );			// Zero out structure
	tx_ctrl_addrin.sin_family = AF_INET;							// Internet address family
	tx_ctrl_addrin.sin_addr.s_addr = inet_addr( sendtoIP );			// Destination IP address
	tx_ctrl_addrin.sin_port = htons(sendtoPort);					// Destination port
}

void initUDPCmdSend( char * sendtoIP, unsigned short sendtoPort )
{
	tx_command_socketfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);	// Create socket for sending
	memset( &tx_cmd_addrin, 0, sizeof( tx_cmd_addrin ) );			// Zero out structure
	tx_cmd_addrin.sin_family = AF_INET;								// Internet address family
	tx_cmd_addrin.sin_addr.s_addr = inet_addr( sendtoIP );			// Destination IP address
	tx_cmd_addrin.sin_port = htons(sendtoPort);						// Destination port
}

void UDPBindSend( char * data, int len )
{
	if ( tx_command_socketfd >= 0 && isBoundToClient )
	{
		sendto( tx_command_socketfd, data, len, 0, ( struct sockaddr * )&tx_cmd_addrin, sizeof( tx_cmd_addrin ) );
	}
}

void UDPCtrlSend( char * data )
{
	if(tx_control_socketfd >= 0)
	{
		sendto( tx_control_socketfd, data, strlen(data), 0, ( struct sockaddr * )&tx_ctrl_addrin, sizeof( tx_ctrl_addrin ) );
	}
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

int checkUDPReady( char * udpBuffer, int * p_socket )
{
	int bytesAv = 0;

	/* If there is data to be read on the socket bring it in and capture the source IP */
	if( ioctl( *p_socket, FIONREAD, &bytesAv ) > 0 || bytesAv > 0 )
	{
		int udpMsgLen = 0;
		struct sockaddr_in rx_from_addr;
		socklen_t len = sizeof( rx_from_addr );
		//Receive UDP data
		udpMsgLen = recvfrom( *p_socket, udpBuffer, MAXMESSAGESIZE, 0, ( struct sockaddr * )&rx_from_addr, &len );
		udpBuffer[ udpMsgLen ] = 0; //Null terminate UDP RX string

		//Get address from this received packet
		char thisRXaddress[16] = {0};
		sprintf( thisRXaddress, "%s", inet_ntoa( rx_from_addr.sin_addr ) );

		//If this RX address does not match the last RX address && is a control packet...
		if ( p_socket == &rx_control_socketfd && memcmp( lastRXAddress, thisRXaddress, sizeof(lastRXAddress) ) )
		{
			UDPCloseCtrlTX(); //...close the control TX socket
			initUDPCtrlSend( thisRXaddress, UDP_RESPOND_PORT ); //and re-open with the address we need to respond to
		}

		//Store the last RX address
		strcpy( lastRXAddress, thisRXaddress );

		return 1;
	}

	return 0;
}

void UDPCloseTX()
{
	close( tx_command_socketfd );
	tx_command_socketfd = -1;
}
void UDPCloseCtrlTX()
{
	close( tx_control_socketfd );
	tx_control_socketfd = -1;
}

void* udplistener_Thread( void * arg )
{
	signal( SIGINT, udp_signal_handler );
	signal( SIGHUP, udp_signal_handler );
	signal( SIGTERM, udp_signal_handler );
	prctl(PR_SET_NAME, "updlistener", 0, 0, 0);

	initListener( UDP_COMMAND_PORT, &rx_command_socketfd, &rx_cmd_addrin );
	initListener( UDP_CONTROL_PORT, &rx_control_socketfd, &rx_ctrl_addrin );

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
			if ( isBoundToClient && !memcmp( lastRXAddress, commandBindAddress, sizeof( lastRXAddress ) ) )
			{
				(*commandFunctionPtr)(incomingUDP);
			}
			bzero( incomingUDP, sizeof( incomingUDP ) );
		}
	}

	return NULL;
}

#endif