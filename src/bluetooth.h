#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <strings.h>

#include <sys/ioctl.h>

#include <sys/prctl.h>

pthread_t BTlistenerThread;

void (*commandFunctionPtr)(char *);

/*
BlueZ5 working method for SPP - Tested on Android, Linux, Windows:
//Setup 
	Edit /etc/systemd/system/bluetooth.service
		Change ExecStart=/usr/lib/bluez5/bluetooth/bluetoothd --compat
	systemctl daemon-reload
	systemctl restart bluetooth
//Start bluetooth and add Serial Port to Service Discovery Protocol
	rfkill unblock bluetooth
	sdptool add --channel=1 SP
//Start agent to auto accept connections
	simple-agent-renee
//Once paired with agent we can accept connections via RFCOMM
	rfcomm listen /dev/rfcomm0 1
	read /dev/rfcomm0 once connected
*/

int * isRunning;
// Define the exit signal handler
void bluetooth_signal_handler(int signum)
{
	(*isRunning) = 0;
}

void BluetoothInit( void * p_cmdFuncPtr, int * p_running )
{
	commandFunctionPtr = p_cmdFuncPtr;
	isRunning = p_running;
	
	system( "/home/root/EddieBalance/extras/bluetooth/./runOnceBluetoothSetup" );
	system( "/home/root/download/bluez-5.28/test/./simple-agent-renee &" );
}

int file_exist (char *filename)
{
  struct stat   buffer;   
  return (stat (filename, &buffer) == 0);
}

int checkBTReady( int * p_fd )
{
	int bytesAv = 0;

	/* If there is data to be read on the socket bring it in and capture the source IP */
	if( ioctl( *p_fd, FIONREAD, &bytesAv ) > 0 || bytesAv > 0 )
	{
		return 1;
	}
	
	return 0;
}

void* BTlistener_Thread( void * arg )
{  
	signal( SIGINT, bluetooth_signal_handler );
	signal( SIGHUP, bluetooth_signal_handler );
	signal( SIGTERM, bluetooth_signal_handler );
	prctl(PR_SET_NAME,"bluetoothlistener",0,0,0);

	while( (*isRunning) )
	{
		printf( "Eddie::BT:: Starting RFCOMM to receive a new connection..\r\n" );
		
		system( "rfcomm listen /dev/rfcomm0 1 &" );
	
		printf( "Waiting for /dev/rfcomm0\r\n" );
		while ( !file_exist ("/dev/rfcomm0") && (*isRunning) )
		{	
			sleep(1);
		}
	
		int pipe = open( "/dev/rfcomm0", O_RDONLY );
		if (pipe == -1 ) 
		{
			printf("Eddie::BT: ERROR: Unable to open /dev/rfcomm0 =(\r\n");
			return NULL;
		}
		printf("Eddie::BT: SUCCESS: Connected to /dev/rfcomm0 =)\r\n" );
	
		char readBuffer[64] = {0};
		while ( file_exist ("/dev/rfcomm0") && (*isRunning) )
		{
			if ( checkBTReady( &pipe ) )
			{
				read( pipe, readBuffer, sizeof(readBuffer) );
				printf( "Eddie::BT: Read data: %s\r\n", readBuffer );
(*commandFunctionPtr)(readBuffer);
				bzero( readBuffer, sizeof( readBuffer ) );
			}
			else usleep( 20000 );
		}
		close( pipe );
	}
	
system("killall simple-agent-renee");
	
	return NULL;
}

#endif //--BLUETOOTH_H