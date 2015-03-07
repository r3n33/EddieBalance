#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <strings.h>

#include <sys/ioctl.h>

/*
BlueZ5 working method for: 
	advertising SerialPortProfile service, 
	registering agent, 
	accepting pair requests, 
	connecting rfcomm, 
	reading serial data over bluetooth

Edit /etc/systemd/system/bluetooth.service
	Change ExecStart=/usr/lib/bluez5/bluetooth/bluetoothd --compat
systemctl daemon-reload
systemctl restart bluetooth
...
rfkill unblock bluetooth
sdptool add --channel=3 SP
...
simple-agent-renee
...
rfcomm listen /dev/rfcomm0 3
read /dev/rfcomm0 once connected
*/

int * isRunning;

void BluetoothInit( int * p_running )
{
	isRunning = p_running;
	
	system( "rfkill unblock bluetooth" );
	system( "sdptool add --channel=3 SP" );
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

void BluetoothManageService()
{
	system( "rfcomm listen /dev/rfcomm0 3 &" );
	
	while ( !file_exist ("/dev/rfcomm0") && (*isRunning) )
	{
		printf( "Waiting for /dev/rfcomm0\r\n" );
		sleep(1);
	}
	
	int pipe = open( "/dev/rfcomm0", O_RDONLY );
	if (pipe == -1 ) 
	{
		printf("Eddie::BT: Didn't open /dev/rfcomm0 =(\r\n");
		return;
	}
	printf("Eddie::BT: Connected to /dev/rfcomm0 =)\r\n" );
	
	char readBuffer[64] = {0};
	while ( file_exist ("/dev/rfcomm0") && (*isRunning) )
	{
		if ( checkBTReady( &pipe ) )
		{
			read( pipe, readBuffer, sizeof(readBuffer) );
			printf( "Eddie::BT: Read data: %s\r\n", readBuffer );
			bzero( readBuffer, sizeof( readBuffer ) );
		}
		else usleep( 20000 );
	}
	
	close( pipe );
}

#endif //--BLUETOOTH_H