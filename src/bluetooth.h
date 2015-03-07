#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <strings.h>

/* This works once with BlueZ 2.25 but when the application closes a kernel panic happens when wifi is received */
/* The kernel panic has been avoided with a few cruddy commands but only once and the connection could not be re-established */

/* Edison is setup with:
 * Bluez 5 removed and replaced with:
 * http://bluez.sf.net/download/bluez-libs-2.25.tar.gz
 * http://bluez.sf.net/download/bluez-utils-2.25.tar.gz
 *
 * A properly configured (pin helper): /etc/bluetooth/hcid.conf
 */

//export PATH=$PATH:/opt/bluetooth/bin:/opt/bluetooth/sbin

int * isRunning;

void BluetoothInit( int * p_running )
{
	isRunning = p_running;
	
	system( "rfkill unblock bluetooth" );
	system( "hcid" );
	system( "sdpd" );
	system( "sdptool add --channel=3 SP" );
	system( "hciconfig hci0 piscan" );
}

int file_exist (char *filename)
{
  struct stat   buffer;   
  return (stat (filename, &buffer) == 0);
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
		if ( read( pipe, readBuffer, sizeof(readBuffer) ) )
		{
			printf( "Eddie::BT: Read data: %s\r\n", readBuffer );
			bzero( readBuffer, sizeof( readBuffer ) );
		}
		else usleep( 20000 );
	}
	
	if ( 0 ) //Some combination.. likely the topmost command prevents kernel panic once
	{
		system( "bluetooth_rfkill_event &" );
		system( "killall rfcomm" );
		system( "rfkill block bluetooth" );
		system("systemctl restart wpa_supplicant");
	}
	close( pipe );
}

#endif //--BLUETOOTH_H