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

#include <errno.h>
#include <sys/wait.h>

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
	struct stat sts;
	if (stat(filename, &sts) == -1 && errno == ENOENT) {
	  return 0;
	}
	return 1;
}

int checkBTReady( int * p_fd )
{
	int bytesAv = 0;

	/* If there is data to be read on the socket bring it in and capture the source IP */
	if( ioctl( *p_fd, FIONREAD, &bytesAv ) > 0 || bytesAv > 0 )
	{
		return 1;
	} else if ( errno == EBADF )
	{
printf("EBADF detected.\r\n");
	}
	
	return 0;
}
#include <sys/stat.h>
int checkPID( pid_t p_pid )
{
	struct stat sts;
	char c_proc[64] = {0};
	sprintf( c_proc, "/proc/%d", p_pid );
	if (stat(c_proc, &sts) == -1 && errno == ENOENT) {
	  printf( "The child PID no longer exists!\r\n" );
	  return 0;
	}
	/*
	if ( !kill( p_pid, 0 ) )
	{
		//printf( "PID kill was < 1\r\n" );
		if ( errno == ESRCH )
		{
			printf( "PID was ESRCH\r\n" );
			return -1;
		}
	}
	*/
	return 1;
}

pid_t pid;
void forktest()
{
	// create child process
	pid = fork();
	switch (pid)
	{
		case -1: // error
			printf( "Unable to fork() boooo\r\n" );
			exit(1);

		case 0: // child process
			printf( "I'm the child and I'm starting rfcomm\r\n" );

			system("rfcomm listen /dev/rfcomm0 1"); // run the command
		
			printf( "The child has returned from rfcomm\r\n" );

			exit(1);

		default: // parent process, pid now contains the child pid
			printf( "I'm the parent %d my child is %d\r\n", getpid(), pid );

		break;
	}
}
void* BTlistener_Thread( void * arg )
{  
	signal( SIGINT, bluetooth_signal_handler );
	signal( SIGHUP, bluetooth_signal_handler );
	signal( SIGTERM, bluetooth_signal_handler );
	prctl(PR_SET_NAME,"bluetoothlistener",0,0,0);

	//Prevent zombies (from eating our brains) and so we can monitor the termination of the child PID running RFCOMM listen
	struct sigaction sigchld_action = {
	  .sa_handler = SIG_DFL,
	  .sa_flags = SA_NOCLDWAIT
	};
	sigaction(SIGCHLD, &sigchld_action, NULL);
	
	printf( "Eddie::BT:: Starting RFCOMM to receive new connections..\r\n" );
	system("rfcomm watch /dev/rfcomm0 1 &"); //Start RFCOMM in watch mode so we don't have to restart RFCOMM.. kernel panic still exists =(
	
	while( (*isRunning) )
	{
		//printf( "Eddie::BT:: Starting RFCOMM to receive a new connection..\r\n" );
		//forktest();
	
		printf( "Waiting for /dev/rfcomm0...\r\n" );
		while ( !file_exist ("/dev/rfcomm0") && (*isRunning) )
		{	
			sleep(1);
		}
		printf( "...I see /dev/rfcomm0 so I'm attempting to open\r\n" );
	
		printf( "Opening /dev/rfcomm0...\r\n" );
		FILE * fStream = fopen( "/dev/rfcomm0", "rw" );
		if ( fStream == NULL ) 
		{
			printf("Eddie::BT: ERROR: Unable to open /dev/rfcomm0 =(\r\n");
			return NULL;
		}
		printf("...Opened /dev/rfcomm0 SUCCESS =)\r\n" );
	 	
		char readBuffer[64] = {0};
		while ( (*isRunning) && !feof(fStream) && fStream != NULL )
		{
			fgets( readBuffer, sizeof(readBuffer), fStream );
			printf( "Eddie::BT: Read data: %s\r\n", readBuffer );
			bzero( readBuffer, sizeof( readBuffer ) );
		}
		printf( "End while reading /dev/rfcomm fopen\r\n" );
		
		sleep(3);
system( "cat /dev/rfcomm0" ); //TODO: dev/rfcomm0 does not go away in the filesystem on hangup.. until you cat w/error expected: cat: can't open '/dev/rfcomm0': No such device
		sleep(3);
		
		if ( fStream != NULL )
		{
			printf( "fStream was not NULL.. closing..\r\n" );
fclose( fStream );
			printf( "fStream close didn't crash me.\r\n" );
		}

		/*
		while ( file_exist ("/dev/rfcomm0") //checkPID( pid )// && (*isRunning) )
		{
			
			if ( checkBTReady( &fStream ) )
			{
				read( fStream, readBuffer, sizeof(readBuffer) );
				printf( "Eddie::BT: Read data: %s\r\n", readBuffer );
(*commandFunctionPtr)(readBuffer); //Execute!
				bzero( readBuffer, sizeof( readBuffer ) );
			}
			else usleep( 20000 );
		}
		
		printf( "(5)Child PID is gone.. closing /dev/rfcomm0\r\n" );
		close( fStream );
		*/
	}
	
//system("killall simple-agent-renee");
	
	return NULL;
}

#endif //--BLUETOOTH_H