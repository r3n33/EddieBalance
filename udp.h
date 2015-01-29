

int bsock, sockfd, fd_SERIAL;
struct sockaddr_in broadcastAddr,servaddr,cliaddr;
pthread_t udplistenerThread;

void commsighandler( int sig )
{
	printf( "\nCaught sig %d.\n", sig );
	listening = 0;
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
int checkUDPReady( char * udpBuffer )
{
	int bytesAv = 0;
	if( ioctl( sockfd, FIONREAD, &bytesAv ) > 0 || bytesAv > 0 )
	{
		socklen_t len = sizeof( cliaddr );
		udpMsgLen = recvfrom( sockfd, udpBuffer, MAXMESSAGESIZE, 0, ( struct sockaddr * ) &cliaddr, &len );
		udpBuffer[ udpMsgLen ] = 0;
		
		if( startup )
		{
			if(!memcmp( udpBuffer, startupMessage, strlen( startupMessage ) ))
			{	
				selfaddr = cliaddr.sin_addr.s_addr;
				startup = 0;
			}
			else
			{
				SendStartupMessage();
			}
		}
		else if( cliaddr.sin_addr.s_addr != selfaddr )
		{
			return 1;
		}
	}
		
	return 0;
}

void* udplistener_Thread( void *arg )
{  
	printf("-- updlistener_Thread Started\n");
	signal( SIGINT, commsighandler );
	signal( SIGHUP, commsighandler );
	signal( SIGTERM, commsighandler );
	prctl(PR_SET_NAME,"updlistener",0,0,0);
	
	initBroadCast( "255.255.255.255", 6001 );
	initListener( 6001 );
	
	char incomingUDP[MAXMESSAGESIZE+1];
	
  while(listening)
  {
    usleep( 20000 );
	
		while( checkUDPReady( incomingUDP ) )
		{		
			switch(serialIncomingType)
			{ 
				case BINARY:
					for(int i=0;i<udpMsgLen;++i)
						parseIncomingBinary(incomingUDP[i],UDP);
					break;
				case CANONICAL:
					parseIncomingMessage(incomingUDP,UDP);
					break;					
			}
			incomingUDP[ 0 ] = 0;
		}
	}

  return NULL;
}

void initComms( void (*dataReceiver)(FlexMessage*), int serial )
{
	incomingData = dataReceiver;
	systemSerial = serial;
	
  pthread_create( &udplistenerThread, NULL, &udplistener_Thread, NULL );
  pthread_create( &seriallistenerThread, NULL, &seriallistener_Thread, NULL );  
  pthread_create( &transmitThread, NULL, &transmit_Thread, NULL );
}


#endif

