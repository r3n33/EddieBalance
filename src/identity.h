#ifndef IDENTITY_H
#define IDENTITY_H

#include <stdio.h>

int thisEddieSerial;

void initIdentity();
unsigned short checksum( const char * key, int len );

void initIdentity()
{
	char responseBuff[256] = {0};
	FILE * pipe = popen( "ls /dev/disk/by-uuid\n", "r" );
	if (pipe == NULL ) 
	{
		printf("Eddie::initIdentity: Invoking command failed.\r\n");
	}
	while( fgets( responseBuff + strlen(responseBuff), sizeof(responseBuff) - strlen(responseBuff), pipe ) != NULL );
	pclose( pipe );
	
	thisEddieSerial = checksum( responseBuff, strlen( responseBuff ) );
	
	//DEBUG: printf("Eddie Identity Found: %04x\r\n", thisEddieSerial);
}

unsigned short checksum( const char * key, int len )
{
	unsigned short crc = 0xFFFF;
	int i, j;
	
	for( i=0; i<len; ++i )
	{
		char data = key[i];
		crc = crc ^ ( data << 8 );
		for ( j=0; j < 8; ++j )
		{
			if ( ( crc & 0x8000 ) != 0 )
			{
				crc = ( crc << 1 ) ^ 0x1021;
			}
			else
			{
				crc <<= 1;
			}
		}
	}
	return crc;         
}


#endif //--IDENTITY_H