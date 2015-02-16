#ifndef ENCODER_H
#define ENCODER_H

#include<stdio.h>
#include<string.h>
#include<pthread.h>
#include<stdlib.h>
#include<unistd.h>

#include "mraa.h"

//left encoder: 183 46
//right encoder: 45 44

static volatile double position[ 2 ];
mraa_gpio_context encoderx[ 4 ];

int lastpins[ 4 ];
void EncoderInterruptA( void * args ) 
{	
	int currentpins[ 2 ];
	
	int change=0;
  currentpins[ 0 ] =  mraa_gpio_read( encoderx[ 0 ] );
  currentpins[ 1 ] =  mraa_gpio_read( encoderx[ 1 ] );
  
  if( currentpins[ 0 ] != lastpins[ 0 ] )
  {
  	if( currentpins[ 0 ] > lastpins[ 0 ] )
		{
			if( currentpins[ 1 ]  )
			{
				--change;
			}
			else
			{
				++change;
			}
		}
		else 
		{
			if( currentpins[ 1 ] )
			{
				++change;
			}
			else
			{
				--change;
			}
		}
	}
	else if( currentpins[ 1 ] != lastpins[ 1 ] )
  {
  	if( currentpins[ 1 ] > lastpins[ 1 ] )
		{
			if( currentpins[ 0 ] )
			{
				++change;
			}
			else
			{
				--change;
			}
		}
		else 
		{
			if( currentpins[ 0 ] )
			{
				--change;
			}
			else
			{
				++change;
			}
		}
	}
	
	position[ 0 ] += change;
	
	lastpins[ 0 ] = currentpins[ 0 ];
	lastpins[ 1 ] = currentpins[ 1 ];	
}

void EncoderInterruptB( void * args ) 
{		
	int currentpins[ 2 ];
	
	int change=0;
  currentpins[ 0 ] =  mraa_gpio_read( encoderx[ 2 ] );
  currentpins[ 1 ] =  mraa_gpio_read( encoderx[ 3 ] );
  
  if( currentpins[ 0 ] != lastpins[ 2 ] )
  {
  	if( currentpins[ 0 ] > lastpins[ 2 ] )
		{
			if( currentpins[ 1 ]  )
			{
				++change;
			}
			else
			{
				--change;
			}
		}
		else 
		{
			if( currentpins[ 1 ] )
			{
				--change;
			}
			else
			{
				++change;
			}
		}
	}
	else if( currentpins[ 1 ] != lastpins[ 3 ] )
  {
  	if( currentpins[ 1 ] > lastpins[ 3 ] )
		{
			if( currentpins[ 0 ] )
			{
				--change;
			}
			else
			{
				++change;
			}
		}
		else 
		{
			if( currentpins[ 0 ] )
			{
				++change;
			}
			else
			{
				--change;
			}
		}
	}
	
	position[ 1 ] += change;	
	lastpins[ 2 ] = currentpins[ 0 ];
	lastpins[ 3 ] = currentpins[ 1 ];	
}

void ResetEncoders()
{
	position[ 0 ] = position[ 1 ] = 0;
}

double GetEncoder()
{
	return ( position[ 0 ] + position[ 1 ] ) / 2;
}

void GetEncoders( double * temp )
{	
	temp[0] = position[ 0 ];
	temp[1] = position[ 1 ];
}

void GetEncoderChange( double * temp )
{	
	temp[0] = position[ 0 ];
	temp[1] = position[ 1 ];
	position[ 0 ] = position[ 1 ] = 0;
}

void Move( double distance )
{
	position[0] += distance;
	position[1] += distance;
}

void initEncoders( int a, int b, int c, int d )
{
	mraa_init();	
	position[0] = position[1] = 0;  
	encoderx[ 0 ] = mraa_gpio_init_raw( a );
	encoderx[ 1 ] = mraa_gpio_init_raw( b );
	encoderx[ 2 ] = mraa_gpio_init_raw( c );
	encoderx[ 3 ] = mraa_gpio_init_raw( d );
	
	mraa_gpio_dir( encoderx[ 0 ], MRAA_GPIO_IN );
	mraa_gpio_isr( encoderx[ 0 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptA, NULL );
	mraa_gpio_dir( encoderx[ 1 ], MRAA_GPIO_IN);
	mraa_gpio_isr( encoderx[ 1 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptA, NULL );	
	mraa_gpio_dir( encoderx[ 2 ], MRAA_GPIO_IN );
	mraa_gpio_isr( encoderx[ 2 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptB, NULL );
	mraa_gpio_dir( encoderx[ 3 ], MRAA_GPIO_IN);
	mraa_gpio_isr( encoderx[ 3 ], MRAA_GPIO_EDGE_BOTH, &EncoderInterruptB, NULL );
	
}

void CloseEncoder()
{
  mraa_gpio_close( encoderx[ 0 ] );	  
  mraa_gpio_close( encoderx[ 1 ] );
  mraa_gpio_close( encoderx[ 2 ] );	  
  mraa_gpio_close( encoderx[ 3 ] );		  
}

#endif