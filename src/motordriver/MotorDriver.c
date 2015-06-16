#include "MotorDriver.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <fcntl.h>

int pwm0, pwm1, ain1, ain2, bin1, bin2;

//Datasheet spec is 100kHz Maximum PWM switching frequency
#define PWM_PERIOD 1000000 //nano seconds for 1kHz

char c_system_command[128] = {0};

int motor_driver_enable()
{		
	//Setup GPIO Pins
	system( "echo 48 > /sys/class/gpio/export" );
	system( "echo 47 > /sys/class/gpio/export" );
	system( "echo 15 > /sys/class/gpio/export" );
	system( "echo 14 > /sys/class/gpio/export" );
	system( "echo 49 > /sys/class/gpio/export" );
	system( "echo out > /sys/class/gpio/gpio48/direction" );
	system( "echo out > /sys/class/gpio/gpio47/direction" );
	system( "echo out > /sys/class/gpio/gpio15/direction" );
	system( "echo out > /sys/class/gpio/gpio14/direction" );
	system( "echo out > /sys/class/gpio/gpio49/direction" );

	//Setup PWM0 (Left Motor)
	system( "echo mode1 > /sys/kernel/debug/gpio_debug/gpio12/current_pinmux" );
	system( "echo 0 > /sys/class/pwm/pwmchip0/export" );
	system( "echo 1000000 > /sys/class/pwm/pwmchip0/pwm0/period" ); //TODO: Consider using PWM_PERIOD
	system( "echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable" );
	//Setup PWM1 (Right Motor)
	system( "echo mode1 > /sys/kernel/debug/gpio_debug/gpio13/current_pinmux" );
	system( "echo 1 > /sys/class/pwm/pwmchip0/export" );
	system( "echo 1000000 > /sys/class/pwm/pwmchip0/pwm1/period" ); //TODO: Consider using PWM_PERIOD
	system( "echo 1 > /sys/class/pwm/pwmchip0/pwm1/enable" );
	
	//Take H Bridge out of standby mode
	system( "echo 1 > /sys/class/gpio/gpio49/value" );
			
	return 1;
}

void motor_driver_standby( char p_option )
{
	if ( p_option > 0 )
	{
		system( "echo 0 > /sys/class/gpio/gpio49/value" ); //Enter Standby
	}
	else
	{
		system( "echo 1 > /sys/class/gpio/gpio49/value" ); //Exit Standby
	}
}

void motor_driver_disable()
{
	motor_driver_standby(1);

	system( "echo 48 > /sys/class/gpio/unexport" );
	system( "echo 47 > /sys/class/gpio/unexport" );
	system( "echo 15 > /sys/class/gpio/unexport" );
	system( "echo 14 > /sys/class/gpio/unexport" );
	system( "echo 49 > /sys/class/gpio/unexport" );
	
	system( "echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable" );
	system( "echo 0 > /sys/class/pwm/pwmchip0/pwm1/enable" );
	system( "echo 0 > /sys/class/pwm/pwmchip0/unexport" );
	system( "echo 1 > /sys/class/pwm/pwmchip0/unexport" );
}

void set_motor_direction_left ( char p_direction )
{
	ain1 = open("/sys/class/gpio/gpio48/value", O_RDWR);
	ain2 = open("/sys/class/gpio/gpio47/value", O_RDWR);

	if ( p_direction == FORWARD ) //Left motor CCW
	{
		write(ain1, "0", sizeof(char)); //AIN1 Low
		write(ain2, "1", sizeof(char)); //AIN2 High
	}
	else if ( p_direction == REVERSE ) //Left motor CW
	{
		write(ain1, "1", sizeof(char)); //AIN1 High
		write(ain2, "0", sizeof(char)); //AIN2 Low
	}

	close(ain1);
	close(ain2);
}                          
void set_motor_direction_right( char p_direction )
{
	bin1 = open("/sys/class/gpio/gpio15/value", O_RDWR);
	bin2 = open("/sys/class/gpio/gpio14/value", O_RDWR);

	if ( p_direction == FORWARD ) //Right motor CW
	{
		write(bin1, "1", sizeof(char)); //BIN1 High
		write(bin2, "0", sizeof(char)); //BIN2 Low
	}
	else if ( p_direction == REVERSE ) //Right motor CCW
	{
		write(bin1, "0", sizeof(char)); //BIN1 Low
		write(bin2, "1", sizeof(char)); //BIN2 High
	}
	
	close(bin1);
	close(bin2);
}                          
                                                                                
void set_motor_speed_left ( float p_speed )
{
	if ( p_speed < 0.0f )
	{
		if ( p_speed < -100.0f ) p_speed = -100.0f;
		set_motor_direction_left( REVERSE );
	}
	else //if ( p_speed > 0.0f )
	{
		if ( p_speed > 100.0f ) p_speed = 100.0f;
		set_motor_direction_left( FORWARD );
	}

	pwm0 = open("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", O_RDWR);
	int length = sprintf(c_system_command, "%d", (int)(PWM_PERIOD * fabs(p_speed/100)));
	write(pwm0, c_system_command, length * sizeof(char));
	close(pwm0);
}                             
void set_motor_speed_right( float p_speed )
{
	if ( p_speed < 0.0f )
	{
		if ( p_speed < -100.0f ) p_speed = -100.0f;
		set_motor_direction_right( REVERSE );
	}
	else //if ( p_speed > 0.0f )
	{
		if ( p_speed > 100.0f ) p_speed = 100.0f;
		set_motor_direction_right( FORWARD );
	}

	pwm1 = open("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", O_RDWR);
	int length = sprintf(c_system_command, "%d", (int)(PWM_PERIOD * fabs(p_speed/100)));
	write(pwm1, c_system_command, length * sizeof(char));
	close(pwm1);
} 
