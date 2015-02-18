#include "MotorDriver.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <fcntl.h>

int pwm0, pwm1, ain1, ain2, bin1, bin2;

/*
#include "mraa.h"

mraa_gpio_context gpio_ain1;
mraa_gpio_context gpio_ain2;
mraa_gpio_context gpio_bin1;
mraa_gpio_context gpio_bin2;
mraa_gpio_context gpio_stby;
mraa_pwm_context pwm0;
mraa_pwm_context pwm1;
*/

//Datasheet spec is 100kHz Maximum PWM switching frequency
#define PWM_PERIOD 1000000 //nano seconds for 1kHz

char c_system_command[128] = {0};

int motor_driver_enable()
{		
	//Setup GPIO Pins
	system( "echo 48 > /sys/class/gpio/export" ); //This causes the wifi to work very very poorly until you run iwconfig.. STRANGE
	system( "echo 47 > /sys/class/gpio/export" );
	system( "echo 15 > /sys/class/gpio/export" );
	system( "echo 14 > /sys/class/gpio/export" );
	system( "echo 49 > /sys/class/gpio/export" );
	system( "echo out > /sys/class/gpio/gpio48/direction" );
	system( "echo out > /sys/class/gpio/gpio47/direction" );
	system( "echo out > /sys/class/gpio/gpio15/direction" );
	system( "echo out > /sys/class/gpio/gpio14/direction" );
	system( "echo out > /sys/class/gpio/gpio49/direction" );

	//system("systemctl restart wpa_supplicant"); //TODO: This is a temporary workaround to resolve wifi issue with GPIO 48

	/* MRAA GPIO Initialization.. Works fine but I didn't use it.
	gpio_ain1 = mraa_gpio_init(48);
	gpio_ain2 = mraa_gpio_init(47);
	gpio_bin1 = mraa_gpio_init(15);
	gpio_bin2 = mraa_gpio_init(14);
	gpio_stby = mraa_gpio_init(49);
	
	mraa_gpio_dir(gpio_ain1, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_ain2, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_bin1, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_bin2, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_stby, MRAA_GPIO_OUT);
	*/
	
	/* MRAA PWM Initialization.. DOES NOT WORK.. Will return -1 with latest mraa library
	pwm0 = mraa_pwm_init(12);
	pwm1 = mraa_pwm_init(13);
	
	if ( pwm0 == NULL )
	{
		printf("PWM0 was NULL must be some MRAA bug cause I can't fix it :(\r\n");
		return -1;
	}
	if ( pwm1 == NULL )
	{
		printf("PWM1 was NULL\r\n");
		return -1;
	}
	printf("PWM Init OK\r\n");
	mraa_pwm_period_us(pwm0, 200);
	mraa_pwm_period_us(pwm1, 200);
	printf("PWM Period OK\r\n");
	mraa_pwm_enable(pwm0, 1);
	mraa_pwm_enable(pwm1, 1);
	printf("PWM Enabed\r\n");
	*/	

	//Setup PWM0 (Left Motor)
	system( "echo mode1 > /sys/kernel/debug/gpio_debug/gpio12/current_pinmux" );
	system( "echo 0 > /sys/class/pwm/pwmchip0/export" );
	system( "echo 1000000 > /sys/class/pwm/pwmchip0/pwm0/period" ); //TODO: Consider using PWM_PERIOD
	//TESTING: system( "echo 1000000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle" ); //TODO: This will not be needed after testing
	system( "echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable" );
	//Setup PWM1 (Right Motor)
	system( "echo mode1 > /sys/kernel/debug/gpio_debug/gpio13/current_pinmux" );
	system( "echo 1 > /sys/class/pwm/pwmchip0/export" );
	system( "echo 1000000 > /sys/class/pwm/pwmchip0/pwm1/period" ); //TODO: Consider using PWM_PERIOD
	//TESTING: system( "echo 1000000 > /sys/class/pwm/pwmchip0/pwm1/duty_cycle" ); //TODO: This will not be needed after testing
	system( "echo 1 > /sys/class/pwm/pwmchip0/pwm1/enable" );
	
	//Take H Bridge out of standby mode
	system( "echo 1 > /sys/class/gpio/gpio49/value" ); //mraa_gpio_write(gpio_stby, 1);
			
	return 1;
}

void motor_driver_disable()
{
	//Put H Bridge into standby mode
	system( "echo 0 > /sys/class/gpio/gpio49/value" ); //mraa_gpio_write(gpio_stby, 0);

	//Cleaning up GPIO stuff
	/*
	mraa_gpio_close(gpio_ain1);
	mraa_gpio_close(gpio_ain2);
	mraa_gpio_close(gpio_bin1);
	mraa_gpio_close(gpio_bin2);
	mraa_gpio_close(gpio_stby);
	*/
	
	system( "echo 48 > /sys/class/gpio/unexport" );
	system( "echo 47 > /sys/class/gpio/unexport" );
	system( "echo 15 > /sys/class/gpio/unexport" );
	system( "echo 14 > /sys/class/gpio/unexport" );
	system( "echo 49 > /sys/class/gpio/unexport" );
	
	//if ( pwm0 != NULL ) mraa_pwm_close(pwm0);
	//if ( pwm1 != NULL ) mraa_pwm_close(pwm1);
	
	system( "echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable" );
	system( "echo 0 > /sys/class/pwm/pwmchip0/pwm1/enable" );
	system( "echo 0 > /sys/class/pwm/pwmchip0/unexport" );
	system( "echo 1 > /sys/class/pwm/pwmchip0/unexport" );
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

void set_motor_direction_left ( char p_direction )
{

ain1 = open("/sys/class/gpio/gpio48/value", O_RDWR);
ain2 = open("/sys/class/gpio/gpio47/value", O_RDWR);

	if ( p_direction == FORWARD ) //Left motor CCW
	{
		write(ain1, "0", sizeof(char)); //system( "echo 0 > /sys/class/gpio/gpio48/value" ); //mraa_gpio_write(gpio_ain1, 0); //AIN1 Low
		write(ain2, "1", sizeof(char)); //system( "echo 1 > /sys/class/gpio/gpio47/value" ); //mraa_gpio_write(gpio_ain2, 1); //AIN2 High
	}
	else if ( p_direction == REVERSE ) //Left motor CW
	{
		write(ain1, "1", sizeof(char)); //system( "echo 1 > /sys/class/gpio/gpio48/value" ); //mraa_gpio_write(gpio_ain1, 1); //AIN1 High
		write(ain2, "0", sizeof(char)); //system( "echo 0 > /sys/class/gpio/gpio47/value" ); //mraa_gpio_write(gpio_ain2, 0); //AIN2 Low
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
		write(bin1, "1", sizeof(char)); //system( "echo 1 > /sys/class/gpio/gpio15/value" ); //mraa_gpio_write(gpio_bin1, 1); //BIN1 High
		write(bin2, "0", sizeof(char)); //system( "echo 0 > /sys/class/gpio/gpio14/value" ); //mraa_gpio_write(gpio_bin2, 0); //BIN2 Low
	}
	else if ( p_direction == REVERSE ) //Right motor CCW
	{
		write(bin1, "0", sizeof(char)); //system( "echo 0 > /sys/class/gpio/gpio15/value" ); //mraa_gpio_write(gpio_bin1, 0); //BIN1 Low
		write(bin2, "1", sizeof(char)); //system( "echo 1 > /sys/class/gpio/gpio14/value" ); //mraa_gpio_write(gpio_bin2, 1); //BIN2 High
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

	//NOTE: The original debug method below is too slow for 100Hz
	//sprintf( (char*)c_system_command, "echo %d > /sys/class/pwm/pwmchip0/pwm0/duty_cycle", (int)(PWM_PERIOD * fabs(p_speed/100)) );
	//system( c_system_command );
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

	//NOTE: The original debug method below is too slow for 100Hz
	//sprintf( (char*)c_system_command, "echo %d > /sys/class/pwm/pwmchip0/pwm1/duty_cycle", (int)(PWM_PERIOD * fabs(p_speed/100)) );
	//system( c_system_command );
} 
