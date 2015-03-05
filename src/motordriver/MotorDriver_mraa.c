#include "MotorDriver.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <fcntl.h>

#include "mraa.h"

mraa_gpio_context gpio_ain1;
mraa_gpio_context gpio_ain2;
mraa_gpio_context gpio_bin1;
mraa_gpio_context gpio_bin2;
mraa_gpio_context gpio_stby;
mraa_pwm_context pwm0;
mraa_pwm_context pwm1;


//Datasheet spec is 100kHz Maximum PWM switching frequency
#define PWM_PERIOD 1000000 //nano seconds for 1kHz

char c_system_command[128] = {0};

int motor_driver_enable()
{		
	/* MRAA GPIO Initialization..*/
	gpio_ain1 = mraa_gpio_init_raw(48);
	gpio_ain2 = mraa_gpio_init_raw(47);
	gpio_bin1 = mraa_gpio_init_raw(15);
	gpio_bin2 = mraa_gpio_init_raw(14);
	gpio_stby = mraa_gpio_init_raw(49);
	
	mraa_gpio_dir(gpio_ain1, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_ain2, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_bin1, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_bin2, MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio_stby, MRAA_GPIO_OUT);
	
	
	/* MRAA PWM Initialization.. DOES NOT WORK.. Will return -1 with latest mraa library */
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

	//Take H Bridge out of standby mode
	mraa_gpio_write(gpio_stby, 1);
			
	return 1;
}

void motor_driver_disable()
{
	//Put H Bridge into standby mode
	mraa_gpio_write(gpio_stby, 0);

	//Cleaning up GPIO stuff
	mraa_gpio_close(gpio_ain1);
	mraa_gpio_close(gpio_ain2);
	mraa_gpio_close(gpio_bin1);
	mraa_gpio_close(gpio_bin2);
	mraa_gpio_close(gpio_stby);
	
	if ( pwm0 != NULL ) mraa_pwm_close(pwm0);
	if ( pwm1 != NULL ) mraa_pwm_close(pwm1);
	
}                                   

void motor_driver_standby( char p_option )
{
	if ( p_option > 0 )
	{
		mraa_gpio_write(gpio_stby, 0); //Enter Standby
	}
	else
	{
		mraa_gpio_write(gpio_stby, 1); //Exit Standby
	}
}

void set_motor_direction_left ( char p_direction )
{
	if ( p_direction == FORWARD ) //Left motor CCW
	{
		mraa_gpio_write(gpio_ain1, 0); //AIN1 Low
		mraa_gpio_write(gpio_ain2, 1); //AIN2 High
	}
	else if ( p_direction == REVERSE ) //Left motor CW
	{
		mraa_gpio_write(gpio_ain1, 1); //AIN1 High
		mraa_gpio_write(gpio_ain2, 0); //AIN2 Low
	}
}                          
void set_motor_direction_right( char p_direction )
{
	if ( p_direction == FORWARD ) //Right motor CW
	{
		mraa_gpio_write(gpio_bin1, 1); //BIN1 High
		mraa_gpio_write(gpio_bin2, 0); //BIN2 Low
	}
	else if ( p_direction == REVERSE ) //Right motor CCW
	{
		mraa_gpio_write(gpio_bin1, 0); //BIN1 Low
		mraa_gpio_write(gpio_bin2, 1); //BIN2 High
	}
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

	/* TODO: Use MRAA to set PWM duty_cycle
	pwm0 = open("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", O_RDWR);
	int length = sprintf(c_system_command, "%d", (int)(PWM_PERIOD * fabs(p_speed/100)));
	write(pwm0, c_system_command, length * sizeof(char));
	close(pwm0);
	*/
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

	/* TODO: Use MRAA to set PWM duty_cycle
	pwm1 = open("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", O_RDWR);
	int length = sprintf(c_system_command, "%d", (int)(PWM_PERIOD * fabs(p_speed/100)));
	write(pwm1, c_system_command, length * sizeof(char));
	close(pwm1);
	*/
} 
