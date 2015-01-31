#ifndef MOTORDRIVER
#define MOTORDRIVER

/* 
#define PWM0 "GP12" //Left Motor
#define PWM1 "GP13"
#define AIN1 "GP48" //Left Motor
#define AIN2 "GP47" //Left Motor
#define BIN1 "GP15"
#define BIN2 "GP14"
#define STBY "GP49" //Low Standby Endable
*/

//CCW Drive = IN1 Low , IN2 High, PWM speed
//CW  Drive = IN1 High, IN2 Low , PWM speed
//PWM Low will enable breaking

//http://www.emutexlabs.com/project/215-intel-edison-gpio-pin-multiplexing-guide
//https://www.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf
//https://www.sparkfun.com/products/13043

#define STOP	2
#define FORWARD 1
#define REVERSE 0

int motor_driver_enable();
void motor_driver_disable();

void set_motor_direction_left ( char p_direction );
void set_motor_direction_right( char p_direction );

void set_motor_speed_left ( float p_speed );
void set_motor_speed_right( float p_speed );

#endif //MOTORDRIVER
