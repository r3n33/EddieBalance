#include "imu.h"
#include "mraa.h"
#include "math.h"

#define MAX_BUFFER_LENGTH 512
#define GYRO_I2C_ADDR 0x6B
#define XM_I2C_ADDR 0x1D

uint8_t rx_tx_buf[MAX_BUFFER_LENGTH];
mraa_i2c_context i2c;

float temp;
float mx,my,mz;
float ax,ay,az;
float gx,gy,gz;
    
float i2cHeading,i2cPitch,i2cRoll;
    
void sendi2c(unsigned int address, unsigned int reg, unsigned char tosend)
{
	mraa_i2c_address(i2c, address);
  rx_tx_buf[0] = reg;
  rx_tx_buf[1] = tosend;
  mraa_i2c_write(i2c, rx_tx_buf, 2);
}

char readi2c(int address, int reg, int count)
{
	int i=0;
	mraa_i2c_address(i2c, address);
  for (i = 0; i < count; i++) {
      rx_tx_buf[i] = mraa_i2c_read_byte_data(i2c, reg+i);
  }
	if(count==1)return rx_tx_buf[0];
	return 0;
}

void readGyro()
{
	readi2c(GYRO_I2C_ADDR, OUT_X_L_G, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx = (float)((short)(rx_tx_buf[1] << 8) | rx_tx_buf[0])*0.06103515625;//0.007476806640625; // Store x-axis values into gx
	gy = (float)((short)(rx_tx_buf[3] << 8) | rx_tx_buf[2])*0.06103515625;//0.007476806640625; // Store y-axis values into gy
	gz = (float)((short)(rx_tx_buf[5] << 8) | rx_tx_buf[4])*0.06103515625;//0.007476806640625; // Store z-axis values into gz
	//TODO fix orientation
}

void readAccel()
{
	readi2c(XM_I2C_ADDR, OUT_X_L_A, 6); // Read 6 bytes, beginning at OUT_X_L_G
	float tax = (float)((short)(rx_tx_buf[1] << 8) | rx_tx_buf[0])*0.00006103515625; // Store x-axis values into gx
	float tay = (float)((short)(rx_tx_buf[3] << 8) | rx_tx_buf[2])*0.00006103515625; // Store y-axis values into gy
	float taz = (float)((short)(rx_tx_buf[5] << 8) | rx_tx_buf[4])*0.00006103515625; // Store z-axis values into gz
	
	ax = taz;
	ay = tay;
	az = tax;
	
	readi2c(XM_I2C_ADDR, OUT_TEMP_L_XM, 2);
	temp = (float)((short)(rx_tx_buf[1] << 8) | rx_tx_buf[0]);
}

void readMag()
{
	readi2c(XM_I2C_ADDR, OUT_X_L_M, 6); // Read 6 bytes, beginning at OUT_X_L_G
	float tmx = (float)((short)(rx_tx_buf[1] << 8) | rx_tx_buf[0])*0.00006103515625; // Store x-axis values into gx
	float tmy = (float)((short)(rx_tx_buf[3] << 8) | rx_tx_buf[2])*0.00006103515625; // Store y-axis values into gy
	float tmz = (float)((short)(rx_tx_buf[5] << 8) | rx_tx_buf[4])*0.00006103515625; // Store z-axis values into gz
	
	mx = tmz;
	my = tmy;
	mz = tmx;
}

void readSensors()
{
	readGyro();
	readAccel();
	readMag();
}

void getOrientation()
{
	readGyro(); //Don't forget me says the gyros!
  // Grab an acceleromter and magnetometer reading.
 	readAccel();
	readMag();

  float const PI_F = 3.14159265F;

  // i2cRoll: Rotation around the X-axis. -180 <= i2cRoll <= 180                                          
  // a positive i2cRoll angle is defined to be a clockwise rotation about the positive X-axis                                                                                                          
  //                    y                                                                           
  //      i2cRoll = atan2(---)                                                                         
  //                    z                                                                           
  // where:  y, z are returned value from accelerometer sensor                                      
  i2cRoll = (float)atan2(ay, az);

  // i2cPitch: Rotation around the Y-axis. -180 <= i2cRoll <= 180                                         
  // a positive i2cPitch angle is defined to be a clockwise rotation about the positive Y-axis                                                                                                   
  //                                 -x                                                             
  //      i2cPitch = atan(-------------------------------)                                             
  //                    y * sin(i2cRoll) + z * cos(i2cRoll)                                                                                                                                             
  // where:  x, y, z are returned value from accelerometer sensor                                   
  if (ay * sin(i2cRoll) + az * cos(i2cRoll) == 0)    	i2cPitch = ax > 0 ? (PI_F / 2) : (-PI_F / 2);
  else			    																i2cPitch = (float)atan(-ax / (ay * sin(i2cRoll) + az * cos(i2cRoll)));

  // i2cHeading: Rotation around the Z-axis. -180 <= i2cRoll <= 180                                       
  // a positive i2cHeading angle is defined to be a clockwise rotation about the positive Z-axis                                                                                                   
  //                                       z * sin(i2cRoll) - y * cos(i2cRoll)                            
  //   i2cHeading = atan2(--------------------------------------------------------------------------)  
  //                    x * cos(i2cPitch) + y * sin(i2cPitch) * sin(i2cRoll) + z * sin(i2cPitch) * cos(i2cRoll))                                                                                  
  // where:  x, y, z are returned value from magnetometer sensor                                    
//  i2cHeading = (float)atan2(mz * sin(i2cRoll) - my * cos(i2cRoll), mx * cos(i2cPitch) + my * sin(i2cPitch) * sin(i2cRoll) + mz * sin(i2cPitch) * cos(i2cRoll));

  // Convert angular data to degree 
  i2cRoll 	= - i2cRoll * 180.0 / PI_F;
  i2cPitch 	= - i2cPitch * 180.0 / PI_F;
  i2cHeading = - i2cHeading * 180.0 / PI_F;

}

void imuinit()
{
	mraa_init();
	i2c = mraa_i2c_init(1);
	
	sendi2c( GYRO_I2C_ADDR, FIFO_CTRL_REG_G, 0 );
	sendi2c( GYRO_I2C_ADDR, CTRL_REG1_G, 0xFF ); //??unknown config??
	sendi2c( GYRO_I2C_ADDR, CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
	sendi2c( GYRO_I2C_ADDR, CTRL_REG4_G, 0x30 ); // Set scale to 2000 dps
	sendi2c( GYRO_I2C_ADDR, CTRL_REG5_G, 0x00 ); // FIFO Disabled, HPF Disabled
	
	sendi2c( XM_I2C_ADDR, FIFO_CTRL_REG, 0 );
	sendi2c( XM_I2C_ADDR, CTRL_REG1_XM, 0xFF );
	sendi2c( XM_I2C_ADDR, CTRL_REG2_XM, 0x00); 
	sendi2c( XM_I2C_ADDR, CTRL_REG4_XM, 0x30 );
	
	sendi2c( XM_I2C_ADDR, CTRL_REG5_XM, 0x94);
	sendi2c( XM_I2C_ADDR, CTRL_REG6_XM, 0x00);
	sendi2c( XM_I2C_ADDR, CTRL_REG7_XM, 0x00);
/*
	return;
	
  while(1)
	{
		readGyro();
		readAccel();
		readMag();
		
		printf("gx:%6.2f gy:%6.2f gz:%6.2f  ax:%6.2f ay:%6.2f az:%6.2f  mx:%6.2f my:%6.2f mz:%6.2f  temp:%0.0f\n",gx,gy,gz,ax,ay,az,mx,my,mz,temp);
		usleep(20000);
	}
	*/
}


