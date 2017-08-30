#include <stdio.h>
#include <iostream>
#include <errno.h>
#include <pigpio.h>

#include "MAX30102.h"

using namespace std;

#define MAX_BRIGHTNESS 255

//lookup table http://www.raspberry-projects.com/pi/programming-in-c/memory/variables

unsigned int aun_ir_buffer[500]; //IR LED sensor data
int n_ir_buffer_length;    //data length
unsigned int aun_red_buffer[500];    //Red LED sensor data
int n_sp02; //SPO2 value
signed char ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int n_heart_rate;   //heart rate value
signed char ch_hr_valid;    //indicator to show if the heart rate calculation is valid
unsigned char uch_dummy;

int main()
{
int part, revision;

//const int buttonPin = 22;
//const int intPin = 17;


//unsigned int un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
int i;
//int n_brightness;
//float f_temp;

if (gpioInitialise() < 0)
	printf("pigpio initialisation failed.\n");
else
	printf("pigpio initialised okay.\n");
//gpioWrite(17, 0);

int handle;
handle = i2cOpen(1, 0x57, 0);
printf("%d\n", handle);

part = i2cReadByteData(0, REG_PART_ID);
printf("%#010x\n", part);

revision = i2cReadByteData(0, REG_REV_ID);
printf("%#010x\n", revision);

//Main program sequence begins here...
maxim_max30102_reset();

//read and clear status register
maxim_max30102_read_reg(0, &uch_dummy);

maxim_max30102_init();

//n_brightness=0;
//un_min=0x3FFFF;
//un_max=0;
  
n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps

	//read the first 500 samples and determine the signal range
	for (i=0; i< n_ir_buffer_length; i++)
	{
		while(gpioRead(17)==1) {printf("Hello");}; //wait until the interrupt pin asserts

	}


}// end main

bool maxim_max30102_write_reg(unsigned char uch_addr, unsigned char uch_data)
{
  char ach_i2c_data[2];
  ach_i2c_data[0]=uch_addr;
  ach_i2c_data[1]=uch_data;
  
  //if(i2c.write(I2C_WRITE_ADDR, ach_i2c_data, 2, false)==0)
  //if (i2cWriteWordData(0, uch_addr, uch_data)==0)
  if(i2cWriteI2CBlockData(0, uch_addr, ach_i2c_data, 2)==0)
    return true;
  else
    return false;
}

bool maxim_read_fifo()
{
	printf("Read FIFO????\n");
	return true;
}

bool maxim_max30102_read_reg(unsigned char uch_addr, unsigned char *puch_data)
{
  char ch_i2c_data;
  ch_i2c_data=uch_addr;
  
  //if(i2c.write(I2C_WRITE_ADDR, &ch_i2c_data, 1, true)!=0)
  if(i2cWriteI2CBlockData(0, uch_addr, &ch_i2c_data, 1)!=0)
    return false;
  //if(i2c.read(I2C_READ_ADDR, &ch_i2c_data, 1, false)==0)
  if(i2cReadI2CBlockData(0, uch_addr, &ch_i2c_data, 1)==0)
  {
    *puch_data=(unsigned char) ch_i2c_data;
    return true;
  }
  else
    return false;
}


bool maxim_max30102_init()
{
	printf("I2C Inititialised???\n");
	if(!i2cWriteByteData(0, REG_INTR_ENABLE_1, 0xc0));		//INTR setting
		return false;
	if(!i2cWriteByteData(0, REG_INTR_ENABLE_2, 0x00));
		return false;
	if(!i2cWriteByteData(0, REG_FIFO_WR_PTR, 0x00));		//FIFO_WR_PTR[4:0]
		return false;
	if(!i2cWriteByteData(0, REG_OVF_COUNTER, 0x00));		//OVF_COUNTER[4:0]
		return false;
	if(!i2cWriteByteData(0, REG_FIFO_RD_PTR, 0x00));		//FIFI_RD_PTR[4:0]
		return false;
	if(!i2cWriteByteData(0, REG_FIFO_CONFIG, 0x0f));
		return false;
	if(!i2cWriteByteData(0, REG_MODE_CONFIG, 0x03));
		return false;
	if(!i2cWriteByteData(0, REG_SPO2_CONFIG, 0x27));
		return false;
	
	if(!i2cWriteByteData(0, REG_LED1_PA, 0x24));
		return false;
	if(!i2cWriteByteData(0, REG_LED2_PA, 0x24));
		return false;
	if(!i2cWriteByteData(0, REG_PILOT_PA, 0x7f));
		return false;
	return true;  
}

bool maxim_max30102_reset()
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x40))
        return false;
    else
        return true;    
}
