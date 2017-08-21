#include <stdio.h>
#include <iostream>
#include <errno.h>
#include <pigpio.h>

#include "MAX30102.h"

using namespace std;

#define MAX_BRIGHTNESS 255

bool initialise(int handle)
{
	printf("I2C Inititialised???\n");
	if(!i2cWriteByteData(handle, REG_INTR_ENABLE_1, 0xc0));		//INTR setting
		return false;
	if(!i2cWriteByteData(handle, REG_INTR_ENABLE_2, 0x00));
		return false;
	if(!i2cWriteByteData(handle, REG_FIFO_WR_PTR, 0x00));		//FIFO_WR_PTR[4:0]
		return false;
	if(!i2cWriteByteData(handle, REG_OVF_COUNTER, 0x00));		//OVF_COUNTER[4:0]
		return false;
	if(!i2cWriteByteData(handle, REG_FIFO_RD_PTR, 0x00));		//FIFI_RD_PTR[4:0]
		return false;
	if(!i2cWriteByteData(handle, REG_FIFO_CONFIG, 0x0f));
		return false;
	if(!i2cWriteByteData(handle, REG_MODE_CONFIG, 0x03));
		return false;
	if(!i2cWriteByteData(handle, REG_SPO2_CONFIG, 0x27));
		return false;
	
	if(!i2cWriteByteData(handle, REG_LED1_PA, 0x24));
		return false;
	if(!i2cWriteByteData(handle, REG_LED2_PA, 0x24));
		return false;
	if(!i2cWriteByteData(handle, REG_PILOT_PA, 0x7f));
		return false;
	return true;  
}

bool read_fifo()
{
	printf("Read FIFO????\n");
	return true;
}

int handle;

int main()
{
int part, revision, clear, reset , init, value;

reset=10;
const int buttonPin = 22;

if (gpioInitialise() < 0)
	printf("pigpio initialisation failed.\n");
else
	printf("pigpio initialised okay.\n");
	
handle = i2cOpen(1, 0x57, 0);
printf("%#010x\n", handle);

part = i2cReadByteData(handle, REG_PART_ID);
printf("%#010x\n", part);

revision = i2cReadByteData(handle, REG_REV_ID);
printf("%#010x\n", revision);

//Main program sequence begins here...
initialise(handle);

reset = (i2cWriteByteData(handle, 0x09, 0x40));
printf("rest %d\n", reset);
gpioDelay(5);
reset = (i2cWriteByteData(handle, 0x21, 0x01));
gpioDelay(5);
value = i2cReadByteData(handle, 0x1F);
printf("value %d\n", value);



for (int i=0;i<6;i++)
{
	i2cWriteByteData(handle,I2C_WRITE_ADDR,REG_FIFO_DATA);
value = i2cReadByteData(handle, I2C_READ_ADDR);
printf("value %d\n", value);
}

clear = i2cWriteByteData(handle, REG_INTR_STATUS_1, 0x00);
printf("%d\n", clear);

printf("Press button to begin scan\n");
while(gpioRead(buttonPin)) {gpioDelay(75);}

init = initialise(handle);
printf("%d\n", init);


/**
int output;
output = gpioWrite(17,1);
printf("%d\n", output);
gpioDelay(1000000);
output = gpioWrite(17,0);
printf("%d\n", output);
**/

int intPin = 17;
int irqValue;

for(;;) 
{	
	irqValue = gpioRead(intPin);
	gpioDelay(75);
	printf("%d\n", irqValue);
}

//unsigned int un_min, un_max, un_prev_data; //variables to calculate the on-board LED brightness that reflects the heartbeats
/**
int i;
int n_ir_buffer_length;

n_ir_buffer_length=500;

	//read the first 500 samples and determine the signal range
	for (i=0; i< n_ir_buffer_length; i++)
	{
		while(gpioRead(intPin)==1) {printf("Hello");}; //wait until the interrupt pin asserts

	}
**/
}




bool maxim_max30102_reset()
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x40))
        return false;
    else
        return true;    
}
bool maxim_max30102_write_reg(unsigned char uch_addr, unsigned char uch_data)
{
  /*char ach_i2c_data[2];
  ach_i2c_data[0]=uch_addr;
  ach_i2c_data[1]=uch_data;
  */
  //if(i2c.write(I2C_WRITE_ADDR, ach_i2c_data, 2, false)==0)
  if ((i2cWriteByteData(handle, 0x09, 0x40)==0))
    return true;
  else
    return false;
}
