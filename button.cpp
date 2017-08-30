/**
initialise(handle);

reset = (i2cWriteByteData(handle, 0x09, 0x40));
printf("reset %d\n", reset);
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

**/

/**
int output;
output = gpioWrite(17,1);
printf("%d\n", output);
gpioDelay(1000000);
output = gpioWrite(17,0);
printf("%d\n", output);
**/


/**
for(;;) 
{	
	irqValue = gpioRead(intPin);
	gpioDelay(75);
	printf("%d\n", irqValue);
}

//unsigned int un_min, un_max, un_prev_data; //variables to calculate the on-board LED brightness that reflects the heartbeats

int i;
int n_ir_buffer_length;

n_ir_buffer_length=500;

	//read the first 500 samples and determine the signal range
	for (i=0; i< n_ir_buffer_length; i++)
	{
		while(gpioRead(intPin)==1) {printf("Hello");}; //wait until the interrupt pin asserts

	}
**/
