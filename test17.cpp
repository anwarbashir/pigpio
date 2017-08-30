#include <stdio.h>
#include <iostream>
#include <errno.h>
#include <pigpio.h>

main(){
int output;

if (gpioInitialise() < 0)
	printf("pigpio initialisation failed.\n");
else
	printf("pigpio initialised okay.\n");
	
output = gpioWrite(17,1);
printf("%d\n", output);
gpioDelay(1000000);
output = gpioWrite(17,0);
printf("%d\n", output);
}
