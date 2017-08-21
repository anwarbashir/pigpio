# max30102-pigpio

This is work in progress!

This interface program is being developed for a maxrefdes117#:Heart-Rate and Pulse-Oximetry monitor.
Device details are available at //https://www.maximintegrated.com/en/design/reference-design-center/system-board/6300.html

Under the Design Resources Firmware, quick start guides and demo programs are available for arduino and mbed platforms.

However, the Raspberry Pi is not supported and I think this would be a useful addition.

I should point out I am a novice and this is my first attempt at devloping a program to interface with a sensor, so bear with me. 
Also I wish to thank Naing Ye Aung, Applications Engineer at MamimIntegrated for his
assistance in helping me develop this program.

Before starting you should inspect the datasheet available https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf

The main component of the device is an integrated LED and IR LED and an on-chip temperature sensor.

I think that the basic functionality is that the device generates an interupt, when this interupt is low we can read the data in the FIFO.
The data contains both the LED and an IR LED and needs to be split. There are two Interupt registers that set the sources of the interupts, 
i.e. conditions under which an interupt will be triggered such as when FIFO is almost full or when new FIFO data data is ready. More 
information can be found on page 12 of the datasheet. For example, if you have A_FULL_EN set to 1, there will be an interupt every time
there are a certain number of entries in the FIFO 

The device is equipped with an IC2 interface. We need to connect 5 pins to the Rapberry Pi;

GND, INT, SCL, SDA, VIN (3,3 volts), there is a step down chip on the break out board.

Okay, assuming you have connected to the Raspberry Pi, we need need to check that the Raspberry Pi can see the device.

To do this ensure that the IC2 interface is enabled using raspi-config, then enter;

i2cdetect -y 0 or i2cdetect -y 1 (depends if you have earlier or later version of Raspberry Pi)

This command will display a table and within that table the device id will be displayed. In the case of max301002 it should be 57.

If the number does not appear check your connections and ensure that your device is not faulty.

Once you have this number we can proceed to developing/implementing the program.

But before starting lets create some pseudo-code that we intend to follow. 

Step 1. Sanity check 
This will ensure that we have the product id and revision id (optionally) and thereby some confidence that our program is communicating with the device

Step 2. Intialize the device.
In the mbed code there is a init() function in the MAX30102.cpp file, we can use this as a guide to intialise the device.

Step 3. Loop
Keep checking for when the IRQ goes low and when it does read the FIFO
