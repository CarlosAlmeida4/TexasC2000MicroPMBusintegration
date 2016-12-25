# TexasC2000MicroPMBusintegration
Code used to communicate with a TPS40422 controller over PMBus interface
The controller was a C2000 dps (the 28035).
Using the I2C pins 32 e 33 from that board.
I used it to control a TPS40422 evaluation board from texas (two buck controllers and the TPS to control both)
I was able to write words and bytes to device, the read didnt work (I was always getting 0xFF from device and I dont know why).
