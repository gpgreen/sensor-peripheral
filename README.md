# sensor-peripheral
Firmware for ATtiny26 that uses ADC in response to SPI
requests. Intended for use with Raspberry PI boat computer project

Makefile is setup to use avr-gcc, avr-libc, and avrdude to compile and program the chip respectively. 

The fuse settings assumed in this program can be set via avrdude:
```
avrude -p attiny26 -c <your programmer here> -U lfuse:w:0xe3:m -U hfuse:w:0xf5:m
```

The compiled firmware (sensor.hex) can be programmed into the hardware using
```
avrdude -p attiny26 -c <your programmer here> -U flash:w:sensor.hex
```

This can also be done with the makefile target 'program', or `make program`

See the main source code file for how the SPI protocol works.

