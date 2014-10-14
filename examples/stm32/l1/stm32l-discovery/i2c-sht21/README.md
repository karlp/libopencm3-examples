Console on PA2 (tx only)  115200@8n1

1. Eventually, reads the sht21 temp/humidity


Recommended wiring:
* PB8 to SCL of SHT21
* PB9 to SDA of SHT21
* VDD/anything to VDD of SHT21
* Grounds
* i2c pullups

(i2c remap used on L1 side, as the LEDs are on the normal pins)

example output:

	hi guys! this is: i2c-sht21.c
	raw res = 0x3a
	temp resolution is in 14 bits
	battery status: good

