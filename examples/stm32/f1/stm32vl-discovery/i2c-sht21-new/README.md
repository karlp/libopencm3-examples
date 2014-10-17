Console on PA2 (tx only)  115200@8n1

1. Eventually, reads the sht21 temp/humidity


Recommended wiring:
* PB6 to SCL of SHT21
* PB7 to SDA of SHT21
* VDD/anything to VDD of SHT21
* Grounds
* i2c pullups

example output:

	hi guys! this is: i2c-sht21.c
	raw res = 0x3a
	temp resolution is in 14 bits
	battery status: good

