Console on PA2 (tx only)  115200@8n1

1. Reads the id value of the CS43L22 connected to a STM32F4 Discovery board
1. Writes 0xff to the "pass through volume A register"
1. reads that value, shifts it right (>>1) and writes it again

Demonstrates how to do fully blocking i2c standard mode reads/writes,
as required by _this_ i2c peripheral.  Largely a copy of the I2C portion
of ST's "stm32f4_discovery_audio_codec.c" file, from the "Utilities"
directory of the STM32F4 Discovery FW V1.1.0 download.

Not in any way a full demonstration of the codec, or i2s.
See also the F4 version of this demo.

Recommended wiring:
* disconnect USB on F4 disco
* Connect grounds
* Connect "5V" pin of F4 disco to EXT_5V pin on L1 Disco
* connect PB6 of the F4 Disco to PB8 of the L1 Disco (SCL)
* connect PB9 of the F4 Disco to PB9 of the L1 Disco (SDA)

(i2c remap used on L1 side, as the LEDs are on the normal pins)

example output:

    hi guys!
    raw res = 0xe3 Codec is 0x1c, revision 3
    Passthrough vol A was: 0xff, Shifting and writing back...
    Passthrough vol A was: 0x7f, Shifting and writing back...
    Passthrough vol A was: 0x3f, Shifting and writing back...
    Passthrough vol A was: 0x1f, Shifting and writing back...
    Passthrough vol A was: 0xf, Shifting and writing back...
    Passthrough vol A was: 0x7, Shifting and writing back...
    Passthrough vol A was: 0x3, Shifting and writing back...
    Passthrough vol A was: 0x1, Shifting and writing back...

