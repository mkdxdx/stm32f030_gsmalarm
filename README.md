# stm32f030_gsmalarm

Simple GSM alarm project that supports up to 2 trip channels, can send SMS on tripping those, and turn on stuff if it is tripped.
Made with STM32F030F4P6 microcontroller board and SIM800L module.

You have to provide gsmconf.h file with format described in main.c with owner's number where notifications go.

Later I2C sensor and/or EEPROM will be added.

Schematics will be added soon after i throw everything in enclosure.

