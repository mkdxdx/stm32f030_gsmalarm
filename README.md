# stm32f030_gsmalarm

Simple GSM alarm project that supports up to 2 trip channels, can send SMS on tripping those, and turn on stuff if it is tripped.
Made with STM32F030F4P6 microcontroller board and SIM800L module.

Usage:
1. Turn alarm on, it loads into IDLE state.
2. Once KEY_PIN becomes LOW, alarm goes into ARMING state and after ARMING_TIMEOUT it becomes ARMED.
3. Once TRIP_CH1 or TRIP_CH2 go LOW, alarm goes into TRIPPED state and waits for TRIPPED_TIMEOUT before trying to send SMS or set ACTION_PIN HIGH. After this interval SMS will be sent and output will be triggered which can be connected to siren relay etc.
4. Alarm waits for ALARM_TIMEOUT after which it will return to ARMING state or if KEY_PIN goes HIGH it will return to IDLE state.

TRIP_CHx can be connected to open drain of a PIR sensor, a reed switch or something else that will pull line to ground once triggered.
KEY_PIN can be any kind of switch that pulls line to ground once activated to arm the system.

In IDLE state, LED_STATUS blinks if any of the channels are tripped to test if circuits work properly. In ARMING LED_ARMED will blink during arming interval and then turns on in ARMED state. During transition from ARMED to TRIPPED states LED_STATUS turns on and after interval LED_ARMED and LED_STATUS will blink rapidly during notification phase.

You have to provide gsmconf.h file with format described in main.c with owner's number where notifications go.

Later I2C sensor and/or EEPROM will be added.

Schematics will be added soon after i throw everything in enclosure.

