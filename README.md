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

Binding alarm to owner's number and/or setting delay values:
1. Restart alarm and wait until it boots (STATUS led will be lit up only). Do not turn arming state on.
2. Send an SMS to alarm's number with format "TARM:XXX,TTRIP:XXX,TALRM:XXX" where XXX is a number between 0 and 255 in seconds. TARM sets arming period, TTRIP sets tripping delay (before turning siren on), TALRM defines how long siren will sound.
3. Short CONF_PIN to ground (by default its PA14). If STATUS led blinks quikcly few times, alarm is configured and will send sms with parameters confirmation.
4. If STATUS led doesn't blink, start process over.
5. On success turn off and on power to confirm that alarm sends messages to correct number.

Schematics will be added soon.

Build looks like this https://youtu.be/tgss5XSQ9zk
