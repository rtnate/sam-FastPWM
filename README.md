# sam-FastPWM
Arduino FastPWM library that supports the SAMD platform.  Allows for faster updating of PWM duty cycle in interrupts.

## Why
Calls to the Arduino core analogWrite() function take almost 5 microseconds.  
This function automatically takes care of pin initialization and is very robust and safe while sacrificing speed.  

This library cuts the processing time under 3 microseconds which makes it better for use in interrupts.  

The speed advantage comes with some memory usage (13 bytes).  However, the ample memory on the SAMD series makes for any easy sacrifice, especially when trying to update multiple PWM duty cycles from a
time-sensitive interrupt
## Usage

The library provides the class FastPWMPin to create and manipulate PWM pins.

It can be added from the Arduino library manager or one only need to include <FastPWM.h>.

#### Pin Creation
Create a pin by providing the FastPWMPin constructor with the Arduino pin number for the desired pin.

The following creates a manager pin on Arduino pin #5.
```
FastPWMPin pwmPin(5);
```

#### Initialization
Prepare the pin for PWM by calling init().
```
pwmPin.init();
```
This will initialize all the timers cache some variables for manipulating the duty cycle.

#### Set the duty cycle
The pin duty cycle is set with ```update(newDutyCycle);```.

The new duty cycle should be a 16 bit value.  0 being 0% and 0xFFFF 100%.
For example, to set a 50% duty cycle:
```
pwmPin.update(0x7FFF);
```


