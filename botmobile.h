#ifndef _botmobile_H
#define _botmobile_H
#include <cmath>
#include "io430.h"

// definitions for functions
#define MOTOR_A 0
#define MOTOR_B 1
#define RED_LED 2
#define GREEN_LED 3
#define BUMPER 4
#define REFLECT_1 5
#define REFLECT_2 6
#define BUTTON 7

// MSP 430 configuration and motor controller initialization
void initialize();

// timing function prototypes
void wait1Msec(int t);
void wait10Msec(int t);
void wait1Sec(int t);

// motor functions
void setMotor(int motorNum, int speed);

// get current value of sensor
int getSensor(int sensor);

// LED functions
void LEDoff(int led);
void LEDon(int led);
void LEDnum(int led);

//elapsed time
void resetTimer();
int time1();
int time10();
int time100();


/*
    Internal definitions and prototypes
*/

// #define _RESET_MOT_CNTL

// Port directions
#define _INPUT (unsigned char)0
#define _OUTPUT (unsigned char)1

// Signal definitions
#define _LOW (unsigned char)0
#define _HIGH (unsigned char)1

// Clock choices for 16MHz CPU clock
#define _BCS_CLK  CALBC1_1MHZ
#define _DCO_CLK  CALDCO_1MHZ

// function loop consumes 2 cycles plus overhead
#define _US_10_TICK  6L
#define _MS_TICK  996L

/*
   MSP 430 pin definitions
   P1.0 RED LED
   P1.3 MSP 430 launchpad push button
   P1.4 ADC 4 connected to reflectance sensor
   P1.5 ADC 5 connected to reflectance sensor
   P1.6 GREEN LED
   P2.2, P2.1, P2.0 light bar LED to light (before decode), MSB is P2.2
   P2.3 output connected to motor reset pin  (no longer connected as of v2.1)
   P2.4 output connected to motor serial pin
   P2.5 input connected to bumper microswitch
*/

#define _ADC_0 4
#define _ADC_1 5
#define _LED_SHIFT 0
#define _LED_MASK (unsigned char)0x07
#define _P_MOTOR P4
#define _P_SWITCH P5

// motor communication definitions
#define _MOTORA (unsigned char)0x00
#define _MOTORB (unsigned char)0x02
#define _FWD (unsigned char)0x00
#define _REV (unsigned char)0x01
#define _START_BYTE (unsigned char)0x80
#define _CNTL_DEVICE (unsigned char)0x00
#define _SETUP_DEVICE (unsigned char)0x02 //motor controller reset byte
#define _DEFAULT_SETUP (unsigned char)0x00 //two-motor mode, respond to motor 0 and 1 

// motor communication settings: 9600 baud, 8 bits, no parity, 1 stop bit
#define _BIT_PERIOD 40 //2400 baud to increase serial robustness
#define _EIGHT_BITS 8
#define _SEVEN_BITS 7
#define _ONE_STOP_BIT 1
#define _TWO_STOP_BIT 2

// Reflectance sensor maximum setting for sensor and motor
#define _ADC10MAX 1023
#define _MOTORMAX 255

// Valid LED numbers
#define _LED_MIN 0
#define _LED_MAX 7

//elapsed time
#define _INT_PERIOD 500 //500ms period on our timer interrupts
#define _TIMER_COUNTMAX 62500 //counting to 62500 at 125KHz gives us 500ms/interrupt

// timing function prototypes
void wait10Usec(int t);

// sensor functions
void _initADC();

// motor functions
void _initMotorController();
void _sendMotorChar(unsigned char b);

#ifdef _RESET_MOT_CNTL
void _resetMotorController();
#endif

//elapsed time
void _elapsedEnable();
unsigned long _millis();

#endif
