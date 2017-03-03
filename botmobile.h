#ifndef _botmobile_H
#define _botmobile_H
#include <cmath>
#include "io430.h"

#define MOTOR_A 0
#define MOTOR_B 1
#define RED_LED 2
#define GREEN_LED 3
#define BUMPER 4
#define REFLECT_1 5
#define REFLECT_2 6
#define BUTTON 7

void initialize();

void wait1Msec(int t);
void wait10Msec(int t);
void wait1Sec(int t);

void setMotor(int motorNum, int speed);

int getSensor(int sensor);

void LEDoff(int led);
void LEDon(int led);
void LEDnum(int led);

void resetTimer();
int time1();
int time10();
int time100();

#define _INPUT (unsigned char)0
#define _OUTPUT (unsigned char)1

#define _LOW (unsigned char)0
#define _HIGH (unsigned char)1

#define _BCS_CLK  CALBC1_1MHZ
#define _DCO_CLK  CALDCO_1MHZ

#define _US_10_TICK  6L
#define _MS_TICK  996L

#define _ADC_0 4
#define _ADC_1 5
#define _LED_SHIFT 0
#define _LED_MASK (unsigned char)0x07
#define _P_MOTOR P4
#define _P_SWITCH P5

#define _MOTORA (unsigned char)0x00
#define _MOTORB (unsigned char)0x02
#define _FWD (unsigned char)0x00
#define _REV (unsigned char)0x01
#define _START_BYTE (unsigned char)0x80
#define _CNTL_DEVICE (unsigned char)0x00
#define _SETUP_DEVICE (unsigned char)0x0
#define _DEFAULT_SETUP (unsigned char)0x00

#define _BIT_PERIOD 40
#define _EIGHT_BITS 8
#define _SEVEN_BITS 7
#define _ONE_STOP_BIT 1
#define _TWO_STOP_BIT 2

#define _ADC10MAX 1023
#define _MOTORMAX 255

#define _LED_MIN 0
#define _LED_MAX 7

#define _INT_PERIOD 500
#define _TIMER_COUNTMAX 62500

void wait10Usec(int t);

void _initADC();

void _initMotorController();
void _sendMotorChar(unsigned char b);

#ifdef _RESET_MOT_CNTL
void _resetMotorController();
#endif

void _elapsedEnable();
unsigned long _millis();

#endif
