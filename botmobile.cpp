#ifndef _botmobile_CPP
#define _botmobile_CPP
#include <cmath>
#include "io430.h"
#include "botmobile.h"

unsigned long _timerOverflows;
void _resetMotorController();

void initialize() {
    WDTCTL = WDTPW + WDTHOLD;
    BCSCTL1 = _BCS_CLK;
    DCOCTL = _DCO_CLK;
    P1DIR_bit.P0 = _OUTPUT;
    P1DIR_bit.P6 = _OUTPUT;
    P1DIR_bit.P3 = _INPUT;
    P1REN_bit.P3 = _HIGH;
    P1OUT_bit.P3 = _HIGH;
    _initMotorController();
	#ifdef _RESET_MOT_CNTL
    _resetMotorController();
	#endif
    _initADC();
    P2DIR_bit._P_SWITCH = _INPUT;
    P2REN_bit._P_SWITCH = _HIGH;
    P2OUT_bit._P_SWITCH = _HIGH; 
    P2DIR |= _LED_MASK;
    resetTimer();
    __enable_interrupt();
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void) {
  _timerOverflows++;
}

void resetTimer() {
	TACCTL0 &= ~CCIE;
    TA0CTL_bit.TACLR = 1;
    TA0CTL_bit.TACLR = 0;
    TACCR0 = _TIMER_COUNTMAX - 1;
    TACTL = TASSEL_2 | ID_3 | MC_1 | TACLR;
    _timerOverflows = 0;
    TACCTL0 |= CCIE;
}

unsigned long _millis() {
    double time = double(TAR);
    time = time / _TIMER_COUNTMAX * _INT_PERIOD;
    return _timerOverflows * _INT_PERIOD + (unsigned long)(time);
}

int time1() {
    return int(_millis());
}

int time10() {
    return int((_millis() + 5) / 10);
}

int time100() {
    return int((_millis() + 50) / 100);
}

void wait10Usec(int t) {
    for ( ; t > 0; t--)
        __delay_cycles(_US_10_TICK);
}

void wait1Msec(int t) {
    for ( ; t > 0; t--)
        __delay_cycles(_MS_TICK);
}

void wait10Msec(int t) {
    for ( ; t > 0; t--)
        for (int i = 0; i < 10; i++)
            __delay_cycles(_MS_TICK);
}

void wait1Sec(int t) {
    for ( ; t > 0; t--)
        wait1Msec(1000);
}

void setMotor(int motorNum, int dirSpeed) {
    unsigned char direction = _FWD;
    if (dirSpeed < 0) {
        direction = _REV;
        dirSpeed *= -1;
    }
    if (dirSpeed > 100)
        dirSpeed = 100;
    unsigned char speed = dirSpeed * _MOTORMAX / 100;
    unsigned char motor = _MOTORB;
    switch (motorNum) {
      case MOTOR_A:
          motor = _MOTORA;
      case MOTOR_B:
          _sendMotorChar(_START_BYTE);
          _sendMotorChar(_CNTL_DEVICE);
          _sendMotorChar(motor | direction);
          _sendMotorChar(speed & 0x7F);
          break;
    }
}

int getSensor(int sensorNum) {
    int sensorValue = 0;
    if (sensorNum == BUMPER)
        sensorValue = P2IN_bit._P_SWITCH;
    else if (sensorNum == BUTTON)
        sensorValue = !P1IN_bit.P3;
    else if (sensorNum == REFLECT_1 || sensorNum == REFLECT_2) {
        int pinADC = _ADC_0;
        if (sensorNum == REFLECT_2)
            pinADC = _ADC_1;
        ADC10CTL1 = (pinADC << 12) | ADC10SSEL1 | ADC10SSEL0;
        ADC10CTL0 = ADC10ON | ENC | ADC10SC;
        while (ADC10CTL1 & ADC10BUSY);
        sensorValue = _ADC10MAX - ADC10MEM;
        ADC10CTL0 &= ~ENC;
    }
    return sensorValue;
}

void LEDoff(int led) {
    if (led == RED_LED)
        P1OUT_bit.P0 = _LOW;
    else if (led == GREEN_LED)
        P1OUT_bit.P6 = _LOW;
}

void LEDon(int led) {
    if (led == RED_LED)
        P1OUT_bit.P0 = _HIGH;
    else if (led == GREEN_LED)
        P1OUT_bit.P6 = _HIGH;
}

void LEDnum(int led) {
    if (led >= _LED_MIN && led <= _LED_MAX) {
        unsigned char led_set = (unsigned char)led & _LED_MASK;
        P2OUT &= ~_LED_MASK;
        P2OUT |= led_set;
    }
}

void _initADC() {
    ADC10AE0 = (1 << _ADC_0) | (1 << _ADC_1);
    ADC10CTL0 = ADC10ON;
}

void _initMotorController() {
    P2DIR_bit._P_MOTOR = _OUTPUT;
    P2OUT_bit._P_MOTOR = _HIGH;
    wait1Msec(1);
}

#ifdef _RESET_MOT_CNTL
void _resetMotorController() {
   _sendMotorChar(_START_BYTE);
   _sendMotorChar(_SETUP_DEVICE);
   _sendMotorChar(_DEFAULT_SETUP);
}
#endif

void _sendMotorChar(unsigned char b) {
    P2OUT_bit._P_MOTOR = _LOW;
    wait10Usec(_BIT_PERIOD);
    for (int i = 0; i < _EIGHT_BITS; i++) {
        P2OUT_bit._P_MOTOR = b & 0x01;
        b >>= 1;
        wait10Usec(_BIT_PERIOD);
    }
    for (int i = 0; i < _TWO_STOP_BIT; i++) {
        P2OUT_bit._P_MOTOR = _HIGH;
        wait10Usec(_BIT_PERIOD);
    }
}
#endif
