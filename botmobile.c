#include "botmobile.h"

void main (void) {
    initialize();
    wait1Msec(200);
    LEDon(RED_LED);
    LEDon(GREEN_LED);
    while (getSensor(BUTTON) == 0);
    while (getSensor(BUTTON) != 0);
    int bump = 0;
    int color1 = getSensor(REFLECT_1);
    int color2 = getSensor(REFLECT_2);
    while (bump != 5) {
    	while (color1 == color2) {
    		setMotor(MOTOR_A, 20);
    		setMotor(MOTOR_B, 20);
		}
    	while(color1 > color2) {
        	setMotor(MOTOR_A, 0);
    	}
    	setMotor(MOTOR_A, 20);
    	while(color1 < color2) {
        	setMotor(MOTOR_B, 0);
    	}
    	setMotor(MOTOR_B, 20);
    	if(getSensor(BUMPER) == 0) {
        	setMotor(MOTOR_A, -20);
        	setMotor(MOTOR_B, 0);
        	wait1Msec(5000);
        	setMotor(MOTOR_A, 20);
        	setMotor(MOTOR_B, 20);
        	bump++;
    	}
    }
	setMotor(MOTOR_A, 0);
    setMotor(MOTOR_B, 0);
}
