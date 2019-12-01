//******************************************************************************
//  Name:   main.c (Rulebased obstacle avoidance)
//  Author: 
//  Date:   
//  Rev:    October 5, 2015 by Florian Maushart
//  Descr:  Skeleton for a rule-based obstacle avoidance algorithm
//******************************************************************************

#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"

#include "./codec/e_sound.h"
#include "./motor_led/e_init_port.h"
#include "./motor_led/e_led.h"
#include "./motor_led/e_motors.h"
#include "./uart/e_uart_char.h"
//#include "./a_d/advance_ad_scan/e_acc.h"
#include "./a_d/advance_ad_scan/e_prox.h"
#include "./a_d/advance_ad_scan/e_ad_conv.h"


// Waits for a certain amount of time
// Note that the effective waiting time is not always the same (because of possible interrupts).
void wait(unsigned long num) {
	while (num > 0) {num--;}
}

// Main program
int main() {
	char buffer[80];
	int leftwheel, rightwheel;
	int sensor[8], value;
	int i;

	// Initialize system and sensors
	e_init_port();
	e_init_uart1();
	e_init_motors();
	e_init_ad_scan();

	// Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR = 0;
		__asm__ volatile ("reset");
	}

	// Say hello
	// You'll receive these messages in Minicom. They are therefore well suited for debugging.
	// Note, however, that sending such messages is time-consuming and can slow down your program significantly.
	sprintf(buffer, "Rule-based starting\r\n");
	e_send_uart1_char(buffer, strlen(buffer));

	// Run the rule-based algorithm
	while (1) {
		// Forward speed
		leftwheel = 200;
		rightwheel = 200;

		// Get sensor values
		for (i = 0; i < 8; i++) {
			sensor[i] = e_get_prox(i);
			//sprintf(buffer, "%d, ", sensor[i]);
			//e_send_uart1_char(buffer, strlen(buffer));
		}

		// ******************************************
		// add your rule-based code here



		// *****************************************

		//sprintf(buffer, " -> desired speed: %d %d\r\n", leftwheel, rightwheel);
		//e_send_uart1_char(buffer, strlen(buffer));

		// Speed bounds, to avoid setting to high speeds to the motor
		if (leftwheel > 1000) {leftwheel = 1000;}
		if (rightwheel > 1000) {rightwheel = 1000;}
		if (leftwheel < -1000) {leftwheel = -1000;}
		if (rightwheel < -1000) {rightwheel = -1000;}
		e_set_speed_left(leftwheel);
		e_set_speed_right(rightwheel);

		// Indicate with leds on which side we are turning (leds are great for debugging) 
		if (leftwheel>rightwheel) {
			e_set_led(1, 1);
			e_set_led(7, 0);
		} else {
			e_set_led(1, 0);
			e_set_led(7, 1);
		}

		// Wait for some time
		wait(100000);
	}

	return 0;
}
