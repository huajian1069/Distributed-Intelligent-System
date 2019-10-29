
/*
    Copyright 2007 Alexandre Campo, Alvaro Gutierrez, Valentin Longchamp.

    This file is part of libIrcom.

    libIrcom is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License.

    libIrcom is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libIrcom.  If not, see <http://www.gnu.org/licenses/>.
*/

// simple test :  send or receive numbers, and avoid obstacles in the same time.

#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <stdio.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>

float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void obstacleAvoidance();

int main()
{
    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();	
    e_init_motors();
    e_start_agendas_processing();

    // wait for s to start
    //btcomWaitForCommand('s');
    //btcomSendString("==== READY - IR TESTING ====\n\n");

    e_calibrate_ir(); 

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();

    // rely on selector to define the role
    int selector = getselector();

    // show selector choosen
    int i;
    long int j;
    for (i = 0; i < selector; i++)
    {
	e_led_clear();
	
	for(j = 0; j < 200000; j++)
	    asm("nop");
	
	e_set_led(i%8, 1);
	
	for(j = 0; j < 300000; j++)
	    asm("nop");

	e_led_clear();

	for(j = 0; j < 300000; j++)
	    asm("nop");
    }

    // activate obstacle avoidance
    //e_activate_agenda(obstacleAvoidance, 10000);

    // acting as sender
    if (selector == 1)
    {    	
	btcomSendString("==== EMITTER ====\n\n");

	int i;
	for (i = 0; i < 10000; i++)
	{
	    // takes ~15knops for a 32window, avoid putting messages too close...
	    for(j = 0; j < 100000; j++)	asm("nop");

	    ircomSend(i % 256);	    
	    while (ircomSendDone() == 0);

	    btcomSendString(".");
	}
    }
    
    // acting as receiver
    else if (selector == 2)
    {
	btcomSendString("==== RECEIVER ====\n\n");

	int i = 0;
	while (i < 200)
	{
	    // ircomListen();
	    IrcomMessage imsg;
	    ircomPopMessage(&imsg);
	    if (imsg.error == 0)
	    {
			e_set_led(1, 2);
		int val = (int) imsg.value;
	    
		/* Send Value*/		
		char tmp[128];
		sprintf(tmp, "Receive successful : %d  - distance=%f \t direction=%f \n", val, (double)imsg.distance, (double)imsg.direction);
		btcomSendString(tmp);
	    }
	    else if (imsg.error > 0)
	    {
		btcomSendString("Receive failed \n");		
	    }
	    // else imsg.error == -1 -> no message available in the queue

	    if (imsg.error != -1) i++;
	}
    }
    // no proper role defined...
    else 
    {
	int i = 0;
	long int j;
	while(1)
	{
	    e_led_clear();
	    
	    for(j = 0; j < 200000; j++)
		asm("nop");
	    
	    e_set_led(i, 1);
	    
	    for(j = 0; j < 300000; j++)
		asm("nop");
	    
	    i++;
	    i = i%8;
	}	
    }    
    
    ircomStop();
    return 0;
}

int obstacleAvoidanceThreshold = 30.0;
int obstacleAvoidanceSpeed = 500.0;
void obstacleAvoidance()
{    
    // check if an obstacle is perceived 
    double reading = 0.0;
    int obstaclePerceived = 0;
    int i=0;
    double x = 0.0, y = 0.0;
    for (i = 0; i < 8; i++)
    {
        reading = e_get_calibrated_prox(i);
	// if signal above noise
	if(reading >= obstacleAvoidanceThreshold)
	{
	    obstaclePerceived = 1;
	    
	    // compute direction to escape
	    double signal = reading - obstacleAvoidanceThreshold;
	    x += -cos(sensorDir[i]) * signal / 8.0;
	    y += sin(sensorDir[i]) * signal / 8.0;
	}
    }
    
    // no obstacles to avoid, return immediately
    if (obstaclePerceived == 0)
    {
	// go straight forward
	// change movement direction
	e_set_speed_left(obstacleAvoidanceSpeed);
	e_set_speed_right(obstacleAvoidanceSpeed);
	// return obstaclePerceived;
	return;
    }
    
    double desiredAngle = atan2 (y, x);
    
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    
    // turn left
    if (desiredAngle >= 0.0)
    {
	leftSpeed  = cos(desiredAngle);
	rightSpeed = 1;
    }
    // turn right
    else
    {
	leftSpeed = 1;
	rightSpeed = cos(desiredAngle);
    }
    
    // rescale values
    leftSpeed *= obstacleAvoidanceSpeed;
    rightSpeed *= obstacleAvoidanceSpeed;
    
    // change movement direction
    e_set_speed_left(leftSpeed);
    e_set_speed_right(rightSpeed);
    
    // advertise obstacle avoidance in progress
    // return 1;
}
