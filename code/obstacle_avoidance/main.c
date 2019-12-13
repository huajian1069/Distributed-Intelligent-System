/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      3.0                                                         */
/* Date:         06-Oct-15                                                   */
/* Description:  Reynolds flocking with relative positions		             */
/*                                                                           */
/* Author: 	      06-Oct-15 by Ali Marjovi				                     */
/* Last revision: 12-Oct-15 by Florian Maushart				                 */
/* Modified to make it suitable for DIS project: Nov-19 by Group 17          */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <ircom/ircom.h>
#include <btcom/btcom.h>

#include <time.h>
#include <p30f6014a.h>

#ifndef M_PI
#define M_PI 3.14
#endif

#define NB_SENSORS 8	   // Number of distance sensors
#define MIN_SENS 350	   // Minimum sensibility value
#define MAX_SENS 4096	  // Maximum sensibility value
#define MAX_SPEED 800	  // Maximum speed
#define MAX_SPEED_WEB 6.28 // Maximum speed webots
#define FLOCK_SIZE 3	   // Size of flock
#define N_GROUPS 1
#define TIME_STEP 64 // [ms] Length of time step

#define AXLE_LENGTH 0.052		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205		// Wheel radius (meters)
#define DELTA_T 0.064			// Timestep (seconds)

#define RULE1_THRESHOLD 0.1	// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT ( 2.0 / 10) // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD 0.07		 // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT (1.0 / 10) // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT (1.0 / 10) // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT (0.6 / 10) // Wheight of attraction towards the common goal. default 0.01/10; weigth 0.4 works when only migration

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x >= 0) ? (x) : -(x))

#define OBS_RANGE 70

#define OBS_GAIN 200
#define NUM_SENS_POTENTIAL 6

#define eps 0.05

#ifndef  ROBOT_ID
#define  ROBOT_ID 0
#endif


int e_puck_matrix[16] = {47, 59, 64, 50, 38, -48, -66, -86, -82, -68, -46, 38, 50, 66, 58, 48}; // for obstacle avoidance
int front_sensors[NUM_SENS_POTENTIAL] = {0, 1, 2, 5, 6, 7};
float sensor_vectors[NUM_SENS_POTENTIAL][2] = {{0.351441063, -0.936197964}, {0.865973584, -0.500089743}, {1, 0}, {-1, 0}, {-0.865973584, -0.500089743}, {-0.351441063, -0.936197964}};

int group_id;
int robot_id=ROBOT_ID; // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[N_GROUPS][FLOCK_SIZE][3];	  // relative X, Z, Theta of all robots
float prev_relative_pos[N_GROUPS][FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
float my_position[3];						   // X, Z, Theta of the current robot
float prev_my_position[3];					   // X, Z, Theta of the current robot in the previous time step
float speed[N_GROUPS][FLOCK_SIZE][2];		   // Speeds calculated with Reynold's rules
float relative_speed[N_GROUPS][FLOCK_SIZE][2]; // Speeds calculated with Reynold's rules
float migr[2] = {0, -1};					   // Migration vector
char *robot_name;
float theta_robots[N_GROUPS][FLOCK_SIZE];
int potential_left;
int potential_right;
float w_difference;
char buffer[80];
time_t time_ptr;
float prev_direction;
int fast_rotation=0;
float rel_avg_loc[N_GROUPS][2];   // Flock average positions
float rel_avg_speed[N_GROUPS][2]; // Flock average speeds
int counter=0;
float prev_rel_avg_y=0;

void wait(unsigned long num) {
	while (num > 0) {num--;}
}

/*
 * Reset the robot's devices and get its ID
 */
static void reset()
{
	e_init_port();
	e_init_uart1();
	e_init_motors();
	e_init_ad_scan();

	e_start_agendas_processing();

    e_calibrate_ir();

	ircomStart();
	ircomEnableContinuousListening();
	ircomListen();

	// Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR = 0;
		__asm__ volatile ("reset");
	}

	group_id = 0;
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit)
{
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr)
{
	float theta = my_position[2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl) / 2.0;
	float dtheta = (dr - dl) / AXLE_LENGTH;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);

	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2 * M_PI)
		my_position[2] -= 2.0 * M_PI;
	if (my_position[2] < 0)
		my_position[2] += 2.0 * M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr)
{
	// Compute wanted position from Reynold's speed and current location
	float x = speed[group_id][robot_id][0] * cosf(my_position[2]) + speed[group_id][robot_id][1] * sinf(my_position[2]);  // x in robot coordinates
	float z = -speed[group_id][robot_id][0] * sinf(my_position[2]) + speed[group_id][robot_id][1] * cosf(my_position[2]); // z in robot coordinates
																														  //	printf("id = %d, x = %f, y = %f\n", robot_id, x, z);
	float Ku = 0.12;																										  // Forward control coefficient
	float Kw = 0.2;																										  // Rotational control coefficient
	float range = sqrtf(x * x + z * z);																					  // Distance to the wanted position
	float bearing = -atan2(x, z);																						  // Orientation of the wanted position

	// Compute forward control
	float u = Ku * range * cosf(bearing);

	// Compute rotational control
	float w = Kw * bearing;

	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
	limit(msl, MAX_SPEED);
	limit(msr, MAX_SPEED);
}

// update speeds according to the potential fiel
void potential_field()
{
	int distances[NUM_SENS_POTENTIAL]; // Array for the distance sensor readings
	int max_sens = 0;				   // Store highest sensor value
	int sum_sensors = 0;
	float mean_vector[2] = {0, 0};
	float normal_vector[2];
	double angle;
	double norm;
	int i, j;

	for (i = 0; i < NUM_SENS_POTENTIAL; i++)
	{
		distances[i] = e_get_prox(i) > 0 ? e_get_prox(i) : 0;
		sum_sensors += distances[i];								  // Add up sensor values
		max_sens = max_sens > distances[i] ? max_sens : distances[i]; // Check if new highest sensor value
	}
	if (sum_sensors <= 0)
		return;
	//printf("robot id %d %s %d\n", robot_id, "sum_sensors", sum_sensors);
	for (i = 0; i < NUM_SENS_POTENTIAL; i++)
	{
		for (j = 0; j < 2; j++)
		{
			mean_vector[j] += ((float)distances[i] / (float)sum_sensors) * sensor_vectors[i][j];
		}
	}
	if (mean_vector[0] < 0)
	{
		normal_vector[0] = (-mean_vector[1]);
		normal_vector[1] = mean_vector[0];
	}
	else
	{
		normal_vector[0] = mean_vector[1];
		normal_vector[1] = (-mean_vector[0]);
	}


	if (max_sens > OBS_RANGE)
	{
	  w_difference = 2 * (max_sens - OBS_RANGE) * sqrt(pow(normal_vector[0], 2) + pow(normal_vector[1] + 1, 2)) * (ABS(normal_vector[0]) / normal_vector[0]);
		//sprintf(buffer, "diiff %f \r\n", w_difference);
		//e_send_uart1_char(buffer, strlen(buffer));
		potential_right = OBS_GAIN * (log10(max_sens - OBS_RANGE)) - w_difference / 2;
		potential_left = OBS_GAIN * (log10(max_sens - OBS_RANGE)) + w_difference / 2;
	}
	else
	{
		potential_right = 0;
		potential_left = 0;
	}
}
float limit_difference(float d){
	const float DIFF=200.0;
	float difference;
	if(d > DIFF){ difference=DIFF;}
	else {
		if (d < -DIFF) {difference=-DIFF;}
	  else { difference=d;}}
	return difference;
}


/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules()
{
	int i, j, g;					  // Loop counters
	float abs_avg_loc[N_GROUPS][2];
	float cohesion[2] = {0, 0};
	float dispersion[2] = {0, 0};
	float consistency[2] = {0, 0};

	for (g = 0; g < N_GROUPS; g++)
	{
		for (j = 0; j < 2; j++)
		{
			rel_avg_loc[g][j] = 0;
			rel_avg_speed[g][j] = 0;
		}
	}

	/* Compute averages over the whole flock */
	for (g = 0; g < N_GROUPS; g++)
	{
		for (i = 0; i < FLOCK_SIZE; i++)
		{
			for (j = 0; j < 2; j++)
			{
				rel_avg_speed[g][j] += relative_speed[g][i][j];
				rel_avg_loc[g][j] += relative_pos[g][i][j];
			}
		}
	}

	for (g = 0; g < N_GROUPS; g++)
	{
		for (j = 0; j < 2; j++)
		{
			rel_avg_speed[g][j] /= FLOCK_SIZE - 1;
			rel_avg_loc[g][j] /= FLOCK_SIZE - 1;
		}
	}

	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	if (sqrt(pow(rel_avg_loc[group_id][0], 2) + pow(rel_avg_loc[group_id][1], 2)) >
			RULE1_THRESHOLD) {
	    for (j = 0; j < 2; j++){
				cohesion[j] = rel_avg_loc[group_id][j];
			}
	}

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */

	for (i = 0; i < FLOCK_SIZE; i++)
	{
		if (i == robot_id)
			continue;
		if (sqrt(pow(relative_pos[group_id][i][0], 2) + pow(relative_pos[group_id][i][1], 2)) <
			RULE2_THRESHOLD)
		{
			for (j = 0; j < 2; j++)
			{
				dispersion[j] -= 1 / relative_pos[group_id][i][j]; // Relative distance to k
			}
		}
	}

	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	for (j = 0; j < 2; j++)
	{
		consistency[j] = rel_avg_speed[group_id][j];
	}

	//aggregation of all behaviors with relative influence determined by weights
	for (j = 0; j < 2; j++)
	{
		speed[group_id][robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		//speed[group_id][robot_id][j] += dispersion[j] * RULE2_WEIGHT;
		//speed[group_id][robot_id][j] += consistency[j] * RULE3_WEIGHT;
	}
	//speed[group_id][robot_id][1] *= -1; //y axis of webots is inverted

	//move the robot according to some migration rule
	if (MIGRATORY_URGE == 0)
	{
		speed[group_id][robot_id][0] += 0.01 * cos(my_position[2] + M_PI / 2);
		speed[group_id][robot_id][1] += 0.01 * sin(my_position[2] + M_PI / 2);
	}
	else
	{
		float normalize=sqrt(pow((migr[0] - my_position[0]),2)+pow((migr[1] - my_position[1]),2));
		if (normalize > eps) {
		speed[group_id][robot_id][0] += MIGRATION_WEIGHT*(migr[0] - my_position[0]);///normalize;
		speed[group_id][robot_id][1] -= MIGRATION_WEIGHT*(migr[1] - my_position[1]);///normalize; //y axis of webots is inverted
	 }else {
		 speed[group_id][robot_id][0] = 0.0;
		 speed[group_id][robot_id][1] = 0.0;
   }
 }
 if (prev_rel_avg_y==rel_avg_loc[group_id][1]){
	 counter++;
 }else{
	 counter=0;
 }
 prev_rel_avg_y=rel_avg_loc[group_id][1];

	sprintf(buffer, "rel avg x is %f  rel avg y is %f\r\n",rel_avg_loc[group_id][0], rel_avg_loc[group_id][1]);
	e_send_uart1_char(buffer, strlen(buffer));
}



/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)
{
	ircomSend(ROBOT_ID);
	while (ircomSendDone() == 0);
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
	float message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer; // Buffer for the receiver node
	int other_robot_id, other_group_id;
	float other_x, other_y;

	IrcomMessage imsg;
	ircomPopMessage(&imsg);
	if((imsg.error == 0) && (fast_rotation==0))
	{

		// imsg.value
		message_direction = imsg.direction;
		message_rssi = imsg.distance;
		theta = imsg.direction; // find the relative theta;
		range = imsg.distance*0.01;
		other_robot_id = (int)imsg.value;
		other_group_id=group_id;
		//sprintf(buffer, "angle %f\r\n", imsg.direction);
	  //e_send_uart1_char(buffer, strlen(buffer));

		//if (theta < 0.3 && prev_direction > 5.8  ) theta=0;
	  //else{ if (theta > 5.8 && prev_direction < 0.3 ) theta =0;
		  //   else {
				//	 theta=(theta+prev_direction)/2;
				// }

	//}
  //prev_direction=theta;

		prev_relative_pos[other_group_id][other_robot_id][0] = relative_pos[other_group_id][other_robot_id][0];
		prev_relative_pos[other_group_id][other_robot_id][1] = relative_pos[other_group_id][other_robot_id][1];

		relative_pos[other_group_id][other_robot_id][0] = -1.0 * range * sin(theta);		 //relative x pos
		relative_pos[other_group_id][other_robot_id][1] = -1.0 * range * cos(theta); // relative y pos


		relative_speed[other_group_id][other_robot_id][0] = relative_speed[other_group_id][other_robot_id][0] * 0.0 + 1.0 * (1 / DELTA_T) * (relative_pos[other_group_id][other_robot_id][0] - prev_relative_pos[other_group_id][other_robot_id][0]);
		relative_speed[other_group_id][other_robot_id][1] = relative_speed[other_group_id][other_robot_id][1] * 0.0 + 1.0 * (1 / DELTA_T) * (relative_pos[other_group_id][other_robot_id][1] - prev_relative_pos[other_group_id][other_robot_id][1]);

		//sprintf(buffer, "from %d and relative pos x is %f relative pos y is %f\r\n", other_robot_id, 	relative_pos[other_group_id][other_robot_id][0], relative_pos[other_group_id][other_robot_id][1]);
	  //e_send_uart1_char(buffer, strlen(buffer));
	}

}
void wait_ms(unsigned long ms){
	volatile unsigned long time_start = get_epoch_ms();
	volatile unsigned long time_stop;
	while(1){
		time_stop=get_epoch_ms();
		//sprintf(buffer, "timer %lu \r\n", time_stop);
	  //e_send_uart1_char(buffer, strlen(buffer));
		e_set_speed_left(0);
		e_set_speed_right(0);
		if(time_stop-time_start>ms) break;
	}
}




// the main function
int main()
{

	int msl, msr; // Wheel speeds
	float msl_w, msr_w;
	int bmsl, bmsr, sum_sensors; // Braitenberg parameters
	int i;						 // Loop counter
	int distances[NB_SENSORS];   // Array for the distance sensor readings
	int max_sens;				 // Store highest sensor value

	reset(); // Resetting the robot



	msl = 0;
	msr = 0;
	max_sens = 0;

	// Forever
	for (;;)
	{

		bmsl = 0;
		bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
    prev_direction=0;
		/* Send and get information */
		send_ping(); // sending a ping to other robot, so they can measure their distance to this robot

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];

		update_self_motion(msl, msr);
		//sprintf(buffer, "timer %lu \r\n", get_epoch_ms());
	  //e_send_uart1_char(buffer, strlen(buffer));

		process_received_ping_messages();
		//sprintf(buffer, "sent \r\n");
		//e_send_uart1_char(buffer, strlen(buffer));

		speed[group_id][robot_id][0] = (1 / DELTA_T) * (my_position[0] - prev_my_position[0]);
		speed[group_id][robot_id][1] = (1 / DELTA_T) * (my_position[1] - prev_my_position[1]);

		//different approach, two states, reynolds and potential field
		for (i = 0; i < NUM_SENS_POTENTIAL; i++)
		{
			distances[i] = e_get_prox(front_sensors[i]);								  // Add up sensor values
			max_sens = max_sens > distances[i] ? max_sens : distances[i]; // Check if new highest sensor value
		}
		potential_field();
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		compute_wheel_speeds(&msl, &msr);

		// Compute wheels speed from reynold's speed

		// Adapt speed instinct to distance sensor values
   fast_rotation=0;
	 if (max_sens > OBS_RANGE)
		{
			fast_rotation=1;
			msl = (((float)(log10(max_sens - OBS_RANGE))) / log10(max_sens)) * potential_left + (1 - ((float)(log10(max_sens - OBS_RANGE)) / log10(max_sens))) * msl;
			msr = (((float)(log10(max_sens - OBS_RANGE))) / log10(max_sens)) * potential_right + (1 - ((float)(log10(max_sens - OBS_RANGE)) / log10(max_sens))) * msr;
		}
    //limit wheels
		limit(&msl, 999);
		limit(&msr, 999);

		//checking he is not too away
		//sprintf(buffer, "counter: %d\r\n", counter);
	  //e_send_uart1_char(buffer, strlen(buffer));
    if (counter>10){
			if(rel_avg_loc[group_id][1]>0.05){
				msl=0.0;
				msr=0.0;
				wait_ms(2000);
				counter=0;
			}
		}




		// Set speed
		msl_w = msl * MAX_SPEED_WEB / 1000;
		msr_w = msr * MAX_SPEED_WEB / 1000;
		e_set_speed_left(msl);
	  e_set_speed_right(msr);
		//e_set_speed_left((int)(200));
		//e_set_speed_right((int)(200));

		//sprintf(buffer, "Time: %d\r\n", epoch);
	//	e_send_uart1_char(buffer, strlen(buffer));

		// Continue one step
		wait(100000);
	}
}
