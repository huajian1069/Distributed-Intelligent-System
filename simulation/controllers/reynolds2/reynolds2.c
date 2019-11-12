/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      2.0                                                         */
/* Date:         06-Oct-15                                                   */
/* Description:  Reynolds flocking with relative positions		     */
/*                                                                           */
/* Author: 	 06-Oct-15 by Ali Marjovi				     */
/* Last revision:12-Oct-15 by Florian Maushart				     */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS 8	   // Number of distance sensors
#define MIN_SENS 350	   // Minimum sensibility value
#define MAX_SENS 4096	  // Maximum sensibility value
#define MAX_SPEED 800	  // Maximum speed
#define MAX_SPEED_WEB 6.28 // Maximum speed webots
#define FLOCK_SIZE 5	   // Size of flock
#define N_GROUPS 2
#define TIME_STEP 64 // [ms] Length of time step

#define AXLE_LENGTH 0.052		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205		// Wheel radius (meters)
#define DELTA_T 0.064			// Timestep (seconds)

#define RULE1_THRESHOLD 0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT (10 / 10) // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD 1		 // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT (0.02 / 10) // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT (1.0 / 10) // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT (1.9 / 10) // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x >= 0) ? (x) : -(x))

WbDeviceTag left_motor;  //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot

int e_puck_matrix[16] = {17, 29, 34, 10, 8, -38, -56, -76, -72, -58, -36, 8, 10, 36, 28, 18}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS]; // Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node

int group_id, robot_id; // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[N_GROUPS][FLOCK_SIZE][3];	  // relative X, Z, Theta of all robots
float prev_relative_pos[N_GROUPS][FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
float absolute_pos[N_GROUPS][FLOCK_SIZE][2];
float my_position[3];						   // X, Z, Theta of the current robot
float prev_my_position[3];					   // X, Z, Theta of the current robot in the previous time step
float speed[N_GROUPS][FLOCK_SIZE][2];		   // Speeds calculated with Reynold's rules
float relative_speed[N_GROUPS][FLOCK_SIZE][2]; // Speeds calculated with Reynold's rules
int initialized[N_GROUPS][FLOCK_SIZE];		   // != 0 if initial positions have been received
float migr[2] = {0, -2};					   // Migration vector
char *robot_name;

float theta_robots[N_GROUPS][FLOCK_SIZE];
/*
 * Reset the robot's devices and get its ID
 */
static void reset()
{
	int i, f;
	char s[4] = "ps0";

	wb_robot_init();

	receiver2 = wb_robot_get_device("receiver2");
	emitter2 = wb_robot_get_device("emitter2");

	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);

	for (i = 0; i < NB_SENSORS; i++)
	{
		ds[i] = wb_robot_get_device(s); // the device name is specified in the world file
		s[2]++;							// increases the device number
	}
	robot_name = (char *)wb_robot_get_name();

	for (i = 0; i < NB_SENSORS; i++)
		wb_distance_sensor_enable(ds[i], 64);

	wb_receiver_enable(receiver2, 64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name, "epuck_%d_%d", &group_id, &robot_id); // read robot id from the robot's name
	printf("Robot with ID %d initialized in group %d\n", robot_id, group_id);

	for (f = 0; f < N_GROUPS; f++)
	{
		for (i = 0; i < FLOCK_SIZE; i++)
		{
			initialized[f][i] = 0; // Set initialization to 0 (= not yet initialized)
		}
	}
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
	//float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	//float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates

	float x = speed[group_id][robot_id][0] * cosf(my_position[2]) + speed[group_id][robot_id][1] * sinf(my_position[2]);  // x in robot coordinates
	float z = -speed[group_id][robot_id][0] * sinf(my_position[2]) + speed[group_id][robot_id][1] * cosf(my_position[2]); // z in robot coordinates
																														  //	printf("id = %d, x = %f, y = %f\n", robot_id, x, z);
	float Ku = 0.2;																										  // Forward control coefficient
	float Kw = 1;																										  // Rotational control coefficient
	float range = sqrtf(x * x + z * z);																					  // Distance to the wanted position
	float bearing = -atan2(x, z);																						  // Orientation of the wanted position

	// Compute forward control
	float u = Ku * range * cosf(bearing);
	// Compute rotational control
	float w = Kw * bearing;

	// Convert to wheel speeds!

	*msl = (u - AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
	//	printf("bearing = %f, u = %f, w = %f, msl = %f, msr = %f\n", bearing, u, w, msl, msr);
	limit(msl, MAX_SPEED);
	limit(msr, MAX_SPEED);
}

/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules()
{
	int i, j, k, g;					  // Loop counters
	float rel_avg_loc[N_GROUPS][2];   // Flock average positions
	float rel_avg_speed[N_GROUPS][2]; // Flock average speeds
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
			abs_avg_loc[g][j] = 0;
		}
	}

	/* Compute averages over the whole flock */
	for (g = 0; g < N_GROUPS; g++)
	{
		for (i = 0; i < FLOCK_SIZE; i++)
		{
			if (i == robot_id)
				continue; // don't consider yourself for the average
			for (j = 0; j < 2; j++)
			{
				rel_avg_speed[g][j] += relative_speed[g][i][j];
				rel_avg_loc[g][j] += relative_pos[g][i][j];
				abs_avg_loc[g][j] += absolute_pos[g][i][j];
			}
		}
	}

	for (g = 0; g < N_GROUPS; g++)
	{
		for (j = 0; j < 2; j++)
		{
			rel_avg_speed[g][j] /= FLOCK_SIZE - 1;
			rel_avg_loc[g][j] /= FLOCK_SIZE - 1;
			abs_avg_loc[g][j] /= FLOCK_SIZE;
		}
	}

	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	for (j = 0; j < 2; j++)
	{
		cohesion[j] = rel_avg_loc[group_id][j];
	}

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (g = 0; g < N_GROUPS; g++)
	{
		if (g != group_id)
		{
			if (pow(rel_avg_loc[g][0], 2) + pow(rel_avg_loc[g][1], 2) <
				RULE2_THRESHOLD)
			{
				for (j = 0; j < 2; j++)
				{
					dispersion[j] -= 1 / rel_avg_loc[g][j]; // Relative distance to k
				}
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
		speed[group_id][robot_id][j] += dispersion[j] * RULE2_WEIGHT;
		speed[group_id][robot_id][j] += consistency[j] * RULE3_WEIGHT;
	}
	speed[group_id][robot_id][1] *= -1; //y axis of webots is inverted

	//move the robot according to some migration rule
	if (MIGRATORY_URGE == 0)
	{
		speed[group_id][robot_id][0] += 0.01 * cos(my_position[2] + M_PI / 2);
		speed[group_id][robot_id][1] += 0.01 * sin(my_position[2] + M_PI / 2);
	}
	else
	{
		speed[group_id][robot_id][0] += MIGRATION_WEIGHT * (migr[0] - abs_avg_loc[group_id][0]);
		speed[group_id][robot_id][1] -= MIGRATION_WEIGHT * (migr[1] - abs_avg_loc[group_id][1]); //y axis of webots is inverted
	}

	// printf("Migration goal: %f %f\n", migr[0], migr[1]);

	// printf("Robot speed: %f %f\n", speed[group_id][robot_id][0], speed[group_id][robot_id][1]);

	// printf("My position: %f %f\n", my_position[0], my_position[1]);
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)
{
	char out[20];
	sprintf(out, "%d,%d,%.3f,%.3f", group_id, robot_id, my_position[0], my_position[1]);

	wb_emitter_send(emitter2, out, strlen(out) + 1);
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer; // Buffer for the receiver node
	int other_robot_id, other_group_id;
	float other_x, other_y;
	while (wb_receiver_get_queue_length(receiver2) > 0)
	{
		inbuffer = (char *)wb_receiver_get_data(receiver2);
		message_direction = wb_receiver_get_emitter_direction(receiver2);
		message_rssi = wb_receiver_get_signal_strength(receiver2);
		double y = message_direction[2];
		double x = message_direction[1];

		theta = -atan2(y, x);
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1 / message_rssi));

		sscanf(inbuffer, "%d,%d,%f,%f", &other_group_id, &other_robot_id, &other_x, &other_y);
		absolute_pos[other_group_id][other_robot_id][0] = other_x;
		absolute_pos[other_group_id][other_robot_id][1] = other_y;
		// printf("Other position: %f %f\n", other_x, other_y);

		// Get position update
		//theta += dtheta_g[other_robot_id];
		//theta_robots[other_robot_id] = 0.8*theta_robots[other_robot_id] + 0.2*theta;
		prev_relative_pos[other_group_id][other_robot_id][0] = relative_pos[other_group_id][other_robot_id][0];
		prev_relative_pos[other_group_id][other_robot_id][1] = relative_pos[other_group_id][other_robot_id][1];

		relative_pos[other_group_id][other_robot_id][0] = range * cos(theta);		 // relative x pos
		relative_pos[other_group_id][other_robot_id][1] = -1.0 * range * sin(theta); // relative y pos

		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

		relative_speed[other_group_id][other_robot_id][0] = relative_speed[other_group_id][other_robot_id][0] * 0.0 + 1.0 * (1 / DELTA_T) * (relative_pos[other_group_id][other_robot_id][0] - prev_relative_pos[other_group_id][other_robot_id][0]);
		relative_speed[other_group_id][other_robot_id][1] = relative_speed[other_group_id][other_robot_id][1] * 0.0 + 1.0 * (1 / DELTA_T) * (relative_pos[other_group_id][other_robot_id][1] - prev_relative_pos[other_group_id][other_robot_id][1]);

		wb_receiver_next_packet(receiver2);
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

		/* Braitenberg */
		for (i = 0; i < NB_SENSORS; i++)
		{
			distances[i] = wb_distance_sensor_get_value(ds[i]);			  //Read sensor values
			sum_sensors += distances[i];								  // Add up sensor values
			max_sens = max_sens > distances[i] ? max_sens : distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i + NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl /= MIN_SENS;
		bmsr /= MIN_SENS;
		// bmsl = (bmsl * bmsl) / 100;
		// bmsr = (bmsl * bmsr) / 100;

		/* Send and get information */
		send_ping(); // sending a ping to other robot, so they can measure their distance to this robot

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];

		update_self_motion(msl, msr);

		process_received_ping_messages();

		speed[group_id][robot_id][0] = (1 / DELTA_T) * (my_position[0] - prev_my_position[0]);
		speed[group_id][robot_id][1] = (1 / DELTA_T) * (my_position[1] - prev_my_position[1]);

		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();

		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);

		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS * MIN_SENS)
		{
			msl -= msl * max_sens / (2 * MAX_SENS);
			msr -= msr * max_sens / (2 * MAX_SENS);
		}

		// Add Braitenberg
		msl += bmsl * 10;
		msr += bmsr * 10;

		// Set speed
		msl_w = msl * MAX_SPEED_WEB / 1000;
		msr_w = msr * MAX_SPEED_WEB / 1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);

		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}
