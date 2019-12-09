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

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "optim.h"

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

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x >= 0) ? (x) : -(x))

#define OBS_RANGE 70
#define OBS_GAIN 200
#define NUM_SENS_POTENTIAL 6

float aggregation_threshold = 0.1;
float aggregation_weight = 0.2;
float dispersion_threshold = 0.3;
float dispersion_weight = 0.002;
float consistency_weight = 0.1;
float migration_weight = 0.3;

WbDeviceTag left_motor;  //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot

int front_sensors[NUM_SENS_POTENTIAL] = {0, 1, 2, 5, 6, 7};
float sensor_vectors[NUM_SENS_POTENTIAL][2] = {{0.351441063, -0.936197964}, {0.865973584, -0.500089743}, {1, 0}, {-1, 0}, {-0.865973584, -0.500089743}, {-0.351441063, -0.936197964}};

WbDeviceTag ds[NB_SENSORS]; // Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node

int group_id, robot_id; // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[N_GROUPS][FLOCK_SIZE][3];	  // relative X, Z, Theta of all robots
float prev_relative_pos[N_GROUPS][FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
float my_position[3];						   // X, Z, Theta of the current robot
float prev_my_position[3];					   // X, Z, Theta of the current robot in the previous time step
float speed[N_GROUPS][FLOCK_SIZE][2];		   // Speeds calculated with Reynold's rules
float relative_speed[N_GROUPS][FLOCK_SIZE][2]; // Speeds calculated with Reynold's rules
float migr[2] = {0, -4};					   // Migration vector
char *robot_name;
int potential_left;
int potential_right;
float w_difference;


void reinitialize_states() {
	for (int i = 0; i < 3; i++) {
		my_position[i] = 0;
		prev_my_position[i] = 0;
	}

	for (int i = 0; i < N_GROUPS; i++) {
		for (int j = 0; j < FLOCK_SIZE; j++) {
			relative_pos[i][j][0] = 0;
			relative_pos[i][j][1] = 0;
			relative_pos[i][j][2] = 0;
			prev_relative_pos[i][j][0] = 0;
			prev_relative_pos[i][j][1] = 0;
			prev_relative_pos[i][j][2] = 0;
			relative_speed[i][j][0] = 0;
			relative_speed[i][j][1] = 0;
			speed[i][j][0] = 0;
			speed[i][j][1] = 0;
		}
	}

	potential_left = 0;
	potential_right = 0;
	w_difference = 0;
}

/*
 * Reset the robot's devices and get its ID
 */
static void reset()
{
	int i;
	char s[4] = "ps0";

	wb_robot_init();
	optim_init();

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

	for (int i = 0; i < NUM_SENS_POTENTIAL; i++)
	{
		distances[i] = wb_distance_sensor_get_value(ds[front_sensors[i]]) > 0 ? wb_distance_sensor_get_value(ds[front_sensors[i]]) : 0;
		sum_sensors += distances[i];								  // Add up sensor values
		max_sens = max_sens > distances[i] ? max_sens : distances[i]; // Check if new highest sensor value
	}
	if (sum_sensors <= 0)
		return;
	//printf("robot id %d %s %d\n", robot_id, "sum_sensors", sum_sensors);
	for (int i = 0; i < NUM_SENS_POTENTIAL; i++)
	{
		for (int j = 0; j < 2; j++)
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
		potential_right = OBS_GAIN * (log10(max_sens - OBS_RANGE)) - w_difference / 2;
		potential_left = OBS_GAIN * (log10(max_sens - OBS_RANGE)) + w_difference / 2;
	}
	else
	{
		potential_right = 0;
		potential_left = 0;
	}
}

/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules()
{
	int i, j, g;					  // Loop counters
	float rel_avg_loc[N_GROUPS][2];   // Flock average positions
	float rel_avg_speed[N_GROUPS][2]; // Flock average speeds
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
	for (j = 0; j < 2; j++)
	{
		cohesion[j] = rel_avg_loc[group_id][j];
	}

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */

	for (i = 0; i < FLOCK_SIZE; i++)
	{
		if (i == robot_id)
			continue;
		if (pow(relative_pos[group_id][i][0], 2) + pow(relative_pos[group_id][i][1], 2) <
			dispersion_threshold)
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
		speed[group_id][robot_id][j] = cohesion[j] * aggregation_weight;
		speed[group_id][robot_id][j] += dispersion[j] * dispersion_weight;
		speed[group_id][robot_id][j] += consistency[j] * consistency_weight;
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
		/*
		speed[group_id][robot_id][0] += migration_weight * (migr[0] - my_position[0]);
		speed[group_id][robot_id][1] -= migration_weight * (migr[1] - my_position[1]); //y axis of webots is inverted
		*/
	
		float norm = pow(migr[0] - my_position[0], 2) + pow(migr[1] - my_position[1], 2);
		speed[group_id][robot_id][0] += migration_weight * (migr[0] - my_position[0]) / norm;
		speed[group_id][robot_id][1] -= migration_weight * (migr[1] - my_position[1]) / norm; 
		
	}
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)
{
	char out[1];
	out[0] = (group_id << 4) | robot_id;
	wb_emitter_send(emitter2, out, 1);
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

		other_group_id = inbuffer[0] >> 4;
		other_robot_id = inbuffer[0] & 0x0F;

		// Get position update
		prev_relative_pos[other_group_id][other_robot_id][0] = relative_pos[other_group_id][other_robot_id][0];
		prev_relative_pos[other_group_id][other_robot_id][1] = relative_pos[other_group_id][other_robot_id][1];

		relative_pos[other_group_id][other_robot_id][0] = range * cos(theta);		 // relative x pos
		relative_pos[other_group_id][other_robot_id][1] = -1.0 * range * sin(theta); // relative y pos

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
	int i;						 // Loop counter
	int distances[NB_SENSORS];   // Array for the distance sensor readings
	int max_sens;				 // Store highest sensor value

	reset(); // Resetting the robot

	msl = 0;
	msr = 0;

	// Forever
	for (;;)
	{
		max_sens = 0;

		/* Send and get information */
		send_ping(); // sending a ping to other robot, so they can measure their distance to this robot

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];

		update_self_motion(msl, msr);

		process_received_ping_messages();

		speed[group_id][robot_id][0] = (1 / DELTA_T) * (my_position[0] - prev_my_position[0]);
		speed[group_id][robot_id][1] = (1 / DELTA_T) * (my_position[1] - prev_my_position[1]);

		//different approach, two states, reynolds and potential field
		for (i = 0; i < NUM_SENS_POTENTIAL; i++)
		{
			distances[i] = wb_distance_sensor_get_value(ds[front_sensors[i]]); // Add up sensor values
			max_sens = max_sens > distances[i] ? max_sens : distances[i];	  // Check if new highest sensor value
		}
		potential_field();
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		compute_wheel_speeds(&msl, &msr);

		// Compute wheels speed from reynold's speed
		// Adapt speed instinct to distance sensor values
		if (max_sens > OBS_RANGE)
		{
			msl = (((float)(log10(max_sens - OBS_RANGE))) / log10(max_sens)) * potential_left + (1 - ((float)(log10(max_sens - OBS_RANGE)) / log(max_sens))) * msl;
			msr = (((float)(log10(max_sens - OBS_RANGE))) / log10(max_sens)) * potential_right + (1 - ((float)(log10(max_sens - OBS_RANGE)) / log10(max_sens))) * msr;
		}

		limit(&msl, 999);
		limit(&msr, 999);

		// Set speed
		msl_w = msl * MAX_SPEED_WEB / 1000;
		msr_w = msr * MAX_SPEED_WEB / 1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);

		optim_state_t optim_state = optim_update(ds, NB_SENSORS, msl_w, msr_w);
		if (optim_state == OPTIM_CHANGE_CONFIG) {
			printf("Change config %d %d %f...\n", optim_config.n_params, optim_config.n_iters, optim_config.params[0]);
			wb_robot_init();
			reinitialize_states();
			msl = msr = 0;
			//aggregation_threshold = optim_config.params[0];
		}
		if (optim_state == OPTIM_SEND_STATS) {
			printf("Sending stats: %f %f...\n", optim_stats.sum_prox, optim_stats.max_prox);
		}



		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}
