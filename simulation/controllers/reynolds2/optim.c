#include "optim.h"

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <stdlib.h>

WbDeviceTag emitter;
WbDeviceTag receiver;
optim_state_t state;

void collect_stats(WbDeviceTag *ds, int ds_n, double msl_w, double msr_w);
bool recv_config();
void send_stats();
void reset();


void reset() {
	optim_stats.max_prox = 0;
	optim_stats.sum_prox = 0;
	optim_stats.max_prox = 0;
	optim_stats.sum_prox = 0;
	optim_stats.iter_counter = 0;
}

void optim_init()
{
	emitter = wb_robot_get_device("emitter_super");
	receiver = wb_robot_get_device("receiver_super");
	wb_receiver_enable(receiver, 32);
	state = OPTIM_RECV_CONFIG;
	reset();
}

optim_state_t optim_update(WbDeviceTag *ds, int ds_n, double msl_w, double msr_w)
{
	switch (state)
	{
	case OPTIM_RECV_CONFIG:
		if (recv_config())
		{
			state = OPTIM_CHANGE_CONFIG;
		}
		break;
	case OPTIM_CHANGE_CONFIG:
		state = OPTIM_COLLECT_STATS;
		break;
	case OPTIM_COLLECT_STATS:
		collect_stats(ds, ds_n, msl_w, msr_w);
		if (optim_stats.iter_counter >= optim_config.n_iters)
		{
			state = OPTIM_SEND_STATS;
		}
		break;
	case OPTIM_SEND_STATS:
		send_stats();
		state = OPTIM_RECV_CONFIG;
		break;
	}

	return state;
}

bool recv_config()
{
	double *rbuffer;

	// Check if the packet exists
	if (wb_receiver_get_queue_length(receiver) == 0)
	{
		return false;
	}

	// Parse the packet
	rbuffer = (double *)wb_receiver_get_data(receiver);
	optim_config.n_params = rbuffer[0];
	for (int i = 0; i < optim_config.n_params; i++)
	{
		optim_config.params[i] = rbuffer[i + 1];
	}
	optim_config.n_iters = rbuffer[optim_config.n_params + 1];

	return true;
}

void collect_stats(WbDeviceTag *ds, int ds_n, double msl_w, double msr_w)
{
	double max_prox = 0;
	double max_speed = (abs(msl_w) > abs(msr_w)) ? abs(msl_w) : abs(msr_w);

	// Speed should not exceed 6.28
	if (max_speed > 6) { 
		if (max_speed > optim_stats.max_speed) {
			optim_stats.max_speed = max_speed;
		}
		optim_stats.sum_speed += max_speed;
	}

	// Find the smallest distance
	for (int i = 0; i < ds_n; i++)
	{
		double prox = wb_distance_sensor_get_value(ds[i]);
		if (prox > max_prox)
		{
			max_prox = prox;
		}
	}

	// Update stats
	optim_stats.iter_counter++;
	if (max_prox > 200)
	{
		if (max_prox > optim_stats.max_prox)
		{
			optim_stats.max_prox = max_prox;
		}
		optim_stats.sum_prox += max_prox;
	}
}

void send_stats()
{
	double buffer[255];
	double fit;

	// Calculate fitness
	fit = optim_stats.max_prox + (optim_stats.sum_prox / optim_stats.iter_counter);
	fit += optim_stats.max_speed + (optim_stats.sum_speed / optim_stats.iter_counter) * 100000000;

	// Send fitness
	buffer[0] = fit;
	wb_emitter_send(emitter, (void *)buffer, sizeof(double));
	wb_receiver_next_packet(receiver);

	// Reset stats
	reset();
}