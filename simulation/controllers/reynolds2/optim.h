#ifndef _OPTIM_H_
#define _OPTIM_H_

#include <stdbool.h>
#include <stdint.h>
#include <webots/device.h>

typedef struct _optim_stats_t {
  double max_prox;
  double sum_prox;
  int iter_counter;
  double max_speed;
  double sum_speed;
  bool done;
} optim_stats_t;

typedef struct _optim_config_t {
  double params[40];
  int n_params;
  int n_iters;
} optim_config_t;

typedef enum _optim_state_t {
  OPTIM_RECV_CONFIG = 0,
  OPTIM_CHANGE_CONFIG,
  OPTIM_COLLECT_STATS,
  OPTIM_SEND_STATS
} optim_state_t;

optim_stats_t optim_stats;
optim_config_t optim_config;

optim_state_t optim_update(WbDeviceTag *ds, int ds_n,  double msl_w, double msr_w);
void optim_init();

#endif