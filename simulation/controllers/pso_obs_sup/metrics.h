#ifndef _METRICS_H_
#define _METRICS_H_

#include <webots/robot.h>
#include <webots/supervisor.h>
#include "config.h"

void metrics_reset();
void metrics_update(WbNodeRef* p_robots);
void metrics_init(WbNodeRef* p_robots);
double metrics_get_performance();

#endif