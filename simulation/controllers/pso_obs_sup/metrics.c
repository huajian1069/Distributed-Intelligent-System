#include "metrics.h"

#include <stdio.h>

#define DIMENSION 2

double loc[FLOCK_SIZE][3];
WbNodeRef robots[FLOCK_SIZE];
int k = 0;
double avg_pt = 0.0;
double prev_avg[2] = {0, 0};

void get_robot_position(double position[], WbNodeRef robot);
double dist(double *x, double *avg);
void compute_fitness();

void get_robot_position(double position[], WbNodeRef robot)
{
    const double *translation;
    const double *rotation;

    translation = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robot, "translation"));
    rotation = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robot, "rotation"));

    position[0] = translation[0];
    position[1] = translation[2];
    position[2] = rotation[3];
}

void metrics_init(WbNodeRef *p_robots)
{
    for (int i = 0; i < FLOCK_SIZE; i++)
    {
        robots[i] = p_robots[i];
        get_robot_position(loc[i], robots[i]);
    }
}

void metrics_reset()
{
    // Compute performance indices
    // Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
    // the migration goal
    k = 0;
    avg_pt = 0.0;
    prev_avg[0] = 0;
    prev_avg[1] = 0;
}

void metrics_update(WbNodeRef *p_robots)
{
    for (int i = 0; i < FLOCK_SIZE; i++)
    {
        get_robot_position(loc[i], p_robots[i]);
    }
    compute_fitness();
}

double dist(double *x, double *avg)
{
    double s = 0;
    for (int i = 0; i < DIMENSION; i++)
    {
        s += pow((x[i] - avg[i]), 2.0);
    }
    return sqrt(s);
}

void compute_fitness()
{
    //Orientation: measure the alignment bewtween robots
    double sumSin = 0, sumCos = 0;
    double ot, ct, vt, pt;
    for (int i = 0; i < FLOCK_SIZE; i++)
    {
        sumCos += cos(loc[i][2]);
        sumSin += sin(loc[i][2]);
    }
    ot = sqrt(pow(sumCos, 2) + pow(sumSin, 2)) / FLOCK_SIZE;

    //Cohesion: measure the dispersion of robots,
    double avg[2] = {0, 0};
    for (int i = 0; i < FLOCK_SIZE; i++)
    {
        avg[0] += loc[i][0];
        avg[1] += loc[i][1];
    }
    avg[0] /= FLOCK_SIZE;
    avg[1] /= FLOCK_SIZE;
    float sumDis = 0;
    for (int i = 0; i < FLOCK_SIZE; i++)
    {
        sumDis += dist(loc[i], avg);
    }
    ct = 1.0 / (1.0 + sumDis / FLOCK_SIZE);

    // Velocity: measure the average displacement velocity of the centers of mass along the direction of the migratory urge
    vt = 1;

    // Performance(instant)
    pt = ot * ct * vt;

    // performance(overall)
    k++;
    avg_pt = avg_pt + (pt - avg_pt) / k;
}

double metrics_get_performance() {
    return avg_pt;
}
