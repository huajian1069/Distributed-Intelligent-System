#include "metrics.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>


#define DIMENSION 2

double loc[FLOCK_SIZE][3];
WbNodeRef robots[FLOCK_SIZE];
int k = 0;
double avg_pt = 0.0;
double prev_avg[2] = {0, 0};
float absolute_migration[2];
char log_filename[100];

static void get_robot_position(double position[], WbNodeRef robot);
static double dist(double *x, double *avg);
static void compute_fitness();
static float dot(float *x, float *y);
static inline float max(float x, float y);

static void get_robot_position(double position[], WbNodeRef robot)
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
    double initial_avg_location[] = {0, 0, 0};

    for (int i = 0; i < FLOCK_SIZE; i++)
    {
        robots[i] = p_robots[i];
        get_robot_position(loc[i], robots[i]);

        initial_avg_location[0] += loc[i][0] / FLOCK_SIZE;
        initial_avg_location[1] += loc[i][1] / FLOCK_SIZE;
        initial_avg_location[2] += loc[i][2] / FLOCK_SIZE;

        printf("Migration urge: %f %f\n", absolute_migration[0], absolute_migration[1]);
    }

    absolute_migration[0] = initial_avg_location[0] + 4;
    absolute_migration[1] = initial_avg_location[1];

    sprintf(log_filename, "%s/webots_pso_%lu.csv", getenv("HOME"), (unsigned long)time(NULL));
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

static float dot(float *x, float *y)
{
    return x[0] * y[0] + x[1] * y[1];
}

static inline float max(float x, float y)
{
    return x > y ? x : y;
}

void metrics_save(double *params, int n_params, double fitness)
{
    FILE *fp;
    fp = fopen(log_filename, "a");

    for (int i = 0; i < n_params; i++)
    {
        fprintf(fp, "%f,", params[i]);
    }
    fprintf(fp, "%f\r\n", fitness);
    fclose(fp);
}

void metrics_update(WbNodeRef *p_robots)
{
    for (int i = 0; i < FLOCK_SIZE; i++)
    {
        get_robot_position(loc[i], p_robots[i]);
    }
    compute_fitness();
}

static double dist(double *x, double *avg)
{
    double s = 0;
    for (int i = 0; i < DIMENSION; i++)
    {
        s += pow((x[i] - avg[i]), 2.0);
    }
    return sqrt(s);
}

static void compute_fitness()
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
    float relative_urge[2];
    float norm;
    float dif_avg[2];

    relative_urge[0] = absolute_migration[0] - avg[0];
    relative_urge[1] = absolute_migration[1] - avg[1];
    norm = sqrt(pow(relative_urge[0], 2) + pow(relative_urge[1], 2));
    relative_urge[0] /= norm;
    relative_urge[1] /= norm;

    dif_avg[0] = avg[0] - prev_avg[0];
    dif_avg[1] = avg[1] - prev_avg[1];
    vt = max(dot(relative_urge, dif_avg), 0) / WEBOTS_MAX_VELOCITY;
    prev_avg[0] = avg[0];
    prev_avg[1] = avg[1];

    // Performance(instant)
    pt = ot * ct * vt;

    // performance(overall)
    k++;
    avg_pt = avg_pt + (pt - avg_pt) / k;
}

double metrics_get_performance()
{
    return avg_pt;
}
