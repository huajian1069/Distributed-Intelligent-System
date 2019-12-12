#include <stdio.h>
#include <math.h>
#include "config.h"
#include "pso.h"
#include "metrics.h"
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

static WbNodeRef robs[MAX_ROB];
WbDeviceTag emitter[MAX_ROB];
WbDeviceTag rec[MAX_ROB];
const double *loc[MAX_ROB];
const double *rot[MAX_ROB];
double initial_loc[MAX_ROB][3];
double initial_rot[MAX_ROB][4];

void calc_fitness(double[][DATASIZE], double[], int, int);
void nRandom(int[][SWARMSIZE], int);
void nClosest(int[][SWARMSIZE], int);
void fixedRadius(int[][SWARMSIZE], double);

/* RESET - Get device handles and starting locations */
void reset(void)
{
  int i;

  
  for (i = 0; i < ROBOTS; i++)
  {
    char robot_name[15];
    char emitter_name[15];
    char receiver_name[15];
    sprintf(robot_name, "epuck_%d_%d", i / FLOCK_SIZE, i % FLOCK_SIZE);
    sprintf(emitter_name, "emitter%d", i);
    sprintf(receiver_name, "receiver%d", i);

    robs[i] = wb_supervisor_node_get_from_def(robot_name);
    loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i], "translation"));
    initial_loc[i][0] = loc[i][0];
    initial_loc[i][1] = loc[i][1];
    initial_loc[i][2] = loc[i][2];
    rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i], "rotation"));
    initial_rot[i][0] = rot[i][0];
    initial_rot[i][1] = rot[i][1];
    initial_rot[i][2] = rot[i][2];
    initial_rot[i][3] = rot[i][3];
    emitter[i] = wb_robot_get_device(emitter_name);
    rec[i] = wb_robot_get_device(receiver_name);
  }

  metrics_init(robs);
  
}

/* MAIN - Distribute and test conctrollers */
int main()
{
  double *weights;    // Optimized result
  double buffer[255]; // Buffer for emitter
  int i, j, k;        // Counter variables

  /* Initialisation */
  wb_robot_init();
  printf("Particle Swarm Optimization Super Controller\n");
  reset();
  for (i = 0; i < MAX_ROB; i++)
  {
    wb_receiver_enable(rec[i], 32);
  }

  wb_robot_step(256);

  double fit;                  // Fitness of the current FINALRUN
  double endfit;               // Best fitness over 10 runs
  double w[MAX_ROB][DATASIZE]; // Weights to be send to robots (determined by pso() )
  double f[MAX_ROB];           // Evaluated fitness (modified by calc_fitness() )
  double bestfit, bestw[DATASIZE];

  /* Evolve controllers */
  endfit = 0.0;
  bestfit = 0.0;

  // Do 10 runs and send the best controller found to the robot
  for (j = 0; j < 10; j++)
  {

    // Get result of optimization
    weights = pso(SWARMSIZE, NB, LWEIGHT, NBWEIGHT, VMAX, MININIT, MAXINIT, ITS, DATASIZE, 1);

    // Set robot weights to optimization results
    fit = 0.0;
    for (i = 0; i < MAX_ROB; i++)
    {
      for (k = 0; k < DATASIZE; k++)
        w[i][k] = weights[k];
    }

    // Run FINALRUN tests and calculate average
    for (i = 0; i < FINALRUNS; i += 1)
    {
      calc_fitness(w, f, FIT_ITS, 1);
      for (k = 0; k < 1 && i + k < FINALRUNS; k++)
      {
        fit += f[k];
      }
    }
    fit /= FINALRUNS;

    // Check for new best fitness
    if (fit > bestfit)
    {
      bestfit = fit;
      for (i = 0; i < DATASIZE; i++)
      {
        bestw[i] = weights[i];
      }
    }

    printf("Performance of the best solution: %.3f\n", fit);
    endfit += fit / 10; // average over the 10 runs
  }
  printf("~~~~~~~~ Optimization finished.\n");
  printf("Best performance: %.3f\n", bestfit);
  printf("Average performance: %.3f\n", endfit);

  /* Send best controller to robots */
  for (j = 0; j < DATASIZE; j++)
  {
    buffer[j] = bestw[j];
  }
  buffer[DATASIZE] = 1000000;
  for (i = 0; i < ROBOTS; i++)
  {
    wb_emitter_send(emitter[i], (void *)buffer, (DATASIZE + 1) * sizeof(double));
  }

  /* Wait forever */
  while (1)
    wb_robot_step(64);

  return 0;
}

void initialise_position(int rob_id)
{
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id], "translation"), initial_loc[rob_id]);
  wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id], "rotation"), initial_rot[rob_id]);
  wb_supervisor_node_restart_controller(robs[rob_id]);
}

// Distribute fitness functions among robots
void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs)
{
  double buffer[255];
  double *rbuffer;
  int i, j;

  // Initialise robots for fitness
    wb_supervisor_simulation_reset_physics();

  for (i = 0; i < ROBOTS; i++)
  {
    initialise_position(i);
  }

for (i = 0; i < ROBOTS; i++) {
    buffer[0] = DATASIZE;
    for (j = 0; j < DATASIZE; j++)
    {
      buffer[j + 1] = weights[0][j];
    }
    buffer[DATASIZE + 1] = its;
    wb_emitter_send(emitter[i], (void *)buffer, (DATASIZE + 2) * sizeof(double));
  }
  metrics_reset();

  // Keep robots move and collect data
  while (wb_receiver_get_queue_length(rec[0]) == 0)
  {
    wb_robot_step(64);
    metrics_update(robs);
  }

  // Get fitness, end of run
  for (i = 0; i < numRobs; i++)
  {
    rbuffer = (double *)wb_receiver_get_data(rec[i]);
    fit[i] = metrics_get_performance();

    printf("Fitness: %f\n", metrics_get_performance());

    wb_receiver_next_packet(rec[i]);
  }

  metrics_save(buffer + 1, DATASIZE, metrics_get_performance());
}

/* Optimization fitness function , used in pso.c */
/************************************************************************************/
/* Use the NEIHBORHOOD definition at the top of this file to                        */
/* change the neighborhood type for the PSO. The possible values are:               */
/* STANDARD    : Local neighborhood with 2*NB (defined above) nearest neighbors     */
/*               NEIGHBORHOOD is set to STANDARD by default                         */
/* RAND_NB     : 2*NB random neighbors                                              */
/* NCLOSE_NB   : 2*NB closest neighbors                                             */
/* FIXEDRAD_NB : All robots within a defined radius are neighbors                   */
/************************************************************************************/
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE])
{
  calc_fitness(weights, fit, FIT_ITS, ROBOTS);

#if NEIGHBORHOOD == RAND_NB
  nRandom(neighbors, 2 * NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
  nClosest(neighbors, 2 * NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
  fixedRadius(neighbors, RADIUS);
#endif
}

/* Get distance between robots */
double robdist(int i, int j)
{
  const double* trans_i;
  const double* trans_j;
  trans_i = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i], "translation"));
  trans_j = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[j], "translation"));

  return sqrt(pow(trans_i[0] - trans_j[0], 2) + pow(trans_i[2] - trans_j[2], 2));
}

/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB)
{

  int i, j;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++)
  {

    /* Clear old neighbors */
    for (j = 0; j < ROBOTS; j++)
      neighbors[i][j] = 0;

    /* Set new neighbors randomly */
    for (j = 0; j < numNB; j++)
      neighbors[i][(int)(SWARMSIZE * rnd())] = 1;
  }
}

/* Choose the n closest robots */
void nClosest(int neighbors[SWARMSIZE][SWARMSIZE], int numNB)
{

  int r[numNB];
  int tempRob;
  double dist[numNB];
  double tempDist;
  int i, j, k;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++)
  {

    /* Clear neighbors */
    for (j = 0; j < numNB; j++)
      dist[j] = ARENA_SIZE;

    /* Find closest robots */
    for (j = 0; j < ROBOTS; j++)
    {

      /* Don't use self */
      if (i == j)
        continue;

      /* Check if smaller distance */
      if (dist[numNB - 1] > robdist(i, j))
      {
        dist[numNB - 1] = robdist(i, j);
        r[numNB - 1] = j;

        /* Move new distance to proper place */
        for (k = numNB - 1; k > 0 && dist[k - 1] > dist[k]; k--)
        {

          tempDist = dist[k];
          dist[k] = dist[k - 1];
          dist[k - 1] = tempDist;
          tempRob = r[k];
          r[k] = r[k - 1];
          r[k - 1] = tempRob;
        }
      }
    }

    /* Update neighbor table */
    for (j = 0; j < ROBOTS; j++)
      neighbors[i][j] = 0;
    for (j = 0; j < numNB; j++)
      neighbors[i][r[j]] = 1;
  }
}

/* Choose all robots within some range */
void fixedRadius(int neighbors[SWARMSIZE][SWARMSIZE], double radius)
{
  int i, j;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++)
  {
    /* Find robots within range */
    for (j = 0; j < ROBOTS; j++)
    {
      if (i == j)
        continue;

      if (robdist(i, j) < radius)
        neighbors[i][j] = 1;
      else
        neighbors[i][j] = 0;
    }
  }
}

void step_rob()
{
  wb_robot_step(64);
}