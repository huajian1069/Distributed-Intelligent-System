#ifndef _CONFIG_H_
#define _CONFIG_H_

#define FLOCK_SIZE	5 
#define WEBOTS_MAX_VELOCITY 6.28
#define ROBOTS 5
#define MAX_ROB ROBOTS

#define DATASIZE 6
#define SWARMSIZE 6

#define ROB_RAD 0.035
#define ARENA_SIZE 0.94

/* PSO definitions */
#define NB 1          // Number of neighbors on each side
#define LWEIGHT 2.0   // Weight of attraction to personal best
#define NBWEIGHT 2.0  // Weight of attraction to neighborhood best
#define VMAX 2.0     // Maximum velocity particle can attain
#define MININIT 0.0 // Lower bound on initialization value
#define MAXINIT 1.0  // Upper bound on initialization value
#define ITS 200        // Number of iterations to run

/* Neighborhood types */
#define STANDARD -1
#define RAND_NB 0
#define NCLOSE_NB 1
#define FIXEDRAD_NB 2

/* Fitness definitions */
#define FIT_ITS 600 // Number of fitness steps to run during optimization

#define FINALRUNS 10
#define NEIGHBORHOOD FIXEDRAD_NB
#define RADIUS 0.8


#endif