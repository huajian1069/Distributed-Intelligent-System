/*****************************************************************************/
/* File:         performance_estimation.c                                    */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  estimating the performance of a formation 		     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi				     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	5 		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step
#define DIMENSION 2
#define WEBOTS_MAX_VELOCITY 6.28

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define RULE1_THRESHOLD 0.2
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0


int offset;				// Offset of robots number
float migr[2];			// Migration vector
float orient_migr; 			// Migration orientation
int t;

/*
 * Initialize flock position and devices
 */
void reset(bool flag) {
	wb_robot_init();
	char rob[10] = "epuck_0_0";
	if(flag) rob[6] = '1';
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		printf("%s\n", rob);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
		rob[8]++;
	}
}

/*
 * Utilise function
*/

float dist(float* x, float* avg){
	float s = 0;
	for(int i = 0; i < DIMENSION; i++){
		s += pow((double)(x[i] - avg[i]), 2.0); 
	}
	return (float)sqrt((double)s);
}

/*
*	inner (dot) product of vectors
*/

float dot(float *x, float *y){
	return x[0] * y[0] + x[1] * y[1];
}

float max(float x, float y){
	return x > y ? x : y;
}

/*
*	get the normalized migratory vector
*/
void getMigratoryUrge(float *migrUrg, float *avg){
	float norm;
	migrUrg[0] = migr[0] - avg[0];
	migrUrg[1] = migr[1] - avg[1];
	norm = sqrt(pow(migrUrg[0], 2) + pow(migrUrg[1], 2));
	migrUrg[0] /= norm;
	migrUrg[1] /= norm;
}


/*
 * Compute performance metric.
 */
void compute_fitness() {
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	static int k = 0, i = 0;
	static float avg_pt = 0.0;
	static float prev_avg[2] = {0, 0};

	//Orientation: measure the alignment bewtween robots
	float sumSin=0, sumCos=0;
	float ot, ct, vt, pt; 
	for(i=0; i<FLOCK_SIZE; i++){
		sumCos += cos(loc[i][2]);
		sumSin += sin(loc[i][2]);
	}
	ot = sqrt( pow(sumCos, 2) + pow(sumSin, 2) ) / FLOCK_SIZE;

	//Cohesion: measure the dispersion of robots,
	float avg[2]={0, 0};
	for(i=0; i<FLOCK_SIZE; i++){
		avg[0] += loc[i][0];
		avg[1] += loc[i][1];
	}
	avg[0] /= FLOCK_SIZE;
	avg[1] /= FLOCK_SIZE;
	float sumDis = 0;
	for(i=0; i<FLOCK_SIZE; i++){
		sumDis += dist(loc[i], avg);
	}
	ct = 1.0 / (1.0 + sumDis / FLOCK_SIZE);

	// Velocity: measure the average displacement velocity of the centers of mass along the direction of the migratory urge
	float migrUrg[2];
	getMigratoryUrge(migrUrg, avg);
	float dif_avg[2];
	dif_avg[0] = avg[0] - prev_avg[0];
	dif_avg[1] = avg[1] - prev_avg[1];
	vt = max( dot(migrUrg, dif_avg), 0) / WEBOTS_MAX_VELOCITY ;
	prev_avg[0] = avg[0];
	prev_avg[1] = avg[1];

	// Performance(instant)
	pt = ot * ct * vt;

	// performance(overall)
	k++;
	avg_pt = avg_pt + (pt - avg_pt) / k;
	printf("time:%d, Orientation: %f, Cohesion: %f, Velocity: %f, Performance(instant): %f, Performance(overall): %f\n", 
			k, ot, ct, vt, pt, avg_pt);	
}



/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;			// Index

	if(argc != 4){
		printf("Please specify the migration urge from webots world file");
		exit(1);
	}
	migr[0] = atof(args[1]);
	migr[1] = atof(args[3]);

	bool flag;
	if(!strcmp(args[1],"-0.1")) flag = true;
	else flag = false;
	printf("%s  ,  %f,   %d\n", args[1], migr[0], flag);
	reset(flag);
	// Compute reference fitness values
	

	for(;;) {
		wb_robot_step(TIME_STEP);
		
		if (t % 10 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
    			}
			//Compute and normalize fitness values
			compute_fitness();		
			
		}
		
		t += TIME_STEP;
	}

}
