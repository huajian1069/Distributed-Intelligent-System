#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>



#define NB_SENSORS	  8	  // Number of distance sensors
#define FLOCK_SIZE	  5	  // Size of flock
#define MAX_SPEED         800     // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define TIME_STEP	  64	  // [ms] Length of time step
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)

WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node



int robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
char* robot_name;
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {50, 50};	        // Migration vector
float my_position[3];


static void reset() 
{
	wb_robot_init();

	//get receiver and emitter
	receiver2 = wb_robot_get_device("receiver");
	emitter2 = wb_robot_get_device("emitter");
	wb_receiver_enable(receiver2,64);

	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
	
	//get infra-red distance sensor
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	for(i=0;i<NB_SENSORS;i++)
          wb_distance_sensor_enable(ds[i],64);

	//get robot name and id
	robot_name=(char*) wb_robot_get_name(); 
	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id); // read robot id from the robot's name
	robot_id = robot_id%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	for(i=0; i<FLOCK_SIZE; i++) 
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
        
    //set abosute coordinates of robots' starting position in scenaro one: 
    //initial x and z coordinate
    my_position[0] = -2.9;
    my_position[2] = -1.57;
    //initial theta angle
    switch (robot_id){
    	case 0:
    		my_position[1] = 0;
    		break;
    	case 1:
    		my_position[1] = 0.1;
    		break;
    	case 2:
    		my_position[1] = -0.1;
    		break;
    	case 3:
    		my_position[1] = 0.21; // very close to 0.2, ideal: 0.2
    		break;
    	case 4:
    		my_position[1] = -0.2;
    		break;
    }

    printf("Reset: robot %d\n",robot_id);
}

void update_self_motion(int msl, int msr) { 
	float theta = my_position[2];
  
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
  
	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;
  
	// Keep orientation within -pi, pi
	if (my_position[2] > M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < -M_PI) my_position[2] += 2.0*M_PI;
	printf("epcuk %d: x=%f, z=%f, theta=%f\n", robot_id, my_position[0], my_position[1], my_position[2]);
}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

void Braitenberg(int *bmsl, int *bmsr, int *sum_sensors, int *max_sens){

	int distances[NB_SENSORS];	// Array for the distance sensor readings
	for(int i=0;i<NB_SENSORS;i++) 
	{
		distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
        *sum_sensors += distances[i]; // Add up sensor values
        *max_sens = *max_sens>distances[i]?*max_sens:distances[i]; // Check if new highest sensor value
        if(robot_id==2) printf("distances[%d]=%d\t", i, distances[i]);
        // Weighted sum of distance sensor values for Braitenburg vehicle
        *bmsr += e_puck_matrix[i] * distances[i];
        *bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
    }
    if(robot_id==2) printf("\n");
	// Adapt Braitenberg values (empirical tests)
    *bmsl/=MIN_SENS; *bmsr/=MIN_SENS;
}

float Reynolds(int *msl, int *msr, int max_sens){

    float weight_rey;			// weight of reynolds rule in combined speed
	//impose migrate urge
	*msl = 10 * migr[0] + 100*(my_position[2] + 1.57)*MIN_SENS/max_sens;
	*msr = 10 * migr[1] - 100*(my_position[2] + 1.57)*MIN_SENS/max_sens;
	
	// Adapt speed instinct to distance sensor values
	//if (max_sens > MAX_SENS*0.7) {
	weight_rey = 1 - max_sens/(2*MAX_SENS);
	*msl *= weight_rey;
	*msr *= weight_rey;
	//}
	return weight_rey;
}

void epuck_move(int msl, int msr){
	msl = msl*MAX_SPEED_WEB/1000;
	msr = msr*MAX_SPEED_WEB/1000;
	wb_motor_set_velocity(left_motor, msl);
    wb_motor_set_velocity(right_motor, msr);
}

// the main function
int main(){ 
    int msl, msr;			// Wheel speeds
    int bmsl=0, bmsr=0;
    int sum_sensors, max_sens;  // Store highest sensor value   

 	reset();			// Resetting the robot
	// Forever
	for(;;){

        sum_sensors = 0;
		max_sens = 0;

        Braitenberg(&bmsl, &bmsr, &sum_sensors, &max_sens);       
        printf("epuck%d: bmsl= %d, bmsr= %d\n", robot_id, bmsl, bmsr);

        float weight_rey = Reynolds(&msl, &msr, max_sens);
		printf("epuck%d:  weight_rey= %f, msl= %d, msr= %d\n", robot_id, weight_rey, msl, msr);

		// combine speed of migrate and Braitenbergs, impose speed upper bound
		msl += bmsl;
		msr += bmsr;
      	limit(&msl,1000);
      	limit(&msr,1000);
      	printf("epuck%d: msl= %d, msr= %d\n", robot_id, msl, msr);

      	// implement odometry 
      	update_self_motion(msl, msr);

		// actually activate motors
		epuck_move(msl, msr);

		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  
