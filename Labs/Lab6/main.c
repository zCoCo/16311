#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S1,     sonarSensor,         sensorSONAR)
/**********************************************
 * Lab 6 :
 * Use Bayesian Localization to Determine Position in a Given Map Composed of 16
 * Blocks Arranged around a Circle. Bit Vector of Whether Blocks are Present at
 * Each Posible Location is Given.
 * Team 15
 **********************************************/

//Global variables - you will need to change some of these
//Robot's positions

#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Positioning/PositioningStack.h"
#include "Toolbox/HALs/HAL.h"

#define MainClock T1

// ---- WORLD DATA ---- //
#define MAP_SIZE 16
int bitmap[MAP_SIZE] = {0,1,0,1, 0,1,1,0, 0,0,1,1, 1,1,0,1};

void create_bitmap_from_hex(int hex_in){
	int mask = 1;
	for(int i=0; i<MAP_SIZE; i++){
	    bitmap[i] = (hex_in >> i) & mask;
	}
} // #create_bitmap_from_hex

// Target Stop Location:
#define TARGET_LOCATION (0) // Zero-Indexed

// Returns the Value of the Map at the Given Position (adjusting for Wrap-Arounds)
int get_map_val(long pos){
	int idx = pos % MAP_SIZE;
	idx = (idx<0) ? (MAP_SIZE + idx - 1) : idx; // idx=-2 -> 16+-2-1 -> idx=13 (14th pos)
	return bitmap[idx];
} // #get_map_pos

// ---- BLOCK SENSOR DATA ---- //
#define SONAR_THRESH 25 //Inclusive
// Returns Whether the Robot Sees a Block at its Current Position
int at_block(){
	static int seeing_block;
	seeing_block = ( (SensorValue[sonarSensor] <= SONAR_THRESH) ? 1 : 0 );
	return seeing_block;
}

// ---- MOTION DATA ---- //
// Amount by Which Line-Following Oscillations cause the Travelled Distance to
// Exceed the Distance Travelled Around the Circle
float LINEAR_OVERDRIVE_FACTOR = (3*16.0 + 1.0) / (3*16.0);
// Radius of the Track (meters)
#define TRACK_RADIUS (0.3048)
float BLOCKS_PER_METER = 16.0 * LINEAR_OVERDRIVE_FACTOR / TRACK_RADIUS / 6.28318;

// ---- POSITION DATA ---- //
// Change in Blocks since Initialization based on Odometry Alone.
float DBlock_odo = 0.0;

// ---- PROBABILITY DATA ---- //
float probmap[MAP_SIZE] = {1,1,1,1,
										 			 1,1,1,1,
						 	 		 	 			 1,1,1,1,
						 	 		 	 			 1,1,1,1}; // Initial Assumption is of Equal Probability for All Locations

float block_location = 0; // Best Estimate of Current Block Location

void normalize_prob_map(){
	float sum = 0.0;
	for(int i=0; i<MAP_SIZE; i++){ // Find Sum
		sum += probmap[i];
	}
	if(sum != 0){
		for(int i=0; i<MAP_SIZE; i++){ // Normalize
			probmap[i] = probmap[i] / sum;
		}
	}
} // #normalize_prob_map

// Circularly Shift the Probility Map Left (offsetting clockwise motion of robot) by n Spaces.
void shift_prob_map(int n){
	int amt = n % MAP_SIZE;
	float temp[MAP_SIZE];
	for(int i=0; i<MAP_SIZE; i++){ // Copy into temp
		temp[i] = probmap[i];
	}
	for(int i=0; i<(MAP_SIZE-amt); i++){
		probmap[i] = temp[i+amt];
	}
	for(int i=0; i<amt; i++){
		probmap[MAP_SIZE - amt + i] = temp[i];
	}
} // #shift_prob_map
// Circularly Shift the Probility Map Right (matching clockwise motion of robot) by n Spaces.
void shift_prob_map_right(int n){
	int amt = n % MAP_SIZE;
	float temp[MAP_SIZE];
	for(int i=0; i<MAP_SIZE; i++){ // Copy into temp
		temp[i] = probmap[i];
	}
	for(int i=0; i<amt; i++){
		probmap[i] = temp[MAP_SIZE - amt + i];
	}
	for(int i=0; i<(MAP_SIZE-amt); i++){
		probmap[i+amt] = temp[i];
	}
} // #shift_prob_map_right

// Perform a Discrete Bayes Filter Update
#define PROB_INC (4.0) // Factor to Increase Probability by if Map agrees with Measurement
#define PROB_DEC ((1.0) / PROB_INC)
void update_bayes(){
	static int last_DBlock_odo = 0;
	static int not_gone = 1;
	int Du = ((int) (floor(DBlock_odo))) - last_DBlock_odo; // Distance Travelled in Blocks since Last
																												 //  Bayes Update was Performed.

	if(Du >=  1 || not_gone){
		last_DBlock_odo = floor(DBlock_odo);
		not_gone = 0;

		shift_prob_map_right(Du); // Shift Probability Map by Action Distance

		int z = at_block(); // Measurement

		if(z){
			playTone(780, 10);
		} else{
			playTone(445, 15);
		}

		for(int i=0; i<MAP_SIZE; i++){
			probmap[i] *= (bitmap[i] == z) ? PROB_INC : PROB_DEC;
		}

		normalize_prob_map(); // Renormalize

		// Find Most Likely Block Location (highest probability in probmap represents
		// starting location.
		float max_prob = 0.0;
		for(int i=0; i<MAP_SIZE; i++){
			if(probmap[i] > max_prob){
				max_prob = probmap[i];
				block_location = i;
			}
		}

	} // Du >= 1
} // #update_bayes

void update_block_location(){
	DBlock_odo = BLOCKS_PER_METER * TSF_Last(Hist_Dist); // Update Odo Estimate

	update_bayes(); // Update Probabilistic Estimate (used Odo -> u)
} // #update_block_location

// --- LIGHT SENSOR DATA ---- //

#define SearchTimer T3

#define SEARCH_LIGHT_THRESH 35
void search__i_a(){
	static bool first_call = true;
	static float last_th = 0.0;
	static float flipped = false;
	if(first_call || SensorValue[lightSensor] < SEARCH_LIGHT_THRESH){
		clearTimer(SearchTimer);
		first_call = false;
		last_th = rob_pos_TH;
	}
	if(time1[SearchTimer] > 350){
		static int dir = 1;
		while(SensorValue[lightSensor] > SEARCH_LIGHT_THRESH){
			if(adel(rob_pos_TH, last_th) < 120*DEG && !flipped && dir>0
			|| adel(rob_pos_TH, last_th) > -120*DEG &&!flipped && dir<0){
				moveAt(0,dir*0.35*MAX_OMEGA);
			} else {
				while(adel(rob_pos_TH, last_th) > 0 && dir>0
					 || adel(rob_pos_TH, last_th) < 0 && dir<0){
					moveAt(0,-dir*0.8*MAX_OMEGA);
					nxtSetPixel(50 + (int)(135.0 * 0.0), 32 + (int)(100.0 * 0.0));
					nxtDisplayTextLine(0, "L: %d", SensorValue[lightSensor]);
					nxtDisplayTextLine(1, "t: %dms", TSF_Last(Hist_Time));
					wait1Msec(1);
				}
				flipped = true;
			}
			if(flipped){
				moveAt(0,-dir*0.35*MAX_OMEGA);
			}
			wait1Msec(2);
		}
		flipped = false;
		moveAt(0,0);
	}
}

// ---- DISPLAY --- //
void update_display(){
	nxtSetPixel(50 + (int)(100.0 * 0.0), 32 + (int)(100.0 * 0.0));
	nxtDisplayTextLine(0, "L: %d", SensorValue[lightSensor]);
	nxtDisplayTextLine(1, "S: %d", SensorValue[sonarSensor]);
	nxtDisplayTextLine(2, "DB: %f", DBlock_odo);
	nxtDisplayTextLine(3, "Block: %d", block_location);
	nxtDisplayTextLine(4, "t: %dms", TSF_Last(Hist_Time));
}

/*****************************************
 * Main function
 *****************************************/

float rightMotorSpeed = 0;
float leftMotorSpeed = 0;
int start_block = 0;

int orientaion_started = 0;
float DB_at_orientation_start = 0.0
int orientation_done = 0;
task main()
{
	// Team 15 PID Code
	#define Kp 2 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
	#define Kd (0.2) // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
	#define RIGHT_MAX_SPEED 60 // max speed of the robot
	#define LEFT_MAX_SPEED 60  // max speed of the robot
	#define RIGHT_BASE_SPEED 20 // this is the speed at which the motors should spin when the robot is perfectly on the line
	#define LEFT_BASE_SPEED 20 // this is the speed at which the motors should spin when the robot is perfectly on the line
	static float lastError = 0;

	init_HAL();
	startTask(odometry);

	normalize_prob_map(); // Initial Call

	//Pre-Allocate:
	static float error, motorPower;
	while(DBlock_odo < 16.0
	   || ((block_location < TARGET_LOCATION) && (start_block <= TARGET_LOCATION))
 	   || (((start_block + DBlock_odo) < (TARGET_LOCATION + 32.0)) && (start_block > TARGET_LOCATION))
 	){// for odometry calibration: (DBlock_odo < 3.0*16.0){
		// Might have to adjust the middle dark value

		error = SensorValue[lightSensor] - 35;

		motorPower = Kp * error + Kd * (error - lastError);

		lastError = error;

		rightMotorSpeed = RIGHT_BASE_SPEED + motorPower;
		leftMotorSpeed  = LEFT_BASE_SPEED  - motorPower;
		// Ensure Bounds aren't Exceeded. Scale Both Motor Speeds if one is capped.
	  if (rightMotorSpeed > RIGHT_MAX_SPEED ) { leftMotorSpeed = leftMotorSpeed * RIGHT_MAX_SPEED / rightMotorSpeed; rightMotorSpeed = RIGHT_MAX_SPEED; } // prevent the motor from going beyond max speed
	  if (leftMotorSpeed > LEFT_MAX_SPEED ) { rightMotorSpeed = rightMotorSpeed * LEFT_MAX_SPEED / leftMotorSpeed; leftMotorSpeed = LEFT_MAX_SPEED; } // prevent the motor from going beyond max speed
	  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
	  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

		search__i_a();
		wait1Msec(2);

		motor[RightMotor] = rightMotorSpeed;
		motor[LeftMotor] = leftMotorSpeed;

		if(orientation_done){
			update_block_location();
			start_block = block_location - floor(DBlock_odo);
			if(start_block < 0){
				start_block = MAP_SIZE + start_block;
			}
		} else{
			DBlock_odo = BLOCKS_PER_METER * TSF_Last(Hist_Dist); // Update Odo Estimate
			if(!orientaion_started && (SensorValue[sonarSensor] > SONAR_THRESH) && DBlock_odo > 0.6){ // Trips as soon as Block Edge is Reached (after has settled in line-following D>0.6)
				DB_at_orientation_start = DBlock_odo;
				orientaion_started = 1;
			} // !orientaion_started?
			if(orientaion_started && ((DBlock_odo - DB_at_orientation_start) > 0.35)){ // Ends orientation in middle of Block
				TSF_add(Hist_Dist, 0.0); // Reset Arc Distance Odometry
				TSF_add(Hist_Dist, 0.0); // (and its delta)
				orientation_done = 1;
			} // orientaion_started?
		} // orientation_done?

		update_display();
	} // Line Following Loop
	motor[RightMotor] = 0;
	motor[LeftMotor] = 0;

	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
