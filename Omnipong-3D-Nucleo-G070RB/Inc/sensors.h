#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdint.h>

extern uint8_t buf[64];

//variables for ultrasonic sensor
extern int current_channel;
extern uint8_t status[3];
extern int us_rising_edge[3];
extern int us_falling_edge[3];
extern int distance[3];

//Variables for light_barrier
extern int pulse_counter[3];
extern int old_value[3];

extern int wait_sec_status;

extern int lowest_risk;
extern int signal_received;

extern int side_risk;



/**
 * @brief   calculates the distance to the next object from the pulselength,
 *          supplied by one of the three ultrasonic sensors
 * 
 * @param[in] sensor is the us-sensor in travel direction 
 */
void ultrasonic(int sensor, int channel);



/**
 * @brief   is used to increment the pulse counter for each motor, every time a
 *          rising edge supplied by a light barrier is detected
 * 
 * @param motor is the motor which belongs to the light barrier
 */
void light_barrier();



/**
 * @brief   chooses which us-sensor should be triggered int the next measuring cycle
 * 
 */
void switch_us_trigger();


/**
 * @brief   checks if distance to the next object in travel direction is big enough; stops 
 *          vehicle, if distance is to short to prevent collision
 */
void distance_check(int side);


void debug_distance();

void debug_counter();

void button();


/**
 * @brief returns a random integer between 0 and 1
 * 
 * @return int the random value
 */
int random_2();

/**
 * @brief returns a random integer between 0 and 7
 * 
 * @return int 
 */
int random_8();

#endif