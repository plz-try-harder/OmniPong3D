#ifndef __SENSORS_H
#define __SENSORS_H

//#include "stm32f4xx_hal_tim.h"

#include "main.h"


extern int status[3];
extern int us_rising_edge[3];
extern int us_falling_edge[3];
extern int us_distance[3];

extern int pulse_counter[3];
extern uint8_t buf[128];




extern int current_channel;


extern void ultrasonic(int sensor);
extern void light_sensor(int motor);

extern void debug_distance();
extern void debug_counter();

#endif