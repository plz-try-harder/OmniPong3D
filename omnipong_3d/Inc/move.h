#ifndef __MOVE_H
#define __MOVE_H

#include "main.h"


void rot_direction(int motor, int direction);

void rotate(int rotation_direction, int degrees);

void adjust_speed(int motor, int speed);

void vehicle_direction(int direction);

void stop_all_motors();


#endif