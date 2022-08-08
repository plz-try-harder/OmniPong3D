#include "move.h"
#include "sensors.h"

int square_step = 0;


void square()
{
    switch(square_step)
    {
        case 0: travel_direction(0); wait_counter = 30; wait_status = 1; shape_status = 1; break;
        case 1: shape_status = 1; rotate(0,6); break;
        case 2: travel_direction(1); wait_counter = 30; wait_status = 1; shape_status = 1; break;
        case 3: shape_status = 1; rotate(0,6); break;
        case 4: travel_direction(2); wait_counter = 30; wait_status = 1; shape_status = 1; break;
        case 5: shape_status = 1; rotate(0,6); break;
        case 6: travel_direction(0); wait_counter = 30; wait_status = 1; shape_status = 1; break;
        case 7: shape_status = 1; rotate(0,6); break;    
    }
    square_step++;
}