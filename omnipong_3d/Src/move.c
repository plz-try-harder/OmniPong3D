#include "move.h"
#include "main.h"
#include "stm32f4xx_hal.h"




void rot_direction(int motor, int direction)  //o means counterclockwise, 1 clockwise // 2, locked
{   
    int direction2 = 0;

    if(direction == 0) {
        direction2 = 1;
    }
    else if(direction == 1) {
        direction2 = 0;
    }
    else if(direction == 2) {
        direction = 0;
        direction2 = 0;
    }

    if(motor == 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, direction2);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, direction);
    }
    else if(motor == 1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, direction2);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, direction);
    }
    else if (motor == 2)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, direction2);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, direction);
    }
    
}



void rotate(int rotation_direction, int degrees) //rotation_direction 0  clockwise, 1 counter-clockwise
{
    rot_direction(0, rotation_direction);
    rot_direction(1, rotation_direction);
    rot_direction(2, rotation_direction);
}




void vehicle_direction(int direction) //direction is side of the riangle 0/1/2
{
    switch(direction)
    {
        case 0: rot_direction(2, 0); rot_direction(0, 1); rot_direction(1, 2); break;
        case 1: rot_direction(0, 0); rot_direction(1, 1); rot_direction(2, 2); break;
        case 2: rot_direction(1, 0); rot_direction(2, 1); rot_direction(0, 2); break;
    }
}

void stop_all_motors()
{
    rot_direction(0, 2);
    rot_direction(1, 2);
    rot_direction(2, 2);
}



void adjust_speed(int motor, int speed) //speed is a percentage
{
    switch(motor)
    {
        case 0: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed); break;
        case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed); break;
        case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed); break;
    }
    
}