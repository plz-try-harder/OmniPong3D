#include "move.h"
#include "main.h"
#include "sensors.h"
#include "shape.h"

int trav_dir = 0;
int trav_speed;

int wait_counter = 0;
int wait_status = 0;

int current_compare[3];

int shape_step = 0;
int shape_status = 0;



void wheel_direction(int motor, int dir)  //0 ccw, 1 cw
{  
    int dir1 = 0;

    switch(dir)
    {
        case 0: dir1 = 1; break;
        case 1: dir1 = 0; break;
        case 2: dir =  0; break;
    }

    switch(motor)
    {
        case 0: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, dir1); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, dir); break;
        case 1: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, dir1); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, dir); break;
        case 2: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, dir1); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, dir); break;
    }    
}


void set_trav_speed(int speed)
{
    trav_speed = speed;

    if(speed>19)
    {
        for(int i=0; i<3; i++) {set_pwm(i, 200);}
    }
    else
    {
        for(int i=0; i<3; i++) {set_pwm(i, 60);}
    }
}



void travel_direction(int dir)
{

    trav_dir = dir;
    switch(modus)
    {
        case 1:  set_trav_speed(25); break;
        default: set_trav_speed(30); break;
    }
    switch(dir)
    {
        case 0: wheel_direction(0, 0); wheel_direction(2, 1); wheel_direction(1, 2); trav_dir = 0; break;
        case 1: wheel_direction(0, 1); wheel_direction(1, 0); wheel_direction(2, 2); trav_dir = 1; break;
        case 2: wheel_direction(1, 1); wheel_direction(2, 0); wheel_direction(0, 2); trav_dir = 2; break;
        case 3: stop_motors(); break;
    }
}


void stop_motors()
{
    wheel_direction(0, 2);
    wheel_direction(1, 2);
    wheel_direction(2, 2);
}


void rotate(int dir, int angle) //0 ccw, 1 cw
{
    wheel_direction(0, dir);
    wheel_direction(1, dir);
    wheel_direction(2, dir);

    switch(modus)
    {
        case 1:  set_trav_speed(11); break;
        default: set_trav_speed(11); break;
    }

    wait_status = 1;
    wait_counter = angle;
}



void set_pwm(int motor, int compare)
{
    switch(motor)
    {
        case 0: __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, compare); current_compare[0] = compare; break;
        case 1: __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, compare); current_compare[1] = compare; break;
        case 2: __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, compare); current_compare[2] = compare; break;
    }
}



void adjust_pwm()
{
    int correct = 10;
    if(trav_speed < 15)
    {
        correct =  2;
    }      

    for(int i=0; i<=2; i++)
    {
        if(pulse_counter[i] < trav_speed)
        {
            if(current_compare[i] < 980) 
            {
            set_pwm(i, (current_compare[i] + correct));
            }
        }
        else if(pulse_counter[i] > trav_speed)
        {   
            if(current_compare[i] > 20)
            {
            set_pwm(i, (current_compare[i] - correct));
            }
        }
        else if(pulse_counter[i] == trav_speed)
        {
            set_pwm(i, current_compare[i]);
        }        
        pulse_counter[i] = 0;
    }
}


void check_wait_status()
{       
    if(trav_dir == 4)
    {   
        if(modus == 2)
        {
            rotate(random_2(), random_8());
            trav_dir = 5;
            return;
        }
    }    
    
    if(wait_status)
    {  
        if(wait_counter == 0)
        {   
            stop_motors();
            wait_status = 0;
            shape_status = 0;


            if(square_step == 8)
            {
                button();
                square_step = 0; 
                shape_status = 0; 
            }
        }
        else
        {
            wait_counter--;
        }
    }
    else if(trav_dir == 5)
    {
        travel_direction(lowest_risk);
    }
}

