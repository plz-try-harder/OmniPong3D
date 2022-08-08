#include "sensors.h"
#include <string.h>
#include <stdio.h>


int status[3];
int us_rising_edge[3];
int us_falling_edge[3];
int us_distance[3];
uint8_t buf[128];

int pulse_counter[3] = {0,0,0};


int current_channel;


void ultrasonic(int sensor) {

    switch(sensor) 
    {
        case 0: current_channel = TIM_CHANNEL_2; break;
        case 1: current_channel = TIM_CHANNEL_3; break;
        case 2: current_channel = TIM_CHANNEL_4; break;
    }


    if (status[sensor] == 0) 
    {
        us_rising_edge[sensor] = __HAL_TIM_GET_COMPARE(&htim1, current_channel);
        status[sensor] = 1;
    }
    else if(status[sensor] == 1)
    {
        us_falling_edge[sensor] = __HAL_TIM_GET_COMPARE(&htim1, current_channel);
        if(us_rising_edge[sensor] < us_falling_edge[sensor])
        {
        us_distance[sensor] = (us_falling_edge[sensor] - us_rising_edge[sensor]) / 58;
        } else {

        us_distance[sensor] = (65535 - us_rising_edge[sensor] - us_falling_edge[sensor]) / 58;
      
        }

    us_rising_edge[sensor] = 0;
    us_falling_edge[sensor] = 0;
    status[sensor] = 0;

    }
}


void light_sensor(int motor) 
{
    pulse_counter[motor]++;
}


void debug_counter() {
  sprintf((char*)buf, "%d %d %d\n\r", pulse_counter[0], pulse_counter[1], pulse_counter[2]);
  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 500);
}


void debug_distance() {

  sprintf((char*)buf, "Sensor 1: %d cm\n\rSensory 2: %d cm\n\rSensor 3: %d cm\n\n\n\r", us_distance[0], us_distance[1], us_distance[2]);
  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 500);

}