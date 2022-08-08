#include "main.h"
#include "sensors.h"
#include "stdio.h"
#include "string.h"
#include "move.h"
#include "shape.h"
#include "ws2812.h"


uint8_t buf[64];

//variables for ultrasonic sensor
uint8_t status[3] = {0};
int us_rising_edge[3]  = {0};
int us_falling_edge[3] = {0};
int distance[3];
int old_distance[3] = {0};


//Variables for light_barrier
int pulse_counter[3] = {0};


//variables for button
int debounce_button = 0;
int wait_sec_status = 0;

int lowest_risk;
int signal_received = 0;

int side_risk;


void ultrasonic(int sensor, int channel) {   

  switch(status[sensor])
  {
    case 0:
      us_rising_edge[sensor] = __HAL_TIM_GET_COMPARE(&htim1, channel);
      status[sensor] = 1;
      break;

    case 1:
      us_falling_edge[sensor] = __HAL_TIM_GET_COMPARE(&htim1, channel);
      
      if(us_rising_edge[sensor] < us_falling_edge[sensor])
      { 
        distance[sensor] = (us_falling_edge[sensor] - us_rising_edge[sensor]) / 58;
        old_distance[sensor] = distance[sensor];
        
        signal_received++;
        if(signal_received == 3)
        {
          lowest_risk = sensor;
        }
      }
      else
      {
        distance[sensor] = old_distance[sensor];
      }

      status[sensor] = 0;
      break;
  }
}

void debug_distance() {

  sprintf((char*)buf, "dis 0: %d cm\n\rdis 1: %d cm\n\rdis 2: %d cm\n\rLowest Risk: %d\n\n\n\r", distance[0], distance[1], distance[2], lowest_risk);
  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 500);

}

void debug_counter() {
  sprintf((char*)buf, "%d \n\r", trav_dir);
  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 500);
}


void button()
{
    mode_step++;
    mode_step %= 4;

    stop_motors();

    switch(mode_step)
    {
      case 0: modus = 0;  set_led(0, 0, 64, 0); break;
      case 1: modus = 1;  square_step = 0; set_led(0, 0, 0, 64); break;
      case 2: modus = 0;  set_led(0, 0, 64, 0); break;
      case 3: modus = 2;  set_led(0, 64, 0, 0); travel_direction(0); wait_status=1; wait_counter=10; break;
    }

    ws2812_send();
  }



void distance_check(int side)
{
    if(modus < 2) //wird im Figurenmodus abgebrochen, weil zu wenig Platz
    {
        return;
    }

    if(trav_dir < 4) // nicht während Drehung, oder wenn es schon steht. 
    {
        if(distance[side] < 40)
        {
          stop_motors();
          trav_dir = 4;
        }
      }
}



int random_2() //returns a random value between 0 and 1
{
    int random2 = __HAL_TIM_GET_COUNTER(&htim1);
    random2 &= 1;
    return random2;
}

int random_8()  //returns a random value between 0 and 8
{
    int random8 = __HAL_TIM_GET_COUNTER(&htim1);
    random8 &= 0b111;
    return random8;
}