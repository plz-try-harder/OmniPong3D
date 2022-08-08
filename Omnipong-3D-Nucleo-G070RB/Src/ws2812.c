#include "ws2812.h"
#include "main.h"

#define LEDAMOUNT 6

uint8_t led_data[LEDAMOUNT][4];
uint16_t pwm_data[(24*LEDAMOUNT) + 50];

void set_led(int num, int red, int green, int blue)
{
    led_data[num][0] = num;
    led_data[num][1] = green;
    led_data[num][2] = red;
    led_data[num][3] = blue;
}

void ws2812_send()
{
    uint32_t color;
    uint32_t index = 0;

    for(int i=0; i<LEDAMOUNT; i++)
    {
        color = ((led_data[i][1] << 16 | (led_data[i][2] << 8) | (led_data[i][3])));

        for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwm_data[index] = 53;
			}

			else pwm_data[index] = 27;

			index++;
		}
    }

    for (int i=0; i<50; i++)
	{
		pwm_data[index] = 0;
		index++;
	}
    HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t *) pwm_data, index);
}

