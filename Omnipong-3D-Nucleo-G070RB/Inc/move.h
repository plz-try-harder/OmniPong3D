#ifndef __MOVE_H
#define __MOVE_H

extern int trav_dir;
extern int trav_speed;

extern int current_compare[3];

extern int wait_counter;
extern int wait_status;

extern int shape_step;
extern int shape_status;

extern int timer;




void wheel_direction(int motor, int dir);

void travel_direction(int dir);

void stop_motors();

void rotate(int dir, int angle);

void set_pwm(int motor, int compare);

void adjust_pwm();

void set_trav_speed(int speed);

void check_wait_status();

int random_2();

int random_8();



void shape_mode();



#endif