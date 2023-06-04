#include <DynamixelShield.h>
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>



void boucle_moteur();
void init_motors();
void send_command();
void get_feedback();
void send_smooth_command();
void go_to_home_pos();
