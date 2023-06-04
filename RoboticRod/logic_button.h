#include <math.h>
#include <stdlib.h>
#include <Arduino.h>

/* mode symbolisé par un booléen. True -> mode 1, False -> Mode 2 */
extern float position_a_atteindre[2];
extern float position_actuelle[2];

extern bool mode; 
extern int last_state_button_1;
extern int last_state_button_6;
extern int pinButton[6];
extern bool wristOrientation;
extern float speedRatio;
extern bool inverseKinematicNeedeed;
extern bool shutdown_arm;

extern unsigned long pressedDownTime;
extern unsigned long elapsedTime;

void recuperation_commande_utilisateur();
