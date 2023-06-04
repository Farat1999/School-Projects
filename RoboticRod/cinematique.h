#include <math.h>
#include <stdlib.h>
#include <Arduino.h>

extern const float min_angle_1;
extern const float min_angle_2;
extern const float min_angle_3;
extern const float min_angle_4;
extern const float min_angle_5;
extern const float min_angle_6;

extern const float max_angle_1;
extern const float max_angle_2;
extern const float max_angle_3;
extern const float max_angle_4;
extern const float max_angle_5;
extern const float max_angle_6;

extern const float max_speed_1;
extern const float max_speed_2;
extern const float max_speed_3;
extern const float max_speed_4;
extern const float max_speed_5;
extern const float max_speed_6;
extern const float max_speed_7;

extern const float max_toque_1;
extern const float max_toque_2;
extern const float max_toque_3;
extern const float max_toque_4;
extern const float max_toque_5;
extern const float max_toque_6;
extern const float max_toque_7;

extern const float g;
extern const float pi;

extern const float l1;
extern const float l2;
extern const float l3;
extern const float l4;

extern const float seuil_satur; // en Volt

extern const float qLowLim[6];
extern const float qHighLim[6];

extern int qInit[6];
extern int qOff[6];
extern int qRead[6];
extern int qWrite[6];

extern float littleSpace;
extern float lmin;

int check_pos();
void modify_goal_pos(int error);
void check_range();
void cinematique_directe();
void cinematique_inverse();


/*
#Paramètres liés aux position du bras dans l'espace 
pos = np.array([0.3, 0.3] )                             # Position dans l'espace de q3
deltaPos = [0., 0.]                                     # Variation de la position dans l'espace
lmin = 2*l2*np.cos(((2048-qLowLim[2])/2)*2*np.pi/4096)  # Distance minimale entre q1 et q3 
minRquadrant2 = -0.3                                    # Postion minimale selon l'axe r dans le 2e quadrant
maxZquadrant4 = -np.sqrt(l3**2 - (l3-littleSpace)**2)   # Position maximale en z dans la limite circulaire du quadrant 4
*/
