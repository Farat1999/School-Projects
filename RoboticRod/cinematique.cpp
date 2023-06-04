#include "cinematique.h"
#include "logic_button.h"

const float min_angle_1 = 104;
const float min_angle_2 = 91;
const float min_angle_3 = 66;
const float min_angle_4 = 63;
const float min_angle_5 = 118;
const float min_angle_6 = 70;

const float max_angle_1 = 180;
const float max_angle_2 = 272;
const float max_angle_3 = 289;
const float max_angle_4 = 292;
const float max_angle_5 = 208;
const float max_angle_6 = 135;

const float max_speed_1     =     230;
const float max_speed_2     =     230;
const float max_speed_3     =     max_speed_2;
const float max_speed_4     =     230;
const float max_speed_5     =     230;
const float max_speed_6     =     230;
const float max_speed_7     =     230;

const float max_toque_1     =     2047;
const float max_toque_2     =     2047;
const float max_toque_3     =     max_toque_2;
const float max_toque_4     =     2047;
const float max_toque_5     =     1193;
const float max_toque_6     =     1193;
const float max_toque_7     =     50;

const float g = 9.81;
const float pi = 3.1415;

const float l1   = 0.14025;
const float l2   = 0.29975;
const float l3   = 0.29975;
const float l4   = 0.115;

const float seuil_satur = 12; // en Volt

const float qLowLim[6] =  {min_angle_1 *4096/360, min_angle_2*4096/360, min_angle_3*4096/360, min_angle_4*4096/360, min_angle_5*4096/360, min_angle_6*4096/360};
const float qHighLim[6] = {max_angle_1/360*4096, max_angle_2/360*4096, max_angle_3/360*4096, max_angle_4/360*4096, max_angle_5/360*4096, max_angle_6/360*4096};

int qInit[6]= {1375, 2047, 1025, 2167, qHighLim[4]-10, 1500};
int qOff[6]= {1375, 973, 1047, 2097, qHighLim[4]-10, 1500};
int qRead[6];
int qWrite[6] = {1375, 2047, 1025, 2167, qHighLim[4]-10, 1500};

float littleSpace = 0.02;
float lmin = 2*l2*cos(((2048-qLowLim[2])/2)*2*pi/4096);


int check_pos(){
  int error = 0;
  float norm = sqrt(pow(position_a_atteindre[0],2) + pow(position_a_atteindre[1],2));

  if(norm > l2+l3+littleSpace){
    error = 1;
    return error;
  }
  if(norm < lmin){
    error = 2;
    return error;
  }
  return error;
}

void modify_goal_pos(int error){
  float norm = sqrt(pow(position_a_atteindre[0],2) + pow(position_a_atteindre[1],2));
  Serial.print("Erreur : ");
  Serial.println(error);
  if(error == 1){
    float facteur = norm / (l2+l3+littleSpace);
    position_a_atteindre[0] = position_a_atteindre[0]/facteur;
    position_a_atteindre[1] = position_a_atteindre[1]/facteur;
  }
  else if(error == 2){
    float facteur = norm / lmin;
    position_a_atteindre[0] = position_a_atteindre[0]/facteur;
    position_a_atteindre[1] = position_a_atteindre[1]/facteur;
  }
}

void check_range(){
  for (int i=0; i<6; i++){
    if(qWrite[i]<qLowLim[i]){
      qWrite[i] = qLowLim[i];
    }
    if(qWrite[i]>qHighLim[i]){
      qWrite[i] = qHighLim[i];
    }
  }
}
   


void cinematique_directe(){
  float q1 = -((qRead[1]- 2048)*2*pi/4096);
  float q2 = -((qRead[2]- 2048)*2*pi/4096);
  float r = l2*sin(q1) + l3*sin(q1+q2);
  float z = l2*cos(q1) + l3*cos(q1+q2);
  position_actuelle[0] = r;
  position_actuelle[1] = z;
  }


void cinematique_inverse(){
  /* If too far or too close, we put back in the "donut" */
  int error = check_pos();
  if(error!=0){
    modify_goal_pos(error);
    Serial.print("Nouvelle cible : "); 
    Serial.print(position_a_atteindre[0],4);
    Serial.print("\t");    // prints a tab
    Serial.println(position_a_atteindre[1],4);
  }

  /* We calculate the theoretical solution*/
  float K = (pow(position_a_atteindre[1],2) + pow(position_a_atteindre[0],2) - pow(l2,2) - pow(l3,2))/(2*l2*l3);
  if(K>1){return;}

  float theta2[2] = {atan2(sqrt(1-pow(K,2)),K), atan2(-sqrt(1-pow(K,2)),K)};
  float M = position_a_atteindre[1];
  float L = position_a_atteindre[0];
  float G[2] = {sin(theta2[0])*l3, sin(theta2[1])*l3};
  float H = K*l3+l2;
  float theta1[2] = {atan2(M,L)-atan2(G[0],H), atan2(M,L)-atan2(G[1],H)};
  float theta3[2] = {-(theta1[0] + theta2[0]), -(theta1[1] + theta2[1])};

  /* On prend la configuration coude en haut */
  int indexMax = 1;
  if(theta1[0]>=theta1[1]){int indexMax = 0;}

  /* On change la cible des moteurs */
  qWrite[1] = theta1[indexMax]*4096/(2*pi) + 1024;
  qWrite[2] = theta2[indexMax]*4096/(2*pi) + 2048; 
  qWrite[3] = theta3[indexMax]*4096/(2*pi) + 2048; 

  /* On s'assure que les moteurs soient dans les limites*/
  if(qWrite[1]<qLowLim[1] || qWrite[1]>qHighLim[1]){
    if(qWrite[1]<qLowLim[1]){qWrite[1]=qLowLim[1];}
    if(qWrite[1]>qHighLim[1]){qWrite[1]=qHighLim[1];}
    
    float q1 = -(qWrite[1]-2048)*2*pi/4096;
    float facteur = sqrt(pow((position_a_atteindre[0]-l2*sin(q1)),2) + pow(position_a_atteindre[1],2))/l3;
  
    position_a_atteindre[0] = l2*sin(q1)+(position_a_atteindre[0]-l2*sin(q1))/facteur;
    position_a_atteindre[1] = position_a_atteindre[1]/facteur;
  
    /* Nouvelle cinématique inverse */
    float K = (pow(position_a_atteindre[1],2) + pow(position_a_atteindre[0],2) - pow(l2,2) - pow(l3,2))/(2*l2*l3);
    //if(K>1){return}
  
    float theta2[2] = {atan2(sqrt(1-pow(K,2)),K), atan2(-sqrt(1-pow(K,2)),K)};
    float M = position_a_atteindre[1];
    float L = position_a_atteindre[0];
    float G[2] = {sin(theta2[0])*l3, sin(theta2[1])*l3};
    float H = K*l3+l2;
    float theta1[2] = {atan2(M,L)-atan2(G[0],H), atan2(M,L)-atan2(G[1],H)};
    float theta3[2] = {-(theta1[0] + theta2[0]), -(theta1[1] + theta2[1])};

    if(theta1[0]>=theta1[1]){int indexMax = 0;}
    else{int indexMax = 1;}
  
    qWrite[1] = theta1[indexMax]*4096/(2*pi) + 1024;
    qWrite[2] = theta2[indexMax]*4096/(2*pi) + 2048; 
    qWrite[3] = theta3[indexMax]*4096/(2*pi) + 2048; 
  }
  if(qWrite[3]<qLowLim[3]){
    int variation = qLowLim[3]-qWrite[3];
    qWrite[3]= qLowLim[3];
    qWrite[2] = qWrite[2] - variation;
    }
  if(qWrite[3]>qHighLim[3]){
    int variation = qHighLim[3]-qWrite[3];
    qWrite[3]= qHighLim[3];
    qWrite[2] = qWrite[2] - variation;
    }

  /* Sécurité pour que ce soit TOUJOURS dans les limites */
  check_range();
}
