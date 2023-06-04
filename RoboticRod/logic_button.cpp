#include "logic_button.h"
#include "cinematique.h"
#include "commande_moteurs.h"


/* mode symbolized by a boolean. True -> mode 1, False -> Mode 2 */
float position_a_atteindre[2] = {0.3, 0.3};
float position_actuelle[2] = {0.3, 0.3};

bool mode = true; 
int last_state_button_1 = 1;
int last_state_button_6 = 1;
// int pinButton[6] = {3,4,5,6,11,12};
int pinButton[6] = {3,4,5,6,9,10};
bool wristOrientation = true;
float speedRatio = 1;
bool inverseKinematicNeedeed = false;
bool shutdown_arm = false;

unsigned long pressedDownTime = 0;
unsigned long elapsedTime = 0;

void recuperation_commande_utilisateur(){
  // get_feedback();
//  for(int i=0; i<6; i++){
//    qWrite[i] = qRead[i];
//  }
//  cinematique_directe();
//  position_a_atteindre[0] = position_actuelle[0];
//  position_a_atteindre[1] = position_actuelle[1];
  
  /* Reading of button 1 + verification of a first press */
  if(digitalRead(pinButton[0])==LOW && last_state_button_1 == 1){
    mode = !mode; /* we change mode */
  }
  last_state_button_1 = digitalRead(pinButton[0]);

  /* Button 2 management (gripper closing / base+ rotation) */
  if (digitalRead(pinButton[1])==LOW && digitalRead(pinButton[2])==HIGH){
    /* Mode 1 */
    if(mode==true){
      float presentCurrent = 0.;// check_current(); TO BE CHANGED!!!!
      if(presentCurrent < 350){
        qWrite[5] = qWrite[5] + 30*speedRatio;
        Serial.println(qWrite[5]); 
      }
    }
    /* Mode 2 */
    else{
      qWrite[0] = qWrite[0] + 30*speedRatio;
    }
  }

  /* Button 3 management (gripper opening / base rotation) */
  if (digitalRead(pinButton[2])==LOW && digitalRead(pinButton[1])==HIGH){
    /* Mode 1 */
    if(mode==true){
        qWrite[5] = qWrite[5] - 30*speedRatio; 
    }
    /* Mode 2 */
    else{
      qWrite[0] = qWrite[0] - 30*speedRatio;
    }
  }

  /* Management button 4 (+r or +z) */
  if (digitalRead(pinButton[3])==LOW && digitalRead(pinButton[4])==HIGH){
    /* Mode 1 */
    if(mode==true){
      position_a_atteindre[0] = position_a_atteindre[0]+0.005*speedRatio;
      inverseKinematicNeedeed = true;
    }
    /* Mode 2 */
    else{
      position_a_atteindre[1] = position_a_atteindre[1]+0.005*speedRatio;
      inverseKinematicNeedeed = true;
    }
  }
  
  /* Management button 5 (-r or -z)*/
  if (digitalRead(pinButton[4])==LOW && digitalRead(pinButton[3])==HIGH){
    /* Mode 1 */
    if(mode==true){
      position_a_atteindre[0] = position_a_atteindre[0]-0.005*speedRatio;
      inverseKinematicNeedeed = true;
    }
    /* Mode 2 */
    else{
      position_a_atteindre[1] = position_a_atteindre[1]-0.005*speedRatio;
      inverseKinematicNeedeed = true;
    }
  }

  if(digitalRead(pinButton[5])==LOW && last_state_button_6 == 1){
    last_state_button_6 = 0;
    pressedDownTime = millis();
  }
  
  /* Management of button 6 (gear change or turn wrist) */
  if(digitalRead(pinButton[5])==HIGH && last_state_button_6 == 0){
    last_state_button_6 = 1;
    elapsedTime = millis() - pressedDownTime;
    /* If long press */
    if(elapsedTime>2000){
      if(speedRatio==1){speedRatio=0.5;}
      else if(speedRatio==0.5){speedRatio=1;}
    }
    /* If short press */
    else{
      wristOrientation = !wristOrientation;
      if(wristOrientation==true){
        qWrite[4] = qLowLim[4] + 10;
      }
      else{
        qWrite[4] = qHighLim[4] - 10;
      }
    }
  }

  /* Management button 5 (-r or -z)*/
//  if (digitalRead(pinButton[2])==LOW && digitalRead(pinButton[3])==LOW){
//    shutdown_arm=true;
//  }

  if(inverseKinematicNeedeed==true){
    cinematique_inverse();
    inverseKinematicNeedeed=false;
  }
}
