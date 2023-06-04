#include "commande_moteurs.h"
#include "cinematique.h"
#include "logic_button.h"


// This code returns a number of 6 digits (each digit either 0 or 1) depending on which buttons are pressed or not
bool isActiveArm = false; /*Arm on ?*/


void setup() {
  // put your setup code here, to run once:
  
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(12, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  init_motors(); 
}

void loop() {
  /* If the arm is on */
  if(isActiveArm==true){
    /* Do we want to turn it off? */
    if(digitalRead(pinButton[1])==LOW && digitalRead(pinButton[4])==LOW){
      Serial.print("Switching off \n");
      isActiveArm=false;
      delay(1000);
      Serial.print("Switched off \n");
    }

    /* The command is retrieved from the buttons */
    recuperation_commande_utilisateur();
    Serial.print(qWrite[0]);
    Serial.print("\t");    // prints a tab
    Serial.print(qWrite[1]);
    Serial.print("\t");    // prints a tab
    Serial.print(qWrite[2]);
    Serial.print("\t");    // prints a tab
    Serial.print(qWrite[3]);
    Serial.print("\t");    // prints a tab
    Serial.print(qWrite[4]);
    Serial.print("\t");    // prints a tab
    Serial.println(qWrite[5]);
    check_range();
    delay(50);
    send_command();
//    get_feedback();
    
  }
  /* If it is off, we check that we are not turning it on.*/
  else{
    if(digitalRead(pinButton[1])==LOW && digitalRead(pinButton[4])==LOW){
      /*Tensioning of the arm*/
      Serial.print("Switching on \n");
      isActiveArm=true;
      delay(1000);
      go_to_home_pos();
      Serial.print("Ready \n");
//      switch_on_arm();
    }
  }
}
