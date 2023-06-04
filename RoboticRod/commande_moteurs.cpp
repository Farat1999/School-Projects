#include "commande_moteurs.h"
#include "logic_button.h"
#include "cinematique.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #define DEBUG_SERIAL Serial
  /*#include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial*/
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB  
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID_CNT = 6;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t SR_START_ADDR = 132;
const uint16_t SR_ADDR_LEN = 4; //2+4+4
const uint16_t SW_START_ADDR = 116; //Goal velocity
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  //int16_t present_current;
  //int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

/*sw_data_t sw_data_2[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos_2;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_2[DXL_ID_CNT];
*/
DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;



void boucle_moteur(){
  uint8_t i;
  //if(digitalRead(3)==LOW || digitalRead(4)==LOW || digitalRead(5)==LOW || digitalRead(6)==LOW || digitalRead(11)==LOW || digitalRead(12)==LOW ){
  if(digitalRead(3)==LOW || digitalRead(4)==LOW || digitalRead(5)==LOW || digitalRead(6)==LOW || digitalRead(9)==LOW || digitalRead(10)==LOW ){
    // put your main code here, to run repeatedly:
    static uint32_t try_count = 0;
    uint8_t i, recv_cnt;
    
    for(i=0; i<DXL_ID_CNT; i++){
      sw_data[i].goal_position+=500;
      if(sw_data[i].goal_position >= 2000){
        sw_data[i].goal_position = 0;
      }
    }
    sw_infos.is_info_changed = true;
  
    DEBUG_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
    DEBUG_SERIAL.println(try_count++);
    if(dxl.syncWrite(&sw_infos) == true){
      DEBUG_SERIAL.println("[SyncWrite] Success");
      for(i=0; i<sw_infos.xel_count; i++){
        DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.println(sw_infos.p_xels[i].id);
        DEBUG_SERIAL.print("\t Goal Velocity: ");DEBUG_SERIAL.println(sw_data[i].goal_position);
      }
    }else{
      DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
      DEBUG_SERIAL.print(dxl.getLastLibErrCode());
    }
    DEBUG_SERIAL.println();
  
    delay(250);
  
    recv_cnt = dxl.syncRead(&sr_infos);
    if(recv_cnt > 0){
      DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
      DEBUG_SERIAL.println(recv_cnt);
      for(i=0; i<recv_cnt; i++){
        DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
        DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(sr_infos.p_xels[i].error);
        //DEBUG_SERIAL.print("\t Present Current: ");DEBUG_SERIAL.println(sr_data[i].present_current);
        //DEBUG_SERIAL.print("\t Present Velocity: ");DEBUG_SERIAL.println(sr_data[i].present_velocity);
        DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(sr_data[i].present_position);
      }
    }else{
      DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
    }
    DEBUG_SERIAL.println("=======================================================");
  
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(750);
    
    
  }
}

void init_motors(){
  uint8_t i;
  DEBUG_SERIAL.begin(57600);
  Serial.begin(57600);
  dxl.begin(57600);

  for(i=0; i<DXL_ID_CNT; i++){
    dxl.ledOn(DXL_ID_LIST[i]);
    Serial.print("Motor led on : ");Serial.println(DXL_ID_LIST[i]);
    dxl.ledOff(DXL_ID_LIST[i]);
    delay(50);
  }
  
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
    dxl.torqueOn(DXL_ID_LIST[i]);
    Serial.print("Motor on : ");Serial.println(DXL_ID_LIST[i]);
    delay(50);
  }

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;


  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  // Fill the members of structure to syncWrite using internal packet buffer
/*  sw_infos_2.packet.p_buf = nullptr;
  sw_infos_2.packet.is_completed = false;
  sw_infos_2.addr = SW_START_ADDR;
  sw_infos_2.addr_length = SW_ADDR_LEN;
  sw_infos_2.p_xels = info_xels_sw_2;
  sw_infos_2.xel_count = 0;*/

  sw_data[0].goal_position = 500;
  sw_data[1].goal_position = 1500;
  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}


void send_command(){
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;

  //get_feedback();
  
  for(i=0; i<DXL_ID_CNT; i++){
    sw_data[i].goal_position=qWrite[i];
    Serial.print("command");Serial.print(i);
  }
  sw_infos.is_info_changed = true;
  dxl.syncWrite(&sw_infos);
}

void get_feedback(){
  uint8_t recv_cnt;
  recv_cnt = 0;

  while(recv_cnt==0){
    recv_cnt = dxl.syncRead(&sr_infos);
    if(recv_cnt > 0){
      for(int i=0; i<recv_cnt; i++){
        Serial.print("  ID: ");Serial.print(sr_infos.p_xels[i].id);
        Serial.print("\t Present Position: ");Serial.println(sr_data[i].present_position);
  
        qRead[i] = sr_data[i].present_position;
      }
    }
  }
}

void send_smooth_command(){
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;
  
  get_feedback();
  
  // int distance_max = max(abs(qWrite[0]-qRead[0]),max(abs(qWrite[1]-qRead[1]),max(abs(qWrite[2]-qRead[2]),max(abs(qWrite[3]-qRead[3]),max(abs(qWrite[4]-qRead[4]),abs(qWrite[5]-qRead[5]))))));
  int distance_max = max(abs(qWrite[0]-qRead[0]),abs(qWrite[1]-qRead[1]));
  int nb_points = (int) distance_max/10;
  Serial.print("  Je lisse avec: ");Serial.print(nb_points);
  for(int j=0; j<nb_points; j++){
    for(i=0; i<DXL_ID_CNT; i++){
      sw_data[i].goal_position=qRead[i]+(qWrite[i]-qRead[i])/nb_points*j;
    }
    sw_infos.is_info_changed = true;
    dxl.syncWrite(&sw_infos);
    delay(40);
  }
}

void go_to_home_pos(){
  for(int i=0; i<6; i++){
    qWrite[i] = qInit[i];
    Serial.print(i);
  }
  send_smooth_command();
}
