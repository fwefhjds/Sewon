//버튼에 릴레이동작 추가된 버전
//EOCR과 전력량계 통신 하나로 통합, 디버깅 꼭 필요, 210720
//Relay 확인(에러)코드 추가,door 에러코드 추가 210721
//EV_Error loop에 329line, current demand에만 일단 추가함 210723
//fault 초기화 방지.. loop(line 1070) read reponse() 뒤에 에러 없음으로 초기화 -> 맨 위부터 에러를 체크해서 에러가 있다면 초기화 x 에러가 없다면 초기화 o
//0726 fault 초기화 방지는 total_Fault 검색 1: 반복 검출 2: 한번만 검출, 5초뒤 정상 복귀 line 1193
//0727 ECOR 임시 통신 x,, 과전압, 과전류, 온도이상시 충전 종료 line 503
//0727 커넥터 연결 오류가 가끔 발생 커넥터 위치센서를 활용해야할듯
//0729 IMIU 통신 및 파싱코드 삽입
//0730 멀티루프로 전환, total_Fault line887
//0802 파워모듈 Fail 업데이트 line 898, Pre-charging 오류 PLC 동작 부분에 업데이트
//     전력량계에서 전압, 전류 받는 코드 추가해야함, EOCR을 Serail3번으로 옮겨야 할듯
//0803 EOCR 그냥 Serial0번으로 사용해도 괜찮을듯 함
//0805 Button2_OK에 스위치 작동시 릴레이만 제어 line 663
//0809 커넥터를 꽂아주세요에서 2분동안 아무동작 없으면 초기화면으로 감, 이때 Charging_Start 1로 유지되면서
//     커넥터스위치 에러가 뜨지 않아서 Charging_State 이용함
//0810 커런트디맨드에 Fault = 0x00이 숨어있었음, 파워모듈 On 상태에서 통신이 5초이상 안되면 파워모듈 에러 line833
//     Button2_OK에 릴레이 바로 동작
//0813 Bd_Status.Charge_Fin = 0x01; 이게 에러랑 겹치면서 에러가 0.1초만에 사라지는 것이였음
//     파워모듈 에러 테스트 성공, Power_ON만 신경쓰다가 릴레이 Off가 안돼서 Charging_State 같이씀
//     허부하시험에서 스타트하면 릴레이만 동작, 저항부하시험은 동일 동작으로 바꿈

//...  EOCR 전압 디텍팅해서 차단기 상태 확인 나중에 추가
//     계량기, EOCR 통신문제는 일단 해결된듯
#include <Arduino.h>
#include <Scheduler.h>
#include "variant.h"
#include <stdio.h>
#include <due_can.h>


unsigned long PLCTime = 0;
unsigned long MCTime = 0;
unsigned long RelayTime = 0;
unsigned long RelayTime_2 = 0;
unsigned long RelayTime_test = 0;
unsigned long StopTime = 0;
unsigned long StopTime_2 = 0;
unsigned long StopTime2 = 0;
unsigned long DCTime2 = 0;
unsigned long EOCRTime = 0;
unsigned long FaultTime = 0;
unsigned long FaultTime_2 = 0;
unsigned long IMIUTime = 0;
unsigned long DCTime = 0;
unsigned long ConnTime = 0;
unsigned long powermoduleTime = 0;
unsigned long plcstartTime = 0;
unsigned long plcstartTime_2 = 0;
unsigned long Init_Time = 0;
unsigned long Init_Time_2 = 0;
unsigned long RFID_R_Time = 0;
unsigned long Connect_Check = 0;
unsigned long Connect_Check_2 = 0;
unsigned long cable_Check = 0;
unsigned long cable_Check_2 = 0;
int Connector_Fault = 0;
int Connector_Fault_2 = 0;
int Chagring_Reset = 0;
int Chagring_Reset_2 = 0;
int Start = 0; //PC에서 스타트 버튼이 안눌린 상태에서 사용
int start_sign = 0;
int start_sign_2 = 0;
int Charging_State = 0;
int Charging_Start = 0;
int Charging_State_2 = 0;
int Charging_Start_2 = 0;
int Mode = 1; //차를 충전할지, 부하장치에 테스트할지 선택
int power_ON = 0; //부하장치 테스트에서 사용
int power_ON_2 = 0;
int MC_ON = 0;
int Remain_sec = 0;
const int front_door = 2;
// const int back_door = 4;
const int Panel_ON = 36;
const int RFID_ON = 52;
int RFID_Reset = 0;
int door_OK = 0;
const int R_Relay = 23;
const int R_Relay_R = 8;    //릴레이 동작확인, J40
int R_Relay_OK = 0;
const int L_Relay = 22;
const int L_Relay_R = 7;   //릴레이 동작확인, J39
int L_Relay_OK = 0;
const int M_Relay = 25;
const int M_Relay_R = 4;   //릴레이 동작확인, J33
int M_Relay_OK = 0;
const int MC85 = 26;
const int MC85_R = 9;    //릴레이 동작확인
int MC85_OK = 0;

const int R_485 = 46;   //485 control
const int R_485_3 = 10;
const int R_485_4 = 47;

byte PC_data[40] = "";
byte LED_data[40] = "";
byte Check_data[40] = "";
byte CRC_Check[2] = "";
int CRC_OK = 0;
byte address[421][2] = {0,};
byte Write_res[8] = "";
byte Read_res[100] = "";
byte Write_DCmeter[8] = "";
byte DCmeter_address1 = 0;
byte DCmeter_address2 = 0;
byte DCmeter_address_select = 0;
byte DCmeter_ad = 0;
int DC_address = 0;
int modbus_set = 0;
int extract_data = 0; 
int EB_Array[16] = {0,};
int EB_Stat;
int total_Fault = 0;
int total_Fault_2 = 0;
int Plug_Check = 0;
struct EB_Data {
  int Standby, Ready, Run, Start, Stop, Door_Open, Fault, TestMode, ResetReq;   //200
  double Set_Voltage, Set_Current;                                              //206,208
  byte Run_Count[2];                                                               //203
  int False_Test, Resistance_Test;                                              //204
  int Power_Value, Order_bit;                                                   //202
  byte Protocol_Ver[2];                                                            //209
};
struct Charger_data {
  int Board_Ready, Charge_Ready, Plug_Check, Door_Check, Charge_Run, Charge_Fin, Fault;
  int Reset_Status, Connector_Lock, Remain_Time;
  byte Board_Ver[2], Battery_SOC;
  double Energy;
  double Voltage, Current;
  word Po_Module_Status1, Po_Module_Status2;
  word DC_MeterVer[2];
};
struct Ch_Fault {
  byte Main_Switch, Over_ACVoltage, Low_ACVoltage, Over_ACCurrent, Ground, MC, Relay, Over_DCVoltage;  //419
  byte Over_DCCurrent, Short, Pre_Charging, Power_Module, Temperature;   //419
  byte CP_Error, EV_Error, PLC_Error, CAN_Error, Control_Board, Car_Error, Emergency, Connector_Lock;
  byte Connector_position, DC_Meter, Door_Open, the_others;
};
struct EB_Data EB_Status, EB_Status_2;
struct EB_Data Test_Mode, Test_Mode_2;
struct Charger_data Bd_Status, Bd_Status_2;
struct Ch_Fault Ch_Fault1 , Ch_Fault1_2;
struct Ch_Fault Ch_Fault2 , Ch_Fault2_2;
int testmode = 0;
int testmode_2 = 0;


int DC_E=0, DC_M=0, DC_pos=0;
unsigned int DC_decimal = 0;
int DC_bin[32];         
int Power_ON = 0;
int Power_ON_2 = 0;
int Charge_On = 0;
int Relay_ON = 0;
byte zero = 0x00;
int car = 0;
int car_2 = 0;
int PLC_Reset = 0;
int PLC_Reset_2 = 0;
int power = 0;
int power_1 = 0;
int power_2 = 0;
int setVoltage = 0;
byte EVSEStatusCode = 0x01;
byte ResponseCode = 0x00;
byte nowvol1, nowvol2;
int nowvol_ch;                //nowvol2 change
byte nowcur1, nowcur2;
int nowcur_ch;
byte curlimit1, curlimit2;
int curlimit_ch;
byte nowvol1_2, nowvol2_2;
int nowvol_ch_2;                //nowvol2 change
byte nowcur1_2, nowcur2_2;
int nowcur_ch_2;
byte curlimit1_2, curlimit2_2;
int curlimit_ch_2;
unsigned int pre_id;
char PLC_data[400] = "";
char PLC_data_2[400] = "";
byte IMIUdata[400] = "";
word IMIU_CRC = 0;
char DCdata[200] = "";
char DCdata_2[200] = "";
int EOCR = 2;
byte EOCRdata[7] = "";
byte EOCR_CRC[7] = "";
int EOCR_Read = 0;
double IMIUvol = 0; 
int IMIU_Start = 0;
int IMIUCount = 0;
char IMIU_vol[16]="";
int LEDCount = 0;
byte Power_State = 0x00;
byte Board_State = 0x00;
int checksum=0;
byte checksum1, checksum2;
int checksum_ch;
byte plccur1, plccur2, plccur3, plccur4;
int plccur_ch;
int plccur_ch_2;
byte plcvol1, plcvol2, plcvol3, plcvol4;
int plcvol_ch;
int plcvol_ch_2;
byte plcvol[4] = "";
byte plccur[4] = "";
// double maxpower = 50;       //설정 전력 값
double maxpower = 100;       //설정 전력 값
double maxpower_2 = 100;       //설정 전력 값
double voltage = 0;
double current = 0;
int maxcurrent = 10000;
int plccurrent = 0;
int plcvoltage = 0;
int DCmeter = 1;
char DC_id[6] = "";
byte DC_V[3] = "";                //여기부터 3줄 DC_Voltage값 문자열로 변환하는데 사용
char DC_VCH[4] = "";
int DC_Cut_V = 2;
char DC_Voltage[10] = "001.0";
byte DC_C[3] = "";                //여기부터 3줄 DC_Current값 문자열로 변환하는데 사용
char DC_CCH[4] = "";
int DC_Cut_C = 2;
char DC_Current[10] = "001.000";
byte DC_W[4] = "";                //여기부터 3줄 DC_Watt값 문자열로 변환하는데 사용
char DC_WCH[4] = "";
int DC_Cut_W = 2;
char DC_Watt[11] = "000000.00";
int DC_Count = 0;
int DC_Error = 0;
int EOCR_Count = 0;
double Energy = 0;
double Previous_Energy = 0;
double Energy_2 = 0;
double Previous_Energy_2 = 0;
double R_thermistor = 0;        //온도센서 저항
double T_temperature = 0;         //온도센서 온도
int T_temperature_10 = 0;         //온도센서 온도
double T_Voltage = 0;           //온도센서(아날로그 10핀)의 전압
int T_Sensor = 0;               //아날로그 10핀에서 읽은 값    

double Me_Voltage = 1;
double Me_Current = 1;
double Me_Voltage_2 = 1;
double Me_Current_2 = 1;
int bufl = 0;               //DC 버퍼길이
const int Emergency_R = 6;
int Emergency_OK = 0;
const int Button1_R = 45;
int Button1_OK = 0;
const int Button2_R = 44;
int Button2_OK = 0;
const int Conn_R = 5;
int Conn_OK = 0;

CAN_FRAME incoming;
#define ADDRESS                        0b1100000  
#define ALLCALL_ADDRESS                0b1101000
#define RESET_ADDRESS                  0b1101011
#define I2C_READ                       1
#define I2C_WRITE                      0
#define NO_AUTO_INCREMENT              0b00000000
#define AUTO_INCREMENT_ALL_REGISTERS   0b10000000
#define AUTO_INCREMENT_BRIGHTNESS      0b10100000
#define AUTO_INCREMENT_CONTROL         0b11000000
#define AUTO_INCREMENT_BRIGHT_CONTROL  0b11100000
#define TLC59116_GRPPWM                0x12
#define TLC59116_LEDOUT0               0x14
#define LED_OUTPUT_OFF                 0b00
#define LED_OUTPUT_GROUP               0b11

void sendCarMsg() {
  int i = 0;
  if(car == 1) {          //report test
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0xF1);
    Serial2.write(0x04);
    Serial2.write(0x49);
    Serial2.write(0x4E);
    Serial2.write(0x49);
    Serial2.write(0x54);
    Serial2.write(0x34);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 2) {      //Start Request
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0xFC);
    Serial2.write(0x04);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 3) {       //Session Setup Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x82);
    Serial2.write(0x2C);
    Serial2.write(ResponseCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x36);        //현재시간으로 변경해야 할수도?
    Serial2.write(0x08);
    Serial2.write(0xF2);
    Serial2.write(0x5B);
    Serial2.write(0x02);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x30);
    Serial2.write(0x30);
    for(i=0;i<15;i++) {
    Serial2.write(zero);
    Serial2.write(zero);
    }
    Serial2.write(0xEE + ResponseCode);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 4) {       //Service Discovery Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x83);
    Serial2.write(0x0C);
    Serial2.write(ResponseCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x03);
    Serial2.write(0x01);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x06 + ResponseCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 5) {       //Service Payment Selection Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x85);
    Serial2.write(0x04);
    Serial2.write(ResponseCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(ResponseCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 6) {       //Contract Authentication Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x87);
    Serial2.write(0x04);
    Serial2.write(ResponseCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(ResponseCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 7) {       //Charge Parameter Discovery Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x88);
    Serial2.write(0x74);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(EVSEStatusCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0xFF);
    Serial2.write(0x03);
    Serial2.write(0x90);
    Serial2.write(0x01);
    Serial2.write(0x01);
    Serial2.write(0x07);
    Serial2.write(0x70);
    Serial2.write(0x17);
    Serial2.write(0xFF);
    Serial2.write(0x05);
    Serial2.write(0x10);      //1130 = 440, 11F8 = 460, 12C0 = 480, 1388 = 500
    Serial2.write(0x27);
    Serial2.write(0xFF);
    Serial2.write(0x03);
    Serial2.write(0x0A);
    Serial2.write(zero);
    Serial2.write(0xFF);
    Serial2.write(0x05);
    Serial2.write(0xD0);
    Serial2.write(0x07);
    Serial2.write(0xFF);
    Serial2.write(0x03);
    Serial2.write(0x0A);
    Serial2.write(zero);
    Serial2.write(0xFF);
    Serial2.write(0x03);
    Serial2.write(0x0A);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(0x07);
    Serial2.write(0xC4);
    Serial2.write(0x09);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x0A);
    Serial2.write(zero);
    Serial2.write(0x14);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x20);
    Serial2.write(0x4E);
    for(i=0;i<7;i++) {
      Serial2.write(zero);
      Serial2.write(zero);
    }
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x80);
    Serial2.write(0x51);
    Serial2.write(0x01);
    Serial2.write(zero);
    for(i=0;i<16;i++) {
      Serial2.write(zero);
      Serial2.write(zero);
    }
    Serial2.write(0x01 + EVSEStatusCode);
    Serial2.write(0x0B);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 8) {     //Cablecheck Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x8F);
    Serial2.write(0x10);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(EVSEStatusCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01 + EVSEStatusCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 9) {       //Precharge Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x90);
    Serial2.write(0x10);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(EVSEStatusCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0xFF);
    Serial2.write(0x05);
    Serial2.write(nowvol1);
    Serial2.write(nowvol2);
    checksum = 0x0106 + nowvol1 + nowvol2 + EVSEStatusCode;
    checksum1 = checksum&0xff;
    checksum_ch = checksum&(0xff<<8);
    checksum_ch = checksum_ch>>8;
    checksum2 = checksum_ch&0xff;
    Serial2.write(checksum);
    Serial2.write(checksum2);
    Serial2.write(zero);
    Serial2.write(zero);
    
  }
  else if(car == 10) {      //Current Demand Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x91);
    Serial2.write(0x24);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(EVSEStatusCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0xff);
    Serial2.write(0x05);
    Serial2.write(nowvol1);
    Serial2.write(nowvol2);
    Serial2.write(0xff);
    Serial2.write(0x03);
    Serial2.write(nowcur1);
    Serial2.write(nowcur2);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0xff);
    Serial2.write(0x05);
    Serial2.write(0x10);      //1130 = 440, 11F8 = 460, 12C0 = 480, 1388 = 500
    Serial2.write(0x27);
    Serial2.write(0xff);
    Serial2.write(0x03);
    Serial2.write(curlimit1);
    Serial2.write(curlimit2);
    Serial2.write(0x01);
    Serial2.write(0x07);
    Serial2.write(0x70);
    Serial2.write(0x17);
    checksum = 0x0537 + nowvol1 + nowvol2 + nowcur1 + nowcur2 + curlimit1 + curlimit2 +EVSEStatusCode;
    checksum1 = checksum&0xff;
    checksum_ch = checksum&(0xff<<8);
    checksum_ch = checksum_ch>>8;
    checksum2 = checksum_ch&0xff;
    Serial2.write(checksum1);
    Serial2.write(checksum2);
    Serial2.write(zero);
    Serial2.write(zero);
    
  }
  else if(car == 11) {      //PowerDelivery Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x89);
    Serial2.write(0x0c);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(EVSEStatusCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01 + EVSEStatusCode);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 12) {      //WeldingDetection Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x92);
    Serial2.write(0x10);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(0x02);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x05);
    Serial2.write(0x5A);
    Serial2.write(zero);
    Serial2.write(0x62);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car ==13) {       //SessionStop Response
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0x8C);
    Serial2.write(0x04);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else if(car == 14) {        //종료
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0xfe);
    Serial2.write(0x04);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Charging_Start = 0;

  }
  else if(car == 15) {        //Reset
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0xfe);
    Serial2.write(0x04);
    Serial2.write(0x48);
    Serial2.write(0x52);
    Serial2.write(0x53);
    Serial2.write(0x54);
    Serial2.write(0x41);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    car = 0;
    PLC_Reset = 1;
  }
  else if(car == 16) {
    Serial2.write(0x4B);
    Serial2.write(0x52);
    Serial2.write(0xf5);
    Serial2.write(0x04);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(0x01);
    Serial2.write(zero);
    Serial2.write(zero);
    Serial2.write(zero);
  }
  else {};
}
void sendCarMsg_2() {
  int i = 0;
  if(car_2 == 1) {          //report test
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0xF1);
    Serial.write(0x04);
    Serial.write(0x49);
    Serial.write(0x4E);
    Serial.write(0x49);
    Serial.write(0x54);
    Serial.write(0x34);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 2) {      //Start Request
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0xFC);
    Serial.write(0x04);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 3) {       //Session Setup Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x82);
    Serial.write(0x2C);
    Serial.write(ResponseCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x36);
    Serial.write(0x08);
    Serial.write(0xF2);
    Serial.write(0x5B);
    Serial.write(0x02);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x30);
    Serial.write(0x30);
    for(i=0;i<15;i++) {
    Serial.write(zero);
    Serial.write(zero);
    }
    Serial.write(0xEE + ResponseCode);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 4) {       //Service Discovery Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x83);
    Serial.write(0x0C);
    Serial.write(ResponseCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x03);
    Serial.write(0x01);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x06 + ResponseCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 5) {       //Service Payment Selection Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x85);
    Serial.write(0x04);
    Serial.write(ResponseCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(ResponseCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 6) {       //Contract Authentication Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x87);
    Serial.write(0x04);
    Serial.write(ResponseCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(ResponseCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 7) {       //Charge Parameter Discovery Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x88);
    Serial.write(0x74);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(EVSEStatusCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0xFF);
    Serial.write(0x03);
    Serial.write(0x90);
    Serial.write(0x01);
    Serial.write(0x01);
    Serial.write(0x07);
    Serial.write(0x70);
    Serial.write(0x17);
    Serial.write(0xFF);
    Serial.write(0x05);
    Serial.write(0x10);      //1130 = 440, 11F8 = 460, 12C0 = 480, 1388 = 500
    Serial.write(0x27);
    Serial.write(0xFF);
    Serial.write(0x03);
    Serial.write(0x0A);
    Serial.write(zero);
    Serial.write(0xFF);
    Serial.write(0x05);
    Serial.write(0xD0);
    Serial.write(0x07);
    Serial.write(0xFF);
    Serial.write(0x03);
    Serial.write(0x0A);
    Serial.write(zero);
    Serial.write(0xFF);
    Serial.write(0x03);
    Serial.write(0x0A);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(0x07);
    Serial.write(0xC4);
    Serial.write(0x09);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x0A);
    Serial.write(zero);
    Serial.write(0x14);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x20);
    Serial.write(0x4E);
    for(i=0;i<7;i++) {
      Serial.write(zero);
      Serial.write(zero);
    }
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x80);
    Serial.write(0x51);
    Serial.write(0x01);
    Serial.write(zero);
    for(i=0;i<16;i++) {
      Serial.write(zero);
      Serial.write(zero);
    }
    Serial.write(0x01 + EVSEStatusCode);
    Serial.write(0x0B);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 8) {     //Cablecheck Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x8F);
    Serial.write(0x10);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(EVSEStatusCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01 + EVSEStatusCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 9) {       //Precharge Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x90);
    Serial.write(0x10);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(EVSEStatusCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0xFF);
    Serial.write(0x05);
    Serial.write(nowvol1_2);
    Serial.write(nowvol2_2);
    checksum = 0x0106 + nowvol1_2 + nowvol2_2 + EVSEStatusCode;
    checksum1 = checksum&0xff;
    checksum_ch = checksum&(0xff<<8);
    checksum_ch = checksum_ch>>8;
    checksum2 = checksum_ch&0xff;
    Serial.write(checksum);
    Serial.write(checksum2);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 10) {      //Current Demand Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x91);
    Serial.write(0x24);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(EVSEStatusCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0xff);
    Serial.write(0x05);
    Serial.write(nowvol1_2);
    Serial.write(nowvol2_2);
    Serial.write(0xff);
    Serial.write(0x03);
    Serial.write(nowcur1_2);
    Serial.write(nowcur2_2);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0xff);
    Serial.write(0x05);
    Serial.write(0x10);      //1130 = 440, 11F8 = 460, 12C0 = 480, 1388 = 500
    Serial.write(0x27);
    Serial.write(0xff);
    Serial.write(0x03);
    Serial.write(curlimit1);
    Serial.write(curlimit2);
    Serial.write(0x01);
    Serial.write(0x07);
    Serial.write(0x70);
    Serial.write(0x17);
    checksum = 0x0537 + nowvol1_2 + nowvol2_2 + nowcur1_2 + nowcur2_2 + curlimit1 + curlimit2 +EVSEStatusCode;
    checksum1 = checksum&0xff;
    checksum_ch = checksum&(0xff<<8);
    checksum_ch = checksum_ch>>8;
    checksum2 = checksum_ch&0xff;
    Serial.write(checksum1);
    Serial.write(checksum2);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 11) {      //PowerDelivery Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x89);
    Serial.write(0x0c);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(EVSEStatusCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01 + EVSEStatusCode);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 12) {      //WeldingDetection Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x92);
    Serial.write(0x10);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(0x02);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x05);
    Serial.write(0x5A);
    Serial.write(zero);
    Serial.write(0x62);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 ==13) {       //SessionStop Response
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0x8C);
    Serial.write(0x04);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else if(car_2 == 14) {        //종료
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0xfe);
    Serial.write(0x04);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Charging_Start_2 = 0;
  }
  else if(car_2 == 15) {        //Reset
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0xfe);
    Serial.write(0x04);
    Serial.write(0x48);
    Serial.write(0x52);
    Serial.write(0x53);
    Serial.write(0x54);
    Serial.write(0x41);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    car_2 = 0;
    PLC_Reset_2 = 1;
  }
  else if(car_2 == 16) {
    Serial.write(0x4B);
    Serial.write(0x52);
    Serial.write(0xf5);
    Serial.write(0x04);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(0x01);
    Serial.write(zero);
    Serial.write(zero);
    Serial.write(zero);
  }
  else {};
}
void canData()                            //파워모듈 캔통신
{
  CAN_FRAME outgoing;
  outgoing.extended = true;
  outgoing.priority = 0; //0-15 lower is higher priority
  outgoing.length = 8;
  if(Power_State == 1 || Power_State == 2) {          //1채널 충전일 경우
    if(power == 1 || power ==2) {           //파워모듈 실행/정지
      outgoing.id = 0x029a3ff0;           //전체 실행
      if(power == 1) {
        outgoing.data.byte[0] = 0x00;
      }
      else {
        outgoing.data.byte[0] = 0x01;
      }
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
      if(power == 2) {
        MC_ON = 2;
        Charge_On = 2;
        Relay_ON = 0;
      }
    }
    else if(power == 3) {                     //전압, 전류 체크
      outgoing.id = 0x02813ff0;
      outgoing.data.byte[0] = 0x00;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
    }
    else if(power == 4) {                     //온도 체크
      outgoing.id = 0x028401f0;
      outgoing.data.byte[0] = 0x00;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
    }
    else if(power == 5) {                     //전압, 전류 세팅
      outgoing.id = 0x029c3ff0;
      outgoing.data.byte[0] = plcvol4;
      outgoing.data.byte[1] = plcvol3;
      outgoing.data.byte[2] = plcvol2;
      outgoing.data.byte[3] = plcvol1;
      outgoing.data.byte[4] = plccur4;
      outgoing.data.byte[5] = plccur3;
      outgoing.data.byte[6] = plccur2;
      outgoing.data.byte[7] = plccur1;
    } 
  }
  else if(Power_State == 3) {               //동시충전
    if(power == 1 || power ==2) {           //파워모듈 실행/정지
      // outgoing.id = 0x029a3ff0;
      outgoing.id = 0x02Da01f0;             //1번 ID만 출력
      if(power == 1) {
        outgoing.data.byte[0] = 0x00;
      }
      else {
        outgoing.data.byte[0] = 0x01;
      }
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
      if(power == 2) {
        MC_ON = 2;
        Charge_On = 2;
        Relay_ON = 0;
      }
    }
    else if(power == 3) {                     //전압, 전류 체크
      outgoing.id = 0x02c101f0;
      outgoing.data.byte[0] = 0x00;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
    }
    else if(power == 4) {                     //온도 체크
      outgoing.id = 0x028401f0;
      outgoing.data.byte[0] = 0x00;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
    }
    else if(power == 5) {                     //전압, 전류 세팅
      outgoing.id = 0x02dcf001;
      outgoing.data.byte[0] = plcvol4;
      outgoing.data.byte[1] = plcvol3;
      outgoing.data.byte[2] = plcvol2;
      outgoing.data.byte[3] = plcvol1;
      outgoing.data.byte[4] = plccur4;
      outgoing.data.byte[5] = plccur3;
      outgoing.data.byte[6] = plccur2;
      outgoing.data.byte[7] = plccur1;
    }
    else if(power == 7 || power == 8) {           //파워모듈 실행/정지
      // outgoing.id = 0x029a3ff0;
      outgoing.id = 0x02Da02f0;                   //2번 ID만 출력
      if(power == 7) {
        outgoing.data.byte[0] = 0x00;
      }
      else {
        outgoing.data.byte[0] = 0x01;
      }
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
      if(power == 8) {
        MC_ON = 2;
        Charge_On = 2;
        Relay_ON = 0;
      }
    }
    else if(power == 9) {                     //전압, 전류 체크
      outgoing.id = 0x02c102f0;
      outgoing.data.byte[0] = 0x00;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
    }
    else if(power == 11) {                     //전압, 전류 세팅
      outgoing.id = 0x02dcf002;
      outgoing.data.byte[0] = plcvol[0];
      outgoing.data.byte[1] = plcvol[1];
      outgoing.data.byte[2] = plcvol[2];
      outgoing.data.byte[3] = plcvol[3];
      outgoing.data.byte[4] = plccur[0];
      outgoing.data.byte[5] = plccur[1];
      outgoing.data.byte[6] = plccur[2];
      outgoing.data.byte[7] = plccur[3];
    }
  }
  if(power == 6) {                     //저소음모드
    outgoing.id = 0x028f3ff0;
    outgoing.data.byte[0] = 0x11;
    outgoing.data.byte[1] = 0x13;
    outgoing.data.byte[2] = 0x00;
    outgoing.data.byte[3] = 0x00;
    outgoing.data.byte[4] = 0x00;
    outgoing.data.byte[5] = 0x00;
    outgoing.data.byte[6] = 0x00;
    outgoing.data.byte[7] = 0xA1;
  }
  
  Can0.sendFrame(outgoing);
}
void DC_Write() {                     //modbus 후아방 계량기
  if(DCmeter == 1) {
    if(DCmeter_address_select == 0) {
      Write_DCmeter[0] = DCmeter_address1;
    }
    else {
      Write_DCmeter[0] = DCmeter_address2;
    }
    Write_DCmeter[1] = 0x03;
    Write_DCmeter[2] = 0;
    Write_DCmeter[3] = 0x64;
    Write_DCmeter[4] = 0;
    Write_DCmeter[5] = 0x02;
    Write_DCmeter[6] = CRC16(Write_DCmeter,6);
    Write_DCmeter[7] = CRC16(Write_DCmeter,7);
    digitalWrite(R_485, HIGH);
    // Serial.write(DCmeter_address);
    Serial3.write(Write_DCmeter[0]);
    Serial3.write(Write_DCmeter[1]);
    Serial3.write(Write_DCmeter[2]);
    Serial3.write(Write_DCmeter[3]);
    Serial3.write(Write_DCmeter[4]);
    Serial3.write(Write_DCmeter[5]);
    Serial3.write(Write_DCmeter[6]);
    Serial3.write(Write_DCmeter[7]);
    delay(10);
    digitalWrite(R_485, LOW);
  }
  else if(DCmeter == 2) {
    if(DCmeter_address_select == 0) {
      Write_DCmeter[0] = DCmeter_address1;
    }
    else {
      Write_DCmeter[0] = DCmeter_address2;
    }
    Write_DCmeter[1] = 0x03;
    Write_DCmeter[2] = 0;
    Write_DCmeter[3] = 0x6A;
    Write_DCmeter[4] = 0;
    Write_DCmeter[5] = 0x02;
    Write_DCmeter[6] = CRC16(Write_DCmeter,6);
    Write_DCmeter[7] = CRC16(Write_DCmeter,7);
    digitalWrite(R_485, HIGH);
    // Serial3.write(DCmeter_address);
    Serial3.write(Write_DCmeter[0]);
    Serial3.write(Write_DCmeter[1]);
    Serial3.write(Write_DCmeter[2]);
    Serial3.write(Write_DCmeter[3]);
    Serial3.write(Write_DCmeter[4]);
    Serial3.write(Write_DCmeter[5]);
    Serial3.write(Write_DCmeter[6]);
    Serial3.write(Write_DCmeter[7]);
    delay(10);
    digitalWrite(R_485, LOW);
  }
  else if(DCmeter == 3) {
    if(DCmeter_address_select == 0) {
      Write_DCmeter[0] = DCmeter_address1;
      DCmeter_address_select = 1;
    }
    else {
      Write_DCmeter[0] = DCmeter_address2;
      DCmeter_address_select = 0;
    }
    Write_DCmeter[1] = 0x03;
    Write_DCmeter[2] = 0;
    Write_DCmeter[3] = 0;
    Write_DCmeter[4] = 0;
    Write_DCmeter[5] = 0x02;
    Write_DCmeter[6] = CRC16(Write_DCmeter,6);
    Write_DCmeter[7] = CRC16(Write_DCmeter,7);
    digitalWrite(R_485, HIGH);
    // Serial3.write(DCmeter_address);
    Serial3.write(Write_DCmeter[0]);
    Serial3.write(Write_DCmeter[1]);
    Serial3.write(Write_DCmeter[2]);
    Serial3.write(Write_DCmeter[3]);
    Serial3.write(Write_DCmeter[4]);
    Serial3.write(Write_DCmeter[5]);
    Serial3.write(Write_DCmeter[6]);
    Serial3.write(Write_DCmeter[7]);
    delay(10);
    digitalWrite(R_485, LOW);
  }
}
void ReadData() {               //PLC 통신 값 읽어오기
  int a;
  int z = 0;
  z=0;
  a = Serial2.read();
  if(a == 0x47) {
    PLC_data[z] = a;
    z++;
    while(z<4) {
      a = Serial2.read();
      if(a != -1) {
        PLC_data[z] = a;
        z++;
      }
    }
    while(1) {
    a = Serial2.read();
      if(a != -1) {
        PLC_data[z] = a;
        z++;
        PLCTime = millis();
      }
      if(z==PLC_data[3]+7) {
        break;
      }
      if((millis()-PLCTime) > 1500) {
        break;
      }
    }
  }
  delay(3);
}
void ReadData_2() {               //PLC 통신 값 읽어오기
  int a;
  int z = 0;
  z=0;
  a = Serial.read();
  if(a == 0x47) {
    PLC_data_2[z] = a;
    z++;
    while(z<4) {
      a = Serial.read();
      if(a != -1) {
        PLC_data_2[z] = a;
        z++;
      }
    }
    while(1) {
    a = Serial.read();
      if(a != -1) {
        PLC_data_2[z] = a;
        z++;
        PLCTime = millis();
      }
      if(z==PLC_data_2[3]+7) {
        break;
      }
      if((millis()-PLCTime) > 1500) {
        break;
      }
    }
  }
  delay(3);
}
void PC_Read() {
  int pcdata;
  pcdata = Serial1.read();
  delay(2);
  if(pcdata == 0x01) {
    PC_data[0] = 0x01; 
    Check_data[0] = 0x01; 
    pcdata = Serial1.read();
    delay(2);
    if(pcdata == 0x10) {
      PC_data[1] = 0x10;
      Check_data[1] = 0x10;
      for(int j=0; j<5; j++) {
        pcdata = Serial1.read();
        PC_data[j+2] = pcdata;
        Check_data[j+2] = pcdata;
        delay(2);
      }
      delay(PC_data[6]+10);
      for(int j=0; j<PC_data[6]+2; j++) {
        pcdata = Serial1.read();
        PC_data[j+7] = pcdata;
        Check_data[j+7] = pcdata;
      }
      Check_data[PC_data[6]+7] = '\0';
      Check_data[PC_data[6]+8] = '\0';
    }
    else if(pcdata == 0x04) {
      PC_data[1] = 0x04;
      Check_data[1] = pcdata;
      delay(10);
      for(int j=0; j<6; j++) {
        pcdata = Serial1.read();
        PC_data[j+2] = pcdata;
        Check_data[j+2] = pcdata;
      }
      Check_data[6] = '\0';
      Check_data[7] = '\0';
    }
  }
}
void Write_Response() {
  if(CRC_OK == 1) {
    digitalWrite(R_485_3, HIGH);
    Serial1.write(Write_res[0]);
    Serial1.write(Write_res[1]);
    Serial1.write(Write_res[2]);
    Serial1.write(Write_res[3]);
    Serial1.write(Write_res[4]);
    Serial1.write(Write_res[5]);
    Serial1.write(Write_res[6]);
    Serial1.write(Write_res[7]);
    delay(4);
    digitalWrite(R_485_3, LOW);
  }
  else {
    digitalWrite(R_485_3, HIGH);
    Serial1.write(Write_res[0]);
    Serial1.write(Write_res[1]);
    Serial1.write(Write_res[2]);
    Serial1.write(Write_res[3]);
    Serial1.write(Write_res[4]);
    delay(3);
    digitalWrite(R_485_3, LOW);
  }
  for(int j=0; j<40; j++) {
    PC_data[j] = '\0';
  }
}
void Read_Response() {
  if(CRC_OK == 1) {
    digitalWrite(R_485_3, HIGH);
    Serial1.write(Read_res[0]);
    Serial1.write(Read_res[1]);
    Serial1.write(Read_res[2]);
    for(int j=0; j<Read_res[2]+2; j++) {
      Serial1.write(Read_res[j+3]);
    }
    if(Read_res[2]-4 < 3) {
      delay(3);
    }
    else {
      delay(Read_res[2]/4+3);
    }
    digitalWrite(R_485_3, LOW);
  }
  else {
    digitalWrite(R_485_3, HIGH);
    Serial1.write(Read_res[0]);
    Serial1.write(Read_res[1]);
    Serial1.write(Read_res[2]);
    Serial1.write(Read_res[3]);
    Serial1.write(Read_res[4]);
    delay(5);
    digitalWrite(R_485_3, LOW);
  }
  for(int j=0; j<40; j++) {
    PC_data[j] = '\0';
  }
}

void RS485_Read() {               //전력량계, EOCR 통신 값 읽어오기
  int b;
  b = Serial3.read();
  if(b == DCmeter_address1) {
    DCdata[DC_Count] = b;
    DC_Count++;
    delay(30);
    for(int p = 0; p<8; p++) {
      b = Serial3.read();
      DCdata[DC_Count] = b;
      DC_Count++;
    }
  }
  if(b == DCmeter_address2) {
    DCdata_2[DC_Count] = b;
    DC_Count++;
    delay(30);
    for(int p = 0; p<8; p++) {
      b = Serial3.read();
      DCdata_2[DC_Count] = b;
      DC_Count++;
    }
  }
}
word CRC16 (const byte *nData, word wlength) {
  static const word wCRCTable[] = {
  0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
  0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
  0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
  0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
  0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
  0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
  0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
  0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
  0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
  0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
  0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
  0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
  0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
  0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
  0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
  0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
  0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
  0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
  0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
  0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
  0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
  0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
  0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
  0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
  0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
  0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
  0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
  0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
  0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
  0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
  0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
  0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

  byte nTemp;
  word wCRCWord = 0xffff;

  while(wlength--) {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}


void setup() {                            //기본세팅
  //Wire1.begin();
  Serial.begin(38400);                     //PLC meter
  //Serial.begin(38400);                   //DC 통신
  Serial1.begin(38400);                   //DC 통신
  Serial2.begin(38400);                    //PLC 통신
  Serial3.begin(9600,SERIAL_8E1);                    //DC 통신
  // Serial3.begin(9600,SERIAL_8E1);                    //IMIU 통신
  Can0.begin(CAN_BPS_125K);               //파워모듈 통신
  // Can1.begin(CAN_BPS_125K);
  Can0.watchFor();
  analogReference(AR_DEFAULT);
  analogReadResolution(12);
  pinMode(R_Relay, OUTPUT);                  //릴레이 동작
  pinMode(L_Relay, OUTPUT);
  pinMode(M_Relay, OUTPUT);
  pinMode(MC85, OUTPUT);
  pinMode(R_Relay_R, INPUT_PULLUP);                  //릴레이 동작
  pinMode(L_Relay_R, INPUT_PULLUP);                //릴레이 인풋 -> 485,pc통신 제어
  pinMode(M_Relay_R, INPUT_PULLUP); 
  pinMode(MC85_R, INPUT_PULLUP);
  pinMode(front_door, INPUT_PULLUP);
  // pinMode(back_door, INPUT_PULLUP);             //도어 아웃풋임 수정필요,일단 485로 사용중
  pinMode(RFID_ON, OUTPUT);
  pinMode(R_485, OUTPUT);
  pinMode(R_485_3, OUTPUT);
  pinMode(R_485_4, OUTPUT);
  pinMode(Emergency_R, INPUT_PULLUP);
  pinMode(Button1_R, INPUT_PULLUP);
  pinMode(Button2_R, INPUT_PULLUP);
  pinMode(Conn_R, INPUT_PULLUP);
  digitalWrite(R_Relay, LOW);
  digitalWrite(L_Relay, LOW);
  digitalWrite(M_Relay, LOW);
  digitalWrite(MC85, LOW);
  //digitalWrite(RFID_ON, HIGH);
  digitalWrite(R_485, LOW);
  digitalWrite(R_485_3, LOW);
  digitalWrite(R_485_4, LOW);
  pinMode(Panel_ON, OUTPUT);
  digitalWrite(Panel_ON, HIGH);
  
  Bd_Status.Board_Ver[0] = 0x27;
  Bd_Status.Board_Ver[1] = 0x11;
  Bd_Status.DC_MeterVer[0] = 0x00;
  Bd_Status.DC_MeterVer[1] = 0x00;
  Bd_Status.Board_Ready = 0x01;
  Bd_Status.Connector_Lock = 0x00;
  Bd_Status.Charge_Ready = 0x01;
  Write_DCmeter[1] = 0x03;
  Write_DCmeter[2] = 0;
  Write_DCmeter[3] = 0;
  Write_DCmeter[4] = 0;
  Write_DCmeter[5] = 0x02;
  delay(5000);
  
  Scheduler.startLoop(Serial0_485);
  Scheduler.startLoop(PLC_Communication);
}

void loop() {                                       //메인 루프
  if(modbus_set == 0) {                             //계량기 초기 세팅
    delay(20);
    if(Serial3.available()) {
      delay(5);
      byte b = Serial3.read();
      if(b == DC_address && DC_address != 0) {      //ID 2개
        b = Serial3.read();
        delay(10);
        if(b == 0x03) {
          // Serial3.flush();                       //안먹는거 같아서 뺌
          if(DCmeter_ad == 0) {
            DCmeter_address1 = DC_address;          //1번 ID
            DCmeter_ad = 1;
            delay(100);
          }
          else if(DCmeter_ad == 1) {
            DCmeter_address2 = DC_address;          //2번 ID
            digitalWrite(RFID_ON, HIGH);            //2개의 ID를 제대로 읽어왔다면
            modbus_set = 1;
            DCmeter = 4;
            DCTime2 = millis();
          }
          for(int z=0; z<7; z++) {
            b = Serial3.read();
            delay(2);
          }
        }
      }
    }
    else if(DC_address > 247) {
      // digitalWrite(RFID_ON, HIGH);
      modbus_set = 1;
      DCmeter = 4;
      DCTime2 = millis();
    }
    DC_address++;
    Write_DCmeter[0] = DC_address;                        //ID를 1씩 올리면서 커맨드를 날려봄
    Write_DCmeter[6] = CRC16(Write_DCmeter,6);
    Write_DCmeter[7] = CRC16(Write_DCmeter,7);
    digitalWrite(R_485, HIGH);
    // delay(2);
    Serial3.write(Write_DCmeter[0]);
    Serial3.write(Write_DCmeter[1]);
    Serial3.write(Write_DCmeter[2]);
    Serial3.write(Write_DCmeter[3]);
    Serial3.write(Write_DCmeter[4]);
    Serial3.write(Write_DCmeter[5]);
    Serial3.write(Write_DCmeter[6]);
    Serial3.write(Write_DCmeter[7]);
    delay(10);
    digitalWrite(R_485, LOW);
    // delay(5);
  }
  else {
    static unsigned long lastTime = 0;          //주기적으로 plc, 파워모듈 통신
    static unsigned long canTime = 0;           //파워모듈에서 전압,전류 세팅 시간
    static unsigned long TemperTime = 0;        //온도센서 계산 시간
    static unsigned long TestmodeTime = 0;      
    char powervol[16]="";
    char powercur[16]="";
    char id[8]="";
    double index = 0;
    double curdecimal = 0;
    double voldecimal = 0;
    int volbin[32];
    int modulebin[8];
    int curbin[32];
  
    char s3[4] = "";
    Emergency_OK = digitalRead(Emergency_R);      //비상스위치 읽기
    Button1_OK = digitalRead(Button1_R);          
    Button2_OK = digitalRead(Button2_R);
    Conn_OK = digitalRead(Conn_R);
    M_Relay_OK = digitalRead(M_Relay_R);          //중앙 릴레이 읽기
    // door_OK = digitalRead(front_door) || digitalRead(back_door);
    door_OK = digitalRead(front_door);            //도어는 4개라서 커넥터 하나로 통일, 어차피 하나만 열려도 도어오픈 에러임.
    if(millis() - TestmodeTime > 5000) {
      TestmodeTime = 0;
    }
    if(EB_Status.Standby == 0) {                  //초기화
      if(Chagring_Reset == 0){                    //충전 후 1분동안만 커넥터 위치센서 오류가 뜨게끔 함
        Chagring_Reset = 1;
        Init_Time = millis();
      }
      if(EB_Status.TestMode == 0x00) {            //테스트모드가 아닌경우에
        MCTime = 0;
        RelayTime = 0;
        // StopTime = 0;
        StopTime2 = 0;
        FaultTime = 0;
        plcstartTime = 0;
        Charging_State = 0;
        Charging_Start = 0;
        car = 0;
        if(TestmodeTime == 0) {                   //테스트모드 중지 정지 시쿼스가 안 지났는데 들어갈까봐
          StopTime = 0;
          Power_ON = 0;
          // digitalWrite(MC85, LOW);
          if(Button2_OK == HIGH) {
            digitalWrite(L_Relay, LOW);
          }
        }
      }
      Bd_Status.Charge_Fin = 0x00;
      Bd_Status.Plug_Check = 0x00;
      Plug_Check = 0;
      Ch_Fault1.Main_Switch = 0;
      Ch_Fault1.Over_ACVoltage = 0;
      Ch_Fault1.Low_ACVoltage = 0;
      Ch_Fault1.Over_ACCurrent = 0;
      Ch_Fault1.Ground = 0;
      Ch_Fault1.MC = 0;
      Ch_Fault1.Relay = 0;
      Ch_Fault1.Over_DCVoltage = 0;
      Ch_Fault1.Over_DCCurrent = 0;
      Ch_Fault1.Short = 0;
      Ch_Fault1.Pre_Charging = 0;
      Ch_Fault1.Power_Module = 0;
      Ch_Fault1.Temperature = 0;
      Ch_Fault2.CP_Error = 0;
      Ch_Fault2.EV_Error = 0;
      Ch_Fault2.PLC_Error = 0;
      Ch_Fault2.CAN_Error = 0;
      Ch_Fault2.Control_Board = 0;
      Ch_Fault2.Car_Error = 0;
      Ch_Fault2.Emergency = 0;
      Ch_Fault2.DC_Meter = 0;
      Ch_Fault2.the_others = 0;
      start_sign = 0;
      Connect_Check = millis();
      if(PLC_Reset == 0) {                      //PLC모듈을 리셋함
        car = 15;                               //
        Start = 0;
        cable_Check = millis();
      }
    }
    if(cable_Check != 0) {
      if(millis() - cable_Check > 13000) {      //리셋 후 커맨드가 먹히는 시간 약 13초
        car = 16;
        cable_Check = 0;
        }
    }
    // if(RFID_Reset == 1) {
    //   digitalWrite(RFID_ON, LOW);
    //   RFID_R_Time = millis();
    //   RFID_Reset = 0;
    // }
    // else {
    //   if(millis() - RFID_R_Time > 2000) {
    //     digitalWrite(RFID_ON, HIGH);
    //   }
    // }
    if(Chagring_Reset == 1){                        //충전 후 커넥터센서 오류 없애기까지 1분~
      if(millis() - Init_Time > 60000) {
        Connector_Fault = 1;
      }
    }

    //테스트 모드 또는 일반모드 진입 시 초기 설정을 위한 부분
    if(EB_Status.TestMode == zero) {                
      if(Mode == 1) {
        testmode = 0;
      }
      else {
        testmode = 1;
      }
    }
    if(EB_Status.TestMode == 0x01) {
      if(Mode == 2) {
        testmode = 0;
      }
      else {
        testmode = 2;
      }
    }
    if(testmode == 1) {
      Mode = 1;   //차 충전
      plcvoltage = 0;
      plcvol1 = plcvoltage&0xff;
      plcvol_ch = plcvoltage&(0xff<<8);
      plcvol_ch = plcvol_ch>>8;
      plcvol2 = plcvol_ch&0xff;
      plcvol_ch = plcvoltage&(0xff<<16);
      plcvol_ch = plcvol_ch>>16;
      plcvol3 = plcvol_ch&0xff;
      plcvol_ch = plcvoltage&(0xff<<24);
      plcvol_ch = plcvol_ch>>24;
      plcvol4 = plcvol_ch&0xff;
    
      plccurrent = 0;
      plccur1 = plccurrent&0xff;                     //1의자리
      plccur_ch = plccurrent&(0xff<<8);
      plccur_ch = plccur_ch>>8;                
      plccur2 = plccur_ch&0xff;                      //10의자리
      plccur_ch = plccurrent&(0xff<<16);
      plccur_ch = plccur_ch>>16;
      plccur3 = plccur_ch&0xff;
      plccur_ch = plccurrent&(0xff<<24);
      plccur_ch = plccur_ch>>24;
      plccur4 = plccur_ch&0xff; 
      //Power_ON = 0;
      testmode = 0;
      StopTime = 0;
      car = 14;
      power = 2;
    }
    else if(testmode == 2) {
      Mode = 2;   //로드 테스트       Test_Mode.Set_Voltage
      plcvoltage = Test_Mode.Set_Voltage * 1000;
      //plcvoltage = 480000;
      plcvol1 = plcvoltage&0xff;
      plcvol_ch = plcvoltage&(0xff<<8);
      plcvol_ch = plcvol_ch>>8;
      plcvol2 = plcvol_ch&0xff;
      plcvol_ch = plcvoltage&(0xff<<16);
      plcvol_ch = plcvol_ch>>16;
      plcvol3 = plcvol_ch&0xff;
      plcvol_ch = plcvoltage&(0xff<<24);
      plcvol_ch = plcvol_ch>>24;
      plcvol4 = plcvol_ch&0xff;
      if(Power_ON_2 > 0) {
        plccurrent = (Test_Mode.Set_Current/4) * 1000;
      }
      else {
        plccurrent = (Test_Mode.Set_Current/8) * 1000;
      }
      plccur1 = plccurrent&0xff;                     //1의자리
      plccur_ch = plccurrent&(0xff<<8);
      plccur_ch = plccur_ch>>8;                
      plccur2 = plccur_ch&0xff;                      //10의자리
      plccur_ch = plccurrent&(0xff<<16);
      plccur_ch = plccur_ch>>16;
      plccur3 = plccur_ch&0xff;
      plccur_ch = plccurrent&(0xff<<24);
      plccur_ch = plccur_ch>>24;
      plccur4 = plccur_ch&0xff; 
      Power_ON = 0;
      testmode = 0;
      RelayTime_test = 0;
      RelayTime = 0;
      if(Test_Mode.Resistance_Test == 1) {
        MCTime = millis();
        RelayTime_test = 0;
      }
      else if(Test_Mode.False_Test == 1) {
        MCTime = 0;
        RelayTime_test = millis();
      }
      RelayTime = 0;
      power_ON = 0;
      StopTime = 0;
      TestmodeTime = millis();
    }

    //일반 충전모드
    if(EB_Status.TestMode == zero) {                    
      ReadData();
      if(EB_Status.Start == zero) {                           //대기상태
        if(PLC_data[4] == 0x01 && PLC_data[5] == 0x09) {      //커넥터 연결 상태
          // Bd_Status.Plug_Check = 0x01;
          Plug_Check = 0x01;
          Bd_Status.Charge_Ready = 0x01;
          Start = 1;
        }
        else if(PLC_data[4] == 0x01 && PLC_data[5] == 0x0C) {  //커넥터 연결 해제
          Bd_Status.Plug_Check = zero;
          Plug_Check = 0;
          Start = 0;
          start_sign = 0;
        }
        if(Charging_State == 0 || Charging_State == 15) {
          Charging_Start = 0;
        }
        if(EB_Status.Standby == 1) {                            //충전 후 안 뽑은 상태
          if(Start == 1) {
            // Bd_Status.Plug_Check = 0x01;
            Plug_Check = 0x01;
          }
        }
      }
      else if(EB_Status.Start == 0x01) {                        //충전 시작
        Charging_Start = 1;
      }
      if(Charging_Start == 1) {
        ConnTime = millis();
        if(Charging_State == 0 || Charging_State > 12) {
          if(Connect_Check != 0) {                                        //커넥터 연결여부 확인, 리포트를 이용해서
            if(millis() - Connect_Check > 2000) {
              Charging_State = 0;
              car = 16;
              Connect_Check = millis();
              delay(10);
            }
          }
        }
        if(PLC_data[0] == 0x47) {
          if(Charging_State == 0) {
            if(PLC_data[4] == 0x01 && PLC_data[5] == 0x09) {              //커넥터 연결
              Bd_Status.Plug_Check = 0x01;
              Bd_Status.Charge_Ready = 0x01;
              Bd_Status.Charge_Fin = 0x00;
              EVSEStatusCode = 0x01;
              ResponseCode = 0x01;
              Charging_State = 1;
              // Charge_On = 1;
              start_sign = 1;
              MCTime = 0;
              Connect_Check = 0;
            }
            else if(PLC_data[4] == 0x01 && PLC_data[5] == 0x0C) {
              start_sign = 0;
              Connect_Check = millis();
            }
          }
          else if(Charging_State != 0 && Charging_State < 11) {           //충전 도중 재연결?, 말이 안 됨 에러 뱉어
            if(PLC_data[4] == 0x01 && PLC_data[5] == 0x09) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
            else if(PLC_data[4] == 0x01 && PLC_data[5] == 0x0C) {
              start_sign = 0;
            }
          }
          if(start_sign == 1) {                                           //MC 동작 명령, PLC에 리포트 요청 보냄(충전 시작전 루틴)
            if(LED_data[2] == 0x00) {
              MCTime = millis();
              car = 1;
              start_sign = 2;
            }
          }
          if(plcstartTime != 0) {                                       //25초내로 충전 시작 안 하면 스타트 커맨드 다시 날림
            if(millis() - plcstartTime > 25000) {
              car = 2;
              plcstartTime = millis();
            }
            else if(millis() - plcstartTime > 20000) {
              car = 1;
            }
          }
          if (PLC_data[2] == 0x02) {
            Bd_Status.Plug_Check = 0x01;
            Bd_Status.Charge_Fin = 0x00;
            EVSEStatusCode = 0x01;
            ResponseCode = 0x01;
            car = 3;                    
            Charging_State = 3;
            plcstartTime = 0;
            PLC_Reset = 0;
            maxpower = 10;
            maxpower_2 = 10;
          }
          else if (PLC_data[2] == 0x03) {             //Session Setup Response
            Bd_Status.Plug_Check = 0x01;
            Bd_Status.Charge_Fin = 0x00;
            car = 4;       
            Charging_State = 4;
            plcstartTime = 0;
            if(ResponseCode != 0x04) {
              ResponseCode = 0x00;
            }
          }
          else if (PLC_data[2] == 0x05) {             //Service Discovery Response
            Bd_Status.Plug_Check = 0x01;
            car = 5;
            Charging_State = 5;
          }
          else if (PLC_data[2] == 0x07) {               //Service Payment Selection Response
            car = 6;
            Charging_State = 6;
          }
          else if (PLC_data[2] == 0x08) {               //Contract Authentication Response
            car = 7;
            Charging_State = 7;
            setVoltage = 0;
            // if(Power_ON_2 > 0 || Charging_State_2 > 0) {
            //   digitalWrite(M_Relay, LOW);
            // }
            if(PLC_data[11] != zero) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
          }
          else if (PLC_data[2] == 0x0f) {             //Charge Parameter Discovery Response
            car = 8;
            Charging_State = 8;
            if(PLC_data[7] != zero) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
          }
          else if (PLC_data[2] == 0x10) {          //전압 세팅해줘야됨     //Cablecheck Response
            Charging_State = 9;
            if(Power_ON == 0) {                                 //릴레이 닫음
              if(Me_Current_2 <50) {               //2번 채널이 충전중일 경우 충분히 전류가 내려가고 미드릴레이를 제어
                if(Power_ON_2 > 0) {               //2번 채널이 충전중일 때
                  digitalWrite(M_Relay, LOW);
                  delay(200);
                  digitalWrite(L_Relay, HIGH);
                }
                else {
                  digitalWrite(L_Relay, HIGH);
                }
                // RelayTime = millis();
                Power_ON++;
                powermoduleTime = millis();
              }
            }
            else if(RelayTime == 0) {
              L_Relay_OK = digitalRead(L_Relay_R);
              // L_Relay_OK = LOW;
              // R_Relay_OK = LOW;
              if(L_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
                car = 9;
                power_1 = 1;
              }
              else {
                car = 14;         //relay error
                power_1 = 2;
                Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
                Ch_Fault1.Relay = 1;
                total_Fault = 2;
                FaultTime = millis();
              }
            }
            plccur1 = 0x00;
            plccur2 = 0x00;
            plccur3 = 0x00;
            plccur4 = 0x00;
            if(PLC_data[12] == 0xFF) {         //10^-1
              index = 0.1;
            }
            else if(PLC_data[12] == 0x00) {    //10^0
              index = 1;
            }
            else if(PLC_data[12] == 0x01) {    //10^1
              index = 10;
            }
            voldecimal = PLC_data[15]*256 + PLC_data[14];
            plcvoltage = (voldecimal*index)*1000;        //파워모듈 세팅 전압값
            plcvol1 = plcvoltage&0xff;
            plcvol_ch = plcvoltage&(0xff<<8);
            plcvol_ch = plcvol_ch>>8;
            plcvol2 = plcvol_ch&0xff;
            plcvol_ch = plcvoltage&(0xff<<16);
            plcvol_ch = plcvol_ch>>16;
            plcvol3 = plcvol_ch&0xff;
            plcvol_ch = plcvoltage&(0xff<<24);
            plcvol_ch = plcvol_ch>>24;
            plcvol4 = plcvol_ch&0xff;
            Bd_Status.Battery_SOC = PLC_data[8];
            if(PLC_data[7] != zero) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
          }
          else if (PLC_data[2] == 0x11) {    //전류 들어가기 시작함
            Charging_State = 10;
            if(Charging_State_2 > 0 && Charging_State_2 < 9) {        //2채널에서 사용하려고 하면 출력 전력을 줄임
              maxpower_2 = 10;
              maxpower = 10;
            }
            else if(Charging_State_2 > 8 && Charging_State_2 < 11) {  //2채널 중전이 시작되면 출력 올림
              maxpower_2 = 100;
              maxpower = 100;
            }
            else if(Power_ON_2 == 0) {                                //2채널 사용 안하면 출력 최대로
              maxpower_2 = 200;
              maxpower = 200;
            }
            Bd_Status.Charge_Run = 0x01;
            Connector_Fault = 0;
            Chagring_Reset = 0;
            Init_Time = millis();
            L_Relay_OK = digitalRead(L_Relay_R);
            // L_Relay_OK = LOW;
            // R_Relay_OK = LOW;
            // if(L_Relay_OK == LOW && R_Relay_OK == LOW) {
            if(Relay_ON < 10) {
              car = 10;
              power_1 = 1;
              if(PLC_data[12] == 0xFF) {
                index = 0.1;
              }
              else if(PLC_data[12] == 0x00) {
                index = 1;
              }
              else if(PLC_data[12] == 0x01) {
                index = 10;
              }
              curdecimal = PLC_data[15]*256 + PLC_data[14];
              if(curdecimal*index >20) {
                plccurrent = (curdecimal*index)*1000;         //파워모듈 세팅 전류값
              }
              else {
                plccurrent = (curdecimal*index+2)*1000;
              }
              if((curdecimal*index) - current > 20) {           //한번에 20A이상 안올라 감
                plccurrent = (current + 20)*1000;
              }
              if(M_Relay_OK > 0) {                              //중앙 릴레이가 동작 안 한 상태면 2채널 운영 전류는 모듈 4개에서
                plccurrent = plccurrent/4;
                maxcurrent = (int)((((maxpower*1000)/voltage)*1000)/4);
              }
              else {                                            //중앙 릴레이가 붙어있으면 전류는 모듈 8개에서
                plccurrent = plccurrent/8;
                maxcurrent = (int)((((maxpower*1000)/voltage)*1000)/8);
              }
              if(maxcurrent > 27500) {                          //파워모듈 개당 최대전류
                maxcurrent = 27500;
              }
              if(plccurrent >= maxcurrent) {
                plccurrent = maxcurrent;
              }
              plccur1 = plccurrent&0xff;                     //1의자리
              plccur_ch = plccurrent&(0xff<<8);
              plccur_ch = plccur_ch>>8;                
              plccur2 = plccur_ch&0xff;                      //10의자리
              plccur_ch = plccurrent&(0xff<<16);
              plccur_ch = plccur_ch>>16;
              plccur3 = plccur_ch&0xff;
              plccur_ch = plccurrent&(0xff<<24);
              plccur_ch = plccur_ch>>24;
              plccur4 = plccur_ch&0xff;
              
              if(PLC_data[40] == 0xFF) {         //10^-1
                index = 0.1;
              }
              else if(PLC_data[40] == 0x00) {    //10^0
                index = 1;
              }
              else if(PLC_data[40] == 0x01) {    //10^1
                index = 10;
              }
              voldecimal = PLC_data[43]*256 + PLC_data[42];
              plcvoltage = (voldecimal*index)*1000;        //파워모듈 세팅 전압값
              plcvol1 = plcvoltage&0xff;                   //plccur_ch 자리수 저장
              plcvol_ch = plcvoltage&(0xff<<8);
              plcvol_ch = plcvol_ch>>8;
              plcvol2 = plcvol_ch&0xff;
              plcvol_ch = plcvoltage&(0xff<<16);
              plcvol_ch = plcvol_ch>>16;
              plcvol3 = plcvol_ch&0xff;
              plcvol_ch = plcvoltage&(0xff<<24);
              plcvol_ch = plcvol_ch>>24;
              plcvol4 = plcvol_ch&0xff;
            }
            else {
              car = 14;
              power_1 = 2;        //Relay Error
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Relay = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
            if(PLC_data[36] == 0xFF) {         //10^-1
              index = 0.1;
            }
            else if(PLC_data[36] == 0x00) {    //10^0
              index = 1;
            }
            else if(PLC_data[36] == 0x01) {    //10^1
              index = 10;
            }
            if(PLC_data[7] > 0) {
              car = 14;
              power_1 = 2;        //Relay Error
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault2.EV_Error = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
            else {
              //Bd_Status.Fault = 0x00;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault2.EV_Error = 0;
            }
            Remain_sec = (PLC_data[39]*256 + PLC_data[38])*index;
            Bd_Status.Remain_Time = Remain_sec/60;
            Bd_Status.Battery_SOC = PLC_data[8];
            if(PLC_data[7] != zero) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.EV_Error = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
            if(Bd_Status.Fault == 1) {
              car = 14;
              power_1 = 2;
            }
          }
          else if (PLC_data[2] == 0x09) {       //PowerDelivery
            car = 11;
            Charging_State = 11;
            if(PLC_data[4] == 0x00) {           //이게 0이면 충전 종료로 넘어가는 것
              // car = 14;
              power_1 = 2;
            }
            if(PLC_data[11] != zero) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
          }
          else if (PLC_data[2] == 0x12) {         //weldingDetection
            Charging_State = 12;
            car = 12;
            power_1 = 2;
            if(PLC_data[7] != zero) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
          }
          else if (PLC_data[2] == 0x0C) {    //충전종료
            if(Charging_State < 9) {
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
            Charging_State = 13;
            car = 13;
            power_1 = 2;
            Charging_Start = 0;
          }
        }
        if(MCTime != 0) {                       //MC제어
          if(millis() - MCTime > 2000) {
            MC85_OK = digitalRead(MC85_R);
            delay(5);
            if(MC85_OK == HIGH) {
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.MC = 1;
              car = 14;
              power_1 = 2;
              total_Fault = 2;
              FaultTime = millis();
            }
            else {
              car = 2;
              Charge_On = 1;
              plcstartTime = millis();
              Bd_Status.Plug_Check = 0x01;
            }
            MCTime = 0;
          }
          else if(millis() - MCTime > 1000) {         //MC동작
            if(Power_ON_2 == 0){                      
              digitalWrite(MC85, HIGH);
              digitalWrite(L_Relay, LOW);
            }
            else if(Power_ON_2 > 0) {
              digitalWrite(L_Relay, LOW);
            }
            Bd_Status.Plug_Check = 0x01;
            Bd_Status.Charge_Fin = 0x00;
            EVSEStatusCode = 0x01;
            ResponseCode = 0x01;
          }
        }
        if(RelayTime != 0) {                          //릴레이 동작이라고 넣어놓긴 했는데 PLC_data[2] == 10 일때도 동작 코드가 들어가있음..
          if(millis() - RelayTime > 1000) {
            RelayTime = millis();
            L_Relay_OK = digitalRead(L_Relay_R);
            // L_Relay_OK = LOW;
            // R_Relay_OK = LOW;
            delay(2);
            if(L_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
              car = 9;
              power_1 = 1;
            }
            else {
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Relay = 1;
              car = 14;         //relay error
              power_1 = 2;
              total_Fault = 2;
              FaultTime = millis();
            }
            RelayTime = 0;
          }
        }
        if(Charge_On == 1) {                                        //충전중 갑자기 뽑혔을 때?
          if(PLC_data[4] == 0x01 && PLC_data[5] == 0x0C) {
            Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault2.PLC_Error = 1;
            Bd_Status.Plug_Check = zero;
            Plug_Check = 0;
            Start = 0;              
            car = 14;
            power_1 = 2;
            start_sign = 0;
            total_Fault = 2;
            FaultTime = millis();
          }
        }
        else if(Charging_State != 0 || Charging_State < 13) {                              //이것도 뭔가 이상한데.. Charging_State가 0이여야 하지 않나,,, Charging_State <13 일 때(충전종료 전) 조건을 추가하고 에러 뱉게 함
          if(PLC_data[4] == 0x01 && PLC_data[5] == 0x0C) {
            Bd_Status.Plug_Check = zero;
            Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault2.PLC_Error = 1;
            Plug_Check = 0;
            Start = 0;           
            car = 14;
            power_1 = 2;
            start_sign = 0;
            total_Fault = 2;
            FaultTime = millis();
          }
        }
        if(EB_Status.Start == 0) {                        //혹시나의 오류 때문에 있는듯
          if(Bd_Status.Charge_Fin == 0x00) {
            if(Charging_State < 3) {
            car = 14;
            power_1 = 2;
            }
            else if(Charging_State < 9) {
              //ResponseCode = 0x04;
              car = 14;
              power_1 = 2;
            }
            else if(Charging_State < 13) {
              EVSEStatusCode = 0x02;
            }
          }
        }
        // else if(Charge_On == 0) {
        //   Bd_Status.Plug_Check = zero;
        //   Start = 0;
        //   car = 14;
        // }
      }
      else {                                        //종료통신 전에 충전루프를 빠져나올경우 여기로 대신함
        if(PLC_data[0] == 0x47) {
          if (PLC_data[2] == 0x12) {
            Charging_State = 12;
            car = 12;
            power_1 = 2;
            if(PLC_data[7] != zero) {
              car = 14;
              power_1 = 2;
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
          }
          else if (PLC_data[2] == 0x0C) {    //충전종료
            if(Charging_State < 9) {
              Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1.Pre_Charging = 1;
              total_Fault = 2;
              FaultTime = millis();
            }
            Charging_State = 13;
            car = 13;
            power_1 = 2;
            Charging_Start = 0;
          }
        }
      }
      if(Bd_Status.Charge_Run == 0x01) {
        L_Relay_OK = digitalRead(L_Relay_R);
        if(L_Relay_OK == LOW) {
          Relay_ON = 0;
        }
        else {
          Relay_ON++;
        }
        if(total_Fault == 2) {
          Relay_ON = 0;
        }
        if(Relay_ON > 10) {
          if(Charging_State != 13 || Charging_State != 12) {
            car = 14;
            power_1 = 2;        //Relay Error
            Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1.Relay = 1;
            total_Fault = 2;
            FaultTime = millis();
          }
        }
      }
      if(Charging_State == 10) {                      //충전중일때는 Charge_Fin = 0
        Bd_Status.Charge_Fin = 0x00;
      }
      if (Charge_On == 2) {                               //충전 종료
        Relay_ON = 0;
        // if(FaultTime == 0) {
        //   if(Bd_Status.Fault == 0x00) {
        //     Bd_Status.Charge_Run = 0x00;
        //     Bd_Status.Charge_Fin = 0x01;
        //   }
        // }
        // Charging_Start = 0;
        // Charging_State = 0;
        if(StopTime == 0) {
          StopTime = millis();
        }
        else if(StopTime != 0) {                        //종류 후 MC, 릴레이 제어
          if(millis() - StopTime > 3000) {
            if(Power_ON_2 == 0) {
              digitalWrite(MC85, LOW);
            }
            
            if(Button2_OK == HIGH) {
              digitalWrite(L_Relay, LOW);
            }
            Start = 1; 
            if(Power_ON == 0) {
              Start = 0;
            }
            else {
              StopTime2 = millis();
            }
            Power_ON = 0;
            Charge_On = 0;
            StopTime = 0;
            MCTime = 0;
            RelayTime = 0;
            RelayTime_test = 0;
            TestmodeTime = 0;
            MC_ON = 0;
            Charging_Start = 0;
            Charging_State = 15;
            Bd_Status.Charge_Run = 0x00;
            Relay_ON = 0;
            Bd_Status.Charge_Fin = 0x01;
            Connect_Check = millis();
          }
          // else if(millis() - StopTime > 1000) {
          //   if(FaultTime == 0) {
          //     if(Bd_Status.Fault == 0x01) {
          //       Bd_Status.Charge_Run = 0x00;
          //       Bd_Status.Charge_Fin = 0x01;
          //     }
          //   }
          // }
        }
        power = 0;
        power_1 = 0;
      }
      if(Plug_Check == 1) {                       //커넥터가 연결되어 있으면 커넥터센서 오류가 뜨지않음
        ConnTime = millis();
        Ch_Fault2.Connector_position = 0;
      }
      else if(Plug_Check == zero) {
        if(millis() - ConnTime > 35000) {
          Ch_Fault2.Connector_position = 0;
          Connector_Fault = 1;
        }
        else if(millis() - ConnTime > 15000) {
          if(Connector_Fault == 0){
            Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault2.Connector_position = 1;
            if(total_Fault == 0) {
              total_Fault = 1;
            }
            // Init_Time = millis();
          }
        }
      }
      if(EB_Status.Stop == 0x01) {
        if(Bd_Status.Charge_Fin == 0x00) {
          if(Charging_State < 3) {
            car = 14;
            power_1 = 2;
          }
          else if(Charging_State < 9) {
            //ResponseCode = 0x04;
            car = 14;
            power_1 = 2;
          }
          else if(Charging_State < 13) {
            EVSEStatusCode = 0x02;        //Current demand단계에서 이것만 바꿔주면 충전 종료시쿼스로 넘어감
          }
        }
      }
      if(Power_ON == 0) {                 //테스트용 수동 릴레이 제어
        if(Button2_OK == LOW) {
          digitalWrite(L_Relay, HIGH);
        }
        else {
          digitalWrite(L_Relay, LOW);
        }
      }
      // if(LED_data[2] == 0x01) {
      //   if(Bd_Status.Charge_Fin == 0x00) {
      //     if(Charge_On != 2 || Charge_On != 0) {
      //       car = 14;
      //       power_1 = 2;
      //     }
      //   }
      //   Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
      //   Ch_Fault2.Emergency = 1;
      //   if(total_Fault == 0) {
      //     total_Fault = 1;
      //   }
      // }
      // else 
      if(Emergency_OK == LOW) {
        if(Bd_Status.Charge_Fin == 0x00) {
          if(Charge_On != 2 || Charge_On != 0) {
            car = 14;
            power_1 = 2;
            start_sign = 2;
          }
        }
        Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
        Ch_Fault2.Emergency = 1;
        if(total_Fault == 0) {
          total_Fault = 1;
          // RFID_Reset = 1;
        }
      }
      else {
        Ch_Fault2.Emergency = 0;
      }
      if(Ch_Fault1.Power_Module == 0x01) {
        if(Bd_Status.Charge_Fin == 0x00) {
          if(Charge_On != 2 || Charge_On != 0) {
            car = 14;
            power_1 = 2;
          }
        }
      }
      if(Bd_Status.Fault == 1 || Ch_Fault1.Over_DCVoltage == 1 || Ch_Fault1.Over_DCCurrent == 1 || Ch_Fault1.Temperature == 1) {        //Fault가 나면 당연히 충전종료
        if(Bd_Status.Charge_Fin == 0x00) {
          if(Charge_On != 2 || Charge_On != 0) {
            car = 14;
            power_1 = 2;
            start_sign = 2;
          }
        }
      }
      if(Power_ON == 0 && Power_ON_2) {                     //채널별 상황에 따른 파워모듈 제어를 위한 부분
        Power_State = 0;
      }
      else if(Power_ON > 0 && Power_ON_2 == 0) {
        Power_State = 1;
      }
      else if(Power_ON == 0 && Power_ON_2 > 0) {
        Power_State = 2;
      }
      else if(Power_ON > 0 &&Power_ON_2 > 0) {
        Power_State = 3;
      }

      if((millis() - lastTime) > 100) {      //send Can data, PLC data
        lastTime = millis();
        if(Power_State == 0) {                                //파워모듈은 이 루프 안에서만 제어함, 밑에 PLC_Communication 루프에서는 제어 안함
          power = 2;
          canData();
          power = 3;        //전압, 전류 값 세팅 및 확인
          canData();
          power = 4;
          canData();
        }
        else if(Power_State == 1) {
          if(power_1 == 1) {
            power = 1;
          }
          else if(power_1 == 2) {
            power = 2;
          }
          canData();
          if((millis() - canTime) > 1000) {
            canTime = millis();
            if(power == 1) {
              power = 5;
              canData();
              power = 6;
              canData();
              setVoltage++;
              delay(5);
            }
            power = 3;        //전압, 전류 값 세팅 및 확인
            canData();
            power = 4;
            canData();
          }
          power_1 = 0;
        }
        else if(Power_State == 2) {
          if(power_2 == 1) {
            power = 1;
          }
          else if(power_2 == 2) {
            power = 2;
          }
          canData();
          if((millis() - canTime) > 1000) {
            canTime = millis();
            if(power == 1) {
              power = 5;
              canData();
              power = 6;
              canData();
              setVoltage++;
              delay(5);
            }
            power = 3;        //전압, 전류 값 세팅 및 확인
            canData();
            power = 4;
            canData();
          }
          power_2 = 0;
        }
        else if(Power_State == 3) {
          if(power_1 == 1) {
            power = 1;
          }
          else if(power_1 == 2) {
            power = 2;
          }
          canData();
          if((millis() - canTime) > 1000) {
            if(power == 1) {
              power = 5;
              canData();
              setVoltage++;
              delay(5);
            }
            power = 3;        //전압, 전류 값 세팅 및 확인
            canData();
            power = 4;
            canData();
          }
          if(power_2 == 1) {
            power = 7;
          }
          else if(power_2 == 2) {
            power = 8;
          }
          canData();
          if((millis() - canTime) > 1000) {
            canTime = millis();
            if(power == 7) {
              power = 11;
              canData();
              power = 6;
              canData();
              setVoltage++;
              delay(5);
            }
            power = 9;        //전압, 전류 값 세팅 및 확인
            canData();
          }
          power_1 = 0;
          power_2 = 0;
        }
        power = 0;
        sendCarMsg();
        car = 0;
      }
      for(int j=0;j<400;j++) {      //data 초기화
        PLC_data[j] = '\0';
      }
    
    }
    else if(EB_Status.TestMode == 0x01) {
      TestmodeTime = millis();
      ConnTime = millis();
      if(Test_Mode.Resistance_Test == 1) {
        if(Power_ON_2 > 0) {
          if(MCTime != 0) {
            if(millis() - MCTime > 2000) {
              MC85_OK = digitalRead(MC85_R);
              if(MC85_OK == HIGH) {
                //Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
                power_1 = 2;
              }
              MCTime = 0;
              RelayTime_test = millis();
            }
            else if(millis() - MCTime > 1000) {
              digitalWrite(MC85, HIGH);
            }
          }
          else if(RelayTime_test != 0) {
            if(millis() - RelayTime_test > 2000) {
              digitalWrite(L_Relay, HIGH);
              digitalWrite(M_Relay, LOW);
              RelayTime_test = 0;
              RelayTime = millis();
            }
          }
          else if(RelayTime != 0) {
            if(millis() - RelayTime > 1000) {
              L_Relay_OK = digitalRead(L_Relay_R);
              // L_Relay_OK = LOW;
              // R_Relay_OK = LOW;
              
              if(L_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
                power_1 = 1;
                power_ON = 1;
                Power_ON = 1;
                Charge_On = 1;
                powermoduleTime = millis();
              }
              else {
                power_1 = 2;
                Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
                Ch_Fault1.Relay = 1;
                total_Fault = 2;
                FaultTime = millis();
              }
              RelayTime = 0;
            }
          }
        }
        else {
          if(MCTime != 0) {
            if(millis() - MCTime > 2000) {
              MC85_OK = digitalRead(MC85_R);
              if(MC85_OK == HIGH) {
                //Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
                power_1 = 2;
              }
              MCTime = 0;
              RelayTime_test = millis();
            }
            else if(millis() - MCTime > 1000) {
              digitalWrite(MC85, HIGH);
            }
          }
          else if(RelayTime_test != 0) {
            if(millis() - RelayTime_test > 2000) {
              digitalWrite(L_Relay, HIGH);
              digitalWrite(M_Relay, LOW);
              RelayTime_test = 0;
              RelayTime = millis();
            }
          }
          else if(RelayTime != 0) {
            if(millis() - RelayTime > 1000) {
              L_Relay_OK = digitalRead(L_Relay_R);
              // L_Relay_OK = LOW;
              // R_Relay_OK = LOW;
              
              if(L_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
                power_1 = 1;
                power_ON = 1;
                Power_ON = 1;
                Charge_On = 1;
                powermoduleTime = millis();
              }
              else {
                power_1 = 2;
                Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
                Ch_Fault1.Relay = 1;
                total_Fault = 2;
                FaultTime = millis();
              }
              RelayTime = 0;
            }
          }
        }
      }
      else if(Test_Mode.False_Test == 1) {
        if(RelayTime_test != 0) {
          if(millis() - RelayTime_test > 2000) {
            digitalWrite(L_Relay, HIGH);
            RelayTime_test = 0;
            // RelayTime = millis();
            Charge_On = 1;
          }
        }
        else if(RelayTime != 0) {
          if(millis() - RelayTime > 1000) {
            //L_Relay_OK = digitalRead(L_Relay_R);
            //R_Relay_OK = digitalRead(R_Relay_R);
            L_Relay_OK = LOW;
            // if(L_Relay_OK == LOW && R_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
            //   power_1 = 1;
            //   power_ON = 1;
            //   Power_ON = 1;
            //   Charge_On = 1;
            // }
            // else {
            //   power_1 = 2;
            // }
            RelayTime = 0;
          }
        }
      }
      if(Ch_Fault1.Over_DCVoltage == 1 || Ch_Fault1.Over_DCCurrent == 1 || Ch_Fault1.Temperature == 1) {
        if(millis() - StopTime > 3500) {
          power_ON = 2; 
          delay(5);
        }
      }
      
      if(LED_data[2] == 0x01) {
        if(millis() - StopTime > 3500) {
          power_ON = 2; 
          delay(5);
        }
      }
      if((millis() - lastTime) > 100) {      //send Can data, PLC data
        lastTime = millis();
        if(power_ON == 1) {
          power = 1;
        }
        else if(power_ON == 2) {
          power = 2;
          power_ON = 0;
          Power_ON = 0;
          // Charge_On = 0;
        }
        canData();
        if((millis() - canTime) > 1000) {
          canTime = millis();
          if(power == 1) {
            power = 5;
            canData();
            power = 6;
            canData();
            delay(5);
          }
          power = 3;        //전압, 전류 값 세팅 및 확인
          canData();
          power = 4;
          canData();
          delay(10);
        }
      }
      for(int j=0;j<400;j++) {      //data 초기화
        PLC_data[j] = '\0';
      }
    }
    Can0.read(incoming);
    int decimal = 0;                  // 10진수를 저장할 변수
    int position = 0;
    int sendvoltage = 0;
    int sendcurrent = 0;
    int E=0, M=0, pos=0;
    sprintf(id, "%x", incoming.id);
    //itoa(incoming.id, id, 16);
    if(Power_ON > 0 && Charging_State > 0) {
      if(pre_id != incoming.id) {
        powermoduleTime = millis();
      }
      pre_id = incoming.id;
      if(millis() - powermoduleTime > 5000) {
        Bd_Status.Fault = 0x01;
        Ch_Fault1.Power_Module = 0x01;
        total_Fault = 2;
        FaultTime = millis();
      }
      else {
        if(FaultTime == 0) {
          Ch_Fault1.Power_Module = 0x00;
        }
      }
    }
    if(id[1] == '8' && id[2] == '1') {               //전압, 전류 읽기
      for(int k=0;k<8;k++) {
        sprintf(s3, "%2x", incoming.data.byte[k]);
        //itoa(incoming.data.byte[k], s3, 16);
        if(s3[0] == ' ') {
          s3[0] = '0';
        }
        if(k > 3) {
          strcat(powercur, s3);
        }
        else {
          strcat(powervol, s3);
        }
      }
      for (int i = strlen(powervol) - 1; i >= 0; i--)    
      {
        char ch = powervol[i];         
    
        if (ch >= 48 && ch <= 57)         
        {
            decimal += (ch - 48) * pow(16, position);
        }
        else if (ch >= 65 && ch <= 70)    
        {                                 
            decimal += (ch - (65 - 10)) * pow(16, position);
        }
        else if (ch >= 97 && ch <= 102)   
        {                                 
            decimal += (ch - (97 - 10)) * pow(16, position);
        }
        position++;
      }
      position = 0;
      for(int i=31; i>=0; i--) {
        volbin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(volbin[0] == 0) {
        for(int i=8; i>0; i--) {
          if(volbin[i] == 1) {
            E += 1<<pos;
          }
          pos++;
        }
        pos = 0;
        for(int i=sizeof(volbin)/sizeof(int)-1; i>8; i--) {
          if(volbin[i] == 1) {
            M += 1<<pos;
          }
          pos++;
        }
      }
      voltage = (1+M*pow(2,-23))*pow(2,(E-127));      //인피파워 전압계산
      //Bd_Status.Voltage = voltage;
      for (int i = strlen(powercur) - 1; i >= 0; i--)    
      {
        char ch = powercur[i];         
    
        if (ch >= 48 && ch <= 57)         
        {
            decimal += (ch - 48) * pow(16, position);
        }
        else if (ch >= 65 && ch <= 70)    
        {
            decimal += (ch - (65 - 10)) * pow(16, position);
        }
        else if (ch >= 97 && ch <= 102)   
        {
            decimal += (ch - (97 - 10)) * pow(16, position);
        }
    
        position++;
      }
      pos = 0;
      E=0, M=0;
      for(int i=31; i>=0; i--) {
        curbin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(curbin[0] == 0) {
        for(int i=8; i>0; i--) {
          if(curbin[i] == 1) {
            E += 1<<pos;
          }
          pos++;
        }
        pos = 0;
        for(int i=sizeof(curbin)/sizeof(int)-1; i>8; i--) {
          if(curbin[i] == 1) {
            M += 1<<pos;
          }
          pos++;
        }
      }
      current = (1+M*pow(2,-23))*pow(2,(E-127));      //인피파워 전류계산
      //Bd_Status.Current = current;

                                  
      sendvoltage = voltage*10;                 //파워모듈 전압 Read
      nowvol1 = sendvoltage&0xff;
      nowvol_ch = sendvoltage&(0xff<<8);
      nowvol_ch = nowvol_ch>>8;
      nowvol2 = nowvol_ch&0xff;
      nowvol1_2 = nowvol1;
      nowvol2_2 = nowvol2;

          
      sendcurrent = current*10;             //파워모듈 전류 Read
      if(current > 1000) {
        current = 0;
        sendcurrent = 0;
      }
      nowcur1 = sendcurrent&0xff;
      nowcur_ch = sendcurrent&(0xff<<8);
      nowcur_ch = nowcur_ch>>8;
      nowcur2 = nowcur_ch&0xff;
      nowcur1_2 = nowcur1;
      nowcur2_2 - nowcur2;
      // if(current < 25) {
      //   sendcurrent = 250;
      // }
      sendcurrent = (current+3)*10;
      if(Power_State == 1) {
        sendcurrent = ((maxpower*1000)/Me_Voltage)*10;
      }
      else if(Power_State == 2) {
        sendcurrent = ((maxpower_2*1000)/Me_Voltage_2)*10;
      }
      if(sendcurrent > 2000) {
        sendcurrent = 2000;
      }
      // sendcurrent = 1250;
      curlimit1 = sendcurrent&0xff;
      curlimit_ch = sendcurrent&(0xff<<8);
      curlimit_ch = curlimit_ch>>8;
      curlimit2 = curlimit_ch&0xff;
      curlimit1_2 = curlimit1;
      curlimit2_2 = curlimit2;
    }
    else if(id[1] == '8' && id[2] == '4') {    //온도읽기
      decimal = incoming.data.byte[5];
      for(int i=7; i>=0; i--) {
        modulebin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(modulebin[5] == 1) {
        Bd_Status.Fault = 0x01;     //fault
        Ch_Fault1.Power_Module = 1;
        total_Fault = 2;
        FaultTime = millis();
      }
      decimal = incoming.data.byte[6];
      for(int i=7; i>=0; i--) {
        modulebin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(modulebin[4] == 1 || modulebin[6] == 1) {
        Bd_Status.Fault = 0x01;     //fault
        Ch_Fault1.Power_Module = 1;
        total_Fault = 2;
        FaultTime = millis();
      }
      decimal = incoming.data.byte[7];
      for(int i=7; i>=0; i--) {
        modulebin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(modulebin[7] == 1) {
        Bd_Status.Fault = 0x01;     //fault
        Ch_Fault1.Short = 1;
        total_Fault = 2;
        FaultTime = millis();
      }
    }
    else if(id[1] == 'c' && id[2] == '1' && id[4] == '1') {               //전압, 전류 읽기
      for(int k=0;k<8;k++) {
        sprintf(s3, "%2x", incoming.data.byte[k]);
        //itoa(incoming.data.byte[k], s3, 16);
        if(s3[0] == ' ') {
          s3[0] = '0';
        }
        if(k > 3) {
          strcat(powercur, s3);
        }
        else {
          strcat(powervol, s3);
        }
      }
      for (int i = strlen(powervol) - 1; i >= 0; i--)    
      {
        char ch = powervol[i];         
    
        if (ch >= 48 && ch <= 57)         
        {
            decimal += (ch - 48) * pow(16, position);
        }
        else if (ch >= 65 && ch <= 70)    
        {                                 
            decimal += (ch - (65 - 10)) * pow(16, position);
        }
        else if (ch >= 97 && ch <= 102)   
        {                                 
            decimal += (ch - (97 - 10)) * pow(16, position);
        }
        position++;
      }
      position = 0;
      for(int i=31; i>=0; i--) {
        volbin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(volbin[0] == 0) {
        for(int i=8; i>0; i--) {
          if(volbin[i] == 1) {
            E += 1<<pos;
          }
          pos++;
        }
        pos = 0;
        for(int i=sizeof(volbin)/sizeof(int)-1; i>8; i--) {
          if(volbin[i] == 1) {
            M += 1<<pos;
          }
          pos++;
        }
      }
      voltage = (1+M*pow(2,-23))*pow(2,(E-127));      //인피파워 전압계산
      //Bd_Status.Voltage = voltage;
      for (int i = strlen(powercur) - 1; i >= 0; i--)    
      {
        char ch = powercur[i];         
    
        if (ch >= 48 && ch <= 57)         
        {
            decimal += (ch - 48) * pow(16, position);
        }
        else if (ch >= 65 && ch <= 70)    
        {
            decimal += (ch - (65 - 10)) * pow(16, position);
        }
        else if (ch >= 97 && ch <= 102)   
        {
            decimal += (ch - (97 - 10)) * pow(16, position);
        }
    
        position++;
      }
      pos = 0;
      E=0, M=0;
      for(int i=31; i>=0; i--) {
        curbin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(curbin[0] == 0) {
        for(int i=8; i>0; i--) {
          if(curbin[i] == 1) {
            E += 1<<pos;
          }
          pos++;
        }
        pos = 0;
        for(int i=sizeof(curbin)/sizeof(int)-1; i>8; i--) {
          if(curbin[i] == 1) {
            M += 1<<pos;
          }
          pos++;
        }
      }
      current = (1+M*pow(2,-23))*pow(2,(E-127));      //인피파워 전류계산
      //Bd_Status.Current = current;

                                  
      sendvoltage = voltage*10;                 //파워모듈 전압 Read
      nowvol1 = sendvoltage&0xff;
      nowvol_ch = sendvoltage&(0xff<<8);
      nowvol_ch = nowvol_ch>>8;
      nowvol2 = nowvol_ch&0xff;

          
      sendcurrent = current*10;             //파워모듈 전류 Read
      if(current > 1000) {
        current = 0;
        sendcurrent = 0;
      }
      nowcur1 = sendcurrent&0xff;
      nowcur_ch = sendcurrent&(0xff<<8);
      nowcur_ch = nowcur_ch>>8;
      nowcur2 = nowcur_ch&0xff;
      // if(current < 25) {
      //   sendcurrent = 250;
      // }
      sendcurrent = (current+3)*10;
      sendcurrent = ((maxpower*1000)/Me_Voltage)*10;
      // sendcurrent = 1250;
      if(sendcurrent > 1100) {
        sendcurrent = 1100;
      }
      curlimit1 = sendcurrent&0xff;
      curlimit_ch = sendcurrent&(0xff<<8);
      curlimit_ch = curlimit_ch>>8;
      curlimit2 = curlimit_ch&0xff;
    }
    else if(id[1] == 'c' && id[2] == '1' && id[4] == '2') {               //전압, 전류 읽기
      for(int k=0;k<8;k++) {
        sprintf(s3, "%2x", incoming.data.byte[k]);
        //itoa(incoming.data.byte[k], s3, 16);
        if(s3[0] == ' ') {
          s3[0] = '0';
        }
        if(k > 3) {
          strcat(powercur, s3);
        }
        else {
          strcat(powervol, s3);
        }
      }
      for (int i = strlen(powervol) - 1; i >= 0; i--)    
      {
        char ch = powervol[i];         
    
        if (ch >= 48 && ch <= 57)         
        {
            decimal += (ch - 48) * pow(16, position);
        }
        else if (ch >= 65 && ch <= 70)    
        {                                 
            decimal += (ch - (65 - 10)) * pow(16, position);
        }
        else if (ch >= 97 && ch <= 102)   
        {                                 
            decimal += (ch - (97 - 10)) * pow(16, position);
        }
        position++;
      }
      position = 0;
      for(int i=31; i>=0; i--) {
        volbin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(volbin[0] == 0) {
        for(int i=8; i>0; i--) {
          if(volbin[i] == 1) {
            E += 1<<pos;
          }
          pos++;
        }
        pos = 0;
        for(int i=sizeof(volbin)/sizeof(int)-1; i>8; i--) {
          if(volbin[i] == 1) {
            M += 1<<pos;
          }
          pos++;
        }
      }
      voltage = (1+M*pow(2,-23))*pow(2,(E-127));      //인피파워 전압계산
      //Bd_Status.Voltage = voltage;
      for (int i = strlen(powercur) - 1; i >= 0; i--)    
      {
        char ch = powercur[i];         
    
        if (ch >= 48 && ch <= 57)         
        {
            decimal += (ch - 48) * pow(16, position);
        }
        else if (ch >= 65 && ch <= 70)    
        {
            decimal += (ch - (65 - 10)) * pow(16, position);
        }
        else if (ch >= 97 && ch <= 102)   
        {
            decimal += (ch - (97 - 10)) * pow(16, position);
        }
    
        position++;
      }
      pos = 0;
      E=0, M=0;
      for(int i=31; i>=0; i--) {
        curbin[i] = decimal%2;
        decimal = decimal/2;
      }
      if(curbin[0] == 0) {
        for(int i=8; i>0; i--) {
          if(curbin[i] == 1) {
            E += 1<<pos;
          }
          pos++;
        }
        pos = 0;
        for(int i=sizeof(curbin)/sizeof(int)-1; i>8; i--) {
          if(curbin[i] == 1) {
            M += 1<<pos;
          }
          pos++;
        }
      }
      current = (1+M*pow(2,-23))*pow(2,(E-127));      //인피파워 전류계산
      //Bd_Status.Current = current;

                                  
      sendvoltage = voltage*10;                 //파워모듈 전압 Read
      nowvol1_2 = sendvoltage&0xff;
      nowvol_ch_2 = sendvoltage&(0xff<<8);
      nowvol_ch_2 = nowvol_ch_2>>8;
      nowvol2_2 = nowvol_ch_2&0xff;

          
      sendcurrent = current*10;             //파워모듈 전류 Read
      if(current > 1000) {
        current = 0;
        sendcurrent = 0;
      }
      nowcur1_2 = sendcurrent&0xff;
      nowcur_ch_2 = sendcurrent&(0xff<<8);
      nowcur_ch_2 = nowcur_ch_2>>8;
      nowcur2_2 = nowcur_ch_2&0xff;
      // if(current < 25) {
      //   sendcurrent = 250;
      // }
      sendcurrent = (current+3)*10;
      sendcurrent = ((maxpower_2*1000)/Me_Voltage_2)*10;
      // sendcurrent = 1250;
      if(sendcurrent > 1100) {
        sendcurrent = 1100;
      }
      curlimit1_2 = sendcurrent&0xff;
      curlimit_ch_2 = sendcurrent&(0xff<<8);
      curlimit_ch_2 = curlimit_ch_2>>8;
      curlimit2_2 = curlimit_ch_2&0xff;
    }
    delay(2);
    if(door_OK == HIGH){
      Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
      Ch_Fault2.Door_Open = 1;
      if(total_Fault == 0) {
        total_Fault = 1;
      }
      Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
      Ch_Fault2_2.Door_Open = 1;
      if(total_Fault_2 == 0) {
        total_Fault_2 = 1;
      }
    }
    else {
      Ch_Fault2.Door_Open = 0;
      Ch_Fault2_2.Door_Open = 0;
    }
    if(Conn_OK == LOW) {
      Bd_Status.Plug_Check = 0;
      Ch_Fault2.Connector_position = 0;
      Plug_Check = 0;
      ConnTime = millis();
      StopTime2 = 0;
      Start=0;
      // Bd_Status.Charge_Fin = 0x01;
    }
    // ConnTime = millis();
    if(millis() - TemperTime > 1000) {
      TemperTime = millis();
      T_Sensor = analogRead(A10);
      T_Voltage = T_Sensor*(3.3/4095.0);
      R_thermistor = (62000*2000*T_Voltage)/(62000*3.3-64000*T_Voltage);
      T_temperature = (1023630-(937755+(81354*log(R_thermistor/10000))))/((298*log(R_thermistor/10000))+3435);
      T_temperature_10 = T_temperature * 10;
      if(T_temperature > 75) {
        Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
        Ch_Fault1.Temperature = 1;
        total_Fault = 2;
        FaultTime = millis();
        Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
        Ch_Fault1_2.Temperature = 1;
        total_Fault_2 = 2;
        FaultTime_2 = millis();
        // if(total_Fault == 0) {
        //   total_Fault = 1;
        // }
      }
      else {
        if(FaultTime == 0) {
          Ch_Fault1.Temperature = 0;
        }
        if(FaultTime_2 == 0) {
          Ch_Fault1_2.Temperature = 0;
        }
      }
    }


    int StAddr = 0;
    int Regi_Count = 0;
    if(Bd_Status.Fault == 0x01) {
      PLC_Reset = 0;
    }
    if(Bd_Status_2.Fault == 0x01) {
      PLC_Reset_2 = 0;
    }
    if(Serial1.available()) {
      if(total_Fault == 0 && FaultTime == 0) {
        Bd_Status.Fault = 0x00;
        FaultTime = 0;
      }
      if(total_Fault_2 == 0 && FaultTime_2 == 0) {
        Bd_Status_2.Fault = 0x00;
        FaultTime_2 = 0;
      }
      PC_Read();
      if(PC_data[1] == 0x10) {
        CRC_Check[0] = CRC16(Check_data, PC_data[6]+7);
        Check_data[PC_data[6]+7] = PC_data[PC_data[6]+7];
        CRC_Check[1] = CRC16(Check_data, PC_data[6]+8);
        if(CRC_Check[0] == PC_data[PC_data[6]+7] && CRC_Check[1] == PC_data[PC_data[6]+8]) {
          CRC_OK = 1;
        }
        else {
          CRC_OK = 0;
        }
        if(CRC_OK == 1) {
          StAddr = (PC_data[2]*256) + PC_data[3];
          Regi_Count = (PC_data[4]*256) + PC_data[5];
          for(int j=0; j<Regi_Count; j++) {
            for(int k=0; k<2; k++) {
              address[StAddr+j][k] = PC_data[7+(2*j)+k];
              if(StAddr+j == 200) {
                if(k == 0 ) {
                  EB_Stat= PC_data[7+2*j];
                  for(int m=8; m<16; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                }
                else {
                  EB_Stat= PC_data[8+2*j];
                  for(int m=0; m<8; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                  EB_Status.Standby = EB_Array[0];
                  EB_Status.Ready = EB_Array[1];
                  EB_Status.Run = EB_Array[2];
                  EB_Status.Start = EB_Array[3];
                  EB_Status.Stop = EB_Array[4];
                  EB_Status.Door_Open = EB_Array[6];
                  EB_Status.Fault = EB_Array[7];
                  EB_Status.TestMode = EB_Array[9];
                  EB_Status.ResetReq = EB_Array[11];
                  if(EB_Status.Stop == 1) {
                    EB_Status.Start = 0;
                  }
                }
              }
              else if(StAddr+j == 202){
                if(k == 0 ) {
                  EB_Stat= PC_data[7+2*j];
                  for(int m=8; m<16; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                }
                else {
                  EB_Stat= PC_data[8+2*j];
                  for(int m=0; m<8; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                  Test_Mode.Power_Value = PC_data[8+2*j];
                  Test_Mode.Order_bit = EB_Array[15];
                }
              }
              else if(StAddr+j == 203) {
                EB_Status.Run_Count[0] = PC_data[7+2*j];
                EB_Status.Run_Count[1] = PC_data[8+2*j];
              }
              else if(StAddr+j == 204){
                if(k == 0 ) {
                  EB_Stat= PC_data[7+2*j];
                  for(int m=8; m<16; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                }
                else {
                  EB_Stat= PC_data[8+2*j];
                  for(int m=0; m<8; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                  if(PC_data[8+2*j] == 0x04) {
                    Test_Mode.False_Test = 1;
                    Test_Mode.Resistance_Test = 0;
                  }
                  else if(PC_data[8+2*j] == 0x08) {
                    Test_Mode.Resistance_Test = 1;
                    Test_Mode.False_Test = 0;
                  }
                }
              }
              else if(StAddr+j == 206) {
                Test_Mode.Set_Voltage = ((PC_data[7+2*j]*256) + PC_data[8+2*j])/10;
              }
              else if(StAddr+j == 208) {
                Test_Mode.Set_Current = ((PC_data[7+2*j]*256) + PC_data[8+2*j])/10;
              }
              else if(StAddr+j == 209) {
                EB_Status.Protocol_Ver[0] = PC_data[7+2*j];
                EB_Status.Protocol_Ver[1] = PC_data[8+2*j];
              }
              if(StAddr+j == 300) {
                if(k == 0 ) {
                  EB_Stat= PC_data[7+2*j];
                  for(int m=8; m<16; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                }
                else {
                  EB_Stat= PC_data[8+2*j];
                  for(int m=0; m<8; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                  EB_Status_2.Standby = EB_Array[0];
                  EB_Status_2.Ready = EB_Array[1];
                  EB_Status_2.Run = EB_Array[2];
                  EB_Status_2.Start = EB_Array[3];
                  EB_Status_2.Stop = EB_Array[4];
                  EB_Status_2.Door_Open = EB_Array[6];
                  EB_Status_2.Fault = EB_Array[7];
                  EB_Status_2.TestMode = EB_Array[9];
                  EB_Status_2.ResetReq = EB_Array[11];
                  if(EB_Status_2.Stop == 1) {
                    EB_Status_2.Start = 0;
                  }
                }
              }
              else if(StAddr+j == 302){
                if(k == 0 ) {
                  EB_Stat= PC_data[7+2*j];
                  for(int m=8; m<16; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                }
                else {
                  EB_Stat= PC_data[8+2*j];
                  for(int m=0; m<8; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                  Test_Mode_2.Power_Value = PC_data[8+2*j];
                  Test_Mode_2.Order_bit = EB_Array[15];
                }
              }
              else if(StAddr+j == 303) {
                EB_Status_2.Run_Count[0] = PC_data[7+2*j];
                EB_Status_2.Run_Count[1] = PC_data[8+2*j];
              }
              else if(StAddr+j == 304){
                if(k == 0 ) {
                  EB_Stat= PC_data[7+2*j];
                  for(int m=8; m<16; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                }
                else {
                  EB_Stat= PC_data[8+2*j];
                  for(int m=0; m<8; m++) {
                    EB_Array[m] = EB_Stat%2;
                    EB_Stat = EB_Stat/2; 
                  }
                  if(PC_data[8+2*j] == 0x04) {
                    Test_Mode_2.False_Test = 1;
                    Test_Mode_2.Resistance_Test = 0;
                  }
                  else if(PC_data[8+2*j] == 0x08) {
                    Test_Mode_2.Resistance_Test = 1;
                    Test_Mode_2.False_Test = 0;
                  }
                }
              }
              else if(StAddr+j == 306) {
                Test_Mode_2.Set_Voltage = ((PC_data[7+2*j]*256) + PC_data[8+2*j])/10;
              }
              else if(StAddr+j == 308) {
                Test_Mode_2.Set_Current = ((PC_data[7+2*j]*256) + PC_data[8+2*j])/10;
              }
              else if(StAddr+j == 309) {
                EB_Status_2.Protocol_Ver[0] = PC_data[7+2*j];
                EB_Status_2.Protocol_Ver[1] = PC_data[8+2*j];
              }
            }
          }
          for(int j=0; j<6; j++) {
            Write_res[j] = PC_data[j];
          }
          Write_res[6] = CRC16(Write_res, 6);
          Write_res[7] = CRC16(Write_res, 7);
        }
        else {
          Write_res[0] = PC_data[0];
          Write_res[1] = PC_data[1];
          Write_res[2] = zero;
          Write_res[3] = CRC16(Write_res, 3);
          Write_res[4] = CRC16(Write_res, 4);
        }
        Write_Response();
        // for(int j=0; j<421; j++) {
        //   for(int k=0; k<2; k++) {
        //     Serial1.write(address[j][k]);
        //   }
        // }
        // delay(100);
      }
      else if(PC_data[1] == 0x04) {
        CRC_Check[0] = CRC16(Check_data, 6);
        Check_data[6] = PC_data[6];
        CRC_Check[1] = CRC16(Check_data, 7);
        if(CRC_Check[0] == PC_data[6] && CRC_Check[0] == PC_data[6]) {
          CRC_OK = 1;
        }
        else {
          CRC_OK = 0;
        }
        if(CRC_OK == 1) {
          Read_res[0] = PC_data[0];
          Read_res[1] = PC_data[1];
          Read_res[2] = PC_data[5]*2;
          extract_data = PC_data[2]*256+PC_data[3];
          for(int j=0; j<PC_data[5]; j++) {
            for(int k=0; k<2; k++) {
              if(extract_data+j == 400) {
                address[extract_data+j][0] = Bd_Status.Connector_Lock*2;
                address[extract_data+j][1] = Bd_Status.Board_Ready+Bd_Status.Charge_Ready*2+
                                            Bd_Status.Plug_Check*4+Bd_Status.Door_Check*8+
                                            Bd_Status.Charge_Run*16+Bd_Status.Charge_Fin*32+
                                            Bd_Status.Fault*64+Bd_Status.Reset_Status*128;
                // address[extract_data+j][0] = zero;
                // address[extract_data+j][1] = zero;
              }
              else if(extract_data+j == 401) {
                address[extract_data+j][0] = Bd_Status.Board_Ver[0];
                address[extract_data+j][1] = Bd_Status.Board_Ver[1];
              }
              else if(extract_data+j == 402) {
                address[extract_data+j][0] = EB_Status.Protocol_Ver[0];
                address[extract_data+j][1] = EB_Status.Protocol_Ver[1];
              }
              else if(extract_data+j == 403) {
                address[extract_data+j][0] = EB_Status.Run_Count[0];
                address[extract_data+j][1] = EB_Status.Run_Count[1];
              }
              else if(extract_data+j == 404) {
                address[extract_data+j][0] = Bd_Status.Remain_Time/256;
                address[extract_data+j][1] = Bd_Status.Remain_Time%256;
              }
              else if(extract_data+j == 405) {
                address[extract_data+j][0] = 0;
                address[extract_data+j][1] = Bd_Status.Battery_SOC;
              }
              else if(extract_data+j == 406) {
                address[extract_data+j][0] = (Bd_Status.Energy*100)/16777216;
                address[extract_data+j][1] = ((Bd_Status.Energy*100)-(address[406][0]*16777216))/65536;
              }
              else if(extract_data+j == 407) {
                address[extract_data+j][0] = ((Bd_Status.Energy*100)-(address[406][0]*16777216)-
                                            (address[406][1]*65536))/256;
                address[extract_data+j][1] = (Bd_Status.Energy*100)-(address[406][0]*16777216)-
                                            (address[406][1]*65536)-(address[407][0]*256);
              }
              else if(extract_data+j == 408) {
                address[extract_data+j][0] = (Bd_Status.Voltage*10)/256;
                address[extract_data+j][1] = (Bd_Status.Voltage*10)-(address[408][0]*256);
              }
              else if(extract_data+j == 409) {
                address[extract_data+j][0] = (Bd_Status.Current*10)/256;
                address[extract_data+j][1] = (Bd_Status.Current*10)-(address[409][0]*256);
              }
              else if(extract_data+j == 410) {
                address[extract_data+j][0] = 0;
                address[extract_data+j][1] = 0;       //얘만 변경
              }
              else if(extract_data+j == 411) {
                address[extract_data+j][0] = 0;
                address[extract_data+j][1] = 0;
              }
              else if(extract_data+j == 418) {
                address[extract_data+j][0] = Bd_Status.DC_MeterVer[0];
                address[extract_data+j][1] = Bd_Status.DC_MeterVer[1];
              }
              else if(extract_data+j == 419) {
                address[extract_data+j][0] = Ch_Fault1.Over_DCCurrent +Ch_Fault1.Short*2+
                                            Ch_Fault1.Pre_Charging*4+Ch_Fault1.Power_Module*8+
                                            Ch_Fault1.Temperature*16;
                address[extract_data+j][1] = Ch_Fault1.Main_Switch +Ch_Fault1.Over_ACVoltage*2+
                                            Ch_Fault1.Low_ACVoltage*4+Ch_Fault1.Over_ACCurrent*8+
                                            Ch_Fault1.Ground*16+Ch_Fault1.MC*32+
                                            Ch_Fault1.Relay*64+Ch_Fault1.Over_DCVoltage*128;
              }
              else if(extract_data+j == 420) {  
                address[extract_data+j][0] = Ch_Fault2.Connector_position +Ch_Fault2.DC_Meter*2+
                                            Ch_Fault2.Door_Open*4+Ch_Fault2.the_others*128;
                address[extract_data+j][1] = Ch_Fault2.CP_Error +Ch_Fault2.EV_Error*2+
                                            Ch_Fault2.PLC_Error*4+Ch_Fault2.CAN_Error*8+
                                            Ch_Fault2.Control_Board*16+Ch_Fault2.Car_Error*32+
                                            Ch_Fault2.Emergency*64+Ch_Fault2.Connector_Lock*128;
              }
              if(extract_data+j == 500) {
                address[extract_data+j][0] = Bd_Status_2.Connector_Lock*2;
                address[extract_data+j][1] = Bd_Status_2.Board_Ready+Bd_Status_2.Charge_Ready*2+
                                            Bd_Status_2.Plug_Check*4+Bd_Status_2.Door_Check*8+
                                            Bd_Status_2.Charge_Run*16+Bd_Status_2.Charge_Fin*32+
                                            Bd_Status_2.Fault*64+Bd_Status_2.Reset_Status*128;
                address[extract_data+j][0] = zero;
                address[extract_data+j][1] = zero;
              }
              else if(extract_data+j == 501) {
                address[extract_data+j][0] = Bd_Status_2.Board_Ver[0];
                address[extract_data+j][1] = Bd_Status_2.Board_Ver[1];
              }
              else if(extract_data+j == 502) {
                address[extract_data+j][0] = EB_Status_2.Protocol_Ver[0];
                address[extract_data+j][1] = EB_Status_2.Protocol_Ver[1];
              }
              else if(extract_data+j == 503) {
                address[extract_data+j][0] = EB_Status_2.Run_Count[0];
                address[extract_data+j][1] = EB_Status_2.Run_Count[1];
              }
              else if(extract_data+j == 504) {
                address[extract_data+j][0] = Bd_Status_2.Remain_Time/256;
                address[extract_data+j][1] = Bd_Status_2.Remain_Time%256;
              }
              else if(extract_data+j == 505) {
                address[extract_data+j][0] = 0;
                address[extract_data+j][1] = Bd_Status_2.Battery_SOC;
              }
              else if(extract_data+j == 506) {
                address[extract_data+j][0] = (Bd_Status_2.Energy*100)/16777216;
                address[extract_data+j][1] = ((Bd_Status_2.Energy*100)-(address[506][0]*16777216))/65536;
              }
              else if(extract_data+j == 507) {
                address[extract_data+j][0] = ((Bd_Status_2.Energy*100)-(address[507][0]*16777216)-
                                            (address[506][1]*65536))/256;
                address[extract_data+j][1] = (Bd_Status_2.Energy*100)-(address[506][0]*16777216)-
                                            (address[506][1]*65536)-(address[507][0]*256);
              }
              else if(extract_data+j == 508) {
                address[extract_data+j][0] = (Bd_Status_2.Voltage*10)/256;
                address[extract_data+j][1] = (Bd_Status_2.Voltage*10)-(address[508][0]*256);
              }
              else if(extract_data+j == 509) {
                address[extract_data+j][0] = (Bd_Status_2.Current*10)/256;
                address[extract_data+j][1] = (Bd_Status_2.Current*10)-(address[509][0]*256);
              }
              else if(extract_data+j == 510) {
                address[extract_data+j][0] = 0;
                address[extract_data+j][1] = 0;       //얘만 변경
              }
              else if(extract_data+j == 511) {
                address[extract_data+j][0] = 0;
                address[extract_data+j][1] = 0;
              }
              else if(extract_data+j == 518) {
                address[extract_data+j][0] = Bd_Status_2.DC_MeterVer[0];
                address[extract_data+j][1] = Bd_Status_2.DC_MeterVer[1];
              }
              else if(extract_data+j == 519) {
                address[extract_data+j][0] = Ch_Fault1_2.Over_DCCurrent +Ch_Fault1_2.Short*2+
                                            Ch_Fault1_2.Pre_Charging*4+Ch_Fault1_2.Power_Module*8+
                                            Ch_Fault1_2.Temperature*16;
                address[extract_data+j][1] = Ch_Fault1_2.Main_Switch +Ch_Fault1_2.Over_ACVoltage*2+
                                            Ch_Fault1_2.Low_ACVoltage*4+Ch_Fault1_2.Over_ACCurrent*8+
                                            Ch_Fault1_2.Ground*16+Ch_Fault1_2.MC*32+
                                            Ch_Fault1_2.Relay*64+Ch_Fault1_2.Over_DCVoltage*128;
              }
              else if(extract_data+j == 520) {  
                address[extract_data+j][0] = Ch_Fault2_2.Connector_position +Ch_Fault2_2.DC_Meter*2+
                                            Ch_Fault2_2.Door_Open*4+Ch_Fault2_2.the_others*128;
                address[extract_data+j][1] = Ch_Fault2_2.CP_Error +Ch_Fault2_2.EV_Error*2+
                                            Ch_Fault2_2.PLC_Error*4+Ch_Fault2_2.CAN_Error*8+
                                            Ch_Fault2_2.Control_Board*16+Ch_Fault2_2.Car_Error*32+
                                            Ch_Fault2_2.Emergency*64+Ch_Fault2_2.Connector_Lock*128;
              }
              Read_res[3+(2*j)+k] = address[extract_data+j][k];
            }
          }
          Read_res[Read_res[2]+3] = CRC16(Read_res, Read_res[2]+3);
          Read_res[Read_res[2]+4] = CRC16(Read_res, Read_res[2]+4);
        }
        else {
          Read_res[0] = PC_data[0];
          Read_res[1] = PC_data[1];
          Read_res[2] = zero;
          Read_res[3] = CRC16(Write_res, 3);
          Read_res[4] = CRC16(Write_res, 4);
        }
        Read_Response();
        if(FaultTime == 0) {
          total_Fault = 0;
        }
        else if(millis() - FaultTime > 6000) {
          total_Fault = 0;
          FaultTime = 0;
          Ch_Fault2.PLC_Error = 0;
          Ch_Fault1.MC = 0;
          Ch_Fault2.EV_Error = 0;
          Ch_Fault1.Relay = 0;
          Ch_Fault2.Connector_position = 0;
          Ch_Fault1.Pre_Charging = 0;
          Ch_Fault1.Power_Module = 0;
          Bd_Status.Charge_Run = 0x00;
          // Bd_Status.Charge_Fin = 0x01;
        }
        if(FaultTime_2 == 0) {
          total_Fault_2 = 0;
        }
        else if(millis() - FaultTime_2 > 6000) {
          total_Fault_2 = 0;
          FaultTime_2 = 0;
          Ch_Fault2_2.PLC_Error = 0;
          Ch_Fault1_2.MC = 0;
          Ch_Fault2_2.EV_Error = 0;
          Ch_Fault1_2.Relay = 0;
          Ch_Fault2_2.Connector_position = 0;
          Ch_Fault1_2.Pre_Charging = 0;
          Ch_Fault1_2.Power_Module = 0;
          Bd_Status_2.Charge_Run = 0x00;
          // Bd_Status.Charge_Fin = 0x01;
        }
      }
    }
    if(EB_Status.ResetReq == 0x01) {
      digitalWrite(Panel_ON, LOW);
      delay(2000);
      digitalWrite(Panel_ON, HIGH);
      EB_Status.ResetReq = 0;
    }
    if(EB_Status_2.ResetReq == 0x01) {
      digitalWrite(Panel_ON, LOW);
      delay(2000);
      digitalWrite(Panel_ON, HIGH);
      EB_Status_2.ResetReq = 0;
    }
    
    //delay(10);
  }
}
void Serial0_485() {
  
  if(modbus_set == 0) { 
    delay(500);
  }
  else {
    delay(20);
    if(Serial3.available()) {
      RS485_Read();
      delay(3);
      if (DCdata[1] == 0x03) {
        DC_decimal = (DCdata[3] * 16777216) + (DCdata[4] * 65536) + (DCdata[5] * 256) + DCdata[6];
        delay(1);
        for(int i=31; i>=0; i--) {
          DC_bin[i] = DC_decimal%2;
          DC_decimal = DC_decimal/2;
        }
        if(DC_bin[0] == 0) {
          for(int i=8; i>0; i--) {
            if(DC_bin[i] == 1) {
              DC_E += 1<<DC_pos;
            }
            DC_pos++;
          }
          DC_pos = 0;
          for(int i=sizeof(DC_bin)/sizeof(int)-1; i>8; i--) {
            if(DC_bin[i] == 1) {
              DC_M += 1<<DC_pos;
            }
            DC_pos++;
          }
          DC_pos = 0;
        }
        if(DCmeter == 1) {
          Me_Voltage = (1+DC_M*pow(2,-23))*pow(2,(DC_E-127));      //인피파워 전압계산
          if(Me_Voltage > 2000) {         //DC 오류방지
            Me_Voltage = voltage;
          }
          if(Me_Voltage > 1100) {           //출력 과전압
            Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1.Over_DCVoltage = 1;
            total_Fault = 2;
            FaultTime = millis();
          //  if(total_Fault == 0) {
          //    total_Fault = 1;
          //  }
          }
          else {
            if(FaultTime == 0) {
              Ch_Fault1.Over_DCVoltage = 0;
            }
          }
          Bd_Status.Voltage = Me_Voltage;
          DCTime = millis();
        }
        else if(DCmeter == 2) {
          Me_Current = (1+DC_M*pow(2,-23))*pow(2,(DC_E-127));      //인피파워 전압계산
          if(Me_Current > 500) {        //DC 오류방지
            Me_Current = current;
          }
          if(Me_Current > 115) {         //출력 과전류
            Bd_Status.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1.Over_DCCurrent = 1;
            total_Fault = 2;
            FaultTime = millis();
          //  if(total_Fault == 0) {
          //    total_Fault = 1;
          //  }
          }
          else {
            if(FaultTime == 0) {
              Ch_Fault1.Over_DCCurrent = 0;
            }
          }
          Bd_Status.Current = Me_Current;
          DCTime = millis();
        }
        else if(DCmeter == 3) {
          Energy = (1+DC_M*pow(2,-23))*pow(2,(DC_E-127));      //인피파워 전압계산
          if(abs(Energy-Previous_Energy) > 0.3 && Previous_Energy != 0) {
            Energy = Previous_Energy;
          }
          else {
            Previous_Energy = Energy;
          }
          Bd_Status.Energy = Energy;
          DCTime = millis();
        }
        DC_E = 0; DC_M = 0;
        DC_Error = 0;
      }
      if (DCdata_2[1] == 0x03) {
        DC_decimal = (DCdata_2[3] * 16777216) + (DCdata_2[4] * 65536) + (DCdata_2[5] * 256) + DCdata_2[6];
        delay(1);
        for(int i=31; i>=0; i--) {
          DC_bin[i] = DC_decimal%2;
          DC_decimal = DC_decimal/2;
        }
        if(DC_bin[0] == 0) {
          for(int i=8; i>0; i--) {
            if(DC_bin[i] == 1) {
              DC_E += 1<<DC_pos;
            }
            DC_pos++;
          }
          DC_pos = 0;
          for(int i=sizeof(DC_bin)/sizeof(int)-1; i>8; i--) {
            if(DC_bin[i] == 1) {
              DC_M += 1<<DC_pos;
            }
            DC_pos++;
          }
          DC_pos = 0;
        }
        if(DCmeter == 1) {
          Me_Voltage_2 = (1+DC_M*pow(2,-23))*pow(2,(DC_E-127));      //인피파워 전압계산
          if(Me_Voltage_2 > 2000) {         //DC 오류방지
            Me_Voltage_2 = voltage;
          }
          if(Me_Voltage_2 > 1100) {           //출력 과전압
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Over_DCVoltage = 1;
            total_Fault_2 = 2;
            FaultTime_2 = millis();
          //  if(total_Fault == 0) {
          //    total_Fault = 1;
          //  }
          }
          else {
            if(FaultTime_2 == 0) {
              Ch_Fault1_2.Over_DCVoltage = 0;
            }
          }
          Bd_Status_2.Voltage = Me_Voltage_2;
          DCTime = millis();
        }
        else if(DCmeter == 2) {
          Me_Current_2 = (1+DC_M*pow(2,-23))*pow(2,(DC_E-127));      //인피파워 전압계산
          if(Me_Current_2 > 500) {        //DC 오류방지
            Me_Current_2 = current;
          }
          if(Me_Current_2 > 115) {         //출력 과전류
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Over_DCCurrent = 1;
            total_Fault_2 = 2;
            FaultTime_2 = millis();
          //  if(total_Fault == 0) {
          //    total_Fault = 1;
          //  }
          }
          else {
            if(FaultTime_2 == 0) {
              Ch_Fault1_2.Over_DCCurrent = 0;
            }
          }
          Bd_Status_2.Current = Me_Current_2;
          DCTime = millis();
        }
        else if(DCmeter == 3) {
          Energy_2 = (1+DC_M*pow(2,-23))*pow(2,(DC_E-127));      //인피파워 전압계산
          if(abs(Energy_2-Previous_Energy_2) > 0.3 && Previous_Energy_2 != 0) {
            Energy_2 = Previous_Energy_2;
          }
          else {
            Previous_Energy_2 = Energy_2;
          }
          Bd_Status_2.Energy = Energy_2;
          DCTime = millis();
        }
        DC_E = 0; DC_M = 0;
        DC_Error = 0;
      }
    }
    if(millis()-DCTime2 < 500) {
      if(DCTime != 0) {
        if(millis() - DCTime > 100) {
          for(int j=1;j<100;j++) {
            DCdata[j] = '\0';
            DCdata_2[j] = '\0';
          }
          if(DCmeter == 1) {
            DCmeter = 2;
          }
          else if(DCmeter == 2) {
            DCmeter = 3;
          }
          else if(DCmeter == 3) {
            DCmeter = 1;
          }
          delay(2);
          DC_Count = 0;
          DC_Write();
          EOCR_Read = 0;
          DCTime = 0;
          DCTime2 = millis();
        }
      }
    }
    else {
      if(EOCR_Read == 0) {
        for(int j=1;j<100;j++) {
          DCdata[j] = '\0';
          DCdata_2[j] = '\0';
        }
        if(DCmeter == 1) {
          DCmeter = 2;
        }
        else if(DCmeter == 2) {
          DCmeter = 3;
        }
        else if(DCmeter == 3) {
          DCmeter = 1;
        }
      }
      delay(2);
      if(DC_Error > 50) {
        Bd_Status.Fault = 0x01;
        Ch_Fault2.DC_Meter = 1;
        total_Fault = 2;
        FaultTime = millis();
        DCmeter = 1;
      }
      else if(DC_Error > 10) {
        DCmeter = 1;
      }
      else {
        Ch_Fault2.DC_Meter = 0;
      }
      if(EOCR_Read == 1) {
        EOCR_Count = 0;
        // EOCR_Command(1);
        EOCRTime = 0;
      }
      else {
        DC_Count = 0;
        DC_Write();
        if(Ch_Fault2.DC_Meter == 1) {
          DCmeter = 3;
        }
        DCTime = 0;
        DC_Error += 1;
      }
      DCTime2 = millis();
    }
  }
}
void PLC_Communication() {
  static unsigned long lastTime = 0;
  static unsigned long canTime = 0;
  static unsigned long TemperTime = 0;
  static unsigned long TestmodeTime = 0;
  char powervol[16]="";
  char powercur[16]="";
  char id[8]="";
  double index = 0;
  double curdecimal = 0;
  double voldecimal = 0;
  int volbin[32];
  int modulebin[8];
  int curbin[32];

  char s3[4] = "";
  if(millis() - TestmodeTime > 5000) {
    TestmodeTime = 0;
  }
  if(EB_Status_2.Standby == 0) {
    if(Chagring_Reset_2 == 0){
      Chagring_Reset_2 = 1;
      Init_Time_2 = millis();
    }
    if(EB_Status_2.TestMode == 0x00) {
      MCTime = 0;
      RelayTime_2 = 0;
      // StopTime_2 = 0;
      FaultTime_2 = 0;
      plcstartTime_2 = 0;
      Charging_State_2 = 0;
      Charging_Start_2 = 0;
      car_2 = 0;
      if(TestmodeTime == 0) {
        StopTime_2 = 0;
        Power_ON_2 = 0;
        if(Button1_OK == HIGH) {
          digitalWrite(R_Relay, LOW);
        }
      }
    }
    Bd_Status_2.Charge_Fin = 0x00;
    Bd_Status_2.Plug_Check = 0x00;
    Plug_Check = 0;
    Ch_Fault1_2.Main_Switch = 0;
    Ch_Fault1_2.Over_ACVoltage = 0;
    Ch_Fault1_2.Low_ACVoltage = 0;
    Ch_Fault1_2.Over_ACCurrent = 0;
    Ch_Fault1_2.Ground = 0;
    Ch_Fault1_2.MC = 0;
    Ch_Fault1_2.Relay = 0;
    Ch_Fault1_2.Over_DCVoltage = 0;
    Ch_Fault1_2.Over_DCCurrent = 0;
    Ch_Fault1_2.Short = 0;
    Ch_Fault1_2.Pre_Charging = 0;
    Ch_Fault1_2.Power_Module = 0;
    Ch_Fault1_2.Temperature = 0;
    Ch_Fault2_2.CP_Error = 0;
    Ch_Fault2_2.EV_Error = 0;
    Ch_Fault2_2.PLC_Error = 0;
    Ch_Fault2_2.CAN_Error = 0;
    Ch_Fault2_2.Control_Board = 0;
    Ch_Fault2_2.Car_Error = 0;
    Ch_Fault2_2.Emergency = 0;
    Ch_Fault2_2.DC_Meter = 0;
    Ch_Fault2_2.the_others = 0;
    start_sign_2 = 0;
    Connect_Check_2 = millis();
    if(PLC_Reset_2 == 0) {
      car_2 = 15;
      cable_Check_2 = millis();
    }
  }
  if(cable_Check_2 != 0) {
    if(millis() - cable_Check_2 > 13000) {
      car_2 = 16;
      cable_Check_2 = 0;
      }
  }
  if(Chagring_Reset_2 == 1){
    if(millis() - Init_Time_2 > 60000) {
      Connector_Fault_2 = 1;
    }
  }
  if(EB_Status_2.TestMode == zero) {
    if(Mode == 1) {
      testmode_2 = 0;
    }
    else {
      testmode_2 = 1;
    }
  }
  if(EB_Status_2.TestMode == 0x01) {
    if(Mode == 2) {
      testmode_2 = 0;
    }
    else {
      testmode_2 = 2;
    }
  }
  if(testmode_2 == 1) {
    Mode = 1;   //차 충전
    plcvoltage = 0;
    plcvol[1] = plcvoltage&0xff;
    plcvol_ch_2 = plcvoltage&(0xff<<8);
    plcvol_ch_2 = plcvol_ch_2>>8;
    plcvol[2] = plcvol_ch_2&0xff;
    plcvol_ch_2 = plcvoltage&(0xff<<16);
    plcvol_ch_2 = plcvol_ch_2>>16;
    plcvol[3] = plcvol_ch_2&0xff;
    plcvol_ch_2 = plcvoltage&(0xff<<24);
    plcvol_ch_2 = plcvol_ch_2>>24;
    plcvol[4] = plcvol_ch_2&0xff;
  
    plccurrent = 0;
    plccur[1] = plccurrent&0xff;                     //1의자리
    plccur_ch_2 = plccurrent&(0xff<<8);
    plccur_ch_2 = plccur_ch_2>>8;                
    plccur[2] = plccur_ch_2&0xff;                      //10의자리
    plccur_ch_2 = plccurrent&(0xff<<16);
    plccur_ch_2 = plccur_ch_2>>16;
    plccur[3] = plccur_ch_2&0xff;
    plccur_ch_2 = plccurrent&(0xff<<24);
    plccur_ch_2 = plccur_ch_2>>24;
    plccur[4] = plccur_ch_2&0xff; 
    //Power_ON_2 = 0;
    testmode_2 = 0;
    StopTime_2 = 0;
    car_2 = 14;
    power_2 = 2;
  }
  else if(testmode_2 == 2) {
    Mode = 2;   //로드 테스트       Test_Mode.Set_Voltage
    plcvoltage = Test_Mode_2.Set_Voltage * 1000;
    //plcvoltage = 480000;
    plcvol[1] = plcvoltage&0xff;
    plcvol_ch_2 = plcvoltage&(0xff<<8);
    plcvol_ch_2 = plcvol_ch_2>>8;
    plcvol[2] = plcvol_ch_2&0xff;
    plcvol_ch_2 = plcvoltage&(0xff<<16);
    plcvol_ch_2 = plcvol_ch_2>>16;
    plcvol[3] = plcvol_ch_2&0xff;
    plcvol_ch_2 = plcvoltage&(0xff<<24);
    plcvol_ch_2 = plcvol_ch_2>>24;
    plcvol[4] = plcvol_ch_2&0xff;

    if(Power_ON > 0) {
      plccurrent = (Test_Mode_2.Set_Current/4) * 1000;
    }
    else {
      plccurrent = (Test_Mode_2.Set_Current/8) * 1000;
    }
    plccur[1] = plccurrent&0xff;                     //1의자리
    plccur_ch_2 = plccurrent&(0xff<<8);
    plccur_ch_2 = plccur_ch_2>>8;                
    plccur[2] = plccur_ch_2&0xff;                      //10의자리
    plccur_ch_2 = plccurrent&(0xff<<16);
    plccur_ch_2 = plccur_ch_2>>16;
    plccur[3] = plccur_ch_2&0xff;
    plccur_ch_2 = plccurrent&(0xff<<24);
    plccur_ch_2 = plccur_ch_2>>24;
    plccur[4] = plccur_ch_2&0xff; 
    Power_ON_2 = 0;
    testmode_2 = 0;
    RelayTime_test = 0;
    RelayTime_2 = 0;
    if(Test_Mode_2.Resistance_Test == 1) {
      MCTime = millis();
      RelayTime_test = 0;
    }
    else if(Test_Mode_2.False_Test == 1) {
      MCTime = 0;
      RelayTime_test = millis();
    }
    RelayTime_2 = 0;
    power_ON_2 = 0;
    StopTime_2 = 0;
    TestmodeTime = millis();
  }
  if(EB_Status_2.TestMode == zero) {
    ReadData_2();
    if(EB_Status_2.Start == zero) {
      if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x09) {
        // Bd_Status_2.Plug_Check = 0x01;
        Plug_Check = 0x01;
        Bd_Status_2.Charge_Ready = 0x01;
      }
      else if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x0C) {
        Bd_Status_2.Plug_Check = zero;
        Plug_Check = 0;
        start_sign_2 = 0;
      }
      if(Charging_State_2 == 0 || Charging_State_2 == 15) {
        Charging_Start_2 = 0;
      }
    }
    else if(EB_Status_2.Start == 0x01) {
      Charging_Start_2 = 1;
      // if(start_sign_2 != 2) {
      //   start_sign_2 = 1;
      // }
    }
    if(Charging_Start_2 == 1) {
      ConnTime = millis();
      if(Charging_State_2 == 0 || Charging_State_2 > 12) {
        if(Connect_Check_2 != 0) {
          if(millis() - Connect_Check_2 > 2000) {
            car_2 = 16;
            Connect_Check_2 = millis();
            Charging_State_2 = 0;
            delay(10);
          }
        }
      }
      if(PLC_data_2[0] == 0x47) {
        if(Charging_State_2 == 0) {
          if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x09) {
            Bd_Status_2.Plug_Check = 0x01;
            Bd_Status_2.Charge_Ready = 0x01;
            Bd_Status_2.Charge_Fin = 0x00;
            EVSEStatusCode = 0x01;
            ResponseCode = 0x01;
            Charging_State_2 = 1;
            // Charge_On = 1;
            start_sign_2 = 1;
            MCTime = 0;
            Connect_Check_2 = 0;
          }
          else if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x0C) {
            start_sign_2 = 0;
            Connect_Check_2 = millis();
          }
        }
        else if(Charging_State_2 != 0 && Charging_State_2 < 11) {
          if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x09) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          else if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x0C) {
            start_sign_2 = 0;
          }
        }
        if(start_sign_2 == 1) {
          if(LED_data[2] == 0x00) {
            MCTime = millis();
            car_2 = 1;
            start_sign_2 = 2;
          }
        }
        if(plcstartTime_2 != 0) {
          if(millis() - plcstartTime_2 > 25000) {
            car_2 = 2;
            plcstartTime_2 = millis();
          }
          else if(millis() - plcstartTime_2 > 20000) {
            car_2 = 1;
          }
        }
        if (PLC_data_2[2] == 0x02) {
          Bd_Status_2.Plug_Check = 0x01;
          Bd_Status_2.Charge_Fin = 0x00;
          EVSEStatusCode = 0x01;
          ResponseCode = 0x01;
          car_2 = 3;                    //Session Setup Response
          Charging_State_2 = 3;
          plcstartTime_2 = 0;
          PLC_Reset_2 = 0;
          maxpower = 10;
          maxpower_2 = 10;
        }
        else if (PLC_data_2[2] == 0x03) {
          Bd_Status_2.Plug_Check = 0x01;
          Bd_Status_2.Charge_Fin = 0x00;
          car_2 = 4;       //Service Discovery Response
          Charging_State_2 = 4;
          plcstartTime_2 = 0;
          if(ResponseCode != 0x04) {
            ResponseCode = 0x00;
          }
        }
        else if (PLC_data_2[2] == 0x05) {
          Bd_Status_2.Plug_Check = 0x01;
          car_2 = 5;       //Service Payment Selection Response
          Charging_State_2 = 5;
        }
        else if (PLC_data_2[2] == 0x07) {
          car_2 = 6;       //Contract Authentication Response
          Charging_State_2 = 6;
        }
        else if (PLC_data_2[2] == 0x08) {
          car_2 = 7;       //Charge Parameter Discovery Response
          Charging_State_2 = 7;
          setVoltage = 0;
          // if(Power_ON > 0) {
          //   digitalWrite(M_Relay, LOW);
          // }
          if(PLC_data_2[11] != zero) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
        }
        else if (PLC_data_2[2] == 0x0f) {
          car_2 = 8;     //Cablecheck Response
          Charging_State_2 = 8;
          if(PLC_data_2[7] != zero) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
        }
        else if (PLC_data_2[2] == 0x10) {          //전압 세팅해줘야됨
          Charging_State_2 = 9;
          if(Power_ON_2 == 0) {                                 //릴레이 닫음
            if(Me_Current <50) {
              if(Power_ON > 0) {
                digitalWrite(M_Relay, LOW);
                delay(50);
              }
              digitalWrite(R_Relay, HIGH);
              RelayTime_2 = millis();
              Power_ON_2++;
            }
            powermoduleTime = millis();
          }
          else if(RelayTime_2 == 0) {
            R_Relay_OK = digitalRead(R_Relay_R);
            // L_Relay_OK = LOW;
            // R_Relay_OK = LOW;
            if(R_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
              car_2 = 9;
              power_2 = 1;
            }
            else {
              car_2 = 14;         //relay error
              power_2 = 2;
              Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1_2.Relay = 1;
              total_Fault = 2;
              FaultTime_2 = millis();
            }
          }
          plccur[1] = 0x00;
          plccur[2] = 0x00;
          plccur[3] = 0x00;
          plccur[4] = 0x00;
          if(PLC_data_2[12] == 0xFF) {         //10^-1
            index = 0.1;
          }
          else if(PLC_data_2[12] == 0x00) {    //10^0
            index = 1;
          }
          else if(PLC_data_2[12] == 0x01) {    //10^1
            index = 10;
          }
          voldecimal = PLC_data_2[15]*256 + PLC_data_2[14];
          plcvoltage = (voldecimal*index)*1000;        //파워모듈 세팅 전압값
          plcvol[1] = plcvoltage&0xff;
          plcvol_ch_2 = plcvoltage&(0xff<<8);
          plcvol_ch_2 = plcvol_ch_2>>8;
          plcvol[2] = plcvol_ch_2&0xff;
          plcvol_ch_2 = plcvoltage&(0xff<<16);
          plcvol_ch_2 = plcvol_ch_2>>16;
          plcvol[3] = plcvol_ch_2&0xff;
          plcvol_ch_2 = plcvoltage&(0xff<<24);
          plcvol_ch_2 = plcvol_ch_2>>24;
          plcvol[4] = plcvol_ch_2&0xff;
          Bd_Status_2.Battery_SOC = PLC_data_2[8];
          if(PLC_data_2[7] != zero) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
        }
        else if (PLC_data_2[2] == 0x11) {    //전류 들어가기 시작함
          Charging_State_2 = 10;
          if(Charging_State > 0 && Charging_State < 9) {
            maxpower_2 = 10;
            maxpower = 10;
          }
          else if(Charging_State > 8 && Charging_State_2 < 11) {
            maxpower_2 = 100;
            maxpower = 100;
          }
          else if(Power_ON == 0) {
            maxpower_2 = 200;
            maxpower = 200;
          }
          Bd_Status_2.Charge_Run = 0x01;
          Connector_Fault_2 = 0;
          Chagring_Reset_2 = 0;
          Init_Time_2 = millis();
          R_Relay_OK = digitalRead(R_Relay_R);
          // L_Relay_OK = LOW;
          // R_Relay_OK = LOW;
          // if(L_Relay_OK == LOW && R_Relay_OK == LOW) {
          if(Relay_ON < 10) {
            car_2 = 10;
            power_2 = 1;
            if(PLC_data_2[12] == 0xFF) {
              index = 0.1;
            }
            else if(PLC_data_2[12] == 0x00) {
              index = 1;
            }
            else if(PLC_data_2[12] == 0x01) {
              index = 10;
            }
            curdecimal = PLC_data_2[15]*256 + PLC_data_2[14];
            if(curdecimal*index >20) {
              plccurrent = (curdecimal*index)*1000;         //파워모듈 세팅 전류값
            }
            else {
              plccurrent = (curdecimal*index+2)*1000;
            }
            if((curdecimal*index) - current > 20) {
              plccurrent = (current + 20)*1000;
            }
            if(M_Relay_OK > 0) {
              plccurrent = plccurrent/4;
              maxcurrent = (int)((((maxpower_2*1000)/voltage)*1000)/4);
            }
            else {
              plccurrent = plccurrent/8;
              maxcurrent = (int)((((maxpower_2*1000)/voltage)*1000)/8);
            }
            if(maxcurrent > 27500) {
              maxcurrent = 27500;
            }
            if(plccurrent >= maxcurrent) {
              plccurrent = maxcurrent;
            }
            plccur[1] = plccurrent&0xff;                     //1의자리
            plccur_ch_2 = plccurrent&(0xff<<8);
            plccur_ch_2 = plccur_ch_2>>8;                
            plccur[2] = plccur_ch_2&0xff;                      //10의자리
            plccur_ch_2 = plccurrent&(0xff<<16);
            plccur_ch_2 = plccur_ch_2>>16;
            plccur[3] = plccur_ch_2&0xff;
            plccur_ch_2 = plccurrent&(0xff<<24);
            plccur_ch_2 = plccur_ch_2>>24;
            plccur[4] = plccur_ch_2&0xff;
            
            if(PLC_data_2[40] == 0xFF) {         //10^-1
              index = 0.1;
            }
            else if(PLC_data_2[40] == 0x00) {    //10^0
              index = 1;
            }
            else if(PLC_data_2[40] == 0x01) {    //10^1
              index = 10;
            }
            voldecimal = PLC_data_2[43]*256 + PLC_data_2[42];
            plcvoltage = (voldecimal*index)*1000;        //파워모듈 세팅 전압값
            plcvol[1] = plcvoltage&0xff;                   //plccur_ch_2 자리수 저장
            plcvol_ch_2 = plcvoltage&(0xff<<8);
            plcvol_ch_2 = plcvol_ch_2>>8;
            plcvol[2] = plcvol_ch_2&0xff;
            plcvol_ch_2 = plcvoltage&(0xff<<16);
            plcvol_ch_2 = plcvol_ch_2>>16;
            plcvol[3] = plcvol_ch_2&0xff;
            plcvol_ch_2 = plcvoltage&(0xff<<24);
            plcvol_ch_2 = plcvol_ch_2>>24;
            plcvol[4] = plcvol_ch_2&0xff;
          }
          else {
            car_2 = 14;
            power_2 = 2;        //Relay Error
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Relay = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          if(PLC_data_2[36] == 0xFF) {         //10^-1
            index = 0.1;
          }
          else if(PLC_data_2[36] == 0x00) {    //10^0
            index = 1;
          }
          else if(PLC_data_2[36] == 0x01) {    //10^1
            index = 10;
          }
          if(PLC_data_2[7] > 0) {
            car_2 = 14;
            power_2 = 2;        //Relay Error
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault2_2.EV_Error = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          else {
            //Bd_Status_2.Fault = 0x00;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault2_2.EV_Error = 0;
          }
          Remain_sec = (PLC_data_2[39]*256 + PLC_data_2[38])*index;
          Bd_Status_2.Remain_Time = Remain_sec/60;
          Bd_Status_2.Battery_SOC = PLC_data_2[8];
          if(PLC_data_2[7] != zero) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.EV_Error = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          if(Bd_Status_2.Fault == 1) {
            car_2 = 14;
            power_2 = 2;
          }
        }
        else if (PLC_data_2[2] == 0x09) {
          car_2 = 11;
          Charging_State_2 = 11;
          if(PLC_data_2[4] == 0x00) {
            car_2 = 14;
            power_2 = 2;
          }
          if(PLC_data_2[11] != zero) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
        }
        else if (PLC_data_2[2] == 0x12) {
          Charging_State_2 = 12;
          car_2 = 12;
          power_2 = 2;
          if(PLC_data_2[7] != zero) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
        }
        else if (PLC_data_2[2] == 0x0C) {    //충전종료
          if(Charging_State_2 < 9) {
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          Charging_State_2 = 13;
          car_2 = 13;
          power_2 = 2;
          Charging_Start_2 = 0;
        }
      }
      if(MCTime != 0) {
        if(millis() - MCTime > 2000) {
          MC85_OK = digitalRead(MC85_R);
          delay(5);
          if(MC85_OK == HIGH) {
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.MC = 1;
            car_2 = 14;
            power_2 = 2;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          else {
            car_2 = 2;
            Charge_On = 1;
            plcstartTime_2 = millis();
            Bd_Status_2.Plug_Check = 0x01;
          }
          MCTime = 0;
        }
        else if(millis() - MCTime > 1000) {
          if(Power_State == 0) {
            digitalWrite(MC85, HIGH);
          }
          digitalWrite(R_Relay, LOW);
          Bd_Status_2.Plug_Check = 0x01;
          Bd_Status_2.Charge_Fin = 0x00;
          EVSEStatusCode = 0x01;
          ResponseCode = 0x01;
        }
      }
      if(RelayTime_2 != 0) {
        if(millis() - RelayTime_2 > 1000) {
          RelayTime_2 = millis();
          R_Relay_OK = digitalRead(R_Relay_R);
          // L_Relay_OK = LOW;
          // R_Relay_OK = LOW;
          delay(2);
          if(R_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
            car_2 = 9;
            power_2 = 1;
          }
          else {
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Relay = 1;
            car_2 = 14;         //relay error
            power_2 = 2;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          RelayTime_2 = 0;
        }
      }
      if(Charge_On == 1) {
        if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x0C) {
          Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
          Ch_Fault2_2.PLC_Error = 1;
          Bd_Status_2.Plug_Check = zero;
          Plug_Check = 0;
          car_2 = 14;
          power_2 = 2;
          start_sign_2 = 0;
          total_Fault = 2;
          FaultTime_2 = millis();
        }
      }
      else if(Charging_State_2 != 0) {
        if(PLC_data_2[4] == 0x01 && PLC_data_2[5] == 0x0C) {
          Bd_Status_2.Plug_Check = zero;
          Plug_Check = 0;
          car_2 = 14;
          power_2 = 2;
          start_sign_2 = 0;
        }
      }
      if(EB_Status_2.Start == 0) {
        if(Bd_Status_2.Charge_Fin == 0x00) {
          if(Charging_State_2 < 3) {
          car_2 = 14;
          power_2 = 2;
          }
          else if(Charging_State_2 < 9) {
            //ResponseCode = 0x04;
            car_2 = 14;
            power_2 = 2;
          }
          else if(Charging_State_2 < 13) {
            EVSEStatusCode = 0x02;
          }
        }
      }
      // else if(Charge_On == 0) {
      //   Bd_Status_2.Plug_Check = zero;
      //   car_2 = 14;
      // }
    }
    else {
      if(PLC_data_2[0] == 0x47) {
        if (PLC_data_2[2] == 0x12) {
          Charging_State_2 = 12;
          car_2 = 12;
          power_2 = 2;
          if(PLC_data_2[7] != zero) {
            car_2 = 14;
            power_2 = 2;
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
        }
        else if (PLC_data_2[2] == 0x0C) {    //충전종료
          if(Charging_State_2 < 9) {
            Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
            Ch_Fault1_2.Pre_Charging = 1;
            total_Fault = 2;
            FaultTime_2 = millis();
          }
          Charging_State_2 = 13;
          car_2 = 13;
          power_2 = 2;
          Charging_Start_2 = 0;
        }
      }
    }
    if(Bd_Status_2.Charge_Run == 0x01) {
      R_Relay_OK = digitalRead(R_Relay_R);
      if(R_Relay_OK == LOW) {
        Relay_ON = 0;
      }
      else {
        Relay_ON++;
      }
      if(total_Fault == 2) {
        Relay_ON = 0;
      }
      if(Relay_ON > 10) {
        if(Charging_State_2 != 13 || Charging_State_2 != 12) {
          car_2 = 14;
          power_2 = 2;        //Relay Error
          Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
          Ch_Fault1_2.Relay = 1;
          total_Fault = 2;
          FaultTime_2 = millis();
        }
      }
    }
    if(Charging_State_2 == 10) {
      Bd_Status_2.Charge_Fin = 0x00;
    }
    if (Charge_On == 2) {
      Relay_ON = 0;
      // if(FaultTime_2 == 0) {
      //   if(Bd_Status_2.Fault == 0x00) {
      //     Bd_Status_2.Charge_Run = 0x00;
      //     Bd_Status_2.Charge_Fin = 0x01;
      //   }
      // }
      // Charging_Start_2 = 0;
      // Charging_State_2 = 0;
      if(StopTime_2 == 0) {
        StopTime_2 = millis();
      }
      else if(StopTime_2 != 0) {
        if(millis() - StopTime_2 > 3000) {
          if(Power_ON == 0) {
            digitalWrite(MC85, LOW);
          }
          if(Button1_OK == HIGH) {
            digitalWrite(R_Relay, LOW);
          }
          Power_ON_2 = 0;
          Charge_On = 0;
          StopTime_2 = 0;
          MCTime = 0;
          RelayTime_2 = 0;
          RelayTime_test = 0;
          TestmodeTime = 0;
          MC_ON = 0;
          Charging_Start_2 = 0;
          Charging_State_2 = 15;
          Bd_Status_2.Charge_Run = 0x00;
          Relay_ON = 0;
          Bd_Status_2.Charge_Fin = 0x01;
          Connect_Check_2 = millis();
        }
        // else if(millis() - StopTime_2 > 1000) {
        //   if(FaultTime_2 == 0) {
        //     if(Bd_Status_2.Fault == 0x01) {
        //       Bd_Status_2.Charge_Run = 0x00;
        //       Bd_Status_2.Charge_Fin = 0x01;
        //     }
        //   }
        // }
      }
      power_2 = 0;
    }
    if(Plug_Check == 1) {
      ConnTime = millis();
      Ch_Fault2_2.Connector_position = 0;
    }
    else if(Plug_Check == zero) {
      if(millis() - ConnTime > 35000) {
        Ch_Fault2_2.Connector_position = 0;
        Connector_Fault_2 = 1;
      }
      else if(millis() - ConnTime > 15000) {
        if(Connector_Fault_2 == 0){
          Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
          Ch_Fault2_2.Connector_position = 1;
          if(total_Fault == 0) {
            total_Fault = 1;
          }
          // Init_Time_2 = millis();
        }
      }
    }
    if(EB_Status_2.Stop == 0x01) {
      if(Bd_Status_2.Charge_Fin == 0x00) {
        if(Charging_State_2 < 3) {
          car_2 = 14;
          power_2 = 2;
        }
        else if(Charging_State_2 < 9) {
          //ResponseCode = 0x04;
          car_2 = 14;
          power_2 = 2;
        }
        else if(Charging_State_2 < 13) {
          EVSEStatusCode = 0x02;
        }
      }
    }
    if(Power_ON_2 == 0) {
      if(Button1_OK == LOW) {
        digitalWrite(R_Relay, HIGH);
      }
      else {
        digitalWrite(R_Relay, LOW);
      }
    }
    // if(LED_data[2] == 0x01) {
    //   if(Bd_Status_2.Charge_Fin == 0x00) {
    //     if(Charge_On != 2 || Charge_On != 0) {
    //       car_2 = 14;
    //       power_2 = 2;
    //     }
    //   }
    //   Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
    //   Ch_Fault2_2.Emergency = 1;
    //   if(total_Fault == 0) {
    //     total_Fault = 1;
    //   }
    // }
    // else 
    if(Emergency_OK == LOW) {
      if(Bd_Status_2.Charge_Fin == 0x00) {
        if(Charge_On != 2 || Charge_On != 0) {
          car_2 = 14;
          power_2 = 2;
          start_sign_2 = 2;
        }
      }
      Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
      Ch_Fault2_2.Emergency = 1;
      if(total_Fault == 0) {
        total_Fault = 1;
      }
    }
    else {
      Ch_Fault2_2.Emergency = 0;
    }
    if(Ch_Fault1_2.Power_Module == 0x01) {
      if(Bd_Status_2.Charge_Fin == 0x00) {
        if(Charge_On != 2 || Charge_On != 0) {
          car_2 = 14;
          power_2 = 2;
        }
      }
    }
    if(Bd_Status_2.Fault == 1 || Ch_Fault1_2.Over_DCVoltage == 1 || Ch_Fault1_2.Over_DCCurrent == 1 || Ch_Fault1_2.Temperature == 1) {
      if(Bd_Status_2.Charge_Fin == 0x00) {
        if(Charge_On != 2 || Charge_On != 0) {
          car_2 = 14;
          power_2 = 2;
          start_sign_2 = 2;
        }
      }
    }
    if((millis() - lastTime) > 100) {      //send Can data, PLC data
      lastTime = millis();
      power_2 = 0;
      sendCarMsg_2();
      car_2 = 0;
    }
    for(int j=0;j<400;j++) {      //data 초기화
      PLC_data_2[j] = '\0';
    }
  
  }
  else if(EB_Status_2.TestMode == 0x01) {
    TestmodeTime = millis();
    ConnTime = millis();
    if(Test_Mode_2.Resistance_Test == 1) {
      if(Power_ON > 0) {
        if(MCTime != 0) {
          if(millis() - MCTime > 200) {
            // MC85_OK = digitalRead(MC85_R);
            MC85_OK = LOW;
            if(MC85_OK == HIGH) {
              //Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              power_2 = 2;
            }
            MCTime = 0;
            RelayTime_test = millis();
          }
          else if(millis() - MCTime > 1000) {
            if(Power_ON == 0) {
              digitalWrite(MC85, HIGH);
            }
            else if(Power_ON == 1) {
              digitalWrite(M_Relay, LOW);
            }
          }
        }
        else if(RelayTime_test != 0) {
          if(millis() - RelayTime_test > 2000) {
            digitalWrite(R_Relay, HIGH);
            digitalWrite(M_Relay, LOW);
            RelayTime_test = 0;
            RelayTime_2 = millis();
          }
        }
        else if(RelayTime_2 != 0) {
          if(millis() - RelayTime_2 > 1000) {
            R_Relay_OK = digitalRead(R_Relay_R);
            // L_Relay_OK = LOW;
            // R_Relay_OK = LOW;
            
            if(R_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
              power_2 = 1;
              power_ON_2 = 1;
              Power_ON_2 = 1;
              Charge_On = 1;
              powermoduleTime = millis();
            }
            else {
              power_2 = 2;
              Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1_2.Relay = 1;
              total_Fault = 2;
              FaultTime_2 = millis();
            }
            RelayTime_2 = 0;
          }
        }
      }
      else {
        if(MCTime != 0) {
          if(millis() - MCTime > 2000) {
            MC85_OK = digitalRead(MC85_R);
            if(MC85_OK == HIGH) {
              //Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              power_2 = 2;
            }
            MCTime = 0;
            RelayTime_test = millis();
          }
          else if(millis() - MCTime > 1000) {
            digitalWrite(MC85, HIGH);
          }
        }
        else if(RelayTime_test != 0) {
          if(millis() - RelayTime_test > 2000) {
            digitalWrite(R_Relay, HIGH);
            RelayTime_test = 0;
            RelayTime_2 = millis();
          }
        }
        else if(RelayTime_2 != 0) {
          if(millis() - RelayTime_2 > 1000) {
            R_Relay_OK = digitalRead(R_Relay_R);
            // L_Relay_OK = LOW;
            // R_Relay_OK = LOW;
            
            if(R_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
              power_2 = 1;
              power_ON_2 = 1;
              Power_ON_2 = 1;
              Charge_On = 1;
              powermoduleTime = millis();
            }
            else {
              power_2 = 2;
              Bd_Status_2.Fault = 0x01;     //fault 이유 주소 419번에 추가해야됨
              Ch_Fault1_2.Relay = 1;
              total_Fault = 2;
              FaultTime_2 = millis();
            }
            RelayTime_2 = 0;
          }
        }
      }
    }
    else if(Test_Mode_2.False_Test == 1) {
      if(RelayTime_test != 0) {
        if(millis() - RelayTime_test > 2000) {
          digitalWrite(R_Relay, HIGH);
          RelayTime_test = 0;
          // RelayTime = millis();
          Charge_On = 1;
        }
      }
      else if(RelayTime_2 != 0) {
        if(millis() - RelayTime_2 > 1000) {
          //L_Relay_OK = digitalRead(L_Relay_R);
          //R_Relay_OK = digitalRead(R_Relay_R);
          R_Relay_OK = LOW;
          // if(L_Relay_OK == LOW && R_Relay_OK == LOW) {           //릴레이 정상 동작시 파워모듈 작동
          //   power_2 = 1;
          //   power_ON_2 = 1;
          //   Power_ON_2 = 1;
          //   Charge_On = 1;
          // }
          // else {
          //   power_2 = 2;
          // }
          RelayTime_2 = 0;
        }
      }
    }
    if(Ch_Fault1_2.Over_DCVoltage == 1 || Ch_Fault1_2.Over_DCCurrent == 1 || Ch_Fault1_2.Temperature == 1) {
      if(millis() - StopTime_2 > 3500) {
        power_ON_2 = 2; 
        delay(5);
      }
    }
    
    if(LED_data[2] == 0x01) {
      if(millis() - StopTime_2 > 3500) {
        power_ON_2 = 2; 
        delay(5);
      }
    }
    if((millis() - lastTime) > 100) {      //send Can data, PLC data
      lastTime = millis();
      if(power_ON_2 == 1) {
        power_2 = 1;
      }
      else if(power_ON_2 == 2) {
        power_2 = 2;
        power_ON_2 = 0;
        Power_ON_2 = 0;
        // Charge_On = 0;
      }
      canData();
      if((millis() - canTime) > 1000) {
        canTime = millis();
        if(power_2 == 1) {
          power_2 = 5;
          canData();
          power_2 = 6;
          canData();
          delay(5);
        }
        power_2 = 3;        //전압, 전류 값 세팅 및 확인
        canData();
        power_2 = 4;
        canData();
        delay(10);
      }
    }
    for(int j=0;j<400;j++) {      //data 초기화
      PLC_data_2[j] = '\0';
    }
  }
  
  delay(2);
  if(Conn_OK == LOW) {
    Bd_Status_2.Plug_Check = 0;
    Ch_Fault2_2.Connector_position = 0;
    Plug_Check = 0;
    ConnTime = millis();
    // Bd_Status_2.Charge_Fin = 0x01;
  }
  
  //delay(10);


}
